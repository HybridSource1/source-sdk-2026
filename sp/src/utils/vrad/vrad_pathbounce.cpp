//-----------------------------------------------------------------------------
// vrad_pathbounce.cpp - indirect lighting through the 64-bit vrad15_bake helper.
//
// Enabled with -pathtrace. VRAD still does everything else: direct lighting, BSP
// reading and writing, static props and ambient cubes. Only the patch transfer and
// bounce stages are replaced. The helper's indirect light is added to style 0 of
// the direct samples, so FinalLightFace filters it the way it filters direct light,
// and props and ambient cubes pick it up from the finished lightmaps.
//-----------------------------------------------------------------------------
#include "vrad.h"
#include "lightmap.h"
#include "../vrad15_bake/vb15_format.h"
#include <stdio.h>

bool g_bPathTrace = false;
int g_nPathSamples = 256;
int g_nPathBounces = 8;
bool g_bPathDenoise = true;

extern char platformPath[MAX_PATH];
extern void BaseLightForFace( dface_t *f, Vector &light, float *parea, Vector &reflectivity );
extern void GetBumpNormals( const float *sVect, const float *tVect, const Vector &flatNormal,
	const Vector &phongNormal, Vector bumpNormals[NUM_BUMP_VECTS] );
extern void PreGetBumpNormalsForDisp( texinfo_t *pTexinfo, Vector &vecU, Vector &vecV, Vector &vecNormal );

// SetupAccelerationStructure replaces triangle vertices with intersection data, so
// the geometry has to be copied out first.
static CUtlVector<Vb15Tri> s_Tris;

void PathBounce_CaptureTriangles()
{
	int count = g_RtEnv.OptimizedTriangleList.Count();
	s_Tris.SetCount( count );
	for ( int i = 0; i < count; i++ )
	{
		const CacheOptimizedTriangle &src = g_RtEnv.OptimizedTriangleList[i];
		Vb15Tri &dst = s_Tris[i];
		for ( int v = 0; v < 3; v++ )
		{
			const Vector &p = src.Vertex( v );
			dst.v[v * 3 + 0] = p.x;
			dst.v[v * 3 + 1] = p.y;
			dst.v[v * 3 + 2] = p.z;
		}
		int id = src.m_Data.m_GeometryData.m_nTriangleID;
		if ( id & TRACE_ID_SKY )
			dst.kind = VB15_TRI_SKY;
		else if ( id & TRACE_ID_STATICPROP )
			dst.kind = VB15_TRI_PROP;
		else
			dst.kind = VB15_TRI_WORLD;
	}
}

// Index of light style 0 on a face, or -1. Only style 0 bounces, as in stock VRAD.
static int Style0Index( const dface_t *f )
{
	for ( int k = 0; k < MAXLIGHTMAPS; k++ )
	{
		if ( f->styles[k] == 0 )
			return k;
	}
	return -1;
}

static bool FaceTakesPart( int facenum )
{
	const dface_t *f = &g_pFaces[facenum];
	if ( texinfo[f->texinfo].flags & TEX_SPECIAL )
		return false;
	return facelight[facenum].numsamples > 0 && Style0Index( f ) >= 0;
}

static inline void CopyVec( float *dst, const Vector &v )
{
	dst[0] = v.x;
	dst[1] = v.y;
	dst[2] = v.z;
}

static void WriteOrDie( const void *data, size_t size, size_t count, FILE *fp, const char *path )
{
	if ( count && fwrite( data, size, count, fp ) != count )
		Error( "-pathtrace: could not write %s (is the disk full?)\n", path );
}

static void WriteScene( const char *path )
{
	FILE *fp = fopen( path, "wb" );
	if ( !fp )
		Error( "-pathtrace: could not create %s\n", path );

	unsigned int totalSamples = 0;
	for ( int i = 0; i < numfaces; i++ )
	{
		if ( FaceTakesPart( i ) )
			totalSamples += facelight[i].numsamples;
	}

	Vb15SceneHeader hdr;
	hdr.magic = VB15_SCENE_MAGIC;
	hdr.version = VB15_VERSION;
	hdr.numTris = s_Tris.Count();
	hdr.numSamples = totalSamples;
	hdr.raysPerSample = g_nPathSamples;
	hdr.maxBounces = g_nPathBounces;
	hdr.options = g_bPathDenoise ? VB15_OPT_DENOISE : 0;
	hdr.threads = numthreads;
	hdr.seed = 1;
	WriteOrDie( &hdr, sizeof( hdr ), 1, fp, path );
	WriteOrDie( s_Tris.Base(), sizeof( Vb15Tri ), s_Tris.Count(), fp, path );
	s_Tris.Purge();

	for ( int facenum = 0; facenum < numfaces; facenum++ )
	{
		if ( !FaceTakesPart( facenum ) )
			continue;

		dface_t *f = &g_pFaces[facenum];
		facelight_t *fl = &facelight[facenum];
		texinfo_t *tex = &texinfo[f->texinfo];
		const int style = Style0Index( f );
		const bool bDisp = ( f->dispinfo != -1 );
		const bool bBumped = ( tex->flags & SURF_BUMPLIGHT ) && fl->light[style][1] != NULL;
		const Vector flatNormal = dplanes[f->planenum].normal;
		const float radius = sqrt( fl->worldAreaPerLuxel );

		Vector baseLight, reflectivity;
		float baseArea;
		BaseLightForFace( f, baseLight, &baseArea, reflectivity );

		for ( int i = 0; i < fl->numsamples; i++ )
		{
			const sample_t &sample = fl->sample[i];
			Vb15Sample out;
			memset( &out, 0, sizeof( out ) );
			CopyVec( out.pos, sample.pos );
			CopyVec( out.direct, fl->light[style][0][i].m_vecLighting );
			CopyVec( out.albedo, reflectivity );
			out.radius = radius;
			out.face = facenum;
			out.s = sample.s;
			out.t = sample.t;

			// Match the normals stock GatherLight uses for this kind of surface.
			Vector hemisphere = sample.normal;
			if ( bBumped )
			{
				Vector bump[NUM_BUMP_VECTS];
				if ( bDisp )
				{
					Vector smooth = sample.normal, texU, texV;
					PreGetBumpNormalsForDisp( tex, texU, texV, smooth );
					GetBumpNormals( texU, texV, smooth, smooth, bump );
				}
				else
				{
					GetBumpNormals( tex->textureVecsTexelsPerWorldUnits[0], tex->textureVecsTexelsPerWorldUnits[1],
						flatNormal, sample.normal, bump );
					hemisphere = flatNormal;
				}
				for ( int b = 0; b < NUM_BUMP_VECTS; b++ )
					CopyVec( out.bump[b], bump[b] );
				out.flags |= VB15_SAMPLE_BUMPED;
			}
			CopyVec( out.normal, hemisphere );

			// Luxels past a wall's edge can land inside solid, where every ray would
			// hit the inside of the brush and come back black. Displacements are not
			// tested because they can legally sit below brush surfaces.
			if ( !bDisp )
			{
				Vector probe = sample.pos + hemisphere * 0.1f;
				if ( dleafs[PointLeafnum( probe )].contents & CONTENTS_SOLID )
					out.flags |= VB15_SAMPLE_INVALID;
			}

			WriteOrDie( &out, sizeof( out ), 1, fp, path );
		}
	}

	if ( fclose( fp ) != 0 )
		Error( "-pathtrace: could not finish writing %s\n", path );
}

static void RunHelper( const char *scenePath, const char *resultPath )
{
#ifdef _WIN32
	char exePath[MAX_PATH];
	GetModuleFileName( NULL, exePath, sizeof( exePath ) );
	Q_ExtractFilePath( exePath, exePath, sizeof( exePath ) );
	Q_strncat( exePath, "vrad15_bake.exe", sizeof( exePath ), COPY_ALL_CHARACTERS );

	char cmdLine[MAX_PATH * 3 + 16];
	Q_snprintf( cmdLine, sizeof( cmdLine ), "\"%s\" \"%s\" \"%s\"", exePath, scenePath, resultPath );

	STARTUPINFO si;
	PROCESS_INFORMATION pi;
	memset( &si, 0, sizeof( si ) );
	si.cb = sizeof( si );
	memset( &pi, 0, sizeof( pi ) );

	fflush( stdout );
	if ( !CreateProcess( exePath, cmdLine, NULL, NULL, TRUE, 0, NULL, NULL, &si, &pi ) )
		Error( "-pathtrace: could not start %s (error %lu). It and its DLLs must be next to vrad.exe.\n", exePath, GetLastError() );

	WaitForSingleObject( pi.hProcess, INFINITE );
	DWORD exitCode = 1;
	GetExitCodeProcess( pi.hProcess, &exitCode );
	CloseHandle( pi.hProcess );
	CloseHandle( pi.hThread );
	if ( exitCode != 0 )
		Error( "-pathtrace: vrad15_bake failed (exit code %lu). See its output above.\n", exitCode );
#else
	Error( "-pathtrace is only implemented on Windows\n" );
#endif
}

static void ReadOrDie( void *data, size_t size, size_t count, FILE *fp, const char *path )
{
	if ( count && fread( data, size, count, fp ) != count )
		Error( "-pathtrace: %s is truncated\n", path );
}

static void ApplyResults( const char *path )
{
	FILE *fp = fopen( path, "rb" );
	if ( !fp )
		Error( "-pathtrace: could not open %s\n", path );

	Vb15ResultHeader hdr;
	ReadOrDie( &hdr, sizeof( hdr ), 1, fp, path );
	if ( hdr.magic != VB15_RESULT_MAGIC || hdr.version != VB15_VERSION )
		Error( "-pathtrace: %s came from a different version of vrad15_bake\n", path );

	unsigned int expected = 0;
	for ( int i = 0; i < numfaces; i++ )
	{
		if ( FaceTakesPart( i ) )
			expected += facelight[i].numsamples;
	}
	if ( hdr.numSamples != expected )
		Error( "-pathtrace: expected %u results, %s has %u\n", expected, path, hdr.numSamples );

	for ( int facenum = 0; facenum < numfaces; facenum++ )
	{
		if ( !FaceTakesPart( facenum ) )
			continue;

		dface_t *f = &g_pFaces[facenum];
		facelight_t *fl = &facelight[facenum];
		const int style = Style0Index( f );
		const bool bBumped = ( texinfo[f->texinfo].flags & SURF_BUMPLIGHT ) && fl->light[style][1] != NULL;
		const int channels = bBumped ? NUM_BUMP_VECTS + 1 : 1;

		for ( int i = 0; i < fl->numsamples; i++ )
		{
			Vb15Result r;
			ReadOrDie( &r, sizeof( r ), 1, fp, path );
			for ( int c = 0; c < channels; c++ )
			{
				Vector v( r.light[c][0], r.light[c][1], r.light[c][2] );
				// One bad value would otherwise spread through the radial filter.
				if ( !( v.x >= 0 && v.y >= 0 && v.z >= 0 && v.x < 1e10f && v.y < 1e10f && v.z < 1e10f ) )
					v.Init( 0, 0, 0 );
				fl->light[style][c][i].m_vecLighting += v;
			}
		}
	}
	fclose( fp );
}

void PathBounce_Run()
{
	char scenePath[MAX_PATH], resultPath[MAX_PATH];
	Q_snprintf( scenePath, sizeof( scenePath ), "%s.vb15scene", platformPath );
	Q_snprintf( resultPath, sizeof( resultPath ), "%s.vb15result", platformPath );

	Msg( "Path tracing indirect light (%d rays per sample, up to %d bounces, denoise %s)\n",
		g_nPathSamples, g_nPathBounces, g_bPathDenoise ? "on" : "off" );

	double start = Plat_FloatTime();
	WriteScene( scenePath );
	RunHelper( scenePath, resultPath );
	ApplyResults( resultPath );
	remove( scenePath );
	remove( resultPath );
	Msg( "Path tracing done (%.1f seconds)\n", Plat_FloatTime() - start );
}
