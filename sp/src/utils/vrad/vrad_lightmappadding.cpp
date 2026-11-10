//-----------------------------------------------------------------------------
// vrad_lightmappadding.cpp - VRAD's -bicubiclightmaps.
//
// Lighting is computed on unpadded lightmaps as usual. Padding is added just before
// the BSP is written, and removed again when VRAD loads a padded BSP, so every other
// part of VRAD (static props, detail props, ambient cubes, incremental lighting)
// only ever sees the stock layout. See lightmap_padding_core.h for the format.
//-----------------------------------------------------------------------------
// Standard library headers first: Source's min/max macros break them otherwise.
#include "lightmap_padding_core.h"
#include "vrad.h"

bool g_bBicubicLightmaps = false;

static const char *s_pszPaddingKey = "vrad_lightmap_padding";

static std::string GetEntityString()
{
	if ( !dentdata.Count() )
		return std::string();
	return std::string( dentdata.Base(), strnlen( dentdata.Base(), dentdata.Count() ) );
}

static void SetEntityString( const std::string &ents )
{
	dentdata.SetSize( (int)ents.size() + 1 );
	memcpy( dentdata.Base(), ents.c_str(), ents.size() + 1 );
}

static void ReshapeSet( dface_t *pFaces, int nFaces, CUtlVector<byte> &lightdata, int dir, const char *pszName )
{
	if ( nFaces <= 0 || lightdata.Count() == 0 )
		return;

	std::vector<LmpFace> faces( nFaces );
	for ( int i = 0; i < nFaces; i++ )
	{
		const dface_t &src = pFaces[i];
		LmpFace &f = faces[i];
		f.lightofs = src.lightofs;
		f.mins[0] = src.m_LightmapTextureMinsInLuxels[0];
		f.mins[1] = src.m_LightmapTextureMinsInLuxels[1];
		f.size[0] = src.m_LightmapTextureSizeInLuxels[0];
		f.size[1] = src.m_LightmapTextureSizeInLuxels[1];
		f.numStyles = 0;
		while ( f.numStyles < MAXLIGHTMAPS && src.styles[f.numStyles] != 255 )
			f.numStyles++;
		const int flags = texinfo[src.texinfo].flags;
		f.blocks = ( flags & SURF_BUMPLIGHT ) ? NUM_BUMP_VECTS + 1 : 1;
		f.paddable = !( flags & TEX_SPECIAL ) && src.dispinfo == -1;
	}

	std::vector<unsigned char> in( lightdata.Base(), lightdata.Base() + lightdata.Count() ), out;
	std::string err;
	if ( !LmpReshape( faces, in, out, dir, err ) )
		Error( "-bicubiclightmaps: %s lighting: %s\n", pszName, err.c_str() );

	for ( int i = 0; i < nFaces; i++ )
	{
		dface_t &dst = pFaces[i];
		dst.lightofs = faces[i].lightofs;
		dst.m_LightmapTextureMinsInLuxels[0] = faces[i].mins[0];
		dst.m_LightmapTextureMinsInLuxels[1] = faces[i].mins[1];
		dst.m_LightmapTextureSizeInLuxels[0] = faces[i].size[0];
		dst.m_LightmapTextureSizeInLuxels[1] = faces[i].size[1];
	}
	lightdata.SetSize( (int)out.size() );
	if ( !out.empty() )
		memcpy( lightdata.Base(), &out[0], out.size() );
}

//-----------------------------------------------------------------------------
// Displacements can't be padded, so they must keep bilinear filtering. The engine
// gives shaders no way to tell a displacement from a brush, so each displacement
// gets a patched copy of its material that sets $nobicubiclightmap, the same way
// VBSP patches materials for cubemaps.
//-----------------------------------------------------------------------------

static void GetPatchPrefix( char *pszOut, int nSize )
{
	Q_snprintf( pszOut, nSize, "maps/%s/nobicubic/", source );
	Q_strlower( pszOut );
}

static int FindTexData( const char *pszName )
{
	for ( int i = 0; i < numtexdata; i++ )
	{
		if ( !Q_stricmp( TexDataStringTable_GetString( dtexdata[i].nameStringTableID ), pszName ) )
			return i;
	}
	return -1;
}

// A texinfo identical to iTexInfo but using iTexData, reusing an existing one if possible
// so that recompiling the same map doesn't keep adding texinfos.
static int FindOrAddTexInfoWithTexData( int iTexInfo, int iTexData )
{
	texinfo_t want = texinfo[iTexInfo];
	want.texdata = iTexData;
	for ( int i = 0; i < texinfo.Count(); i++ )
	{
		if ( !memcmp( &texinfo[i], &want, sizeof( want ) ) )
			return i;
	}
	if ( texinfo.Count() >= MAX_MAP_TEXINFO )
		Error( "-bicubiclightmaps: too many texinfos to mark displacement materials\n" );
	return texinfo.AddToTail( want );
}

static int PatchedTexInfo( int iTexInfo, const char *pszPrefix )
{
	const int iOrigData = texinfo[iTexInfo].texdata;
	const char *pszName = TexDataStringTable_GetString( dtexdata[iOrigData].nameStringTableID );
	if ( !Q_strnicmp( pszName, pszPrefix, Q_strlen( pszPrefix ) ) )
		return iTexInfo;

	char szPatched[MAX_PATH];
	Q_snprintf( szPatched, sizeof( szPatched ), "%s%s", pszPrefix, pszName );
	int iData = FindTexData( szPatched );
	if ( iData < 0 )
	{
		if ( numtexdata >= MAX_MAP_TEXDATA )
			Error( "-bicubiclightmaps: too many materials to mark displacement materials\n" );
		iData = numtexdata++;
		dtexdata[iData] = dtexdata[iOrigData];
		dtexdata[iData].nameStringTableID = TexDataStringTable_AddOrFindString( szPatched );
	}

	// (Re)write the patch every time, so it always points at the current original.
	char szVMT[MAX_PATH * 2];
	Q_snprintf( szVMT, sizeof( szVMT ),
		"\"patch\"\n{\n\t\"include\"\t\"materials/%s.vmt\"\n\t\"insert\"\n\t{\n\t\t\"$nobicubiclightmap\"\t\"1\"\n\t}\n}\n", pszName );
	char szPakName[MAX_PATH];
	Q_snprintf( szPakName, sizeof( szPakName ), "materials/%s.vmt", szPatched );
	AddBufferToPak( GetPakFile(), szPakName, szVMT, Q_strlen( szVMT ), true );

	return FindOrAddTexInfoWithTexData( iTexInfo, iData );
}

static int UnpatchedTexInfo( int iTexInfo, const char *pszPrefix )
{
	const char *pszName = TexDataStringTable_GetString( dtexdata[texinfo[iTexInfo].texdata].nameStringTableID );
	const int nPrefix = Q_strlen( pszPrefix );
	if ( Q_strnicmp( pszName, pszPrefix, nPrefix ) )
		return iTexInfo;
	int iOrigData = FindTexData( pszName + nPrefix );
	if ( iOrigData < 0 )
		Error( "-bicubiclightmaps: material %s is missing from the map\n", pszName + nPrefix );
	return FindOrAddTexInfoWithTexData( iTexInfo, iOrigData );
}

static void RemapDisplacementTexInfos( bool bPatch )
{
	char szPrefix[MAX_PATH];
	GetPatchPrefix( szPrefix, sizeof( szPrefix ) );

	CUtlVector<int> remap;
	remap.SetCount( texinfo.Count() );
	for ( int i = 0; i < remap.Count(); i++ )
		remap[i] = -1;

	dface_t *sets[2] = { dfaces, dfaces_hdr };
	int counts[2] = { numfaces, numfaces_hdr };
	for ( int set = 0; set < 2; set++ )
	{
		for ( int i = 0; i < counts[set]; i++ )
		{
			dface_t &f = sets[set][i];
			if ( f.dispinfo == -1 || f.texinfo < 0 || f.texinfo >= remap.Count() )
				continue;
			if ( texinfo[f.texinfo].flags & TEX_SPECIAL )
				continue;
			int &to = remap[f.texinfo];
			if ( to < 0 )
				to = bPatch ? PatchedTexInfo( f.texinfo, szPrefix ) : UnpatchedTexInfo( f.texinfo, szPrefix );
			f.texinfo = to;
		}
	}
}

// Call right after LoadBSPFile, before anything reads faces or lighting.
void LightmapPadding_OnLoad()
{
	if ( atoi( LmpGetWorldKey( GetEntityString(), s_pszPaddingKey ).c_str() ) == 0 )
		return;

	ReshapeSet( dfaces, numfaces, dlightdataLDR, -1, "LDR" );
	ReshapeSet( dfaces_hdr, numfaces_hdr, dlightdataHDR, -1, "HDR" );
	RemapDisplacementTexInfos( false );
	Msg( "Removed bicubic lightmap padding from the previous compile\n" );
}

// Call just before WriteBSPFile.
void LightmapPadding_OnFinish()
{
	std::string ents = GetEntityString();
	if ( g_bBicubicLightmaps )
	{
		// Both sets are padded: the one just lit, and the other one, which OnLoad unpadded.
		ReshapeSet( dfaces, numfaces, dlightdataLDR, +1, "LDR" );
		ReshapeSet( dfaces_hdr, numfaces_hdr, dlightdataHDR, +1, "HDR" );
		RemapDisplacementTexInfos( true );
		if ( !LmpSetWorldKey( ents, s_pszPaddingKey, "1" ) )
			Error( "-bicubiclightmaps: could not find worldspawn in the entity lump\n" );
		SetEntityString( ents );
		Msg( "Padded lightmaps for bicubic filtering\n" );
	}
	else if ( !LmpGetWorldKey( ents, s_pszPaddingKey ).empty() )
	{
		// A previous compile padded this map, and this one did not.
		LmpSetWorldKey( ents, s_pszPaddingKey, "0" );
		SetEntityString( ents );
	}
}
