// vrad15_bake - 64-bit path-traced indirect lighting for VRAD.
//
// VRAD computes direct lighting at every lightmap sample and hands those samples
// and the occluder triangles to this program. Each sample then gathers cosine-
// weighted paths. A path that hits a surface picks up the nearest stored sample's
// direct light, scaled by albedo, so direct light (attenuation, sky, texture
// shadows) stays exactly what VRAD computed. Sky is not added here because VRAD's
// direct pass already includes it.
#include "vb15_format.h"
#include <embree4/rtcore.h>
#include <pmmintrin.h>
#include <xmmintrin.h>
#ifdef VB15_USE_OIDN
#include <OpenImageDenoise/oidn.h>
#endif
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <thread>
#include <vector>

struct V3
{
	float x, y, z;
	V3( float a = 0, float b = 0, float c = 0 ) : x( a ), y( b ), z( c ) {}
	explicit V3( const float *p ) : x( p[0] ), y( p[1] ), z( p[2] ) {}
	V3 operator+( V3 o ) const { return V3( x + o.x, y + o.y, z + o.z ); }
	V3 operator-( V3 o ) const { return V3( x - o.x, y - o.y, z - o.z ); }
	V3 operator*( float s ) const { return V3( x * s, y * s, z * s ); }
	V3 operator*( V3 o ) const { return V3( x * o.x, y * o.y, z * o.z ); }
	float operator[]( int i ) const { return i == 0 ? x : ( i == 1 ? y : z ); }
};
static inline float Dot( V3 a, V3 b ) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline V3 Cross( V3 a, V3 b ) { return V3( a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x ); }
static inline V3 Norm( V3 a ) { float l = std::sqrt( Dot( a, a ) ); return l > 0 ? a * ( 1.0f / l ) : a; }

struct Rng // PCG32
{
	uint64_t state;
	explicit Rng( uint64_t seed ) : state( seed * 0x9E3779B97F4A7C15ull + 0x853C49E6748FEA9Bull ) { Next(); Next(); }
	uint32_t Next()
	{
		uint64_t old = state;
		state = old * 6364136223846793005ull + 0xDA3E39CB94B95BDBull;
		uint32_t xs = (uint32_t)( ( ( old >> 18 ) ^ old ) >> 27 ), rot = (uint32_t)( old >> 59 );
		return ( xs >> rot ) | ( xs << ( ( -(int)rot ) & 31 ) );
	}
	float F() { return std::min( Next() * ( 1.0f / 4294967296.0f ), 0.99999994f ); }
};

static inline V3 CosineDir( V3 n, float u1, float u2 )
{
	V3 a = std::fabs( n.x ) > 0.9f ? V3( 0, 1, 0 ) : V3( 1, 0, 0 );
	V3 t = Norm( Cross( a, n ) ), b = Cross( n, t );
	float r = std::sqrt( u1 ), phi = 6.28318530718f * u2;
	return Norm( t * ( r * std::cos( phi ) ) + b * ( r * std::sin( phi ) ) + n * std::sqrt( std::max( 0.0f, 1.0f - u1 ) ) );
}

// True when nothing blocks the straight line between two points just above a surface.
static RTCScene g_occlusionScene = nullptr;
static bool Visible( V3 a, V3 b )
{
	V3 d = b - a;
	float len = std::sqrt( Dot( d, d ) );
	if ( len < 1e-3f ) return true;
	d = d * ( 1.0f / len );
	RTCRay r;
	r.org_x = a.x; r.org_y = a.y; r.org_z = a.z; r.tnear = 0.0f;
	r.dir_x = d.x; r.dir_y = d.y; r.dir_z = d.z; r.time = 0.0f;
	r.tfar = len; r.mask = 0xFFFFFFFFu; r.id = 0; r.flags = 0;
	rtcOccluded1( g_occlusionScene, &r );
	return r.tfar >= 0.0f; // Embree sets tfar to -inf on a hit
}

// Static kd-tree over sample positions, for finding the sample lighting a hit point.
class SampleTree
{
public:
	void Build( const std::vector<Vb15Sample> &samples )
	{
		m_s = &samples;
		for ( uint32_t i = 0; i < samples.size(); i++ )
		{
			if ( samples[i].flags & VB15_SAMPLE_INVALID ) continue;
			m_idx.push_back( i );
			m_maxReach = std::max( m_maxReach, Reach( samples[i] ) );
		}
		if ( !m_idx.empty() ) BuildNode( 0, (int)m_idx.size() );
	}

	static float Reach( const Vb15Sample &s ) { return s.radius * 2.0f + 8.0f; }

	// Nearest sample on the surface at p, seen from the side the ray arrived on.
	int FindSurface( V3 p, V3 rayDir ) const
	{
		int best = -1;
		float bestD2 = m_maxReach * m_maxReach;
		if ( !m_idx.empty() ) Query( 0, (int)m_idx.size(), p, rayDir, best, bestD2 );
		return best;
	}

	template <class F> void ForRadius( V3 p, float r, F fn ) const
	{
		if ( !m_idx.empty() ) Radius( 0, (int)m_idx.size(), p, r, fn );
	}

private:
	const std::vector<Vb15Sample> *m_s = nullptr;
	std::vector<uint32_t> m_idx;
	std::vector<uint8_t> m_axis;
	float m_maxReach = 8.0f;

	V3 Pos( uint32_t i ) const { return V3( ( *m_s )[i].pos ); }

	void BuildNode( int lo, int hi )
	{
		if ( hi - lo <= 8 ) return;
		V3 mn = Pos( m_idx[lo] ), mx = mn;
		for ( int i = lo; i < hi; i++ )
		{
			V3 p = Pos( m_idx[i] );
			mn = V3( std::min( mn.x, p.x ), std::min( mn.y, p.y ), std::min( mn.z, p.z ) );
			mx = V3( std::max( mx.x, p.x ), std::max( mx.y, p.y ), std::max( mx.z, p.z ) );
		}
		V3 d = mx - mn;
		int axis = ( d.x >= d.y && d.x >= d.z ) ? 0 : ( d.y >= d.z ? 1 : 2 );
		int mid = ( lo + hi ) / 2;
		std::nth_element( m_idx.begin() + lo, m_idx.begin() + mid, m_idx.begin() + hi,
			[&]( uint32_t a, uint32_t b ) { return Pos( a )[axis] < Pos( b )[axis]; } );
		if ( m_axis.empty() ) m_axis.resize( m_idx.size(), 0 );
		m_axis[mid] = (uint8_t)axis;
		BuildNode( lo, mid );
		BuildNode( mid + 1, hi );
	}

	void Consider( uint32_t i, V3 p, V3 rayDir, int &best, float &bestD2 ) const
	{
		const Vb15Sample &s = ( *m_s )[i];
		V3 d = p - V3( s.pos );
		float d2 = Dot( d, d );
		if ( d2 >= bestD2 ) return;
		V3 n( s.normal );
		if ( Dot( n, rayDir ) >= 0.0f ) return;                                   // arrived from behind
		float dist = std::sqrt( d2 );
		if ( std::fabs( Dot( n, d ) ) > 0.35f * dist + 2.0f ) return;             // different surface
		if ( dist > Reach( s ) ) return;
		// A floor running under a wall is coplanar on both sides, so distance alone
		// would let light through the wall. Check last: this is the costly test.
		V3 lift = n * 0.5f;
		if ( !Visible( p + lift, V3( s.pos ) + lift ) ) return;
		bestD2 = d2;
		best = (int)i;
	}

	void Query( int lo, int hi, V3 p, V3 rayDir, int &best, float &bestD2 ) const
	{
		if ( hi - lo <= 8 )
		{
			for ( int i = lo; i < hi; i++ ) Consider( m_idx[i], p, rayDir, best, bestD2 );
			return;
		}
		int mid = ( lo + hi ) / 2, axis = m_axis[mid];
		float delta = p[axis] - Pos( m_idx[mid] )[axis];
		Consider( m_idx[mid], p, rayDir, best, bestD2 );
		if ( delta < 0 )
		{
			Query( lo, mid, p, rayDir, best, bestD2 );
			if ( delta * delta < bestD2 ) Query( mid + 1, hi, p, rayDir, best, bestD2 );
		}
		else
		{
			Query( mid + 1, hi, p, rayDir, best, bestD2 );
			if ( delta * delta < bestD2 ) Query( lo, mid, p, rayDir, best, bestD2 );
		}
	}

	template <class F> void Radius( int lo, int hi, V3 p, float r, F &fn ) const
	{
		if ( hi - lo <= 8 )
		{
			for ( int i = lo; i < hi; i++ )
			{
				V3 d = p - Pos( m_idx[i] );
				if ( Dot( d, d ) <= r * r ) fn( m_idx[i] );
			}
			return;
		}
		int mid = ( lo + hi ) / 2, axis = m_axis[mid];
		float delta = p[axis] - Pos( m_idx[mid] )[axis];
		V3 d = p - Pos( m_idx[mid] );
		if ( Dot( d, d ) <= r * r ) fn( m_idx[mid] );
		if ( delta - r <= 0 ) Radius( lo, mid, p, r, fn );
		if ( delta + r >= 0 ) Radius( mid + 1, hi, p, r, fn );
	}
};

struct Bake
{
	Vb15SceneHeader hdr;
	std::vector<Vb15Tri> tris;
	std::vector<Vb15Sample> samples;
	std::vector<Vb15Result> results;
	SampleTree tree;
	RTCDevice device = nullptr;
	RTCScene scene = nullptr;
	std::atomic<uint64_t> rays{ 0 };
};

static bool ReadAll( const char *path, Bake &b )
{
	FILE *f = fopen( path, "rb" );
	if ( !f ) { fprintf( stderr, "vrad15_bake: cannot open %s\n", path ); return false; }
	bool ok = fread( &b.hdr, sizeof( b.hdr ), 1, f ) == 1 && b.hdr.magic == VB15_SCENE_MAGIC && b.hdr.version == VB15_VERSION;
	if ( ok )
	{
		b.tris.resize( b.hdr.numTris );
		b.samples.resize( b.hdr.numSamples );
		ok = ( b.tris.empty() || fread( b.tris.data(), sizeof( Vb15Tri ), b.tris.size(), f ) == b.tris.size() ) &&
			 ( b.samples.empty() || fread( b.samples.data(), sizeof( Vb15Sample ), b.samples.size(), f ) == b.samples.size() );
	}
	fclose( f );
	if ( !ok ) fprintf( stderr, "vrad15_bake: %s is not a valid scene file\n", path );
	return ok;
}

static void EmbreeError( void *, RTCError code, const char *msg )
{
	fprintf( stderr, "vrad15_bake: Embree error %d: %s\n", (int)code, msg ? msg : "" );
}

static bool BuildScene( Bake &b, int threads )
{
	char cfg[64];
	snprintf( cfg, sizeof( cfg ), "threads=%d", threads );
	b.device = rtcNewDevice( cfg );
	if ( !b.device ) { fprintf( stderr, "vrad15_bake: could not create Embree device\n" ); return false; }
	rtcSetDeviceErrorFunction( b.device, EmbreeError, nullptr );
	b.scene = rtcNewScene( b.device );
	rtcSetSceneBuildQuality( b.scene, RTC_BUILD_QUALITY_HIGH );
	rtcSetSceneFlags( b.scene, RTC_SCENE_FLAG_ROBUST );
	RTCGeometry g = rtcNewGeometry( b.device, RTC_GEOMETRY_TYPE_TRIANGLE );
	size_t n = b.tris.size();
	// Embree reads vertices with 16-byte loads, so it needs its own padded buffers.
	float *v = (float *)rtcSetNewGeometryBuffer( g, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof( float ), n * 3 + 1 );
	unsigned *idx = (unsigned *)rtcSetNewGeometryBuffer( g, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof( unsigned ), n );
	for ( size_t i = 0; i < n; i++ )
	{
		memcpy( v + i * 9, b.tris[i].v, 9 * sizeof( float ) );
		idx[i * 3] = (unsigned)( i * 3 ); idx[i * 3 + 1] = (unsigned)( i * 3 + 1 ); idx[i * 3 + 2] = (unsigned)( i * 3 + 2 );
	}
	rtcCommitGeometry( g );
	rtcAttachGeometry( b.scene, g );
	rtcReleaseGeometry( g );
	rtcCommitScene( b.scene );
	g_occlusionScene = b.scene;
	return true;
}

static inline bool Trace( const Bake &b, V3 org, V3 dir, float &t, unsigned &prim )
{
	RTCRayHit rh;
	rh.ray.org_x = org.x; rh.ray.org_y = org.y; rh.ray.org_z = org.z; rh.ray.tnear = 0.0f;
	rh.ray.dir_x = dir.x; rh.ray.dir_y = dir.y; rh.ray.dir_z = dir.z; rh.ray.time = 0.0f;
	rh.ray.tfar = 1e7f; rh.ray.mask = 0xFFFFFFFFu; rh.ray.id = 0; rh.ray.flags = 0;
	rh.hit.geomID = RTC_INVALID_GEOMETRY_ID; rh.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
	rtcIntersect1( b.scene, &rh );
	if ( rh.hit.geomID == RTC_INVALID_GEOMETRY_ID ) return false;
	t = rh.ray.tfar; prim = rh.hit.primID;
	return true;
}

// Light arriving along dir, excluding anything VRAD's direct pass already counted.
static V3 PathRadiance( const Bake &b, V3 org, V3 dir, Rng &rng, uint64_t &rayCount )
{
	V3 total, through( 1, 1, 1 );
	for ( unsigned bounce = 0; bounce < b.hdr.maxBounces; bounce++ )
	{
		float t; unsigned prim;
		rayCount++;
		if ( !Trace( b, org, dir, t, prim ) ) break;
		if ( b.tris[prim].kind != VB15_TRI_WORLD ) break; // sky is in direct; props absorb, as in stock VRAD
		V3 hit = org + dir * t;
		int si = b.tree.FindSurface( hit, dir );
		if ( si < 0 ) break;
		const Vb15Sample &s = b.samples[si];
		through = through * V3( s.albedo );
		total = total + through * V3( s.direct );
		if ( bounce >= 3 ) // Russian roulette keeps long paths cheap and unbiased
		{
			float p = std::min( 0.95f, std::max( through.x, std::max( through.y, through.z ) ) );
			if ( p <= 0.0f || rng.F() >= p ) break;
			through = through * ( 1.0f / p );
		}
		V3 n( s.normal );
		org = hit + n * 0.1f;
		dir = CosineDir( n, rng.F(), rng.F() );
	}
	return total;
}

static void GatherSample( Bake &b, uint32_t i, uint64_t &rayCount )
{
	const Vb15Sample &s = b.samples[i];
	Vb15Result &out = b.results[i];
	memset( &out, 0, sizeof( out ) );
	if ( s.flags & VB15_SAMPLE_INVALID ) return;

	Rng rng( ( (uint64_t)b.hdr.seed << 32 ) ^ i );
	V3 n( s.normal ), org = V3( s.pos ) + n * 0.1f;
	const bool bumped = ( s.flags & VB15_SAMPLE_BUMPED ) != 0;
	const int N = (int)std::max( 1u, b.hdr.raysPerSample );
	const int grid = std::max( 1, (int)std::sqrt( (double)N ) );
	V3 sum[4];
	for ( int r = 0; r < N; r++ )
	{
		// Stratify the first ray; rays beyond the square grid are plain random.
		float u1 = rng.F(), u2 = rng.F();
		if ( r < grid * grid ) { u1 = ( ( r % grid ) + u1 ) / grid; u2 = ( ( r / grid ) + u2 ) / grid; }
		V3 dir = CosineDir( n, u1, u2 );
		V3 L = PathRadiance( b, org, dir, rng, rayCount );
		sum[0] = sum[0] + L;
		if ( bumped )
		{
			float dn = std::max( Dot( dir, n ), 1e-3f );
			for ( int k = 0; k < 3; k++ )
			{
				float db = Dot( dir, V3( s.bump[k] ) );
				// Same form as stock GatherLight's dot(delta, basis) / dot(delta, normal).
				// The clamp stops grazing rays from producing fireflies.
				if ( db > 0 ) sum[k + 1] = sum[k + 1] + L * std::min( db / dn, 8.0f );
			}
		}
	}
	float inv = 1.0f / N;
	for ( int c = 0; c < ( bumped ? 4 : 1 ); c++ )
	{
		out.light[c][0] = sum[c].x * inv; out.light[c][1] = sum[c].y * inv; out.light[c][2] = sum[c].z * inv;
	}
}

// Fill samples that sat inside solid from valid samples on the same face.
static void FillInvalid( Bake &b )
{
	std::map<uint64_t, uint32_t> grid;
	auto key = []( unsigned f, int s, int t ) { return ( (uint64_t)f << 40 ) ^ ( (uint64_t)(uint32_t)( s & 0xFFFFF ) << 20 ) ^ (uint32_t)( t & 0xFFFFF ); };
	std::vector<uint8_t> valid( b.samples.size() );
	std::vector<uint32_t> todo;
	for ( uint32_t i = 0; i < b.samples.size(); i++ )
	{
		const Vb15Sample &s = b.samples[i];
		valid[i] = !( s.flags & VB15_SAMPLE_INVALID );
		grid[key( s.face, s.s, s.t )] = i;
		if ( !valid[i] ) todo.push_back( i );
	}
	for ( int pass = 0; pass < 64 && !todo.empty(); pass++ )
	{
		std::vector<uint32_t> next, filled;
		std::vector<Vb15Result> values;
		for ( uint32_t i : todo )
		{
			const Vb15Sample &s = b.samples[i];
			Vb15Result acc; memset( &acc, 0, sizeof( acc ) );
			int cnt = 0;
			for ( int dt = -1; dt <= 1; dt++ ) for ( int ds = -1; ds <= 1; ds++ )
			{
				auto it = grid.find( key( s.face, s.s + ds, s.t + dt ) );
				if ( it == grid.end() || !valid[it->second] ) continue;
				const Vb15Result &o = b.results[it->second];
				for ( int c = 0; c < 4; c++ ) for ( int k = 0; k < 3; k++ ) acc.light[c][k] += o.light[c][k];
				cnt++;
			}
			if ( !cnt ) { next.push_back( i ); continue; }
			for ( int c = 0; c < 4; c++ ) for ( int k = 0; k < 3; k++ ) acc.light[c][k] /= cnt;
			filled.push_back( i ); values.push_back( acc );
		}
		if ( filled.empty() ) break;
		for ( size_t j = 0; j < filled.size(); j++ ) { b.results[filled[j]] = values[j]; valid[filled[j]] = 1; }
		todo.swap( next );
	}
}

#ifdef VB15_USE_OIDN
// Lay each face's luxel grid out in one image with padded gutters, so the denoiser
// never sees a hard edge between unrelated faces.
static bool Denoise( Bake &b )
{
	const int gutter = 8, atlasW = 4096;
	struct Rect { int minS, minT, maxS, maxT, x, y; std::vector<uint32_t> ids; };
	std::map<unsigned, Rect> faces;
	for ( uint32_t i = 0; i < b.samples.size(); i++ )
	{
		const Vb15Sample &s = b.samples[i];
		auto it = faces.find( s.face );
		if ( it == faces.end() ) it = faces.insert( { s.face, Rect{ s.s, s.t, s.s, s.t, 0, 0, {} } } ).first;
		Rect &r = it->second;
		r.minS = std::min( r.minS, s.s ); r.minT = std::min( r.minT, s.t );
		r.maxS = std::max( r.maxS, s.s ); r.maxT = std::max( r.maxT, s.t );
		r.ids.push_back( i );
	}
	int x = 0, y = 0, rowH = 0;
	for ( auto &kv : faces )
	{
		Rect &r = kv.second;
		int w = r.maxS - r.minS + 1 + gutter * 2, h = r.maxT - r.minT + 1 + gutter * 2;
		if ( w > atlasW ) { fprintf( stderr, "vrad15_bake: face %u is too wide to denoise\n", kv.first ); return false; }
		if ( x + w > atlasW ) { x = 0; y += rowH; rowH = 0; }
		r.x = x; r.y = y;
		x += w; rowH = std::max( rowH, h );
	}
	const int atlasH = y + rowH;
	if ( atlasH <= 0 ) return true;

	OIDNDevice dev = oidnNewDevice( OIDN_DEVICE_TYPE_DEFAULT );
	oidnCommitDevice( dev );
	std::vector<float> color( (size_t)atlasW * atlasH * 3 ), out( color.size() );
	std::vector<uint8_t> mask( (size_t)atlasW * atlasH );

	for ( int channel = 0; channel < 4; channel++ )
	{
		std::fill( color.begin(), color.end(), 0.0f );
		std::fill( mask.begin(), mask.end(), (uint8_t)0 );
		bool any = false;
		for ( auto &kv : faces )
		{
			Rect &r = kv.second;
			for ( uint32_t id : r.ids )
			{
				const Vb15Sample &s = b.samples[id];
				if ( channel > 0 && !( s.flags & VB15_SAMPLE_BUMPED ) ) continue;
				size_t p = (size_t)( r.y + gutter + s.t - r.minT ) * atlasW + ( r.x + gutter + s.s - r.minS );
				memcpy( &color[p * 3], b.results[id].light[channel], 12 );
				mask[p] = 1; any = true;
			}
		}
		if ( !any ) continue;

		// Dilate into holes and gutters, face by face.
		for ( auto &kv : faces )
		{
			Rect &r = kv.second;
			int rx0 = r.x, ry0 = r.y, rw = r.maxS - r.minS + 1 + gutter * 2, rh = r.maxT - r.minT + 1 + gutter * 2;
			for ( int pass = 0; pass < rw + rh; pass++ )
			{
				std::vector<std::pair<size_t, V3>> add;
				for ( int py = ry0; py < ry0 + rh; py++ ) for ( int px = rx0; px < rx0 + rw; px++ )
				{
					size_t p = (size_t)py * atlasW + px;
					if ( mask[p] ) continue;
					V3 acc; int cnt = 0;
					for ( int dy = -1; dy <= 1; dy++ ) for ( int dx = -1; dx <= 1; dx++ )
					{
						int qx = px + dx, qy = py + dy;
						if ( qx < rx0 || qy < ry0 || qx >= rx0 + rw || qy >= ry0 + rh ) continue;
						size_t q = (size_t)qy * atlasW + qx;
						if ( mask[q] ) { acc = acc + V3( &color[q * 3] ); cnt++; }
					}
					if ( cnt ) add.push_back( { p, acc * ( 1.0f / cnt ) } );
				}
				if ( add.empty() ) break;
				for ( auto &a : add )
				{
					color[a.first * 3] = a.second.x; color[a.first * 3 + 1] = a.second.y; color[a.first * 3 + 2] = a.second.z;
					mask[a.first] = 1;
				}
			}
		}

		OIDNFilter filter = oidnNewFilter( dev, "RT" );
		oidnSetSharedFilterImage( filter, "color", color.data(), OIDN_FORMAT_FLOAT3, atlasW, atlasH, 0, 0, 0 );
		oidnSetSharedFilterImage( filter, "output", out.data(), OIDN_FORMAT_FLOAT3, atlasW, atlasH, 0, 0, 0 );
		oidnSetFilterBool( filter, "hdr", true );
		oidnCommitFilter( filter );
		oidnExecuteFilter( filter );
		oidnReleaseFilter( filter );
		const char *err;
		if ( oidnGetDeviceError( dev, &err ) != OIDN_ERROR_NONE )
		{
			fprintf( stderr, "vrad15_bake: denoiser error: %s\n", err );
			oidnReleaseDevice( dev );
			return false;
		}
		for ( auto &kv : faces )
		{
			Rect &r = kv.second;
			for ( uint32_t id : r.ids )
			{
				const Vb15Sample &s = b.samples[id];
				if ( channel > 0 && !( s.flags & VB15_SAMPLE_BUMPED ) ) continue;
				size_t p = (size_t)( r.y + gutter + s.t - r.minT ) * atlasW + ( r.x + gutter + s.s - r.minS );
				for ( int k = 0; k < 3; k++ ) b.results[id].light[channel][k] = std::max( 0.0f, out[p * 3 + k] );
			}
		}
	}
	oidnReleaseDevice( dev );
	return true;
}
#endif

// Denoising each face alone can leave a step where faces meet. Blend samples next to
// a coplanar neighbouring face; samples with no such neighbour are left untouched.
static void WeldSeams( Bake &b )
{
	const std::vector<Vb15Result> src = b.results;
	for ( uint32_t i = 0; i < b.samples.size(); i++ )
	{
		const Vb15Sample &s = b.samples[i];
		if ( s.flags & VB15_SAMPLE_INVALID ) continue;
		V3 p( s.pos ), n( s.normal );
		float r = s.radius * 1.5f, wsum[4] = { 1, 1, 1, 1 };
		float acc[4][3];
		memcpy( acc, src[i].light, sizeof( acc ) );
		bool otherFace = false;
		b.tree.ForRadius( p, r, [&]( uint32_t j )
		{
			if ( j == i ) return;
			const Vb15Sample &o = b.samples[j];
			if ( Dot( n, V3( o.normal ) ) < 0.98f ) return;
			V3 d = V3( o.pos ) - p;
			if ( std::fabs( Dot( n, d ) ) > 1.0f ) return;
			V3 lift = n * 0.5f;
			if ( !Visible( p + lift, V3( o.pos ) + lift ) ) return;
			if ( o.face != s.face ) otherFace = true;
			float w = std::max( 0.0f, 1.0f - std::sqrt( Dot( d, d ) ) / r );
			for ( int k = 0; k < 3; k++ ) acc[0][k] += src[j].light[0][k] * w;
			wsum[0] += w;
			// Bump bases are only comparable when they point the same way.
			if ( ( s.flags & o.flags & VB15_SAMPLE_BUMPED ) && Dot( V3( s.bump[0] ), V3( o.bump[0] ) ) > 0.98f )
				for ( int c = 1; c < 4; c++ ) { for ( int k = 0; k < 3; k++ ) acc[c][k] += src[j].light[c][k] * w; wsum[c] += w; }
		} );
		if ( !otherFace ) continue;
		for ( int c = 0; c < 4; c++ ) for ( int k = 0; k < 3; k++ ) b.results[i].light[c][k] = acc[c][k] / wsum[c];
	}
}

int main( int argc, char **argv )
{
	if ( argc < 3 ) { fprintf( stderr, "usage: vrad15_bake <scene file> <result file>\n" ); return 2; }
	Bake b;
	if ( !ReadAll( argv[1], b ) ) return 1;
	int threads = b.hdr.threads > 0 ? b.hdr.threads : (int)std::max( 1u, std::thread::hardware_concurrency() );
	printf( "vrad15_bake: %u triangles, %u samples, %u rays per sample, up to %u bounces, %d threads\n",
		b.hdr.numTris, b.hdr.numSamples, b.hdr.raysPerSample, b.hdr.maxBounces, threads );
	fflush( stdout );

	auto t0 = std::chrono::steady_clock::now();
	if ( !BuildScene( b, threads ) ) return 1;
	b.tree.Build( b.samples );
	b.results.resize( b.samples.size() );

	std::atomic<uint32_t> next{ 0 };
	const uint32_t total = (uint32_t)b.samples.size(), chunk = 64;
	std::vector<std::thread> pool;
	for ( int t = 0; t < threads; t++ )
		pool.emplace_back( [&]
		{
			// Embree runs faster with denormals flushed, and each thread has its own setting.
			_MM_SET_FLUSH_ZERO_MODE( _MM_FLUSH_ZERO_ON );
			_MM_SET_DENORMALS_ZERO_MODE( _MM_DENORMALS_ZERO_ON );
			uint64_t rays = 0;
			for ( ;; )
			{
				uint32_t start = next.fetch_add( chunk );
				if ( start >= total ) break;
				for ( uint32_t i = start; i < std::min( start + chunk, total ); i++ ) GatherSample( b, i, rays );
			}
			b.rays += rays;
		} );
	for ( auto &t : pool ) t.join();
	double secs = std::chrono::duration<double>( std::chrono::steady_clock::now() - t0 ).count();
	printf( "vrad15_bake: traced %llu rays in %.1f s\n", (unsigned long long)b.rays.load(), secs );

	FillInvalid( b );
	if ( b.hdr.options & VB15_OPT_DENOISE )
	{
#ifdef VB15_USE_OIDN
		if ( !Denoise( b ) ) return 1;
		printf( "vrad15_bake: denoised\n" );
#else
		fprintf( stderr, "vrad15_bake: denoising requested, but this build has no OIDN\n" );
		return 1;
#endif
	}
	WeldSeams( b );

	rtcReleaseScene( b.scene );
	rtcReleaseDevice( b.device );

	// Write beside the target and rename, so VRAD never reads a half-written file.
	std::string tmp = std::string( argv[2] ) + ".tmp";
	FILE *f = fopen( tmp.c_str(), "wb" );
	if ( !f ) { fprintf( stderr, "vrad15_bake: cannot write %s\n", tmp.c_str() ); return 1; }
	Vb15ResultHeader rh = { VB15_RESULT_MAGIC, VB15_VERSION, total };
	bool ok = fwrite( &rh, sizeof( rh ), 1, f ) == 1 && ( total == 0 || fwrite( b.results.data(), sizeof( Vb15Result ), total, f ) == total );
	ok = ( fclose( f ) == 0 ) && ok;
	remove( argv[2] );
	if ( !ok || rename( tmp.c_str(), argv[2] ) != 0 ) { fprintf( stderr, "vrad15_bake: failed writing %s\n", argv[2] ); return 1; }
	return 0;
}
