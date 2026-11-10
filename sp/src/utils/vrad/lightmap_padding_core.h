//-----------------------------------------------------------------------------
// lightmap_padding_core.h - engine-free core of VRAD's -bicubiclightmaps.
//
// The engine packs face lightmaps into atlas pages with no gap between them.
// Bilinear filtering never reads past a face's edge luxels, but a bicubic filter
// reads one luxel further. Padding stores each brush face's lightmap with one extra
// ring of luxels, copied from its edge, and widens the face's lightmap rectangle in
// the BSP to match: mins move back by one luxel and the size grows by two. The
// engine derives lightmap texture coordinates from those fields, so the face still
// lands on its original luxels.
//
// Displacements are left alone, because the engine computes their lightmap
// coordinates from the displacement itself rather than from these fields.
//-----------------------------------------------------------------------------
#ifndef LIGHTMAP_PADDING_CORE_H
#define LIGHTMAP_PADDING_CORE_H

#include <string.h>
#include <string>
#include <vector>

struct LmpFace
{
	int  lightofs;   // byte offset of the first luxel, or -1 for no lightmap
	int  mins[2];    // m_LightmapTextureMinsInLuxels
	int  size[2];    // m_LightmapTextureSizeInLuxels (luxel count minus one)
	int  numStyles;  // light styles stored for the face
	int  blocks;     // lightmaps per style: 4 when bump lit, otherwise 1
	bool paddable;   // brush face that takes part in padding
};

// Rebuilds the light lump with every paddable face padded (dir = +1) or unpadded
// (dir = -1). Other faces are copied unchanged. Updates the faces in place.
// Returns false with a message on any inconsistency, leaving faces untouched.
inline bool LmpReshape( std::vector<LmpFace> &faces, const std::vector<unsigned char> &in,
	std::vector<unsigned char> &out, int dir, std::string &err )
{
	std::vector<LmpFace> result = faces;
	out.clear();
	out.reserve( in.size() + in.size() / 4 );
	for ( size_t i = 0; i < result.size(); i++ )
	{
		LmpFace &f = result[i];
		if ( f.lightofs < 0 || f.numStyles <= 0 )
			continue;

		const int w = f.size[0] + 1, h = f.size[1] + 1;
		const size_t avgBytes = (size_t)f.numStyles * 4;
		const size_t maps = (size_t)f.numStyles * f.blocks;
		const size_t bytes = maps * w * h * 4;
		if ( w <= 0 || h <= 0 || (size_t)f.lightofs < avgBytes || (size_t)f.lightofs + bytes > in.size() )
		{
			err = "face " + std::to_string( i ) + " has lighting outside the light lump";
			return false;
		}

		// Average colours sit just before lightofs, one per style.
		out.insert( out.end(), in.begin() + ( f.lightofs - avgBytes ), in.begin() + f.lightofs );
		const int newOfs = (int)out.size();

		if ( !f.paddable )
		{
			out.insert( out.end(), in.begin() + f.lightofs, in.begin() + f.lightofs + bytes );
			f.lightofs = newOfs;
			continue;
		}

		const int nw = w + 2 * dir, nh = h + 2 * dir;
		if ( nw < 1 || nh < 1 )
		{
			err = "face " + std::to_string( i ) + " is too small to have been padded";
			return false;
		}
		const unsigned char *src = &in[f.lightofs];
		for ( size_t m = 0; m < maps; m++ )
		{
			const unsigned char *map = src + m * w * h * 4;
			for ( int y = 0; y < nh; y++ )
			{
				int sy = y - dir;
				sy = sy < 0 ? 0 : ( sy >= h ? h - 1 : sy );
				for ( int x = 0; x < nw; x++ )
				{
					int sx = x - dir;
					sx = sx < 0 ? 0 : ( sx >= w ? w - 1 : sx );
					const unsigned char *p = map + ( (size_t)sy * w + sx ) * 4;
					out.insert( out.end(), p, p + 4 );
				}
			}
		}
		f.lightofs = newOfs;
		f.mins[0] -= dir;
		f.mins[1] -= dir;
		f.size[0] += 2 * dir;
		f.size[1] += 2 * dir;
	}
	faces.swap( result );
	return true;
}

// Finds the quoted value of a key in the first entity block (worldspawn).
// On success, [start, end) is the value's range inside ents.
inline bool LmpFindWorldKey( const std::string &ents, const char *key, size_t &start, size_t &end )
{
	size_t p = ents.find( '{' );
	if ( p == std::string::npos )
		return false;
	std::string tokens[2];
	size_t valStart = 0, valEnd = 0;
	int n = 0;
	for ( p++; p < ents.size(); p++ )
	{
		char c = ents[p];
		if ( c == '}' )
			return false;
		if ( c != '"' )
			continue;
		size_t close = ents.find( '"', p + 1 );
		if ( close == std::string::npos )
			return false;
		tokens[n] = ents.substr( p + 1, close - p - 1 );
		if ( n == 1 )
		{
			valStart = p + 1;
			valEnd = close;
			if ( tokens[0] == key )
			{
				start = valStart;
				end = valEnd;
				return true;
			}
		}
		n ^= 1;
		p = close;
	}
	return false;
}

inline std::string LmpGetWorldKey( const std::string &ents, const char *key )
{
	size_t s, e;
	return LmpFindWorldKey( ents, key, s, e ) ? ents.substr( s, e - s ) : std::string();
}

inline bool LmpSetWorldKey( std::string &ents, const char *key, const char *value )
{
	size_t s, e;
	if ( LmpFindWorldKey( ents, key, s, e ) )
	{
		ents.replace( s, e - s, value );
		return true;
	}
	size_t brace = ents.find( '{' );
	if ( brace == std::string::npos )
		return false;
	size_t at = brace + 1;
	if ( at < ents.size() && ents[at] == '\r' ) at++;
	if ( at < ents.size() && ents[at] == '\n' ) at++;
	ents.insert( at, std::string( "\"" ) + key + "\" \"" + value + "\"\n" );
	return true;
}

#endif
