//========= Mapbase - https://github.com/mapbase-source/source-sdk-2013 ============//
//
// Purpose: Shared code for func_shatterglass, a Source 2-style breakable glass pane.
//
//			The pane is split into a procedural Voronoi shard pattern which is
//			generated identically on the server and the client from a seed, so only
//			per-shard state bits need to be networked.
//
// Author(s): HybridSource1
//
//=============================================================================//

#ifndef SHATTERGLASS_SHARED_H
#define SHATTERGLASS_SHARED_H
#ifdef _WIN32
#pragma once
#endif

#include "mathlib/vector.h"
#include "mathlib/vector2d.h"
#include "tier1/utlvector.h"

#define SHATTERGLASS_MAX_SHARDS			512
#define SHATTERGLASS_STATE_WORDS		( SHATTERGLASS_MAX_SHARDS / 32 )
#define SHATTERGLASS_MAX_VERTS			16
#define SHATTERGLASS_MAX_NEIGHBORS		16

#define SHATTERGLASS_MIN_SHARD_SIZE		2.0f
#define SHATTERGLASS_DEFAULT_SHARD_SIZE	8.0f

// Edge tags for shard polygon edges which lie on the pane border
enum
{
	SHATTERGLASS_EDGE_LEFT = -1,	// u == 0
	SHATTERGLASS_EDGE_RIGHT = -2,	// u == width
	SHATTERGLASS_EDGE_BOTTOM = -3,	// v == 0
	SHATTERGLASS_EDGE_TOP = -4,		// v == height
};

// Bits for anchored pane borders
#define SHATTERGLASS_ANCHOR_LEFT		(1<<0)
#define SHATTERGLASS_ANCHOR_RIGHT		(1<<1)
#define SHATTERGLASS_ANCHOR_BOTTOM		(1<<2)
#define SHATTERGLASS_ANCHOR_TOP			(1<<3)
#define SHATTERGLASS_ANCHOR_ALL			(SHATTERGLASS_ANCHOR_LEFT|SHATTERGLASS_ANCHOR_RIGHT|SHATTERGLASS_ANCHOR_BOTTOM|SHATTERGLASS_ANCHOR_TOP)

inline int ShatterGlass_EdgeTagToAnchorBit( int nTag )
{
	switch ( nTag )
	{
	case SHATTERGLASS_EDGE_LEFT:	return SHATTERGLASS_ANCHOR_LEFT;
	case SHATTERGLASS_EDGE_RIGHT:	return SHATTERGLASS_ANCHOR_RIGHT;
	case SHATTERGLASS_EDGE_BOTTOM:	return SHATTERGLASS_ANCHOR_BOTTOM;
	case SHATTERGLASS_EDGE_TOP:		return SHATTERGLASS_ANCHOR_TOP;
	}
	return 0;
}

//-----------------------------------------------------------------------------
// State bit helpers (networked as arrays of ints)
//-----------------------------------------------------------------------------
inline bool ShatterGlass_IsBitSet( const int *pWords, int i )
{
	return ( ((unsigned int)pWords[i >> 5]) & ( 1u << ( i & 31 ) ) ) != 0;
}

//-----------------------------------------------------------------------------
// Purpose: Describes where the pane sits in the entity's local space.
//			The pane is the entity's OBB; its thinnest axis is the normal.
//-----------------------------------------------------------------------------
struct ShatterGlassPanel_t
{
	void Init( const Vector &vecMins, const Vector &vecMaxs );

	// Local-space position of a point on the pane
	Vector LocalPoint( float u, float v, float n ) const;

	// Converts a local-space position to pane coordinates (u, v), and returns distance along the normal axis
	float LocalToPanel( const Vector &vecLocal, Vector2D &uv ) const;

	Vector	m_vecMins;
	Vector	m_vecMaxs;
	int		m_iU, m_iV, m_iN;	// Local axis indices
	float	m_flWidth;			// Extent along U
	float	m_flHeight;			// Extent along V
	float	m_flThickness;		// Extent along N
};

//-----------------------------------------------------------------------------
// Purpose: A procedural Voronoi shard pattern covering a width x height pane
//-----------------------------------------------------------------------------
class CShatterGlassPattern
{
public:
	struct Shard_t
	{
		Vector2D	verts[SHATTERGLASS_MAX_VERTS];	// Counter-clockwise in (u, v)
		short		edgeTag[SHATTERGLASS_MAX_VERTS];	// Tag of the edge from verts[i] to verts[i+1]: neighbor shard index or SHATTERGLASS_EDGE_*
		int			nVerts;

		Vector2D	centroid;
		Vector2D	mins;
		Vector2D	maxs;
		float		area;

		short		neighbors[SHATTERGLASS_MAX_NEIGHBORS];
		int			nNeighbors;
		int			nBorderAnchorBits;	// SHATTERGLASS_ANCHOR_* bits for borders this shard touches
	};

	CShatterGlassPattern();

	void			Generate( float flWidth, float flHeight, float flShardSize, int nSeed );
	void			Clear() { m_Shards.Purge(); m_flTotalArea = 0.0f; }

	bool			IsValid() const { return m_Shards.Count() > 0; }
	int				Count() const { return m_Shards.Count(); }
	const Shard_t	&Get( int i ) const { return m_Shards[i]; }
	float			TotalArea() const { return m_flTotalArea; }
	float			CellSize() const { return m_flCellSize; }

	// Returns the shard containing the given point, or -1
	int				FindShardAt( const Vector2D &pt ) const;

	// Separating axis test between a shard and an axis-aligned rectangle (center + half extents)
	static bool		ShardOverlapsRect( const Shard_t &shard, const Vector2D &vecCenter, const Vector2D &vecHalfExtents );

	// Clips a convex polygon against the half-plane dot( p, normal ) <= dist. Returns the new vertex count.
	// pTags may be NULL; nClipTag is assigned to the newly created edge.
	static int		ClipPolygon( const Vector2D *pIn, const short *pInTags, int nIn, const Vector2D &normal, float dist, short nClipTag,
						Vector2D *pOut, short *pOutTags, int nMaxOut );

	// Area and centroid of a convex polygon
	static void		ComputeAreaAndCentroid( const Vector2D *pVerts, int nVerts, float &flArea, Vector2D &vecCentroid );

private:
	CUtlVector<Shard_t>	m_Shards;
	float				m_flTotalArea;
	float				m_flCellSize;

	int					m_nGridX, m_nGridY;
	float				m_flGridCellW, m_flGridCellH;

public:
	// Collects shards whose bounds overlap the given rectangle. Returns the number written.
	int				GatherShardsInRect( const Vector2D &vecMins, const Vector2D &vecMaxs, int *pOut, int nMaxOut ) const;
};

//-----------------------------------------------------------------------------
// Purpose: Ray/box test against the shards that are still intact.
//			Used by TestCollision() on both the server and the client so bullets,
//			players and debris can pass through holes in the pane.
//-----------------------------------------------------------------------------
struct Ray_t;
class CGameTrace;
typedef CGameTrace trace_t;

bool ShatterGlass_TestCollision( const Ray_t &ray, unsigned int fContentsMask, trace_t &tr,
	const matrix3x4_t &entityToWorld, const ShatterGlassPanel_t &panel, const CShatterGlassPattern &pattern,
	const int *pBrokenBits, int nBrokenCount, int nContents, unsigned short nSurfaceProps );

#endif // SHATTERGLASS_SHARED_H
