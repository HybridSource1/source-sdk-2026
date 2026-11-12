//========= Mapbase - https://github.com/mapbase-source/source-sdk-2013 ============//
//
// Purpose: Shared code for func_shatterglass, a Source 2-style breakable glass pane.
//
// Author(s): HybridSource1
//
//=============================================================================//

#include "cbase.h"
#include "shatterglass_shared.h"
#include "vstdlib/random.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-----------------------------------------------------------------------------
// ShatterGlassPanel_t
//-----------------------------------------------------------------------------
void ShatterGlassPanel_t::Init( const Vector &vecMins, const Vector &vecMaxs )
{
	m_vecMins = vecMins;
	m_vecMaxs = vecMaxs;

	Vector vecSize = vecMaxs - vecMins;

	// The thinnest axis is the pane's normal
	m_iN = 0;
	if ( vecSize[1] < vecSize[m_iN] )
		m_iN = 1;
	if ( vecSize[2] < vecSize[m_iN] )
		m_iN = 2;

	// "Up" is Z whenever the pane is vertical, otherwise Y
	m_iV = ( m_iN == 2 ) ? 1 : 2;
	m_iU = 3 - m_iN - m_iV;

	m_flWidth = MAX( vecSize[m_iU], 1.0f );
	m_flHeight = MAX( vecSize[m_iV], 1.0f );
	m_flThickness = vecSize[m_iN];
}

Vector ShatterGlassPanel_t::LocalPoint( float u, float v, float n ) const
{
	Vector vecResult;
	vecResult[m_iU] = m_vecMins[m_iU] + u;
	vecResult[m_iV] = m_vecMins[m_iV] + v;
	vecResult[m_iN] = m_vecMins[m_iN] + n;
	return vecResult;
}

float ShatterGlassPanel_t::LocalToPanel( const Vector &vecLocal, Vector2D &uv ) const
{
	uv.x = vecLocal[m_iU] - m_vecMins[m_iU];
	uv.y = vecLocal[m_iV] - m_vecMins[m_iV];
	return vecLocal[m_iN] - m_vecMins[m_iN];
}

//-----------------------------------------------------------------------------
// CShatterGlassPattern
//-----------------------------------------------------------------------------
CShatterGlassPattern::CShatterGlassPattern()
{
	m_flTotalArea = 0.0f;
	m_flCellSize = SHATTERGLASS_DEFAULT_SHARD_SIZE;
	m_nGridX = m_nGridY = 0;
	m_flGridCellW = m_flGridCellH = 1.0f;
}

int CShatterGlassPattern::ClipPolygon( const Vector2D *pIn, const short *pInTags, int nIn, const Vector2D &normal, float dist, short nClipTag,
	Vector2D *pOut, short *pOutTags, int nMaxOut )
{
	int nOut = 0;
	for ( int i = 0; i < nIn; i++ )
	{
		const Vector2D &a = pIn[i];
		const Vector2D &b = pIn[(i + 1) % nIn];
		short nTag = pInTags ? pInTags[i] : 0;

		float da = DotProduct2D( a, normal ) - dist;
		float db = DotProduct2D( b, normal ) - dist;
		bool bAIn = ( da <= 0.0f );
		bool bBIn = ( db <= 0.0f );

		if ( bAIn )
		{
			if ( nOut < nMaxOut )
			{
				pOut[nOut] = a;
				if ( pOutTags ) pOutTags[nOut] = nTag;
				nOut++;
			}

			if ( !bBIn && nOut < nMaxOut )
			{
				// Leaving: the new edge runs along the clip line
				float t = da / ( da - db );
				pOut[nOut] = a + ( b - a ) * t;
				if ( pOutTags ) pOutTags[nOut] = nClipTag;
				nOut++;
			}
		}
		else if ( bBIn && nOut < nMaxOut )
		{
			// Entering: the rest of this edge keeps its original tag
			float t = da / ( da - db );
			pOut[nOut] = a + ( b - a ) * t;
			if ( pOutTags ) pOutTags[nOut] = nTag;
			nOut++;
		}
	}

	return nOut;
}

void CShatterGlassPattern::ComputeAreaAndCentroid( const Vector2D *pVerts, int nVerts, float &flArea, Vector2D &vecCentroid )
{
	flArea = 0.0f;
	vecCentroid.Init();

	if ( nVerts <= 0 )
		return;

	float flCx = 0.0f, flCy = 0.0f;
	for ( int i = 0; i < nVerts; i++ )
	{
		const Vector2D &a = pVerts[i];
		const Vector2D &b = pVerts[(i + 1) % nVerts];
		float flCross = a.x * b.y - b.x * a.y;
		flArea += flCross;
		flCx += ( a.x + b.x ) * flCross;
		flCy += ( a.y + b.y ) * flCross;
	}
	flArea *= 0.5f;

	if ( fabs( flArea ) > 1e-6f )
	{
		vecCentroid.x = flCx / ( 6.0f * flArea );
		vecCentroid.y = flCy / ( 6.0f * flArea );
	}
	else
	{
		// Degenerate; use the vertex average
		for ( int i = 0; i < nVerts; i++ )
			vecCentroid += pVerts[i];
		vecCentroid /= (float)nVerts;
	}

	flArea = fabs( flArea );
}

void CShatterGlassPattern::Generate( float flWidth, float flHeight, float flShardSize, int nSeed )
{
	m_Shards.Purge();
	m_flTotalArea = 0.0f;

	flWidth = MAX( flWidth, 1.0f );
	flHeight = MAX( flHeight, 1.0f );
	flShardSize = MAX( flShardSize, SHATTERGLASS_MIN_SHARD_SIZE );

	// Fit the shard grid, growing the shard size if there would be too many
	int nx, ny;
	for ( ;; )
	{
		nx = MAX( 1, (int)( flWidth / flShardSize + 0.5f ) );
		ny = MAX( 1, (int)( flHeight / flShardSize + 0.5f ) );
		if ( nx * ny <= SHATTERGLASS_MAX_SHARDS )
			break;
		flShardSize *= 1.1f;
	}

	m_flCellSize = flShardSize;
	m_nGridX = nx;
	m_nGridY = ny;
	m_flGridCellW = flWidth / nx;
	m_flGridCellH = flHeight / ny;

	// Jittered grid of Voronoi sites
	CUniformRandomStream random;
	random.SetSeed( nSeed );

	CUtlVector<Vector2D> sites;
	sites.SetCount( nx * ny );
	for ( int j = 0; j < ny; j++ )
	{
		for ( int i = 0; i < nx; i++ )
		{
			Vector2D &site = sites[j * nx + i];
			site.x = ( i + 0.5f + random.RandomFloat( -0.4f, 0.4f ) ) * m_flGridCellW;
			site.y = ( j + 0.5f + random.RandomFloat( -0.4f, 0.4f ) ) * m_flGridCellH;
		}
	}

	m_Shards.SetCount( nx * ny );

	Vector2D bufA[SHATTERGLASS_MAX_VERTS], bufB[SHATTERGLASS_MAX_VERTS];
	short tagsA[SHATTERGLASS_MAX_VERTS], tagsB[SHATTERGLASS_MAX_VERTS];

	for ( int j = 0; j < ny; j++ )
	{
		for ( int i = 0; i < nx; i++ )
		{
			int nIndex = j * nx + i;
			const Vector2D &site = sites[nIndex];

			// Start with the whole pane (counter-clockwise)
			int nVerts = 4;
			bufA[0].Init( 0.0f, 0.0f );			tagsA[0] = SHATTERGLASS_EDGE_BOTTOM;
			bufA[1].Init( flWidth, 0.0f );		tagsA[1] = SHATTERGLASS_EDGE_RIGHT;
			bufA[2].Init( flWidth, flHeight );	tagsA[2] = SHATTERGLASS_EDGE_TOP;
			bufA[3].Init( 0.0f, flHeight );		tagsA[3] = SHATTERGLASS_EDGE_LEFT;

			Vector2D *pCur = bufA, *pNext = bufB;
			short *pCurTags = tagsA, *pNextTags = tagsB;

			// Clip against the bisectors of nearby sites
			for ( int nj = MAX( 0, j - 2 ); nj <= MIN( ny - 1, j + 2 ); nj++ )
			{
				for ( int ni = MAX( 0, i - 2 ); ni <= MIN( nx - 1, i + 2 ); ni++ )
				{
					int nOther = nj * nx + ni;
					if ( nOther == nIndex )
						continue;

					const Vector2D &other = sites[nOther];
					Vector2D normal = other - site;
					float dist = 0.5f * ( other.LengthSqr() - site.LengthSqr() );

					int nNew = ClipPolygon( pCur, pCurTags, nVerts, normal, dist, (short)nOther, pNext, pNextTags, SHATTERGLASS_MAX_VERTS );
					if ( nNew < 3 )
					{
						nVerts = 0;
						break;
					}

					nVerts = nNew;
					V_swap( pCur, pNext );
					V_swap( pCurTags, pNextTags );
				}

				if ( nVerts == 0 )
					break;
			}

			Shard_t &shard = m_Shards[nIndex];
			shard.nVerts = nVerts;
			shard.nNeighbors = 0;
			shard.nBorderAnchorBits = 0;
			shard.mins.Init( FLT_MAX, FLT_MAX );
			shard.maxs.Init( -FLT_MAX, -FLT_MAX );

			for ( int v = 0; v < nVerts; v++ )
			{
				shard.verts[v] = pCur[v];
				shard.edgeTag[v] = pCurTags[v];

				shard.mins.x = MIN( shard.mins.x, pCur[v].x );
				shard.mins.y = MIN( shard.mins.y, pCur[v].y );
				shard.maxs.x = MAX( shard.maxs.x, pCur[v].x );
				shard.maxs.y = MAX( shard.maxs.y, pCur[v].y );

				int nTag = pCurTags[v];
				if ( nTag < 0 )
				{
					shard.nBorderAnchorBits |= ShatterGlass_EdgeTagToAnchorBit( nTag );
				}
				else
				{
					bool bFound = false;
					for ( int n = 0; n < shard.nNeighbors; n++ )
					{
						if ( shard.neighbors[n] == nTag )
						{
							bFound = true;
							break;
						}
					}

					if ( !bFound && shard.nNeighbors < SHATTERGLASS_MAX_NEIGHBORS )
						shard.neighbors[shard.nNeighbors++] = (short)nTag;
				}
			}

			if ( nVerts == 0 )
			{
				shard.mins = shard.maxs = site;
				shard.area = 0.0f;
				shard.centroid = site;
			}
			else
			{
				ComputeAreaAndCentroid( shard.verts, nVerts, shard.area, shard.centroid );
			}

			m_flTotalArea += shard.area;
		}
	}

	// Make adjacency symmetric in case floating point error dropped a sliver edge on one side
	for ( int i = 0; i < m_Shards.Count(); i++ )
	{
		Shard_t &shard = m_Shards[i];
		for ( int n = 0; n < shard.nNeighbors; n++ )
		{
			Shard_t &other = m_Shards[shard.neighbors[n]];

			bool bFound = false;
			for ( int m = 0; m < other.nNeighbors; m++ )
			{
				if ( other.neighbors[m] == i )
				{
					bFound = true;
					break;
				}
			}

			if ( !bFound && other.nNeighbors < SHATTERGLASS_MAX_NEIGHBORS )
				other.neighbors[other.nNeighbors++] = (short)i;
		}
	}
}

static bool PointInConvexPolygon( const Vector2D *pVerts, int nVerts, const Vector2D &pt )
{
	for ( int i = 0; i < nVerts; i++ )
	{
		const Vector2D &a = pVerts[i];
		const Vector2D &b = pVerts[(i + 1) % nVerts];
		float flCross = ( b.x - a.x ) * ( pt.y - a.y ) - ( b.y - a.y ) * ( pt.x - a.x );
		if ( flCross < -1e-4f )
			return false;
	}
	return nVerts >= 3;
}

int CShatterGlassPattern::FindShardAt( const Vector2D &pt ) const
{
	if ( m_Shards.Count() == 0 )
		return -1;

	int ci = clamp( (int)( pt.x / m_flGridCellW ), 0, m_nGridX - 1 );
	int cj = clamp( (int)( pt.y / m_flGridCellH ), 0, m_nGridY - 1 );

	// A site never leaves its own grid cell, so the containing shard is almost always nearby
	for ( int nj = MAX( 0, cj - 1 ); nj <= MIN( m_nGridY - 1, cj + 1 ); nj++ )
	{
		for ( int ni = MAX( 0, ci - 1 ); ni <= MIN( m_nGridX - 1, ci + 1 ); ni++ )
		{
			int nIndex = nj * m_nGridX + ni;
			const Shard_t &shard = m_Shards[nIndex];
			if ( PointInConvexPolygon( shard.verts, shard.nVerts, pt ) )
				return nIndex;
		}
	}

	// Fall back to the nearest centroid
	int nBest = -1;
	float flBestDist = FLT_MAX;
	for ( int i = 0; i < m_Shards.Count(); i++ )
	{
		float flDist = m_Shards[i].centroid.DistToSqr( pt );
		if ( flDist < flBestDist )
		{
			flBestDist = flDist;
			nBest = i;
		}
	}
	return nBest;
}

bool CShatterGlassPattern::ShardOverlapsRect( const Shard_t &shard, const Vector2D &vecCenter, const Vector2D &vecHalfExtents )
{
	if ( shard.nVerts < 3 )
		return false;

	const float flEpsilon = 0.01f;

	// Rectangle axes
	if ( vecCenter.x - vecHalfExtents.x > shard.maxs.x + flEpsilon || vecCenter.x + vecHalfExtents.x < shard.mins.x - flEpsilon )
		return false;
	if ( vecCenter.y - vecHalfExtents.y > shard.maxs.y + flEpsilon || vecCenter.y + vecHalfExtents.y < shard.mins.y - flEpsilon )
		return false;

	// Polygon edge axes
	for ( int i = 0; i < shard.nVerts; i++ )
	{
		const Vector2D &a = shard.verts[i];
		const Vector2D &b = shard.verts[(i + 1) % shard.nVerts];

		// Outward normal of a counter-clockwise edge
		Vector2D normal( b.y - a.y, a.x - b.x );

		float flPolyMax = DotProduct2D( a, normal );
		float flRectMin = DotProduct2D( vecCenter, normal ) - ( vecHalfExtents.x * fabs( normal.x ) + vecHalfExtents.y * fabs( normal.y ) );

		if ( flRectMin > flPolyMax + flEpsilon * normal.Length() )
			return false;
	}

	return true;
}

int CShatterGlassPattern::GatherShardsInRect( const Vector2D &vecMins, const Vector2D &vecMaxs, int *pOut, int nMaxOut ) const
{
	if ( m_Shards.Count() == 0 )
		return 0;

	// Voronoi cells of a jittered grid never reach further than two grid cells from their own
	int i0 = clamp( (int)floor( vecMins.x / m_flGridCellW ) - 2, 0, m_nGridX - 1 );
	int i1 = clamp( (int)floor( vecMaxs.x / m_flGridCellW ) + 2, 0, m_nGridX - 1 );
	int j0 = clamp( (int)floor( vecMins.y / m_flGridCellH ) - 2, 0, m_nGridY - 1 );
	int j1 = clamp( (int)floor( vecMaxs.y / m_flGridCellH ) + 2, 0, m_nGridY - 1 );

	int nCount = 0;
	for ( int j = j0; j <= j1; j++ )
	{
		for ( int i = i0; i <= i1; i++ )
		{
			int nIndex = j * m_nGridX + i;
			const Shard_t &shard = m_Shards[nIndex];
			if ( shard.nVerts < 3 )
				continue;

			if ( shard.mins.x > vecMaxs.x || shard.maxs.x < vecMins.x || shard.mins.y > vecMaxs.y || shard.maxs.y < vecMins.y )
				continue;

			if ( nCount >= nMaxOut )
				return nCount;

			pOut[nCount++] = nIndex;
		}
	}

	return nCount;
}

//-----------------------------------------------------------------------------
// Collision
//-----------------------------------------------------------------------------
static bool ShatterGlass_RectHitsIntact( const ShatterGlassPanel_t &panel, const CShatterGlassPattern &pattern, const int *pBrokenBits, int nBrokenCount,
	const Vector2D &vecCenter, const Vector2D &vecHalfExtents, int &nHitShard )
{
	nHitShard = -1;

	// Entirely off the pane?
	if ( vecCenter.x + vecHalfExtents.x < 0.0f || vecCenter.x - vecHalfExtents.x > panel.m_flWidth ||
		vecCenter.y + vecHalfExtents.y < 0.0f || vecCenter.y - vecHalfExtents.y > panel.m_flHeight )
		return false;

	if ( nBrokenCount <= 0 )
	{
		// Fully intact pane; it's just a box
		Vector2D vecClamped( clamp( vecCenter.x, 0.0f, panel.m_flWidth ), clamp( vecCenter.y, 0.0f, panel.m_flHeight ) );
		nHitShard = pattern.FindShardAt( vecClamped );
		return true;
	}

	int nCandidates[SHATTERGLASS_MAX_SHARDS];
	int nCount = pattern.GatherShardsInRect( vecCenter - vecHalfExtents, vecCenter + vecHalfExtents, nCandidates, SHATTERGLASS_MAX_SHARDS );
	for ( int i = 0; i < nCount; i++ )
	{
		int nShard = nCandidates[i];
		if ( ShatterGlass_IsBitSet( pBrokenBits, nShard ) )
			continue;

		if ( CShatterGlassPattern::ShardOverlapsRect( pattern.Get( nShard ), vecCenter, vecHalfExtents ) )
		{
			nHitShard = nShard;
			return true;
		}
	}

	return false;
}

//-----------------------------------------------------------------------------
// Purpose: When a rectangle is just short of touching a shard, returns the
//			separating axis with the largest gap, pointing from the shard
//			towards the rectangle. That's the contact normal.
//-----------------------------------------------------------------------------
static Vector2D ShatterGlass_ContactNormal( const CShatterGlassPattern::Shard_t &shard, const Vector2D &vecCenter, const Vector2D &vecHalfExtents )
{
	Vector2D vecBest( 0, 0 );
	float flBestGap = -FLT_MAX;

	// Rectangle axes
	float flGaps[4] =
	{
		( vecCenter.x - vecHalfExtents.x ) - shard.maxs.x,	// +X
		shard.mins.x - ( vecCenter.x + vecHalfExtents.x ),	// -X
		( vecCenter.y - vecHalfExtents.y ) - shard.maxs.y,	// +Y
		shard.mins.y - ( vecCenter.y + vecHalfExtents.y ),	// -Y
	};
	const Vector2D vecAxes[4] = { Vector2D( 1, 0 ), Vector2D( -1, 0 ), Vector2D( 0, 1 ), Vector2D( 0, -1 ) };
	for ( int i = 0; i < 4; i++ )
	{
		if ( flGaps[i] > flBestGap )
		{
			flBestGap = flGaps[i];
			vecBest = vecAxes[i];
		}
	}

	// Shard edge normals
	for ( int i = 0; i < shard.nVerts; i++ )
	{
		const Vector2D &a = shard.verts[i];
		const Vector2D &b = shard.verts[(i + 1) % shard.nVerts];

		Vector2D normal( b.y - a.y, a.x - b.x );
		if ( Vector2DNormalize( normal ) < 1e-6f )
			continue;

		float flGap = DotProduct2D( vecCenter, normal ) - ( vecHalfExtents.x * fabs( normal.x ) + vecHalfExtents.y * fabs( normal.y ) ) - DotProduct2D( a, normal );
		if ( flGap > flBestGap )
		{
			flBestGap = flGap;
			vecBest = normal;
		}
	}

	return vecBest;
}

bool ShatterGlass_TestCollision( const Ray_t &ray, unsigned int fContentsMask, trace_t &tr,
	const matrix3x4_t &entityToWorld, const ShatterGlassPanel_t &panel, const CShatterGlassPattern &pattern,
	const int *pBrokenBits, int nBrokenCount, int nContents, unsigned short nSurfaceProps )
{
	if ( !( fContentsMask & nContents ) )
		return false;

	if ( !pattern.IsValid() || nBrokenCount >= pattern.Count() )
		return false;

	const int iU = panel.m_iU, iV = panel.m_iV, iN = panel.m_iN;

	// Move the ray into entity space
	Vector vecStart, vecDelta, vecExtents;
	VectorITransform( ray.m_Start, entityToWorld, vecStart );
	VectorIRotate( ray.m_Delta, entityToWorld, vecDelta );
	for ( int k = 0; k < 3; k++ )
	{
		vecExtents[k] = fabs( entityToWorld[0][k] ) * ray.m_Extents.x +
						fabs( entityToWorld[1][k] ) * ray.m_Extents.y +
						fabs( entityToWorld[2][k] ) * ray.m_Extents.z;
	}

	// Find where the swept volume is inside the pane's slab
	float flStartN = vecStart[iN] - panel.m_vecMins[iN];
	float flDeltaN = vecDelta[iN];
	float flLo = -vecExtents[iN];
	float flHi = panel.m_flThickness + vecExtents[iN];

	bool bStartInside = ( flStartN >= flLo && flStartN <= flHi );

	float flEnter, flExit;
	if ( fabs( flDeltaN ) < 1e-6f )
	{
		if ( !bStartInside )
			return false;
		flEnter = 0.0f;
		flExit = 1.0f;
	}
	else
	{
		float t0 = ( flLo - flStartN ) / flDeltaN;
		float t1 = ( flHi - flStartN ) / flDeltaN;
		if ( t0 > t1 )
			V_swap( t0, t1 );

		flEnter = MAX( t0, 0.0f );
		flExit = MIN( t1, 1.0f );
		if ( flEnter > flExit )
			return false;
	}

	Vector2D vecStartUV( vecStart[iU] - panel.m_vecMins[iU], vecStart[iV] - panel.m_vecMins[iV] );
	Vector2D vecDeltaUV( vecDelta[iU], vecDelta[iV] );
	Vector2D vecHalfExtents( vecExtents[iU], vecExtents[iV] );

	// Sample the in-slab part of the sweep finely enough to not skip over shard edges
	float flTravel = vecDeltaUV.Length() * ( flExit - flEnter );
	int nSamples = 1;
	if ( flTravel > 0.0f )
		nSamples = clamp( (int)ceil( flTravel / ( pattern.CellSize() * 0.25f ) ) + 1, 2, 24 );

	float flHitFraction = -1.0f;
	Vector vecNormalLocal( 0, 0, 0 );
	bool bStartSolid = false;
	bool bAllSolid = false;

	float flPrevT = flEnter;
	for ( int k = 0; k < nSamples; k++ )
	{
		float t = ( nSamples > 1 ) ? flEnter + ( flExit - flEnter ) * ( (float)k / (float)( nSamples - 1 ) ) : flEnter;
		Vector2D vecCenter = vecStartUV + vecDeltaUV * t;

		int nShard;
		if ( !ShatterGlass_RectHitsIntact( panel, pattern, pBrokenBits, nBrokenCount, vecCenter, vecHalfExtents, nShard ) )
		{
			flPrevT = t;
			continue;
		}

		if ( k == 0 )
		{
			if ( bStartInside )
			{
				// Already overlapping glass
				bStartSolid = true;
				flHitFraction = 0.0f;

				int nEndShard;
				Vector2D vecEnd = vecStartUV + vecDeltaUV * flExit;
				bAllSolid = ( flExit >= 1.0f ) && ShatterGlass_RectHitsIntact( panel, pattern, pBrokenBits, nBrokenCount, vecEnd, vecHalfExtents, nEndShard );
			}
			else
			{
				// Hit the face of the pane
				flHitFraction = flEnter;
			}

			vecNormalLocal[iN] = ( flDeltaN > 0.0f ) ? -1.0f : 1.0f;
		}
		else
		{
			// Hit the edge of a shard while moving through a hole
			flHitFraction = flPrevT;

			Vector2D vecAway = -vecDeltaUV;
			if ( nShard >= 0 )
				vecAway = ShatterGlass_ContactNormal( pattern.Get( nShard ), vecStartUV + vecDeltaUV * flPrevT, vecHalfExtents );
			if ( vecAway.LengthSqr() < 1e-6f )
				vecAway = -vecDeltaUV;
			Vector2DNormalize( vecAway );

			vecNormalLocal[iU] = vecAway.x;
			vecNormalLocal[iV] = vecAway.y;
		}

		break;
	}

	if ( flHitFraction < 0.0f )
		return false;

	// Back off slightly so movement code doesn't end up inside the glass
	float flDeltaLength = ray.m_Delta.Length();
	if ( !bStartSolid && flDeltaLength > 0.0f )
		flHitFraction = MAX( 0.0f, flHitFraction - ( 0.03125f / flDeltaLength ) );

	Vector vecNormal;
	VectorRotate( vecNormalLocal, entityToWorld, vecNormal );
	VectorNormalize( vecNormal );

	tr.fraction = flHitFraction;
	tr.fractionleftsolid = 0.0f;
	tr.startsolid = bStartSolid;
	tr.allsolid = bAllSolid;
	tr.startpos = ray.m_Start + ray.m_StartOffset;
	tr.endpos = tr.startpos + ray.m_Delta * flHitFraction;
	tr.plane.normal = vecNormal;
	tr.plane.dist = DotProduct( tr.endpos, vecNormal );
	tr.plane.type = 3; // PLANE_ANYZ
	tr.plane.signbits = ( vecNormal.x < 0 ? 1 : 0 ) | ( vecNormal.y < 0 ? 2 : 0 ) | ( vecNormal.z < 0 ? 4 : 0 );
	tr.contents = nContents;
	tr.surface.name = "**shatterglass**";
	tr.surface.surfaceProps = nSurfaceProps;
	tr.surface.flags = 0;
	tr.hitgroup = 0;
	tr.hitbox = 0;
	tr.physicsbone = 0;

	return true;
}
