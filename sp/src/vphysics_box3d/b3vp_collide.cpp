//=================================================================================================
//
// VPhysics Box3D - collision models and IPhysicsCollision
//
//=================================================================================================

#include "b3vp_base.h"
#include "b3vp_collide.h"
#include "b3vp_parse.h"

#include "cmodel.h"
#include "gametrace.h"
#include "bspflags.h"
#include "coordsize.h"
#include "mathlib/polyhedron.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

CBox3DPhysicsCollision CBox3DPhysicsCollision::s_Instance;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR( CBox3DPhysicsCollision, IPhysicsCollision, VPHYSICS_COLLISION_INTERFACE_VERSION, CBox3DPhysicsCollision::GetInstance() );

//-------------------------------------------------------------------------------------------------
// Small math helpers
//-------------------------------------------------------------------------------------------------

b3Quat B3QuatFromZAxis( const b3Vec3 &axis )
{
	return b3ComputeQuatBetweenUnitVectors( b3Vec3_axisZ, b3Normalize( axis ) );
}

b3Quat B3QuatFromAxes( const b3Vec3 &xAxisIn, const b3Vec3 &yHint )
{
	b3Vec3 xAxis = b3Normalize( xAxisIn );
	b3Vec3 zAxis = b3Cross( xAxis, yHint );
	if ( b3Dot( zAxis, zAxis ) < 1e-8f )
	{
		// Pick any perpendicular
		b3Vec3 other = fabsf( xAxis.x ) < 0.9f ? b3Vec3_axisX : b3Vec3_axisY;
		zAxis = b3Cross( xAxis, other );
	}
	zAxis = b3Normalize( zAxis );
	b3Vec3 yAxis = b3Cross( zAxis, xAxis );

	b3Matrix3 m;
	m.cx = xAxis;
	m.cy = yAxis;
	m.cz = zAxis;
	return b3NormalizeQuat( b3MakeQuatFromMatrix( &m ) );
}

static inline b3AABB B3VP_EmptyAABB()
{
	b3AABB box;
	box.lowerBound = b3Vec3{ FLT_MAX, FLT_MAX, FLT_MAX };
	box.upperBound = b3Vec3{ -FLT_MAX, -FLT_MAX, -FLT_MAX };
	return box;
}

static inline void B3VP_AddPoint( b3AABB &box, const b3Vec3 &p )
{
	box.lowerBound = b3Min( box.lowerBound, p );
	box.upperBound = b3Max( box.upperBound, p );
}

static inline bool B3VP_Overlaps( const b3AABB &a, const b3AABB &b )
{
	return !( a.upperBound.x < b.lowerBound.x || a.upperBound.y < b.lowerBound.y || a.upperBound.z < b.lowerBound.z ||
			  b.upperBound.x < a.lowerBound.x || b.upperBound.y < a.lowerBound.y || b.upperBound.z < a.lowerBound.z );
}

// Source <-> Box3D in collide space (inches) only needs a type change
static inline b3Vec3 V2B( const Vector &v ) { b3Vec3 r = { v.x, v.y, v.z }; return r; }
static inline Vector B2V( const b3Vec3 &v ) { return Vector( v.x, v.y, v.z ); }

// A transform in inches (collide space -> world space in Source units)
static inline b3Transform MakeHLTransform( const Vector &origin, const QAngle &angles )
{
	b3Transform xf;
	xf.p = V2B( origin );
	xf.q = AnglesToB3Quat( angles );
	return xf;
}

//-------------------------------------------------------------------------------------------------
// CPhysConvex
//-------------------------------------------------------------------------------------------------

CPhysConvex::~CPhysConvex()
{
	if ( m_pHull )
		b3DestroyHull( m_pHull );
}

CPhysConvex *CPhysConvex::CreateFromPoints( const b3Vec3 *pPoints, int nPoints )
{
	if ( nPoints < 4 )
		return NULL;

	b3HullData *pHull = b3CreateHull( pPoints, nPoints, B3_MAX_HULL_VERTICES );
	if ( !pHull )
	{
		// Degenerate input (flat or tiny). Inflate a bounding box so the game still has something solid.
		b3AABB box = B3VP_EmptyAABB();
		for ( int i = 0; i < nPoints; i++ )
			B3VP_AddPoint( box, pPoints[i] );

		const float minHalf = 0.25f;
		b3Vec3 center = b3AABB_Center( box );
		b3Vec3 half = b3AABB_Extents( box );
		half.x = MAX( half.x, minHalf );
		half.y = MAX( half.y, minHalf );
		half.z = MAX( half.z, minHalf );
		b3BoxHull boxHull = b3MakeOffsetBoxHull( half.x, half.y, half.z, center );
		pHull = b3CloneHull( &boxHull.base );
		if ( !pHull )
			return NULL;
	}

	CPhysConvex *pConvex = new CPhysConvex;
	pConvex->m_pHull = pHull;
	return pConvex;
}

CPhysConvex *CPhysConvex::CreateBox( const b3Vec3 &mins, const b3Vec3 &maxs )
{
	b3Vec3 center = b3MulSV( 0.5f, b3Add( mins, maxs ) );
	b3Vec3 half = b3MulSV( 0.5f, b3Sub( maxs, mins ) );
	half.x = MAX( half.x, 0.01f );
	half.y = MAX( half.y, 0.01f );
	half.z = MAX( half.z, 0.01f );

	b3BoxHull boxHull = b3MakeOffsetBoxHull( half.x, half.y, half.z, center );
	CPhysConvex *pConvex = new CPhysConvex;
	pConvex->m_pHull = b3CloneHull( &boxHull.base );
	return pConvex;
}

CPhysConvex *CPhysConvex::Clone() const
{
	CPhysConvex *pConvex = new CPhysConvex;
	pConvex->m_pHull = m_pHull ? b3CloneHull( m_pHull ) : NULL;
	pConvex->m_gameData = m_gameData;
	pConvex->m_materialIndex = m_materialIndex;
	return pConvex;
}

//-------------------------------------------------------------------------------------------------
// CPhysCollide
//-------------------------------------------------------------------------------------------------

CPhysCollide::CPhysCollide()
	: m_Type( COLLIDE_CONVEXES )
	, m_pMesh( NULL )
	, m_nMeshSurfaceProp( -1 )
	, m_flVolume( 0.0f )
	, m_flSurfaceArea( 0.0f )
	, m_bMassCenterSet( false )
	, m_OrthoAreas( 1.0f, 1.0f, 1.0f )
	, m_nIndex( 0 )
	, m_nRefCount( 0 )
	, m_bPendingDestroy( false )
	, m_bCached( false )
{
	m_Bounds.lowerBound = b3Vec3_zero;
	m_Bounds.upperBound = b3Vec3_zero;
	m_MassCenter = b3Vec3_zero;
}

CPhysCollide::~CPhysCollide()
{
	m_Convexes.PurgeAndDeleteElements();
	if ( m_pMesh )
		b3DestroyMesh( m_pMesh );
}

void CPhysCollide::Release()
{
	Assert( m_nRefCount > 0 );
	--m_nRefCount;
	if ( m_nRefCount <= 0 && m_bPendingDestroy && !m_bCached )
		delete this;
}

void CPhysCollide::ComputeDerivedData()
{
	b3AABB bounds = B3VP_EmptyAABB();
	float volume = 0.0f;
	float area = 0.0f;
	b3Vec3 weightedCenter = b3Vec3_zero;

	if ( m_Type == COLLIDE_MESH && m_pMesh )
	{
		bounds = m_pMesh->bounds;
		area = m_pMesh->surfaceArea;
	}
	else
	{
		for ( int i = 0; i < m_Convexes.Count(); i++ )
		{
			const b3HullData *pHull = m_Convexes[i]->m_pHull;
			if ( !pHull )
				continue;

			B3VP_AddPoint( bounds, pHull->aabb.lowerBound );
			B3VP_AddPoint( bounds, pHull->aabb.upperBound );
			volume += pHull->volume;
			area += pHull->surfaceArea;
			weightedCenter = b3MulAdd( weightedCenter, pHull->volume, pHull->center );
		}
	}

	if ( bounds.lowerBound.x > bounds.upperBound.x )
	{
		bounds.lowerBound = b3Vec3_zero;
		bounds.upperBound = b3Vec3_zero;
	}

	m_Bounds = bounds;
	m_flVolume = volume;
	m_flSurfaceArea = area;

	if ( !m_bMassCenterSet )
	{
		if ( volume > 0.0f )
			m_MassCenter = b3MulSV( 1.0f / volume, weightedCenter );
		else
			m_MassCenter = b3AABB_Center( bounds );
	}

	// Default drag areas from the bounds if the file didn't provide them
	if ( m_OrthoAreas == Vector( 1.0f, 1.0f, 1.0f ) )
	{
		b3Vec3 size = b3Sub( bounds.upperBound, bounds.lowerBound );
		m_OrthoAreas.Init( size.y * size.z, size.x * size.z, size.x * size.y );
	}
}

//-------------------------------------------------------------------------------------------------
// Hull topology helpers
//-------------------------------------------------------------------------------------------------

// Calls fn( a, b, c ) for every triangle of a fan triangulation of the hull faces (CCW from outside)
template < typename Fn >
static void ForEachHullTriangle( const b3HullData *pHull, Fn &fn )
{
	const b3Vec3 *pPoints = b3GetHullPoints( pHull );
	const b3HullHalfEdge *pEdges = b3GetHullEdges( pHull );
	const b3HullFace *pFaces = b3GetHullFaces( pHull );

	for ( int f = 0; f < pHull->faceCount; f++ )
	{
		const int e0 = pFaces[f].edge;
		const int v0 = pEdges[e0].origin;
		int e1 = pEdges[e0].next;
		int e2 = pEdges[e1].next;
		int guard = 0;
		while ( e2 != e0 && guard++ < 256 )
		{
			fn( pPoints[v0], pPoints[pEdges[e1].origin], pPoints[pEdges[e2].origin] );
			e1 = e2;
			e2 = pEdges[e2].next;
		}
	}
}

static int CountHullTriangles( const b3HullData *pHull )
{
	struct Counter { int n; void operator()( const b3Vec3 &, const b3Vec3 &, const b3Vec3 & ) { n++; } } counter = { 0 };
	ForEachHullTriangle( pHull, counter );
	return counter.n;
}

static bool PointInsideHull( const b3HullData *pHull, const b3Vec3 &p, float tolerance )
{
	const b3Plane *pPlanes = b3GetHullPlanes( pHull );
	for ( int i = 0; i < pHull->faceCount; i++ )
	{
		if ( b3Dot( pPlanes[i].normal, p ) - pPlanes[i].offset > tolerance )
			return false;
	}
	return true;
}

//-------------------------------------------------------------------------------------------------
// Traces
//-------------------------------------------------------------------------------------------------

void B3VP_ClearTrace( trace_t *pTrace )
{
	memset( pTrace, 0, sizeof( *pTrace ) );
	pTrace->fraction = 1.0f;
	pTrace->fractionleftsolid = 0.0f;
	pTrace->surface.name = "**empty**";
}

int B3VP_MakeBoxProxy( const b3Vec3 &centerLocal, const b3Vec3 &halfExtentWorld, const b3Quat &collideRotation, b3Vec3 *pOutPoints )
{
	int n = 0;
	for ( int i = 0; i < 8; i++ )
	{
		b3Vec3 corner = {
			( i & 1 ) ? halfExtentWorld.x : -halfExtentWorld.x,
			( i & 2 ) ? halfExtentWorld.y : -halfExtentWorld.y,
			( i & 4 ) ? halfExtentWorld.z : -halfExtentWorld.z };
		pOutPoints[n++] = b3Add( centerLocal, b3InvRotateVector( collideRotation, corner ) );
	}
	return n;
}

static void SetTracePlane( trace_t *pTrace, const Vector &normal, float dist )
{
	pTrace->plane.normal = normal;
	pTrace->plane.dist = dist;
	if ( fabsf( normal.x ) > 0.9999f )		pTrace->plane.type = PLANE_X;
	else if ( fabsf( normal.y ) > 0.9999f )	pTrace->plane.type = PLANE_Y;
	else if ( fabsf( normal.z ) > 0.9999f )	pTrace->plane.type = PLANE_Z;
	else									pTrace->plane.type = PLANE_ANYZ;
	pTrace->plane.signbits = SignbitsForPlane( &pTrace->plane );
}

// Converts a raw hit fraction to Source's, which backs off DIST_EPSILON along the surface normal
static float SourceFraction( const Vector &delta, float fraction, const Vector &normal )
{
	const float length = delta.Length();
	if ( length <= 0.0f )
		return 0.0f;

	const Vector dir = delta / length;
	float hitLength = length * fraction;
	const float dot = DotProduct( dir, normal );
	if ( dot < 0.0f )
		hitLength += DIST_EPSILON / dot;

	return clamp( hitLength / length, 0.0f, 1.0f );
}

struct TraceHit_t
{
	bool			bHit;
	float			fraction;
	b3Vec3			normal;		// collide space
	b3Vec3			point;		// collide space
	unsigned int	contents;
};

static void TraceAgainstConvexes( const CPhysCollide *pCollide, unsigned int contentsMask, IConvexInfo *pConvexInfo,
								  bool bIsRay, bool bSwept, const b3Vec3 &startLocal, const b3Vec3 &deltaLocal,
								  const b3ShapeProxy &proxy, const b3AABB &sweptBounds, TraceHit_t &hit, bool &bStartSolid, unsigned int &startContents )
{
	for ( int i = 0; i < pCollide->m_Convexes.Count(); i++ )
	{
		const CPhysConvex *pConvex = pCollide->m_Convexes[i];
		const b3HullData *pHull = pConvex->m_pHull;
		if ( !pHull )
			continue;

		if ( !B3VP_Overlaps( sweptBounds, pHull->aabb ) )
			continue;

		const unsigned int contents = pConvexInfo ? pConvexInfo->GetContents( pConvex->m_gameData ) : CONTENTS_SOLID;
		if ( !( contents & contentsMask ) )
			continue;

		// Starting inside a solid clips the trace completely (matches IVP)
		bool bInside;
		if ( bIsRay )
			bInside = PointInsideHull( pHull, startLocal, 0.0f );
		else
			bInside = b3OverlapHull( pHull, b3Transform_identity, &proxy );

		if ( bInside )
		{
			bStartSolid = true;
			startContents = contents;
			return;
		}

		if ( !bSwept )
			continue;

		b3CastOutput output;
		if ( bIsRay )
		{
			b3RayCastInput input;
			input.origin = startLocal;
			input.translation = deltaLocal;
			input.maxFraction = hit.fraction;
			output = b3RayCastHull( pHull, &input );
		}
		else
		{
			b3ShapeCastInput input;
			input.proxy = proxy;
			input.translation = deltaLocal;
			input.maxFraction = hit.fraction;
			// Allows boxes that rest right against a face to slide along it instead of reporting
			// an initial overlap. Safe for convex hulls (a tangential move can't hit another face).
			input.canEncroach = true;
			output = b3ShapeCastHull( pHull, &input );
		}

		if ( output.hit && ( !hit.bHit || output.fraction < hit.fraction ) )
		{
			hit.bHit = true;
			hit.fraction = output.fraction;
			hit.normal = output.normal;
			hit.point = output.point;
			hit.contents = contents;
		}
	}
}

static void TraceAgainstMesh( const CPhysCollide *pCollide, unsigned int contentsMask, bool bIsRay, bool bSwept,
							  const b3Vec3 &startLocal, const b3Vec3 &deltaLocal, const b3ShapeProxy &proxy,
							  const b3AABB &sweptBounds, TraceHit_t &hit )
{
	if ( !pCollide->m_pMesh || !bSwept || !( contentsMask & CONTENTS_SOLID ) )
		return;

	if ( !B3VP_Overlaps( sweptBounds, pCollide->m_pMesh->bounds ) )
		return;

	b3Mesh mesh;
	mesh.data = pCollide->m_pMesh;
	mesh.scale = b3Vec3_one;

	b3CastOutput output;
	if ( bIsRay )
	{
		b3RayCastInput input;
		input.origin = startLocal;
		input.translation = deltaLocal;
		input.maxFraction = hit.fraction;
		output = b3RayCastMesh( &mesh, &input );
	}
	else
	{
		b3ShapeCastInput input;
		input.proxy = proxy;
		input.translation = deltaLocal;
		input.maxFraction = hit.fraction;
		input.canEncroach = true;
		output = b3ShapeCastMesh( &mesh, &input );
	}

	if ( output.hit && ( !hit.bHit || output.fraction < hit.fraction ) )
	{
		hit.bHit = true;
		hit.fraction = output.fraction;
		hit.normal = output.normal;
		hit.point = output.point;
		hit.contents = CONTENTS_SOLID;
	}
}

void B3VP_TraceCollide( const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo,
						const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *pTrace )
{
	B3VP_ClearTrace( pTrace );

	const Vector rayStart( ray.m_Start.x, ray.m_Start.y, ray.m_Start.z );
	const Vector rayDelta( ray.m_Delta.x, ray.m_Delta.y, ray.m_Delta.z );
	const Vector rayExtents( ray.m_Extents.x, ray.m_Extents.y, ray.m_Extents.z );
	const Vector rayStartOffset( ray.m_StartOffset.x, ray.m_StartOffset.y, ray.m_StartOffset.z );

	pTrace->startpos = rayStart + rayStartOffset;
	pTrace->endpos = pTrace->startpos + rayDelta;

	if ( !pCollide )
		return;

	const b3Transform xf = MakeHLTransform( collideOrigin, collideAngles );
	const b3Vec3 startLocal = b3InvTransformPoint( xf, V2B( rayStart ) );
	const b3Vec3 deltaLocal = b3InvRotateVector( xf.q, V2B( rayDelta ) );
	const bool bIsRay = ray.m_IsRay || rayExtents.LengthSqr() < 1e-6f;
	const bool bSwept = ray.m_IsSwept && rayDelta.LengthSqr() > 0.0f;

	b3Vec3 boxPoints[8];
	b3ShapeProxy proxy;
	proxy.points = boxPoints;
	proxy.count = 0;
	proxy.radius = 0.0f;

	b3AABB sweptBounds = B3VP_EmptyAABB();
	if ( bIsRay )
	{
		B3VP_AddPoint( sweptBounds, startLocal );
		B3VP_AddPoint( sweptBounds, b3Add( startLocal, deltaLocal ) );
	}
	else
	{
		proxy.count = B3VP_MakeBoxProxy( startLocal, V2B( rayExtents ), xf.q, boxPoints );
		for ( int i = 0; i < proxy.count; i++ )
		{
			B3VP_AddPoint( sweptBounds, boxPoints[i] );
			B3VP_AddPoint( sweptBounds, b3Add( boxPoints[i], deltaLocal ) );
		}
	}

	// Small slack so touching geometry is considered
	const b3Vec3 slack = { 0.1f, 0.1f, 0.1f };
	sweptBounds.lowerBound = b3Sub( sweptBounds.lowerBound, slack );
	sweptBounds.upperBound = b3Add( sweptBounds.upperBound, slack );

	TraceHit_t hit;
	hit.bHit = false;
	hit.fraction = 1.0f;
	hit.normal = b3Vec3_zero;
	hit.point = b3Vec3_zero;
	hit.contents = 0;

	bool bStartSolid = false;
	unsigned int startContents = 0;

	if ( pCollide->IsMesh() )
		TraceAgainstMesh( pCollide, contentsMask, bIsRay, bSwept, startLocal, deltaLocal, proxy, sweptBounds, hit );
	else
		TraceAgainstConvexes( pCollide, contentsMask, pConvexInfo, bIsRay, bSwept, startLocal, deltaLocal, proxy, sweptBounds, hit, bStartSolid, startContents );

	if ( bStartSolid )
	{
		pTrace->startsolid = true;
		pTrace->allsolid = true;
		pTrace->fraction = 0.0f;
		pTrace->fractionleftsolid = 0.0f;
		pTrace->endpos = pTrace->startpos;
		pTrace->contents = startContents;
		return;
	}

	if ( !hit.bHit )
		return;

	const Vector normal = B2V( b3Normalize( b3RotateVector( xf.q, hit.normal ) ) );
	const Vector point = B2V( b3TransformPoint( xf, hit.point ) );

	pTrace->fraction = SourceFraction( rayDelta, hit.fraction, normal );
	pTrace->endpos = pTrace->startpos + rayDelta * pTrace->fraction;
	pTrace->contents = hit.contents;
	SetTracePlane( pTrace, normal, DotProduct( point, normal ) );
}

//-------------------------------------------------------------------------------------------------
// IPhysicsCollision: convexes
//-------------------------------------------------------------------------------------------------

CBox3DPhysicsCollision::CBox3DPhysicsCollision()
{
}

CBox3DPhysicsCollision::~CBox3DPhysicsCollision()
{
	// Static destruction order is unknowable, the OS reclaims the memory anyway.
}

CPhysConvex *CBox3DPhysicsCollision::ConvexFromVerts( Vector **pVerts, int vertCount )
{
	std::vector< b3Vec3 > points( vertCount );
	for ( int i = 0; i < vertCount; i++ )
		points[i] = V2B( *pVerts[i] );

	return CPhysConvex::CreateFromPoints( points.data(), vertCount );
}

CPhysConvex *CBox3DPhysicsCollision::ConvexFromPlanes( float *pPlanes, int planeCount, float mergeDistance )
{
	CPolyhedron *pPolyhedron = GeneratePolyhedronFromPlanes( pPlanes, planeCount, MAX( mergeDistance, 0.001f ), true );
	if ( !pPolyhedron )
		return NULL;

	std::vector< b3Vec3 > points( pPolyhedron->iVertexCount );
	for ( int i = 0; i < pPolyhedron->iVertexCount; i++ )
		points[i] = V2B( pPolyhedron->pVertices[i] );

	pPolyhedron->Release();
	return CPhysConvex::CreateFromPoints( points.data(), (int)points.size() );
}

float CBox3DPhysicsCollision::ConvexVolume( CPhysConvex *pConvex )
{
	return ( pConvex && pConvex->m_pHull ) ? pConvex->m_pHull->volume : 0.0f;
}

float CBox3DPhysicsCollision::ConvexSurfaceArea( CPhysConvex *pConvex )
{
	return ( pConvex && pConvex->m_pHull ) ? pConvex->m_pHull->surfaceArea : 0.0f;
}

void CBox3DPhysicsCollision::SetConvexGameData( CPhysConvex *pConvex, unsigned int gameData )
{
	if ( pConvex )
		pConvex->m_gameData = gameData;
}

void CBox3DPhysicsCollision::ConvexFree( CPhysConvex *pConvex )
{
	delete pConvex;
}

CPhysConvex *CBox3DPhysicsCollision::BBoxToConvex( const Vector &mins, const Vector &maxs )
{
	return CPhysConvex::CreateBox( V2B( mins ), V2B( maxs ) );
}

CPhysConvex *CBox3DPhysicsCollision::ConvexFromConvexPolyhedron( const CPolyhedron &ConvexPolyhedron )
{
	std::vector< b3Vec3 > points( ConvexPolyhedron.iVertexCount );
	for ( int i = 0; i < ConvexPolyhedron.iVertexCount; i++ )
		points[i] = V2B( ConvexPolyhedron.pVertices[i] );

	return CPhysConvex::CreateFromPoints( points.data(), (int)points.size() );
}

void CBox3DPhysicsCollision::ConvexesFromConvexPolygon( const Vector &vPolyNormal, const Vector *pPoints, int iPointCount, CPhysConvex **pOutput )
{
	// Extrudes the polygon into a thin slab. Only used by tools in practice.
	std::vector< b3Vec3 > points;
	points.reserve( iPointCount * 2 );
	for ( int i = 0; i < iPointCount; i++ )
	{
		points.push_back( V2B( pPoints[i] ) );
		points.push_back( V2B( pPoints[i] - vPolyNormal * 1.0f ) );
	}

	if ( pOutput )
		*pOutput = CPhysConvex::CreateFromPoints( points.data(), (int)points.size() );
}

//-------------------------------------------------------------------------------------------------
// IPhysicsCollision: polysoups and meshes
//-------------------------------------------------------------------------------------------------

CPhysPolysoup *CBox3DPhysicsCollision::PolysoupCreate( void )
{
	return new CPhysPolysoup;
}

void CBox3DPhysicsCollision::PolysoupDestroy( CPhysPolysoup *pSoup )
{
	delete pSoup;
}

void CBox3DPhysicsCollision::PolysoupAddTriangle( CPhysPolysoup *pSoup, const Vector &a, const Vector &b, const Vector &c, int materialIndex7bits )
{
	pSoup->m_Verts.push_back( V2B( a ) );
	pSoup->m_Verts.push_back( V2B( b ) );
	pSoup->m_Verts.push_back( V2B( c ) );
	pSoup->m_Materials.push_back( (uint8)( materialIndex7bits & 0x7F ) );
}

CPhysCollide *CBox3DPhysicsCollision::ConvertPolysoupToCollide( CPhysPolysoup *pSoup, bool useMOPP )
{
	// MOPP is an IVP acceleration structure, Box3D always builds a BVH. NOTE: does not free the soup.
	const int triangleCount = (int)pSoup->m_Materials.size();
	if ( !triangleCount )
		return NULL;

	return CreateMeshCollide( pSoup->m_Verts.data(), pSoup->m_Materials.data(), triangleCount, true );
}

CPhysCollide *CBox3DPhysicsCollision::CreateMeshCollide( const b3Vec3 *pTriangleVerts, const uint8 *pMaterials, int triangleCount, bool bAutoOrient )
{
	if ( triangleCount <= 0 )
		return NULL;

	const int vertCount = triangleCount * 3;
	std::vector< b3Vec3 > verts( pTriangleVerts, pTriangleVerts + vertCount );
	std::vector< int32_t > indices( vertCount );
	std::vector< uint8_t > materials( triangleCount, 0 );
	for ( int i = 0; i < vertCount; i++ )
		indices[i] = i;
	if ( pMaterials )
	{
		for ( int i = 0; i < triangleCount; i++ )
			materials[i] = pMaterials[i];
	}

	// Box3D triangles are one-sided. Source doesn't promise a winding for polysoups or virtual
	// meshes, so work out which side faces "out": closed meshes by signed volume, open surfaces
	// (displacements, terrain) by facing upwards on average.
	bool bClockwise = true;
	if ( bAutoOrient )
	{
		double signedVolume = 0.0;
		b3Vec3 normalSum = b3Vec3_zero;
		b3AABB bounds = B3VP_EmptyAABB();
		for ( int i = 0; i < triangleCount; i++ )
		{
			const b3Vec3 &a = verts[i * 3 + 0];
			const b3Vec3 &b = verts[i * 3 + 1];
			const b3Vec3 &c = verts[i * 3 + 2];
			const b3Vec3 n = b3Cross( b3Sub( b, a ), b3Sub( c, a ) ); // CCW normal, area weighted
			normalSum = b3Add( normalSum, n );
			signedVolume += b3Dot( a, b3Cross( b, c ) ) / 6.0;
			B3VP_AddPoint( bounds, a );
			B3VP_AddPoint( bounds, b );
			B3VP_AddPoint( bounds, c );
		}

		const b3Vec3 size = b3Sub( bounds.upperBound, bounds.lowerBound );
		const double boundsVolume = (double)MAX( size.x, 0.01f ) * MAX( size.y, 0.01f ) * MAX( size.z, 0.01f );
		if ( fabs( signedVolume ) > 0.05 * boundsVolume )
			bClockwise = signedVolume < 0.0;
		else if ( fabsf( normalSum.z ) > 1e-3f )
			bClockwise = normalSum.z < 0.0f;
	}

	b3MeshDef def;
	memset( &def, 0, sizeof( def ) );
	def.vertices = verts.data();
	def.stride = 0;
	def.indices = indices.data();
	def.materialIndices = materials.data();
	def.weldTolerance = 0.01f;
	def.vertexCount = vertCount;
	def.triangleCount = triangleCount;
	def.weldVertices = true;
	def.useMedianSplit = false;
	def.identifyEdges = true;
	def.clockWiseWinding = bClockwise;

	b3MeshData *pMesh = b3CreateMesh( &def, NULL, 0 );
	if ( !pMesh )
	{
		B3VP_Warning( "Failed to build a mesh collide (%d triangles)\n", triangleCount );
		return NULL;
	}

	CPhysCollide *pCollide = new CPhysCollide;
	pCollide->m_Type = CPhysCollide::COLLIDE_MESH;
	pCollide->m_pMesh = pMesh;
	pCollide->ComputeDerivedData();
	return pCollide;
}

CPhysCollide *CBox3DPhysicsCollision::CreateVirtualMesh( const virtualmeshparams_t &params )
{
	IVirtualMeshEvent *pHandler = params.pMeshEventHandler;
	if ( !pHandler )
		return NULL;

	virtualmeshlist_t *pList = new virtualmeshlist_t;
	memset( pList, 0, sizeof( *pList ) );
	pHandler->GetVirtualMesh( params.userData, pList );

	std::vector< b3Vec3 > verts;
	verts.reserve( pList->triangleCount * 3 );
	for ( int i = 0; i < pList->triangleCount; i++ )
	{
		for ( int j = 0; j < 3; j++ )
		{
			const int index = pList->indices[i * 3 + j];
			verts.push_back( ( index >= 0 && index < pList->vertexCount ) ? V2B( pList->pVerts[index] ) : b3Vec3_zero );
		}
	}

	CPhysCollide *pCollide = CreateMeshCollide( verts.data(), NULL, pList->triangleCount, true );
	if ( pCollide )
		pCollide->m_nMeshSurfaceProp = pList->surfacePropsIndex;

	delete pList;
	return pCollide;
}

bool CBox3DPhysicsCollision::SupportsVirtualMesh()
{
	return true;
}

//-------------------------------------------------------------------------------------------------
// IPhysicsCollision: collides
//-------------------------------------------------------------------------------------------------

CPhysCollide *CBox3DPhysicsCollision::ConvertConvexToCollide( CPhysConvex **pConvex, int convexCount )
{
	CPhysCollide *pCollide = new CPhysCollide;
	for ( int i = 0; i < convexCount; i++ )
	{
		if ( pConvex[i] )
			pCollide->m_Convexes.AddToTail( pConvex[i] ); // takes ownership
	}
	pCollide->ComputeDerivedData();
	return pCollide;
}

CPhysCollide *CBox3DPhysicsCollision::ConvertConvexToCollideParams( CPhysConvex **pConvex, int convexCount, const convertconvexparams_t &convertParams )
{
	CPhysCollide *pCollide = ConvertConvexToCollide( pConvex, convexCount );
	if ( pCollide && convertParams.pForcedOuterHull )
		ConvexFree( convertParams.pForcedOuterHull );
	return pCollide;
}

void CBox3DPhysicsCollision::DestroyCollide( CPhysCollide *pCollide )
{
	if ( !pCollide || pCollide->m_bCached )
		return;

	// Objects may still be using it, free when the last one lets go
	if ( pCollide->m_nRefCount > 0 )
	{
		pCollide->m_bPendingDestroy = true;
		return;
	}

	delete pCollide;
}

//-------------------------------------------------------------------------------------------------
// Serialization. Box3D collides round-trip through a private layout tagged with the VPHY header
// so VCollideLoad() can tell them apart from IVP compact surfaces.
//-------------------------------------------------------------------------------------------------

namespace ivp_compat
{
	struct collideheader_t
	{
		int		vphysicsID;
		short	version;
		short	modelType;
	};

	struct compactsurfaceheader_t
	{
		int		surfaceSize;
		Vector	dragAxisAreas;
		int		axisMapSize;
	};

	struct compactsurface_t
	{
		float	mass_center[3];
		float	rotation_inertia[3];
		float	upper_limit_radius;

		unsigned int	max_factor_surface_deviation	: 8;
		int				byte_size						: 24;
		int				offset_ledgetree_root;
		int				dummy[3];
	};

	struct compactledge_t
	{
		int		c_point_offset;
		union
		{
			int	ledgetree_node_offset;
			int	client_data;
		};
		unsigned int	has_children_flag	: 2;
		int				is_compact_flag		: 2;
		unsigned int	dummy				: 4;
		unsigned int	size_div_16			: 24;
		short	n_triangles;
		short	for_future_use;
	};

	struct compactedge_t
	{
		unsigned int	start_point_index	: 16;
		int				opposite_index		: 15;
		unsigned int	is_virtual			: 1;
	};

	struct compacttriangle_t
	{
		unsigned int	tri_index		: 12;
		unsigned int	pierce_index	: 12;
		unsigned int	material_index	: 7;
		unsigned int	is_virtual		: 1;
		compactedge_t	c_three_edges[3];
	};

	struct compactledgenode_t
	{
		int		offset_right_node;
		int		offset_compact_ledge;
		float	center[3];
		float	radius;
		unsigned char box_sizes[3];
		unsigned char free_0;

		bool IsTerminal() const { return offset_right_node == 0; }
		const compactledge_t *GetCompactLedge() const { return (const compactledge_t *)( (const char *)this + offset_compact_ledge ); }
		const compactledgenode_t *GetLeftChild() const { return this + 1; }
		const compactledgenode_t *GetRightChild() const { return (const compactledgenode_t *)( (const char *)this + offset_right_node ); }
	};

	static const int	IVP_COMPACT_SURFACE_ID			= MAKEID( 'I', 'V', 'P', 'S' );
	static const int	IVP_COMPACT_SURFACE_ID_SWAPPED	= MAKEID( 'S', 'P', 'V', 'I' );
	static const int	IVP_COMPACT_MOPP_ID				= MAKEID( 'M', 'O', 'P', 'P' );
	static const int	VPHYSICS_COLLISION_ID			= MAKEID( 'V', 'P', 'H', 'Y' );
	static const short	VPHYSICS_COLLISION_VERSION		= 0x0100;

	enum
	{
		COLLIDE_POLY = 0,
		COLLIDE_MOPP = 1,
		COLLIDE_BALL = 2,
		COLLIDE_VIRTUAL = 3,
		COLLIDE_BOX3D = 0x4233, // 'B3', private to this backend
	};

	// IVP stores points in metres with its own axes: hl = ( x, z, -y )
	static inline b3Vec3 IVPPointToHL( const float *p )
	{
		b3Vec3 r = { p[0] * B3VP_IN_PER_M, p[2] * B3VP_IN_PER_M, -p[1] * B3VP_IN_PER_M };
		return r;
	}

	static void GatherLedges( const compactledgenode_t *pNode, CUtlVector< const compactledge_t * > &out, int depth = 0 )
	{
		if ( !pNode || depth > 128 )
			return;

		if ( pNode->IsTerminal() )
		{
			out.AddToTail( pNode->GetCompactLedge() );
		}
		else
		{
			GatherLedges( pNode->GetLeftChild(), out, depth + 1 );
			GatherLedges( pNode->GetRightChild(), out, depth + 1 );
		}
	}

	static CPhysConvex *LedgeToConvex( const compactledge_t *pLedge )
	{
		if ( pLedge->n_triangles <= 0 )
			return NULL;

		const char *pPoints = (const char *)pLedge + pLedge->c_point_offset;
		const compacttriangle_t *pTriangles = (const compacttriangle_t *)( pLedge + 1 );

		std::vector< b3Vec3 > points;
		points.reserve( pLedge->n_triangles * 3 );

		for ( int i = 0; i < pLedge->n_triangles; i++ )
		{
			for ( int j = 0; j < 3; j++ )
			{
				const int index = pTriangles[i].c_three_edges[j].start_point_index;
				const float *p = (const float *)( pPoints + index * 16 ); // IVP points are 16 byte aligned
				points.push_back( IVPPointToHL( p ) );
			}
		}

		CPhysConvex *pConvex = CPhysConvex::CreateFromPoints( points.data(), (int)points.size() );
		if ( pConvex )
		{
			pConvex->m_gameData = (unsigned int)pLedge->client_data;
			pConvex->m_materialIndex = pTriangles[0].material_index;
		}
		return pConvex;
	}

	static CPhysCollide *LoadCompactSurface( const compactsurface_t *pSurface, const Vector *pDragAreas )
	{
		const compactledgenode_t *pRoot = (const compactledgenode_t *)( (const char *)pSurface + pSurface->offset_ledgetree_root );

		CUtlVector< const compactledge_t * > ledges;
		GatherLedges( pRoot, ledges );

		CPhysCollide *pCollide = new CPhysCollide;
		for ( int i = 0; i < ledges.Count(); i++ )
		{
			CPhysConvex *pConvex = LedgeToConvex( ledges[i] );
			if ( pConvex )
				pCollide->m_Convexes.AddToTail( pConvex );
		}

		pCollide->m_MassCenter = IVPPointToHL( pSurface->mass_center );
		pCollide->m_bMassCenterSet = true;
		if ( pDragAreas )
			pCollide->m_OrthoAreas = *pDragAreas;

		pCollide->ComputeDerivedData();
		return pCollide;
	}
}

// Writer/reader for the private format
class CB3VPCollideWriter
{
public:
	CB3VPCollideWriter( char *pDest ) : m_pDest( pDest ), m_nSize( 0 ) {}

	void Write( const void *pData, int size )
	{
		if ( m_pDest )
			memcpy( m_pDest + m_nSize, pData, size );
		m_nSize += size;
	}
	void WriteInt( int v ) { Write( &v, sizeof( v ) ); }
	void WriteFloat( float v ) { Write( &v, sizeof( v ) ); }
	void WriteVec( const b3Vec3 &v ) { WriteFloat( v.x ); WriteFloat( v.y ); WriteFloat( v.z ); }

	char *m_pDest;
	int m_nSize;
};

class CB3VPCollideReader
{
public:
	CB3VPCollideReader( const char *pData, int size ) : m_pData( pData ), m_nSize( size ), m_nPos( 0 ), m_bError( false ) {}

	bool Read( void *pOut, int size )
	{
		if ( m_bError || m_nPos + size > m_nSize )
		{
			m_bError = true;
			memset( pOut, 0, size );
			return false;
		}
		memcpy( pOut, m_pData + m_nPos, size );
		m_nPos += size;
		return true;
	}
	int ReadInt() { int v = 0; Read( &v, sizeof( v ) ); return v; }
	float ReadFloat() { float v = 0; Read( &v, sizeof( v ) ); return v; }
	b3Vec3 ReadVec() { b3Vec3 v; v.x = ReadFloat(); v.y = ReadFloat(); v.z = ReadFloat(); return v; }

	const char *m_pData;
	int m_nSize;
	int m_nPos;
	bool m_bError;
};

static int WriteBox3DCollide( CB3VPCollideWriter &writer, const CPhysCollide *pCollide )
{
	ivp_compat::collideheader_t header;
	header.vphysicsID = ivp_compat::VPHYSICS_COLLISION_ID;
	header.version = ivp_compat::VPHYSICS_COLLISION_VERSION;
	header.modelType = (short)ivp_compat::COLLIDE_BOX3D;
	writer.Write( &header, sizeof( header ) );

	writer.WriteInt( pCollide->m_Type );
	writer.WriteVec( pCollide->m_MassCenter );
	writer.WriteInt( pCollide->m_bMassCenterSet ? 1 : 0 );
	writer.WriteFloat( pCollide->m_OrthoAreas.x );
	writer.WriteFloat( pCollide->m_OrthoAreas.y );
	writer.WriteFloat( pCollide->m_OrthoAreas.z );

	if ( pCollide->m_Type == CPhysCollide::COLLIDE_MESH && pCollide->m_pMesh )
	{
		const b3MeshData *pMesh = pCollide->m_pMesh;
		const b3Vec3 *pVerts = b3GetMeshVertices( pMesh );
		const b3MeshTriangle *pTris = b3GetMeshTriangles( pMesh );
		const uint8_t *pMats = b3GetMeshMaterialIndices( pMesh );

		writer.WriteInt( pMesh->triangleCount );
		writer.WriteInt( pCollide->m_nMeshSurfaceProp );
		for ( int i = 0; i < pMesh->triangleCount; i++ )
		{
			// Written CCW (Box3D's front face)
			writer.WriteVec( pVerts[pTris[i].index1] );
			writer.WriteVec( pVerts[pTris[i].index2] );
			writer.WriteVec( pVerts[pTris[i].index3] );
			writer.WriteInt( pMats ? pMats[i] : 0 );
		}
	}
	else
	{
		writer.WriteInt( pCollide->m_Convexes.Count() );
		for ( int i = 0; i < pCollide->m_Convexes.Count(); i++ )
		{
			const CPhysConvex *pConvex = pCollide->m_Convexes[i];
			const b3HullData *pHull = pConvex->m_pHull;
			const int nPoints = pHull ? pHull->vertexCount : 0;
			writer.WriteInt( (int)pConvex->m_gameData );
			writer.WriteInt( pConvex->m_materialIndex );
			writer.WriteInt( nPoints );
			const b3Vec3 *pPoints = pHull ? b3GetHullPoints( pHull ) : NULL;
			for ( int p = 0; p < nPoints; p++ )
				writer.WriteVec( pPoints[p] );
		}
	}

	return writer.m_nSize;
}

static CPhysCollide *ReadBox3DCollide( const char *pData, int size )
{
	CB3VPCollideReader reader( pData, size );

	ivp_compat::collideheader_t header;
	reader.Read( &header, sizeof( header ) );
	if ( header.vphysicsID != ivp_compat::VPHYSICS_COLLISION_ID || header.modelType != (short)ivp_compat::COLLIDE_BOX3D )
		return NULL;

	const int type = reader.ReadInt();
	const b3Vec3 massCenter = reader.ReadVec();
	const bool bMassCenterSet = reader.ReadInt() != 0;
	Vector ortho;
	ortho.x = reader.ReadFloat();
	ortho.y = reader.ReadFloat();
	ortho.z = reader.ReadFloat();

	CPhysCollide *pCollide = NULL;
	if ( type == CPhysCollide::COLLIDE_MESH )
	{
		const int triCount = reader.ReadInt();
		const int surfaceProp = reader.ReadInt();
		if ( triCount <= 0 || triCount > ( size / 40 ) )
			return NULL;

		std::vector< b3Vec3 > verts( triCount * 3 );
		std::vector< uint8 > mats( triCount );
		for ( int i = 0; i < triCount; i++ )
		{
			// Stored CCW, CreateMeshCollide defaults to clockwise when not auto-orienting
			verts[i * 3 + 0] = reader.ReadVec();
			verts[i * 3 + 2] = reader.ReadVec();
			verts[i * 3 + 1] = reader.ReadVec();
			mats[i] = (uint8)reader.ReadInt();
		}
		if ( reader.m_bError )
			return NULL;

		pCollide = CBox3DPhysicsCollision::GetInstance().CreateMeshCollide( verts.data(), mats.data(), triCount, false );
		if ( pCollide )
			pCollide->m_nMeshSurfaceProp = surfaceProp;
	}
	else
	{
		const int convexCount = reader.ReadInt();
		if ( convexCount < 0 || convexCount > size )
			return NULL;

		pCollide = new CPhysCollide;
		for ( int i = 0; i < convexCount && !reader.m_bError; i++ )
		{
			const unsigned int gameData = (unsigned int)reader.ReadInt();
			const int material = reader.ReadInt();
			const int nPoints = reader.ReadInt();
			if ( nPoints < 0 || nPoints > B3_MAX_HULL_VERTICES * 4 )
				break;

			std::vector< b3Vec3 > points( nPoints );
			for ( int p = 0; p < nPoints; p++ )
				points[p] = reader.ReadVec();

			CPhysConvex *pConvex = CPhysConvex::CreateFromPoints( points.data(), nPoints );
			if ( pConvex )
			{
				pConvex->m_gameData = gameData;
				pConvex->m_materialIndex = material;
				pCollide->m_Convexes.AddToTail( pConvex );
			}
		}
	}

	if ( pCollide )
	{
		pCollide->m_OrthoAreas = ortho;
		pCollide->m_MassCenter = massCenter;
		pCollide->m_bMassCenterSet = bMassCenterSet;
		pCollide->ComputeDerivedData();
	}
	return pCollide;
}

int CBox3DPhysicsCollision::CollideSize( CPhysCollide *pCollide )
{
	if ( !pCollide )
		return 0;
	CB3VPCollideWriter sizer( NULL );
	return WriteBox3DCollide( sizer, pCollide );
}

int CBox3DPhysicsCollision::CollideWrite( char *pDest, CPhysCollide *pCollide, bool bSwap )
{
	if ( !pCollide || !pDest )
		return 0;
	if ( bSwap )
		B3VP_Unsupported( "Byte swapped collide serialization" );

	CB3VPCollideWriter writer( pDest );
	return WriteBox3DCollide( writer, pCollide );
}

CPhysCollide *CBox3DPhysicsCollision::UnserializeCollide( char *pBuffer, int size, int index )
{
	CPhysCollide *pCollide = NULL;

	const ivp_compat::collideheader_t *pHeader = (const ivp_compat::collideheader_t *)pBuffer;
	if ( size >= (int)sizeof( *pHeader ) && pHeader->vphysicsID == ivp_compat::VPHYSICS_COLLISION_ID )
	{
		if ( pHeader->modelType == (short)ivp_compat::COLLIDE_BOX3D )
		{
			pCollide = ReadBox3DCollide( pBuffer, size );
		}
		else if ( pHeader->modelType == ivp_compat::COLLIDE_POLY )
		{
			const ivp_compat::compactsurfaceheader_t *pSurfaceHeader = (const ivp_compat::compactsurfaceheader_t *)( pHeader + 1 );
			const ivp_compat::compactsurface_t *pSurface = (const ivp_compat::compactsurface_t *)( pSurfaceHeader + 1 );
			pCollide = ivp_compat::LoadCompactSurface( pSurface, &pSurfaceHeader->dragAxisAreas );
		}
	}
	else if ( size >= (int)sizeof( ivp_compat::compactsurface_t ) )
	{
		pCollide = ivp_compat::LoadCompactSurface( (const ivp_compat::compactsurface_t *)pBuffer, NULL );
	}

	if ( pCollide )
		pCollide->m_nIndex = index;
	return pCollide;
}

//-------------------------------------------------------------------------------------------------
// IPhysicsCollision: collide queries
//-------------------------------------------------------------------------------------------------

float CBox3DPhysicsCollision::CollideVolume( CPhysCollide *pCollide )
{
	return pCollide ? pCollide->m_flVolume : 0.0f;
}

float CBox3DPhysicsCollision::CollideSurfaceArea( CPhysCollide *pCollide )
{
	return pCollide ? pCollide->m_flSurfaceArea : 0.0f;
}

Vector CBox3DPhysicsCollision::CollideGetExtent( const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, const Vector &direction )
{
	if ( !pCollide )
		return collideOrigin;

	const b3Transform xf = MakeHLTransform( collideOrigin, collideAngles );
	const b3Vec3 localDir = b3InvRotateVector( xf.q, V2B( direction ) );

	float bestDot = -FLT_MAX;
	b3Vec3 best = pCollide->m_MassCenter;

	if ( pCollide->IsMesh() && pCollide->m_pMesh )
	{
		const b3Vec3 *pVerts = b3GetMeshVertices( pCollide->m_pMesh );
		for ( int i = 0; i < pCollide->m_pMesh->vertexCount; i++ )
		{
			const float d = b3Dot( pVerts[i], localDir );
			if ( d > bestDot ) { bestDot = d; best = pVerts[i]; }
		}
	}
	else
	{
		for ( int c = 0; c < pCollide->m_Convexes.Count(); c++ )
		{
			const b3HullData *pHull = pCollide->m_Convexes[c]->m_pHull;
			if ( !pHull )
				continue;
			const b3Vec3 *pPoints = b3GetHullPoints( pHull );
			for ( int i = 0; i < pHull->vertexCount; i++ )
			{
				const float d = b3Dot( pPoints[i], localDir );
				if ( d > bestDot ) { bestDot = d; best = pPoints[i]; }
			}
		}
	}

	return B2V( b3TransformPoint( xf, best ) );
}

void CBox3DPhysicsCollision::CollideGetAABB( Vector *pMins, Vector *pMaxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles )
{
	if ( !pCollide )
	{
		if ( pMins ) *pMins = collideOrigin;
		if ( pMaxs ) *pMaxs = collideOrigin;
		return;
	}

	const b3Transform xf = MakeHLTransform( collideOrigin, collideAngles );
	b3AABB box = B3VP_EmptyAABB();

	const bool bIdentityRotation = collideAngles == vec3_angle;

	if ( pCollide->IsMesh() && pCollide->m_pMesh )
	{
		box = b3AABB_Transform( xf, pCollide->m_pMesh->bounds );
	}
	else
	{
		for ( int c = 0; c < pCollide->m_Convexes.Count(); c++ )
		{
			const b3HullData *pHull = pCollide->m_Convexes[c]->m_pHull;
			if ( !pHull )
				continue;

			// Exact for small collides, bounds-of-bounds for huge ones (the world)
			if ( bIdentityRotation || pCollide->m_Convexes.Count() > 64 )
			{
				b3AABB hullBox = b3AABB_Transform( xf, pHull->aabb );
				B3VP_AddPoint( box, hullBox.lowerBound );
				B3VP_AddPoint( box, hullBox.upperBound );
			}
			else
			{
				const b3Vec3 *pPoints = b3GetHullPoints( pHull );
				for ( int i = 0; i < pHull->vertexCount; i++ )
					B3VP_AddPoint( box, b3TransformPoint( xf, pPoints[i] ) );
			}
		}
	}

	if ( box.lowerBound.x > box.upperBound.x )
	{
		box.lowerBound = V2B( collideOrigin );
		box.upperBound = V2B( collideOrigin );
	}

	if ( pMins ) *pMins = B2V( box.lowerBound );
	if ( pMaxs ) *pMaxs = B2V( box.upperBound );
}

void CBox3DPhysicsCollision::CollideGetMassCenter( CPhysCollide *pCollide, Vector *pOutMassCenter )
{
	if ( pOutMassCenter )
		*pOutMassCenter = pCollide ? B2V( pCollide->m_MassCenter ) : vec3_origin;
}

void CBox3DPhysicsCollision::CollideSetMassCenter( CPhysCollide *pCollide, const Vector &massCenter )
{
	if ( !pCollide )
		return;
	pCollide->m_MassCenter = V2B( massCenter );
	pCollide->m_bMassCenterSet = true;
}

Vector CBox3DPhysicsCollision::CollideGetOrthographicAreas( const CPhysCollide *pCollide )
{
	return pCollide ? pCollide->m_OrthoAreas : Vector( 1, 1, 1 );
}

void CBox3DPhysicsCollision::CollideSetOrthographicAreas( CPhysCollide *pCollide, const Vector &areas )
{
	if ( pCollide )
		pCollide->m_OrthoAreas = areas;
}

int CBox3DPhysicsCollision::CollideIndex( const CPhysCollide *pCollide )
{
	return pCollide ? pCollide->m_nIndex : 0;
}

CPhysCollide *CBox3DPhysicsCollision::BBoxToCollide( const Vector &mins, const Vector &maxs )
{
	// The game never frees these (see PhysCreateBbox), so cache them by size like IVP does
	for ( int i = 0; i < m_BBoxCache.Count(); i++ )
	{
		if ( m_BBoxCache[i].mins == mins && m_BBoxCache[i].maxs == maxs )
			return m_BBoxCache[i].pCollide;
	}

	CPhysConvex *pConvex = BBoxToConvex( mins, maxs );
	CPhysCollide *pCollide = ConvertConvexToCollide( &pConvex, 1 );
	pCollide->m_bCached = true;

	BBoxCacheEntry_t entry;
	entry.mins = mins;
	entry.maxs = maxs;
	entry.pCollide = pCollide;
	m_BBoxCache.AddToTail( entry );
	return pCollide;
}

void CBox3DPhysicsCollision::FreeBBoxCache()
{
	for ( int i = 0; i < m_BBoxCache.Count(); i++ )
	{
		CPhysCollide *pCollide = m_BBoxCache[i].pCollide;
		pCollide->m_bCached = false;
		if ( pCollide->m_nRefCount <= 0 )
			delete pCollide;
		else
			pCollide->m_bPendingDestroy = true;
	}
	m_BBoxCache.RemoveAll();
}

int CBox3DPhysicsCollision::GetConvexesUsedInCollideable( const CPhysCollide *pCollideable, CPhysConvex **pOutputArray, int iOutputArrayLimit )
{
	if ( !pCollideable || !pOutputArray )
		return 0;

	const int count = MIN( pCollideable->m_Convexes.Count(), iOutputArrayLimit );
	for ( int i = 0; i < count; i++ )
		pOutputArray[i] = pCollideable->m_Convexes[i];
	return count;
}

//-------------------------------------------------------------------------------------------------
// IPhysicsCollision: traces
//-------------------------------------------------------------------------------------------------

void CBox3DPhysicsCollision::TraceBox( const Vector &start, const Vector &end, const Vector &mins, const Vector &maxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr )
{
	Ray_t ray;
	ray.Init( start, end, mins, maxs );
	B3VP_TraceCollide( ray, MASK_ALL, NULL, pCollide, collideOrigin, collideAngles, ptr );
}

void CBox3DPhysicsCollision::TraceBox( const Ray_t &ray, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr )
{
	B3VP_TraceCollide( ray, MASK_ALL, NULL, pCollide, collideOrigin, collideAngles, ptr );
}

void CBox3DPhysicsCollision::TraceBox( const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr )
{
	B3VP_TraceCollide( ray, contentsMask, pConvexInfo, pCollide, collideOrigin, collideAngles, ptr );
}

void CBox3DPhysicsCollision::TraceCollide( const Vector &start, const Vector &end, const CPhysCollide *pSweepCollide, const QAngle &sweepAngles, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr )
{
	B3VP_ClearTrace( ptr );
	ptr->startpos = start;
	ptr->endpos = end;

	if ( !pSweepCollide || !pCollide || pSweepCollide->IsMesh() )
		return;

	// Sweep collide space -> target collide space
	const b3Transform targetXf = MakeHLTransform( collideOrigin, collideAngles );
	const b3Transform sweepXf = MakeHLTransform( start, sweepAngles );
	const b3Transform sweepToTarget = b3InvMulTransforms( targetXf, sweepXf );
	const b3Vec3 deltaLocal = b3InvRotateVector( targetXf.q, V2B( end - start ) );
	const bool bSwept = ( end - start ).LengthSqr() > 0.0f;

	TraceHit_t hit;
	hit.bHit = false;
	hit.fraction = 1.0f;
	hit.contents = 0;
	hit.normal = b3Vec3_zero;
	hit.point = b3Vec3_zero;

	b3Vec3 points[B3_MAX_SHAPE_CAST_POINTS];

	for ( int s = 0; s < pSweepCollide->m_Convexes.Count(); s++ )
	{
		const b3HullData *pSweepHull = pSweepCollide->m_Convexes[s]->m_pHull;
		if ( !pSweepHull )
			continue;

		const b3Vec3 *pSweepPoints = b3GetHullPoints( pSweepHull );
		const int nPoints = MIN( pSweepHull->vertexCount, B3_MAX_SHAPE_CAST_POINTS );
		b3AABB sweptBounds = B3VP_EmptyAABB();
		for ( int i = 0; i < nPoints; i++ )
		{
			points[i] = b3TransformPoint( sweepToTarget, pSweepPoints[i] );
			B3VP_AddPoint( sweptBounds, points[i] );
			B3VP_AddPoint( sweptBounds, b3Add( points[i], deltaLocal ) );
		}

		b3ShapeProxy proxy;
		proxy.points = points;
		proxy.count = nPoints;
		proxy.radius = 0.0f;

		bool bStartSolid = false;
		unsigned int startContents = 0;
		if ( pCollide->IsMesh() )
			TraceAgainstMesh( pCollide, MASK_ALL, false, bSwept, b3Vec3_zero, deltaLocal, proxy, sweptBounds, hit );
		else
			TraceAgainstConvexes( pCollide, MASK_ALL, NULL, false, bSwept, b3Vec3_zero, deltaLocal, proxy, sweptBounds, hit, bStartSolid, startContents );

		if ( bStartSolid )
		{
			ptr->startsolid = true;
			ptr->allsolid = true;
			ptr->fraction = 0.0f;
			ptr->endpos = start;
			ptr->contents = startContents;
			return;
		}
	}

	if ( hit.bHit )
	{
		const Vector normal = B2V( b3Normalize( b3RotateVector( targetXf.q, hit.normal ) ) );
		const Vector point = B2V( b3TransformPoint( targetXf, hit.point ) );
		ptr->fraction = SourceFraction( end - start, hit.fraction, normal );
		ptr->endpos = start + ( end - start ) * ptr->fraction;
		ptr->contents = hit.contents;
		SetTracePlane( ptr, normal, DotProduct( point, normal ) );
	}
}

bool CBox3DPhysicsCollision::IsBoxIntersectingCone( const Vector &boxAbsMins, const Vector &boxAbsMaxs, const truncatedcone_t &cone )
{
	// Conservative sphere-vs-cone test
	const Vector center = ( boxAbsMins + boxAbsMaxs ) * 0.5f;
	const float radius = ( boxAbsMaxs - boxAbsMins ).Length() * 0.5f;

	const Vector toCenter = center - cone.origin;
	const float along = DotProduct( toCenter, cone.normal );
	if ( along < -radius || along > cone.h + radius )
		return false;

	const float radial = ( toCenter - cone.normal * along ).Length();
	const float coneRadius = MAX( along, 0.0f ) * tanf( DEG2RAD( cone.theta ) );
	return radial <= coneRadius + radius;
}

//-------------------------------------------------------------------------------------------------
// IPhysicsCollision: vcollide loading
//-------------------------------------------------------------------------------------------------

void CBox3DPhysicsCollision::VCollideLoad( vcollide_t *pOutput, int solidCount, const char *pBuffer, int size, bool swap )
{
	memset( pOutput, 0, sizeof( *pOutput ) );
	if ( swap )
		B3VP_Unsupported( "Byte swapped vcollide data" );

	pOutput->solidCount = solidCount;
	pOutput->solids = new CPhysCollide *[solidCount];

	const char *pCursor = pBuffer;
	const char *pEnd = pBuffer + size;

	for ( int i = 0; i < solidCount; i++ )
	{
		pOutput->solids[i] = NULL;

		if ( pCursor + sizeof( int ) > pEnd )
			break;

		const int solidSize = *(const int *)pCursor;
		pCursor += sizeof( int );

		if ( solidSize <= 0 || pCursor + solidSize > pEnd )
		{
			B3VP_Warning( "VCollideLoad: solid %d has a bad size (%d)\n", i, solidSize );
			break;
		}

		const ivp_compat::collideheader_t *pHeader = (const ivp_compat::collideheader_t *)pCursor;
		CPhysCollide *pCollide = NULL;

		if ( pHeader->vphysicsID == ivp_compat::VPHYSICS_COLLISION_ID )
		{
			switch ( pHeader->modelType )
			{
			case ivp_compat::COLLIDE_POLY:
				{
					const ivp_compat::compactsurfaceheader_t *pSurfaceHeader = (const ivp_compat::compactsurfaceheader_t *)( pHeader + 1 );
					const ivp_compat::compactsurface_t *pSurface = (const ivp_compat::compactsurface_t *)( pSurfaceHeader + 1 );
					pCollide = ivp_compat::LoadCompactSurface( pSurface, &pSurfaceHeader->dragAxisAreas );
				}
				break;

			case ivp_compat::COLLIDE_BOX3D:
				pCollide = ReadBox3DCollide( pCursor, solidSize );
				break;

			default:
				B3VP_Warning( "VCollideLoad: unsupported solid type %d (solid %d)\n", (int)pHeader->modelType, i );
				break;
			}
		}
		else
		{
			// Legacy .phy: a bare IVP compact surface
			const ivp_compat::compactsurface_t *pSurface = (const ivp_compat::compactsurface_t *)pCursor;
			const int legacyType = pSurface->dummy[2];
			if ( legacyType == 0 || legacyType == ivp_compat::IVP_COMPACT_SURFACE_ID || legacyType == ivp_compat::IVP_COMPACT_SURFACE_ID_SWAPPED )
				pCollide = ivp_compat::LoadCompactSurface( pSurface, NULL );
			else
				B3VP_Warning( "VCollideLoad: unsupported legacy solid 0x%x (solid %d)\n", legacyType, i );
		}

		if ( pCollide )
			pCollide->m_nIndex = i;

		pOutput->solids[i] = pCollide;
		pCursor += solidSize;
	}

	// The rest of the buffer is the text key data
	const int keySize = (int)MAX( 0, (int)( pEnd - pCursor ) );
	pOutput->pKeyValues = new char[keySize + 1];
	if ( keySize > 0 )
		memcpy( pOutput->pKeyValues, pCursor, keySize );
	pOutput->pKeyValues[keySize] = '\0';
	pOutput->descSize = keySize;
	pOutput->isPacked = false;
}

void CBox3DPhysicsCollision::VCollideUnload( vcollide_t *pVCollide )
{
	if ( !pVCollide )
		return;

	for ( int i = 0; i < pVCollide->solidCount; i++ )
		DestroyCollide( pVCollide->solids[i] );

	delete[] pVCollide->solids;
	delete[] pVCollide->pKeyValues;
	memset( pVCollide, 0, sizeof( *pVCollide ) );
}

IVPhysicsKeyParser *CBox3DPhysicsCollision::VPhysicsKeyParserCreate( const char *pKeyData )
{
	return B3VP_CreateKeyParser( pKeyData );
}

void CBox3DPhysicsCollision::VPhysicsKeyParserDestroy( IVPhysicsKeyParser *pParser )
{
	B3VP_DestroyKeyParser( pParser );
}

//-------------------------------------------------------------------------------------------------
// IPhysicsCollision: debug meshes and query models
//-------------------------------------------------------------------------------------------------

int CBox3DPhysicsCollision::CreateDebugMesh( CPhysCollide const *pCollisionModel, Vector **outVerts )
{
	*outVerts = NULL;
	if ( !pCollisionModel )
		return 0;

	std::vector< Vector > verts;

	if ( pCollisionModel->IsMesh() && pCollisionModel->m_pMesh )
	{
		const b3MeshData *pMesh = pCollisionModel->m_pMesh;
		const b3Vec3 *pVerts = b3GetMeshVertices( pMesh );
		const b3MeshTriangle *pTris = b3GetMeshTriangles( pMesh );
		for ( int i = 0; i < pMesh->triangleCount; i++ )
		{
			// Source debug meshes are clockwise
			verts.push_back( B2V( pVerts[pTris[i].index1] ) );
			verts.push_back( B2V( pVerts[pTris[i].index3] ) );
			verts.push_back( B2V( pVerts[pTris[i].index2] ) );
		}
	}
	else
	{
		struct Collector
		{
			std::vector< Vector > *pOut;
			void operator()( const b3Vec3 &a, const b3Vec3 &b, const b3Vec3 &c )
			{
				pOut->push_back( B2V( a ) );
				pOut->push_back( B2V( c ) );
				pOut->push_back( B2V( b ) );
			}
		} collector = { &verts };

		for ( int i = 0; i < pCollisionModel->m_Convexes.Count(); i++ )
		{
			if ( pCollisionModel->m_Convexes[i]->m_pHull )
				ForEachHullTriangle( pCollisionModel->m_Convexes[i]->m_pHull, collector );
		}
	}

	if ( verts.empty() )
		return 0;

	Vector *pOut = new Vector[verts.size()];
	for ( size_t i = 0; i < verts.size(); i++ )
		pOut[i] = verts[i];

	*outVerts = pOut;
	return (int)verts.size();
}

void CBox3DPhysicsCollision::DestroyDebugMesh( int vertCount, Vector *outVerts )
{
	delete[] outVerts;
}

class CBox3DCollisionQuery : public ICollisionQuery
{
public:
	CBox3DCollisionQuery( CPhysCollide *pCollide ) : m_pCollide( pCollide )
	{
		for ( int c = 0; c < pCollide->m_Convexes.Count(); c++ )
		{
			ConvexData_t &data = m_Convexes[m_Convexes.AddToTail()];
			data.bDirty = false;

			struct Collector
			{
				ConvexData_t *pData;
				void operator()( const b3Vec3 &a, const b3Vec3 &b, const b3Vec3 &c )
				{
					pData->verts.push_back( B2V( a ) );
					pData->verts.push_back( B2V( c ) );
					pData->verts.push_back( B2V( b ) );
				}
			} collector = { &data };

			const CPhysConvex *pConvex = pCollide->m_Convexes[c];
			if ( pConvex->m_pHull )
				ForEachHullTriangle( pConvex->m_pHull, collector );
			data.materials.resize( data.verts.size() / 3, pConvex->m_materialIndex );
		}
	}

	~CBox3DCollisionQuery()
	{
		// Write back edited convexes
		for ( int c = 0; c < m_Convexes.Count(); c++ )
		{
			if ( !m_Convexes[c].bDirty )
				continue;

			std::vector< b3Vec3 > points( m_Convexes[c].verts.size() );
			for ( size_t i = 0; i < points.size(); i++ )
				points[i] = V2B( m_Convexes[c].verts[i] );

			CPhysConvex *pNew = CPhysConvex::CreateFromPoints( points.data(), (int)points.size() );
			if ( pNew )
			{
				CPhysConvex *pOld = m_pCollide->m_Convexes[c];
				pNew->m_gameData = pOld->m_gameData;
				pNew->m_materialIndex = m_Convexes[c].materials.empty() ? pOld->m_materialIndex : m_Convexes[c].materials[0];
				m_pCollide->m_Convexes[c] = pNew;
				delete pOld;
			}
		}
		m_pCollide->ComputeDerivedData();
	}

	int ConvexCount( void ) { return m_Convexes.Count(); }
	int TriangleCount( int convexIndex ) { return IsValid( convexIndex ) ? (int)m_Convexes[convexIndex].verts.size() / 3 : 0; }
	unsigned int GetGameData( int convexIndex ) { return IsValid( convexIndex ) ? m_pCollide->m_Convexes[convexIndex]->m_gameData : 0; }

	void GetTriangleVerts( int convexIndex, int triangleIndex, Vector *verts )
	{
		if ( !IsValid( convexIndex, triangleIndex ) )
			return;
		for ( int i = 0; i < 3; i++ )
			verts[i] = m_Convexes[convexIndex].verts[triangleIndex * 3 + i];
	}

	void SetTriangleVerts( int convexIndex, int triangleIndex, const Vector *verts )
	{
		if ( !IsValid( convexIndex, triangleIndex ) )
			return;
		for ( int i = 0; i < 3; i++ )
			m_Convexes[convexIndex].verts[triangleIndex * 3 + i] = verts[i];
		m_Convexes[convexIndex].bDirty = true;
	}

	int GetTriangleMaterialIndex( int convexIndex, int triangleIndex )
	{
		return IsValid( convexIndex, triangleIndex ) ? m_Convexes[convexIndex].materials[triangleIndex] : 0;
	}

	void SetTriangleMaterialIndex( int convexIndex, int triangleIndex, int index7bits )
	{
		if ( !IsValid( convexIndex, triangleIndex ) )
			return;
		m_Convexes[convexIndex].materials[triangleIndex] = index7bits & 0x7F;
		m_pCollide->m_Convexes[convexIndex]->m_materialIndex = index7bits & 0x7F;
	}

private:
	bool IsValid( int c ) const { return c >= 0 && c < m_Convexes.Count(); }
	bool IsValid( int c, int t ) const { return IsValid( c ) && t >= 0 && t * 3 + 2 < (int)m_Convexes[c].verts.size(); }

	struct ConvexData_t
	{
		std::vector< Vector > verts;
		std::vector< int > materials;
		bool bDirty;
	};

	CPhysCollide *m_pCollide;
	CUtlVector< ConvexData_t > m_Convexes;
};

ICollisionQuery *CBox3DPhysicsCollision::CreateQueryModel( CPhysCollide *pCollide )
{
	if ( !pCollide )
		return NULL;
	return new CBox3DCollisionQuery( pCollide );
}

void CBox3DPhysicsCollision::DestroyQueryModel( ICollisionQuery *pQuery )
{
	delete pQuery;
}

//-------------------------------------------------------------------------------------------------
// IPhysicsCollision: misc
//-------------------------------------------------------------------------------------------------

IPhysicsCollision *CBox3DPhysicsCollision::ThreadContextCreate( void )
{
	// All queries are stateless
	return this;
}

void CBox3DPhysicsCollision::ThreadContextDestroy( IPhysicsCollision *pThreadContex )
{
}

bool CBox3DPhysicsCollision::GetBBoxCacheSize( int *pCachedSize, int *pCachedCount )
{
	if ( pCachedCount )
		*pCachedCount = m_BBoxCache.Count();
	if ( pCachedSize )
		*pCachedSize = m_BBoxCache.Count() * (int)( sizeof( CPhysCollide ) + sizeof( CPhysConvex ) + sizeof( b3BoxHull ) );
	return true;
}

CPolyhedron *CBox3DPhysicsCollision::PolyhedronFromConvex( CPhysConvex * const pConvex, bool bUseTempPolyhedron )
{
	if ( !pConvex || !pConvex->m_pHull )
		return NULL;

	const b3HullData *pHull = pConvex->m_pHull;
	const b3Vec3 *pPoints = b3GetHullPoints( pHull );
	const b3HullHalfEdge *pEdges = b3GetHullEdges( pHull );
	const b3HullFace *pFaces = b3GetHullFaces( pHull );
	const b3Plane *pPlanes = b3GetHullPlanes( pHull );

	// Map half edges onto unique lines
	std::vector< int > lineOfEdge( pHull->edgeCount, -1 );
	int lineCount = 0;
	for ( int e = 0; e < pHull->edgeCount; e++ )
	{
		if ( lineOfEdge[e] >= 0 )
			continue;
		lineOfEdge[e] = lineCount;
		lineOfEdge[pEdges[e].twin] = lineCount;
		lineCount++;
	}

	int indexCount = 0;
	for ( int f = 0; f < pHull->faceCount; f++ )
	{
		int e = pFaces[f].edge;
		int guard = 0;
		do { indexCount++; e = pEdges[e].next; } while ( e != pFaces[f].edge && guard++ < 256 );
	}

	CPolyhedron *pOut = bUseTempPolyhedron
		? GetTempPolyhedron( (unsigned short)pHull->vertexCount, (unsigned short)lineCount, (unsigned short)indexCount, (unsigned short)pHull->faceCount )
		: CPolyhedron_AllocByNew::Allocate( (unsigned short)pHull->vertexCount, (unsigned short)lineCount, (unsigned short)indexCount, (unsigned short)pHull->faceCount );
	if ( !pOut )
		return NULL;

	for ( int v = 0; v < pHull->vertexCount; v++ )
		pOut->pVertices[v] = B2V( pPoints[v] );

	// Canonical half edge of each line defines its point order
	std::vector< int > canonical( lineCount, -1 );
	for ( int e = 0; e < pHull->edgeCount; e++ )
	{
		const int line = lineOfEdge[e];
		if ( canonical[line] < 0 )
		{
			canonical[line] = e;
			pOut->pLines[line].iPointIndices[0] = pEdges[e].origin;
			pOut->pLines[line].iPointIndices[1] = pEdges[pEdges[e].twin].origin;
		}
	}

	int index = 0;
	for ( int f = 0; f < pHull->faceCount; f++ )
	{
		pOut->pPolygons[f].iFirstIndex = (unsigned short)index;
		pOut->pPolygons[f].polyNormal = B2V( pPlanes[f].normal );

		int e = pFaces[f].edge;
		int count = 0;
		int guard = 0;
		do
		{
			const int line = lineOfEdge[e];
			pOut->pIndices[index].iLineIndex = (unsigned short)line;
			pOut->pIndices[index].iEndPointIndex = ( canonical[line] == e ) ? 0 : 1;
			index++;
			count++;
			e = pEdges[e].next;
		} while ( e != pFaces[f].edge && guard++ < 256 );

		pOut->pPolygons[f].iIndexCount = (unsigned short)count;
	}

	return pOut;
}

void CBox3DPhysicsCollision::OutputDebugInfo( const CPhysCollide *pCollide )
{
	if ( !pCollide )
	{
		Msg( "NULL collide\n" );
		return;
	}

	if ( pCollide->IsMesh() )
	{
		Msg( "Box3D mesh collide: %d triangles, area %.2f in^2\n", pCollide->m_pMesh ? pCollide->m_pMesh->triangleCount : 0, pCollide->m_flSurfaceArea );
	}
	else
	{
		int triangles = 0;
		for ( int i = 0; i < pCollide->m_Convexes.Count(); i++ )
			triangles += pCollide->m_Convexes[i]->m_pHull ? CountHullTriangles( pCollide->m_Convexes[i]->m_pHull ) : 0;
		Msg( "Box3D convex collide: %d convexes, %d triangles, volume %.2f in^3\n", pCollide->m_Convexes.Count(), triangles, pCollide->m_flVolume );
	}
}

unsigned int CBox3DPhysicsCollision::ReadStat( int statID )
{
	return 0;
}
