//=================================================================================================
//
// VPhysics Box3D - collision models (CPhysConvex / CPhysCollide) and IPhysicsCollision
//
//=================================================================================================

#ifndef B3VP_COLLIDE_H
#define B3VP_COLLIDE_H

#ifdef _WIN32
#pragma once
#endif

#include "b3vp_base.h"

struct Ray_t;
class CGameTrace;
typedef CGameTrace trace_t;

//-------------------------------------------------------------------------------------------------
// NOTE: collision geometry is stored in INCHES (Source units), in the local space of the collide.
//
// Box3D's geometric queries use B3_LINEAR_SLOP as a tolerance (5mm when the world runs in metres),
// which would leave traces a quarter inch short of every surface. Keeping the trace geometry in
// inches makes that tolerance 0.005", which is tighter than IVP's. Rigid bodies get metre-scaled
// copies of the hulls through b3CreateTransformedHullShape (shared via Box3D's hull database).
//-------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------
// A single convex piece.
//-------------------------------------------------------------------------------------------------
class CPhysConvex
{
public:
	CPhysConvex() : m_pHull( NULL ), m_gameData( 0 ), m_materialIndex( 0 ) {}
	~CPhysConvex();

	// Builds a hull from a point cloud (inches). Returns NULL if degenerate.
	static CPhysConvex *CreateFromPoints( const b3Vec3 *pPoints, int nPoints );
	static CPhysConvex *CreateBox( const b3Vec3 &mins, const b3Vec3 &maxs );

	CPhysConvex *Clone() const;

	b3HullData		*m_pHull;
	unsigned int	m_gameData;			// SetConvexGameData() / IVP ledge client data
	int				m_materialIndex;	// 7-bit material index from IVP data (map-local for world brushes)
};

//-------------------------------------------------------------------------------------------------
// A triangle soup that will become a static mesh collide
//-------------------------------------------------------------------------------------------------
class CPhysPolysoup
{
public:
	std::vector< b3Vec3 >	m_Verts;		// 3 per triangle, inches
	std::vector< uint8 >	m_Materials;	// 1 per triangle
};

//-------------------------------------------------------------------------------------------------
// A collision model: either a set of convex hulls or a static triangle mesh.
//-------------------------------------------------------------------------------------------------
class CPhysCollide
{
public:
	enum CollideType_t
	{
		COLLIDE_CONVEXES = 0,
		COLLIDE_MESH,
	};

	CPhysCollide();
	~CPhysCollide();

	// Recomputes bounds, volume, surface area and (unless set explicitly) mass centre
	void			ComputeDerivedData();

	bool			IsMesh() const		{ return m_Type == COLLIDE_MESH; }

	// Usage tracking so collides referenced by live objects aren't freed under them
	void			AddRef()			{ ++m_nRefCount; }
	void			Release();

	CollideType_t				m_Type;
	CUtlVector< CPhysConvex * >	m_Convexes;			// owned

	// Mesh collides
	b3MeshData *				m_pMesh;			// owned
	int							m_nMeshSurfaceProp;	// surfaceprop index from a virtual mesh, or -1

	// Derived data (inches)
	b3AABB						m_Bounds;
	float						m_flVolume;			// in^3
	float						m_flSurfaceArea;	// in^2
	b3Vec3						m_MassCenter;
	bool						m_bMassCenterSet;	// mass centre came from the file / was set explicitly
	Vector						m_OrthoAreas;		// in^2, from the .phy header (drag)
	int							m_nIndex;			// solid index inside its vcollide_t

	int							m_nRefCount;
	bool						m_bPendingDestroy;
	bool						m_bCached;			// lives in the bbox cache, never destroyed by the game
};

//-------------------------------------------------------------------------------------------------
// Trace helpers shared with the environment
//-------------------------------------------------------------------------------------------------
void B3VP_ClearTrace( trace_t *pTrace );
void B3VP_TraceCollide( const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo,
						const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *pTrace );

// Returns the convex/mesh shapes of a collide in a world transform. Fills a proxy for GJK style queries.
int  B3VP_MakeBoxProxy( const b3Vec3 &centerLocal, const b3Vec3 &halfExtentWorld, const b3Quat &worldToLocal, b3Vec3 *pOutPoints );

//-------------------------------------------------------------------------------------------------
// IPhysicsCollision implementation
//-------------------------------------------------------------------------------------------------
class CBox3DPhysicsCollision : public IPhysicsCollision
{
public:
	CBox3DPhysicsCollision();
	~CBox3DPhysicsCollision();

	static CBox3DPhysicsCollision &GetInstance() { return s_Instance; }

	CPhysConvex		*ConvexFromVerts( Vector **pVerts, int vertCount );
	CPhysConvex		*ConvexFromPlanes( float *pPlanes, int planeCount, float mergeDistance );
	float			ConvexVolume( CPhysConvex *pConvex );
	float			ConvexSurfaceArea( CPhysConvex *pConvex );
	void			SetConvexGameData( CPhysConvex *pConvex, unsigned int gameData );
	void			ConvexFree( CPhysConvex *pConvex );
	CPhysConvex		*BBoxToConvex( const Vector &mins, const Vector &maxs );
	CPhysConvex		*ConvexFromConvexPolyhedron( const CPolyhedron &ConvexPolyhedron );
	void			ConvexesFromConvexPolygon( const Vector &vPolyNormal, const Vector *pPoints, int iPointCount, CPhysConvex **pOutput );

	CPhysPolysoup	*PolysoupCreate( void );
	void			PolysoupDestroy( CPhysPolysoup *pSoup );
	void			PolysoupAddTriangle( CPhysPolysoup *pSoup, const Vector &a, const Vector &b, const Vector &c, int materialIndex7bits );
	CPhysCollide	*ConvertPolysoupToCollide( CPhysPolysoup *pSoup, bool useMOPP );

	CPhysCollide	*ConvertConvexToCollide( CPhysConvex **pConvex, int convexCount );
	CPhysCollide	*ConvertConvexToCollideParams( CPhysConvex **pConvex, int convexCount, const convertconvexparams_t &convertParams );
	void			DestroyCollide( CPhysCollide *pCollide );

	int				CollideSize( CPhysCollide *pCollide );
	int				CollideWrite( char *pDest, CPhysCollide *pCollide, bool bSwap = false );
	CPhysCollide	*UnserializeCollide( char *pBuffer, int size, int index );

	float			CollideVolume( CPhysCollide *pCollide );
	float			CollideSurfaceArea( CPhysCollide *pCollide );
	Vector			CollideGetExtent( const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, const Vector &direction );
	void			CollideGetAABB( Vector *pMins, Vector *pMaxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles );
	void			CollideGetMassCenter( CPhysCollide *pCollide, Vector *pOutMassCenter );
	void			CollideSetMassCenter( CPhysCollide *pCollide, const Vector &massCenter );
	Vector			CollideGetOrthographicAreas( const CPhysCollide *pCollide );
	void			CollideSetOrthographicAreas( CPhysCollide *pCollide, const Vector &areas );
	int				CollideIndex( const CPhysCollide *pCollide );

	CPhysCollide	*BBoxToCollide( const Vector &mins, const Vector &maxs );
	int				GetConvexesUsedInCollideable( const CPhysCollide *pCollideable, CPhysConvex **pOutputArray, int iOutputArrayLimit );

	void			TraceBox( const Vector &start, const Vector &end, const Vector &mins, const Vector &maxs, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr );
	void			TraceBox( const Ray_t &ray, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr );
	void			TraceBox( const Ray_t &ray, unsigned int contentsMask, IConvexInfo *pConvexInfo, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr );
	void			TraceCollide( const Vector &start, const Vector &end, const CPhysCollide *pSweepCollide, const QAngle &sweepAngles, const CPhysCollide *pCollide, const Vector &collideOrigin, const QAngle &collideAngles, trace_t *ptr );

	bool			IsBoxIntersectingCone( const Vector &boxAbsMins, const Vector &boxAbsMaxs, const truncatedcone_t &cone );

	void			VCollideLoad( vcollide_t *pOutput, int solidCount, const char *pBuffer, int size, bool swap = false );
	void			VCollideUnload( vcollide_t *pVCollide );

	IVPhysicsKeyParser	*VPhysicsKeyParserCreate( const char *pKeyData );
	void			VPhysicsKeyParserDestroy( IVPhysicsKeyParser *pParser );

	int				CreateDebugMesh( CPhysCollide const *pCollisionModel, Vector **outVerts );
	void			DestroyDebugMesh( int vertCount, Vector *outVerts );

	ICollisionQuery	*CreateQueryModel( CPhysCollide *pCollide );
	void			DestroyQueryModel( ICollisionQuery *pQuery );

	IPhysicsCollision *ThreadContextCreate( void );
	void			ThreadContextDestroy( IPhysicsCollision *pThreadContex );

	CPhysCollide	*CreateVirtualMesh( const virtualmeshparams_t &params );
	bool			SupportsVirtualMesh();

	bool			GetBBoxCacheSize( int *pCachedSize, int *pCachedCount );

	CPolyhedron		*PolyhedronFromConvex( CPhysConvex * const pConvex, bool bUseTempPolyhedron );

	void			OutputDebugInfo( const CPhysCollide *pCollide );
	unsigned int	ReadStat( int statID );

public:
	// Internal helpers
	CPhysCollide	*CreateMeshCollide( const b3Vec3 *pTriangleVerts, const uint8 *pMaterials, int triangleCount, bool bAutoOrient );
	void			FreeBBoxCache();

private:
	static CBox3DPhysicsCollision s_Instance;

	struct BBoxCacheEntry_t
	{
		Vector mins;
		Vector maxs;
		CPhysCollide *pCollide;
	};
	CUtlVector< BBoxCacheEntry_t > m_BBoxCache;
};

#endif // B3VP_COLLIDE_H
