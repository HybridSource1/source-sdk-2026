//=================================================================================================
//
// VPhysics Box3D - IPhysics, collision sets, object pair hashes and Box3D hooks
//
//=================================================================================================

#include "b3vp_base.h"
#include "b3vp_environment.h"
#include "b3vp_collide.h"

#include "tier1/tier1.h"
#include "tier0/memalloc.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

ConVar box3d_threads( "box3d_threads", "1", FCVAR_ARCHIVE, "Worker threads Box3D uses for each physics environment (takes effect on map load). 1 = single threaded.", true, 1, true, B3_MAX_WORKERS );

//-------------------------------------------------------------------------------------------------
// Box3D allocation, assert and log hooks. Installed at static init so collision models loaded
// before IPhysics::Init still go through the engine allocator.
//-------------------------------------------------------------------------------------------------

static void *B3VP_Alloc( size_t size, int32_t alignment )
{
	return MemAlloc_AllocAligned( size, MAX( alignment, 16 ) );
}

static void B3VP_Free( void *mem, size_t size )
{
	if ( mem )
		MemAlloc_FreeAligned( mem );
}

static int B3VP_Assert( const char *condition, const char *fileName, int lineNumber )
{
	Warning( B3VP_LOG_PREFIX "Box3D assert: %s (%s:%d)\n", condition, fileName, lineNumber );
	return 0; // never break into the debugger from the engine
}

static void B3VP_Log( const char *message )
{
	DevMsg( B3VP_LOG_PREFIX "%s\n", message );
}

class CB3VPHooks
{
public:
	CB3VPHooks()
	{
		b3SetAllocator( B3VP_Alloc, B3VP_Free );
		b3SetAssertFcn( B3VP_Assert );
		b3SetLogFcn( B3VP_Log );
		b3SetLengthUnitsPerMeter( 1.0f );
	}
};
static CB3VPHooks s_Box3DHooks;

//-------------------------------------------------------------------------------------------------
// Collision sets: symmetric bit matrix of which ragdoll parts collide with each other
//-------------------------------------------------------------------------------------------------

class CBox3DCollisionSet : public IPhysicsCollisionSet
{
public:
	enum { MAX_ELEMENTS = 64 };

	CBox3DCollisionSet()
	{
		for ( int i = 0; i < MAX_ELEMENTS; i++ )
			m_Bits[i] = ~(uint64)0; // everything collides until told otherwise
	}

	void EnableCollisions( int index0, int index1 )
	{
		if ( !Valid( index0, index1 ) )
			return;
		m_Bits[index0] |= (uint64)1 << index1;
		m_Bits[index1] |= (uint64)1 << index0;
	}

	void DisableCollisions( int index0, int index1 )
	{
		if ( !Valid( index0, index1 ) )
			return;
		m_Bits[index0] &= ~( (uint64)1 << index1 );
		m_Bits[index1] &= ~( (uint64)1 << index0 );
	}

	bool ShouldCollide( int index0, int index1 )
	{
		if ( !Valid( index0, index1 ) )
			return true;
		return ( m_Bits[index0] & ( (uint64)1 << index1 ) ) != 0;
	}

private:
	static bool Valid( int a, int b ) { return a >= 0 && b >= 0 && a < MAX_ELEMENTS && b < MAX_ELEMENTS; }

	uint64 m_Bits[MAX_ELEMENTS];
};

//-------------------------------------------------------------------------------------------------
// Object pair hash: the game uses this to disable collisions between specific entity pairs
//-------------------------------------------------------------------------------------------------

class CBox3DObjectPairHash : public IPhysicsObjectPairHash
{
public:
	void AddObjectPair( void *pObject0, void *pObject1 )
	{
		if ( IsObjectPairInHash( pObject0, pObject1 ) )
			return;
		m_Pairs.insert( MakePair( pObject0, pObject1 ) );
		m_Adjacency[pObject0].push_back( pObject1 );
		if ( pObject0 != pObject1 )
			m_Adjacency[pObject1].push_back( pObject0 );
	}

	void RemoveObjectPair( void *pObject0, void *pObject1 )
	{
		if ( !m_Pairs.erase( MakePair( pObject0, pObject1 ) ) )
			return;
		RemoveAdjacency( pObject0, pObject1 );
		if ( pObject0 != pObject1 )
			RemoveAdjacency( pObject1, pObject0 );
	}

	bool IsObjectPairInHash( void *pObject0, void *pObject1 )
	{
		return m_Pairs.find( MakePair( pObject0, pObject1 ) ) != m_Pairs.end();
	}

	void RemoveAllPairsForObject( void *pObject0 )
	{
		std::map< void *, std::vector< void * > >::iterator it = m_Adjacency.find( pObject0 );
		if ( it == m_Adjacency.end() )
			return;

		std::vector< void * > others = it->second;
		for ( size_t i = 0; i < others.size(); i++ )
			RemoveObjectPair( pObject0, others[i] );
		m_Adjacency.erase( pObject0 );
	}

	bool IsObjectInHash( void *pObject0 )
	{
		std::map< void *, std::vector< void * > >::iterator it = m_Adjacency.find( pObject0 );
		return it != m_Adjacency.end() && !it->second.empty();
	}

	int GetPairCountForObject( void *pObject0 )
	{
		std::map< void *, std::vector< void * > >::iterator it = m_Adjacency.find( pObject0 );
		return it != m_Adjacency.end() ? (int)it->second.size() : 0;
	}

	int GetPairListForObject( void *pObject0, int nMaxCount, void **ppObjectList )
	{
		std::map< void *, std::vector< void * > >::iterator it = m_Adjacency.find( pObject0 );
		if ( it == m_Adjacency.end() )
			return 0;
		const int count = MIN( nMaxCount, (int)it->second.size() );
		for ( int i = 0; i < count; i++ )
			ppObjectList[i] = it->second[i];
		return count;
	}

private:
	typedef std::pair< void *, void * > Pair_t;
	static Pair_t MakePair( void *a, void *b ) { return a < b ? Pair_t( a, b ) : Pair_t( b, a ); }

	void RemoveAdjacency( void *pFrom, void *pTo )
	{
		std::map< void *, std::vector< void * > >::iterator it = m_Adjacency.find( pFrom );
		if ( it == m_Adjacency.end() )
			return;
		B3VP_EraseFast( it->second, pTo );
		if ( it->second.empty() )
			m_Adjacency.erase( it );
	}

	std::set< Pair_t >							m_Pairs;
	std::map< void *, std::vector< void * > >	m_Adjacency;
};

//-------------------------------------------------------------------------------------------------
// IPhysics
//-------------------------------------------------------------------------------------------------

class CBox3DPhysics : public CTier1AppSystem< IPhysics >
{
	typedef CTier1AppSystem< IPhysics > BaseClass;

public:
	static CBox3DPhysics &GetInstance() { return s_Instance; }

	InitReturnVal_t Init()
	{
		const InitReturnVal_t result = BaseClass::Init();
		if ( result != INIT_OK )
			return result;

		MathLib_Init( 2.2f, 2.2f, 0.0f, 2.0f );

		const b3Version version = b3GetVersion();
		Msg( B3VP_LOG_PREFIX "Using Box3D %d.%d.%d\n", version.major, version.minor, version.revision );
		return INIT_OK;
	}

	void Shutdown()
	{
		DestroyAllCollisionSets();
		CBox3DPhysicsCollision::GetInstance().FreeBBoxCache();
		BaseClass::Shutdown();
	}

	void *QueryInterface( const char *pInterfaceName )
	{
		CreateInterfaceFn factory = Sys_GetFactoryThis();
		return factory( pInterfaceName, NULL );
	}

	IPhysicsEnvironment *CreateEnvironment( void )
	{
		CBox3DPhysicsEnvironment *pEnv = new CBox3DPhysicsEnvironment;
		m_Environments.push_back( pEnv );
		return pEnv;
	}

	void DestroyEnvironment( IPhysicsEnvironment *pEnvironment )
	{
		CBox3DPhysicsEnvironment *pEnv = static_cast< CBox3DPhysicsEnvironment * >( pEnvironment );
		B3VP_EraseFast( m_Environments, pEnv );
		delete pEnv;
	}

	IPhysicsEnvironment *GetActiveEnvironmentByIndex( int index )
	{
		if ( index < 0 || index >= (int)m_Environments.size() )
			return NULL;
		return m_Environments[index];
	}

	IPhysicsObjectPairHash *CreateObjectPairHash()
	{
		return new CBox3DObjectPairHash;
	}

	void DestroyObjectPairHash( IPhysicsObjectPairHash *pHash )
	{
		delete static_cast< CBox3DObjectPairHash * >( pHash );
	}

	IPhysicsCollisionSet *FindOrCreateCollisionSet( unsigned int id, int maxElementCount )
	{
		if ( maxElementCount > CBox3DCollisionSet::MAX_ELEMENTS )
			return NULL;

		IPhysicsCollisionSet *pSet = FindCollisionSet( id );
		if ( pSet )
			return pSet;

		CBox3DCollisionSet *pNew = new CBox3DCollisionSet;
		m_CollisionSets[id] = pNew;
		return pNew;
	}

	IPhysicsCollisionSet *FindCollisionSet( unsigned int id )
	{
		std::map< unsigned int, CBox3DCollisionSet * >::iterator it = m_CollisionSets.find( id );
		return it != m_CollisionSets.end() ? it->second : NULL;
	}

	void DestroyAllCollisionSets()
	{
		for ( std::map< unsigned int, CBox3DCollisionSet * >::iterator it = m_CollisionSets.begin(); it != m_CollisionSets.end(); ++it )
			delete it->second;
		m_CollisionSets.clear();
	}

private:
	static CBox3DPhysics s_Instance;

	std::vector< CBox3DPhysicsEnvironment * >		m_Environments;
	std::map< unsigned int, CBox3DCollisionSet * >	m_CollisionSets;
};

CBox3DPhysics CBox3DPhysics::s_Instance;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR( CBox3DPhysics, IPhysics, VPHYSICS_INTERFACE_VERSION, CBox3DPhysics::GetInstance() );
