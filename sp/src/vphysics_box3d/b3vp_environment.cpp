//=================================================================================================
//
// VPhysics Box3D - IPhysicsEnvironment
//
//=================================================================================================

#include "b3vp_base.h"
#include "b3vp_environment.h"
#include "b3vp_object.h"
#include "b3vp_collide.h"
#include "b3vp_constraints.h"
#include "b3vp_controllers.h"
#include "b3vp_surfaceprops.h"

#include "cmodel.h"
#include "gametrace.h"
#include "isaverestore.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

static ConVar box3d_substeps( "box3d_substeps", "4", 0, "Box3D solver sub-steps per physics tick", true, 1, true, 16 );
static ConVar box3d_max_ticks_per_frame( "box3d_max_ticks_per_frame", "8", 0, "Most physics ticks simulated in one Simulate() call before time is dropped", true, 1, true, 64 );
static ConVar box3d_hit_event_speed( "box3d_hit_event_speed", "20", 0, "Approach speed (in/s) at which contacts generate collision callbacks" );
static ConVar box3d_max_collision_events( "box3d_max_collision_events", "256", 0, "Most collision callbacks dispatched per physics tick" );
static ConVar box3d_friction_events( "box3d_friction_events", "1", 0, "Send friction (scrape) callbacks to the game" );

// Mixing rules used by IVP: multiply the two materials
static float B3VP_FrictionMix( float frictionA, uint64_t, float frictionB, uint64_t )
{
	return frictionA * frictionB;
}

static float B3VP_RestitutionMix( float restitutionA, uint64_t, float restitutionB, uint64_t )
{
	return restitutionA * restitutionB;
}

static bool B3VP_CustomFilter( b3ShapeId shapeIdA, b3ShapeId shapeIdB, void *context )
{
	CBox3DPhysicsEnvironment *pEnv = static_cast< CBox3DPhysicsEnvironment * >( context );
	return pEnv->ShouldCollide( GetShapeObject( shapeIdA ), GetShapeObject( shapeIdB ) );
}

static bool B3VP_PreSolve( b3ShapeId shapeIdA, b3ShapeId shapeIdB, b3Pos point, b3Vec3 normal, void *context )
{
	CBox3DPhysicsEnvironment *pEnv = static_cast< CBox3DPhysicsEnvironment * >( context );
	return pEnv->PreSolve( GetShapeObject( shapeIdA ), GetShapeObject( shapeIdB ) );
}

//-------------------------------------------------------------------------------------------------
// Save/restore records
//-------------------------------------------------------------------------------------------------

#define B3VP_SAVE_MAGIC		MAKEID( 'B', '3', 'V', 'P' )

struct B3VPSaveHeader_t
{
	int		magic;
	int		type;
	void	*pOld;
};

struct B3VPConstraintSave_t
{
	void				*pRef;
	void				*pAtt;
	void				*pGroup;
	bool				bActive;
	B3VPJointDesc_t		desc;
};

struct B3VPSpringSave_t
{
	void						*pStart;
	void						*pEnd;
	CBox3DSpring::SaveData_t	data;
};

struct B3VPMotionSave_t
{
	int		priority;
	int		count;
};

//-------------------------------------------------------------------------------------------------

CBox3DPhysicsEnvironment::CBox3DPhysicsEnvironment()
	: m_pCollisionEvents( NULL )
	, m_pObjectEvents( NULL )
	, m_pConstraintEvents( NULL )
	, m_pCollisionSolver( NULL )
	, m_pDebugOverlay( NULL )
	, m_Gravity( 0, 0, -386.0f )
	, m_flAirDensity( 2.0f )
	, m_flTimestep( 1.0f / 66.0f )
	, m_flSimTime( 0.0f )
	, m_flAccumulator( 0.0f )
	, m_nFrameCounter( 0 )
	, m_bSimulating( false )
	, m_bDeleteQueue( false )
	, m_bQuickDelete( false )
	, m_bConstraintNotify( false )
	, m_bHasSimulated( false )
{
	m_PerformanceParams.Defaults();

	b3WorldDef def = b3DefaultWorldDef();
	def.gravity = HL2B3( m_Gravity );
	def.enableSleep = true;
	def.enableContinuous = true;
	def.frictionCallback = B3VP_FrictionMix;
	def.restitutionCallback = B3VP_RestitutionMix;
	def.hitEventThreshold = HL2B3( box3d_hit_event_speed.GetFloat() );
	def.maximumLinearSpeed = HL2B3( m_PerformanceParams.maxVelocity );
	def.userData = this;

	extern ConVar box3d_threads;
	def.workerCount = (uint32_t)clamp( box3d_threads.GetInt(), 1, B3_MAX_WORKERS );

	m_World = b3CreateWorld( &def );
	b3World_SetCustomFilterCallback( m_World, B3VP_CustomFilter, this );
	b3World_SetPreSolveCallback( m_World, B3VP_PreSolve, this );

	// A static body with no shapes, used to pin hinged objects to the world
	b3BodyDef anchorDef = b3DefaultBodyDef();
	anchorDef.type = b3_staticBody;
	m_AnchorBody = b3CreateBody( m_World, &anchorDef );
}

CBox3DPhysicsEnvironment::~CBox3DPhysicsEnvironment()
{
	// Stop callbacks into the game while tearing down
	m_pCollisionEvents = NULL;
	m_pObjectEvents = NULL;
	m_pConstraintEvents = NULL;
	m_pCollisionSolver = NULL;

	DeleteDeadObjects();

	// Controllers the game forgot about
	std::vector< IBox3DController * > controllers = m_Controllers;
	m_Controllers.clear();
	(void)controllers;

	std::vector< CBox3DPhysicsObject * > objects = m_Objects;
	m_Objects.clear();
	for ( size_t i = 0; i < objects.size(); i++ )
		delete objects[i];

	if ( b3World_IsValid( m_World ) )
		b3DestroyWorld( m_World );
}

//-------------------------------------------------------------------------------------------------
// Debug overlay / gravity / air
//-------------------------------------------------------------------------------------------------

void CBox3DPhysicsEnvironment::SetDebugOverlay( CreateInterfaceFn debugOverlayFactory )
{
	m_pDebugOverlay = debugOverlayFactory ? (IVPhysicsDebugOverlay *)debugOverlayFactory( VPHYSICS_DEBUG_OVERLAY_INTERFACE_VERSION, NULL ) : NULL;
}

IVPhysicsDebugOverlay *CBox3DPhysicsEnvironment::GetDebugOverlay( void )
{
	return m_pDebugOverlay;
}

void CBox3DPhysicsEnvironment::SetGravity( const Vector &gravityVector )
{
	m_Gravity = gravityVector;
	b3World_SetGravity( m_World, HL2B3( gravityVector ) );
}

void CBox3DPhysicsEnvironment::GetGravity( Vector *pGravityVector ) const
{
	if ( pGravityVector )
		*pGravityVector = m_Gravity;
}

void CBox3DPhysicsEnvironment::SetAirDensity( float density )
{
	m_flAirDensity = density;
}

float CBox3DPhysicsEnvironment::GetAirDensity( void ) const
{
	return m_flAirDensity;
}

float CBox3DPhysicsEnvironment::GetMaxLinearSpeed() const
{
	return HL2B3( m_PerformanceParams.maxVelocity );
}

float CBox3DPhysicsEnvironment::GetMaxAngularSpeed() const
{
	return DEG2RAD( m_PerformanceParams.maxAngularVelocity );
}

//-------------------------------------------------------------------------------------------------
// Objects
//-------------------------------------------------------------------------------------------------

IPhysicsObject *CBox3DPhysicsEnvironment::CreatePolyObject( const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams )
{
	if ( !pCollisionModel )
		return NULL;

	CBox3DPhysicsObject *pObject = new CBox3DPhysicsObject;
	if ( !pObject->Init( this, pCollisionModel, 0.0f, materialIndex, position, angles, pParams, false ) )
	{
		delete pObject;
		return NULL;
	}
	m_Objects.push_back( pObject );
	return pObject;
}

IPhysicsObject *CBox3DPhysicsEnvironment::CreatePolyObjectStatic( const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams )
{
	if ( !pCollisionModel )
		return NULL;

	CBox3DPhysicsObject *pObject = new CBox3DPhysicsObject;
	if ( !pObject->Init( this, pCollisionModel, 0.0f, materialIndex, position, angles, pParams, true ) )
	{
		delete pObject;
		return NULL;
	}
	m_Objects.push_back( pObject );
	return pObject;
}

IPhysicsObject *CBox3DPhysicsEnvironment::CreateSphereObject( float radius, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic )
{
	CBox3DPhysicsObject *pObject = new CBox3DPhysicsObject;
	if ( !pObject->Init( this, NULL, MAX( radius, 0.1f ), materialIndex, position, angles, pParams, isStatic ) )
	{
		delete pObject;
		return NULL;
	}
	m_Objects.push_back( pObject );
	return pObject;
}

void CBox3DPhysicsEnvironment::DestroyObject( IPhysicsObject *pIObject )
{
	CBox3DPhysicsObject *pObject = static_cast< CBox3DPhysicsObject * >( pIObject );
	if ( !pObject || pObject->IsMarkedForDelete() )
		return;

	pObject->SetCallbackFlags( pObject->GetCallbackFlags() | CALLBACK_MARKED_FOR_DELETE );

	if ( m_bSimulating || m_bDeleteQueue )
	{
		m_DeadObjects.push_back( pObject );
		return;
	}

	RemoveObjectNow( pObject );
}

void CBox3DPhysicsEnvironment::RemoveObjectNow( CBox3DPhysicsObject *pObject )
{
	B3VP_EraseFast( m_Objects, pObject );
	B3VP_EraseFast( m_ActiveObjects, pObject );
	delete pObject;
}

void CBox3DPhysicsEnvironment::DeleteDeadObjects()
{
	// Deleting can queue more (vehicle wheels, etc.)
	while ( !m_DeadObjects.empty() || !m_DeadConstraints.empty() )
	{
		std::vector< CBox3DConstraint * > constraints;
		constraints.swap( m_DeadConstraints );
		for ( size_t i = 0; i < constraints.size(); i++ )
			delete constraints[i];

		std::vector< CBox3DPhysicsObject * > objects;
		objects.swap( m_DeadObjects );
		for ( size_t i = 0; i < objects.size(); i++ )
			RemoveObjectNow( objects[i] );
	}
}

void CBox3DPhysicsEnvironment::CleanupDeleteList( void )
{
	if ( !m_bSimulating )
		DeleteDeadObjects();
}

void CBox3DPhysicsEnvironment::EnableDeleteQueue( bool enable )
{
	m_bDeleteQueue = enable;
}

//-------------------------------------------------------------------------------------------------
// Controllers
//-------------------------------------------------------------------------------------------------

void CBox3DPhysicsEnvironment::AddController( IBox3DController *pController )
{
	if ( !B3VP_Contains( m_Controllers, pController ) )
		m_Controllers.push_back( pController );
}

void CBox3DPhysicsEnvironment::RemoveController( IBox3DController *pController )
{
	std::vector< IBox3DController * >::iterator it = std::find( m_Controllers.begin(), m_Controllers.end(), pController );
	if ( it != m_Controllers.end() )
		m_Controllers.erase( it ); // keep order (priorities)
}

IPhysicsFluidController *CBox3DPhysicsEnvironment::CreateFluidController( IPhysicsObject *pFluidObject, fluidparams_t *pParams )
{
	CBox3DFluidController *pFluid = new CBox3DFluidController( this, static_cast< CBox3DPhysicsObject * >( pFluidObject ), pParams );
	AddController( pFluid );
	return pFluid;
}

void CBox3DPhysicsEnvironment::DestroyFluidController( IPhysicsFluidController *pController )
{
	CBox3DFluidController *pFluid = static_cast< CBox3DFluidController * >( pController );
	if ( !pFluid )
		return;
	RemoveController( pFluid );
	delete pFluid;
}

IPhysicsSpring *CBox3DPhysicsEnvironment::CreateSpring( IPhysicsObject *pObjectStart, IPhysicsObject *pObjectEnd, springparams_t *pParams )
{
	if ( !pParams )
		return NULL;
	return new CBox3DSpring( this, static_cast< CBox3DPhysicsObject * >( pObjectStart ), static_cast< CBox3DPhysicsObject * >( pObjectEnd ), pParams );
}

void CBox3DPhysicsEnvironment::DestroySpring( IPhysicsSpring *pSpring )
{
	delete static_cast< CBox3DSpring * >( pSpring );
}

IPhysicsShadowController *CBox3DPhysicsEnvironment::CreateShadowController( IPhysicsObject *pIObject, bool allowTranslation, bool allowRotation )
{
	CBox3DPhysicsObject *pObject = static_cast< CBox3DPhysicsObject * >( pIObject );
	if ( !pObject )
		return NULL;

	if ( pObject->GetShadowController() )
		return pObject->GetShadowController();

	CBox3DShadowController *pShadow = new CBox3DShadowController( pObject, allowTranslation, allowRotation );
	AddController( pShadow );
	return pShadow;
}

void CBox3DPhysicsEnvironment::DestroyShadowController( IPhysicsShadowController *pController )
{
	CBox3DShadowController *pShadow = static_cast< CBox3DShadowController * >( pController );
	if ( !pShadow )
		return;

	RemoveController( pShadow );

	CBox3DPhysicsObject *pObject = pShadow->GetObject();
	if ( pObject && pObject->GetShadowController() == pShadow )
		pObject->SetShadowControllerInternal( NULL );

	delete pShadow;
}

IPhysicsPlayerController *CBox3DPhysicsEnvironment::CreatePlayerController( IPhysicsObject *pObject )
{
	if ( !pObject )
		return NULL;
	CBox3DPlayerController *pPlayer = new CBox3DPlayerController( static_cast< CBox3DPhysicsObject * >( pObject ) );
	AddController( pPlayer );
	return pPlayer;
}

void CBox3DPhysicsEnvironment::DestroyPlayerController( IPhysicsPlayerController *pController )
{
	CBox3DPlayerController *pPlayer = static_cast< CBox3DPlayerController * >( pController );
	if ( !pPlayer )
		return;
	RemoveController( pPlayer );
	delete pPlayer;
}

IPhysicsMotionController *CBox3DPhysicsEnvironment::CreateMotionController( IMotionEvent *pHandler )
{
	CBox3DMotionController *pMotion = new CBox3DMotionController( this, pHandler );
	AddController( pMotion );
	return pMotion;
}

void CBox3DPhysicsEnvironment::DestroyMotionController( IPhysicsMotionController *pController )
{
	CBox3DMotionController *pMotion = static_cast< CBox3DMotionController * >( pController );
	if ( !pMotion )
		return;
	RemoveController( pMotion );
	delete pMotion;
}

IPhysicsVehicleController *CBox3DPhysicsEnvironment::CreateVehicleController( IPhysicsObject *pVehicleBodyObject, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace )
{
	CBox3DVehicleController *pVehicle = new CBox3DVehicleController( this, static_cast< CBox3DPhysicsObject * >( pVehicleBodyObject ), params, nVehicleType, pGameTrace );
	AddController( pVehicle );
	return pVehicle;
}

void CBox3DPhysicsEnvironment::DestroyVehicleController( IPhysicsVehicleController *pController )
{
	CBox3DVehicleController *pVehicle = static_cast< CBox3DVehicleController * >( pController );
	if ( !pVehicle )
		return;
	RemoveController( pVehicle );
	delete pVehicle;
}

//-------------------------------------------------------------------------------------------------
// Constraints
//-------------------------------------------------------------------------------------------------

IPhysicsConstraint *CBox3DPhysicsEnvironment::CreateRagdollConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll )
{
	CBox3DConstraint *pConstraint = new CBox3DConstraint( this, static_cast< CBox3DPhysicsObject * >( pReferenceObject ), static_cast< CBox3DPhysicsObject * >( pAttachedObject ) );
	pConstraint->InitRagdoll( static_cast< CBox3DConstraintGroup * >( pGroup ), ragdoll );
	return pConstraint;
}

IPhysicsConstraint *CBox3DPhysicsEnvironment::CreateHingeConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge )
{
	CBox3DConstraint *pConstraint = new CBox3DConstraint( this, static_cast< CBox3DPhysicsObject * >( pReferenceObject ), static_cast< CBox3DPhysicsObject * >( pAttachedObject ) );
	pConstraint->InitHinge( static_cast< CBox3DConstraintGroup * >( pGroup ), hinge );
	return pConstraint;
}

IPhysicsConstraint *CBox3DPhysicsEnvironment::CreateFixedConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed )
{
	CBox3DConstraint *pConstraint = new CBox3DConstraint( this, static_cast< CBox3DPhysicsObject * >( pReferenceObject ), static_cast< CBox3DPhysicsObject * >( pAttachedObject ) );
	pConstraint->InitFixed( static_cast< CBox3DConstraintGroup * >( pGroup ), fixed );
	return pConstraint;
}

IPhysicsConstraint *CBox3DPhysicsEnvironment::CreateSlidingConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding )
{
	CBox3DConstraint *pConstraint = new CBox3DConstraint( this, static_cast< CBox3DPhysicsObject * >( pReferenceObject ), static_cast< CBox3DPhysicsObject * >( pAttachedObject ) );
	pConstraint->InitSliding( static_cast< CBox3DConstraintGroup * >( pGroup ), sliding );
	return pConstraint;
}

IPhysicsConstraint *CBox3DPhysicsEnvironment::CreateBallsocketConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket )
{
	CBox3DConstraint *pConstraint = new CBox3DConstraint( this, static_cast< CBox3DPhysicsObject * >( pReferenceObject ), static_cast< CBox3DPhysicsObject * >( pAttachedObject ) );
	pConstraint->InitBallsocket( static_cast< CBox3DConstraintGroup * >( pGroup ), ballsocket );
	return pConstraint;
}

IPhysicsConstraint *CBox3DPhysicsEnvironment::CreatePulleyConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley )
{
	CBox3DConstraint *pConstraint = new CBox3DConstraint( this, static_cast< CBox3DPhysicsObject * >( pReferenceObject ), static_cast< CBox3DPhysicsObject * >( pAttachedObject ) );
	pConstraint->InitPulley( static_cast< CBox3DConstraintGroup * >( pGroup ), pulley );
	return pConstraint;
}

IPhysicsConstraint *CBox3DPhysicsEnvironment::CreateLengthConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length )
{
	CBox3DConstraint *pConstraint = new CBox3DConstraint( this, static_cast< CBox3DPhysicsObject * >( pReferenceObject ), static_cast< CBox3DPhysicsObject * >( pAttachedObject ) );
	pConstraint->InitLength( static_cast< CBox3DConstraintGroup * >( pGroup ), length );
	return pConstraint;
}

void CBox3DPhysicsEnvironment::DestroyConstraint( IPhysicsConstraint *pIConstraint )
{
	CBox3DConstraint *pConstraint = static_cast< CBox3DConstraint * >( pIConstraint );
	if ( !pConstraint )
		return;

	if ( !m_bQuickDelete )
	{
		if ( pConstraint->GetReferenceObject() ) pConstraint->GetReferenceObject()->Wake();
		if ( pConstraint->GetAttachedObject() ) pConstraint->GetAttachedObject()->Wake();
	}

	B3VP_EraseFast( m_PendingRestoreConstraints, pConstraint );

	if ( m_bSimulating )
	{
		pConstraint->Deactivate();
		if ( !B3VP_Contains( m_DeadConstraints, pConstraint ) )
			m_DeadConstraints.push_back( pConstraint );
		return;
	}

	delete pConstraint;
}

IPhysicsConstraintGroup *CBox3DPhysicsEnvironment::CreateConstraintGroup( const constraint_groupparams_t &groupParams )
{
	return new CBox3DConstraintGroup( groupParams );
}

void CBox3DPhysicsEnvironment::DestroyConstraintGroup( IPhysicsConstraintGroup *pGroup )
{
	delete static_cast< CBox3DConstraintGroup * >( pGroup );
}

void CBox3DPhysicsEnvironment::RegisterJoint( b3JointId joint, CBox3DConstraint *pConstraint )
{
	m_Joints[B3VP_JointKey( joint )] = pConstraint;
}

void CBox3DPhysicsEnvironment::UnregisterJoint( b3JointId joint )
{
	m_Joints.erase( B3VP_JointKey( joint ) );
}

void CBox3DPhysicsEnvironment::NotifyConstraintBroken( CBox3DConstraint *pConstraint )
{
	if ( m_pConstraintEvents && m_bConstraintNotify )
		m_pConstraintEvents->ConstraintBroken( pConstraint );
}

void CBox3DPhysicsEnvironment::EnableConstraintNotify( bool bEnable )
{
	m_bConstraintNotify = bEnable;
}

//-------------------------------------------------------------------------------------------------
// Collision filtering
//-------------------------------------------------------------------------------------------------

void CBox3DPhysicsEnvironment::SetCollisionSolver( IPhysicsCollisionSolver *pSolver )
{
	m_pCollisionSolver = pSolver;
}

bool CBox3DPhysicsEnvironment::ShouldCollide( CBox3DPhysicsObject *pObject0, CBox3DPhysicsObject *pObject1 )
{
	if ( !pObject0 || !pObject1 || pObject0 == pObject1 )
		return pObject0 != pObject1;

	if ( pObject0->IsMarkedForDelete() || pObject1->IsMarkedForDelete() )
		return false;

	if ( !pObject0->IsCollisionEnabled() || !pObject1->IsCollisionEnabled() )
		return false;

	// Two game-driven shadows never push each other
	if ( pObject0->GetShadowController() && pObject1->GetShadowController() )
		return false;

	if ( !m_pCollisionSolver )
		return true;

	return m_pCollisionSolver->ShouldCollide( pObject0, pObject1, pObject0->GetGameData(), pObject1->GetGameData() ) != 0;
}

bool CBox3DPhysicsEnvironment::PreSolve( CBox3DPhysicsObject *pObject0, CBox3DPhysicsObject *pObject1 )
{
	if ( !pObject0 || !pObject1 )
		return true;

	// The player's shadow only pushes objects up to its push mass limit
	CBox3DPlayerController *pPlayer0 = pObject0->GetPlayerControllerInternal();
	CBox3DPlayerController *pPlayer1 = pObject1->GetPlayerControllerInternal();
	if ( pPlayer0 && pObject1->IsMoveable() && pObject1->GetMass() > pPlayer0->GetPushMassLimit() )
		return false;
	if ( pPlayer1 && pObject0->IsMoveable() && pObject0->GetMass() > pPlayer1->GetPushMassLimit() )
		return false;

	return true;
}

//-------------------------------------------------------------------------------------------------
// Simulation
//-------------------------------------------------------------------------------------------------

void CBox3DPhysicsEnvironment::Simulate( float deltaTime )
{
	if ( deltaTime <= 0.0f || !b3World_IsValid( m_World ) )
		return;

	if ( m_bSimulating )
	{
		Warning( B3VP_LOG_PREFIX "Simulate() called recursively from a physics callback, ignoring.\n" );
		return;
	}

	m_bSimulating = true;
	m_nFrameCounter++;
	m_ActiveObjects.clear();

	m_flAccumulator += deltaTime;
	const int maxTicks = box3d_max_ticks_per_frame.GetInt();
	int ticks = 0;
	while ( m_flAccumulator >= m_flTimestep * 0.999f )
	{
		if ( ticks >= maxTicks )
		{
			m_flAccumulator = 0.0f;
			break;
		}

		StepOnce( m_flTimestep );
		m_flAccumulator -= m_flTimestep;
		ticks++;
	}

	if ( m_flAccumulator < 0.0f )
		m_flAccumulator = 0.0f;

	m_bSimulating = false;

	if ( !m_bDeleteQueue )
		DeleteDeadObjects();
}

void CBox3DPhysicsEnvironment::StepOnce( float dt )
{
	// Controllers (shadows, players, motion, fluids, springs, vehicles, pulleys). Callbacks into the
	// game may add or remove controllers, so walk a snapshot and skip anything removed.
	std::vector< IBox3DController * > controllers = m_Controllers;
	for ( size_t i = 0; i < controllers.size(); i++ )
	{
		if ( B3VP_Contains( m_Controllers, controllers[i] ) )
			controllers[i]->OnPreStep( dt );
	}

	for ( size_t i = 0; i < m_Objects.size(); i++ )
	{
		if ( !m_Objects[i]->IsMarkedForDelete() )
			m_Objects[i]->PreStep( dt );
	}

	b3World_SetHitEventThreshold( m_World, HL2B3( box3d_hit_event_speed.GetFloat() ) );
	b3World_Step( m_World, dt, box3d_substeps.GetInt() );
	m_bHasSimulated = true;
	m_flSimTime += dt;

	ProcessMoveEvents();
	ProcessSensorEvents();
	ProcessContactEvents();
	ProcessJointEvents();

	controllers = m_Controllers;
	for ( size_t i = 0; i < controllers.size(); i++ )
	{
		if ( B3VP_Contains( m_Controllers, controllers[i] ) )
			controllers[i]->OnPostStep( dt );
	}

	ProcessFriction( dt );
	ProcessSleepEvents();

	if ( m_pCollisionEvents )
		m_pCollisionEvents->PostSimulationFrame();
}

void CBox3DPhysicsEnvironment::ProcessMoveEvents()
{
	const b3BodyEvents events = b3World_GetBodyEvents( m_World );
	for ( int i = 0; i < events.moveCount; i++ )
	{
		CBox3DPhysicsObject *pObject = static_cast< CBox3DPhysicsObject * >( events.moveEvents[i].userData );
		if ( !pObject || pObject->IsMarkedForDelete() )
			continue;

		pObject->PostStep( m_flTimestep );

		if ( pObject->m_nActiveFrame != m_nFrameCounter )
		{
			pObject->m_nActiveFrame = m_nFrameCounter;
			m_ActiveObjects.push_back( pObject );
		}
	}
}

void CBox3DPhysicsEnvironment::ProcessSensorEvents()
{
	const b3SensorEvents events = b3World_GetSensorEvents( m_World );

	// Copy first, game callbacks may create or destroy shapes
	std::vector< b3SensorBeginTouchEvent > begins( events.beginEvents, events.beginEvents + events.beginCount );
	std::vector< b3SensorEndTouchEvent > ends( events.endEvents, events.endEvents + events.endCount );

	for ( size_t i = 0; i < begins.size(); i++ )
	{
		if ( !b3Shape_IsValid( begins[i].sensorShapeId ) || !b3Shape_IsValid( begins[i].visitorShapeId ) )
			continue;

		CBox3DPhysicsObject *pTrigger = GetShapeObject( begins[i].sensorShapeId );
		CBox3DPhysicsObject *pVisitor = GetShapeObject( begins[i].visitorShapeId );
		if ( !pTrigger || !pVisitor || pTrigger == pVisitor || pTrigger->IsMarkedForDelete() || pVisitor->IsMarkedForDelete() )
			continue;
		if ( pVisitor->IsTrigger() || pVisitor->IsStatic() )
			continue;

		int &count = m_TriggerOverlaps[ObjectPair_t( pTrigger, pVisitor )];
		if ( ++count != 1 )
			continue;

		CBox3DFluidController *pFluid = pTrigger->GetFluidControllerInternal();
		if ( pFluid )
		{
			if ( pVisitor->GetCallbackFlags() & CALLBACK_FLUID_TOUCH )
			{
				pFluid->AddObject( pVisitor );
				if ( m_pCollisionEvents )
					m_pCollisionEvents->FluidStartTouch( pVisitor, pFluid );
			}
		}
		else if ( m_pCollisionEvents )
		{
			m_pCollisionEvents->ObjectEnterTrigger( pTrigger, pVisitor );
		}
	}

	for ( size_t i = 0; i < ends.size(); i++ )
	{
		if ( !b3Shape_IsValid( ends[i].sensorShapeId ) || !b3Shape_IsValid( ends[i].visitorShapeId ) )
			continue;

		CBox3DPhysicsObject *pTrigger = GetShapeObject( ends[i].sensorShapeId );
		CBox3DPhysicsObject *pVisitor = GetShapeObject( ends[i].visitorShapeId );
		if ( !pTrigger || !pVisitor )
			continue;

		std::map< ObjectPair_t, int >::iterator it = m_TriggerOverlaps.find( ObjectPair_t( pTrigger, pVisitor ) );
		if ( it == m_TriggerOverlaps.end() )
			continue;
		if ( --it->second > 0 )
			continue;
		m_TriggerOverlaps.erase( it );

		CBox3DFluidController *pFluid = pTrigger->GetFluidControllerInternal();
		if ( pFluid )
		{
			pFluid->RemoveObject( pVisitor );
			if ( m_pCollisionEvents && ( pVisitor->GetCallbackFlags() & CALLBACK_FLUID_TOUCH ) )
				m_pCollisionEvents->FluidEndTouch( pVisitor, pFluid );
		}
		else if ( m_pCollisionEvents )
		{
			m_pCollisionEvents->ObjectLeaveTrigger( pTrigger, pVisitor );
		}
	}
}

static bool ShouldTouchCallback( CBox3DPhysicsObject *pObject0, CBox3DPhysicsObject *pObject1 )
{
	const unsigned short flags = pObject0->GetCallbackFlags() | pObject1->GetCallbackFlags();
	if ( !( flags & CALLBACK_GLOBAL_TOUCH ) )
		return false;
	if ( !( flags & CALLBACK_GLOBAL_TOUCH_STATIC ) && ( pObject0->IsStatic() || pObject1->IsStatic() ) )
		return false;
	return true;
}

static bool IsCollisionCallback( CBox3DPhysicsObject *pObject0, CBox3DPhysicsObject *pObject1 )
{
	bool bCollision = ( pObject0->GetCallbackFlags() & pObject1->GetCallbackFlags() & CALLBACK_GLOBAL_COLLISION ) != 0;
	if ( pObject0->IsStatic() && !( pObject1->GetCallbackFlags() & CALLBACK_GLOBAL_COLLIDE_STATIC ) )
		bCollision = false;
	if ( pObject1->IsStatic() && !( pObject0->GetCallbackFlags() & CALLBACK_GLOBAL_COLLIDE_STATIC ) )
		bCollision = false;
	return bCollision;
}

static void FillContactData( b3ContactId contactId, CBox3DCollisionData &data )
{
	if ( !b3Contact_IsValid( contactId ) )
		return;

	const b3ContactData contact = b3Contact_GetData( contactId );
	if ( contact.manifoldCount <= 0 || contact.manifolds[0].pointCount <= 0 )
		return;

	const b3Manifold &manifold = contact.manifolds[0];
	const b3Vec3 centerA = b3Body_GetWorldCenter( b3Shape_GetBody( contact.shapeIdA ) );
	data.m_Point = B32HL( b3Add( centerA, manifold.points[0].anchorA ) );
	data.m_Normal = ToHL( manifold.normal );

	const b3Vec3 p = b3Add( centerA, manifold.points[0].anchorA );
	const b3Vec3 vA = b3Body_GetWorldPointVelocity( b3Shape_GetBody( contact.shapeIdA ), p );
	const b3Vec3 vB = b3Body_GetWorldPointVelocity( b3Shape_GetBody( contact.shapeIdB ), p );
	data.m_Speed = B32HL( b3Sub( vB, vA ) );
}

void CBox3DPhysicsEnvironment::ProcessContactEvents()
{
	const b3ContactEvents events = b3World_GetContactEvents( m_World );

	std::vector< b3ContactBeginTouchEvent > begins( events.beginEvents, events.beginEvents + events.beginCount );
	std::vector< b3ContactEndTouchEvent > ends( events.endEvents, events.endEvents + events.endCount );
	std::vector< b3ContactHitEvent > hits( events.hitEvents, events.hitEvents + events.hitCount );

	// Collisions (impact sounds, damage)
	if ( m_pCollisionEvents )
	{
		std::vector< ObjectPair_t > handled;
		const int maxEvents = box3d_max_collision_events.GetInt();
		int sent = 0;

		for ( size_t i = 0; i < hits.size() && sent < maxEvents; i++ )
		{
			const b3ContactHitEvent &hit = hits[i];
			if ( !b3Shape_IsValid( hit.shapeIdA ) || !b3Shape_IsValid( hit.shapeIdB ) )
				continue;

			CBox3DPhysicsObject *pA = GetShapeObject( hit.shapeIdA );
			CBox3DPhysicsObject *pB = GetShapeObject( hit.shapeIdB );
			if ( !pA || !pB || pA == pB || pA->IsMarkedForDelete() || pB->IsMarkedForDelete() )
				continue;
			if ( pA->IsTrigger() || pB->IsTrigger() )
				continue;

			const bool bCollision = IsCollisionCallback( pA, pB );
			const bool bShadow = ( ( pA->GetCallbackFlags() ^ pB->GetCallbackFlags() ) & CALLBACK_SHADOW_COLLISION ) != 0;
			if ( !bCollision && !bShadow )
				continue;

			const ObjectPair_t pair = MakePair( pA, pB );
			if ( B3VP_Contains( handled, pair ) )
				continue;
			handled.push_back( pair );

			PairState_t &state = m_Pairs[pair];
			const float deltaTime = m_flSimTime - state.flLastCollisionTime;
			state.flLastCollisionTime = m_flSimTime;

			CBox3DCollisionData data;
			data.m_Normal = ToHL( hit.normal );
			data.m_Point = B32HL( hit.point );
			data.m_Speed = data.m_Normal * -B32HL( hit.approachSpeed );

			vcollisionevent_t event;
			event.pObjects[0] = pA;
			event.pObjects[1] = pB;
			event.surfaceProps[0] = (int)hit.userMaterialIdA;
			event.surfaceProps[1] = (int)hit.userMaterialIdB;
			event.isCollision = bCollision;
			event.isShadowCollision = bShadow;
			event.deltaCollisionTime = deltaTime;
			event.collisionSpeed = B32HL( hit.approachSpeed );
			event.pInternalData = &data;

			// The game wants the velocities from before the impact in PreCollision
			pA->SetUsePreStepVelocity( true );
			pB->SetUsePreStepVelocity( true );
			m_pCollisionEvents->PreCollision( &event );
			pA->SetUsePreStepVelocity( false );
			pB->SetUsePreStepVelocity( false );

			if ( pA->IsMarkedForDelete() || pB->IsMarkedForDelete() )
				continue;

			event.pInternalData = &data;
			m_pCollisionEvents->PostCollision( &event );
			sent++;
		}
	}

	// Touches
	for ( size_t i = 0; i < begins.size(); i++ )
	{
		if ( !b3Shape_IsValid( begins[i].shapeIdA ) || !b3Shape_IsValid( begins[i].shapeIdB ) )
			continue;

		CBox3DPhysicsObject *pA = GetShapeObject( begins[i].shapeIdA );
		CBox3DPhysicsObject *pB = GetShapeObject( begins[i].shapeIdB );
		if ( !pA || !pB || pA == pB || pA->IsMarkedForDelete() || pB->IsMarkedForDelete() )
			continue;

		PairState_t &state = m_Pairs[MakePair( pA, pB )];
		if ( ++state.nTouchCount != 1 )
			continue;

		if ( m_pCollisionEvents && ShouldTouchCallback( pA, pB ) )
		{
			CBox3DCollisionData data;
			FillContactData( begins[i].contactId, data );
			m_pCollisionEvents->StartTouch( pA, pB, &data );
		}
	}

	for ( size_t i = 0; i < ends.size(); i++ )
	{
		if ( !b3Shape_IsValid( ends[i].shapeIdA ) || !b3Shape_IsValid( ends[i].shapeIdB ) )
			continue;

		CBox3DPhysicsObject *pA = GetShapeObject( ends[i].shapeIdA );
		CBox3DPhysicsObject *pB = GetShapeObject( ends[i].shapeIdB );
		if ( !pA || !pB || pA == pB )
			continue;

		std::map< ObjectPair_t, PairState_t >::iterator it = m_Pairs.find( MakePair( pA, pB ) );
		if ( it == m_Pairs.end() || it->second.nTouchCount <= 0 )
			continue;
		if ( --it->second.nTouchCount > 0 )
			continue;

		if ( m_pCollisionEvents && ShouldTouchCallback( pA, pB ) && !pA->IsMarkedForDelete() && !pB->IsMarkedForDelete() )
		{
			// The game only reads the point/normal for debugging here
			CBox3DCollisionData data;
			pA->GetPosition( &data.m_Point, NULL );
			m_pCollisionEvents->EndTouch( pA, pB, &data );
		}
	}

	// Forget old impact times for pairs that are no longer touching
	if ( ( m_nFrameCounter & 63 ) == 0 )
	{
		for ( std::map< ObjectPair_t, PairState_t >::iterator it = m_Pairs.begin(); it != m_Pairs.end(); )
		{
			if ( it->second.nTouchCount <= 0 && m_flSimTime - it->second.flLastCollisionTime > 5.0f )
				m_Pairs.erase( it++ );
			else
				++it;
		}
	}
}

void CBox3DPhysicsEnvironment::ProcessJointEvents()
{
	const b3JointEvents events = b3World_GetJointEvents( m_World );
	if ( events.count <= 0 )
		return;

	std::vector< b3JointId > broken;
	for ( int i = 0; i < events.count; i++ )
		broken.push_back( events.jointEvents[i].jointId );

	for ( size_t i = 0; i < broken.size(); i++ )
	{
		std::map< uint64, CBox3DConstraint * >::iterator it = m_Joints.find( B3VP_JointKey( broken[i] ) );
		if ( it == m_Joints.end() )
			continue;

		CBox3DConstraint *pConstraint = it->second;
		const B3VPJointDesc_t &desc = pConstraint->GetDesc();
		if ( desc.breakForce <= 0.0f && desc.breakTorque <= 0.0f )
			continue;

		pConstraint->OnBroken();
	}
}

void CBox3DPhysicsEnvironment::ProcessFriction( float dt )
{
	if ( !m_pCollisionEvents || !box3d_friction_events.GetBool() )
		return;

	const int substeps = box3d_substeps.GetInt();
	CUtlVector< b3ContactData > contacts;

	for ( size_t o = 0; o < m_Objects.size(); o++ )
	{
		CBox3DPhysicsObject *pObject = m_Objects[o];
		if ( !( pObject->GetCallbackFlags() & CALLBACK_GLOBAL_FRICTION ) || pObject->IsMarkedForDelete() || pObject->IsTrigger() )
			continue;
		if ( !pObject->IsDynamicBody() || pObject->IsAsleep() )
			continue;

		const b3BodyId body = pObject->GetBody();
		const b3Vec3 v = b3Body_GetLinearVelocity( body );
		if ( b3Dot( v, v ) < 0.0025f ) // slower than ~2 in/s
			continue;

		const int capacity = b3Body_GetContactCapacity( body );
		if ( capacity <= 0 )
			continue;
		contacts.SetCount( capacity );
		const int count = b3Body_GetContactData( body, contacts.Base(), capacity );

		float bestEnergy = 0.0f;
		CBox3DCollisionData bestData;
		int bestSelfMaterial = 0, bestOtherMaterial = 0;
		CBox3DPhysicsObject *pBestOther = NULL;

		for ( int i = 0; i < count; i++ )
		{
			const b3ContactData &data = contacts[i];
			if ( !b3Shape_IsValid( data.shapeIdA ) || !b3Shape_IsValid( data.shapeIdB ) )
				continue;

			const bool bSelfIsA = GetShapeObject( data.shapeIdA ) == pObject;
			const b3ShapeId selfShape = bSelfIsA ? data.shapeIdA : data.shapeIdB;
			const b3ShapeId otherShape = bSelfIsA ? data.shapeIdB : data.shapeIdA;
			CBox3DPhysicsObject *pOther = GetShapeObject( otherShape );
			if ( !pOther || pOther->IsTrigger() || pOther->IsMarkedForDelete() )
				continue;

			const b3Vec3 centerA = b3Body_GetWorldCenter( b3Shape_GetBody( data.shapeIdA ) );
			for ( int m = 0; m < data.manifoldCount; m++ )
			{
				const b3Manifold &manifold = data.manifolds[m];
				if ( manifold.pointCount <= 0 )
					continue;

				const b3Vec3 p = b3Add( centerA, manifold.points[0].anchorA );
				const b3Vec3 rel = b3Sub( b3Body_GetWorldPointVelocity( body, p ), b3Body_GetWorldPointVelocity( pOther->GetBody(), p ) );
				const b3Vec3 tangent = b3MulAdd( rel, -b3Dot( rel, manifold.normal ), manifold.normal );
				const float slide = b3Length( tangent );
				const float energy = B3Energy2HL( b3Length( manifold.frictionImpulse ) * substeps * slide );
				if ( energy > bestEnergy )
				{
					bestEnergy = energy;
					bestData.m_Point = B32HL( p );
					bestData.m_Normal = ToHL( bSelfIsA ? manifold.normal : b3Neg( manifold.normal ) );
					bestData.m_Speed = B32HL( tangent );
					bestSelfMaterial = pObject->GetMaterialIndexForShape( selfShape );
					bestOtherMaterial = pOther->GetMaterialIndexForShape( otherShape );
					pBestOther = pOther;
				}
			}
		}

		if ( pBestOther && bestEnergy > 0.0f )
			m_pCollisionEvents->Friction( pObject, bestEnergy, bestSelfMaterial, bestOtherMaterial, &bestData );
	}
}

void CBox3DPhysicsEnvironment::ProcessSleepEvents()
{
	for ( size_t i = 0; i < m_Objects.size(); i++ )
	{
		CBox3DPhysicsObject *pObject = m_Objects[i];
		if ( pObject->IsStatic() || pObject->IsMarkedForDelete() )
			continue;

		const bool bAwake = !pObject->IsAsleep();
		if ( bAwake == pObject->m_bWasAwake )
			continue;

		pObject->m_bWasAwake = bAwake;
		if ( !m_pObjectEvents )
			continue;

		if ( bAwake )
			m_pObjectEvents->ObjectWake( pObject );
		else
			m_pObjectEvents->ObjectSleep( pObject );
	}
}

void CBox3DPhysicsEnvironment::OnObjectShapesRemoved( CBox3DPhysicsObject *pObject, bool bFireCallbacks )
{
	const bool bFire = bFireCallbacks && m_pCollisionEvents != NULL;

	for ( std::map< ObjectPair_t, PairState_t >::iterator it = m_Pairs.begin(); it != m_Pairs.end(); )
	{
		if ( it->first.first != pObject && it->first.second != pObject )
		{
			++it;
			continue;
		}

		if ( bFire && it->second.nTouchCount > 0 && ShouldTouchCallback( it->first.first, it->first.second ) )
		{
			CBox3DCollisionData data;
			pObject->GetPosition( &data.m_Point, NULL );
			m_pCollisionEvents->EndTouch( it->first.first, it->first.second, &data );
		}
		m_Pairs.erase( it++ );
	}

	for ( std::map< ObjectPair_t, int >::iterator it = m_TriggerOverlaps.begin(); it != m_TriggerOverlaps.end(); )
	{
		CBox3DPhysicsObject *pTrigger = it->first.first;
		CBox3DPhysicsObject *pVisitor = it->first.second;
		if ( pTrigger != pObject && pVisitor != pObject )
		{
			++it;
			continue;
		}

		CBox3DFluidController *pFluid = pTrigger->GetFluidControllerInternal();
		if ( pFluid )
		{
			pFluid->RemoveObject( pVisitor );
			if ( bFire && ( pVisitor->GetCallbackFlags() & CALLBACK_FLUID_TOUCH ) )
				m_pCollisionEvents->FluidEndTouch( pVisitor, pFluid );
		}
		else if ( bFire )
		{
			m_pCollisionEvents->ObjectLeaveTrigger( pTrigger, pVisitor );
		}
		m_TriggerOverlaps.erase( it++ );
	}
}

bool CBox3DPhysicsEnvironment::IsInSimulation() const
{
	return m_bSimulating;
}

float CBox3DPhysicsEnvironment::GetSimulationTimestep() const
{
	return m_flTimestep;
}

void CBox3DPhysicsEnvironment::SetSimulationTimestep( float timestep )
{
	m_flTimestep = MAX( timestep, 0.001f );
}

float CBox3DPhysicsEnvironment::GetSimulationTime() const
{
	return m_flSimTime;
}

void CBox3DPhysicsEnvironment::ResetSimulationClock()
{
	m_flSimTime = 0.0f;
	m_flAccumulator = 0.0f;
}

float CBox3DPhysicsEnvironment::GetNextFrameTime( void ) const
{
	return m_flSimTime + MAX( m_flTimestep - m_flAccumulator, 0.0f );
}

void CBox3DPhysicsEnvironment::SetCollisionEventHandler( IPhysicsCollisionEvent *pCollisionEvents )
{
	m_pCollisionEvents = pCollisionEvents;
}

void CBox3DPhysicsEnvironment::SetObjectEventHandler( IPhysicsObjectEvent *pObjectEvents )
{
	m_pObjectEvents = pObjectEvents;
}

void CBox3DPhysicsEnvironment::SetConstraintEventHandler( IPhysicsConstraintEvent *pConstraintEvents )
{
	m_pConstraintEvents = pConstraintEvents;
}

void CBox3DPhysicsEnvironment::SetQuickDelete( bool bQuick )
{
	m_bQuickDelete = bQuick;
}

int CBox3DPhysicsEnvironment::GetActiveObjectCount() const
{
	return (int)m_ActiveObjects.size();
}

void CBox3DPhysicsEnvironment::GetActiveObjects( IPhysicsObject **pOutputObjectList ) const
{
	for ( size_t i = 0; i < m_ActiveObjects.size(); i++ )
		pOutputObjectList[i] = m_ActiveObjects[i];
}

const IPhysicsObject **CBox3DPhysicsEnvironment::GetObjectList( int *pOutputObjectCount ) const
{
	m_ObjectListCache.resize( m_Objects.size() );
	for ( size_t i = 0; i < m_Objects.size(); i++ )
		m_ObjectListCache[i] = m_Objects[i];

	if ( pOutputObjectCount )
		*pOutputObjectCount = (int)m_ObjectListCache.size();

	return m_ObjectListCache.empty() ? NULL : (const IPhysicsObject **)&m_ObjectListCache[0];
}

bool CBox3DPhysicsEnvironment::TransferObject( IPhysicsObject *pObject, IPhysicsEnvironment *pDestinationEnvironment )
{
	// Box3D bodies belong to one world and the game keeps the object pointer, so this would
	// need a body rebuild behind the same object. Only Portal style simulators use it.
	B3VP_Unsupported( "IPhysicsEnvironment::TransferObject" );
	return false;
}

//-------------------------------------------------------------------------------------------------
// Save / restore
//-------------------------------------------------------------------------------------------------

static void WriteRecord( ISave *pSave, const void *pData, int size )
{
	pSave->WriteData( (const char *)pData, size );
}

static bool ReadRecord( IRestore *pRestore, void *pData, int size )
{
	return pRestore->ReadData( (char *)pData, size, 0 ) == size;
}

bool CBox3DPhysicsEnvironment::Save( const physsaveparams_t &params )
{
	if ( !params.pSave || !params.pObject )
		return false;

	B3VPSaveHeader_t header;
	header.magic = B3VP_SAVE_MAGIC;
	header.type = params.type;
	header.pOld = params.pObject;

	switch ( params.type )
	{
	case PIID_IPHYSICSOBJECT:
		{
			CBox3DPhysicsObject *pObject = static_cast< CBox3DPhysicsObject * >( (IPhysicsObject *)params.pObject );
			B3VPObjectSaveData_t data;
			pObject->WriteSaveData( data );
			WriteRecord( params.pSave, &header, sizeof( header ) );
			WriteRecord( params.pSave, &data, sizeof( data ) );
			return true;
		}

	case PIID_IPHYSICSCONSTRAINT:
		{
			CBox3DConstraint *pConstraint = static_cast< CBox3DConstraint * >( (IPhysicsConstraint *)params.pObject );
			B3VPConstraintSave_t data;
			memset( &data, 0, sizeof( data ) );
			data.pRef = pConstraint->GetReferenceObject();
			data.pAtt = pConstraint->GetAttachedObject();
			data.pGroup = pConstraint->GetGroup() ? (IPhysicsConstraintGroup *)pConstraint->GetGroup() : NULL;
			data.bActive = pConstraint->IsActive();
			data.desc = pConstraint->GetDesc();
			WriteRecord( params.pSave, &header, sizeof( header ) );
			WriteRecord( params.pSave, &data, sizeof( data ) );
			return true;
		}

	case PIID_IPHYSICSCONSTRAINTGROUP:
		{
			CBox3DConstraintGroup *pGroup = static_cast< CBox3DConstraintGroup * >( (IPhysicsConstraintGroup *)params.pObject );
			constraint_groupparams_t groupParams = pGroup->GetParams();
			WriteRecord( params.pSave, &header, sizeof( header ) );
			WriteRecord( params.pSave, &groupParams, sizeof( groupParams ) );
			return true;
		}

	case PIID_IPHYSICSSPRING:
		{
			CBox3DSpring *pSpring = static_cast< CBox3DSpring * >( (IPhysicsSpring *)params.pObject );
			B3VPSpringSave_t data;
			memset( &data, 0, sizeof( data ) );
			data.pStart = pSpring->GetStartObject();
			data.pEnd = pSpring->GetEndObject();
			pSpring->GetSaveData( data.data );
			WriteRecord( params.pSave, &header, sizeof( header ) );
			WriteRecord( params.pSave, &data, sizeof( data ) );
			return true;
		}

	case PIID_IPHYSICSMOTIONCONTROLLER:
		{
			CBox3DMotionController *pMotion = static_cast< CBox3DMotionController * >( (IPhysicsMotionController *)params.pObject );
			B3VPMotionSave_t data;
			data.priority = pMotion->GetPriority();
			data.count = pMotion->CountObjects();
			CUtlVector< IPhysicsObject * > objects;
			objects.SetCount( data.count );
			if ( data.count )
				pMotion->GetObjects( objects.Base() );
			WriteRecord( params.pSave, &header, sizeof( header ) );
			WriteRecord( params.pSave, &data, sizeof( data ) );
			for ( int i = 0; i < data.count; i++ )
			{
				void *pPtr = objects[i];
				WriteRecord( params.pSave, &pPtr, sizeof( pPtr ) );
			}
			return true;
		}

	// Shadows are saved inline with their object. Fluids, players and vehicles are rebuilt by the
	// game (IVP doesn't save fluids or player controllers either).
	default:
		return false;
	}
}

void CBox3DPhysicsEnvironment::PreRestore( const physprerestoreparams_t &params )
{
	m_RestorePointers.clear();
	for ( int i = 0; i < params.recreatedObjectCount; i++ )
		AddSavedPointer( params.recreatedObjectList[i].pOldObject, params.recreatedObjectList[i].pNewObject );
}

void *CBox3DPhysicsEnvironment::RemapSavedPointer( void *pOld ) const
{
	if ( !pOld )
		return NULL;
	std::map< void *, void * >::const_iterator it = m_RestorePointers.find( pOld );
	return it != m_RestorePointers.end() ? it->second : NULL;
}

void CBox3DPhysicsEnvironment::AddSavedPointer( void *pOld, void *pNew )
{
	if ( pOld && pNew )
		m_RestorePointers[pOld] = pNew;
}

void CBox3DPhysicsEnvironment::QueuePostRestore( CBox3DConstraint *pConstraint )
{
	m_PendingRestoreConstraints.push_back( pConstraint );
}

// Pending restores for types that reference objects
struct B3VPPendingSpring_t { CBox3DSpring *pSpring; };
struct B3VPPendingMotion_t { CBox3DMotionController *pMotion; };
static std::vector< CBox3DSpring * > s_PendingSprings;
static std::vector< CBox3DMotionController * > s_PendingMotion;

bool CBox3DPhysicsEnvironment::Restore( const physrestoreparams_t &params )
{
	if ( !params.pRestore || !params.ppObject )
		return false;

	*params.ppObject = NULL;

	B3VPSaveHeader_t header;
	if ( !ReadRecord( params.pRestore, &header, sizeof( header ) ) || header.magic != B3VP_SAVE_MAGIC || header.type != params.type )
	{
		Warning( B3VP_LOG_PREFIX "Save data for a physics object isn't from the Box3D backend, it can't be restored.\n" );
		return false;
	}

	switch ( params.type )
	{
	case PIID_IPHYSICSOBJECT:
		{
			B3VPObjectSaveData_t data;
			if ( !ReadRecord( params.pRestore, &data, sizeof( data ) ) || data.version != B3VP_OBJECT_SAVE_VERSION )
				return false;

			objectparams_t objectParams;
			memset( &objectParams, 0, sizeof( objectParams ) );
			objectParams.mass = data.mass;
			objectParams.inertia = 1.0f;
			objectParams.damping = data.damping;
			objectParams.rotdamping = data.rotDamping;
			objectParams.dragCoefficient = data.dragCoefficient;
			objectParams.volume = data.volume;
			objectParams.pGameData = params.pGameData;
			objectParams.pName = params.pName ? params.pName : data.name;
			objectParams.enableCollisions = true;

			const int materialIndex = MAX( 0, CBox3DPhysicsSurfaceProps::GetInstance().GetSurfaceIndex( data.materialName ) );

			IPhysicsObject *pObject = NULL;
			if ( params.pCollisionModel )
			{
				pObject = data.bStatic
					? CreatePolyObjectStatic( params.pCollisionModel, materialIndex, data.position, data.angles, &objectParams )
					: CreatePolyObject( params.pCollisionModel, materialIndex, data.position, data.angles, &objectParams );
			}
			else if ( data.sphereRadius > 0.0f )
			{
				pObject = CreateSphereObject( data.sphereRadius, materialIndex, data.position, data.angles, &objectParams, data.bStatic );
			}

			if ( !pObject )
				return false;

			static_cast< CBox3DPhysicsObject * >( pObject )->ApplySaveData( data );
			*params.ppObject = pObject;
			AddSavedPointer( header.pOld, pObject );
			return true;
		}

	case PIID_IPHYSICSCONSTRAINT:
		{
			B3VPConstraintSave_t data;
			if ( !ReadRecord( params.pRestore, &data, sizeof( data ) ) )
				return false;

			CBox3DConstraint *pConstraint = new CBox3DConstraint( this, NULL, NULL );
			pConstraint->SetGameData( params.pGameData );
			pConstraint->m_pSavedRef = data.pRef;
			pConstraint->m_pSavedAtt = data.pAtt;
			pConstraint->m_pSavedGroup = data.pGroup;
			pConstraint->m_bSavedActive = data.bActive;
			pConstraint->InitFromDesc( data.desc, NULL, false );
			QueuePostRestore( pConstraint );

			*params.ppObject = static_cast< IPhysicsConstraint * >( pConstraint );
			AddSavedPointer( header.pOld, *params.ppObject );
			return true;
		}

	case PIID_IPHYSICSCONSTRAINTGROUP:
		{
			constraint_groupparams_t groupParams;
			if ( !ReadRecord( params.pRestore, &groupParams, sizeof( groupParams ) ) )
				return false;

			CBox3DConstraintGroup *pGroup = new CBox3DConstraintGroup( groupParams );
			*params.ppObject = static_cast< IPhysicsConstraintGroup * >( pGroup );
			AddSavedPointer( header.pOld, *params.ppObject );
			return true;
		}

	case PIID_IPHYSICSSPRING:
		{
			B3VPSpringSave_t data;
			if ( !ReadRecord( params.pRestore, &data, sizeof( data ) ) )
				return false;

			CBox3DSpring *pSpring = new CBox3DSpring( this, data.data );
			pSpring->m_pSavedStart = data.pStart;
			pSpring->m_pSavedEnd = data.pEnd;
			s_PendingSprings.push_back( pSpring );

			*params.ppObject = static_cast< IPhysicsSpring * >( pSpring );
			AddSavedPointer( header.pOld, *params.ppObject );
			return true;
		}

	case PIID_IPHYSICSMOTIONCONTROLLER:
		{
			B3VPMotionSave_t data;
			if ( !ReadRecord( params.pRestore, &data, sizeof( data ) ) || data.count < 0 || data.count > 4096 )
				return false;

			CBox3DMotionController *pMotion = static_cast< CBox3DMotionController * >( CreateMotionController( NULL ) );
			pMotion->SetPriority( (IPhysicsMotionController::priority_t)data.priority );
			for ( int i = 0; i < data.count; i++ )
			{
				void *pPtr = NULL;
				ReadRecord( params.pRestore, &pPtr, sizeof( pPtr ) );
				pMotion->m_SavedObjects.AddToTail( pPtr );
			}
			s_PendingMotion.push_back( pMotion );

			*params.ppObject = static_cast< IPhysicsMotionController * >( pMotion );
			AddSavedPointer( header.pOld, *params.ppObject );
			return true;
		}

	default:
		return false;
	}
}

void CBox3DPhysicsEnvironment::PostRestore()
{
	for ( size_t i = 0; i < m_PendingRestoreConstraints.size(); i++ )
	{
		CBox3DConstraint *pConstraint = m_PendingRestoreConstraints[i];
		CBox3DPhysicsObject *pRef = static_cast< CBox3DPhysicsObject * >( (IPhysicsObject *)RemapSavedPointer( pConstraint->m_pSavedRef ) );
		CBox3DPhysicsObject *pAtt = static_cast< CBox3DPhysicsObject * >( (IPhysicsObject *)RemapSavedPointer( pConstraint->m_pSavedAtt ) );
		CBox3DConstraintGroup *pGroup = static_cast< CBox3DConstraintGroup * >( (IPhysicsConstraintGroup *)RemapSavedPointer( pConstraint->m_pSavedGroup ) );

		if ( !pRef || !pAtt )
		{
			DevWarning( B3VP_LOG_PREFIX "Restored constraint lost one of its objects.\n" );
			continue;
		}

		pConstraint->BindObjects( pRef, pAtt );
		pConstraint->InitFromDesc( pConstraint->GetDesc(), pGroup, pConstraint->m_bSavedActive );
	}
	m_PendingRestoreConstraints.clear();

	for ( size_t i = 0; i < s_PendingSprings.size(); i++ )
	{
		CBox3DSpring *pSpring = s_PendingSprings[i];
		pSpring->BindObjects( static_cast< CBox3DPhysicsObject * >( (IPhysicsObject *)RemapSavedPointer( pSpring->m_pSavedStart ) ),
							  static_cast< CBox3DPhysicsObject * >( (IPhysicsObject *)RemapSavedPointer( pSpring->m_pSavedEnd ) ) );
	}
	s_PendingSprings.clear();

	for ( size_t i = 0; i < s_PendingMotion.size(); i++ )
	{
		CBox3DMotionController *pMotion = s_PendingMotion[i];
		for ( int j = 0; j < pMotion->m_SavedObjects.Count(); j++ )
		{
			IPhysicsObject *pObject = (IPhysicsObject *)RemapSavedPointer( pMotion->m_SavedObjects[j] );
			if ( pObject )
				pMotion->AttachObject( pObject, true );
		}
		pMotion->m_SavedObjects.RemoveAll();
	}
	s_PendingMotion.clear();

	m_RestorePointers.clear();
}

bool CBox3DPhysicsEnvironment::IsCollisionModelUsed( CPhysCollide *pCollide ) const
{
	for ( size_t i = 0; i < m_Objects.size(); i++ )
	{
		if ( m_Objects[i]->GetCollide() == pCollide )
			return true;
	}
	return false;
}

//-------------------------------------------------------------------------------------------------
// Queries (no-ops in IVP too, the engine traces collides directly)
//-------------------------------------------------------------------------------------------------

void CBox3DPhysicsEnvironment::TraceRay( const Ray_t &ray, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace )
{
	if ( pTrace )
		B3VP_ClearTrace( pTrace );
}

void CBox3DPhysicsEnvironment::SweepCollideable( const CPhysCollide *pCollide, const Vector &vecAbsStart, const Vector &vecAbsEnd,
												 const QAngle &vecAngles, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace )
{
	if ( pTrace )
		B3VP_ClearTrace( pTrace );
}

//-------------------------------------------------------------------------------------------------
// Performance / stats
//-------------------------------------------------------------------------------------------------

void CBox3DPhysicsEnvironment::GetPerformanceSettings( physics_performanceparams_t *pOutput ) const
{
	if ( pOutput )
		*pOutput = m_PerformanceParams;
}

void CBox3DPhysicsEnvironment::SetPerformanceSettings( const physics_performanceparams_t *pSettings )
{
	if ( !pSettings )
		return;

	m_PerformanceParams = *pSettings;
	m_PerformanceParams.minFrictionMass = clamp( m_PerformanceParams.minFrictionMass, 1.0f, VPHYSICS_MAX_MASS );
	m_PerformanceParams.maxFrictionMass = clamp( m_PerformanceParams.maxFrictionMass, 1.0f, VPHYSICS_MAX_MASS );
	b3World_SetMaximumLinearSpeed( m_World, HL2B3( MAX( m_PerformanceParams.maxVelocity, 1.0f ) ) );
}

void CBox3DPhysicsEnvironment::ReadStats( physics_stats_t *pOutput )
{
	if ( !pOutput )
		return;

	memset( pOutput, 0, sizeof( *pOutput ) );
	const b3Counters counters = b3World_GetCounters( m_World );
	pOutput->collisionPairsTotal = counters.contactCount;
}

void CBox3DPhysicsEnvironment::ClearStats()
{
}

void CBox3DPhysicsEnvironment::DebugCheckContacts( void )
{
}

//-------------------------------------------------------------------------------------------------
// Object serialization (used for entity transitions)
//-------------------------------------------------------------------------------------------------

struct B3VPSerializedObject_t
{
	const CPhysCollide		*pCollide;
	B3VPObjectSaveData_t	data;
};

unsigned int CBox3DPhysicsEnvironment::GetObjectSerializeSize( IPhysicsObject *pObject ) const
{
	return sizeof( B3VPSerializedObject_t );
}

void CBox3DPhysicsEnvironment::SerializeObjectToBuffer( IPhysicsObject *pObject, unsigned char *pBuffer, unsigned int bufferSize )
{
	if ( !pObject || !pBuffer || bufferSize < sizeof( B3VPSerializedObject_t ) )
		return;

	B3VPSerializedObject_t serialized;
	serialized.pCollide = pObject->GetCollide();
	static_cast< CBox3DPhysicsObject * >( pObject )->WriteSaveData( serialized.data );
	memcpy( pBuffer, &serialized, sizeof( serialized ) );
}

IPhysicsObject *CBox3DPhysicsEnvironment::UnserializeObjectFromBuffer( void *pGameData, unsigned char *pBuffer, unsigned int bufferSize, bool enableCollisions )
{
	if ( !pBuffer || bufferSize < sizeof( B3VPSerializedObject_t ) )
		return NULL;

	B3VPSerializedObject_t serialized;
	memcpy( &serialized, pBuffer, sizeof( serialized ) );

	objectparams_t params;
	memset( &params, 0, sizeof( params ) );
	params.mass = serialized.data.mass;
	params.inertia = 1.0f;
	params.damping = serialized.data.damping;
	params.rotdamping = serialized.data.rotDamping;
	params.dragCoefficient = serialized.data.dragCoefficient;
	params.volume = serialized.data.volume;
	params.pGameData = pGameData;
	params.pName = serialized.data.name;
	params.enableCollisions = enableCollisions;

	const int materialIndex = MAX( 0, CBox3DPhysicsSurfaceProps::GetInstance().GetSurfaceIndex( serialized.data.materialName ) );

	IPhysicsObject *pObject = NULL;
	if ( serialized.pCollide )
	{
		pObject = serialized.data.bStatic
			? CreatePolyObjectStatic( serialized.pCollide, materialIndex, serialized.data.position, serialized.data.angles, &params )
			: CreatePolyObject( serialized.pCollide, materialIndex, serialized.data.position, serialized.data.angles, &params );
	}
	else if ( serialized.data.sphereRadius > 0.0f )
	{
		pObject = CreateSphereObject( serialized.data.sphereRadius, materialIndex, serialized.data.position, serialized.data.angles, &params, serialized.data.bStatic );
	}

	if ( pObject )
	{
		serialized.data.bCollisionsEnabled = enableCollisions;
		static_cast< CBox3DPhysicsObject * >( pObject )->ApplySaveData( serialized.data );
	}
	return pObject;
}
