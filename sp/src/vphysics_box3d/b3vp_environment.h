//=================================================================================================
//
// VPhysics Box3D - IPhysicsEnvironment
//
//=================================================================================================

#ifndef B3VP_ENVIRONMENT_H
#define B3VP_ENVIRONMENT_H

#ifdef _WIN32
#pragma once
#endif

#include "b3vp_base.h"
#include "b3vp_object.h"

class CBox3DConstraint;
class CBox3DConstraintGroup;

// Anything that runs logic around each simulation step
class IBox3DController
{
public:
	virtual ~IBox3DController() {}
	virtual void OnPreStep( float dt ) = 0;
	virtual void OnPostStep( float dt ) {}
};

// Collision data passed to the game for collisions, touches and friction
class CBox3DCollisionData : public IPhysicsCollisionData
{
public:
	CBox3DCollisionData() : m_Normal( 0, 0, 1 ), m_Point( 0, 0, 0 ), m_Speed( 0, 0, 0 ) {}

	void GetSurfaceNormal( Vector &out )	{ out = m_Normal; }
	void GetContactPoint( Vector &out )		{ out = m_Point; }
	void GetContactSpeed( Vector &out )		{ out = m_Speed; }

	Vector m_Normal;	// points toward object 1
	Vector m_Point;		// world, inches
	Vector m_Speed;		// speed of object 1 relative to object 0 at the contact, in/s
};

class CBox3DPhysicsEnvironment : public IPhysicsEnvironment
{
public:
	CBox3DPhysicsEnvironment();
	virtual ~CBox3DPhysicsEnvironment();

	// IPhysicsEnvironment
	void					SetDebugOverlay( CreateInterfaceFn debugOverlayFactory );
	IVPhysicsDebugOverlay	*GetDebugOverlay( void );

	void					SetGravity( const Vector &gravityVector );
	void					GetGravity( Vector *pGravityVector ) const;
	void					SetAirDensity( float density );
	float					GetAirDensity( void ) const;

	IPhysicsObject			*CreatePolyObject( const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams );
	IPhysicsObject			*CreatePolyObjectStatic( const CPhysCollide *pCollisionModel, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams );
	IPhysicsObject			*CreateSphereObject( float radius, int materialIndex, const Vector &position, const QAngle &angles, objectparams_t *pParams, bool isStatic );
	void					DestroyObject( IPhysicsObject * );

	IPhysicsFluidController	*CreateFluidController( IPhysicsObject *pFluidObject, fluidparams_t *pParams );
	void					DestroyFluidController( IPhysicsFluidController * );

	IPhysicsSpring			*CreateSpring( IPhysicsObject *pObjectStart, IPhysicsObject *pObjectEnd, springparams_t *pParams );
	void					DestroySpring( IPhysicsSpring * );

	IPhysicsConstraint		*CreateRagdollConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll );
	IPhysicsConstraint		*CreateHingeConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_hingeparams_t &hinge );
	IPhysicsConstraint		*CreateFixedConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_fixedparams_t &fixed );
	IPhysicsConstraint		*CreateSlidingConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_slidingparams_t &sliding );
	IPhysicsConstraint		*CreateBallsocketConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket );
	IPhysicsConstraint		*CreatePulleyConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley );
	IPhysicsConstraint		*CreateLengthConstraint( IPhysicsObject *pReferenceObject, IPhysicsObject *pAttachedObject, IPhysicsConstraintGroup *pGroup, const constraint_lengthparams_t &length );
	void					DestroyConstraint( IPhysicsConstraint * );

	IPhysicsConstraintGroup	*CreateConstraintGroup( const constraint_groupparams_t &groupParams );
	void					DestroyConstraintGroup( IPhysicsConstraintGroup *pGroup );

	IPhysicsShadowController *CreateShadowController( IPhysicsObject *pObject, bool allowTranslation, bool allowRotation );
	void					DestroyShadowController( IPhysicsShadowController * );

	IPhysicsPlayerController *CreatePlayerController( IPhysicsObject *pObject );
	void					DestroyPlayerController( IPhysicsPlayerController * );

	IPhysicsMotionController *CreateMotionController( IMotionEvent *pHandler );
	void					DestroyMotionController( IPhysicsMotionController *pController );

	IPhysicsVehicleController *CreateVehicleController( IPhysicsObject *pVehicleBodyObject, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace );
	void					DestroyVehicleController( IPhysicsVehicleController * );

	void					SetCollisionSolver( IPhysicsCollisionSolver *pSolver );

	void					Simulate( float deltaTime );
	bool					IsInSimulation() const;

	float					GetSimulationTimestep() const;
	void					SetSimulationTimestep( float timestep );
	float					GetSimulationTime() const;
	void					ResetSimulationClock();
	float					GetNextFrameTime( void ) const;

	void					SetCollisionEventHandler( IPhysicsCollisionEvent *pCollisionEvents );
	void					SetObjectEventHandler( IPhysicsObjectEvent *pObjectEvents );
	void					SetConstraintEventHandler( IPhysicsConstraintEvent *pConstraintEvents );

	void					SetQuickDelete( bool bQuick );

	int						GetActiveObjectCount() const;
	void					GetActiveObjects( IPhysicsObject **pOutputObjectList ) const;
	const IPhysicsObject	**GetObjectList( int *pOutputObjectCount ) const;
	bool					TransferObject( IPhysicsObject *pObject, IPhysicsEnvironment *pDestinationEnvironment );

	void					CleanupDeleteList( void );
	void					EnableDeleteQueue( bool enable );

	bool					Save( const physsaveparams_t &params );
	void					PreRestore( const physprerestoreparams_t &params );
	bool					Restore( const physrestoreparams_t &params );
	void					PostRestore();

	bool					IsCollisionModelUsed( CPhysCollide *pCollide ) const;

	void					TraceRay( const Ray_t &ray, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace );
	void					SweepCollideable( const CPhysCollide *pCollide, const Vector &vecAbsStart, const Vector &vecAbsEnd,
											  const QAngle &vecAngles, unsigned int fMask, IPhysicsTraceFilter *pTraceFilter, trace_t *pTrace );

	void					GetPerformanceSettings( physics_performanceparams_t *pOutput ) const;
	void					SetPerformanceSettings( const physics_performanceparams_t *pSettings );

	void					ReadStats( physics_stats_t *pOutput );
	void					ClearStats();

	unsigned int			GetObjectSerializeSize( IPhysicsObject *pObject ) const;
	void					SerializeObjectToBuffer( IPhysicsObject *pObject, unsigned char *pBuffer, unsigned int bufferSize );
	IPhysicsObject			*UnserializeObjectFromBuffer( void *pGameData, unsigned char *pBuffer, unsigned int bufferSize, bool enableCollisions );

	void					EnableConstraintNotify( bool bEnable );
	void					DebugCheckContacts( void );

public:
	// Internal interface
	b3WorldId				GetWorld() const							{ return m_World; }
	b3BodyId				GetAnchorBody() const						{ return m_AnchorBody; }
	IPhysicsCollisionEvent	*GetCollisionEventHandler() const			{ return m_pCollisionEvents; }
	IPhysicsCollisionSolver	*GetCollisionSolver() const					{ return m_pCollisionSolver; }
	bool					HasSimulated() const						{ return m_bHasSimulated; }
	float					GetMaxLinearSpeed() const;					// m/s
	float					GetMaxAngularSpeed() const;					// rad/s

	void					AddController( IBox3DController *pController );
	void					RemoveController( IBox3DController *pController );

	// Called when an object's shapes go away (destroyed or rebuilt). Clears touch/trigger bookkeeping.
	void					OnObjectShapesRemoved( CBox3DPhysicsObject *pObject, bool bFireCallbacks );

	// Breakable constraints report through here
	void					NotifyConstraintBroken( CBox3DConstraint *pConstraint );
	void					RegisterJoint( b3JointId joint, CBox3DConstraint *pConstraint );
	void					UnregisterJoint( b3JointId joint );

	// Save/restore pointer remapping
	void					*RemapSavedPointer( void *pOld ) const;
	void					AddSavedPointer( void *pOld, void *pNew );
	void					QueuePostRestore( CBox3DConstraint *pConstraint );

	bool					ShouldCollide( CBox3DPhysicsObject *pObject0, CBox3DPhysicsObject *pObject1 );
	bool					PreSolve( CBox3DPhysicsObject *pObject0, CBox3DPhysicsObject *pObject1 );

	int						GetFrameCounter() const						{ return m_nFrameCounter; }

private:
	void					StepOnce( float dt );
	void					ProcessMoveEvents();
	void					ProcessSensorEvents();
	void					ProcessContactEvents();
	void					ProcessJointEvents();
	void					ProcessFriction( float dt );
	void					ProcessSleepEvents();
	void					DeleteDeadObjects();
	void					RemoveObjectNow( CBox3DPhysicsObject *pObject );

	typedef std::pair< CBox3DPhysicsObject *, CBox3DPhysicsObject * > ObjectPair_t;
	static ObjectPair_t		MakePair( CBox3DPhysicsObject *a, CBox3DPhysicsObject *b ) { return a < b ? ObjectPair_t( a, b ) : ObjectPair_t( b, a ); }

	struct PairState_t
	{
		PairState_t() : nTouchCount( 0 ), flLastCollisionTime( -1000.0f ) {}
		int		nTouchCount;
		float	flLastCollisionTime;
	};

	b3WorldId									m_World;
	b3BodyId									m_AnchorBody;

	std::vector< CBox3DPhysicsObject * >		m_Objects;
	std::vector< CBox3DPhysicsObject * >		m_DeadObjects;
	std::vector< CBox3DConstraint * >			m_DeadConstraints;
	std::vector< IBox3DController * >			m_Controllers;
	std::vector< CBox3DConstraint * >			m_PendingRestoreConstraints;

	mutable std::vector< IPhysicsObject * >		m_ObjectListCache;
	std::vector< CBox3DPhysicsObject * >		m_ActiveObjects;

	std::map< ObjectPair_t, PairState_t >		m_Pairs;			// contact bookkeeping (touch counts, last impact)
	std::map< ObjectPair_t, int >				m_TriggerOverlaps;	// first = trigger, second = visitor (not sorted)
	std::map< uint64, CBox3DConstraint * >		m_Joints;			// for joint events
	std::map< void *, void * >					m_RestorePointers;

	IPhysicsCollisionEvent						*m_pCollisionEvents;
	IPhysicsObjectEvent							*m_pObjectEvents;
	IPhysicsConstraintEvent						*m_pConstraintEvents;
	IPhysicsCollisionSolver						*m_pCollisionSolver;
	IVPhysicsDebugOverlay						*m_pDebugOverlay;

	physics_performanceparams_t					m_PerformanceParams;
	Vector										m_Gravity;
	float										m_flAirDensity;
	float										m_flTimestep;
	float										m_flSimTime;
	float										m_flAccumulator;
	int											m_nFrameCounter;

	bool										m_bSimulating;
	bool										m_bDeleteQueue;
	bool										m_bQuickDelete;
	bool										m_bConstraintNotify;
	bool										m_bHasSimulated;
};

// Joint ids as map keys
inline uint64 B3VP_JointKey( b3JointId id )
{
	return ( (uint64)(uint32)id.index1 ) | ( (uint64)id.world0 << 32 ) | ( (uint64)id.generation << 48 );
}

#endif // B3VP_ENVIRONMENT_H
