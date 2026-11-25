//=================================================================================================
//
// VPhysics Box3D - shadow, player, motion, fluid and vehicle controllers, friction snapshots
//
//=================================================================================================

#ifndef B3VP_CONTROLLERS_H
#define B3VP_CONTROLLERS_H

#ifdef _WIN32
#pragma once
#endif

#include "b3vp_base.h"
#include "b3vp_object.h"
#include "b3vp_environment.h"

//-------------------------------------------------------------------------------------------------
// Shadow controller: drives an object toward a target transform (NPCs, doors, trains, etc.)
//
// When the object may not be moved by physics it becomes a kinematic body that follows the
// target, pushing dynamic objects out of the way. When physics movement is allowed it stays
// dynamic and the controller steers its velocity, like IVP.
//-------------------------------------------------------------------------------------------------
class CBox3DShadowController : public IPhysicsShadowController, public IBox3DController
{
public:
	CBox3DShadowController( CBox3DPhysicsObject *pObject, bool allowTranslation, bool allowRotation );
	~CBox3DShadowController();

	void	Update( const Vector &position, const QAngle &angles, float timeOffset );
	void	MaxSpeed( float maxSpeed, float maxAngularSpeed );
	void	StepUp( float height );
	void	SetTeleportDistance( float teleportDistance );
	bool	AllowsTranslation()								{ return m_bAllowTranslation; }
	bool	AllowsRotation()								{ return m_bAllowRotation; }
	void	SetPhysicallyControlled( bool isPhysicallyControlled );
	bool	IsPhysicallyControlled()						{ return m_bPhysicallyControlled; }
	void	GetLastImpulse( Vector *pOut )					{ if ( pOut ) *pOut = m_LastImpulse; }
	void	UseShadowMaterial( bool bUseShadowMaterial );
	void	ObjectMaterialChanged( int materialIndex );
	float	GetTargetPosition( Vector *pPositionOut, QAngle *pAnglesOut );
	float	GetTeleportDistance( void )						{ return m_flTeleportDistance; }
	void	GetMaxSpeed( float *pMaxSpeedOut, float *pMaxAngularSpeedOut );

	void	OnPreStep( float dt );

	// True when the body is a kinematic follower rather than a steered dynamic body
	bool	IsKinematic() const								{ return !m_bAllowTranslation; }
	CBox3DPhysicsObject *GetObject() const					{ return m_pObject; }
	void	DetachObject();

	// Save/restore
	float	GetSecondsToArrival() const						{ return m_flSecondsToArrival; }
	void	SetSecondsToArrival( float t )					{ m_flSecondsToArrival = t; }

private:
	CBox3DPhysicsObject	*m_pObject;
	Vector				m_TargetPosition;
	QAngle				m_TargetAngles;
	Vector				m_LastImpulse;
	float				m_flSecondsToArrival;
	float				m_flMaxSpeed;
	float				m_flMaxAngular;
	float				m_flTeleportDistance;
	int					m_nSavedMaterial;
	unsigned short		m_nSavedCallbackFlags;
	bool				m_bAllowTranslation;
	bool				m_bAllowRotation;
	bool				m_bPhysicallyControlled;
	bool				m_bEnabled;
};

//-------------------------------------------------------------------------------------------------
// Player controller: the player's physics shadow. Kinematic, follows the position the game's
// movement code computed, pushing light objects and blocked by nothing (game traces already
// handled the world).
//-------------------------------------------------------------------------------------------------
class CBox3DPlayerController : public IPhysicsPlayerController, public IBox3DController, public IBox3DObjectListener
{
public:
	CBox3DPlayerController( CBox3DPhysicsObject *pObject );
	~CBox3DPlayerController();

	void	Update( const Vector &position, const Vector &velocity, float secondsToArrival, bool onground, IPhysicsObject *ground );
	void	SetEventHandler( IPhysicsPlayerControllerEvent *handler )	{ m_pHandler = handler; }
	bool	IsInContact( void );
	void	MaxSpeed( const Vector &maxVelocity );
	void	SetObject( IPhysicsObject *pObject );
	int		GetShadowPosition( Vector *position, QAngle *angles );
	void	StepUp( float height );
	void	Jump();
	void	GetShadowVelocity( Vector *velocity );
	IPhysicsObject *GetObject()											{ return m_pObject; }
	void	GetLastImpulse( Vector *pOut )								{ if ( pOut ) *pOut = m_LastImpulse; }
	void	SetPushMassLimit( float maxPushMass )						{ m_flPushMassLimit = maxPushMass; }
	void	SetPushSpeedLimit( float maxPushSpeed )						{ m_flPushSpeedLimit = maxPushSpeed; }
	float	GetPushMassLimit()											{ return m_flPushMassLimit; }
	float	GetPushSpeedLimit()											{ return m_flPushSpeedLimit; }
	bool	WasFrozen()													{ bool b = m_bWasFrozen; m_bWasFrozen = false; return b; }

	void	OnPreStep( float dt );
	void	OnPostStep( float dt );
	void	OnObjectDestroyed( CBox3DPhysicsObject *pObject );

private:
	void	AttachObject( CBox3DPhysicsObject *pObject );
	void	DetachObject();

	CBox3DPhysicsObject				*m_pObject;
	IPhysicsPlayerControllerEvent	*m_pHandler;
	Vector							m_TargetPosition;
	Vector							m_TargetVelocity;
	Vector							m_MaxSpeed;
	Vector							m_LastImpulse;
	Vector							m_LastPosition;
	float							m_flSecondsToArrival;
	float							m_flPushMassLimit;
	float							m_flPushSpeedLimit;
	bool							m_bOnGround;
	bool							m_bWasFrozen;
	bool							m_bEnabled;
};

//-------------------------------------------------------------------------------------------------
// Motion controller: calls IMotionEvent::Simulate for attached objects every step
//-------------------------------------------------------------------------------------------------
class CBox3DMotionController : public IPhysicsMotionController, public IBox3DController, public IBox3DObjectListener
{
public:
	CBox3DMotionController( CBox3DPhysicsEnvironment *pEnv, IMotionEvent *pHandler );
	~CBox3DMotionController();

	void	SetEventHandler( IMotionEvent *handler )			{ m_pHandler = handler; }
	void	AttachObject( IPhysicsObject *pObject, bool checkIfAlreadyAttached );
	void	DetachObject( IPhysicsObject *pObject );
	int		CountObjects( void )								{ return m_Objects.Count(); }
	void	GetObjects( IPhysicsObject **pObjectList );
	void	ClearObjects( void );
	void	WakeObjects( void );
	void	SetPriority( priority_t priority )					{ m_Priority = priority; }
	priority_t GetPriority() const								{ return m_Priority; }

	void	OnPreStep( float dt );
	void	OnObjectDestroyed( CBox3DPhysicsObject *pObject );

	// Restore bookkeeping: old object pointers to attach after everything is restored
	CUtlVector< void * >			m_SavedObjects;

private:
	CBox3DPhysicsEnvironment			*m_pEnv;
	IMotionEvent						*m_pHandler;
	CUtlVector< CBox3DPhysicsObject * >	m_Objects;
	priority_t							m_Priority;
};

//-------------------------------------------------------------------------------------------------
// Fluid controller: buoyancy and drag for objects overlapping a fluid trigger
//-------------------------------------------------------------------------------------------------
class CBox3DFluidController : public IPhysicsFluidController, public IBox3DController
{
public:
	CBox3DFluidController( CBox3DPhysicsEnvironment *pEnv, CBox3DPhysicsObject *pFluidObject, const fluidparams_t *pParams );
	~CBox3DFluidController();

	void	SetGameData( void *pGameData )				{ m_pGameData = pGameData; }
	void	*GetGameData( void ) const					{ return m_pGameData; }
	void	GetSurfacePlane( Vector *pNormal, float *pDist ) const;
	float	GetDensity() const							{ return m_flDensity; }
	void	WakeAllSleepingObjects();
	int		GetContents() const							{ return m_nContents; }

	void	OnPreStep( float dt );

	void	AddObject( CBox3DPhysicsObject *pObject );
	void	RemoveObject( CBox3DPhysicsObject *pObject );
	CBox3DPhysicsObject *GetFluidObject() const		{ return m_pFluidObject; }

private:
	CBox3DPhysicsEnvironment			*m_pEnv;
	CBox3DPhysicsObject					*m_pFluidObject;
	void								*m_pGameData;
	Vector								m_SurfaceNormal;
	float								m_flSurfaceDist;
	Vector								m_CurrentVelocity;	// in/s
	float								m_flDensity;		// kg/m^3
	float								m_flDamping;
	int									m_nContents;
	CUtlVector< CBox3DPhysicsObject * >	m_Objects;
};

//-------------------------------------------------------------------------------------------------
// Vehicle controller: a simple wheeled car built from Box3D wheel joints. Raycast vehicles
// (airboat, jetski) use the same model, which only works on land.
//-------------------------------------------------------------------------------------------------
class CBox3DVehicleController : public IPhysicsVehicleController, public IBox3DController, public IBox3DObjectListener
{
public:
	CBox3DVehicleController( CBox3DPhysicsEnvironment *pEnv, CBox3DPhysicsObject *pBody, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace );
	~CBox3DVehicleController();

	void	Update( float dt, vehicle_controlparams_t &controls );
	const vehicle_operatingparams_t &GetOperatingParams()	{ return m_OperatingParams; }
	const vehicleparams_t &GetVehicleParams()				{ return m_Params; }
	vehicleparams_t &GetVehicleParamsForChange()			{ return m_Params; }
	float	UpdateBooster( float dt );
	int		GetWheelCount( void )							{ return m_nWheelCount; }
	IPhysicsObject *GetWheel( int index );
	bool	GetWheelContactPoint( int index, Vector *pContactPoint, int *pSurfaceProps );
	void	SetSpringLength( int wheelIndex, float length );
	void	SetWheelFriction( int wheelIndex, float friction );
	void	OnVehicleEnter( void );
	void	OnVehicleExit( void );
	void	SetEngineDisabled( bool bDisable )				{ m_bEngineDisabled = bDisable; }
	bool	IsEngineDisabled( void )						{ return m_bEngineDisabled; }
	void	GetCarSystemDebugData( vehicle_debugcarsystem_t &debugCarSystem );
	void	VehicleDataReload();

	void	OnPreStep( float dt );
	void	OnObjectDestroyed( CBox3DPhysicsObject *pObject );

private:
	void	CreateWheels();
	void	DestroyWheels();

	CBox3DPhysicsEnvironment	*m_pEnv;
	CBox3DPhysicsObject			*m_pBody;
	IPhysicsGameTrace			*m_pGameTrace;
	vehicleparams_t				m_Params;
	vehicle_operatingparams_t	m_OperatingParams;
	vehicle_controlparams_t		m_Controls;
	unsigned int				m_nVehicleType;
	int							m_nWheelCount;
	CBox3DPhysicsObject			*m_pWheels[VEHICLE_MAX_WHEEL_COUNT];
	b3JointId					m_WheelJoints[VEHICLE_MAX_WHEEL_COUNT];
	float						m_flSteering;		// radians
	float						m_flThrottle;
	float						m_flBoostTimeLeft;
	float						m_flBoostDelay;
	bool						m_bEngineDisabled;
	bool						m_bOccupied;
};

//-------------------------------------------------------------------------------------------------
// Friction snapshot: iterates the touching contacts of an object
//-------------------------------------------------------------------------------------------------
class CBox3DFrictionSnapshot : public IPhysicsFrictionSnapshot
{
public:
	CBox3DFrictionSnapshot( CBox3DPhysicsObject *pObject );

	bool			IsValid();
	IPhysicsObject	*GetObject( int index );
	int				GetMaterial( int index );
	void			GetContactPoint( Vector &out );
	void			GetSurfaceNormal( Vector &out );
	float			GetNormalForce();
	float			GetEnergyAbsorbed();
	void			RecomputeFriction();
	void			ClearFrictionForce();
	void			MarkContactForDelete();
	void			DeleteAllMarkedContacts( bool wakeObjects );
	void			NextFrictionData();
	float			GetFrictionCoefficient();

private:
	struct Contact_t
	{
		CBox3DPhysicsObject	*pOther;
		int					materialSelf;
		int					materialOther;
		Vector				point;			// world, inches
		Vector				normal;			// points from this object toward the other one
		float				normalForce;	// HL units (kg * in / s^2)
		float				friction;
		bool				bMarked;
	};

	CBox3DPhysicsObject		*m_pObject;
	CUtlVector< Contact_t >	m_Contacts;
	int						m_nIndex;
};

#endif // B3VP_CONTROLLERS_H
