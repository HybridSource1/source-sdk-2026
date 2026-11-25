//=================================================================================================
//
// VPhysics Box3D - IPhysicsObject
//
//=================================================================================================

#ifndef B3VP_OBJECT_H
#define B3VP_OBJECT_H

#ifdef _WIN32
#pragma once
#endif

#include "b3vp_base.h"

class CPhysCollide;
class CBox3DShadowController;
class CBox3DPlayerController;
class CBox3DFluidController;
class CBox3DConstraint;

// Anything that holds a pointer to an object and must be told when it dies
class IBox3DObjectListener
{
public:
	virtual void OnObjectDestroyed( CBox3DPhysicsObject *pObject ) = 0;
};

// Collision filter categories
enum
{
	B3VP_CATEGORY_STATIC	= 0x0001,
	B3VP_CATEGORY_MOVING	= 0x0002,
	B3VP_CATEGORY_ALL		= 0xFFFF,
};

// Data written by Save()/read by Restore() for an object
struct B3VPObjectSaveData_t;

class CBox3DPhysicsObject : public IPhysicsObject
{
public:
	CBox3DPhysicsObject();
	virtual ~CBox3DPhysicsObject();

	bool					Init( CBox3DPhysicsEnvironment *pEnvironment, const CPhysCollide *pCollide, float sphereRadius,
								  int materialIndex, const Vector &position, const QAngle &angles,
								  const objectparams_t *pParams, bool bStatic );

	// IPhysicsObject
	bool					IsStatic() const;
	bool					IsAsleep() const;
	bool					IsTrigger() const;
	bool					IsFluid() const;
	bool					IsHinged() const;
	bool					IsCollisionEnabled() const;
	bool					IsGravityEnabled() const;
	bool					IsDragEnabled() const;
	bool					IsMotionEnabled() const;
	bool					IsMoveable() const;
	bool					IsAttachedToConstraint( bool bExternalOnly ) const;

	void					EnableCollisions( bool enable );
	void					EnableGravity( bool enable );
	void					EnableDrag( bool enable );
	void					EnableMotion( bool enable );

	void					SetGameData( void *pGameData );
	void					*GetGameData( void ) const;
	void					SetGameFlags( unsigned short userFlags );
	unsigned short			GetGameFlags( void ) const;
	void					SetGameIndex( unsigned short gameIndex );
	unsigned short			GetGameIndex( void ) const;
	void					SetCallbackFlags( unsigned short callbackflags );
	unsigned short			GetCallbackFlags( void ) const;

	void					Wake( void );
	void					Sleep( void );
	void					RecheckCollisionFilter();
	void					RecheckContactPoints();

	void					SetMass( float mass );
	float					GetMass( void ) const;
	float					GetInvMass( void ) const;
	Vector					GetInertia( void ) const;
	Vector					GetInvInertia( void ) const;
	void					SetInertia( const Vector &inertia );

	void					SetDamping( const float *speed, const float *rot );
	void					GetDamping( float *speed, float *rot ) const;

	void					SetDragCoefficient( float *pDrag, float *pAngularDrag );
	void					SetBuoyancyRatio( float ratio );

	int						GetMaterialIndex() const;
	void					SetMaterialIndex( int materialIndex );

	unsigned int			GetContents() const;
	void					SetContents( unsigned int contents );

	float					GetSphereRadius() const;
	float					GetEnergy() const;
	Vector					GetMassCenterLocalSpace() const;

	void					SetPosition( const Vector &worldPosition, const QAngle &angles, bool isTeleport );
	void					SetPositionMatrix( const matrix3x4_t &matrix, bool isTeleport );
	void					GetPosition( Vector *worldPosition, QAngle *angles ) const;
	void					GetPositionMatrix( matrix3x4_t *positionMatrix ) const;

	void					SetVelocity( const Vector *velocity, const AngularImpulse *angularVelocity );
	void					SetVelocityInstantaneous( const Vector *velocity, const AngularImpulse *angularVelocity );
	void					GetVelocity( Vector *velocity, AngularImpulse *angularVelocity ) const;
	void					AddVelocity( const Vector *velocity, const AngularImpulse *angularVelocity );
	void					GetVelocityAtPoint( const Vector &worldPosition, Vector *pVelocity ) const;
	void					GetImplicitVelocity( Vector *velocity, AngularImpulse *angularVelocity ) const;

	void					LocalToWorld( Vector *worldPosition, const Vector &localPosition ) const;
	void					WorldToLocal( Vector *localPosition, const Vector &worldPosition ) const;
	void					LocalToWorldVector( Vector *worldVector, const Vector &localVector ) const;
	void					WorldToLocalVector( Vector *localVector, const Vector &worldVector ) const;

	void					ApplyForceCenter( const Vector &forceVector );
	void					ApplyForceOffset( const Vector &forceVector, const Vector &worldPosition );
	void					ApplyTorqueCenter( const AngularImpulse &torque );

	void					CalculateForceOffset( const Vector &forceVector, const Vector &worldPosition, Vector *centerForce, AngularImpulse *centerTorque ) const;
	void					CalculateVelocityOffset( const Vector &forceVector, const Vector &worldPosition, Vector *centerVelocity, AngularImpulse *centerAngularVelocity ) const;
	float					CalculateLinearDrag( const Vector &unitDirection ) const;
	float					CalculateAngularDrag( const Vector &objectSpaceRotationAxis ) const;

	bool					GetContactPoint( Vector *contactPoint, IPhysicsObject **contactObject ) const;

	void					SetShadow( float maxSpeed, float maxAngularSpeed, bool allowPhysicsMovement, bool allowPhysicsRotation );
	void					UpdateShadow( const Vector &targetPosition, const QAngle &targetAngles, bool tempDisableGravity, float timeOffset );
	int						GetShadowPosition( Vector *position, QAngle *angles ) const;
	IPhysicsShadowController *GetShadowController( void ) const;
	void					RemoveShadowController();
	float					ComputeShadowControl( const hlshadowcontrol_params_t &params, float secondsToArrival, float dt );

	const CPhysCollide		*GetCollide( void ) const;
	const char				*GetName() const;

	void					BecomeTrigger();
	void					RemoveTrigger();
	void					BecomeHinged( int localAxis );
	void					RemoveHinged();

	IPhysicsFrictionSnapshot *CreateFrictionSnapshot();
	void					DestroyFrictionSnapshot( IPhysicsFrictionSnapshot *pSnapshot );

	void					OutputDebugInfo() const;

public:
	// Internal interface
	b3BodyId				GetBody() const					{ return m_Body; }
	CBox3DPhysicsEnvironment *GetEnvironment() const		{ return m_pEnvironment; }
	bool					IsDynamicBody() const;
	bool					IsMarkedForDelete() const		{ return ( m_callbackFlags & CALLBACK_MARKED_FOR_DELETE ) != 0; }

	// Velocity as seen by the game during PreCollision callbacks (pre-step values)
	void					CachePreStepVelocity();
	void					SetUsePreStepVelocity( bool bUse )	{ m_bUsePreStepVelocity = bUse; }

	// Velocity change applied as a force over the next step (motion controllers)
	void					ApplyStepVelocityChange( const Vector &deltaVelocity, const AngularImpulse &deltaAngularLocal, float dt );

	// Called by the environment each step
	void					PreStep( float dt );
	void					PostStep( float dt );

	// Called by controllers
	void					SetShadowControllerInternal( CBox3DShadowController *pShadow );
	void					SetPlayerControllerInternal( CBox3DPlayerController *pPlayer );
	CBox3DPlayerController	*GetPlayerControllerInternal() const	{ return m_pPlayerController; }
	void					SetFluidControllerInternal( CBox3DFluidController *pFluid )	{ m_pFluidController = pFluid; }
	CBox3DFluidController	*GetFluidControllerInternal() const	{ return m_pFluidController; }
	void					UpdateBodyType();
	void					SetTemporaryGravityDisable( bool bDisable );

	void					AddConstraint( CBox3DConstraint *pConstraint );
	void					RemoveConstraint( CBox3DConstraint *pConstraint );

	void					AddListener( IBox3DObjectListener *pListener );
	void					RemoveListener( IBox3DObjectListener *pListener );

	// Surface property index Box3D uses for a shape's material
	int						GetMaterialIndexForShape( b3ShapeId shape ) const;

	float					GetBuoyancyRatio() const		{ return m_flBuoyancyRatio; }
	float					GetVolume() const				{ return m_flVolume; }		// in^3
	float					GetDragCoefficient() const		{ return m_flDragCoefficient; }
	float					GetAngularDragCoefficient() const { return m_flAngularDragCoefficient; }

	// Mass properties in SI units
	const b3MassData		&GetMassData() const			{ return m_MassData; }
	b3Vec3					GetWorldCenterOfMass() const;
	b3AABB					GetWorldBounds() const;		// tight, metres
	b3Matrix3				GetWorldInverseInertia() const;

	// Save/restore
	void					WriteSaveData( B3VPObjectSaveData_t &data ) const;
	void					ApplySaveData( const B3VPObjectSaveData_t &data );

	void					NotifyDestroyed();

	// Moves the body to the environment's anchor/transfer
	void					SetEnvironmentInternal( CBox3DPhysicsEnvironment *pEnvironment ) { m_pEnvironment = pEnvironment; }

	// Last-step awake state for wake/sleep events
	bool					m_bWasAwake;

	// Tracks whether the game is told this moved (GetActiveObjects)
	int						m_nActiveFrame;

private:
	void					CreateShapes();
	void					DestroyShapes();
	b3ShapeDef				MakeShapeDef( int surfaceProp ) const;
	b3Filter				MakeFilter() const;
	void					ComputeMassProperties( float mass, float inertiaScale, float rotInertiaLimit, const Vector *pMassCenterOverride );
	void					ApplyMassData();
	b3Vec3					LocalAngularToWorld( const AngularImpulse &local ) const;
	AngularImpulse			WorldAngularToLocal( const b3Vec3 &world ) const;

	CBox3DPhysicsEnvironment	*m_pEnvironment;
	b3BodyId					m_Body;
	CUtlVector< b3ShapeId >		m_Shapes;
	const CPhysCollide			*m_pCollide;
	float						m_flSphereRadius;			// inches, > 0 for sphere objects

	void						*m_pGameData;
	unsigned short				m_gameFlags;
	unsigned short				m_gameIndex;
	unsigned short				m_callbackFlags;
	int							m_materialIndex;
	unsigned int				m_contents;

	bool						m_bStatic;
	bool						m_bMotionEnabled;
	bool						m_bGravityEnabled;
	bool						m_bTempGravityDisabled;
	bool						m_bDragEnabled;
	bool						m_bCollisionsEnabled;
	bool						m_bTrigger;
	bool						m_bUsePreStepVelocity;

	b3MassData					m_MassData;					// SI, body local space
	float						m_flMass;					// kg (as requested by the game)
	float						m_flLinearDamping;
	float						m_flAngularDamping;
	float						m_flDragCoefficient;
	float						m_flAngularDragCoefficient;
	float						m_flBuoyancyRatio;
	float						m_flVolume;					// in^3
	Vector						m_DragAreas;				// in^2, local axes

	b3Vec3						m_PreStepLinearVelocity;	// SI, world
	b3Vec3						m_PreStepAngularVelocity;	// SI, world

	Vector						m_LastPosition;
	QAngle						m_LastAngles;
	Vector						m_ImplicitVelocity;
	AngularImpulse				m_ImplicitAngularVelocity;

	CBox3DShadowController		*m_pShadowController;
	CBox3DPlayerController		*m_pPlayerController;
	CBox3DFluidController		*m_pFluidController;

	b3JointId					m_HingeJoint;
	int							m_nHingeAxis;
	bool						m_bHinged;
	CUtlVector< CBox3DConstraint * >	m_Constraints;
	CUtlVector< IBox3DObjectListener * > m_Listeners;

	char						m_szName[64];
};

// Plain data so it can be dumped through ISave::WriteData
struct B3VPObjectSaveData_t
{
	int					version;
	Vector				position;
	QAngle				angles;
	Vector				velocity;
	AngularImpulse		angularVelocity;
	float				mass;
	Vector				inertia;
	float				damping;
	float				rotDamping;
	float				dragCoefficient;
	float				angularDragCoefficient;
	float				buoyancyRatio;
	float				volume;
	float				sphereRadius;
	unsigned int		contents;
	unsigned short		gameFlags;
	unsigned short		gameIndex;
	unsigned short		callbackFlags;
	char				materialName[64];
	bool				bStatic;
	bool				bMotionEnabled;
	bool				bGravityEnabled;
	bool				bDragEnabled;
	bool				bCollisionsEnabled;
	bool				bTrigger;
	bool				bAsleep;
	bool				bHinged;
	int					hingeAxis;
	Vector				massCenter;					// local, inches

	// Shadow controller (inline, like IVP)
	bool				bHasShadow;
	bool				bShadowAllowTranslation;
	bool				bShadowAllowRotation;
	bool				bShadowPhysicallyControlled;
	float				shadowMaxSpeed;
	float				shadowMaxAngular;
	float				shadowTeleportDistance;
	Vector				shadowTargetPosition;
	QAngle				shadowTargetAngles;
	float				shadowSecondsToArrival;

	char				name[64];
};

#define B3VP_OBJECT_SAVE_VERSION	1

#endif // B3VP_OBJECT_H
