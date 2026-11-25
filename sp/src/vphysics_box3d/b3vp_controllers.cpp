//=================================================================================================
//
// VPhysics Box3D - controllers and friction snapshots
//
//=================================================================================================

#include "b3vp_base.h"
#include "b3vp_controllers.h"
#include "b3vp_surfaceprops.h"

#include "bspflags.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

static ConVar box3d_player_teleport_distance( "box3d_player_teleport_distance", "24", 0, "Distance (inches) at which the player's physics shadow snaps to the game position instead of moving" );
static ConVar box3d_vehicle_torque_scale( "box3d_vehicle_torque_scale", "4", 0, "Wheel torque (N*m) per engine horsepower for Box3D vehicles" );
static ConVar box3d_fluid_damping_scale( "box3d_fluid_damping_scale", "20", 0, "Scale on the per-second fluid damping applied to submerged objects" );
static ConVar box3d_vehicle_brake_scale( "box3d_vehicle_brake_scale", "1.5", 0, "Wheel brake torque (N*m) per kg of vehicle mass" );

//-------------------------------------------------------------------------------------------------
// Shadow controller
//-------------------------------------------------------------------------------------------------

CBox3DShadowController::CBox3DShadowController( CBox3DPhysicsObject *pObject, bool allowTranslation, bool allowRotation )
	: m_pObject( pObject )
	, m_TargetPosition( 0, 0, 0 )
	, m_TargetAngles( 0, 0, 0 )
	, m_LastImpulse( 0, 0, 0 )
	, m_flSecondsToArrival( 0.0f )
	, m_flMaxSpeed( 1e4f )
	, m_flMaxAngular( 1e4f )
	, m_flTeleportDistance( 0.0f )
	, m_nSavedMaterial( pObject->GetMaterialIndex() )
	, m_nSavedCallbackFlags( pObject->GetCallbackFlags() )
	, m_bAllowTranslation( allowTranslation )
	, m_bAllowRotation( allowRotation )
	, m_bPhysicallyControlled( false )
	, m_bEnabled( false )
{
	pObject->GetPosition( &m_TargetPosition, &m_TargetAngles );
	pObject->SetShadowControllerInternal( this );
}

CBox3DShadowController::~CBox3DShadowController()
{
}

void CBox3DShadowController::DetachObject()
{
	m_pObject = NULL;
}

void CBox3DShadowController::Update( const Vector &position, const QAngle &angles, float timeOffset )
{
	if ( !m_pObject )
		return;

	if ( m_bEnabled && position == m_TargetPosition && angles == m_TargetAngles )
		return;

	m_TargetPosition = position;
	m_TargetAngles = angles;
	m_flSecondsToArrival = MAX( timeOffset, 0.0f );
	m_bEnabled = true;
	m_pObject->Wake();
}

void CBox3DShadowController::MaxSpeed( float maxSpeed, float maxAngularSpeed )
{
	m_flMaxSpeed = maxSpeed;
	m_flMaxAngular = maxAngularSpeed;
}

void CBox3DShadowController::StepUp( float height )
{
	if ( !m_pObject || height == 0.0f )
		return;

	Vector position;
	QAngle angles;
	m_pObject->GetPosition( &position, &angles );
	position.z += height;
	m_pObject->SetPosition( position, angles, false );
}

void CBox3DShadowController::SetTeleportDistance( float teleportDistance )
{
	m_flTeleportDistance = teleportDistance;
}

void CBox3DShadowController::SetPhysicallyControlled( bool isPhysicallyControlled )
{
	m_bPhysicallyControlled = isPhysicallyControlled;
}

void CBox3DShadowController::UseShadowMaterial( bool bUseShadowMaterial )
{
	// IVP swaps in a frictionless material here. Box3D kinematic bodies don't need it.
}

void CBox3DShadowController::ObjectMaterialChanged( int materialIndex )
{
	m_nSavedMaterial = materialIndex;
}

float CBox3DShadowController::GetTargetPosition( Vector *pPositionOut, QAngle *pAnglesOut )
{
	if ( pPositionOut ) *pPositionOut = m_TargetPosition;
	if ( pAnglesOut ) *pAnglesOut = m_TargetAngles;
	return m_flSecondsToArrival;
}

void CBox3DShadowController::GetMaxSpeed( float *pMaxSpeedOut, float *pMaxAngularSpeedOut )
{
	if ( pMaxSpeedOut ) *pMaxSpeedOut = m_flMaxSpeed;
	if ( pMaxAngularSpeedOut ) *pMaxAngularSpeedOut = m_flMaxAngular;
}

static b3Quat B3VP_Slerp( b3Quat a, b3Quat b, float t )
{
	float dot = a.v.x * b.v.x + a.v.y * b.v.y + a.v.z * b.v.z + a.s * b.s;
	if ( dot < 0.0f )
	{
		b.v = b3Neg( b.v );
		b.s = -b.s;
		dot = -dot;
	}

	if ( dot > 0.9995f )
	{
		b3Quat r;
		r.v = b3Add( a.v, b3MulSV( t, b3Sub( b.v, a.v ) ) );
		r.s = a.s + t * ( b.s - a.s );
		return b3NormalizeQuat( r );
	}

	const float theta = acosf( dot );
	const float sinTheta = sinf( theta );
	const float wa = sinf( ( 1.0f - t ) * theta ) / sinTheta;
	const float wb = sinf( t * theta ) / sinTheta;
	b3Quat r;
	r.v = b3Add( b3MulSV( wa, a.v ), b3MulSV( wb, b.v ) );
	r.s = wa * a.s + wb * b.s;
	return b3NormalizeQuat( r );
}

void CBox3DShadowController::OnPreStep( float dt )
{
	if ( !m_pObject || !m_bEnabled || dt <= 0.0f )
		return;

	const b3BodyId body = m_pObject->GetBody();
	if ( !b3Body_IsValid( body ) || m_pObject->IsStatic() || !m_pObject->IsMotionEnabled() )
		return;

	if ( !IsKinematic() )
	{
		// Steer a dynamic body toward the target like IVP
		hlshadowcontrol_params_t params;
		params.targetPosition = m_TargetPosition;
		params.targetRotation = m_TargetAngles;
		params.maxSpeed = m_flMaxSpeed;
		params.maxDampSpeed = m_flMaxSpeed;
		params.maxAngular = m_flMaxAngular;
		params.maxDampAngular = m_flMaxAngular;
		params.dampFactor = 1.0f;
		params.teleportDistance = m_flTeleportDistance;

		Vector before;
		m_pObject->GetVelocity( &before, NULL );
		m_flSecondsToArrival = m_pObject->ComputeShadowControl( params, m_flSecondsToArrival, dt );
		Vector after;
		m_pObject->GetVelocity( &after, NULL );
		m_LastImpulse = after - before;

		if ( !m_bAllowRotation )
		{
			// Keep the game's orientation
			Vector position;
			m_pObject->GetPosition( &position, NULL );
			b3Body_SetTransform( body, HL2B3( position ), AnglesToB3Quat( m_TargetAngles ) );
		}
		return;
	}

	// Kinematic follower
	const b3Transform current = b3Body_GetTransform( body );
	const Vector currentPos = B32HL( current.p );
	const b3Quat targetRot = AnglesToB3Quat( m_TargetAngles );

	Vector delta = m_TargetPosition - currentPos;
	if ( m_flTeleportDistance > 0.0f && delta.LengthSqr() > m_flTeleportDistance * m_flTeleportDistance )
	{
		b3Body_SetTransform( body, HL2B3( m_TargetPosition ), targetRot );
		b3Body_SetLinearVelocity( body, b3Vec3_zero );
		b3Body_SetAngularVelocity( body, b3Vec3_zero );
		m_flSecondsToArrival = 0.0f;
		return;
	}

	const float fraction = m_flSecondsToArrival > 0.0f ? MIN( dt / m_flSecondsToArrival, 1.0f ) : 1.0f;
	m_flSecondsToArrival = MAX( m_flSecondsToArrival - dt, 0.0f );

	Vector step = delta * fraction;
	if ( m_flMaxSpeed > 0.0f )
	{
		const float maxStep = m_flMaxSpeed * dt;
		const float len = step.Length();
		if ( len > maxStep && len > 0.0f )
			step *= maxStep / len;
	}

	b3Transform target;
	target.p = HL2B3( currentPos + step );
	target.q = B3VP_Slerp( current.q, targetRot, fraction );

	const Vector oldVelocity = B32HL( b3Body_GetLinearVelocity( body ) );
	b3Body_SetTargetTransform( body, target, dt, true );
	m_LastImpulse = B32HL( b3Body_GetLinearVelocity( body ) ) - oldVelocity;
}

//-------------------------------------------------------------------------------------------------
// Player controller
//-------------------------------------------------------------------------------------------------

CBox3DPlayerController::CBox3DPlayerController( CBox3DPhysicsObject *pObject )
	: m_pObject( NULL )
	, m_pHandler( NULL )
	, m_TargetPosition( 0, 0, 0 )
	, m_TargetVelocity( 0, 0, 0 )
	, m_MaxSpeed( 0, 0, 0 )
	, m_LastImpulse( 0, 0, 0 )
	, m_LastPosition( 0, 0, 0 )
	, m_flSecondsToArrival( 0.0f )
	, m_flPushMassLimit( 1e4f )
	, m_flPushSpeedLimit( 1e4f )
	, m_bOnGround( false )
	, m_bWasFrozen( false )
	, m_bEnabled( false )
{
	AttachObject( pObject );
}

CBox3DPlayerController::~CBox3DPlayerController()
{
	DetachObject();
}

void CBox3DPlayerController::AttachObject( CBox3DPhysicsObject *pObject )
{
	m_pObject = pObject;
	if ( !m_pObject )
		return;

	m_pObject->AddListener( this );
	m_pObject->SetPlayerControllerInternal( this );
	m_pObject->SetCallbackFlags( m_pObject->GetCallbackFlags() | CALLBACK_IS_PLAYER_CONTROLLER );
	m_pObject->GetPosition( &m_TargetPosition, NULL );
	m_LastPosition = m_TargetPosition;
}

void CBox3DPlayerController::DetachObject()
{
	if ( !m_pObject )
		return;

	m_pObject->RemoveListener( this );
	m_pObject->SetCallbackFlags( m_pObject->GetCallbackFlags() & ~CALLBACK_IS_PLAYER_CONTROLLER );
	m_pObject->SetPlayerControllerInternal( NULL );
	m_pObject = NULL;
}

void CBox3DPlayerController::OnObjectDestroyed( CBox3DPhysicsObject *pObject )
{
	if ( pObject == m_pObject )
		m_pObject = NULL;
}

void CBox3DPlayerController::Update( const Vector &position, const Vector &velocity, float secondsToArrival, bool onground, IPhysicsObject *ground )
{
	m_TargetPosition = position;
	m_TargetVelocity = velocity;
	m_flSecondsToArrival = MAX( secondsToArrival, 0.0f );
	m_bOnGround = onground;
	m_bEnabled = true;
	MaxSpeed( velocity );

	if ( m_pObject )
		m_pObject->Wake();
}

void CBox3DPlayerController::MaxSpeed( const Vector &maxVelocity )
{
	m_MaxSpeed.Init( fabsf( maxVelocity.x ), fabsf( maxVelocity.y ), fabsf( maxVelocity.z ) );
}

void CBox3DPlayerController::SetObject( IPhysicsObject *pObject )
{
	CBox3DPhysicsObject *pNew = static_cast< CBox3DPhysicsObject * >( pObject );
	if ( pNew == m_pObject )
		return;

	DetachObject();
	AttachObject( pNew );
}

int CBox3DPlayerController::GetShadowPosition( Vector *position, QAngle *angles )
{
	if ( !m_pObject )
	{
		if ( position ) *position = m_TargetPosition;
		if ( angles ) angles->Init();
		return 0;
	}

	m_pObject->GetPosition( position, angles );
	return 1;
}

void CBox3DPlayerController::StepUp( float height )
{
	if ( !m_pObject || height == 0.0f )
		return;

	Vector position;
	QAngle angles;
	m_pObject->GetPosition( &position, &angles );
	position.z += height;
	m_pObject->SetPosition( position, angles, true );
}

void CBox3DPlayerController::Jump()
{
	// The shadow follows the game's movement, nothing to do
}

void CBox3DPlayerController::GetShadowVelocity( Vector *velocity )
{
	if ( !velocity )
		return;

	if ( m_pObject )
		m_pObject->GetVelocity( velocity, NULL );
	else
		velocity->Init();
}

bool CBox3DPlayerController::IsInContact( void )
{
	if ( !m_pObject )
		return false;

	const b3BodyId body = m_pObject->GetBody();
	const int capacity = b3Body_GetContactCapacity( body );
	if ( capacity <= 0 )
		return false;

	CUtlVector< b3ContactData > contacts;
	contacts.SetCount( capacity );
	const int count = b3Body_GetContactData( body, contacts.Base(), capacity );
	for ( int i = 0; i < count; i++ )
	{
		if ( contacts[i].manifoldCount <= 0 )
			continue;

		CBox3DPhysicsObject *pA = GetShapeObject( contacts[i].shapeIdA );
		CBox3DPhysicsObject *pB = GetShapeObject( contacts[i].shapeIdB );
		CBox3DPhysicsObject *pOther = ( pA == m_pObject ) ? pB : pA;
		if ( pOther && pOther->IsMoveable() )
			return true;
	}
	return false;
}

void CBox3DPlayerController::OnPreStep( float dt )
{
	if ( !m_pObject || dt <= 0.0f )
		return;

	const b3BodyId body = m_pObject->GetBody();
	if ( !b3Body_IsValid( body ) )
		return;

	const b3Transform current = b3Body_GetTransform( body );
	const Vector currentPos = B32HL( current.p );
	m_LastPosition = currentPos;

	if ( !m_bEnabled )
	{
		b3Body_SetLinearVelocity( body, b3Vec3_zero );
		b3Body_SetAngularVelocity( body, b3Vec3_zero );
		return;
	}

	const Vector delta = m_TargetPosition - currentPos;
	const float teleport = box3d_player_teleport_distance.GetFloat();
	if ( delta.LengthSqr() > teleport * teleport )
	{
		if ( m_pHandler && !m_pHandler->ShouldMoveTo( m_pObject, m_TargetPosition ) )
		{
			m_bWasFrozen = true;
			return;
		}

		b3Body_SetTransform( body, HL2B3( m_TargetPosition ), current.q );
		b3Body_SetLinearVelocity( body, b3Vec3_zero );
		return;
	}

	const float fraction = m_flSecondsToArrival > 0.0f ? MIN( dt / m_flSecondsToArrival, 1.0f ) : 1.0f;
	m_flSecondsToArrival = MAX( m_flSecondsToArrival - dt, 0.0f );

	b3Transform target;
	target.p = HL2B3( currentPos + delta * fraction );
	target.q = current.q;
	b3Body_SetTargetTransform( body, target, dt, true );
	b3Body_SetAngularVelocity( body, b3Vec3_zero );
}

void CBox3DPlayerController::OnPostStep( float dt )
{
	if ( !m_pObject || dt <= 0.0f )
		return;

	Vector position;
	m_pObject->GetPosition( &position, NULL );
	m_LastImpulse = ( position - m_LastPosition ) / dt;
}

//-------------------------------------------------------------------------------------------------
// Motion controller
//-------------------------------------------------------------------------------------------------

CBox3DMotionController::CBox3DMotionController( CBox3DPhysicsEnvironment *pEnv, IMotionEvent *pHandler )
	: m_pEnv( pEnv )
	, m_pHandler( pHandler )
	, m_Priority( MEDIUM_PRIORITY )
{
}

CBox3DMotionController::~CBox3DMotionController()
{
	ClearObjects();
}

void CBox3DMotionController::AttachObject( IPhysicsObject *pObject, bool checkIfAlreadyAttached )
{
	CBox3DPhysicsObject *pBox3DObject = static_cast< CBox3DPhysicsObject * >( pObject );
	if ( !pBox3DObject )
		return;

	if ( checkIfAlreadyAttached && m_Objects.Find( pBox3DObject ) != m_Objects.InvalidIndex() )
		return;

	m_Objects.AddToTail( pBox3DObject );
	pBox3DObject->AddListener( this );
}

void CBox3DMotionController::DetachObject( IPhysicsObject *pObject )
{
	CBox3DPhysicsObject *pBox3DObject = static_cast< CBox3DPhysicsObject * >( pObject );
	if ( m_Objects.FindAndRemove( pBox3DObject ) )
	{
		if ( m_Objects.Find( pBox3DObject ) == m_Objects.InvalidIndex() )
			pBox3DObject->RemoveListener( this );
	}
}

void CBox3DMotionController::GetObjects( IPhysicsObject **pObjectList )
{
	for ( int i = 0; i < m_Objects.Count(); i++ )
		pObjectList[i] = m_Objects[i];
}

void CBox3DMotionController::ClearObjects( void )
{
	for ( int i = 0; i < m_Objects.Count(); i++ )
		m_Objects[i]->RemoveListener( this );
	m_Objects.RemoveAll();
}

void CBox3DMotionController::WakeObjects( void )
{
	for ( int i = 0; i < m_Objects.Count(); i++ )
		m_Objects[i]->Wake();
}

void CBox3DMotionController::OnObjectDestroyed( CBox3DPhysicsObject *pObject )
{
	m_Objects.FindAndRemove( pObject );
	while ( m_Objects.FindAndRemove( pObject ) ) {}
}

void CBox3DMotionController::OnPreStep( float dt )
{
	if ( !m_pHandler || dt <= 0.0f )
		return;

	for ( int i = 0; i < m_Objects.Count(); i++ )
	{
		CBox3DPhysicsObject *pObject = m_Objects[i];
		if ( !pObject->IsMoveable() || !pObject->IsDynamicBody() || pObject->IsAsleep() || pObject->IsMarkedForDelete() )
			continue;

		Vector linear( 0, 0, 0 );
		AngularImpulse angular( 0, 0, 0 );
		const IMotionEvent::simresult_e result = m_pHandler->Simulate( this, pObject, dt, linear, angular );

		// The handler may have detached or destroyed things
		if ( i >= m_Objects.Count() || m_Objects[i] != pObject )
			break;

		Vector deltaVelocity( 0, 0, 0 );
		AngularImpulse deltaAngular( 0, 0, 0 );

		switch ( result )
		{
		case IMotionEvent::SIM_NOTHING:
			continue;

		case IMotionEvent::SIM_LOCAL_ACCELERATION:
			pObject->LocalToWorldVector( &deltaVelocity, linear );
			deltaVelocity *= dt;
			deltaAngular = angular * dt;
			break;

		case IMotionEvent::SIM_LOCAL_FORCE:
			{
				pObject->LocalToWorldVector( &deltaVelocity, linear );
				deltaVelocity *= dt * pObject->GetInvMass();
				const Vector invInertia = pObject->GetInvInertia();
				deltaAngular.Init( angular.x * invInertia.x, angular.y * invInertia.y, angular.z * invInertia.z );
				deltaAngular *= dt;
			}
			break;

		case IMotionEvent::SIM_GLOBAL_ACCELERATION:
			deltaVelocity = linear * dt;
			pObject->WorldToLocalVector( &deltaAngular, angular );
			deltaAngular *= dt;
			break;

		case IMotionEvent::SIM_GLOBAL_FORCE:
			{
				deltaVelocity = linear * ( dt * pObject->GetInvMass() );
				AngularImpulse localTorque;
				pObject->WorldToLocalVector( &localTorque, angular );
				const Vector invInertia = pObject->GetInvInertia();
				deltaAngular.Init( localTorque.x * invInertia.x, localTorque.y * invInertia.y, localTorque.z * invInertia.z );
				deltaAngular *= dt;
			}
			break;
		}

		if ( deltaVelocity.LengthSqr() > 0.0f || deltaAngular.LengthSqr() > 0.0f )
			pObject->ApplyStepVelocityChange( deltaVelocity, deltaAngular, dt );
	}
}

//-------------------------------------------------------------------------------------------------
// Fluid controller
//-------------------------------------------------------------------------------------------------

CBox3DFluidController::CBox3DFluidController( CBox3DPhysicsEnvironment *pEnv, CBox3DPhysicsObject *pFluidObject, const fluidparams_t *pParams )
	: m_pEnv( pEnv )
	, m_pFluidObject( pFluidObject )
	, m_pGameData( pParams ? pParams->pGameData : NULL )
	, m_SurfaceNormal( 0, 0, 1 )
	, m_flSurfaceDist( 0.0f )
	, m_CurrentVelocity( 0, 0, 0 )
	, m_flDensity( 1000.0f )
	, m_flDamping( 0.8f )
	, m_nContents( CONTENTS_WATER )
{
	if ( pParams )
	{
		m_SurfaceNormal.Init( pParams->surfacePlane.x, pParams->surfacePlane.y, pParams->surfacePlane.z );
		const float len = VectorNormalize( m_SurfaceNormal );
		m_flSurfaceDist = len > 0.0f ? pParams->surfacePlane.w / len : pParams->surfacePlane.w;
		m_CurrentVelocity = pParams->currentVelocity;
		m_flDamping = pParams->damping;
		m_nContents = pParams->contents;
	}

	if ( m_pFluidObject )
	{
		m_flDensity = CBox3DPhysicsSurfaceProps::GetInstance().GetDensity( m_pFluidObject->GetMaterialIndex() );
		if ( m_flDensity <= 0.0f )
			m_flDensity = 1000.0f;

		m_pFluidObject->SetFluidControllerInternal( this );
		if ( !m_pFluidObject->IsTrigger() )
			m_pFluidObject->BecomeTrigger();
	}
}

CBox3DFluidController::~CBox3DFluidController()
{
	if ( m_pFluidObject )
		m_pFluidObject->SetFluidControllerInternal( NULL );
}

void CBox3DFluidController::GetSurfacePlane( Vector *pNormal, float *pDist ) const
{
	if ( pNormal ) *pNormal = m_SurfaceNormal;
	if ( pDist ) *pDist = m_flSurfaceDist;
}

void CBox3DFluidController::WakeAllSleepingObjects()
{
	for ( int i = 0; i < m_Objects.Count(); i++ )
		m_Objects[i]->Wake();
}

void CBox3DFluidController::AddObject( CBox3DPhysicsObject *pObject )
{
	if ( m_Objects.Find( pObject ) == m_Objects.InvalidIndex() )
		m_Objects.AddToTail( pObject );
	pObject->Wake();
}

void CBox3DFluidController::RemoveObject( CBox3DPhysicsObject *pObject )
{
	m_Objects.FindAndFastRemove( pObject );
}

void CBox3DFluidController::OnPreStep( float dt )
{
	if ( dt <= 0.0f )
		return;

	Vector gravity;
	m_pEnv->GetGravity( &gravity );
	const float g = HL2B3( gravity.Length() );

	const b3Vec3 normal = ToB3( m_SurfaceNormal );
	const float surfaceDist = HL2B3( m_flSurfaceDist );
	const b3Vec3 current = HL2B3( m_CurrentVelocity );

	for ( int i = 0; i < m_Objects.Count(); i++ )
	{
		CBox3DPhysicsObject *pObject = m_Objects[i];
		if ( !pObject->IsMoveable() || !pObject->IsDynamicBody() || pObject->IsMarkedForDelete() )
			continue;
		if ( !( pObject->GetCallbackFlags() & CALLBACK_DO_FLUID_SIMULATION ) )
			continue;

		const b3BodyId body = pObject->GetBody();
		const b3AABB box = pObject->GetWorldBounds();
		const b3Vec3 center = b3AABB_Center( box );
		const b3Vec3 extents = b3AABB_Extents( box );

		// Fraction of the bounds below the surface along the plane normal
		const float radius = fabsf( extents.x * normal.x ) + fabsf( extents.y * normal.y ) + fabsf( extents.z * normal.z );
		const float height = b3Dot( normal, center ) - surfaceDist;
		const float fraction = radius > 0.0f ? clamp( ( radius - height ) / ( 2.0f * radius ), 0.0f, 1.0f ) : ( height < 0.0f ? 1.0f : 0.0f );
		if ( fraction <= 0.0f )
			continue;

		const float mass = pObject->GetMass();
		const float ratio = MAX( pObject->GetBuoyancyRatio(), 0.01f );
		const float effectiveVolume = mass / ( ratio * m_flDensity );	// m^3 of fluid the object would displace when fully submerged
		const float buoyancy = m_flDensity * effectiveVolume * fraction * g;

		// Push from the submerged part's centre so things right themselves a little
		b3Vec3 point = pObject->GetWorldCenterOfMass();
		point = b3MulAdd( point, -( 1.0f - fraction ) * radius, normal );
		b3Body_ApplyForce( body, b3MulSV( buoyancy, normal ), point, true );

		// Drag relative to the current
		const float damp = MIN( 1.0f, m_flDamping * ( 0.25f + 0.75f * fraction ) * dt * box3d_fluid_damping_scale.GetFloat() );
		const b3Vec3 v = b3Body_GetLinearVelocity( body );
		const b3Vec3 relative = b3Sub( v, current );
		b3Body_SetLinearVelocity( body, b3MulAdd( v, -damp, relative ) );

		const b3Vec3 w = b3Body_GetAngularVelocity( body );
		b3Body_SetAngularVelocity( body, b3MulSV( 1.0f - damp, w ) );
	}
}

//-------------------------------------------------------------------------------------------------
// Vehicle controller
//-------------------------------------------------------------------------------------------------

CBox3DVehicleController::CBox3DVehicleController( CBox3DPhysicsEnvironment *pEnv, CBox3DPhysicsObject *pBody, const vehicleparams_t &params, unsigned int nVehicleType, IPhysicsGameTrace *pGameTrace )
	: m_pEnv( pEnv )
	, m_pBody( pBody )
	, m_pGameTrace( pGameTrace )
	, m_Params( params )
	, m_nVehicleType( nVehicleType )
	, m_nWheelCount( 0 )
	, m_flSteering( 0.0f )
	, m_flThrottle( 0.0f )
	, m_flBoostTimeLeft( 100.0f )
	, m_flBoostDelay( 0.0f )
	, m_bEngineDisabled( false )
	, m_bOccupied( false )
{
	memset( &m_OperatingParams, 0, sizeof( m_OperatingParams ) );
	memset( &m_Controls, 0, sizeof( m_Controls ) );
	m_OperatingParams.boostTimeLeft = 100;

	for ( int i = 0; i < VEHICLE_MAX_WHEEL_COUNT; i++ )
	{
		m_pWheels[i] = NULL;
		m_WheelJoints[i] = b3_nullJointId;
	}

	if ( nVehicleType != VEHICLE_TYPE_CAR_WHEELS )
		B3VP_Unsupported( "Raycast vehicles (airboat/jetski), using wheels instead" );

	if ( m_pBody )
	{
		m_pBody->AddListener( this );
		if ( m_Params.body.massOverride > 0.0f )
			m_pBody->SetMass( m_Params.body.massOverride );
		CreateWheels();
	}
}

CBox3DVehicleController::~CBox3DVehicleController()
{
	DestroyWheels();
	if ( m_pBody )
		m_pBody->RemoveListener( this );
}

void CBox3DVehicleController::OnObjectDestroyed( CBox3DPhysicsObject *pObject )
{
	if ( pObject == m_pBody )
	{
		m_pBody = NULL;
		for ( int i = 0; i < m_nWheelCount; i++ )
			m_WheelJoints[i] = b3_nullJointId;
	}

	for ( int i = 0; i < m_nWheelCount; i++ )
	{
		if ( m_pWheels[i] == pObject )
		{
			m_pWheels[i] = NULL;
			m_WheelJoints[i] = b3_nullJointId;
		}
	}
}

void CBox3DVehicleController::CreateWheels()
{
	const int wheelsPerAxle = clamp( m_Params.wheelsPerAxle, 1, 2 );
	const b3Transform bodyXf = b3Body_GetTransform( m_pBody->GetBody() );

	QAngle bodyAngles;
	m_pBody->GetPosition( NULL, &bodyAngles );

	// Wheel joint frame on the chassis: X = suspension (body up), Z = spin axis (body right)
	b3Matrix3 axes;
	axes.cx = b3Vec3_axisZ;
	axes.cz = b3Vec3_axisX;
	axes.cy = b3Cross( axes.cz, axes.cx );
	const b3Quat frameRotation = b3NormalizeQuat( b3MakeQuatFromMatrix( &axes ) );

	m_nWheelCount = 0;
	for ( int a = 0; a < m_Params.axleCount && a < VEHICLE_MAX_AXLE_COUNT; a++ )
	{
		const vehicle_axleparams_t &axle = m_Params.axles[a];
		for ( int w = 0; w < wheelsPerAxle && m_nWheelCount < VEHICLE_MAX_WHEEL_COUNT; w++ )
		{
			const Vector offset = ( wheelsPerAxle == 2 ) ? ( w == 0 ? axle.offset - axle.wheelOffset : axle.offset + axle.wheelOffset ) : axle.offset;

			Vector worldPos;
			m_pBody->LocalToWorld( &worldPos, offset );

			objectparams_t params;
			memset( &params, 0, sizeof( params ) );
			params.mass = axle.wheels.mass > 0.0f ? axle.wheels.mass : 20.0f;
			params.inertia = axle.wheels.inertia > 0.0f ? axle.wheels.inertia : 1.0f;
			params.damping = axle.wheels.damping;
			params.rotdamping = axle.wheels.rotdamping;
			params.dragCoefficient = 0.0f;
			params.enableCollisions = true;
			params.pGameData = m_pBody->GetGameData();
			params.pName = "VehicleWheel";

			const float radius = axle.wheels.radius > 0.0f ? axle.wheels.radius : 16.0f;
			CBox3DPhysicsObject *pWheel = static_cast< CBox3DPhysicsObject * >(
				m_pEnv->CreateSphereObject( radius, axle.wheels.materialIndex, worldPos, bodyAngles, &params, false ) );
			if ( !pWheel )
				continue;

			pWheel->SetCallbackFlags( pWheel->GetCallbackFlags() | CALLBACK_IS_VEHICLE_WHEEL );
			pWheel->AddListener( this );

			const float quarterMass = m_pBody->GetMass() / MAX( 1, m_Params.axleCount * wheelsPerAxle );
			const float springConstant = axle.suspension.springConstant > 0.0f ? axle.suspension.springConstant : 80.0f;
			const float omega = sqrtf( springConstant );
			const float zeta = clamp( axle.suspension.springDamping / ( 2.0f * omega ), 0.05f, 2.0f );
			const float travel = HL2B3( radius ) * 0.6f + HL2B3( axle.wheels.springAdditionalLength );

			b3WheelJointDef def = b3DefaultWheelJointDef();
			def.base.bodyIdA = m_pBody->GetBody();
			def.base.bodyIdB = pWheel->GetBody();
			def.base.localFrameA.p = HL2B3( offset );
			def.base.localFrameA.q = frameRotation;
			def.base.localFrameB.p = b3Vec3_zero;
			def.base.localFrameB.q = frameRotation;
			def.base.collideConnected = false;
			def.enableSuspensionSpring = true;
			def.suspensionHertz = omega / ( 2.0f * M_PI_F );
			def.suspensionDampingRatio = zeta;
			def.enableSuspensionLimit = true;
			def.lowerSuspensionLimit = -travel;
			def.upperSuspensionLimit = travel;
			def.enableSpinMotor = true;
			def.maxSpinTorque = 0.0f;
			def.spinSpeed = 0.0f;
			def.enableSteering = ( a == 0 );
			def.steeringHertz = 6.0f;
			def.steeringDampingRatio = 1.0f;
			def.maxSteeringTorque = quarterMass * 50.0f;
			def.enableSteeringLimit = ( a == 0 );
			const float maxSteer = DEG2RAD( MAX( m_Params.steering.degreesSlow, MAX( m_Params.steering.degreesFast, m_Params.steering.degreesBoost ) ) );
			def.lowerSteeringLimit = -MAX( maxSteer, 0.01f );
			def.upperSteeringLimit = MAX( maxSteer, 0.01f );
			def.targetSteeringAngle = 0.0f;

			b3BodyId chassis = m_pBody->GetBody();
			(void)chassis;
			(void)bodyXf;

			m_pWheels[m_nWheelCount] = pWheel;
			m_WheelJoints[m_nWheelCount] = b3CreateWheelJoint( m_pEnv->GetWorld(), &def );
			m_nWheelCount++;
		}
	}
}

void CBox3DVehicleController::DestroyWheels()
{
	for ( int i = 0; i < m_nWheelCount; i++ )
	{
		if ( b3Joint_IsValid( m_WheelJoints[i] ) )
			b3DestroyJoint( m_WheelJoints[i], true );
		m_WheelJoints[i] = b3_nullJointId;

		if ( m_pWheels[i] )
		{
			m_pWheels[i]->RemoveListener( this );
			m_pEnv->DestroyObject( m_pWheels[i] );
			m_pWheels[i] = NULL;
		}
	}
	m_nWheelCount = 0;
}

IPhysicsObject *CBox3DVehicleController::GetWheel( int index )
{
	if ( index < 0 || index >= m_nWheelCount )
		return NULL;
	return m_pWheels[index];
}

void CBox3DVehicleController::Update( float dt, vehicle_controlparams_t &controls )
{
	m_Controls = controls;
}

float CBox3DVehicleController::UpdateBooster( float dt )
{
	if ( m_flBoostDelay > 0.0f )
	{
		m_flBoostDelay = MAX( 0.0f, m_flBoostDelay - dt );
	}

	if ( m_Controls.boost > 0.0f && m_flBoostDelay <= 0.0f && m_Params.engine.boostDuration > 0.0f )
	{
		m_flBoostTimeLeft = MAX( 0.0f, m_flBoostTimeLeft - 100.0f * dt / m_Params.engine.boostDuration );
		if ( m_flBoostTimeLeft <= 0.0f )
		{
			m_flBoostDelay = m_Params.engine.boostDelay;
			m_flBoostTimeLeft = 100.0f;
		}
	}

	m_OperatingParams.boostTimeLeft = (int)m_flBoostTimeLeft;
	m_OperatingParams.boostDelay = m_flBoostDelay;
	return m_flBoostDelay;
}

bool CBox3DVehicleController::GetWheelContactPoint( int index, Vector *pContactPoint, int *pSurfaceProps )
{
	if ( index < 0 || index >= m_nWheelCount || !m_pWheels[index] )
		return false;

	IPhysicsObject *pOther = NULL;
	Vector point;
	if ( !m_pWheels[index]->GetContactPoint( &point, &pOther ) )
		return false;

	if ( pContactPoint )
		*pContactPoint = point;
	if ( pSurfaceProps )
		*pSurfaceProps = pOther ? pOther->GetMaterialIndex() : 0;
	return true;
}

void CBox3DVehicleController::SetSpringLength( int wheelIndex, float length )
{
	// Suspension length is set up from the vehicle script
}

void CBox3DVehicleController::SetWheelFriction( int wheelIndex, float friction )
{
	if ( wheelIndex < 0 || wheelIndex >= m_nWheelCount || !m_pWheels[wheelIndex] )
		return;

	b3BodyId body = m_pWheels[wheelIndex]->GetBody();
	b3ShapeId shapes[4];
	const int count = b3Body_GetShapes( body, shapes, 4 );
	for ( int i = 0; i < count; i++ )
		b3Shape_SetFriction( shapes[i], friction );
}

void CBox3DVehicleController::OnVehicleEnter( void )
{
	m_bOccupied = true;
	if ( m_pBody )
		m_pBody->Wake();
}

void CBox3DVehicleController::OnVehicleExit( void )
{
	m_bOccupied = false;
	memset( &m_Controls, 0, sizeof( m_Controls ) );
	m_Controls.handbrake = true;
}

void CBox3DVehicleController::GetCarSystemDebugData( vehicle_debugcarsystem_t &debugCarSystem )
{
	memset( &debugCarSystem, 0, sizeof( debugCarSystem ) );
	for ( int i = 0; i < m_nWheelCount && i < VEHICLE_DEBUGRENDERDATA_MAX_WHEELS; i++ )
	{
		if ( m_pWheels[i] )
			m_pWheels[i]->GetPosition( &debugCarSystem.vecWheelPos[i], NULL );
	}
}

void CBox3DVehicleController::VehicleDataReload()
{
	DestroyWheels();
	if ( m_pBody )
		CreateWheels();
}

void CBox3DVehicleController::OnPreStep( float dt )
{
	if ( !m_pBody || dt <= 0.0f )
		return;

	// Forward speed (body +Y is forward for Source vehicles)
	Vector velocity;
	m_pBody->GetVelocity( &velocity, NULL );
	Vector forward;
	m_pBody->LocalToWorldVector( &forward, Vector( 0, 1, 0 ) );
	const float forwardSpeed = DotProduct( velocity, forward ); // in/s
	const float speedMPH = fabsf( forwardSpeed ) / 17.6f;

	m_OperatingParams.speed = speedMPH;
	m_OperatingParams.isTorqueBoosting = m_Controls.boost > 0.0f && m_flBoostDelay <= 0.0f;

	// Steering
	const vehicle_steeringparams_t &steering = m_Params.steering;
	float maxDegrees = steering.degreesSlow;
	if ( steering.speedFast > steering.speedSlow )
		maxDegrees = RemapValClamped( speedMPH, steering.speedSlow, steering.speedFast, steering.degreesSlow, steering.degreesFast );
	if ( m_OperatingParams.isTorqueBoosting && steering.degreesBoost > 0.0f )
		maxDegrees = steering.degreesBoost;
	m_flSteering = -DEG2RAD( m_Controls.steering * maxDegrees );
	m_OperatingParams.steeringAngle = RAD2DEG( -m_flSteering );

	// Throttle and brakes
	const bool bEngineOn = !m_bEngineDisabled;
	float throttle = bEngineOn ? clamp( m_Controls.throttle, -1.0f, 1.0f ) : 0.0f;
	const bool bBraking = m_Controls.handbrake || m_Controls.brake > 0.0f || ( !m_bOccupied && throttle == 0.0f );
	const float brake = m_Controls.handbrake ? 1.0f : clamp( m_Controls.brake, 0.0f, 1.0f );

	const float maxSpeed = ( throttle >= 0.0f ) ? ( m_OperatingParams.isTorqueBoosting && m_Params.engine.boostMaxSpeed > 0.0f ? m_Params.engine.boostMaxSpeed : m_Params.engine.maxSpeed ) : m_Params.engine.maxRevSpeed;
	float engineTorque = MAX( m_Params.engine.horsepower, 1.0f ) * box3d_vehicle_torque_scale.GetFloat() * fabsf( throttle );
	if ( m_OperatingParams.isTorqueBoosting )
		engineTorque += m_Params.engine.boostForce * box3d_vehicle_torque_scale.GetFloat();
	const float brakeTorque = m_pBody->GetMass() * box3d_vehicle_brake_scale.GetFloat();

	int inContact = 0;
	for ( int i = 0; i < m_nWheelCount; i++ )
	{
		if ( !b3Joint_IsValid( m_WheelJoints[i] ) || !m_pWheels[i] )
			continue;

		const int axleIndex = i / clamp( m_Params.wheelsPerAxle, 1, 2 );
		const vehicle_axleparams_t &axle = m_Params.axles[axleIndex];
		const float radius = HL2B3( axle.wheels.radius > 0.0f ? axle.wheels.radius : 16.0f );

		if ( b3WheelJoint_IsSteeringEnabled( m_WheelJoints[i] ) )
			b3WheelJoint_SetTargetSteeringAngle( m_WheelJoints[i], m_flSteering );

		// Spin about +X (body right) is backwards, so forward wheel speed is negative
		if ( bBraking && ( m_Controls.handbrake ? axleIndex > 0 || m_Params.axleCount == 1 : true ) )
		{
			b3WheelJoint_SetSpinMotorSpeed( m_WheelJoints[i], 0.0f );
			b3WheelJoint_SetMaxSpinTorque( m_WheelJoints[i], brakeTorque * MAX( brake, 0.25f ) * MAX( axle.brakeFactor, 0.25f ) );
		}
		else if ( throttle != 0.0f && axle.torqueFactor > 0.0f )
		{
			const float targetSpeed = HL2B3( maxSpeed ) / MAX( radius, 0.01f );
			b3WheelJoint_SetSpinMotorSpeed( m_WheelJoints[i], throttle > 0.0f ? -targetSpeed : targetSpeed );
			b3WheelJoint_SetMaxSpinTorque( m_WheelJoints[i], engineTorque * axle.torqueFactor );
		}
		else
		{
			// Coasting: a little rolling resistance
			b3WheelJoint_SetSpinMotorSpeed( m_WheelJoints[i], 0.0f );
			b3WheelJoint_SetMaxSpinTorque( m_WheelJoints[i], brakeTorque * 0.02f );
		}

		IPhysicsObject *pOther = NULL;
		if ( m_pWheels[i]->GetContactPoint( NULL, &pOther ) )
			inContact++;
	}

	m_OperatingParams.wheelsInContact = inContact;
	m_OperatingParams.wheelsNotInContact = m_nWheelCount - inContact;

	// Fake an engine for the sounds/HUD
	const float maxRPM = m_Params.engine.maxRPM > 0.0f ? m_Params.engine.maxRPM : 4000.0f;
	const float speedRatio = m_Params.engine.maxSpeed > 0.0f ? clamp( fabsf( forwardSpeed ) / m_Params.engine.maxSpeed, 0.0f, 1.0f ) : 0.0f;
	m_OperatingParams.engineRPM = maxRPM * ( 0.15f + 0.85f * MAX( speedRatio, fabsf( throttle ) * 0.3f ) );
	m_OperatingParams.gear = forwardSpeed < -10.0f ? -1 : 1;
	m_OperatingParams.skidSpeed = 0.0f;
	m_OperatingParams.skidMaterial = 0;

	if ( fabsf( throttle ) > 0.0f || fabsf( m_Controls.steering ) > 0.0f )
	{
		m_pBody->Wake();
		for ( int i = 0; i < m_nWheelCount; i++ )
		{
			if ( m_pWheels[i] )
				m_pWheels[i]->Wake();
		}
	}

	// Keep-down force from the script
	if ( m_Params.body.addGravity > 0.0f && m_pBody->IsDynamicBody() )
	{
		Vector gravity;
		m_pEnv->GetGravity( &gravity );
		b3Body_ApplyForceToCenter( m_pBody->GetBody(), b3MulSV( m_Params.body.addGravity * m_pBody->GetMass(), HL2B3( gravity ) ), false );
	}
}

//-------------------------------------------------------------------------------------------------
// Friction snapshot
//-------------------------------------------------------------------------------------------------

CBox3DFrictionSnapshot::CBox3DFrictionSnapshot( CBox3DPhysicsObject *pObject )
	: m_pObject( pObject )
	, m_nIndex( 0 )
{
	const b3BodyId body = pObject->GetBody();
	if ( !b3Body_IsValid( body ) )
		return;

	const int capacity = b3Body_GetContactCapacity( body );
	if ( capacity <= 0 )
		return;

	CUtlVector< b3ContactData > contacts;
	contacts.SetCount( capacity );
	const int count = b3Body_GetContactData( body, contacts.Base(), capacity );

	const float invDt = 1.0f / MAX( pObject->GetEnvironment()->GetSimulationTimestep(), 0.001f );

	for ( int i = 0; i < count; i++ )
	{
		const b3ContactData &data = contacts[i];
		if ( !b3Shape_IsValid( data.shapeIdA ) || !b3Shape_IsValid( data.shapeIdB ) )
			continue;

		CBox3DPhysicsObject *pA = GetShapeObject( data.shapeIdA );
		CBox3DPhysicsObject *pB = GetShapeObject( data.shapeIdB );
		const bool bSelfIsA = ( pA == pObject );
		CBox3DPhysicsObject *pOther = bSelfIsA ? pB : pA;
		if ( !pOther )
			continue;

		const b3ShapeId selfShape = bSelfIsA ? data.shapeIdA : data.shapeIdB;
		const b3ShapeId otherShape = bSelfIsA ? data.shapeIdB : data.shapeIdA;
		const b3Vec3 centerA = b3Body_GetWorldCenter( b3Shape_GetBody( data.shapeIdA ) );

		for ( int m = 0; m < data.manifoldCount; m++ )
		{
			const b3Manifold &manifold = data.manifolds[m];
			if ( manifold.pointCount <= 0 )
				continue;

			// One entry per manifold, at the deepest point
			int best = 0;
			float impulse = 0.0f;
			for ( int p = 0; p < manifold.pointCount; p++ )
			{
				impulse += manifold.points[p].totalNormalImpulse;
				if ( manifold.points[p].separation < manifold.points[best].separation )
					best = p;
			}

			Contact_t &contact = m_Contacts[m_Contacts.AddToTail()];
			contact.pOther = pOther;
			contact.materialSelf = pObject->GetMaterialIndexForShape( selfShape );
			contact.materialOther = pOther->GetMaterialIndexForShape( otherShape );
			contact.point = B32HL( b3Add( centerA, manifold.points[best].anchorA ) );
			// Manifold normal points A -> B, we want this object -> other
			contact.normal = ToHL( bSelfIsA ? manifold.normal : b3Neg( manifold.normal ) );
			contact.normalForce = B32HL( impulse * invDt );
			contact.friction = b3Shape_GetFriction( selfShape ) * b3Shape_GetFriction( otherShape );
			contact.bMarked = false;
		}
	}
}

bool CBox3DFrictionSnapshot::IsValid()
{
	return m_nIndex < m_Contacts.Count();
}

IPhysicsObject *CBox3DFrictionSnapshot::GetObject( int index )
{
	if ( !IsValid() )
		return NULL;
	return index == 0 ? (IPhysicsObject *)m_pObject : (IPhysicsObject *)m_Contacts[m_nIndex].pOther;
}

int CBox3DFrictionSnapshot::GetMaterial( int index )
{
	if ( !IsValid() )
		return 0;
	return index == 0 ? m_Contacts[m_nIndex].materialSelf : m_Contacts[m_nIndex].materialOther;
}

void CBox3DFrictionSnapshot::GetContactPoint( Vector &out )
{
	out = IsValid() ? m_Contacts[m_nIndex].point : vec3_origin;
}

void CBox3DFrictionSnapshot::GetSurfaceNormal( Vector &out )
{
	out = IsValid() ? m_Contacts[m_nIndex].normal : Vector( 0, 0, 1 );
}

float CBox3DFrictionSnapshot::GetNormalForce()
{
	return IsValid() ? m_Contacts[m_nIndex].normalForce : 0.0f;
}

float CBox3DFrictionSnapshot::GetEnergyAbsorbed()
{
	return 0.0f;
}

void CBox3DFrictionSnapshot::RecomputeFriction()
{
}

void CBox3DFrictionSnapshot::ClearFrictionForce()
{
}

void CBox3DFrictionSnapshot::MarkContactForDelete()
{
	if ( IsValid() )
		m_Contacts[m_nIndex].bMarked = true;
}

void CBox3DFrictionSnapshot::DeleteAllMarkedContacts( bool wakeObjects )
{
	// Box3D rebuilds contacts every step, re-evaluating the filter is the closest equivalent
	for ( int i = 0; i < m_Contacts.Count(); i++ )
	{
		if ( !m_Contacts[i].bMarked )
			continue;

		m_pObject->RecheckContactPoints();
		if ( wakeObjects )
		{
			m_pObject->Wake();
			m_Contacts[i].pOther->Wake();
		}
	}
}

void CBox3DFrictionSnapshot::NextFrictionData()
{
	m_nIndex++;
}

float CBox3DFrictionSnapshot::GetFrictionCoefficient()
{
	return IsValid() ? m_Contacts[m_nIndex].friction : 0.0f;
}
