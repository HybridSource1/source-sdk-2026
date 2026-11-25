//=================================================================================================
//
// VPhysics Box3D - constraints, constraint groups and springs
//
// Mapping to Box3D joints:
//   hinge      -> revolute joint (z axis = hinge axis)
//   ballsocket -> spherical joint
//   fixed      -> weld joint
//   sliding    -> prismatic joint (x axis = slide axis)
//   length     -> distance joint (rope when minLength < totalLength)
//   ragdoll    -> weld / revolute / spherical with cone + twist limits, depending on free axes
//   pulley     -> solved by the backend each step (Box3D has no pulley joint)
//
//=================================================================================================

#include "b3vp_base.h"
#include "b3vp_constraints.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

static ConVar box3d_ragdoll_friction_scale( "box3d_ragdoll_friction_scale", "1.0", 0, "Scale applied to ragdoll joint friction" );
static ConVar box3d_ragdoll_min_friction( "box3d_ragdoll_min_friction", "0.05", 0, "Minimum ragdoll joint friction torque (N*m), keeps limbs from flopping forever" );
static ConVar box3d_break_force_scale( "box3d_break_force_scale", "1.0", 0, "Scale applied to constraint break force limits" );
static ConVar box3d_break_torque_scale( "box3d_break_torque_scale", "1.0", 0, "Scale applied to constraint break torque limits" );

static const float B3VP_MAX_LIMIT_ANGLE = 0.99f * M_PI_F;

//-------------------------------------------------------------------------------------------------
// Helpers
//-------------------------------------------------------------------------------------------------

// Rotation that takes +Z onto the given basis axis (0 = X, 1 = Y, 2 = Z)
static b3Quat ZToAxisRotation( int axis )
{
	switch ( axis )
	{
	case 0: return b3MakeQuatFromAxisAngle( b3Vec3_axisY, 0.5f * M_PI_F );		// Ry(+90): z -> x
	case 1: return b3MakeQuatFromAxisAngle( b3Vec3_axisX, -0.5f * M_PI_F );	// Rx(-90): z -> y
	default: return b3Quat_identity;
	}
}

static b3Transform RotateFrame( const b3Transform &frame, const b3Quat &rotation )
{
	b3Transform out;
	out.p = frame.p;
	out.q = b3NormalizeQuat( b3MulQuat( frame.q, rotation ) );
	return out;
}

static void ClampAngleRange( float &lower, float &upper )
{
	lower = clamp( lower, -B3VP_MAX_LIMIT_ANGLE, B3VP_MAX_LIMIT_ANGLE );
	upper = clamp( upper, -B3VP_MAX_LIMIT_ANGLE, B3VP_MAX_LIMIT_ANGLE );
	if ( lower > upper )
		V_swap( lower, upper );
}

// Break limits: the game passes kg-ish values (lbs converted to kg), treat them as a weight
static float BreakForceToSI( float forceLimit )
{
	return forceLimit > 0.0f ? forceLimit * 9.81f * box3d_break_force_scale.GetFloat() : 0.0f;
}

static float BreakTorqueToSI( float torqueLimit )
{
	return torqueLimit > 0.0f ? torqueLimit * 9.81f * 0.3048f * box3d_break_torque_scale.GetFloat() : 0.0f;
}

//-------------------------------------------------------------------------------------------------
// CBox3DConstraint
//-------------------------------------------------------------------------------------------------

CBox3DConstraint::CBox3DConstraint( CBox3DPhysicsEnvironment *pEnv, CBox3DPhysicsObject *pRef, CBox3DPhysicsObject *pAtt )
	: m_pSavedRef( NULL )
	, m_pSavedAtt( NULL )
	, m_pSavedGroup( NULL )
	, m_bSavedActive( false )
	, m_pEnv( pEnv )
	, m_pRef( NULL )
	, m_pAtt( NULL )
	, m_pGroup( NULL )
	, m_pGameData( NULL )
	, m_bActive( false )
	, m_bPulleyRegistered( false )
{
	m_Joint = b3_nullJointId;
	InitDescDefaults();
	BindObjects( pRef, pAtt );
}

CBox3DConstraint::~CBox3DConstraint()
{
	DestroyJoint();

	if ( m_pRef )
		m_pRef->RemoveConstraint( this );
	if ( m_pAtt && m_pAtt != m_pRef )
		m_pAtt->RemoveConstraint( this );

	if ( m_pGroup )
		m_pGroup->RemoveConstraint( this );
}

void CBox3DConstraint::BindObjects( CBox3DPhysicsObject *pRef, CBox3DPhysicsObject *pAtt )
{
	m_pRef = pRef;
	m_pAtt = pAtt;
	if ( m_pRef )
		m_pRef->AddConstraint( this );
	if ( m_pAtt && m_pAtt != m_pRef )
		m_pAtt->AddConstraint( this );
}

void CBox3DConstraint::InitDescDefaults()
{
	memset( &m_Desc, 0, sizeof( m_Desc ) );
	m_Desc.frameA = b3Transform_identity;
	m_Desc.frameB = b3Transform_identity;
	m_Desc.pulleyRatio = 1.0f;
	m_Desc.breakable.Defaults();
	SetIdentityMatrix( m_Desc.constraintToReference );
	SetIdentityMatrix( m_Desc.constraintToAttached );
}

void CBox3DConstraint::SetGroup( CBox3DConstraintGroup *pGroup )
{
	if ( m_pGroup == pGroup )
		return;
	if ( m_pGroup )
		m_pGroup->RemoveConstraint( this );
	m_pGroup = pGroup;
	if ( m_pGroup )
		m_pGroup->AddConstraint( this );
}

b3BodyId CBox3DConstraint::BodyOf( CBox3DPhysicsObject *pObject ) const
{
	return pObject ? pObject->GetBody() : m_pEnv->GetAnchorBody();
}

void CBox3DConstraint::FinishInit( CBox3DConstraintGroup *pGroup, bool bActive )
{
	m_Desc.breakForce = BreakForceToSI( m_Desc.breakable.forceLimit );
	m_Desc.breakTorque = BreakTorqueToSI( m_Desc.breakable.torqueLimit );

	SetGroup( pGroup );

	// Grouped constraints wait for the group to activate them
	if ( bActive && !pGroup )
		Activate();
}

static b3Transform BodyTransform( CBox3DPhysicsObject *pObject )
{
	if ( !pObject || !b3Body_IsValid( pObject->GetBody() ) )
		return b3Transform_identity;
	return b3Body_GetTransform( pObject->GetBody() );
}

void CBox3DConstraint::InitHinge( CBox3DConstraintGroup *pGroup, const constraint_hingeparams_t &hinge )
{
	m_Desc.sourceType = B3VP_CONSTRAINT_HINGE;
	m_Desc.kind = B3VP_JOINT_REVOLUTE;
	m_Desc.breakable = hinge.constraint;

	Vector axis = hinge.worldAxisDirection;
	if ( VectorNormalize( axis ) <= 0.0f )
		axis.Init( 0, 0, 1 );

	b3Transform world;
	world.p = HL2B3( hinge.worldPosition );
	world.q = B3QuatFromZAxis( ToB3( axis ) );

	m_Desc.frameA = b3InvMulTransforms( BodyTransform( m_pRef ), world );
	m_Desc.frameB = b3InvMulTransforms( BodyTransform( m_pAtt ), world );

	// Source hinge angles run clockwise around the axis
	if ( hinge.hingeAxis.minRotation != hinge.hingeAxis.maxRotation )
	{
		m_Desc.enableLimit = true;
		m_Desc.lower = DEG2RAD( -hinge.hingeAxis.maxRotation );
		m_Desc.upper = DEG2RAD( -hinge.hingeAxis.minRotation );
		ClampAngleRange( m_Desc.lower, m_Desc.upper );
	}

	if ( hinge.hingeAxis.torque != 0.0f )
	{
		m_Desc.enableMotor = true;
		m_Desc.motorSpeed = -DEG2RAD( hinge.hingeAxis.angularVelocity );
		m_Desc.maxMotorForce = HLTorque2B3( fabsf( hinge.hingeAxis.torque ) );
	}

	FinishInit( pGroup, hinge.constraint.isActive );
}

void CBox3DConstraint::InitRagdoll( CBox3DConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll )
{
	m_Desc.sourceType = B3VP_CONSTRAINT_RAGDOLL;
	m_Desc.breakable = ragdoll.constraint;
	m_Desc.constraintToReference = ragdoll.constraintToReference;
	m_Desc.constraintToAttached = ragdoll.constraintToAttached;
	m_Desc.bHasRagdollTransforms = true;

	const b3Transform frameA = MatrixToB3Transform( ragdoll.constraintToReference );
	const b3Transform frameB = MatrixToB3Transform( ragdoll.constraintToAttached );

	// Per-axis limits in radians, flipped for the clockwise convention used by the .phy data
	float lower[3], upper[3];
	int freeAxes[3];
	int freeCount = 0;
	float frictionSum = 0.0f;
	for ( int i = 0; i < 3; i++ )
	{
		float lo = ragdoll.axes[i].minRotation;
		float hi = ragdoll.axes[i].maxRotation;
		if ( ragdoll.useClockwiseRotations )
		{
			const float tmp = lo;
			lo = -hi;
			hi = -tmp;
		}
		lower[i] = DEG2RAD( lo );
		upper[i] = DEG2RAD( hi );
		if ( hi - lo > 0.5f )
			freeAxes[freeCount++] = i;
		frictionSum += fabsf( ragdoll.axes[i].torque );
	}

	const float friction = MAX( HLTorque2B3( frictionSum / 3.0f ) * box3d_ragdoll_friction_scale.GetFloat(), box3d_ragdoll_min_friction.GetFloat() );

	if ( freeCount == 0 )
	{
		m_Desc.kind = B3VP_JOINT_WELD;
		m_Desc.frameA = frameA;
		m_Desc.frameB = frameB;
	}
	else if ( freeCount == 1 )
	{
		const int axis = freeAxes[0];
		const b3Quat rotation = ZToAxisRotation( axis );
		m_Desc.kind = B3VP_JOINT_REVOLUTE;
		m_Desc.frameA = RotateFrame( frameA, rotation );
		m_Desc.frameB = RotateFrame( frameB, rotation );
		m_Desc.enableLimit = true;
		m_Desc.lower = lower[axis];
		m_Desc.upper = upper[axis];
		ClampAngleRange( m_Desc.lower, m_Desc.upper );
		m_Desc.enableMotor = friction > 0.0f;
		m_Desc.motorSpeed = 0.0f;
		m_Desc.maxMotorForce = friction;
	}
	else
	{
		// Twist around the constraint X axis, a circular cone for the other two
		const b3Quat rotation = ZToAxisRotation( 0 );
		m_Desc.kind = B3VP_JOINT_SPHERICAL;
		m_Desc.frameA = RotateFrame( frameA, rotation );
		m_Desc.frameB = RotateFrame( frameB, rotation );

		m_Desc.twistLower = lower[0];
		m_Desc.twistUpper = upper[0];
		ClampAngleRange( m_Desc.twistLower, m_Desc.twistUpper );
		m_Desc.enableTwist = ( upper[0] - lower[0] ) < 2.0f * B3VP_MAX_LIMIT_ANGLE;

		const float swingY = MAX( fabsf( lower[1] ), fabsf( upper[1] ) );
		const float swingZ = MAX( fabsf( lower[2] ), fabsf( upper[2] ) );
		m_Desc.coneAngle = clamp( 0.5f * ( swingY + swingZ ), DEG2RAD( 1.0f ), M_PI_F );
		m_Desc.enableCone = m_Desc.coneAngle < M_PI_F;
		m_Desc.frictionTorque = friction;
	}

	FinishInit( pGroup, ragdoll.isActive && ragdoll.constraint.isActive );
}

void CBox3DConstraint::InitFixed( CBox3DConstraintGroup *pGroup, const constraint_fixedparams_t &fixed )
{
	m_Desc.sourceType = B3VP_CONSTRAINT_FIXED;
	m_Desc.kind = B3VP_JOINT_WELD;
	m_Desc.breakable = fixed.constraint;

	// attachedRefXform: attached object space -> reference object space
	m_Desc.frameA = MatrixToB3Transform( fixed.attachedRefXform );
	m_Desc.frameB = b3Transform_identity;

	FinishInit( pGroup, fixed.constraint.isActive );
}

void CBox3DConstraint::InitSliding( CBox3DConstraintGroup *pGroup, const constraint_slidingparams_t &sliding )
{
	m_Desc.sourceType = B3VP_CONSTRAINT_SLIDING;
	m_Desc.kind = B3VP_JOINT_PRISMATIC;
	m_Desc.breakable = sliding.constraint;

	Vector axis = sliding.slideAxisRef;
	if ( VectorNormalize( axis ) <= 0.0f )
		axis.Init( 1, 0, 0 );

	const b3Transform attToRef = MatrixToB3Transform( sliding.attachedRefXform );
	const b3Vec3 axisRef = ToB3( axis );
	const b3Vec3 hint = fabsf( axisRef.z ) < 0.9f ? b3Vec3_axisZ : b3Vec3_axisX;

	b3Transform frameA;
	frameA.p = attToRef.p;
	frameA.q = B3QuatFromAxes( axisRef, b3Cross( hint, axisRef ) );
	m_Desc.frameA = frameA;
	m_Desc.frameB = b3InvMulTransforms( attToRef, frameA );

	if ( sliding.limitMin != sliding.limitMax )
	{
		m_Desc.enableLimit = true;
		m_Desc.lower = HL2B3( MIN( sliding.limitMin, sliding.limitMax ) );
		m_Desc.upper = HL2B3( MAX( sliding.limitMin, sliding.limitMax ) );
	}

	if ( sliding.velocity != 0.0f || sliding.friction != 0.0f )
	{
		m_Desc.enableMotor = true;
		m_Desc.motorSpeed = HL2B3( sliding.velocity );
		m_Desc.maxMotorForce = HLForce2B3( fabsf( sliding.friction ) );
	}

	FinishInit( pGroup, sliding.constraint.isActive );
}

void CBox3DConstraint::InitBallsocket( CBox3DConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket )
{
	m_Desc.sourceType = B3VP_CONSTRAINT_BALLSOCKET;
	m_Desc.kind = B3VP_JOINT_SPHERICAL;
	m_Desc.breakable = ballsocket.constraint;

	m_Desc.frameA.p = HL2B3( ballsocket.constraintPosition[0] );
	m_Desc.frameA.q = b3Quat_identity;
	m_Desc.frameB.p = HL2B3( ballsocket.constraintPosition[1] );
	m_Desc.frameB.q = b3Quat_identity;

	FinishInit( pGroup, ballsocket.constraint.isActive );
}

void CBox3DConstraint::InitPulley( CBox3DConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley )
{
	m_Desc.sourceType = B3VP_CONSTRAINT_PULLEY;
	m_Desc.kind = B3VP_JOINT_PULLEY;
	m_Desc.breakable = pulley.constraint;

	for ( int i = 0; i < 2; i++ )
	{
		m_Desc.pulleyPoint[i] = HL2B3( pulley.pulleyPosition[i] );
		m_Desc.pulleyAnchor[i] = HL2B3( pulley.objectPosition[i] );
	}
	m_Desc.pulleyLength = HL2B3( pulley.totalLength );
	m_Desc.pulleyRatio = pulley.gearRatio > 0.0f ? pulley.gearRatio : 1.0f;
	m_Desc.pulleyRigid = pulley.isRigid;

	FinishInit( pGroup, pulley.constraint.isActive );
}

void CBox3DConstraint::InitLength( CBox3DConstraintGroup *pGroup, const constraint_lengthparams_t &length )
{
	m_Desc.sourceType = B3VP_CONSTRAINT_LENGTH;
	m_Desc.kind = B3VP_JOINT_DISTANCE;
	m_Desc.breakable = length.constraint;

	m_Desc.frameA.p = HL2B3( length.objectPosition[0] );
	m_Desc.frameA.q = b3Quat_identity;
	m_Desc.frameB.p = HL2B3( length.objectPosition[1] );
	m_Desc.frameB.q = b3Quat_identity;
	m_Desc.length = HL2B3( MAX( length.totalLength, 0.01f ) );
	m_Desc.lower = HL2B3( MAX( length.minLength, 0.0f ) );
	m_Desc.upper = m_Desc.length;
	m_Desc.distanceRigid = length.minLength >= length.totalLength - 0.01f;

	FinishInit( pGroup, length.constraint.isActive );
}

void CBox3DConstraint::InitFromDesc( const B3VPJointDesc_t &desc, CBox3DConstraintGroup *pGroup, bool bActive )
{
	m_Desc = desc;
	SetGroup( pGroup );
	if ( bActive )
		Activate();
}

//-------------------------------------------------------------------------------------------------
// Joint lifetime
//-------------------------------------------------------------------------------------------------

void CBox3DConstraint::CreateJoint()
{
	if ( b3Joint_IsValid( m_Joint ) || m_bPulleyRegistered )
		return;

	// Objects that were destroyed leave the constraint dead until the game removes it
	if ( !m_pRef || !m_pAtt )
		return;

	const b3BodyId bodyA = BodyOf( m_pRef );
	const b3BodyId bodyB = BodyOf( m_pAtt );
	if ( !b3Body_IsValid( bodyA ) || !b3Body_IsValid( bodyB ) )
		return;

	if ( m_Desc.kind == B3VP_JOINT_PULLEY )
	{
		m_pEnv->AddController( this );
		m_bPulleyRegistered = true;
		return;
	}

	b3JointDef base = b3DefaultRevoluteJointDef().base;
	base.userData = this;
	base.bodyIdA = bodyA;
	base.bodyIdB = bodyB;
	base.localFrameA = m_Desc.frameA;
	base.localFrameB = m_Desc.frameB;
	base.collideConnected = false;
	base.forceThreshold = m_Desc.breakForce > 0.0f ? m_Desc.breakForce : FLT_MAX;
	base.torqueThreshold = m_Desc.breakTorque > 0.0f ? m_Desc.breakTorque : FLT_MAX;

	const b3WorldId world = m_pEnv->GetWorld();

	switch ( m_Desc.kind )
	{
	case B3VP_JOINT_WELD:
		{
			b3WeldJointDef def = b3DefaultWeldJointDef();
			def.base = base;
			def.linearHertz = 0.0f;
			def.angularHertz = 0.0f;
			m_Joint = b3CreateWeldJoint( world, &def );
		}
		break;

	case B3VP_JOINT_REVOLUTE:
		{
			b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
			def.base = base;
			def.enableLimit = m_Desc.enableLimit;
			def.lowerAngle = m_Desc.lower;
			def.upperAngle = m_Desc.upper;
			def.enableMotor = m_Desc.enableMotor;
			def.motorSpeed = m_Desc.motorSpeed;
			def.maxMotorTorque = m_Desc.maxMotorForce;
			m_Joint = b3CreateRevoluteJoint( world, &def );
		}
		break;

	case B3VP_JOINT_SPHERICAL:
		{
			b3SphericalJointDef def = b3DefaultSphericalJointDef();
			def.base = base;
			def.enableConeLimit = m_Desc.enableCone;
			def.coneAngle = m_Desc.coneAngle;
			def.enableTwistLimit = m_Desc.enableTwist;
			def.lowerTwistAngle = m_Desc.twistLower;
			def.upperTwistAngle = m_Desc.twistUpper;
			def.enableMotor = m_Desc.frictionTorque > 0.0f;
			def.maxMotorTorque = m_Desc.frictionTorque;
			def.motorVelocity = b3Vec3_zero;
			m_Joint = b3CreateSphericalJoint( world, &def );
		}
		break;

	case B3VP_JOINT_PRISMATIC:
		{
			b3PrismaticJointDef def = b3DefaultPrismaticJointDef();
			def.base = base;
			def.enableLimit = m_Desc.enableLimit;
			def.lowerTranslation = m_Desc.lower;
			def.upperTranslation = m_Desc.upper;
			def.enableMotor = m_Desc.enableMotor;
			def.motorSpeed = m_Desc.motorSpeed;
			def.maxMotorForce = m_Desc.maxMotorForce;
			m_Joint = b3CreatePrismaticJoint( world, &def );
		}
		break;

	case B3VP_JOINT_DISTANCE:
		{
			b3DistanceJointDef def = b3DefaultDistanceJointDef();
			def.base = base;
			def.length = m_Desc.length;
			if ( m_Desc.distanceRigid )
			{
				def.enableSpring = false;
				def.enableLimit = false;
			}
			else
			{
				// A slack rope: no spring force, only the length range
				def.enableSpring = true;
				def.hertz = 0.0f;
				def.dampingRatio = 0.0f;
				def.enableLimit = true;
				def.minLength = m_Desc.lower;
				def.maxLength = m_Desc.upper;
			}
			m_Joint = b3CreateDistanceJoint( world, &def );
		}
		break;

	default:
		break;
	}

	if ( b3Joint_IsValid( m_Joint ) )
	{
		m_pEnv->RegisterJoint( m_Joint, this );
		b3Joint_WakeBodies( m_Joint );
	}
}

void CBox3DConstraint::DestroyJoint()
{
	if ( b3Joint_IsValid( m_Joint ) )
	{
		m_pEnv->UnregisterJoint( m_Joint );
		b3DestroyJoint( m_Joint, true );
	}
	m_Joint = b3_nullJointId;

	if ( m_bPulleyRegistered )
	{
		m_pEnv->RemoveController( this );
		m_bPulleyRegistered = false;
	}
}

void CBox3DConstraint::Activate( void )
{
	if ( m_bActive )
		return;
	m_bActive = true;
	CreateJoint();
}

void CBox3DConstraint::Deactivate( void )
{
	if ( !m_bActive )
		return;
	m_bActive = false;
	DestroyJoint();
	if ( m_pRef ) m_pRef->Wake();
	if ( m_pAtt ) m_pAtt->Wake();
}

void CBox3DConstraint::OnBroken()
{
	Deactivate();
	m_pEnv->NotifyConstraintBroken( this );
}

void CBox3DConstraint::OnObjectDestroyed( CBox3DPhysicsObject *pObject )
{
	// Box3D destroys a body's joints with it, drop ours first so ids don't dangle
	DestroyJoint();

	if ( pObject == m_pRef )
		m_pRef = NULL;
	if ( pObject == m_pAtt )
		m_pAtt = NULL;
}

IPhysicsObject *CBox3DConstraint::GetReferenceObject( void ) const
{
	return m_pRef;
}

IPhysicsObject *CBox3DConstraint::GetAttachedObject( void ) const
{
	return m_pAtt;
}

void CBox3DConstraint::SetLinearMotor( float speed, float maxLinearImpulse )
{
	if ( m_Desc.kind != B3VP_JOINT_PRISMATIC )
		return;

	m_Desc.enableMotor = true;
	m_Desc.motorSpeed = HL2B3( speed );
	m_Desc.maxMotorForce = HLForce2B3( fabsf( maxLinearImpulse ) );

	if ( b3Joint_IsValid( m_Joint ) )
	{
		b3PrismaticJoint_EnableMotor( m_Joint, true );
		b3PrismaticJoint_SetMotorSpeed( m_Joint, m_Desc.motorSpeed );
		b3PrismaticJoint_SetMaxMotorForce( m_Joint, m_Desc.maxMotorForce );
		b3Joint_WakeBodies( m_Joint );
	}
}

void CBox3DConstraint::SetAngularMotor( float rotSpeed, float maxAngularImpulse )
{
	if ( m_Desc.kind == B3VP_JOINT_REVOLUTE )
	{
		m_Desc.enableMotor = true;
		m_Desc.motorSpeed = -DEG2RAD( rotSpeed );
		m_Desc.maxMotorForce = HLTorque2B3( fabsf( maxAngularImpulse ) );

		if ( b3Joint_IsValid( m_Joint ) )
		{
			b3RevoluteJoint_EnableMotor( m_Joint, true );
			b3RevoluteJoint_SetMotorSpeed( m_Joint, m_Desc.motorSpeed );
			b3RevoluteJoint_SetMaxMotorTorque( m_Joint, m_Desc.maxMotorForce );
			b3Joint_WakeBodies( m_Joint );
		}
	}
	else if ( m_Desc.kind == B3VP_JOINT_SPHERICAL )
	{
		// Ragdolls use this for joint friction
		m_Desc.frictionTorque = HLTorque2B3( fabsf( maxAngularImpulse ) );
		if ( b3Joint_IsValid( m_Joint ) )
		{
			b3SphericalJoint_EnableMotor( m_Joint, m_Desc.frictionTorque > 0.0f );
			b3SphericalJoint_SetMotorVelocity( m_Joint, b3Vec3_zero );
			b3SphericalJoint_SetMaxMotorTorque( m_Joint, m_Desc.frictionTorque );
		}
	}
}

void CBox3DConstraint::UpdateRagdollTransforms( const matrix3x4_t &constraintToReference, const matrix3x4_t &constraintToAttached )
{
	if ( m_Desc.sourceType != B3VP_CONSTRAINT_RAGDOLL )
		return;

	// Keep the axis remap we picked, only the frames move
	const b3Transform oldA = MatrixToB3Transform( m_Desc.constraintToReference );
	const b3Transform newA = MatrixToB3Transform( constraintToReference );
	const b3Transform newB = MatrixToB3Transform( constraintToAttached );
	const b3Quat remap = b3InvMulQuat( oldA.q, m_Desc.frameA.q );

	m_Desc.constraintToReference = constraintToReference;
	m_Desc.constraintToAttached = constraintToAttached;
	m_Desc.frameA = RotateFrame( newA, remap );
	m_Desc.frameB = RotateFrame( newB, remap );

	if ( b3Joint_IsValid( m_Joint ) )
	{
		b3Joint_SetLocalFrameA( m_Joint, m_Desc.frameA );
		b3Joint_SetLocalFrameB( m_Joint, m_Desc.frameB );
		b3Joint_WakeBodies( m_Joint );
	}
}

bool CBox3DConstraint::GetConstraintTransform( matrix3x4_t *pConstraintToReference, matrix3x4_t *pConstraintToAttached ) const
{
	if ( m_Desc.bHasRagdollTransforms )
	{
		if ( pConstraintToReference ) *pConstraintToReference = m_Desc.constraintToReference;
		if ( pConstraintToAttached ) *pConstraintToAttached = m_Desc.constraintToAttached;
		return true;
	}

	if ( pConstraintToReference ) B3TransformToMatrix( m_Desc.frameA, *pConstraintToReference );
	if ( pConstraintToAttached ) B3TransformToMatrix( m_Desc.frameB, *pConstraintToAttached );
	return true;
}

bool CBox3DConstraint::GetConstraintParams( constraint_breakableparams_t *pParams ) const
{
	if ( pParams )
		*pParams = m_Desc.breakable;
	return true;
}

void CBox3DConstraint::OutputDebugInfo()
{
	static const char *s_Types[] = { "none", "ragdoll", "hinge", "fixed", "sliding", "ballsocket", "pulley", "length" };
	const int type = clamp( m_Desc.sourceType, 0, (int)ARRAYSIZE( s_Types ) - 1 );
	Msg( "Box3D constraint: %s, %s, joint %s\n", s_Types[type], m_bActive ? "active" : "inactive", b3Joint_IsValid( m_Joint ) ? "valid" : "none" );
}

// Pulley: a one-sided velocity constraint on |a0 - p0| + ratio * |a1 - p1| <= length
void CBox3DConstraint::OnPreStep( float dt )
{
	if ( m_Desc.kind != B3VP_JOINT_PULLEY || !m_pRef || !m_pAtt || dt <= 0.0f )
		return;

	CBox3DPhysicsObject *pObjects[2] = { m_pRef, m_pAtt };
	b3Vec3 anchor[2], dir[2], r[2];
	float len[2];
	for ( int i = 0; i < 2; i++ )
	{
		const b3Transform xf = b3Body_GetTransform( pObjects[i]->GetBody() );
		anchor[i] = b3TransformPoint( xf, m_Desc.pulleyAnchor[i] );
		b3Vec3 d = b3Sub( m_Desc.pulleyPoint[i], anchor[i] );
		len[i] = b3Length( d );
		dir[i] = len[i] > 1e-5f ? b3MulSV( 1.0f / len[i], d ) : b3Vec3_axisZ;
		r[i] = b3Sub( anchor[i], pObjects[i]->GetWorldCenterOfMass() );
	}

	const float ratio = m_Desc.pulleyRatio;
	const float C = len[0] + ratio * len[1] - m_Desc.pulleyLength;
	if ( C <= 0.0f && !m_Desc.pulleyRigid )
		return;

	const float scale[2] = { 1.0f, ratio };
	float Cdot = 0.0f;
	float K = 0.0f;
	for ( int i = 0; i < 2; i++ )
	{
		if ( !pObjects[i]->IsDynamicBody() )
			continue;
		const b3Vec3 v = b3Body_GetWorldPointVelocity( pObjects[i]->GetBody(), anchor[i] );
		Cdot -= scale[i] * b3Dot( dir[i], v );

		const b3Vec3 rn = b3Cross( r[i], dir[i] );
		const float k = pObjects[i]->GetInvMass() + b3Dot( rn, b3MulMV( pObjects[i]->GetWorldInverseInertia(), rn ) );
		K += scale[i] * scale[i] * k;
	}

	if ( K <= 0.0f )
		return;

	const float bias = 0.2f * C / dt;
	float lambda = ( Cdot + bias ) / K;
	if ( !m_Desc.pulleyRigid )
		lambda = MAX( lambda, 0.0f );
	if ( lambda == 0.0f )
		return;

	for ( int i = 0; i < 2; i++ )
	{
		if ( pObjects[i]->IsDynamicBody() )
			b3Body_ApplyLinearImpulse( pObjects[i]->GetBody(), b3MulSV( lambda * scale[i], dir[i] ), anchor[i], true );
	}
}

//-------------------------------------------------------------------------------------------------
// Constraint groups
//-------------------------------------------------------------------------------------------------

CBox3DConstraintGroup::~CBox3DConstraintGroup()
{
	for ( int i = 0; i < m_Constraints.Count(); i++ )
		m_Constraints[i]->ClearGroup();
	m_Constraints.RemoveAll();
}

void CBox3DConstraintGroup::Activate()
{
	for ( int i = 0; i < m_Constraints.Count(); i++ )
		m_Constraints[i]->Activate();
}

void CBox3DConstraintGroup::AddConstraint( CBox3DConstraint *pConstraint )
{
	if ( m_Constraints.Find( pConstraint ) == m_Constraints.InvalidIndex() )
		m_Constraints.AddToTail( pConstraint );
}

void CBox3DConstraintGroup::RemoveConstraint( CBox3DConstraint *pConstraint )
{
	m_Constraints.FindAndRemove( pConstraint );
}

//-------------------------------------------------------------------------------------------------
// Springs
//-------------------------------------------------------------------------------------------------

CBox3DSpring::CBox3DSpring( CBox3DPhysicsEnvironment *pEnv, CBox3DPhysicsObject *pStart, CBox3DPhysicsObject *pEnd, const springparams_t *pParams )
	: m_pSavedStart( NULL )
	, m_pSavedEnd( NULL )
	, m_pEnv( pEnv )
	, m_pStart( NULL )
	, m_pEnd( NULL )
	, m_LocalStart( 0, 0, 0 )
	, m_LocalEnd( 0, 0, 0 )
	, m_flConstant( pParams->constant )
	, m_flNaturalLength( pParams->naturalLength )
	, m_flDamping( pParams->damping )
	, m_flRelativeDamping( pParams->relativeDamping )
	, m_bOnlyStretch( pParams->onlyStretch )
{
	if ( pParams->useLocalPositions )
	{
		m_LocalStart = pParams->startPosition;
		m_LocalEnd = pParams->endPosition;
	}
	else
	{
		if ( pStart ) pStart->WorldToLocal( &m_LocalStart, pParams->startPosition );
		if ( pEnd ) pEnd->WorldToLocal( &m_LocalEnd, pParams->endPosition );
	}

	BindObjects( pStart, pEnd );
	m_pEnv->AddController( this );
}

CBox3DSpring::CBox3DSpring( CBox3DPhysicsEnvironment *pEnv, const SaveData_t &data )
	: m_pSavedStart( NULL )
	, m_pSavedEnd( NULL )
	, m_pEnv( pEnv )
	, m_pStart( NULL )
	, m_pEnd( NULL )
	, m_LocalStart( data.localStart )
	, m_LocalEnd( data.localEnd )
	, m_flConstant( data.constant )
	, m_flNaturalLength( data.naturalLength )
	, m_flDamping( data.damping )
	, m_flRelativeDamping( data.relativeDamping )
	, m_bOnlyStretch( data.onlyStretch )
{
	m_pEnv->AddController( this );
}

CBox3DSpring::~CBox3DSpring()
{
	m_pEnv->RemoveController( this );
	if ( m_pStart ) m_pStart->RemoveListener( this );
	if ( m_pEnd ) m_pEnd->RemoveListener( this );
}

void CBox3DSpring::BindObjects( CBox3DPhysicsObject *pStart, CBox3DPhysicsObject *pEnd )
{
	m_pStart = pStart;
	m_pEnd = pEnd;
	if ( m_pStart ) m_pStart->AddListener( this );
	if ( m_pEnd ) m_pEnd->AddListener( this );
}

void CBox3DSpring::GetSaveData( SaveData_t &data ) const
{
	memset( &data, 0, sizeof( data ) );
	data.constant = m_flConstant;
	data.naturalLength = m_flNaturalLength;
	data.damping = m_flDamping;
	data.relativeDamping = m_flRelativeDamping;
	data.localStart = m_LocalStart;
	data.localEnd = m_LocalEnd;
	data.onlyStretch = m_bOnlyStretch;
}

void CBox3DSpring::OnObjectDestroyed( CBox3DPhysicsObject *pObject )
{
	if ( pObject == m_pStart ) m_pStart = NULL;
	if ( pObject == m_pEnd ) m_pEnd = NULL;
}

void CBox3DSpring::GetEndpoints( Vector *worldPositionStart, Vector *worldPositionEnd )
{
	if ( worldPositionStart )
	{
		if ( m_pStart ) m_pStart->LocalToWorld( worldPositionStart, m_LocalStart );
		else *worldPositionStart = m_LocalStart;
	}
	if ( worldPositionEnd )
	{
		if ( m_pEnd ) m_pEnd->LocalToWorld( worldPositionEnd, m_LocalEnd );
		else *worldPositionEnd = m_LocalEnd;
	}
}

void CBox3DSpring::SetSpringConstant( float flSpringConstant )
{
	m_flConstant = flSpringConstant;
	if ( m_pStart ) m_pStart->Wake();
	if ( m_pEnd ) m_pEnd->Wake();
}

void CBox3DSpring::SetSpringDamping( float flSpringDamping )
{
	m_flDamping = flSpringDamping;
}

void CBox3DSpring::SetSpringLength( float flSpringLength )
{
	m_flNaturalLength = flSpringLength;
	if ( m_pStart ) m_pStart->Wake();
	if ( m_pEnd ) m_pEnd->Wake();
}

void CBox3DSpring::OnPreStep( float dt )
{
	if ( !m_pStart || !m_pEnd || dt <= 0.0f )
		return;

	const bool bStartMoves = m_pStart->IsMoveable() && m_pStart->IsDynamicBody();
	const bool bEndMoves = m_pEnd->IsMoveable() && m_pEnd->IsDynamicBody();
	if ( !bStartMoves && !bEndMoves )
		return;
	if ( ( !bStartMoves || m_pStart->IsAsleep() ) && ( !bEndMoves || m_pEnd->IsAsleep() ) )
		return;

	Vector start, end;
	GetEndpoints( &start, &end );

	Vector delta = end - start;
	const float length = VectorNormalize( delta );
	if ( length < 1e-3f )
		return;

	const float stretch = length - m_flNaturalLength;
	if ( m_bOnlyStretch && stretch <= 0.0f )
		return;

	Vector vStart, vEnd;
	m_pStart->GetVelocityAtPoint( start, &vStart );
	m_pEnd->GetVelocityAtPoint( end, &vEnd );
	const float relativeSpeed = DotProduct( vEnd - vStart, delta ); // in/s, positive when stretching

	// Reduced mass of the moving ends
	const float invMassStart = bStartMoves ? m_pStart->GetInvMass() : 0.0f;
	const float invMassEnd = bEndMoves ? m_pEnd->GetInvMass() : 0.0f;
	const float invMassSum = invMassStart + invMassEnd;
	if ( invMassSum <= 0.0f )
		return;
	const float reducedMass = 1.0f / invMassSum;

	// N, positive pulls the ends together
	const float force = m_flConstant * HL2B3( stretch ) + ( m_flDamping + m_flRelativeDamping ) * reducedMass * HL2B3( relativeSpeed );

	// Never let damping reverse the motion within one step
	float impulse = force * dt;
	const float maxImpulse = reducedMass * fabsf( HL2B3( relativeSpeed ) ) + fabsf( m_flConstant * HL2B3( stretch ) ) * dt;
	impulse = clamp( impulse, -maxImpulse, maxImpulse );

	const b3Vec3 dir = ToB3( delta );
	if ( bStartMoves )
		b3Body_ApplyLinearImpulse( m_pStart->GetBody(), b3MulSV( impulse, dir ), HL2B3( start ), true );
	if ( bEndMoves )
		b3Body_ApplyLinearImpulse( m_pEnd->GetBody(), b3MulSV( -impulse, dir ), HL2B3( end ), true );
}
