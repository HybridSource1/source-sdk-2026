//=================================================================================================
//
// VPhysics Box3D - IPhysicsObject
//
//=================================================================================================

#include "b3vp_base.h"
#include "b3vp_object.h"
#include "b3vp_environment.h"
#include "b3vp_collide.h"
#include "b3vp_surfaceprops.h"
#include "b3vp_controllers.h"
#include "b3vp_constraints.h"

#include "bspflags.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

static ConVar box3d_min_inertia_ratio( "box3d_min_inertia_ratio", "0.05", 0, "Smallest allowed ratio between an object's smallest and largest principal inertia when the game doesn't specify one" );

static const b3Vec3 s_InchesToMetres = { B3VP_M_PER_IN, B3VP_M_PER_IN, B3VP_M_PER_IN };

//-------------------------------------------------------------------------------------------------

CBox3DPhysicsObject::CBox3DPhysicsObject()
	: m_bWasAwake( false )
	, m_nActiveFrame( -1 )
	, m_pEnvironment( NULL )
	, m_pCollide( NULL )
	, m_flSphereRadius( 0.0f )
	, m_pGameData( NULL )
	, m_gameFlags( 0 )
	, m_gameIndex( 0 )
	, m_callbackFlags( CALLBACK_GLOBAL_COLLISION | CALLBACK_GLOBAL_FRICTION | CALLBACK_FLUID_TOUCH | CALLBACK_GLOBAL_TOUCH | CALLBACK_GLOBAL_COLLIDE_STATIC | CALLBACK_DO_FLUID_SIMULATION )
	, m_materialIndex( 0 )
	, m_contents( CONTENTS_SOLID )
	, m_bStatic( false )
	, m_bMotionEnabled( true )
	, m_bGravityEnabled( true )
	, m_bTempGravityDisabled( false )
	, m_bDragEnabled( true )
	, m_bCollisionsEnabled( true )
	, m_bTrigger( false )
	, m_bUsePreStepVelocity( false )
	, m_flMass( 1.0f )
	, m_flLinearDamping( 0.0f )
	, m_flAngularDamping( 0.0f )
	, m_flDragCoefficient( 1.0f )
	, m_flAngularDragCoefficient( 1.0f )
	, m_flBuoyancyRatio( 1.0f )
	, m_flVolume( 0.0f )
	, m_DragAreas( 0, 0, 0 )
	, m_LastPosition( 0, 0, 0 )
	, m_LastAngles( 0, 0, 0 )
	, m_ImplicitVelocity( 0, 0, 0 )
	, m_ImplicitAngularVelocity( 0, 0, 0 )
	, m_pShadowController( NULL )
	, m_pPlayerController( NULL )
	, m_pFluidController( NULL )
	, m_nHingeAxis( 0 )
	, m_bHinged( false )
{
	m_Body = b3_nullBodyId;
	m_HingeJoint = b3_nullJointId;
	m_MassData.mass = 1.0f;
	m_MassData.center = b3Vec3_zero;
	m_MassData.inertia = b3Mat3_identity;
	m_PreStepLinearVelocity = b3Vec3_zero;
	m_PreStepAngularVelocity = b3Vec3_zero;
	m_szName[0] = '\0';
}

CBox3DPhysicsObject::~CBox3DPhysicsObject()
{
	NotifyDestroyed();

	if ( m_pShadowController )
	{
		CBox3DShadowController *pShadow = m_pShadowController;
		m_pShadowController = NULL;
		pShadow->DetachObject();
		m_pEnvironment->DestroyShadowController( pShadow );
	}

	if ( b3Joint_IsValid( m_HingeJoint ) )
		b3DestroyJoint( m_HingeJoint, false );

	if ( b3Body_IsValid( m_Body ) )
	{
		m_pEnvironment->OnObjectShapesRemoved( this, false );
		b3DestroyBody( m_Body );
	}

	if ( m_pCollide )
		const_cast< CPhysCollide * >( m_pCollide )->Release();
}

void CBox3DPhysicsObject::NotifyDestroyed()
{
	// Listeners may remove themselves while being notified
	CUtlVector< IBox3DObjectListener * > listeners;
	listeners.AddVectorToTail( m_Listeners );
	m_Listeners.RemoveAll();
	for ( int i = 0; i < listeners.Count(); i++ )
		listeners[i]->OnObjectDestroyed( this );

	CUtlVector< CBox3DConstraint * > constraints;
	constraints.AddVectorToTail( m_Constraints );
	m_Constraints.RemoveAll();
	for ( int i = 0; i < constraints.Count(); i++ )
		constraints[i]->OnObjectDestroyed( this );
}

bool CBox3DPhysicsObject::Init( CBox3DPhysicsEnvironment *pEnvironment, const CPhysCollide *pCollide, float sphereRadius,
								int materialIndex, const Vector &position, const QAngle &angles,
								const objectparams_t *pParams, bool bStatic )
{
	m_pEnvironment = pEnvironment;
	m_pCollide = pCollide;
	m_flSphereRadius = sphereRadius;
	m_materialIndex = materialIndex;
	m_bStatic = bStatic;

	if ( m_pCollide )
		const_cast< CPhysCollide * >( m_pCollide )->AddRef();

	float mass = 1.0f;
	float inertiaScale = 1.0f;
	float rotInertiaLimit = 0.0f;
	const Vector *pMassCenterOverride = NULL;

	if ( pParams )
	{
		m_pGameData = pParams->pGameData;
		mass = clamp( pParams->mass, VPHYSICS_MIN_MASS, VPHYSICS_MAX_MASS );
		inertiaScale = pParams->inertia > 0.0f ? pParams->inertia : 1.0f;
		rotInertiaLimit = pParams->rotInertiaLimit;
		pMassCenterOverride = pParams->massCenterOverride;
		m_flLinearDamping = pParams->damping;
		m_flAngularDamping = pParams->rotdamping;
		m_flDragCoefficient = pParams->dragCoefficient;
		m_flAngularDragCoefficient = pParams->dragCoefficient;
		m_bCollisionsEnabled = pParams->enableCollisions;
		m_flVolume = pParams->volume;
		if ( pParams->pName )
			Q_strncpy( m_szName, pParams->pName, sizeof( m_szName ) );
	}
	m_flMass = mass;

	if ( m_flVolume <= 0.0f )
	{
		if ( m_pCollide )
			m_flVolume = m_pCollide->m_flVolume;
		else if ( m_flSphereRadius > 0.0f )
			m_flVolume = ( 4.0f / 3.0f ) * M_PI_F * m_flSphereRadius * m_flSphereRadius * m_flSphereRadius;
	}

	if ( m_pCollide )
	{
		m_DragAreas = m_pCollide->m_OrthoAreas;
	}
	else
	{
		const float area = M_PI_F * m_flSphereRadius * m_flSphereRadius;
		m_DragAreas.Init( area, area, area );
	}

	// Buoyancy from density relative to water, like IVP
	if ( m_flVolume > 0.0f )
	{
		const float density = mass / HLVolume2B3( MAX( m_flVolume, 5.0f ) );
		const float materialDensity = CBox3DPhysicsSurfaceProps::GetInstance().GetDensity( m_materialIndex );
		m_flBuoyancyRatio = materialDensity > 0.0f ? density / materialDensity : 1.0f;
	}

	b3BodyDef def = b3DefaultBodyDef();
	def.type = bStatic ? b3_staticBody : b3_dynamicBody;
	def.position = HL2B3( position );
	def.rotation = AnglesToB3Quat( angles );
	def.linearDamping = m_flLinearDamping;
	def.angularDamping = m_flAngularDamping;
	def.gravityScale = 1.0f;
	def.userData = this;
	def.enableSleep = true;
	def.isAwake = false; // the game wakes objects that should start awake
	m_Body = b3CreateBody( m_pEnvironment->GetWorld(), &def );
	if ( !b3Body_IsValid( m_Body ) )
		return false;

	CreateShapes();

	if ( !m_bStatic )
	{
		ComputeMassProperties( mass, inertiaScale, rotInertiaLimit, pMassCenterOverride );
		ApplyMassData();
	}

	GetPosition( &m_LastPosition, &m_LastAngles );
	m_bWasAwake = false;
	return true;
}

//-------------------------------------------------------------------------------------------------
// Shapes
//-------------------------------------------------------------------------------------------------

static float B3VP_MixFriction( float frictionA, uint64_t userMaterialIdA, float frictionB, uint64_t userMaterialIdB )
{
	return frictionA * frictionB;
}

b3Filter CBox3DPhysicsObject::MakeFilter() const
{
	b3Filter filter = b3DefaultFilter();
	filter.categoryBits = m_bStatic ? B3VP_CATEGORY_STATIC : B3VP_CATEGORY_MOVING;
	filter.maskBits = m_bCollisionsEnabled ? B3VP_CATEGORY_ALL : 0;
	filter.groupIndex = 0;
	return filter;
}

b3ShapeDef CBox3DPhysicsObject::MakeShapeDef( int surfaceProp ) const
{
	CBox3DPhysicsSurfaceProps &props = CBox3DPhysicsSurfaceProps::GetInstance();

	b3ShapeDef def = b3DefaultShapeDef();
	def.userData = const_cast< CBox3DPhysicsObject * >( this );
	def.density = 1.0f;
	def.baseMaterial.friction = props.GetFriction( surfaceProp );
	def.baseMaterial.restitution = props.GetElasticity( surfaceProp );
	def.baseMaterial.userMaterialId = (uint64_t)MAX( surfaceProp, 0 );
	def.filter = MakeFilter();
	def.isSensor = m_bTrigger;
	def.enableSensorEvents = !m_bStatic || m_bTrigger; // static world isn't a trigger visitor
	def.enableContactEvents = true;
	def.enableHitEvents = true;
	def.enableCustomFiltering = true;
	def.enablePreSolveEvents = !m_bStatic && !m_bTrigger;
	def.invokeContactCreation = !m_bStatic || m_pEnvironment->HasSimulated();
	def.updateBodyMass = false;
	return def;
}

void CBox3DPhysicsObject::CreateShapes()
{
	CBox3DPhysicsSurfaceProps &props = CBox3DPhysicsSurfaceProps::GetInstance();

	if ( !m_pCollide )
	{
		if ( m_flSphereRadius > 0.0f )
		{
			b3Sphere sphere;
			sphere.center = b3Vec3_zero;
			sphere.radius = HL2B3( m_flSphereRadius );
			b3ShapeDef def = MakeShapeDef( m_materialIndex );
			b3ShapeId shape = b3CreateSphereShape( m_Body, &def, &sphere );
			if ( b3Shape_IsValid( shape ) )
				m_Shapes.AddToTail( shape );
		}
		return;
	}

	if ( m_pCollide->IsMesh() )
	{
		if ( !m_pCollide->m_pMesh )
			return;

		if ( !m_bStatic )
			DevWarning( B3VP_LOG_PREFIX "Mesh collide used on a moving object (%s), it will only collide as a static.\n", m_szName );

		b3SurfaceMaterial materials[128];
		for ( int i = 0; i < 128; i++ )
		{
			int surface = m_pCollide->m_nMeshSurfaceProp;
			if ( surface < 0 )
			{
				const int remapped = props.RemapWorldMaterial( i );
				surface = remapped >= 0 ? remapped : m_materialIndex;
			}
			materials[i] = b3DefaultSurfaceMaterial();
			materials[i].friction = props.GetFriction( surface );
			materials[i].restitution = props.GetElasticity( surface );
			materials[i].userMaterialId = (uint64_t)MAX( surface, 0 );
		}

		const int baseSurface = m_pCollide->m_nMeshSurfaceProp >= 0 ? m_pCollide->m_nMeshSurfaceProp : m_materialIndex;
		b3ShapeDef def = MakeShapeDef( baseSurface );
		def.materials = materials;
		def.materialCount = 128;
		b3ShapeId shape = b3CreateMeshShape( m_Body, &def, m_pCollide->m_pMesh, s_InchesToMetres );
		if ( b3Shape_IsValid( shape ) )
			m_Shapes.AddToTail( shape );
		return;
	}

	for ( int i = 0; i < m_pCollide->m_Convexes.Count(); i++ )
	{
		const CPhysConvex *pConvex = m_pCollide->m_Convexes[i];
		if ( !pConvex->m_pHull )
			continue;

		int surface = m_materialIndex;
		if ( pConvex->m_materialIndex > 0 )
		{
			const int remapped = props.RemapWorldMaterial( pConvex->m_materialIndex );
			if ( remapped >= 0 )
				surface = remapped;
		}

		b3ShapeDef def = MakeShapeDef( surface );
		b3ShapeId shape = b3CreateTransformedHullShape( m_Body, &def, pConvex->m_pHull, b3Transform_identity, s_InchesToMetres );
		if ( b3Shape_IsValid( shape ) )
			m_Shapes.AddToTail( shape );
	}
}

void CBox3DPhysicsObject::DestroyShapes()
{
	m_pEnvironment->OnObjectShapesRemoved( this, true );
	for ( int i = 0; i < m_Shapes.Count(); i++ )
	{
		if ( b3Shape_IsValid( m_Shapes[i] ) )
			b3DestroyShape( m_Shapes[i], false );
	}
	m_Shapes.RemoveAll();
}

int CBox3DPhysicsObject::GetMaterialIndexForShape( b3ShapeId shape ) const
{
	if ( !b3Shape_IsValid( shape ) )
		return m_materialIndex;
	return (int)b3Shape_GetSurfaceMaterial( shape ).userMaterialId;
}

//-------------------------------------------------------------------------------------------------
// Mass properties
//-------------------------------------------------------------------------------------------------

static b3Matrix3 MakeDiagonal( float x, float y, float z )
{
	b3Matrix3 m;
	m.cx = b3Vec3{ x, 0.0f, 0.0f };
	m.cy = b3Vec3{ 0.0f, y, 0.0f };
	m.cz = b3Vec3{ 0.0f, 0.0f, z };
	return m;
}

void CBox3DPhysicsObject::ComputeMassProperties( float mass, float inertiaScale, float rotInertiaLimit, const Vector *pMassCenterOverride )
{
	// Unit density gives volume in m^3 and inertia per unit density about the shape centroid
	b3Body_ApplyMassFromShapes( m_Body );
	b3MassData computed = b3Body_GetMassData( m_Body );

	b3MassData result;
	result.mass = mass;

	if ( computed.mass > 1e-9f )
	{
		const float scale = mass / computed.mass;
		result.center = computed.center;
		result.inertia.cx = b3MulSV( scale, computed.inertia.cx );
		result.inertia.cy = b3MulSV( scale, computed.inertia.cy );
		result.inertia.cz = b3MulSV( scale, computed.inertia.cz );
	}
	else
	{
		// No volume (mesh or degenerate). Use a box the size of the bounds.
		b3Vec3 size = b3Vec3{ 0.3f, 0.3f, 0.3f };
		b3Vec3 center = b3Vec3_zero;
		if ( m_pCollide )
		{
			size = b3MulSV( B3VP_M_PER_IN, b3Sub( m_pCollide->m_Bounds.upperBound, m_pCollide->m_Bounds.lowerBound ) );
			center = b3MulSV( B3VP_M_PER_IN, b3AABB_Center( m_pCollide->m_Bounds ) );
		}
		size = b3Max( size, b3Vec3{ 0.01f, 0.01f, 0.01f } );
		const float k = mass / 12.0f;
		result.center = center;
		result.inertia = MakeDiagonal( k * ( size.y * size.y + size.z * size.z ), k * ( size.x * size.x + size.z * size.z ), k * ( size.x * size.x + size.y * size.y ) );
	}

	// Centre of mass: game override, then the one stored in the .phy
	b3Vec3 newCenter = result.center;
	if ( pMassCenterOverride )
		newCenter = HL2B3( *pMassCenterOverride );
	else if ( m_pCollide && m_pCollide->m_bMassCenterSet )
		newCenter = HL2B3( Vector( m_pCollide->m_MassCenter.x, m_pCollide->m_MassCenter.y, m_pCollide->m_MassCenter.z ) );

	// Parallel axis theorem to move the inertia to the new centre
	const b3Vec3 d = b3Sub( newCenter, result.center );
	const float dd = b3Dot( d, d );
	if ( dd > 1e-12f )
	{
		result.inertia.cx = b3Add( result.inertia.cx, b3MulSV( mass, b3Vec3{ dd - d.x * d.x, -d.y * d.x, -d.z * d.x } ) );
		result.inertia.cy = b3Add( result.inertia.cy, b3MulSV( mass, b3Vec3{ -d.x * d.y, dd - d.y * d.y, -d.z * d.y } ) );
		result.inertia.cz = b3Add( result.inertia.cz, b3MulSV( mass, b3Vec3{ -d.x * d.z, -d.y * d.z, dd - d.z * d.z } ) );
	}
	result.center = newCenter;

	// Game inertia multiplier
	result.inertia.cx = b3MulSV( inertiaScale, result.inertia.cx );
	result.inertia.cy = b3MulSV( inertiaScale, result.inertia.cy );
	result.inertia.cz = b3MulSV( inertiaScale, result.inertia.cz );

	// Keep thin objects from spinning wildly: no principal axis below a fraction of the largest
	const float limit = rotInertiaLimit > 0.0f ? rotInertiaLimit : box3d_min_inertia_ratio.GetFloat();
	const float maxDiag = MAX( result.inertia.cx.x, MAX( result.inertia.cy.y, result.inertia.cz.z ) );
	const float minDiag = maxDiag * limit;
	result.inertia.cx.x = MAX( result.inertia.cx.x, minDiag );
	result.inertia.cy.y = MAX( result.inertia.cy.y, minDiag );
	result.inertia.cz.z = MAX( result.inertia.cz.z, minDiag );

	m_MassData = result;
}

void CBox3DPhysicsObject::ApplyMassData()
{
	if ( !b3Body_IsValid( m_Body ) || b3Body_GetType( m_Body ) != b3_dynamicBody )
		return;

	b3Body_SetMassData( m_Body, m_MassData );
}

b3AABB CBox3DPhysicsObject::GetWorldBounds() const
{
	const b3Transform xf = b3Body_GetTransform( m_Body );
	b3AABB local;
	if ( m_pCollide )
	{
		local.lowerBound = b3MulSV( B3VP_M_PER_IN, m_pCollide->m_Bounds.lowerBound );
		local.upperBound = b3MulSV( B3VP_M_PER_IN, m_pCollide->m_Bounds.upperBound );
	}
	else
	{
		const float r = HL2B3( m_flSphereRadius );
		local.lowerBound = b3Vec3{ -r, -r, -r };
		local.upperBound = b3Vec3{ r, r, r };
	}
	return b3AABB_Transform( xf, local );
}

b3Vec3 CBox3DPhysicsObject::GetWorldCenterOfMass() const
{
	return b3TransformPoint( b3Body_GetTransform( m_Body ), m_MassData.center );
}

b3Matrix3 CBox3DPhysicsObject::GetWorldInverseInertia() const
{
	if ( !IsDynamicBody() )
	{
		b3Matrix3 zero;
		zero.cx = zero.cy = zero.cz = b3Vec3_zero;
		return zero;
	}
	return b3Body_GetWorldInverseRotationalInertia( m_Body );
}

//-------------------------------------------------------------------------------------------------
// Body type management
//-------------------------------------------------------------------------------------------------

bool CBox3DPhysicsObject::IsDynamicBody() const
{
	return b3Body_IsValid( m_Body ) && b3Body_GetType( m_Body ) == b3_dynamicBody;
}

void CBox3DPhysicsObject::UpdateBodyType()
{
	if ( m_bStatic || !b3Body_IsValid( m_Body ) )
		return;

	b3BodyType desired = b3_dynamicBody;
	if ( !m_bMotionEnabled )
		desired = b3_kinematicBody;
	else if ( m_pPlayerController )
		desired = b3_kinematicBody;
	else if ( m_pShadowController && m_pShadowController->IsKinematic() )
		desired = b3_kinematicBody;

	const b3BodyType current = b3Body_GetType( m_Body );
	if ( current != desired )
	{
		b3Body_SetType( m_Body, desired );
		if ( desired == b3_dynamicBody )
			ApplyMassData();
		if ( !m_bMotionEnabled )
		{
			b3Body_SetLinearVelocity( m_Body, b3Vec3_zero );
			b3Body_SetAngularVelocity( m_Body, b3Vec3_zero );
		}
	}

	// Shadows that may translate but not rotate keep a fixed orientation under physics
	const bool bFixedRotation = desired == b3_dynamicBody && m_pShadowController && !m_pShadowController->AllowsRotation();
	if ( b3Body_IsFixedRotation( m_Body ) != bFixedRotation )
	{
		b3Body_SetFixedRotation( m_Body, bFixedRotation );
		if ( !bFixedRotation && desired == b3_dynamicBody )
			ApplyMassData();
	}

	const float gravityScale = ( m_bGravityEnabled && !m_bTempGravityDisabled ) ? 1.0f : 0.0f;
	if ( b3Body_GetGravityScale( m_Body ) != gravityScale )
		b3Body_SetGravityScale( m_Body, gravityScale );
}

void CBox3DPhysicsObject::SetTemporaryGravityDisable( bool bDisable )
{
	if ( m_bTempGravityDisabled == bDisable )
		return;
	m_bTempGravityDisabled = bDisable;
	UpdateBodyType();
}

void CBox3DPhysicsObject::SetShadowControllerInternal( CBox3DShadowController *pShadow )
{
	m_pShadowController = pShadow;
	if ( pShadow )
		m_callbackFlags |= CALLBACK_SHADOW_COLLISION;
	UpdateBodyType();
}

void CBox3DPhysicsObject::SetPlayerControllerInternal( CBox3DPlayerController *pPlayer )
{
	m_pPlayerController = pPlayer;
	UpdateBodyType();
}

//-------------------------------------------------------------------------------------------------
// IPhysicsObject: state queries
//-------------------------------------------------------------------------------------------------

bool CBox3DPhysicsObject::IsStatic() const
{
	return m_bStatic;
}

bool CBox3DPhysicsObject::IsAsleep() const
{
	if ( m_bStatic || !b3Body_IsValid( m_Body ) )
		return true;
	return !b3Body_IsAwake( m_Body );
}

bool CBox3DPhysicsObject::IsTrigger() const
{
	return m_bTrigger;
}

bool CBox3DPhysicsObject::IsFluid() const
{
	return m_pFluidController != NULL;
}

bool CBox3DPhysicsObject::IsHinged() const
{
	return m_bHinged;
}

bool CBox3DPhysicsObject::IsCollisionEnabled() const
{
	return m_bCollisionsEnabled;
}

bool CBox3DPhysicsObject::IsGravityEnabled() const
{
	return m_bGravityEnabled;
}

bool CBox3DPhysicsObject::IsDragEnabled() const
{
	return m_bDragEnabled;
}

bool CBox3DPhysicsObject::IsMotionEnabled() const
{
	return m_bMotionEnabled;
}

bool CBox3DPhysicsObject::IsMoveable() const
{
	return !m_bStatic && m_bMotionEnabled;
}

bool CBox3DPhysicsObject::IsAttachedToConstraint( bool bExternalOnly ) const
{
	for ( int i = 0; i < m_Constraints.Count(); i++ )
	{
		if ( !bExternalOnly )
			return true;

		IPhysicsObject *pRef = m_Constraints[i]->GetReferenceObject();
		IPhysicsObject *pAtt = m_Constraints[i]->GetAttachedObject();
		IPhysicsObject *pOther = ( pRef == this ) ? pAtt : pRef;
		if ( pOther && pOther->GetGameData() != m_pGameData )
			return true;
	}
	return false;
}

//-------------------------------------------------------------------------------------------------
// IPhysicsObject: toggles
//-------------------------------------------------------------------------------------------------

void CBox3DPhysicsObject::EnableCollisions( bool enable )
{
	if ( m_bCollisionsEnabled == enable )
		return;

	m_bCollisionsEnabled = enable;

	if ( enable )
		m_callbackFlags |= CALLBACK_ENABLING_COLLISION;

	const b3Filter filter = MakeFilter();
	for ( int i = 0; i < m_Shapes.Count(); i++ )
		b3Shape_SetFilter( m_Shapes[i], filter, true );

	if ( !enable )
		m_pEnvironment->OnObjectShapesRemoved( this, true );

	m_callbackFlags &= ~CALLBACK_ENABLING_COLLISION;
}

void CBox3DPhysicsObject::EnableGravity( bool enable )
{
	m_bGravityEnabled = enable;
	UpdateBodyType();
}

void CBox3DPhysicsObject::EnableDrag( bool enable )
{
	m_bDragEnabled = enable;
}

void CBox3DPhysicsObject::EnableMotion( bool enable )
{
	if ( m_bStatic || m_bMotionEnabled == enable )
		return;

	m_bMotionEnabled = enable;
	UpdateBodyType();

	if ( enable )
		b3Body_SetAwake( m_Body, true );
}

void CBox3DPhysicsObject::SetGameData( void *pGameData )
{
	m_pGameData = pGameData;
}

void *CBox3DPhysicsObject::GetGameData( void ) const
{
	return m_pGameData;
}

void CBox3DPhysicsObject::SetGameFlags( unsigned short userFlags )
{
	m_gameFlags = userFlags;
}

unsigned short CBox3DPhysicsObject::GetGameFlags( void ) const
{
	return m_gameFlags;
}

void CBox3DPhysicsObject::SetGameIndex( unsigned short gameIndex )
{
	m_gameIndex = gameIndex;
}

unsigned short CBox3DPhysicsObject::GetGameIndex( void ) const
{
	return m_gameIndex;
}

void CBox3DPhysicsObject::SetCallbackFlags( unsigned short callbackflags )
{
	// Keep the internal flags the backend owns
	const unsigned short keep = m_callbackFlags & ( CALLBACK_MARKED_FOR_DELETE );
	m_callbackFlags = callbackflags | keep;
	if ( m_pShadowController )
		m_callbackFlags |= CALLBACK_SHADOW_COLLISION;
}

unsigned short CBox3DPhysicsObject::GetCallbackFlags( void ) const
{
	return m_callbackFlags;
}

void CBox3DPhysicsObject::Wake( void )
{
	if ( m_bStatic || !b3Body_IsValid( m_Body ) )
		return;
	b3Body_SetAwake( m_Body, true );
}

void CBox3DPhysicsObject::Sleep( void )
{
	if ( m_bStatic || !b3Body_IsValid( m_Body ) )
		return;
	b3Body_SetAwake( m_Body, false );
}

void CBox3DPhysicsObject::RecheckCollisionFilter()
{
	// Re-run the game's ShouldCollide for every pair involving this object
	const b3Filter filter = MakeFilter();
	for ( int i = 0; i < m_Shapes.Count(); i++ )
		b3Shape_SetFilter( m_Shapes[i], filter, true );
}

void CBox3DPhysicsObject::RecheckContactPoints()
{
	RecheckCollisionFilter();
}

//-------------------------------------------------------------------------------------------------
// IPhysicsObject: mass
//-------------------------------------------------------------------------------------------------

void CBox3DPhysicsObject::SetMass( float mass )
{
	mass = clamp( mass, VPHYSICS_MIN_MASS, VPHYSICS_MAX_MASS );
	if ( m_MassData.mass > 0.0f )
	{
		const float scale = mass / m_MassData.mass;
		m_MassData.inertia.cx = b3MulSV( scale, m_MassData.inertia.cx );
		m_MassData.inertia.cy = b3MulSV( scale, m_MassData.inertia.cy );
		m_MassData.inertia.cz = b3MulSV( scale, m_MassData.inertia.cz );
	}
	m_MassData.mass = mass;
	m_flMass = mass;
	ApplyMassData();
}

float CBox3DPhysicsObject::GetMass( void ) const
{
	return m_flMass;
}

float CBox3DPhysicsObject::GetInvMass( void ) const
{
	if ( m_bStatic || !m_bMotionEnabled || m_flMass <= 0.0f )
		return 0.0f;
	return 1.0f / m_flMass;
}

Vector CBox3DPhysicsObject::GetInertia( void ) const
{
	// kg*m^2, like IVP
	return Vector( m_MassData.inertia.cx.x, m_MassData.inertia.cy.y, m_MassData.inertia.cz.z );
}

Vector CBox3DPhysicsObject::GetInvInertia( void ) const
{
	if ( m_bStatic || !m_bMotionEnabled )
		return vec3_origin;

	const Vector inertia = GetInertia();
	return Vector( inertia.x > 0.0f ? 1.0f / inertia.x : 0.0f,
				   inertia.y > 0.0f ? 1.0f / inertia.y : 0.0f,
				   inertia.z > 0.0f ? 1.0f / inertia.z : 0.0f );
}

void CBox3DPhysicsObject::SetInertia( const Vector &inertia )
{
	m_MassData.inertia = MakeDiagonal( fabsf( inertia.x ), fabsf( inertia.y ), fabsf( inertia.z ) );
	ApplyMassData();
}

void CBox3DPhysicsObject::SetDamping( const float *speed, const float *rot )
{
	if ( speed )
	{
		m_flLinearDamping = *speed;
		b3Body_SetLinearDamping( m_Body, m_flLinearDamping );
	}
	if ( rot )
	{
		m_flAngularDamping = *rot;
		b3Body_SetAngularDamping( m_Body, m_flAngularDamping );
	}
}

void CBox3DPhysicsObject::GetDamping( float *speed, float *rot ) const
{
	if ( speed ) *speed = m_flLinearDamping;
	if ( rot ) *rot = m_flAngularDamping;
}

void CBox3DPhysicsObject::SetDragCoefficient( float *pDrag, float *pAngularDrag )
{
	if ( pDrag ) m_flDragCoefficient = *pDrag;
	if ( pAngularDrag ) m_flAngularDragCoefficient = *pAngularDrag;
}

void CBox3DPhysicsObject::SetBuoyancyRatio( float ratio )
{
	m_flBuoyancyRatio = ratio;
}

//-------------------------------------------------------------------------------------------------
// IPhysicsObject: materials and contents
//-------------------------------------------------------------------------------------------------

int CBox3DPhysicsObject::GetMaterialIndex() const
{
	return m_materialIndex;
}

void CBox3DPhysicsObject::SetMaterialIndex( int materialIndex )
{
	if ( m_materialIndex == materialIndex )
		return;

	m_materialIndex = materialIndex;

	CBox3DPhysicsSurfaceProps &props = CBox3DPhysicsSurfaceProps::GetInstance();
	for ( int i = 0; i < m_Shapes.Count(); i++ )
	{
		if ( b3Shape_GetType( m_Shapes[i] ) == b3_meshShape )
			continue;

		b3SurfaceMaterial material = b3Shape_GetSurfaceMaterial( m_Shapes[i] );
		material.friction = props.GetFriction( materialIndex );
		material.restitution = props.GetElasticity( materialIndex );
		material.userMaterialId = (uint64_t)MAX( materialIndex, 0 );
		b3Shape_SetSurfaceMaterial( m_Shapes[i], material );
	}

	if ( m_pShadowController )
		m_pShadowController->ObjectMaterialChanged( materialIndex );
}

unsigned int CBox3DPhysicsObject::GetContents() const
{
	return m_contents;
}

void CBox3DPhysicsObject::SetContents( unsigned int contents )
{
	m_contents = contents;
}

float CBox3DPhysicsObject::GetSphereRadius() const
{
	return m_flSphereRadius;
}

float CBox3DPhysicsObject::GetEnergy() const
{
	if ( !IsDynamicBody() )
		return 0.0f;

	const b3Vec3 v = b3Body_GetLinearVelocity( m_Body );
	const b3Vec3 wWorld = b3Body_GetAngularVelocity( m_Body );
	const b3Vec3 wLocal = b3InvRotateVector( b3Body_GetRotation( m_Body ), wWorld );
	const b3Vec3 Iw = b3MulMV( m_MassData.inertia, wLocal );
	const float energy = 0.5f * m_MassData.mass * b3Dot( v, v ) + 0.5f * b3Dot( wLocal, Iw );
	return B3Energy2HL( energy );
}

Vector CBox3DPhysicsObject::GetMassCenterLocalSpace() const
{
	return B32HL( m_MassData.center );
}

//-------------------------------------------------------------------------------------------------
// IPhysicsObject: transforms
//-------------------------------------------------------------------------------------------------

void CBox3DPhysicsObject::SetPosition( const Vector &worldPosition, const QAngle &angles, bool isTeleport )
{
	if ( !b3Body_IsValid( m_Body ) )
		return;

	b3Body_SetTransform( m_Body, HL2B3( worldPosition ), AnglesToB3Quat( angles ) );

	if ( isTeleport )
	{
		m_LastPosition = worldPosition;
		m_LastAngles = angles;
	}
}

void CBox3DPhysicsObject::SetPositionMatrix( const matrix3x4_t &matrix, bool isTeleport )
{
	Vector origin;
	QAngle angles;
	MatrixAngles( matrix, angles, origin );
	SetPosition( origin, angles, isTeleport );
}

void CBox3DPhysicsObject::GetPosition( Vector *worldPosition, QAngle *angles ) const
{
	if ( !b3Body_IsValid( m_Body ) )
	{
		if ( worldPosition ) worldPosition->Init();
		if ( angles ) angles->Init();
		return;
	}

	const b3Transform xf = b3Body_GetTransform( m_Body );
	if ( worldPosition )
		*worldPosition = B32HL( xf.p );
	if ( angles )
		*angles = B3QuatToAngles( xf.q );
}

void CBox3DPhysicsObject::GetPositionMatrix( matrix3x4_t *positionMatrix ) const
{
	if ( !positionMatrix )
		return;

	if ( !b3Body_IsValid( m_Body ) )
	{
		SetIdentityMatrix( *positionMatrix );
		return;
	}

	B3TransformToMatrix( b3Body_GetTransform( m_Body ), *positionMatrix );
}

void CBox3DPhysicsObject::LocalToWorld( Vector *worldPosition, const Vector &localPosition ) const
{
	matrix3x4_t matrix;
	GetPositionMatrix( &matrix );
	VectorTransform( Vector( localPosition ), matrix, *worldPosition );
}

void CBox3DPhysicsObject::WorldToLocal( Vector *localPosition, const Vector &worldPosition ) const
{
	matrix3x4_t matrix;
	GetPositionMatrix( &matrix );
	VectorITransform( Vector( worldPosition ), matrix, *localPosition );
}

void CBox3DPhysicsObject::LocalToWorldVector( Vector *worldVector, const Vector &localVector ) const
{
	matrix3x4_t matrix;
	GetPositionMatrix( &matrix );
	VectorRotate( Vector( localVector ), matrix, *worldVector );
}

void CBox3DPhysicsObject::WorldToLocalVector( Vector *localVector, const Vector &worldVector ) const
{
	matrix3x4_t matrix;
	GetPositionMatrix( &matrix );
	VectorIRotate( Vector( worldVector ), matrix, *localVector );
}

//-------------------------------------------------------------------------------------------------
// IPhysicsObject: velocity
//-------------------------------------------------------------------------------------------------

b3Vec3 CBox3DPhysicsObject::LocalAngularToWorld( const AngularImpulse &local ) const
{
	return b3RotateVector( b3Body_GetRotation( m_Body ), HLAng2B3( local ) );
}

AngularImpulse CBox3DPhysicsObject::WorldAngularToLocal( const b3Vec3 &world ) const
{
	return B3Ang2HL( b3InvRotateVector( b3Body_GetRotation( m_Body ), world ) );
}

void CBox3DPhysicsObject::SetVelocity( const Vector *velocity, const AngularImpulse *angularVelocity )
{
	if ( m_bStatic || !b3Body_IsValid( m_Body ) )
		return;

	// Frozen objects keep zero velocity (kinematic followers are driven by their controller)
	if ( !m_bMotionEnabled )
		return;

	bool bWake = false;
	if ( velocity )
	{
		const b3Vec3 v = HL2B3( *velocity );
		b3Body_SetLinearVelocity( m_Body, v );
		if ( m_bUsePreStepVelocity )
			m_PreStepLinearVelocity = v;
		bWake |= b3Dot( v, v ) > 0.0f;
	}
	if ( angularVelocity )
	{
		const b3Vec3 w = LocalAngularToWorld( *angularVelocity );
		b3Body_SetAngularVelocity( m_Body, w );
		if ( m_bUsePreStepVelocity )
			m_PreStepAngularVelocity = w;
		bWake |= b3Dot( w, w ) > 0.0f;
	}

	if ( bWake )
		b3Body_SetAwake( m_Body, true );
}

void CBox3DPhysicsObject::SetVelocityInstantaneous( const Vector *velocity, const AngularImpulse *angularVelocity )
{
	SetVelocity( velocity, angularVelocity );
}

void CBox3DPhysicsObject::GetVelocity( Vector *velocity, AngularImpulse *angularVelocity ) const
{
	if ( !b3Body_IsValid( m_Body ) || m_bStatic )
	{
		if ( velocity ) velocity->Init();
		if ( angularVelocity ) angularVelocity->Init();
		return;
	}

	const b3Vec3 v = m_bUsePreStepVelocity ? m_PreStepLinearVelocity : b3Body_GetLinearVelocity( m_Body );
	const b3Vec3 w = m_bUsePreStepVelocity ? m_PreStepAngularVelocity : b3Body_GetAngularVelocity( m_Body );

	if ( velocity )
		*velocity = B32HL( v );
	if ( angularVelocity )
		*angularVelocity = WorldAngularToLocal( w );
}

void CBox3DPhysicsObject::AddVelocity( const Vector *velocity, const AngularImpulse *angularVelocity )
{
	if ( !IsMoveable() || !IsDynamicBody() )
		return;

	if ( velocity )
		b3Body_SetLinearVelocity( m_Body, b3Add( b3Body_GetLinearVelocity( m_Body ), HL2B3( *velocity ) ) );
	if ( angularVelocity )
		b3Body_SetAngularVelocity( m_Body, b3Add( b3Body_GetAngularVelocity( m_Body ), LocalAngularToWorld( *angularVelocity ) ) );

	b3Body_SetAwake( m_Body, true );
}

void CBox3DPhysicsObject::GetVelocityAtPoint( const Vector &worldPosition, Vector *pVelocity ) const
{
	if ( !pVelocity )
		return;

	if ( m_bStatic || !b3Body_IsValid( m_Body ) )
	{
		pVelocity->Init();
		return;
	}

	*pVelocity = B32HL( b3Body_GetWorldPointVelocity( m_Body, HL2B3( worldPosition ) ) );
}

void CBox3DPhysicsObject::GetImplicitVelocity( Vector *velocity, AngularImpulse *angularVelocity ) const
{
	if ( velocity ) *velocity = m_ImplicitVelocity;
	if ( angularVelocity ) *angularVelocity = m_ImplicitAngularVelocity;
}

//-------------------------------------------------------------------------------------------------
// IPhysicsObject: forces. VPhysics "forces" are impulses (kg*in/s), torques are angular
// impulses in world space (kg*m^2*deg/s).
//-------------------------------------------------------------------------------------------------

void CBox3DPhysicsObject::ApplyStepVelocityChange( const Vector &deltaVelocity, const AngularImpulse &deltaAngularLocal, float dt )
{
	if ( !IsMoveable() || !IsDynamicBody() || dt <= 0.0f )
		return;

	// Spread the change over the coming step as a force so it integrates with gravity like IVP
	const float invDt = 1.0f / dt;
	if ( deltaVelocity.LengthSqr() > 0.0f )
		b3Body_ApplyForceToCenter( m_Body, b3MulSV( m_MassData.mass * invDt, HL2B3( deltaVelocity ) ), true );

	if ( deltaAngularLocal.LengthSqr() > 0.0f )
	{
		const b3Vec3 alphaLocal = b3MulSV( invDt, HLAng2B3( deltaAngularLocal ) );
		const b3Vec3 torqueLocal = b3MulMV( m_MassData.inertia, alphaLocal );
		b3Body_ApplyTorque( m_Body, b3RotateVector( b3Body_GetRotation( m_Body ), torqueLocal ), true );
	}
}

void CBox3DPhysicsObject::ApplyForceCenter( const Vector &forceVector )
{
	if ( !IsMoveable() || !IsDynamicBody() )
		return;
	b3Body_ApplyLinearImpulseToCenter( m_Body, HL2B3( forceVector ), true );
}

void CBox3DPhysicsObject::ApplyForceOffset( const Vector &forceVector, const Vector &worldPosition )
{
	if ( !IsMoveable() || !IsDynamicBody() )
		return;
	b3Body_ApplyLinearImpulse( m_Body, HL2B3( forceVector ), HL2B3( worldPosition ), true );
}

void CBox3DPhysicsObject::ApplyTorqueCenter( const AngularImpulse &torque )
{
	if ( !IsMoveable() || !IsDynamicBody() )
		return;
	b3Body_ApplyAngularImpulse( m_Body, HLTorque2B3( torque ), true );
}

void CBox3DPhysicsObject::CalculateForceOffset( const Vector &forceVector, const Vector &worldPosition, Vector *centerForce, AngularImpulse *centerTorque ) const
{
	if ( centerForce )
		*centerForce = forceVector;

	if ( centerTorque )
	{
		if ( !b3Body_IsValid( m_Body ) )
		{
			centerTorque->Init();
			return;
		}
		const b3Vec3 r = b3Sub( HL2B3( worldPosition ), GetWorldCenterOfMass() );
		*centerTorque = B3Torque2HL( b3Cross( r, HL2B3( forceVector ) ) );
	}
}

void CBox3DPhysicsObject::CalculateVelocityOffset( const Vector &forceVector, const Vector &worldPosition, Vector *centerVelocity, AngularImpulse *centerAngularVelocity ) const
{
	const float invMass = GetInvMass();

	if ( centerVelocity )
		*centerVelocity = forceVector * invMass;

	if ( centerAngularVelocity )
	{
		if ( !IsDynamicBody() || invMass <= 0.0f )
		{
			centerAngularVelocity->Init();
			return;
		}

		// Local space angular velocity change, like AddVelocity expects
		const b3Vec3 r = b3Sub( HL2B3( worldPosition ), GetWorldCenterOfMass() );
		const b3Vec3 torqueWorld = b3Cross( r, HL2B3( forceVector ) );
		const b3Vec3 dwWorld = b3MulMV( GetWorldInverseInertia(), torqueWorld );
		*centerAngularVelocity = WorldAngularToLocal( dwWorld );
	}
}

float CBox3DPhysicsObject::CalculateLinearDrag( const Vector &unitDirection ) const
{
	// Projected area along a world direction (in^2) times the drag coefficient
	Vector localDir;
	WorldToLocalVector( &localDir, unitDirection );
	const float area = fabsf( localDir.x ) * m_DragAreas.x + fabsf( localDir.y ) * m_DragAreas.y + fabsf( localDir.z ) * m_DragAreas.z;
	return m_flDragCoefficient * area;
}

float CBox3DPhysicsObject::CalculateAngularDrag( const Vector &objectSpaceRotationAxis ) const
{
	const float area = fabsf( objectSpaceRotationAxis.x ) * m_DragAreas.x + fabsf( objectSpaceRotationAxis.y ) * m_DragAreas.y + fabsf( objectSpaceRotationAxis.z ) * m_DragAreas.z;
	return m_flAngularDragCoefficient * area;
}

//-------------------------------------------------------------------------------------------------
// IPhysicsObject: contacts
//-------------------------------------------------------------------------------------------------

bool CBox3DPhysicsObject::GetContactPoint( Vector *contactPoint, IPhysicsObject **contactObject ) const
{
	if ( !b3Body_IsValid( m_Body ) || m_bStatic )
		return false;

	const int capacity = b3Body_GetContactCapacity( m_Body );
	if ( capacity <= 0 )
		return false;

	CUtlVector< b3ContactData > contacts;
	contacts.SetCount( capacity );
	const int count = b3Body_GetContactData( m_Body, contacts.Base(), capacity );

	for ( int i = 0; i < count; i++ )
	{
		const b3ContactData &data = contacts[i];
		for ( int m = 0; m < data.manifoldCount; m++ )
		{
			const b3Manifold &manifold = data.manifolds[m];
			if ( manifold.pointCount <= 0 )
				continue;

			const bool bSelfIsA = b3Body_IsValid( b3Shape_GetBody( data.shapeIdA ) ) && GetShapeObject( data.shapeIdA ) == this;
			const b3ShapeId otherShape = bSelfIsA ? data.shapeIdB : data.shapeIdA;
			CBox3DPhysicsObject *pOther = GetShapeObject( otherShape );

			if ( contactPoint )
			{
				const b3BodyId bodyA = b3Shape_GetBody( data.shapeIdA );
				const b3Vec3 p = b3Add( b3Body_GetWorldCenter( bodyA ), manifold.points[0].anchorA );
				*contactPoint = B32HL( p );
			}
			if ( contactObject )
				*contactObject = pOther;
			return true;
		}
	}
	return false;
}

IPhysicsFrictionSnapshot *CBox3DPhysicsObject::CreateFrictionSnapshot()
{
	return new CBox3DFrictionSnapshot( this );
}

void CBox3DPhysicsObject::DestroyFrictionSnapshot( IPhysicsFrictionSnapshot *pSnapshot )
{
	delete static_cast< CBox3DFrictionSnapshot * >( pSnapshot );
}

//-------------------------------------------------------------------------------------------------
// IPhysicsObject: shadows
//-------------------------------------------------------------------------------------------------

void CBox3DPhysicsObject::SetShadow( float maxSpeed, float maxAngularSpeed, bool allowPhysicsMovement, bool allowPhysicsRotation )
{
	if ( m_pShadowController )
	{
		m_pShadowController->MaxSpeed( maxSpeed, maxAngularSpeed );
		return;
	}

	IPhysicsShadowController *pShadow = m_pEnvironment->CreateShadowController( this, allowPhysicsMovement, allowPhysicsRotation );
	if ( pShadow )
		pShadow->MaxSpeed( maxSpeed, maxAngularSpeed );
}

void CBox3DPhysicsObject::UpdateShadow( const Vector &targetPosition, const QAngle &targetAngles, bool tempDisableGravity, float timeOffset )
{
	if ( !m_pShadowController )
		return;

	if ( m_pShadowController->AllowsTranslation() )
		SetTemporaryGravityDisable( tempDisableGravity );

	m_pShadowController->Update( targetPosition, targetAngles, timeOffset );
}

int CBox3DPhysicsObject::GetShadowPosition( Vector *position, QAngle *angles ) const
{
	GetPosition( position, angles );
	return 1;
}

IPhysicsShadowController *CBox3DPhysicsObject::GetShadowController( void ) const
{
	return m_pShadowController;
}

void CBox3DPhysicsObject::RemoveShadowController()
{
	if ( !m_pShadowController )
		return;

	CBox3DShadowController *pShadow = m_pShadowController;
	m_pEnvironment->DestroyShadowController( pShadow );
	m_pShadowController = NULL;
	UpdateBodyType();
}

// The controller IVP uses for shadows and grabbed objects. Units are whatever the caller uses,
// as long as position and speed agree.
static void ComputeController( Vector &currentVelocity, const Vector &deltaPosition, float maxSpeed, float maxDampSpeed, float scaleDelta, float damping, Vector *pOutImpulse )
{
	const float currentSpeedSq = currentVelocity.LengthSqr();
	if ( currentSpeedSq < 1e-6f )
	{
		currentVelocity.Init();
	}
	else if ( maxDampSpeed > 0.0f )
	{
		Vector accelDampening = currentVelocity * -damping;
		float speed = sqrtf( currentSpeedSq ) * fabsf( damping );
		if ( speed > maxDampSpeed )
			accelDampening *= maxDampSpeed / speed;
		currentVelocity += accelDampening;
	}

	Vector acceleration( 0, 0, 0 );
	if ( maxSpeed > 0.0f )
	{
		acceleration = deltaPosition * scaleDelta;
		const float speed = deltaPosition.Length() * scaleDelta;
		if ( speed > maxSpeed )
			acceleration *= maxSpeed / speed;
		currentVelocity += acceleration;
	}

	if ( pOutImpulse )
		*pOutImpulse = acceleration;
}

float CBox3DPhysicsObject::ComputeShadowControl( const hlshadowcontrol_params_t &params, float secondsToArrival, float dt )
{
	if ( !b3Body_IsValid( m_Body ) || m_bStatic || dt <= 0.0f )
		return secondsToArrival;

	const float fraction = secondsToArrival > 0.0f ? MIN( dt / secondsToArrival, 1.0f ) : 1.0f;
	secondsToArrival = MAX( secondsToArrival - dt, 0.0f );
	if ( fraction <= 0.0f )
		return secondsToArrival;

	b3Transform xf = b3Body_GetTransform( m_Body );
	const b3Quat targetRotation = AnglesToB3Quat( params.targetRotation );

	Vector position = B32HL( xf.p );
	Vector deltaPosition = params.targetPosition - position;

	if ( params.teleportDistance > 0.0f && deltaPosition.LengthSqr() > params.teleportDistance * params.teleportDistance )
	{
		b3Body_SetTransform( m_Body, HL2B3( params.targetPosition ), targetRotation );
		xf = b3Body_GetTransform( m_Body );
		deltaPosition.Init();
	}

	const float scale = fraction / dt;

	Vector linear = B32HL( b3Body_GetLinearVelocity( m_Body ) );
	ComputeController( linear, deltaPosition, params.maxSpeed, params.maxDampSpeed, scale, params.dampFactor, NULL );

	// Rotation error as a world space axis * angle, in degrees to match the max speeds
	b3Quat dq = b3MulQuat( targetRotation, b3Conjugate( xf.q ) );
	if ( dq.s < 0.0f )
	{
		dq.v = b3Neg( dq.v );
		dq.s = -dq.s;
	}
	const float sinHalf = b3Length( dq.v );
	Vector deltaAngles( 0, 0, 0 );
	if ( sinHalf > 1e-6f )
	{
		const float angle = 2.0f * atan2f( sinHalf, dq.s );
		const b3Vec3 axis = b3MulSV( 1.0f / sinHalf, dq.v );
		deltaAngles = Vector( axis.x, axis.y, axis.z ) * RAD2DEG( angle );
	}

	Vector angular = B3Ang2HL( b3Body_GetAngularVelocity( m_Body ) );
	ComputeController( angular, deltaAngles, params.maxAngular, params.maxDampAngular, scale, params.dampFactor, NULL );

	b3Body_SetLinearVelocity( m_Body, HL2B3( linear ) );
	b3Body_SetAngularVelocity( m_Body, HLAng2B3( angular ) );
	if ( linear.LengthSqr() > 0.0f || angular.LengthSqr() > 0.0f )
		b3Body_SetAwake( m_Body, true );

	return secondsToArrival;
}

//-------------------------------------------------------------------------------------------------
// IPhysicsObject: misc
//-------------------------------------------------------------------------------------------------

const CPhysCollide *CBox3DPhysicsObject::GetCollide( void ) const
{
	return m_pCollide;
}

const char *CBox3DPhysicsObject::GetName() const
{
	return m_szName;
}

void CBox3DPhysicsObject::BecomeTrigger()
{
	if ( m_bTrigger )
		return;

	EnableDrag( false );
	EnableGravity( false );

	DestroyShapes();
	m_bTrigger = true;
	CreateShapes();
}

void CBox3DPhysicsObject::RemoveTrigger()
{
	if ( !m_bTrigger )
		return;

	// Fires ObjectLeaveTrigger for everything inside
	DestroyShapes();
	m_bTrigger = false;
	CreateShapes();
	if ( !m_bStatic )
		ApplyMassData();
}

void CBox3DPhysicsObject::BecomeHinged( int localAxis )
{
	if ( m_bHinged || m_bStatic )
		return;

	m_bHinged = true;
	m_nHingeAxis = clamp( localAxis, 0, 2 );

	const b3Transform xf = b3Body_GetTransform( m_Body );
	b3Vec3 localDir = b3Vec3_zero;
	( &localDir.x )[m_nHingeAxis] = 1.0f;

	b3Transform hingeWorld;
	hingeWorld.p = GetWorldCenterOfMass();
	hingeWorld.q = B3QuatFromZAxis( b3RotateVector( xf.q, localDir ) );

	b3RevoluteJointDef def = b3DefaultRevoluteJointDef();
	def.base.bodyIdA = m_pEnvironment->GetAnchorBody();
	def.base.bodyIdB = m_Body;
	def.base.localFrameA = hingeWorld;
	def.base.localFrameB = b3InvMulTransforms( xf, hingeWorld );
	def.base.collideConnected = true;
	m_HingeJoint = b3CreateRevoluteJoint( m_pEnvironment->GetWorld(), &def );
}

void CBox3DPhysicsObject::RemoveHinged()
{
	if ( !m_bHinged )
		return;

	if ( b3Joint_IsValid( m_HingeJoint ) )
		b3DestroyJoint( m_HingeJoint, true );
	m_HingeJoint = b3_nullJointId;
	m_bHinged = false;
}

void CBox3DPhysicsObject::OutputDebugInfo() const
{
	Vector pos;
	QAngle ang;
	GetPosition( &pos, &ang );
	Vector vel;
	AngularImpulse angVel;
	GetVelocity( &vel, &angVel );

	Msg( "-----------------\nObject: %s (Box3D)\n", m_szName );
	Msg( "Mass: %.2f (inv %.4f)\n", m_flMass, GetInvMass() );
	Msg( "Inertia: %.3f %.3f %.3f\n", m_MassData.inertia.cx.x, m_MassData.inertia.cy.y, m_MassData.inertia.cz.z );
	Msg( "Position: %.2f %.2f %.2f, Angles: %.2f %.2f %.2f\n", pos.x, pos.y, pos.z, ang.x, ang.y, ang.z );
	Msg( "Velocity: %.2f %.2f %.2f, Angular: %.2f %.2f %.2f\n", vel.x, vel.y, vel.z, angVel.x, angVel.y, angVel.z );
	Msg( "Damping: %.2f linear, %.2f angular\n", m_flLinearDamping, m_flAngularDamping );
	Msg( "Flags: %s%s%s%s%s%s\n",
		m_bStatic ? "static " : "",
		IsAsleep() ? "asleep " : "",
		m_bMotionEnabled ? "" : "motion-disabled ",
		m_bCollisionsEnabled ? "" : "no-collide ",
		m_bTrigger ? "trigger " : "",
		m_pShadowController ? "shadow " : "" );
	Msg( "Material: %s, Shapes: %d\n", CBox3DPhysicsSurfaceProps::GetInstance().GetPropName( m_materialIndex ), m_Shapes.Count() );
}

//-------------------------------------------------------------------------------------------------
// Per-step work
//-------------------------------------------------------------------------------------------------

void CBox3DPhysicsObject::CachePreStepVelocity()
{
	if ( !b3Body_IsValid( m_Body ) || m_bStatic )
		return;
	m_PreStepLinearVelocity = b3Body_GetLinearVelocity( m_Body );
	m_PreStepAngularVelocity = b3Body_GetAngularVelocity( m_Body );
}

void CBox3DPhysicsObject::PreStep( float dt )
{
	if ( !IsDynamicBody() || !b3Body_IsAwake( m_Body ) )
		return;

	CachePreStepVelocity();

	// Clamp angular speed (Box3D clamps linear speed itself)
	const float maxAngular = m_pEnvironment->GetMaxAngularSpeed();
	b3Vec3 w = b3Body_GetAngularVelocity( m_Body );
	const float wLen = b3Length( w );
	if ( maxAngular > 0.0f && wLen > maxAngular )
	{
		w = b3MulSV( maxAngular / wLen, w );
		b3Body_SetAngularVelocity( m_Body, w );
	}

	// Air drag
	const float airDensity = m_pEnvironment->GetAirDensity();
	if ( m_bDragEnabled && airDensity > 0.0f && m_MassData.mass > 0.0f )
	{
		const b3Quat q = b3Body_GetRotation( m_Body );

		b3Vec3 v = b3Body_GetLinearVelocity( m_Body );
		const float speed = b3Length( v );
		if ( m_flDragCoefficient > 0.0f && speed > 0.01f )
		{
			const b3Vec3 dirLocal = b3InvRotateVector( q, b3MulSV( 1.0f / speed, v ) );
			const float areaIn2 = fabsf( dirLocal.x ) * m_DragAreas.x + fabsf( dirLocal.y ) * m_DragAreas.y + fabsf( dirLocal.z ) * m_DragAreas.z;
			const float area = areaIn2 * B3VP_M_PER_IN * B3VP_M_PER_IN;
			const float force = 0.5f * airDensity * m_flDragCoefficient * area * speed * speed;
			const float dv = MIN( force * dt / m_MassData.mass, 0.5f * speed );
			v = b3MulAdd( v, -dv / speed, v );
			b3Body_SetLinearVelocity( m_Body, v );
		}

		if ( m_flAngularDragCoefficient > 0.0f && wLen > 0.01f )
		{
			const float areaIn2 = ( m_DragAreas.x + m_DragAreas.y + m_DragAreas.z ) / 3.0f;
			const float area = areaIn2 * B3VP_M_PER_IN * B3VP_M_PER_IN;
			const float radiusSq = area / M_PI_F;
			const float inertia = MAX( ( m_MassData.inertia.cx.x + m_MassData.inertia.cy.y + m_MassData.inertia.cz.z ) / 3.0f, 1e-6f );
			const float torque = 0.5f * airDensity * m_flAngularDragCoefficient * area * radiusSq * b3Dot( w, w );
			const float dw = MIN( torque * dt / inertia, 0.5f * b3Length( w ) );
			const float len = b3Length( w );
			if ( len > 0.0f )
				b3Body_SetAngularVelocity( m_Body, b3MulAdd( w, -dw / len, w ) );
		}
	}
}

void CBox3DPhysicsObject::PostStep( float dt )
{
	if ( dt <= 0.0f || !b3Body_IsValid( m_Body ) )
		return;

	Vector position;
	QAngle angles;
	GetPosition( &position, &angles );

	m_ImplicitVelocity = ( position - m_LastPosition ) / dt;

	// Angular velocity actually moved, in local space
	Quaternion q0, q1;
	AngleQuaternion( m_LastAngles, q0 );
	AngleQuaternion( angles, q1 );
	const b3Quat dq = b3MulQuat( b3InvMulQuat( ToB3Quat( q0 ), ToB3Quat( q1 ) ), b3Quat_identity );
	b3Quat d = dq;
	if ( d.s < 0.0f )
	{
		d.v = b3Neg( d.v );
		d.s = -d.s;
	}
	const float sinHalf = b3Length( d.v );
	if ( sinHalf > 1e-7f )
	{
		const float angle = 2.0f * atan2f( sinHalf, d.s );
		const b3Vec3 axis = b3MulSV( 1.0f / sinHalf, d.v );
		m_ImplicitAngularVelocity = Vector( axis.x, axis.y, axis.z ) * ( RAD2DEG( angle ) / dt );
	}
	else
	{
		m_ImplicitAngularVelocity.Init();
	}

	m_LastPosition = position;
	m_LastAngles = angles;
}

//-------------------------------------------------------------------------------------------------
// Constraint and listener bookkeeping
//-------------------------------------------------------------------------------------------------

void CBox3DPhysicsObject::AddConstraint( CBox3DConstraint *pConstraint )
{
	if ( m_Constraints.Find( pConstraint ) == m_Constraints.InvalidIndex() )
		m_Constraints.AddToTail( pConstraint );
}

void CBox3DPhysicsObject::RemoveConstraint( CBox3DConstraint *pConstraint )
{
	m_Constraints.FindAndFastRemove( pConstraint );
}

void CBox3DPhysicsObject::AddListener( IBox3DObjectListener *pListener )
{
	if ( m_Listeners.Find( pListener ) == m_Listeners.InvalidIndex() )
		m_Listeners.AddToTail( pListener );
}

void CBox3DPhysicsObject::RemoveListener( IBox3DObjectListener *pListener )
{
	m_Listeners.FindAndFastRemove( pListener );
}

//-------------------------------------------------------------------------------------------------
// Save / restore
//-------------------------------------------------------------------------------------------------

void CBox3DPhysicsObject::WriteSaveData( B3VPObjectSaveData_t &data ) const
{
	memset( &data, 0, sizeof( data ) );
	data.version = B3VP_OBJECT_SAVE_VERSION;
	GetPosition( &data.position, &data.angles );

	// Velocity of frozen/kinematic objects is meaningless to restore
	if ( IsDynamicBody() )
		GetVelocity( &data.velocity, &data.angularVelocity );

	data.mass = m_flMass;
	data.inertia = GetInertia();
	data.damping = m_flLinearDamping;
	data.rotDamping = m_flAngularDamping;
	data.dragCoefficient = m_flDragCoefficient;
	data.angularDragCoefficient = m_flAngularDragCoefficient;
	data.buoyancyRatio = m_flBuoyancyRatio;
	data.volume = m_flVolume;
	data.sphereRadius = m_flSphereRadius;
	data.contents = m_contents;
	data.gameFlags = m_gameFlags;
	data.gameIndex = m_gameIndex;
	data.callbackFlags = m_callbackFlags & ~( CALLBACK_MARKED_FOR_DELETE | CALLBACK_SHADOW_COLLISION );

	const char *pMaterialName = CBox3DPhysicsSurfaceProps::GetInstance().GetPropName( m_materialIndex );
	Q_strncpy( data.materialName, pMaterialName ? pMaterialName : "default", sizeof( data.materialName ) );

	data.bStatic = m_bStatic;
	data.bMotionEnabled = m_bMotionEnabled;
	data.bGravityEnabled = m_bGravityEnabled;
	data.bDragEnabled = m_bDragEnabled;
	data.bCollisionsEnabled = m_bCollisionsEnabled;
	data.bTrigger = m_bTrigger;
	data.bAsleep = IsAsleep();
	data.bHinged = m_bHinged;
	data.hingeAxis = m_nHingeAxis;
	data.massCenter = B32HL( m_MassData.center );

	if ( m_pShadowController )
	{
		data.bHasShadow = true;
		data.bShadowAllowTranslation = m_pShadowController->AllowsTranslation();
		data.bShadowAllowRotation = m_pShadowController->AllowsRotation();
		data.bShadowPhysicallyControlled = m_pShadowController->IsPhysicallyControlled();
		m_pShadowController->GetMaxSpeed( &data.shadowMaxSpeed, &data.shadowMaxAngular );
		data.shadowTeleportDistance = m_pShadowController->GetTeleportDistance();
		data.shadowSecondsToArrival = m_pShadowController->GetTargetPosition( &data.shadowTargetPosition, &data.shadowTargetAngles );
	}

	Q_strncpy( data.name, m_szName, sizeof( data.name ) );
}

void CBox3DPhysicsObject::ApplySaveData( const B3VPObjectSaveData_t &data )
{
	m_gameFlags = data.gameFlags;
	m_gameIndex = data.gameIndex;
	m_callbackFlags = data.callbackFlags;
	m_contents = data.contents;
	m_flDragCoefficient = data.dragCoefficient;
	m_flAngularDragCoefficient = data.angularDragCoefficient;
	m_flBuoyancyRatio = data.buoyancyRatio;
	m_flVolume = data.volume;
	Q_strncpy( m_szName, data.name, sizeof( m_szName ) );

	if ( !m_bStatic )
	{
		m_flMass = data.mass;
		m_MassData.mass = data.mass;
		m_MassData.center = HL2B3( data.massCenter );
		if ( data.inertia.x > 0.0f && data.inertia.y > 0.0f && data.inertia.z > 0.0f )
			m_MassData.inertia = MakeDiagonal( data.inertia.x, data.inertia.y, data.inertia.z );
		ApplyMassData();
	}

	float damping = data.damping, rotDamping = data.rotDamping;
	SetDamping( &damping, &rotDamping );

	EnableDrag( data.bDragEnabled );
	EnableGravity( data.bGravityEnabled );
	EnableCollisions( data.bCollisionsEnabled );

	if ( data.bTrigger )
		BecomeTrigger();

	if ( data.bHasShadow )
	{
		SetShadow( data.shadowMaxSpeed, data.shadowMaxAngular, data.bShadowAllowTranslation, data.bShadowAllowRotation );
		if ( m_pShadowController )
		{
			m_pShadowController->SetPhysicallyControlled( data.bShadowPhysicallyControlled );
			m_pShadowController->SetTeleportDistance( data.shadowTeleportDistance );
			m_pShadowController->Update( data.shadowTargetPosition, data.shadowTargetAngles, data.shadowSecondsToArrival );
		}
	}

	EnableMotion( data.bMotionEnabled );

	if ( data.bHinged )
		BecomeHinged( data.hingeAxis );

	SetPosition( data.position, data.angles, true );

	if ( !data.bAsleep && IsMoveable() )
	{
		Wake();
		SetVelocity( &data.velocity, &data.angularVelocity );
	}
}
