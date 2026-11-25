//=================================================================================================
//
// VPhysics Box3D - constraints, constraint groups and springs
//
//=================================================================================================

#ifndef B3VP_CONSTRAINTS_H
#define B3VP_CONSTRAINTS_H

#ifdef _WIN32
#pragma once
#endif

#include "b3vp_base.h"
#include "b3vp_object.h"
#include "b3vp_environment.h"

class CBox3DConstraintGroup;

enum B3VPConstraintType_t
{
	B3VP_CONSTRAINT_NONE = 0,
	B3VP_CONSTRAINT_RAGDOLL,
	B3VP_CONSTRAINT_HINGE,
	B3VP_CONSTRAINT_FIXED,
	B3VP_CONSTRAINT_SLIDING,
	B3VP_CONSTRAINT_BALLSOCKET,
	B3VP_CONSTRAINT_PULLEY,
	B3VP_CONSTRAINT_LENGTH,
};

enum B3VPJointKind_t
{
	B3VP_JOINT_NONE = 0,
	B3VP_JOINT_WELD,
	B3VP_JOINT_REVOLUTE,
	B3VP_JOINT_SPHERICAL,
	B3VP_JOINT_PRISMATIC,
	B3VP_JOINT_DISTANCE,
	B3VP_JOINT_PULLEY,		// solved by the backend, Box3D has no pulley joint
};

// Everything needed to (re)build a joint. Frames are relative to the body origins, in metres.
// Plain data so it can be written straight into save games.
struct B3VPJointDesc_t
{
	int				sourceType;			// B3VPConstraintType_t
	int				kind;				// B3VPJointKind_t
	b3Transform		frameA;
	b3Transform		frameB;

	// Revolute (radians) / prismatic (metres) / distance (metres)
	bool			enableLimit;
	float			lower;
	float			upper;

	bool			enableMotor;
	float			motorSpeed;			// rad/s or m/s
	float			maxMotorForce;		// N*m or N

	// Spherical
	bool			enableCone;
	float			coneAngle;
	bool			enableTwist;
	float			twistLower;
	float			twistUpper;
	float			frictionTorque;		// N*m, applied as a zero speed motor

	// Distance
	float			length;
	bool			distanceRigid;

	// Pulley (world points in metres, anchors relative to body origins)
	b3Vec3			pulleyPoint[2];
	b3Vec3			pulleyAnchor[2];
	float			pulleyLength;
	float			pulleyRatio;
	bool			pulleyRigid;

	// Breaking (SI, 0 = unbreakable)
	float			breakForce;
	float			breakTorque;

	// Source side info
	constraint_breakableparams_t	breakable;
	matrix3x4_t		constraintToReference;
	matrix3x4_t		constraintToAttached;
	bool			bHasRagdollTransforms;
};

class CBox3DConstraint : public IPhysicsConstraint, public IBox3DObjectListener, public IBox3DController
{
public:
	CBox3DConstraint( CBox3DPhysicsEnvironment *pEnv, CBox3DPhysicsObject *pRef, CBox3DPhysicsObject *pAtt );
	~CBox3DConstraint();

	// Initializers from game parameters
	void					InitRagdoll( CBox3DConstraintGroup *pGroup, const constraint_ragdollparams_t &ragdoll );
	void					InitHinge( CBox3DConstraintGroup *pGroup, const constraint_hingeparams_t &hinge );
	void					InitFixed( CBox3DConstraintGroup *pGroup, const constraint_fixedparams_t &fixed );
	void					InitSliding( CBox3DConstraintGroup *pGroup, const constraint_slidingparams_t &sliding );
	void					InitBallsocket( CBox3DConstraintGroup *pGroup, const constraint_ballsocketparams_t &ballsocket );
	void					InitPulley( CBox3DConstraintGroup *pGroup, const constraint_pulleyparams_t &pulley );
	void					InitLength( CBox3DConstraintGroup *pGroup, const constraint_lengthparams_t &length );
	void					InitFromDesc( const B3VPJointDesc_t &desc, CBox3DConstraintGroup *pGroup, bool bActive );

	// IPhysicsConstraint
	void					Activate( void );
	void					Deactivate( void );
	void					SetGameData( void *gameData )	{ m_pGameData = gameData; }
	void					*GetGameData( void ) const		{ return m_pGameData; }
	IPhysicsObject			*GetReferenceObject( void ) const;
	IPhysicsObject			*GetAttachedObject( void ) const;
	void					SetLinearMotor( float speed, float maxLinearImpulse );
	void					SetAngularMotor( float rotSpeed, float maxAngularImpulse );
	void					UpdateRagdollTransforms( const matrix3x4_t &constraintToReference, const matrix3x4_t &constraintToAttached );
	bool					GetConstraintTransform( matrix3x4_t *pConstraintToReference, matrix3x4_t *pConstraintToAttached ) const;
	bool					GetConstraintParams( constraint_breakableparams_t *pParams ) const;
	void					OutputDebugInfo();

	// IBox3DObjectListener
	void					OnObjectDestroyed( CBox3DPhysicsObject *pObject );

	// IBox3DController (pulley)
	void					OnPreStep( float dt );

	// Internal
	void					OnBroken();
	bool					IsActive() const				{ return m_bActive; }
	const B3VPJointDesc_t	&GetDesc() const				{ return m_Desc; }
	CBox3DConstraintGroup	*GetGroup() const				{ return m_pGroup; }
	void					SetGroup( CBox3DConstraintGroup *pGroup );
	void					BindObjects( CBox3DPhysicsObject *pRef, CBox3DPhysicsObject *pAtt );
	void					ClearGroup()					{ m_pGroup = NULL; }

	// Restore bookkeeping
	void					*m_pSavedRef;
	void					*m_pSavedAtt;
	void					*m_pSavedGroup;
	bool					m_bSavedActive;

private:
	void					CreateJoint();
	void					DestroyJoint();
	void					InitDescDefaults();
	void					FinishInit( CBox3DConstraintGroup *pGroup, bool bActive );
	b3BodyId				BodyOf( CBox3DPhysicsObject *pObject ) const;

	CBox3DPhysicsEnvironment	*m_pEnv;
	CBox3DPhysicsObject			*m_pRef;
	CBox3DPhysicsObject			*m_pAtt;
	CBox3DConstraintGroup		*m_pGroup;
	void						*m_pGameData;
	b3JointId					m_Joint;
	B3VPJointDesc_t				m_Desc;
	bool						m_bActive;
	bool						m_bPulleyRegistered;
};

class CBox3DConstraintGroup : public IPhysicsConstraintGroup
{
public:
	CBox3DConstraintGroup( const constraint_groupparams_t &params ) : m_Params( params ) {}
	~CBox3DConstraintGroup();

	void Activate();
	bool IsInErrorState()										{ return false; }
	void ClearErrorState()										{}
	void GetErrorParams( constraint_groupparams_t *pParams )	{ if ( pParams ) *pParams = m_Params; }
	void SetErrorParams( const constraint_groupparams_t &params ) { m_Params = params; }
	void SolvePenetration( IPhysicsObject *pObj0, IPhysicsObject *pObj1 ) {}

	void AddConstraint( CBox3DConstraint *pConstraint );
	void RemoveConstraint( CBox3DConstraint *pConstraint );

	const constraint_groupparams_t &GetParams() const			{ return m_Params; }

private:
	constraint_groupparams_t			m_Params;
	CUtlVector< CBox3DConstraint * >	m_Constraints;
};

class CBox3DSpring : public IPhysicsSpring, public IBox3DController, public IBox3DObjectListener
{
public:
	CBox3DSpring( CBox3DPhysicsEnvironment *pEnv, CBox3DPhysicsObject *pStart, CBox3DPhysicsObject *pEnd, const springparams_t *pParams );
	~CBox3DSpring();

	void			GetEndpoints( Vector *worldPositionStart, Vector *worldPositionEnd );
	void			SetSpringConstant( float flSpringContant );
	void			SetSpringDamping( float flSpringDamping );
	void			SetSpringLength( float flSpringLenght );
	IPhysicsObject	*GetStartObject( void )		{ return m_pStart; }
	IPhysicsObject	*GetEndObject( void )		{ return m_pEnd; }

	void			OnPreStep( float dt );
	void			OnObjectDestroyed( CBox3DPhysicsObject *pObject );

	// Save/restore
	struct SaveData_t
	{
		float	constant;
		float	naturalLength;
		float	damping;
		float	relativeDamping;
		Vector	localStart;
		Vector	localEnd;
		bool	onlyStretch;
	};
	void			GetSaveData( SaveData_t &data ) const;
	void			BindObjects( CBox3DPhysicsObject *pStart, CBox3DPhysicsObject *pEnd );

	void			*m_pSavedStart;
	void			*m_pSavedEnd;

private:
	CBox3DPhysicsEnvironment	*m_pEnv;
	CBox3DPhysicsObject			*m_pStart;
	CBox3DPhysicsObject			*m_pEnd;
	Vector						m_LocalStart;		// object space, inches
	Vector						m_LocalEnd;
	float						m_flConstant;
	float						m_flNaturalLength;	// inches
	float						m_flDamping;
	float						m_flRelativeDamping;
	bool						m_bOnlyStretch;

public:
	CBox3DSpring( CBox3DPhysicsEnvironment *pEnv, const SaveData_t &data );
};

#endif // B3VP_CONSTRAINTS_H
