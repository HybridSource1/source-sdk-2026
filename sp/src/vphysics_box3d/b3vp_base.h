//=================================================================================================
//
// VPhysics Box3D - shared header
//
// A VPhysics (IPhysics / IPhysicsCollision / IPhysicsSurfaceProps) implementation that runs on
// Erin Catto's Box3D physics engine (https://github.com/erincatto/box3d).
//
// Conventions:
//   * Box3D runs in SI units (metres, kilograms, seconds, radians) with Source's axes (Z up).
//   * Source passes inches, degrees and HL "impulse" units through the VPhysics interfaces.
//   * A body's origin is the Source object origin (not the centre of mass), so positions map 1:1.
//   * Angular velocities passed to/from the game are in the object's LOCAL space (deg/s),
//     torques passed to ApplyTorqueCenter are in WORLD space, as documented in
//     vphysics_interface.h.
//
//=================================================================================================

#ifndef B3VP_BASE_H
#define B3VP_BASE_H

#ifdef _WIN32
#pragma once
#endif

// STL must come before the Source headers (minmax.h defines min/max macros)
#include <vector>
#include <map>
#include <algorithm>
#include <string>
#include <set>

#include "tier0/basetypes.h"
#include "tier0/dbg.h"
#include "tier1/interface.h"
#include "tier1/strtools.h"
#include "tier1/utlvector.h"
#include "tier1/convar.h"
#include "mathlib/mathlib.h"
#include "mathlib/vector.h"

#include "vphysics_interface.h"
#include "vphysics/collision_set.h"
#include "vphysics/constraints.h"
#include "vphysics/friction.h"
#include "vphysics/object_hash.h"
#include "vphysics/performance.h"
#include "vphysics/player_controller.h"
#include "vphysics/stats.h"
#include "vphysics/vehicles.h"
#include "vphysics/virtualmesh.h"
#include "vcollide_parse.h"

// Box3D is plain C, its headers declare everything extern "C" themselves
#include "box3d/box3d.h"
#include "box3d/collision.h"
#include "box3d/math_functions.h"

//-------------------------------------------------------------------------------------------------
// Unit conversion
//-------------------------------------------------------------------------------------------------

#define B3VP_M_PER_IN	( 0.0254f )
#define B3VP_IN_PER_M	( 1.0f / 0.0254f )

inline float	HL2B3( float v )					{ return v * B3VP_M_PER_IN; }
inline float	B32HL( float v )					{ return v * B3VP_IN_PER_M; }
inline b3Vec3	HL2B3( const Vector &v )			{ b3Vec3 r = { v.x * B3VP_M_PER_IN, v.y * B3VP_M_PER_IN, v.z * B3VP_M_PER_IN }; return r; }
inline Vector	B32HL( const b3Vec3 &v )			{ return Vector( v.x * B3VP_IN_PER_M, v.y * B3VP_IN_PER_M, v.z * B3VP_IN_PER_M ); }

// Plain type conversion (directions, unitless)
inline b3Vec3	ToB3( const Vector &v )				{ b3Vec3 r = { v.x, v.y, v.z }; return r; }
inline Vector	ToHL( const b3Vec3 &v )				{ return Vector( v.x, v.y, v.z ); }

// Angular quantities (degrees <-> radians)
inline b3Vec3	HLAng2B3( const Vector &v )			{ b3Vec3 r = { DEG2RAD( v.x ), DEG2RAD( v.y ), DEG2RAD( v.z ) }; return r; }
inline Vector	B3Ang2HL( const b3Vec3 &v )			{ return Vector( RAD2DEG( v.x ), RAD2DEG( v.y ), RAD2DEG( v.z ) ); }

// Volumes, areas
inline float	HLVolume2B3( float v )				{ return v * ( B3VP_M_PER_IN * B3VP_M_PER_IN * B3VP_M_PER_IN ); }
inline float	B3Volume2HL( float v )				{ return v * ( B3VP_IN_PER_M * B3VP_IN_PER_M * B3VP_IN_PER_M ); }
inline float	B3Area2HL( float v )				{ return v * ( B3VP_IN_PER_M * B3VP_IN_PER_M ); }

// NOTE: like IVP, VPhysics passes rotational quantities in SI with degrees instead of radians.
// Inertia is kg*m^2 (GetInertia/SetInertia), angular impulses kg*m^2*deg/s, torques kg*m^2*deg/s^2.
// Only linear quantities (positions, velocities, forces, impulses) are in inches.

// HL torque / angular impulse -> SI
inline float	HLTorque2B3( float v )				{ return DEG2RAD( v ); }
inline b3Vec3	HLTorque2B3( const Vector &v )		{ return HLAng2B3( v ); }
inline Vector	B3Torque2HL( const b3Vec3 &v )		{ return B3Ang2HL( v ); }
// HL force units (kg * in / s^2) and impulses (kg * in / s) -> SI
inline float	HLForce2B3( float v )				{ return v * B3VP_M_PER_IN; }

// Energy: J (kg m^2/s^2) -> kg in^2 / s^2
inline float	B3Energy2HL( float v )				{ return v * ( B3VP_IN_PER_M * B3VP_IN_PER_M ); }

//-------------------------------------------------------------------------------------------------
// Rotation conversion
//-------------------------------------------------------------------------------------------------

inline b3Quat ToB3Quat( const Quaternion &q )
{
	b3Quat r;
	r.v.x = q.x; r.v.y = q.y; r.v.z = q.z; r.s = q.w;
	return r;
}

inline Quaternion ToHLQuat( const b3Quat &q )
{
	return Quaternion( q.v.x, q.v.y, q.v.z, q.s );
}

inline b3Quat AnglesToB3Quat( const QAngle &angles )
{
	Quaternion q;
	AngleQuaternion( angles, q );
	return b3NormalizeQuat( ToB3Quat( q ) );
}

inline QAngle B3QuatToAngles( const b3Quat &q )
{
	Quaternion hq = ToHLQuat( q );
	QAngle angles;
	QuaternionAngles( hq, angles );
	return angles;
}

inline b3Quat MatrixToB3Quat( const matrix3x4_t &m )
{
	Quaternion q;
	MatrixQuaternion( m, q );
	return b3NormalizeQuat( ToB3Quat( q ) );
}

// Builds a Source matrix (inches) from a Box3D transform (metres)
inline void B3TransformToMatrix( const b3Transform &xf, matrix3x4_t &out )
{
	QuaternionMatrix( ToHLQuat( xf.q ), B32HL( xf.p ), out );
}

// Builds a Box3D transform (metres) from a Source matrix (inches)
inline b3Transform MatrixToB3Transform( const matrix3x4_t &m )
{
	b3Transform xf;
	Vector origin;
	MatrixGetColumn( m, 3, origin );
	xf.p = HL2B3( origin );
	xf.q = MatrixToB3Quat( m );
	return xf;
}

inline b3Transform MakeB3Transform( const Vector &origin, const QAngle &angles )
{
	b3Transform xf;
	xf.p = HL2B3( origin );
	xf.q = AnglesToB3Quat( angles );
	return xf;
}

// Builds a quaternion that rotates the +Z axis onto 'axis' (unit vector)
b3Quat B3QuatFromZAxis( const b3Vec3 &axis );
// Builds a quaternion whose X axis is 'xAxis' and whose Y axis is as close as possible to 'yHint'
b3Quat B3QuatFromAxes( const b3Vec3 &xAxis, const b3Vec3 &yHint );

//-------------------------------------------------------------------------------------------------
// Misc helpers
//-------------------------------------------------------------------------------------------------

template < typename T >
inline void B3VP_EraseFast( std::vector< T > &vec, const T &value )
{
	typename std::vector< T >::iterator it = std::find( vec.begin(), vec.end(), value );
	if ( it != vec.end() )
	{
		*it = vec.back();
		vec.pop_back();
	}
}

template < typename T >
inline bool B3VP_Contains( const std::vector< T > &vec, const T &value )
{
	return std::find( vec.begin(), vec.end(), value ) != vec.end();
}

// Logging
#define B3VP_LOG_PREFIX "[VPhysics Box3D] "
#define B3VP_Msg( ... )			Msg( B3VP_LOG_PREFIX __VA_ARGS__ )
#define B3VP_Warning( ... )		Warning( B3VP_LOG_PREFIX __VA_ARGS__ )
#define B3VP_DevMsg( ... )		DevMsg( B3VP_LOG_PREFIX __VA_ARGS__ )

// Warns once per call site about an unimplemented feature
#define B3VP_Unsupported( what ) \
	do { static bool s_bWarned = false; if ( !s_bWarned ) { s_bWarned = true; DevWarning( B3VP_LOG_PREFIX "%s is not supported by the Box3D backend.\n", what ); } } while ( 0 )

// Reserved material index the game uses for shadow objects (see physics_shared / MATERIAL_INDEX_SHADOW)
#define B3VP_MATERIAL_INDEX_SHADOW		0xF000

class CBox3DPhysicsObject;
class CBox3DPhysicsEnvironment;

// Shape user data holds the owning physics object. These helpers keep the casts in one place.
inline CBox3DPhysicsObject *GetShapeObject( b3ShapeId id )
{
	return static_cast< CBox3DPhysicsObject * >( b3Shape_GetUserData( id ) );
}

inline CBox3DPhysicsObject *GetBodyObject( b3BodyId id )
{
	return static_cast< CBox3DPhysicsObject * >( b3Body_GetUserData( id ) );
}

inline bool B3IdEqual( b3ShapeId a, b3ShapeId b )
{
	return a.index1 == b.index1 && a.world0 == b.world0 && a.generation == b.generation;
}

#endif // B3VP_BASE_H
