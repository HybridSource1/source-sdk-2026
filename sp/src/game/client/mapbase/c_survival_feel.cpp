//=============================================================================
//
// Purpose:	Survival-horror player feel for Mapbase (client presentation).
//			See shared/mapbase/survival_feel.h for an overview.
//
//			All motion is driven by sub-stepped damped springs so it is
//			frame-rate independent and settles with a small, heavy overshoot
//			instead of the stock linear "drift".
//
//=============================================================================

#include "cbase.h"
#include "mapbase/survival_feel.h"
#include "c_baseplayer.h"
#include "c_basehlplayer.h"
#include "baseviewmodel_shared.h"
#include "basecombatweapon_shared.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-----------------------------------------------------------------------------
// Master switches
//-----------------------------------------------------------------------------
ConVar cl_surv_enable( "cl_surv_enable", "1", FCVAR_ARCHIVE, "Enable the survival-horror camera and weapon feel. 0 restores stock HL2 behaviour." );
ConVar cl_surv_camera_scale( "cl_surv_camera_scale", "1.0", FCVAR_ARCHIVE, "Overall scale for camera motion (bob, lean, landing). Lower if it causes motion sickness; 0 disables camera motion only." );
ConVar cl_surv_weapon_scale( "cl_surv_weapon_scale", "1.0", FCVAR_ARCHIVE, "Overall scale for weapon motion." );

//-----------------------------------------------------------------------------
// Gait
//-----------------------------------------------------------------------------
ConVar cl_surv_ref_speed( "cl_surv_ref_speed", "140", 0, "Speed treated as a normal walk when scaling bob (match hl2_normspeed)." );
ConVar cl_surv_stride_walk( "cl_surv_stride_walk", "34", 0, "Units travelled per footstep at walking pace." );
ConVar cl_surv_stride_sprint( "cl_surv_stride_sprint", "52", 0, "Units travelled per footstep when sprinting." );

//-----------------------------------------------------------------------------
// Camera
//-----------------------------------------------------------------------------
ConVar cl_surv_cam_bob_vert( "cl_surv_cam_bob_vert", "0.55", 0, "Camera vertical dip per footstep (units)." );
ConVar cl_surv_cam_bob_roll( "cl_surv_cam_bob_roll", "0.35", 0, "Camera roll sway per stride (degrees)." );
ConVar cl_surv_cam_bob_sprint( "cl_surv_cam_bob_sprint", "1.8", 0, "Camera bob multiplier while sprinting." );
ConVar cl_surv_cam_lean( "cl_surv_cam_lean", "1.1", 0, "Camera roll when strafing at reference speed (degrees)." );
ConVar cl_surv_cam_land( "cl_surv_cam_land", "1.0", 0, "Camera landing impact scale." );
ConVar cl_surv_cam_breath( "cl_surv_cam_breath", "0.25", 0, "Camera breathing pitch when fully winded (degrees)." );

//-----------------------------------------------------------------------------
// Weapon sway (look inertia)
//-----------------------------------------------------------------------------
ConVar cl_surv_sway_inertia( "cl_surv_sway_inertia", "0.011", 0, "How far the weapon trails behind camera rotation, in degrees per degree/sec of turn." );
ConVar cl_surv_sway_max( "cl_surv_sway_max", "5.5", 0, "Maximum look-sway angle (degrees)." );
ConVar cl_surv_sway_stiffness( "cl_surv_sway_stiffness", "110", 0, "Look-sway spring stiffness. Lower = heavier, slower to catch up." );
ConVar cl_surv_sway_damping( "cl_surv_sway_damping", "0.68", 0, "Look-sway damping ratio. 1 = no overshoot, lower = more settle wobble." );
ConVar cl_surv_sway_roll( "cl_surv_sway_roll", "0.45", 0, "Weapon roll into turns, as a fraction of yaw sway." );
ConVar cl_surv_sway_shift( "cl_surv_sway_shift", "0.22", 0, "Weapon positional shift per degree of look sway (units)." );

//-----------------------------------------------------------------------------
// Weapon movement
//-----------------------------------------------------------------------------
ConVar cl_surv_move_lag( "cl_surv_move_lag", "0.0045", 0, "Weapon positional lag per unit/sec of body velocity." );
ConVar cl_surv_move_tilt( "cl_surv_move_tilt", "3.0", 0, "Weapon roll when strafing at reference speed (degrees)." );
ConVar cl_surv_move_stiffness( "cl_surv_move_stiffness", "70", 0, "Weapon body-motion spring stiffness." );
ConVar cl_surv_move_damping( "cl_surv_move_damping", "0.55", 0, "Weapon body-motion damping ratio." );

ConVar cl_surv_bob_lateral( "cl_surv_bob_lateral", "0.32", 0, "Weapon figure-eight lateral bob (units)." );
ConVar cl_surv_bob_vertical( "cl_surv_bob_vertical", "0.28", 0, "Weapon figure-eight vertical bob (units)." );
ConVar cl_surv_bob_angle( "cl_surv_bob_angle", "0.9", 0, "Weapon bob rotation (degrees)." );
ConVar cl_surv_bob_sprint( "cl_surv_bob_sprint", "2.2", 0, "Weapon bob multiplier while sprinting." );

ConVar cl_surv_sprint_pitch( "cl_surv_sprint_pitch", "9", 0, "Sprint low-ready pose: muzzle down (degrees)." );
ConVar cl_surv_sprint_yaw( "cl_surv_sprint_yaw", "14", 0, "Sprint low-ready pose: muzzle across body (degrees)." );
ConVar cl_surv_sprint_roll( "cl_surv_sprint_roll", "-7", 0, "Sprint low-ready pose: cant (degrees)." );
ConVar cl_surv_sprint_drop( "cl_surv_sprint_drop", "1.4", 0, "Sprint low-ready pose: drop (units)." );

ConVar cl_surv_land_weapon( "cl_surv_land_weapon", "1.6", 0, "Weapon landing dip, relative to the camera dip." );
ConVar cl_surv_crouch_drop( "cl_surv_crouch_drop", "0.5", 0, "Weapon drop while crouched (units)." );
ConVar cl_surv_crouch_roll( "cl_surv_crouch_roll", "-2.5", 0, "Weapon cant while crouched (degrees)." );

ConVar cl_surv_wall_dist( "cl_surv_wall_dist", "32", 0, "Distance at which the weapon starts pulling back from walls. 0 disables." );
ConVar cl_surv_wall_pull( "cl_surv_wall_pull", "5", 0, "How far the weapon retracts at a wall (units)." );
ConVar cl_surv_wall_pitch( "cl_surv_wall_pitch", "16", 0, "How far the weapon tips down at a wall (degrees)." );

ConVar cl_surv_breath_idle( "cl_surv_breath_idle", "0.18", 0, "Idle breathing sway on the weapon (degrees)." );
ConVar cl_surv_breath_winded( "cl_surv_breath_winded", "0.9", 0, "Extra breathing sway when fully winded (degrees)." );
ConVar cl_surv_fatigue_gain( "cl_surv_fatigue_gain", "0.3", 0, "Fatigue gained per second of sprinting (0-1 scale)." );
ConVar cl_surv_fatigue_recover( "cl_surv_fatigue_recover", "0.1", 0, "Fatigue recovered per second when not sprinting." );

//-----------------------------------------------------------------------------
// Flashlight (hand-held)
//-----------------------------------------------------------------------------
ConVar cl_surv_flashlight_lag( "cl_surv_flashlight_lag", "1.0", FCVAR_ARCHIVE, "How much the flashlight stays put in the world while you turn (0 = locked to view, 1 = full hand lag)." );
ConVar cl_surv_flashlight_stiffness( "cl_surv_flashlight_stiffness", "40", 0, "How quickly the hand brings the beam back to centre. Lower = lazier." );
ConVar cl_surv_flashlight_damping( "cl_surv_flashlight_damping", "0.8", 0, "Beam settle damping ratio." );
ConVar cl_surv_flashlight_max( "cl_surv_flashlight_max", "14", 0, "Maximum angle the beam can trail behind the view (degrees)." );
ConVar cl_surv_flashlight_bob( "cl_surv_flashlight_bob", "1.1", 0, "Beam sway per stride (degrees)." );
ConVar cl_surv_flashlight_sprint_drop( "cl_surv_flashlight_sprint_drop", "9", 0, "Beam points down this far while sprinting (degrees)." );
ConVar cl_surv_flashlight_shake( "cl_surv_flashlight_shake", "0.35", 0, "Hand tremor when winded (degrees)." );
ConVar cl_surv_flashlight_hand_right( "cl_surv_flashlight_hand_right", "7", 0, "How far right of the eye the light is held (units). Gives shadows depth." );
ConVar cl_surv_flashlight_warmup( "cl_surv_flashlight_warmup", "0.35", 0, "Seconds the bulb sputters after a battery swap." );

ConVar cl_surv_pitch_offset( "cl_surv_pitch_offset", "0.5", 0, "Scale of the stock HL2 look-up/down viewmodel offset (prevents exposed polys)." );

//-----------------------------------------------------------------------------
// Small math helpers
//-----------------------------------------------------------------------------
namespace
{
	// Frame-rate independent exponential approach.
	inline float Approach( float flCurrent, float flTarget, float flRate, float flDt )
	{
		return flTarget + ( flCurrent - flTarget ) * expf( -flRate * flDt );
	}

	// Critically/under-damped spring, sub-stepped for stability at low frame rates.
	struct Spring
	{
		Vector pos;
		Vector vel;

		void Reset() { pos.Init(); vel.Init(); }

		void Step( const Vector &target, float flStiffness, float flDampingRatio, float flDt )
		{
			const float flStep = 1.0f / 240.0f;
			const float flDamp = 2.0f * flDampingRatio * sqrtf( MAX( flStiffness, 0.001f ) );

			while ( flDt > 0.0f )
			{
				float h = MIN( flDt, flStep );
				Vector accel = ( target - pos ) * flStiffness - vel * flDamp;
				vel += accel * h;
				pos += vel * h;
				flDt -= h;
			}
		}
	};

	struct State_t
	{
		int		nLastFrame;
		float	flLastTime;
		bool	bInitialized;

		QAngle	angLastEye;
		Vector	vecLookVel;			// smoothed pitch/yaw rate (deg/s) in x/y

		float	flStridePhase;		// 0..2PI covers two footsteps
		float	flBobAmount;		// 0..~1.5, how strongly we bob
		float	flSprint;			// 0..1 blend
		float	flCrouch;			// 0..1 blend
		float	flLean;				// signed, normalized strafe speed
		float	flWall;				// 0..1 wall proximity

		bool	bWasOnGround;
		float	flAirFallSpeed;		// max downward speed this airtime

		float	flFatigue;			// 0..1
		float	flBreathPhase;

		Spring	flash;				// x = pitch, y = yaw beam offset from view (degrees)
		bool	bWasSwapping;
		float	flSwapDoneTime;

		Spring	look;				// x = pitch, y = yaw sway (degrees)
		Spring	body;				// local-space positional lag (units)
		Spring	land;				// x = vertical landing offset (units)
	};

	State_t s;

	void ResetState( C_BasePlayer *pPlayer )
	{
		memset( &s, 0, sizeof( s ) );
		s.angLastEye = pPlayer ? pPlayer->EyeAngles() : vec3_angle;
		s.bWasOnGround = true;
		s.bInitialized = true;
		s.nLastFrame = -1;
	}

	float WeaponWeight( C_BaseCombatWeapon *pWeapon )
	{
		if ( !pWeapon )
			return 1.0f;

		// Rough mass by HL2 slot: melee, sidearm, SMG, rifle/shotgun, heavy.
		static const float s_flSlotWeight[] = { 0.85f, 0.7f, 1.0f, 1.3f, 1.6f, 1.2f };
		int iSlot = clamp( pWeapon->GetSlot(), 0, (int)ARRAYSIZE( s_flSlotWeight ) - 1 );
		float flWeight = s_flSlotWeight[iSlot];

		// Mapbase weapon scripts can tune per weapon with "swayscale".
		float flScriptScale = pWeapon->GetWpnData().m_flSwayScale;
		if ( flScriptScale > 0.0f )
			flWeight *= flScriptScale;

		return clamp( flWeight, 0.2f, 3.0f );
	}
}

//-----------------------------------------------------------------------------
bool SurvivalFeel::IsEnabled()
{
	return cl_surv_enable.GetBool();
}

//-----------------------------------------------------------------------------
void SurvivalFeel::Update( C_BasePlayer *pPlayer )
{
	if ( !pPlayer )
		return;

	if ( !s.bInitialized || gpGlobals->curtime < s.flLastTime - 1.0f )
		ResetState( pPlayer );	// first use or level change

	if ( s.nLastFrame == gpGlobals->framecount )
		return;
	s.nLastFrame = gpGlobals->framecount;
	s.flLastTime = gpGlobals->curtime;

	float dt = gpGlobals->frametime;
	if ( dt <= 0.0f )
		return;	// paused
	dt = MIN( dt, 0.1f );	// don't explode after a hitch

	// ---- Look rate ----
	QAngle angEye = pPlayer->EyeAngles();
	float flPitchRate = AngleDiff( angEye[PITCH], s.angLastEye[PITCH] ) / dt;
	float flYawRate = AngleDiff( angEye[YAW], s.angLastEye[YAW] ) / dt;
	s.angLastEye = angEye;

	// Flashlight: the beam holds its world direction while the head turns,
	// then the hand catches up. Clamped so it never leaves the screen.
	{
		float flLock = clamp( cl_surv_flashlight_lag.GetFloat(), 0.0f, 1.0f );
		float flMaxBeam = cl_surv_flashlight_max.GetFloat();
		s.flash.pos.x = clamp( s.flash.pos.x - flPitchRate * dt * flLock, -flMaxBeam, flMaxBeam );
		s.flash.pos.y = clamp( s.flash.pos.y - flYawRate * dt * flLock, -flMaxBeam, flMaxBeam );
	}

	// Smooth raw mouse deltas so high-poll mice don't jitter the weapon.
	s.vecLookVel.x = Approach( s.vecLookVel.x, flPitchRate, 30.0f, dt );
	s.vecLookVel.y = Approach( s.vecLookVel.y, flYawRate, 30.0f, dt );

	// ---- Body state ----
	bool bAlive = pPlayer->IsAlive() && pPlayer->GetMoveType() != MOVETYPE_NOCLIP;
	bool bOnGround = bAlive && ( pPlayer->GetFlags() & FL_ONGROUND ) != 0;
	bool bOnLadder = pPlayer->GetMoveType() == MOVETYPE_LADDER;

	Vector vecVel = pPlayer->GetAbsVelocity();
	float flSpeed = vecVel.Length2D();
	float flRef = MAX( cl_surv_ref_speed.GetFloat(), 1.0f );

	Vector vecFwd, vecRight;
	AngleVectors( QAngle( 0, angEye[YAW], 0 ), &vecFwd, &vecRight, NULL );
	Vector vecLocalVel( DotProduct( vecVel, vecFwd ), DotProduct( vecVel, vecRight ), vecVel.z );

	bool bSprinting = false;
	C_BaseHLPlayer *pHLPlayer = dynamic_cast<C_BaseHLPlayer*>( pPlayer );
	if ( pHLPlayer )
		bSprinting = pHLPlayer->IsSprinting() && bOnGround && flSpeed > flRef * 0.5f;

	s.flSprint = Approach( s.flSprint, bSprinting ? 1.0f : 0.0f, 6.0f, dt );
	s.flCrouch = Approach( s.flCrouch, ( pPlayer->GetFlags() & FL_DUCKING ) ? 1.0f : 0.0f, 8.0f, dt );

	// ---- Gait: phase advances with distance walked, so steps match speed ----
	float flBobTarget = ( bOnGround && !bOnLadder ) ? MIN( flSpeed / flRef, 1.6f ) : 0.0f;
	if ( flSpeed < 8.0f )
		flBobTarget = 0.0f;
	s.flBobAmount = Approach( s.flBobAmount, flBobTarget, bOnGround ? 7.0f : 4.0f, dt );

	if ( bOnGround )
	{
		float flStride = Lerp( s.flSprint, cl_surv_stride_walk.GetFloat(), cl_surv_stride_sprint.GetFloat() );
		flStride = MAX( flStride, 4.0f );
		// One full phase cycle (2PI) = two footsteps.
		s.flStridePhase += ( flSpeed * dt ) / ( 2.0f * flStride ) * ( 2.0f * M_PI_F );
		s.flStridePhase = fmodf( s.flStridePhase, 2.0f * M_PI_F );
	}

	s.flLean = Approach( s.flLean, bOnGround ? clamp( vecLocalVel.y / flRef, -1.5f, 1.5f ) : s.flLean * 0.5f, 6.0f, dt );

	// ---- Landing ----
	if ( !bOnGround && bAlive && !bOnLadder )
	{
		s.flAirFallSpeed = MAX( s.flAirFallSpeed, -vecVel.z );
	}
	else
	{
		if ( !s.bWasOnGround && s.flAirFallSpeed > 60.0f )
		{
			// Impulse proportional to impact speed; saturates on big drops.
			float flImpact = MIN( s.flAirFallSpeed, 700.0f );
			s.land.vel.x -= flImpact * 0.065f;
		}
		s.flAirFallSpeed = 0.0f;
	}
	s.bWasOnGround = bOnGround || bOnLadder;
	s.land.Step( vec3_origin, 160.0f, 0.5f, dt );
	s.land.pos.x = clamp( s.land.pos.x, -6.0f, 2.0f );

	// ---- Fatigue & breathing ----
	if ( s.flSprint > 0.5f )
		s.flFatigue += cl_surv_fatigue_gain.GetFloat() * dt;
	else
		s.flFatigue -= cl_surv_fatigue_recover.GetFloat() * dt;
	s.flFatigue = clamp( s.flFatigue, 0.0f, 1.0f );

	float flBreathHz = 0.22f + 0.38f * s.flFatigue;
	s.flBreathPhase = fmodf( s.flBreathPhase + flBreathHz * dt * 2.0f * M_PI_F, 2.0f * M_PI_F * 2.0f );

	// ---- Weapon springs ----
	C_BaseCombatWeapon *pWeapon = pPlayer->GetActiveWeapon();
	float flWeight = WeaponWeight( pWeapon );

	float flMax = cl_surv_sway_max.GetFloat();
	float flInertia = cl_surv_sway_inertia.GetFloat() * flWeight;
	Vector vecLookTarget( clamp( -s.vecLookVel.x * flInertia, -flMax, flMax ),
						  clamp( -s.vecLookVel.y * flInertia, -flMax, flMax ), 0.0f );
	s.look.Step( vecLookTarget, cl_surv_sway_stiffness.GetFloat() / flWeight, cl_surv_sway_damping.GetFloat(), dt );

	float flLag = cl_surv_move_lag.GetFloat() * flWeight;
	Vector vecBodyTarget( clamp( -vecLocalVel.x * flLag, -2.5f, 2.5f ),
						  clamp( -vecLocalVel.y * flLag, -2.5f, 2.5f ),
						  clamp( -vecLocalVel.z * flLag * 0.6f, -2.0f, 2.0f ) );
	s.body.Step( vecBodyTarget, cl_surv_move_stiffness.GetFloat() / flWeight, cl_surv_move_damping.GetFloat(), dt );

	// ---- Wall proximity ----
	float flWallDist = cl_surv_wall_dist.GetFloat();
	float flWallTarget = 0.0f;
	if ( flWallDist > 0.0f && bAlive )
	{
		Vector vecEyeFwd;
		AngleVectors( angEye, &vecEyeFwd );
		Vector vecStart = pPlayer->EyePosition();
		trace_t tr;
		UTIL_TraceHull( vecStart, vecStart + vecEyeFwd * flWallDist, Vector( -2, -2, -2 ), Vector( 2, 2, 2 ),
						MASK_SOLID, pPlayer, COLLISION_GROUP_NONE, &tr );
		if ( tr.fraction < 1.0f && !tr.startsolid )
			flWallTarget = 1.0f - tr.fraction;
	}
	s.flWall = Approach( s.flWall, flWallTarget, 9.0f, dt );

	// ---- Flashlight beam ----
	{
		float flStepSway = s.flBobAmount * Lerp( s.flSprint, 1.0f, 2.0f ) * cl_surv_flashlight_bob.GetFloat();
		float flTremor = cl_surv_flashlight_shake.GetFloat() * s.flFatigue;

		Vector vecBeamTarget(
			-0.5f * ( 1.0f - cosf( 2.0f * s.flStridePhase ) ) * flStepSway * -0.6f	// nods down on each step
				+ cl_surv_flashlight_sprint_drop.GetFloat() * s.flSprint
				+ sinf( s.flBreathPhase ) * flTremor
				- s.land.pos.x * 1.5f,
			sinf( s.flStridePhase ) * flStepSway
				+ sinf( s.flBreathPhase * 1.7f + 1.3f ) * flTremor,
			0.0f );

		s.flash.Step( vecBeamTarget, cl_surv_flashlight_stiffness.GetFloat(), cl_surv_flashlight_damping.GetFloat(), dt );
	}

	// ---- Battery swap -> warm-up sputter ----
	{
		C_BaseHLPlayer *pHL = dynamic_cast<C_BaseHLPlayer*>( pPlayer );
		bool bSwapping = pHL && pHL->m_HL2Local.m_bSurvFlashSwapping;
		if ( s.bWasSwapping && !bSwapping )
			s.flSwapDoneTime = gpGlobals->curtime;
		s.bWasSwapping = bSwapping;
	}
}

//-----------------------------------------------------------------------------
void SurvivalFeel::ApplyFlashlight( C_BasePlayer *pPlayer, Vector &vecPos,
									Vector &vecForward, Vector &vecRight, Vector &vecUp )
{
	if ( !IsEnabled() || !pPlayer )
		return;

	Update( pPlayer );

	QAngle angBeam = pPlayer->EyeAngles();
	angBeam[PITCH] += s.flash.pos.x;
	angBeam[YAW] += s.flash.pos.y;
	angBeam[ROLL] += -s.flash.pos.y * 0.3f + s.flLean * 2.0f;

	// Hold it in the right hand. Trace so it never pokes through a wall.
	Vector vecHand = vecPos + vecRight * cl_surv_flashlight_hand_right.GetFloat();
	trace_t tr;
	UTIL_TraceHull( vecPos, vecHand, Vector( -2, -2, -2 ), Vector( 2, 2, 2 ),
					MASK_SOLID, pPlayer, COLLISION_GROUP_NONE, &tr );
	vecPos = tr.endpos;

	AngleVectors( angBeam, &vecForward, &vecRight, &vecUp );
}

//-----------------------------------------------------------------------------
float SurvivalFeel::GetFlashlightWarmup()
{
	float flDuration = cl_surv_flashlight_warmup.GetFloat();
	if ( flDuration <= 0.0f || s.flSwapDoneTime <= 0.0f )
		return 1.0f;

	float flSince = gpGlobals->curtime - s.flSwapDoneTime;
	if ( flSince < 0.0f || flSince >= flDuration )
		return 1.0f;

	// A few hard stutters, then up to full.
	int nSlot = (int)( flSince * 22.0f );
	bool bOn = ( ( nSlot * 2654435761u ) >> 28 ) & 1;
	float flRamp = flSince / flDuration;
	return bOn ? Lerp( flRamp, 0.5f, 1.0f ) : 0.05f;
}

//-----------------------------------------------------------------------------
void SurvivalFeel::ApplyCamera( C_BasePlayer *pPlayer, Vector &eyeOrigin, QAngle &eyeAngles )
{
	if ( !IsEnabled() || !pPlayer || !pPlayer->IsAlive() || pPlayer->GetMoveType() == MOVETYPE_NOCLIP )
		return;

	Update( pPlayer );

	float flScale = cl_surv_camera_scale.GetFloat();
	if ( flScale <= 0.0f )
		return;

	float flBob = s.flBobAmount * Lerp( s.flSprint, 1.0f, cl_surv_cam_bob_sprint.GetFloat() ) * flScale;

	// Dip at each footfall: (1 - cos 2p)/2 is 0 between steps, 1 on the foot.
	float flStep = 0.5f * ( 1.0f - cosf( 2.0f * s.flStridePhase ) );
	float flSide = sinf( s.flStridePhase );

	eyeOrigin.z -= flStep * cl_surv_cam_bob_vert.GetFloat() * flBob;
	eyeAngles[ROLL] += flSide * cl_surv_cam_bob_roll.GetFloat() * flBob;

	// Strafe lean.
	eyeAngles[ROLL] += s.flLean * cl_surv_cam_lean.GetFloat() * flScale;

	// Landing: drop and a small nod down.
	float flLand = s.land.pos.x * cl_surv_cam_land.GetFloat() * flScale;
	eyeOrigin.z += flLand;
	eyeAngles[PITCH] -= flLand * 0.6f;

	// Heavy breathing after sprinting.
	eyeAngles[PITCH] += sinf( s.flBreathPhase ) * cl_surv_cam_breath.GetFloat() * s.flFatigue * flScale;
}

//-----------------------------------------------------------------------------
void SurvivalFeel::ApplyViewModel( C_BaseViewModel *pViewModel, C_BaseCombatWeapon *pWeapon,
								   Vector &origin, QAngle &angles, const QAngle &eyeAngles )
{
	C_BasePlayer *pPlayer = pViewModel ? ToBasePlayer( pViewModel->GetOwner() ) : NULL;
	if ( !pPlayer )
		return;

	Update( pPlayer );

	float flScale = cl_surv_weapon_scale.GetFloat();

	Vector forward, right, up;
	AngleVectors( eyeAngles, &forward, &right, &up );

	// ---- Look inertia: weapon trails the turn, rolls into it ----
	float flSwayPitch = s.look.pos.x * flScale;
	float flSwayYaw = s.look.pos.y * flScale;

	angles[PITCH] += flSwayPitch;
	angles[YAW] += flSwayYaw;
	angles[ROLL] -= flSwayYaw * cl_surv_sway_roll.GetFloat();

	float flShift = cl_surv_sway_shift.GetFloat();
	VectorMA( origin, flSwayYaw * flShift, right, origin );
	VectorMA( origin, -flSwayPitch * flShift, up, origin );

	// ---- Body inertia: positional lag from acceleration ----
	Vector vecBody = s.body.pos * flScale;
	VectorMA( origin, vecBody.x, forward, origin );
	VectorMA( origin, vecBody.y, right, origin );
	origin.z += vecBody.z;
	angles[PITCH] -= vecBody.z * 1.2f;	// tips up on jump, down on fall

	// Strafe tilt.
	angles[ROLL] += s.flLean * cl_surv_move_tilt.GetFloat() * flScale;

	// ---- Figure-eight gait bob ----
	float flBob = s.flBobAmount * Lerp( s.flSprint, 1.0f, cl_surv_bob_sprint.GetFloat() ) * flScale;
	float flLat = sinf( s.flStridePhase );
	float flVert = -0.5f * ( 1.0f - cosf( 2.0f * s.flStridePhase ) );	// dips on each step

	VectorMA( origin, flLat * cl_surv_bob_lateral.GetFloat() * flBob, right, origin );
	VectorMA( origin, flVert * cl_surv_bob_vertical.GetFloat() * flBob, up, origin );

	float flBobAng = cl_surv_bob_angle.GetFloat() * flBob;
	angles[ROLL] += flLat * flBobAng;
	angles[YAW] -= flLat * flBobAng * 0.5f;
	angles[PITCH] -= flVert * flBobAng * 0.6f;

	// ---- Sprint low-ready pose ----
	float flSprint = s.flSprint * flScale;
	angles[PITCH] += cl_surv_sprint_pitch.GetFloat() * flSprint;
	angles[YAW] += cl_surv_sprint_yaw.GetFloat() * flSprint;
	angles[ROLL] += cl_surv_sprint_roll.GetFloat() * flSprint;
	VectorMA( origin, -cl_surv_sprint_drop.GetFloat() * flSprint, up, origin );

	// ---- Crouch ----
	VectorMA( origin, -cl_surv_crouch_drop.GetFloat() * s.flCrouch * flScale, up, origin );
	angles[ROLL] += cl_surv_crouch_roll.GetFloat() * s.flCrouch * flScale;

	// ---- Landing: the weapon is heavier than your head ----
	float flLand = s.land.pos.x * cl_surv_land_weapon.GetFloat() * flScale;
	origin.z += flLand;
	angles[PITCH] -= flLand * 1.2f;

	// ---- Wall: pull back and tip down instead of clipping ----
	float flWall = s.flWall * flScale;
	VectorMA( origin, -cl_surv_wall_pull.GetFloat() * flWall, forward, origin );
	angles[PITCH] += cl_surv_wall_pitch.GetFloat() * flWall;

	// ---- Breathing ----
	float flBreath = ( cl_surv_breath_idle.GetFloat() + cl_surv_breath_winded.GetFloat() * s.flFatigue ) * flScale;
	angles[PITCH] += sinf( s.flBreathPhase ) * flBreath;
	angles[YAW] += sinf( s.flBreathPhase * 0.5f ) * flBreath * 0.6f;	// slow figure-eight

	// ---- Stock look-up/down offset, scaled (hides exposed polys) ----
	float pitch = eyeAngles[PITCH];
	if ( pitch > 180.0f )
		pitch -= 360.0f;
	else if ( pitch < -180.0f )
		pitch += 360.0f;

	float flPitchScale = cl_surv_pitch_offset.GetFloat();
	VectorMA( origin, -pitch * 0.035f * flPitchScale, forward, origin );
	VectorMA( origin, -pitch * 0.03f * flPitchScale, right, origin );
	VectorMA( origin, -pitch * 0.02f * flPitchScale, up, origin );
}
