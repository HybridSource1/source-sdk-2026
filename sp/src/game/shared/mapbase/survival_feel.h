//=============================================================================
//
// Purpose:	Survival-horror player feel for Mapbase.
//
//			Client-side presentation layer: footstep-synced camera bob, landing
//			impact, strafe lean, and a spring-driven, weight-aware weapon sway
//			that replaces the stock HL2 viewmodel lag and bob.
//
//			Movement-side changes (acceleration, backpedal penalty, jump and
//			landing tuning) live in hl_gamemovement.cpp so they stay predicted.
//
//			Everything is tunable through cl_surv_* (client) and sv_surv_*
//			(replicated) convars. "cl_surv_enable 0" restores stock behaviour.
//
//=============================================================================

#ifndef SURVIVAL_FEEL_H
#define SURVIVAL_FEEL_H
#ifdef _WIN32
#pragma once
#endif

#if defined( CLIENT_DLL ) && defined( MAPBASE )

class C_BasePlayer;
class C_BaseViewModel;
class C_BaseCombatWeapon;

namespace SurvivalFeel
{
	// True when the survival presentation layer is active.
	bool IsEnabled();

	// Advances all simulation state. Safe to call several times per frame;
	// only the first call in a frame does any work.
	void Update( C_BasePlayer *pPlayer );

	// Adds head bob, strafe lean, landing dip and breathing to the camera.
	void ApplyCamera( C_BasePlayer *pPlayer, Vector &eyeOrigin, QAngle &eyeAngles );

	// Adds weighty sway, figure-eight bob, sprint pose, wall push-back and
	// landing dip to a viewmodel. Replaces stock lag + bob when enabled.
	void ApplyViewModel( C_BaseViewModel *pViewModel, C_BaseCombatWeapon *pWeapon,
						 Vector &origin, QAngle &angles, const QAngle &eyeAngles );

	// Hand-held flashlight: offsets the light to the hand and makes it lag
	// behind head turns, bob with steps and droop while sprinting.
	void ApplyFlashlight( C_BasePlayer *pPlayer, Vector &vecPos,
						  Vector &vecForward, Vector &vecRight, Vector &vecUp );

	// Brightness multiplier for the bulb "catching" after a battery swap (0..1).
	float GetFlashlightWarmup();
}

#endif // CLIENT_DLL && MAPBASE

#endif // SURVIVAL_FEEL_H
