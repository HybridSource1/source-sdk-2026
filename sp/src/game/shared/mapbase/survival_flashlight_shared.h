//=============================================================================
//
// Purpose:	Survival-horror flashlight: finite batteries, swaps, spare
//			battery pickups (item_flashlight_battery), and a dying-bulb look.
//
//			Server owns the charge and spare count; they are networked in
//			CHL2PlayerLocalData so the client can dim, flicker and draw HUD.
//
//=============================================================================

#ifndef SURVIVAL_FLASHLIGHT_SHARED_H
#define SURVIVAL_FLASHLIGHT_SHARED_H
#ifdef _WIN32
#pragma once
#endif

#ifdef MAPBASE

extern ConVar sv_surv_flashlight;
extern ConVar sv_surv_flashlight_life;
extern ConVar sv_surv_flashlight_recharge;
extern ConVar sv_surv_flashlight_max_spares;
extern ConVar sv_surv_flashlight_start_spares;
extern ConVar sv_surv_flashlight_swap_time;
extern ConVar sv_surv_flashlight_needsuit;
extern ConVar sv_surv_flashlight_dim_start;
extern ConVar sv_surv_flashlight_flicker_start;

inline bool SurvivalFlashlight_IsEnabled() { return sv_surv_flashlight.GetBool(); }

// Visual output for a given charge (0-100). Shared so server-side logic
// (e.g. NPC illumination) can agree with what the player sees.
struct SurvivalFlashlightLook_t
{
	float flBrightness;		// 0..1 multiplier on light output
	float flFOVLoss;		// degrees to narrow the cone by
	Vector vecTint;			// colour multiplier (warms as it dies)
};

void SurvivalFlashlight_GetLook( float flCharge, float flTime, SurvivalFlashlightLook_t &look );

#endif // MAPBASE

#endif // SURVIVAL_FLASHLIGHT_SHARED_H
