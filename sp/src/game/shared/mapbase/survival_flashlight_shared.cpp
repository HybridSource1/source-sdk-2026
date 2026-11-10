//=============================================================================
//
// Purpose:	Survival-horror flashlight: shared settings and bulb look.
//			See survival_flashlight_shared.h.
//
//=============================================================================

#include "cbase.h"
#include "mapbase/survival_flashlight_shared.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

ConVar sv_surv_flashlight( "sv_surv_flashlight", "1", FCVAR_REPLICATED, "Use finite flashlight batteries with spare-battery pickups instead of the HL2 recharging flashlight." );
ConVar sv_surv_flashlight_life( "sv_surv_flashlight_life", "300", FCVAR_REPLICATED, "Seconds a full battery lasts with the light on." );
ConVar sv_surv_flashlight_recharge( "sv_surv_flashlight_recharge", "0", FCVAR_REPLICATED, "Charge regained per second while the light is off (0 = batteries never recover)." );
ConVar sv_surv_flashlight_max_spares( "sv_surv_flashlight_max_spares", "4", FCVAR_REPLICATED, "Maximum spare batteries the player can carry." );
ConVar sv_surv_flashlight_start_spares( "sv_surv_flashlight_start_spares", "0", FCVAR_REPLICATED, "Spare batteries given on a fresh spawn." );
ConVar sv_surv_flashlight_swap_time( "sv_surv_flashlight_swap_time", "1.6", FCVAR_REPLICATED, "Seconds of darkness while swapping in a new battery." );
ConVar sv_surv_flashlight_needsuit( "sv_surv_flashlight_needsuit", "1", FCVAR_REPLICATED, "Require the HEV suit to use the flashlight (stock HL2 behaviour)." );
ConVar sv_surv_flashlight_dim_start( "sv_surv_flashlight_dim_start", "25", FCVAR_REPLICATED, "Charge (%) below which the bulb starts to dim and warm." );
ConVar sv_surv_flashlight_flicker_start( "sv_surv_flashlight_flicker_start", "10", FCVAR_REPLICATED, "Charge (%) below which the bulb starts to cut out." );

//-----------------------------------------------------------------------------
// Cheap deterministic hash -> [0,1)
//-----------------------------------------------------------------------------
static inline float HashToUnit( unsigned int n )
{
	n = ( n << 13 ) ^ n;
	n = n * ( n * n * 15731u + 789221u ) + 1376312589u;
	return (float)( n & 0x7fffffffu ) / 2147483648.0f;
}

//-----------------------------------------------------------------------------
void SurvivalFlashlight_GetLook( float flCharge, float flTime, SurvivalFlashlightLook_t &look )
{
	look.flBrightness = 1.0f;
	look.flFOVLoss = 0.0f;
	look.vecTint.Init( 1.0f, 1.0f, 1.0f );

	float flDimStart = MAX( sv_surv_flashlight_dim_start.GetFloat(), 0.01f );
	if ( flCharge >= flDimStart )
		return;

	// 0 = dead, 1 = just started dimming
	float t = clamp( flCharge / flDimStart, 0.0f, 1.0f );
	float flCurve = SimpleSpline( t );

	look.flBrightness = Lerp( flCurve, 0.25f, 1.0f );
	look.flFOVLoss = ( 1.0f - flCurve ) * 10.0f;

	// A dying incandescent bulb goes orange: blue drops first, then green.
	look.vecTint.y = Lerp( flCurve, 0.75f, 1.0f );
	look.vecTint.z = Lerp( flCurve, 0.45f, 1.0f );

	float flFlickerStart = sv_surv_flashlight_flicker_start.GetFloat();
	if ( flCharge >= flFlickerStart || flFlickerStart <= 0.0f )
		return;

	// Break time into short slots; each slot may drop out. The lower the
	// charge, the more often it cuts, and drops can chain into stutters.
	float f = clamp( flCharge / flFlickerStart, 0.0f, 1.0f );
	float flChance = Lerp( f, 0.45f, 0.04f );

	unsigned int nSlot = (unsigned int)( flTime * 14.0f );
	float r = HashToUnit( nSlot );
	if ( r < flChance )
	{
		look.flBrightness *= 0.06f;
	}
	else
	{
		// Unstable output between dropouts
		look.flBrightness *= 0.82f + 0.18f * HashToUnit( nSlot * 7919u + 1u );
	}
}
