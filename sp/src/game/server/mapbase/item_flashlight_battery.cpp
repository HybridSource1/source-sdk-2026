//=============================================================================
//
// Purpose:	item_flashlight_battery - a spare battery for the survival
//			flashlight. Picked up into the player's spare count; the player
//			swaps it in when the light dies (press flashlight) or manually
//			with the "flashlight_swap" command.
//
//			Keyvalues:
//				Batteries	- how many spares this pickup gives (default 1)
//				TouchSound	- sound played on pickup
//				model		- custom model (default models/items/battery.mdl)
//
//=============================================================================

#include "cbase.h"
#include "hl2_player.h"
#include "items.h"
#include "mapbase/survival_flashlight_shared.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

#define FLASHLIGHT_BATTERY_MODEL "models/items/battery.mdl"

class CItemFlashlightBattery : public CItem
{
public:
	DECLARE_CLASS( CItemFlashlightBattery, CItem );
	DECLARE_DATADESC();

	CItemFlashlightBattery();

	void	Spawn( void );
	void	Precache( void );
	bool	MyTouch( CBasePlayer *pPlayer );

	void	InputSetBatteries( inputdata_t &inputdata ) { m_nBatteries = MAX( inputdata.value.Int(), 1 ); }

private:
	int			m_nBatteries;
	string_t	m_iszTouchSound;
	COutputEvent m_OnPlayerFull;
	float		m_flNextFullOutput;
};

LINK_ENTITY_TO_CLASS( item_flashlight_battery, CItemFlashlightBattery );
PRECACHE_REGISTER( item_flashlight_battery );

BEGIN_DATADESC( CItemFlashlightBattery )
	DEFINE_KEYFIELD( m_nBatteries, FIELD_INTEGER, "Batteries" ),
	DEFINE_KEYFIELD( m_iszTouchSound, FIELD_STRING, "TouchSound" ),
	DEFINE_FIELD( m_flNextFullOutput, FIELD_TIME ),
	DEFINE_INPUTFUNC( FIELD_INTEGER, "SetBatteries", InputSetBatteries ),
	DEFINE_OUTPUT( m_OnPlayerFull, "OnPlayerFull" ),
END_DATADESC()

//-----------------------------------------------------------------------------
CItemFlashlightBattery::CItemFlashlightBattery()
{
	m_nBatteries = 1;
	m_iszTouchSound = NULL_STRING;
	m_flNextFullOutput = 0.0f;
}

//-----------------------------------------------------------------------------
void CItemFlashlightBattery::Spawn( void )
{
	Precache();
	SetModel( DefaultOrCustomModel( FLASHLIGHT_BATTERY_MODEL ) );
	BaseClass::Spawn();
}

//-----------------------------------------------------------------------------
void CItemFlashlightBattery::Precache( void )
{
	PrecacheModel( DefaultOrCustomModel( FLASHLIGHT_BATTERY_MODEL ) );
	PrecacheScriptSound( "SurvivalFlashlight.Pickup" );

	if ( m_iszTouchSound != NULL_STRING )
		PrecacheScriptSound( STRING( m_iszTouchSound ) );
}

//-----------------------------------------------------------------------------
bool CItemFlashlightBattery::MyTouch( CBasePlayer *pPlayer )
{
	CHL2_Player *pHL2Player = dynamic_cast<CHL2_Player*>( pPlayer );
	if ( !pHL2Player )
		return false;

	if ( !pHL2Player->Flashlight_AddBatteries( MAX( m_nBatteries, 1 ) ) )
	{
		// Pockets full: leave it in the world. Throttle the output so
		// standing on it doesn't spam.
		if ( gpGlobals->curtime >= m_flNextFullOutput )
		{
			m_OnPlayerFull.FireOutput( pPlayer, this );
			m_flNextFullOutput = gpGlobals->curtime + 1.0f;
		}
		return false;
	}

	CSingleUserRecipientFilter filter( pPlayer );
	EmitSound( filter, pPlayer->entindex(),
		m_iszTouchSound != NULL_STRING ? STRING( m_iszTouchSound ) : "SurvivalFlashlight.Pickup" );

	return true;
}
