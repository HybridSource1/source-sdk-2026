//========= Copyright Valve Corporation, All rights reserved. ============//
//
// Purpose: 
//
//=============================================================================//

#include "cbase.h"
#include "hudelement.h"
#include "hud_numericdisplay.h"
#include <vgui_controls/Panel.h>
#include "hud.h"
#include "hud_suitpower.h"
#include "hud_macros.h"
#include "iclientmode.h"
#include <vgui_controls/AnimationController.h>
#include <vgui/ISurface.h>
#include "c_basehlplayer.h"
#ifdef MAPBASE
#include "mapbase/survival_flashlight_shared.h"
#endif

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-----------------------------------------------------------------------------
// Purpose: Shows the flashlight icon
//-----------------------------------------------------------------------------
class CHudFlashlight : public CHudElement, public vgui::Panel
{
	DECLARE_CLASS_SIMPLE( CHudFlashlight, vgui::Panel );

public:
	CHudFlashlight( const char *pElementName );
	virtual void ApplySchemeSettings( vgui::IScheme *pScheme );
#ifdef MAPBASE
	virtual bool ShouldDraw( void );
#endif

protected:
	virtual void Paint();

private:
	void SetFlashlightState( bool flashlightOn );
#ifdef MAPBASE
	void PaintSurvival( C_BaseHLPlayer *pPlayer );
	CPanelAnimationVar( vgui::HFont, m_hTextFont, "TextFont", "Default" );
	CPanelAnimationVarAliasType( float, m_flSpareX, "spare_xpos", "34", "proportional_float" );
	CPanelAnimationVarAliasType( float, m_flSpareY, "spare_ypos", "6", "proportional_float" );
#endif
	void Reset( void );
	
	bool	m_bFlashlightOn;
	CPanelAnimationVar( vgui::HFont, m_hFont, "Font", "WeaponIconsSmall" );
	CPanelAnimationVarAliasType( float, m_IconX, "icon_xpos", "4", "proportional_float" );
	CPanelAnimationVarAliasType( float, m_IconY, "icon_ypos", "4", "proportional_float" );
	
	CPanelAnimationVarAliasType( float, m_flBarInsetX, "BarInsetX", "2", "proportional_float" );
	CPanelAnimationVarAliasType( float, m_flBarInsetY, "BarInsetY", "18", "proportional_float" );

	CPanelAnimationVarAliasType( float, m_flBarWidth, "BarWidth", "28", "proportional_float" );
	CPanelAnimationVarAliasType( float, m_flBarHeight, "BarHeight", "2", "proportional_float" );
	CPanelAnimationVarAliasType( float, m_flBarChunkWidth, "BarChunkWidth", "2", "proportional_float" );
	CPanelAnimationVarAliasType( float, m_flBarChunkGap, "BarChunkGap", "2", "proportional_float" );
};	

using namespace vgui;

#define WCHAR_FLASHLIGHT_ON  169
#define WCHAR_FLASHLIGHT_OFF 174

#if defined( HL2_EPISODIC ) || defined( MAPBASE )
DECLARE_HUDELEMENT( CHudFlashlight );
#endif // HL2_EPISODIC

//-----------------------------------------------------------------------------
// Purpose: Constructor
//-----------------------------------------------------------------------------
CHudFlashlight::CHudFlashlight( const char *pElementName ) : CHudElement( pElementName ), BaseClass( NULL, "HudFlashlight" )
{
	vgui::Panel *pParent = g_pClientMode->GetViewport();
	SetParent( pParent );

#ifdef MAPBASE
	// Suit requirement is checked in ShouldDraw so the survival flashlight
	// can work without the HEV suit.
	SetHiddenBits( HIDEHUD_HEALTH | HIDEHUD_PLAYERDEAD );
#else
	SetHiddenBits( HIDEHUD_HEALTH | HIDEHUD_PLAYERDEAD | HIDEHUD_NEEDSUIT );
#endif
}

#ifdef MAPBASE
//-----------------------------------------------------------------------------
bool CHudFlashlight::ShouldDraw( void )
{
	C_BasePlayer *pPlayer = C_BasePlayer::GetLocalPlayer();
	if ( !pPlayer )
		return false;

	bool bNeedSuit = SurvivalFlashlight_IsEnabled() ? sv_surv_flashlight_needsuit.GetBool() : true;
	if ( bNeedSuit && !pPlayer->IsSuitEquipped() )
		return false;

#ifndef HL2_EPISODIC
	// Plain HL2 builds only have something to show in survival mode.
	if ( !SurvivalFlashlight_IsEnabled() )
		return false;
#endif

	return CHudElement::ShouldDraw();
}

//-----------------------------------------------------------------------------
// Purpose: Icon, charge bar, spare count. Bar blinks while swapping.
//-----------------------------------------------------------------------------
void CHudFlashlight::PaintSurvival( C_BaseHLPlayer *pPlayer )
{
	bool bIsOn = pPlayer->IsEffectActive( EF_DIMLIGHT );
	bool bSwapping = pPlayer->m_HL2Local.m_bSurvFlashSwapping;
	float flCharge = pPlayer->m_HL2Local.m_flSurvFlashCharge;
	int nSpares = pPlayer->m_HL2Local.m_iSurvFlashSpares;

	int chunkCount = m_flBarWidth / ( m_flBarChunkWidth + m_flBarChunkGap );
	int enabledChunks = bSwapping ? 0 : (int)ceilf( (float)chunkCount * ( flCharge / 100.0f ) );
	enabledChunks = clamp( enabledChunks, 0, chunkCount );

	bool bLow = flCharge < sv_surv_flashlight_dim_start.GetFloat();
	Color clr = ( bLow || bSwapping ) ? gHUD.m_clrCaution : gHUD.m_clrNormal;
	clr[3] = ( bIsOn || bSwapping ) ? 255 : 96;

	// Blink the whole element while the battery is out.
	if ( bSwapping && fmodf( gpGlobals->curtime, 0.5f ) < 0.25f )
		clr[3] = 48;

	surface()->DrawSetTextFont( m_hFont );
	surface()->DrawSetTextColor( clr );
	surface()->DrawSetTextPos( m_IconX, m_IconY );
	surface()->DrawUnicodeChar( bIsOn ? WCHAR_FLASHLIGHT_ON : WCHAR_FLASHLIGHT_OFF );

	// Charge bar (always shown - battery is a resource now)
	int xpos = m_flBarInsetX, ypos = m_flBarInsetY;
	surface()->DrawSetColor( clr );
	for ( int i = 0; i < enabledChunks; i++ )
	{
		surface()->DrawFilledRect( xpos, ypos, xpos + m_flBarChunkWidth, ypos + m_flBarHeight );
		xpos += ( m_flBarChunkWidth + m_flBarChunkGap );
	}

	Color clrEmpty = clr;
	clrEmpty[3] = clr[3] / 6;
	surface()->DrawSetColor( clrEmpty );
	for ( int i = enabledChunks; i < chunkCount; i++ )
	{
		surface()->DrawFilledRect( xpos, ypos, xpos + m_flBarChunkWidth, ypos + m_flBarHeight );
		xpos += ( m_flBarChunkWidth + m_flBarChunkGap );
	}

	// Spare batteries
	wchar_t wszSpares[16];
	V_snwprintf( wszSpares, ARRAYSIZE( wszSpares ), L"x%d", nSpares );

	Color clrText = nSpares > 0 ? gHUD.m_clrNormal : gHUD.m_clrCaution;
	clrText[3] = ( bIsOn || bSwapping || nSpares == 0 ) ? 255 : 128;

	surface()->DrawSetTextFont( m_hTextFont );
	surface()->DrawSetTextColor( clrText );
	surface()->DrawSetTextPos( m_flSpareX, m_flSpareY );
	surface()->DrawPrintText( wszSpares, wcslen( wszSpares ) );
}
#endif

//-----------------------------------------------------------------------------
// Purpose: 
// Input  : *pScheme - 
//-----------------------------------------------------------------------------
void CHudFlashlight::ApplySchemeSettings( vgui::IScheme *pScheme )
{
	BaseClass::ApplySchemeSettings(pScheme);
}

//-----------------------------------------------------------------------------
// Purpose: Start with our background off
//-----------------------------------------------------------------------------
void CHudFlashlight::Reset( void )
{
	g_pClientMode->GetViewportAnimationController()->StartAnimationSequence( "SuitFlashlightOn" ); 
}

//-----------------------------------------------------------------------------
// Purpose: data accessor
//-----------------------------------------------------------------------------
void CHudFlashlight::SetFlashlightState( bool flashlightOn )
{
	if ( m_bFlashlightOn == flashlightOn )
		return;

	m_bFlashlightOn = flashlightOn;
}


//-----------------------------------------------------------------------------
// Purpose: draws the flashlight icon
//-----------------------------------------------------------------------------
void CHudFlashlight::Paint()
{
#ifdef MAPBASE
	if ( SurvivalFlashlight_IsEnabled() )
	{
		C_BaseHLPlayer *pSurvPlayer = (C_BaseHLPlayer *)C_BasePlayer::GetLocalPlayer();
		if ( pSurvPlayer )
			PaintSurvival( pSurvPlayer );
		return;
	}
#endif

#ifdef HL2_EPISODIC
	C_BaseHLPlayer *pPlayer = (C_BaseHLPlayer *)C_BasePlayer::GetLocalPlayer();
	if ( !pPlayer )
		return;

	// Only paint if we're using the new flashlight code
	if ( pPlayer->m_HL2Local.m_flFlashBattery < 0.0f )
	{
		SetPaintBackgroundEnabled( false );
		return;
	}

	bool bIsOn = pPlayer->IsEffectActive( EF_DIMLIGHT );
	SetFlashlightState( bIsOn );

	// get bar chunks
	int chunkCount = m_flBarWidth / (m_flBarChunkWidth + m_flBarChunkGap);
	int enabledChunks = (int)((float)chunkCount * (pPlayer->m_HL2Local.m_flFlashBattery * 1.0f/100.0f) + 0.5f );

	Color clrFlashlight;
	clrFlashlight = ( enabledChunks < ( chunkCount / 4 ) ) ? gHUD.m_clrCaution : gHUD.m_clrNormal;
	clrFlashlight[3] = ( bIsOn ) ? 255: 32;

	// Pick the right character given our current state
	wchar_t pState = ( bIsOn ) ? WCHAR_FLASHLIGHT_ON : WCHAR_FLASHLIGHT_OFF;

	surface()->DrawSetTextFont( m_hFont );
	surface()->DrawSetTextColor( clrFlashlight );
	surface()->DrawSetTextPos( m_IconX, m_IconY );
	surface()->DrawUnicodeChar( pState );

	// Don't draw the progress bar is we're fully charged
	if ( bIsOn == false && chunkCount == enabledChunks )
		return;

	// draw the suit power bar
	surface()->DrawSetColor( clrFlashlight );
	int xpos = m_flBarInsetX, ypos = m_flBarInsetY;
	for (int i = 0; i < enabledChunks; i++)
	{
		surface()->DrawFilledRect( xpos, ypos, xpos + m_flBarChunkWidth, ypos + m_flBarHeight );
		xpos += (m_flBarChunkWidth + m_flBarChunkGap);
	}
	
	// Be even less transparent than we already are
	clrFlashlight[3] = clrFlashlight[3] / 8;

	// draw the exhausted portion of the bar.
	surface()->DrawSetColor( clrFlashlight );
	for (int i = enabledChunks; i < chunkCount; i++)
	{
		surface()->DrawFilledRect( xpos, ypos, xpos + m_flBarChunkWidth, ypos + m_flBarHeight );
		xpos += (m_flBarChunkWidth + m_flBarChunkGap);
	}
#endif // HL2_EPISODIC
}
