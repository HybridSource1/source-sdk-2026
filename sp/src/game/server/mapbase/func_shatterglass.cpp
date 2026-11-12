//========= Mapbase - https://github.com/mapbase-source/source-sdk-2013 ============//
//
// Purpose: func_shatterglass, a Source 2-style breakable glass pane.
//
//			Unlike func_breakable_surf, which breaks along a fixed 12-unit grid,
//			the pane is split into a procedural Voronoi shard pattern. Impacts
//			knock out the shards around the hit point and crack the shards around
//			that. Shards stay in place as long as they are connected to an anchored
//			edge of the frame; anything that loses its connection falls out, and
//			cracked shards that are only hanging on by one neighbor drop over time.
//
//			Bullets, players and objects pass through the holes, since the entity
//			uses custom ray/box tests against the shards that are still intact.
//
// Author(s): HybridSource1
//
//=============================================================================//

#include "cbase.h"
#include "mapbase/shatterglass_shared.h"
#include "physics_impact_damage.h"
#include "ndebugoverlay.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

#define SF_SHATTERGLASS_NO_PHYSICS_DAMAGE	(1<<0)
#define SF_SHATTERGLASS_NO_BULLET_DAMAGE	(1<<1)
#define SF_SHATTERGLASS_NO_LOOSE_SHARDS		(1<<2)

#define SHATTERGLASS_THINK_INTERVAL			0.1f

ConVar sv_shatterglass_debug( "sv_shatterglass_debug", "0", FCVAR_CHEAT, "Draws func_shatterglass shard states." );

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CShatterGlass : public CBaseEntity
{
public:
	DECLARE_CLASS( CShatterGlass, CBaseEntity );
	DECLARE_SERVERCLASS();
	DECLARE_DATADESC();

	CShatterGlass();

	virtual void	Spawn( void );
	virtual void	Precache( void );
	virtual bool	CreateVPhysics( void );
	virtual void	OnRestore( void );
	virtual bool	KeyValue( const char *szKeyName, const char *szValue );
	virtual int		ObjectCaps( void ) { return BaseClass::ObjectCaps() & ~FCAP_ACROSS_TRANSITION; }

	virtual bool	TestCollision( const Ray_t &ray, unsigned int mask, trace_t &trace );

	virtual void	TraceAttack( const CTakeDamageInfo &info, const Vector &vecDir, trace_t *ptr, CDmgAccumulator *pAccumulator );
	virtual int		OnTakeDamage( const CTakeDamageInfo &info );
	virtual void	Event_Killed( const CTakeDamageInfo &info ) {}
	virtual void	VPhysicsCollision( int index, gamevcollisionevent_t *pEvent );

	virtual int		DrawDebugTextOverlays( void );

	void			GlassThink( void );

	// Inputs
	void			InputBreak( inputdata_t &inputdata );
	void			InputShatter( inputdata_t &inputdata );
	void			InputBreakAt( inputdata_t &inputdata );

private:
	void			BuildPattern( void );

	Vector2D		WorldToPanel( const Vector &vecWorld, float *pflDistAlongNormal = NULL ) const;
	Vector			PanelToWorld( const Vector2D &uv ) const;
	Vector			WorldNormal( void ) const;
	float			ShardSize( void ) const { return m_Pattern.CellSize(); }

	bool			IsBroken( int i ) const { return ShatterGlass_IsBitSet( m_nBrokenBits.Base(), i ); }
	bool			IsCracked( int i ) const { return ShatterGlass_IsBitSet( m_nCrackedBits.Base(), i ); }
	void			BreakShard( int i );
	void			CrackShard( int i );

	// Knocks out shards within flBreakRadius of the point and cracks shards within flCrackRadius
	void			Fracture( const Vector2D &uv, float flBreakRadius, float flCrackRadius, const Vector &vecForce, CBaseEntity *pActivator, bool bLoudBreak );

	// Knocks out every intact shard overlapping a rectangle on the pane
	int				BreakRect( const Vector2D &vecMins, const Vector2D &vecMaxs );

	// Cracks intact shards around a point without knocking anything out
	void			CrackAround( const Vector2D &uv, float flRadius );

	// Drops shards which are no longer connected to an anchored edge
	int				UpdateSupport( void );
	void			BreakAll( const Vector &vecForce, CBaseEntity *pActivator );
	void			PostBreak( int nBroken, CBaseEntity *pActivator, const Vector2D &uv, bool bLoud );
	void			SetImpact( const Vector2D &uv, const Vector &vecForce, float flRadius );
	void			PlayBreakSound( const Vector &vecPos );

	void			CheckObjectsPassingThrough( void );

	ShatterGlassPanel_t		m_Panel;
	CShatterGlassPattern	m_Pattern;

	// Keyvalues
	float			m_flKVShardSize;
	float			m_flFragility;
	float			m_flCollapseFraction;
	float			m_flLooseShardRate;
	int				m_nAnchorBits;
	string_t		m_iszGlassSurfaceProp;

	int				m_nBrokenCount;
	float			m_flBrokenArea;
	bool			m_bBroken;
	bool			m_bFullyBroken;
	float			m_flNextBreakSound;

	int				m_nContents;
	unsigned short	m_nSurfaceProps;

	COutputEvent	m_OnBroken;
	COutputEvent	m_OnFullyBroken;

	CNetworkVar( int, m_nSeed );
	CNetworkVar( float, m_flShardSize );
	CNetworkVector( m_vecPanelMins );
	CNetworkVector( m_vecPanelMaxs );
	CNetworkArray( int, m_nBrokenBits, SHATTERGLASS_STATE_WORDS );
	CNetworkArray( int, m_nCrackedBits, SHATTERGLASS_STATE_WORDS );

	// Most recent impact, used by the client to launch the shards it knocked out
	CNetworkVar( int, m_nImpactSerial );
	CNetworkVector( m_vecImpactPos );	// Pane coordinates (u, v, 0)
	CNetworkVector( m_vecImpactForce );	// World space velocity
	CNetworkVar( float, m_flImpactRadius );
};

LINK_ENTITY_TO_CLASS( func_shatterglass, CShatterGlass );

BEGIN_DATADESC( CShatterGlass )

	DEFINE_KEYFIELD( m_flKVShardSize, FIELD_FLOAT, "shardsize" ),
	DEFINE_KEYFIELD( m_nSeed, FIELD_INTEGER, "seed" ),
	DEFINE_KEYFIELD( m_flFragility, FIELD_FLOAT, "fragility" ),
	DEFINE_KEYFIELD( m_flCollapseFraction, FIELD_FLOAT, "collapsefraction" ),
	DEFINE_KEYFIELD( m_flLooseShardRate, FIELD_FLOAT, "looseshardrate" ),
	DEFINE_KEYFIELD( m_iszGlassSurfaceProp, FIELD_STRING, "glasssurfaceprop" ),
	DEFINE_FIELD( m_nAnchorBits, FIELD_INTEGER ),

	DEFINE_FIELD( m_flShardSize, FIELD_FLOAT ),
	DEFINE_FIELD( m_vecPanelMins, FIELD_VECTOR ),
	DEFINE_FIELD( m_vecPanelMaxs, FIELD_VECTOR ),
	DEFINE_ARRAY( m_nBrokenBits, FIELD_INTEGER, SHATTERGLASS_STATE_WORDS ),
	DEFINE_ARRAY( m_nCrackedBits, FIELD_INTEGER, SHATTERGLASS_STATE_WORDS ),
	DEFINE_FIELD( m_nBrokenCount, FIELD_INTEGER ),
	DEFINE_FIELD( m_flBrokenArea, FIELD_FLOAT ),
	DEFINE_FIELD( m_bBroken, FIELD_BOOLEAN ),
	DEFINE_FIELD( m_bFullyBroken, FIELD_BOOLEAN ),
	DEFINE_FIELD( m_nImpactSerial, FIELD_INTEGER ),
	DEFINE_FIELD( m_vecImpactPos, FIELD_VECTOR ),
	DEFINE_FIELD( m_vecImpactForce, FIELD_VECTOR ),
	DEFINE_FIELD( m_flImpactRadius, FIELD_FLOAT ),

	DEFINE_INPUTFUNC( FIELD_VOID, "Break", InputBreak ),
	DEFINE_INPUTFUNC( FIELD_VECTOR, "Shatter", InputShatter ),
	DEFINE_INPUTFUNC( FIELD_VECTOR, "BreakAt", InputBreakAt ),

	DEFINE_OUTPUT( m_OnBroken, "OnBroken" ),
	DEFINE_OUTPUT( m_OnFullyBroken, "OnFullyBroken" ),

	DEFINE_THINKFUNC( GlassThink ),

END_DATADESC()

IMPLEMENT_SERVERCLASS_ST( CShatterGlass, DT_ShatterGlass )
	SendPropInt( SENDINFO( m_nSeed ), 32, 0 ),
	SendPropFloat( SENDINFO( m_flShardSize ), 0, SPROP_NOSCALE ),
	SendPropVector( SENDINFO( m_vecPanelMins ), 0, SPROP_NOSCALE ),
	SendPropVector( SENDINFO( m_vecPanelMaxs ), 0, SPROP_NOSCALE ),
	SendPropArray3( SENDINFO_ARRAY3( m_nBrokenBits ), SendPropInt( SENDINFO_ARRAY( m_nBrokenBits ), 32, SPROP_UNSIGNED ) ),
	SendPropArray3( SENDINFO_ARRAY3( m_nCrackedBits ), SendPropInt( SENDINFO_ARRAY( m_nCrackedBits ), 32, SPROP_UNSIGNED ) ),
	SendPropInt( SENDINFO( m_nImpactSerial ), 8, SPROP_UNSIGNED ),
	SendPropVector( SENDINFO( m_vecImpactPos ), 0, SPROP_NOSCALE ),
	SendPropVector( SENDINFO( m_vecImpactForce ), 0, SPROP_NOSCALE ),
	SendPropFloat( SENDINFO( m_flImpactRadius ), 0, SPROP_NOSCALE ),
END_SEND_TABLE()

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
CShatterGlass::CShatterGlass()
{
	m_flKVShardSize = SHATTERGLASS_DEFAULT_SHARD_SIZE;
	m_flFragility = 1.0f;
	m_flCollapseFraction = 0.6f;
	m_flLooseShardRate = 1.0f;
	m_nAnchorBits = SHATTERGLASS_ANCHOR_ALL;
	m_iszGlassSurfaceProp = NULL_STRING;
	m_nSeed = 0;
	m_nContents = CONTENTS_SOLID;
	m_nSurfaceProps = 0;
	m_flNextBreakSound = 0.0f;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool CShatterGlass::KeyValue( const char *szKeyName, const char *szValue )
{
	struct AnchorKey_t { const char *pszKey; int nBit; };
	static const AnchorKey_t s_AnchorKeys[] =
	{
		{ "anchor_left",	SHATTERGLASS_ANCHOR_LEFT },
		{ "anchor_right",	SHATTERGLASS_ANCHOR_RIGHT },
		{ "anchor_bottom",	SHATTERGLASS_ANCHOR_BOTTOM },
		{ "anchor_top",		SHATTERGLASS_ANCHOR_TOP },
	};

	for ( int i = 0; i < ARRAYSIZE( s_AnchorKeys ); i++ )
	{
		if ( FStrEq( szKeyName, s_AnchorKeys[i].pszKey ) )
		{
			if ( atoi( szValue ) != 0 )
				m_nAnchorBits |= s_AnchorKeys[i].nBit;
			else
				m_nAnchorBits &= ~s_AnchorKeys[i].nBit;
			return true;
		}
	}

	return BaseClass::KeyValue( szKeyName, szValue );
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CShatterGlass::Precache( void )
{
	BaseClass::Precache();

	if ( m_iszGlassSurfaceProp == NULL_STRING )
		m_iszGlassSurfaceProp = AllocPooledString( "glass" );

	int nSurface = physprops->GetSurfaceIndex( STRING( m_iszGlassSurfaceProp ) );
	if ( nSurface < 0 )
		nSurface = physprops->GetSurfaceIndex( "glass" );
	m_nSurfaceProps = (unsigned short)MAX( nSurface, 0 );

	const surfacedata_t *pSurface = physprops->GetSurfaceData( m_nSurfaceProps );
	if ( pSurface )
	{
		const char *pszSounds[] =
		{
			physprops->GetString( pSurface->sounds.breakSound ),
			physprops->GetString( pSurface->sounds.impactHard ),
			physprops->GetString( pSurface->sounds.impactSoft ),
			physprops->GetString( pSurface->sounds.bulletImpact ),
		};

		for ( int i = 0; i < ARRAYSIZE( pszSounds ); i++ )
		{
			if ( pszSounds[i] && pszSounds[i][0] )
				PrecacheScriptSound( pszSounds[i] );
		}
	}
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CShatterGlass::Spawn( void )
{
	Precache();

	SetMoveType( MOVETYPE_PUSH );
	SetModel( STRING( GetModelName() ) );
	SetSolid( SOLID_BSP );

	// We do our own tracing against the shards which are still in place
	AddSolidFlags( FSOLID_CUSTOMRAYTEST | FSOLID_CUSTOMBOXTEST );

	m_takedamage = DAMAGE_YES;

	m_flFragility = MAX( m_flFragility, 0.05f );
	m_flCollapseFraction = clamp( m_flCollapseFraction, 0.0f, 1.0f );
	m_flLooseShardRate = MAX( m_flLooseShardRate, 0.0f );

	m_vecPanelMins = CollisionProp()->OBBMins();
	m_vecPanelMaxs = CollisionProp()->OBBMaxs();
	m_flShardSize = MAX( m_flKVShardSize, SHATTERGLASS_MIN_SHARD_SIZE );

	if ( m_nSeed == 0 )
		m_nSeed = RandomInt( 1, 0x3FFFFFFF );

	for ( int i = 0; i < SHATTERGLASS_STATE_WORDS; i++ )
	{
		m_nBrokenBits.Set( i, 0 );
		m_nCrackedBits.Set( i, 0 );
	}

	m_nBrokenCount = 0;
	m_flBrokenArea = 0.0f;
	m_bBroken = false;
	m_bFullyBroken = false;
	m_nImpactSerial = 0;

	BuildPattern();
	CreateVPhysics();
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool CShatterGlass::CreateVPhysics( void )
{
	// Physics objects only collide with an intact pane; once it has holes in it,
	// objects passing through are handled in GlassThink() instead
	if ( m_bBroken )
		return true;

	if ( GetParent() )
		VPhysicsInitShadow( false, false );
	else
		VPhysicsInitStatic();

	return true;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CShatterGlass::OnRestore( void )
{
	BaseClass::OnRestore();

	BuildPattern();

	if ( m_bBroken )
		VPhysicsDestroyObject();
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CShatterGlass::BuildPattern( void )
{
	m_Panel.Init( m_vecPanelMins, m_vecPanelMaxs );
	m_Pattern.Generate( m_Panel.m_flWidth, m_Panel.m_flHeight, m_flShardSize, m_nSeed );

	m_nContents = GetModel() ? modelinfo->GetModelContents( GetModelIndex() ) : CONTENTS_SOLID;
	if ( m_nContents == 0 )
		m_nContents = CONTENTS_SOLID;

	if ( m_Pattern.Count() > 0 && m_Panel.m_flWidth * m_Panel.m_flHeight > 0.0f )
	{
		float flAspect = m_Panel.m_flThickness / MIN( m_Panel.m_flWidth, m_Panel.m_flHeight );
		if ( flAspect > 0.5f )
		{
			Warning( "func_shatterglass '%s' at (%.0f %.0f %.0f) is not a thin pane; it should be much thinner than it is wide.\n",
				GetDebugName(), GetAbsOrigin().x, GetAbsOrigin().y, GetAbsOrigin().z );
		}
	}
}

//-----------------------------------------------------------------------------
// Coordinate helpers
//-----------------------------------------------------------------------------
Vector2D CShatterGlass::WorldToPanel( const Vector &vecWorld, float *pflDistAlongNormal ) const
{
	Vector vecLocal;
	VectorITransform( vecWorld, EntityToWorldTransform(), vecLocal );

	Vector2D uv;
	float flN = m_Panel.LocalToPanel( vecLocal, uv );
	if ( pflDistAlongNormal )
		*pflDistAlongNormal = flN - m_Panel.m_flThickness * 0.5f;
	return uv;
}

Vector CShatterGlass::PanelToWorld( const Vector2D &uv ) const
{
	Vector vecWorld;
	VectorTransform( m_Panel.LocalPoint( uv.x, uv.y, m_Panel.m_flThickness * 0.5f ), EntityToWorldTransform(), vecWorld );
	return vecWorld;
}

Vector CShatterGlass::WorldNormal( void ) const
{
	Vector vecLocal( 0, 0, 0 );
	vecLocal[m_Panel.m_iN] = 1.0f;

	Vector vecWorld;
	VectorRotate( vecLocal, EntityToWorldTransform(), vecWorld );
	return vecWorld;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool CShatterGlass::TestCollision( const Ray_t &ray, unsigned int mask, trace_t &trace )
{
	return ShatterGlass_TestCollision( ray, mask, trace, EntityToWorldTransform(), m_Panel, m_Pattern,
		m_nBrokenBits.Base(), m_nBrokenCount, m_nContents, m_nSurfaceProps );
}

//-----------------------------------------------------------------------------
// Shard state
//-----------------------------------------------------------------------------
void CShatterGlass::BreakShard( int i )
{
	if ( i < 0 || i >= m_Pattern.Count() || IsBroken( i ) )
		return;

	int nWord = i >> 5;
	unsigned int nBit = 1u << ( i & 31 );

	m_nBrokenBits.Set( nWord, (int)( ((unsigned int)m_nBrokenBits[nWord]) | nBit ) );
	if ( ((unsigned int)m_nCrackedBits[nWord]) & nBit )
		m_nCrackedBits.Set( nWord, (int)( ((unsigned int)m_nCrackedBits[nWord]) & ~nBit ) );

	m_nBrokenCount++;
	m_flBrokenArea += m_Pattern.Get( i ).area;
}

void CShatterGlass::CrackShard( int i )
{
	if ( i < 0 || i >= m_Pattern.Count() || IsBroken( i ) || IsCracked( i ) )
		return;

	int nWord = i >> 5;
	m_nCrackedBits.Set( nWord, (int)( ((unsigned int)m_nCrackedBits[nWord]) | ( 1u << ( i & 31 ) ) ) );
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CShatterGlass::SetImpact( const Vector2D &uv, const Vector &vecForce, float flRadius )
{
	m_vecImpactPos = Vector( uv.x, uv.y, 0.0f );
	m_vecImpactForce = vecForce;
	m_flImpactRadius = flRadius;
	m_nImpactSerial = ( m_nImpactSerial + 1 ) & 0xFF;
}

void CShatterGlass::PlayBreakSound( const Vector &vecPos )
{
	if ( gpGlobals->curtime < m_flNextBreakSound )
		return;

	const surfacedata_t *pSurface = physprops->GetSurfaceData( m_nSurfaceProps );
	if ( !pSurface )
		return;

	const char *pszSound = physprops->GetString( pSurface->sounds.breakSound );
	if ( !pszSound || !pszSound[0] )
		return;

	CPASAttenuationFilter filter( vecPos, pszSound );
	EmitSound( filter, entindex(), pszSound, &vecPos );
	m_flNextBreakSound = gpGlobals->curtime + 0.15f;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CShatterGlass::Fracture( const Vector2D &uv, float flBreakRadius, float flCrackRadius, const Vector &vecForce, CBaseEntity *pActivator, bool bLoudBreak )
{
	if ( m_bFullyBroken || !m_Pattern.IsValid() )
		return;

	flBreakRadius *= m_flFragility;
	flCrackRadius *= m_flFragility;

	SetImpact( uv, vecForce, flBreakRadius + ShardSize() );

	int nHit = -1;
	if ( uv.x >= -1.0f && uv.y >= -1.0f && uv.x <= m_Panel.m_flWidth + 1.0f && uv.y <= m_Panel.m_flHeight + 1.0f )
		nHit = m_Pattern.FindShardAt( uv );

	float flReach = MAX( flBreakRadius * 2.0f, flCrackRadius ) + ShardSize();
	Vector2D vecReach( flReach, flReach );

	int nCandidates[SHATTERGLASS_MAX_SHARDS];
	int nCount = m_Pattern.GatherShardsInRect( uv - vecReach, uv + vecReach, nCandidates, SHATTERGLASS_MAX_SHARDS );

	int nBroken = 0;
	for ( int c = 0; c < nCount; c++ )
	{
		int i = nCandidates[c];
		if ( IsBroken( i ) )
			continue;

		const CShatterGlassPattern::Shard_t &shard = m_Pattern.Get( i );

		// Measure to the shard's rough edge rather than its center so large shards near the impact go too
		float flDist = MAX( 0.0f, shard.centroid.DistTo( uv ) - 0.35f * sqrtf( shard.area ) );
		flDist *= RandomFloat( 0.75f, 1.25f );

		bool bBreak = ( i == nHit ) || ( flDist < flBreakRadius ) || ( IsCracked( i ) && flDist < flBreakRadius * 2.0f );
		if ( bBreak )
		{
			BreakShard( i );
			nBroken++;
		}
		else if ( flDist < flCrackRadius )
		{
			CrackShard( i );
		}
	}

	PostBreak( nBroken, pActivator, uv, bLoudBreak );
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
int CShatterGlass::BreakRect( const Vector2D &vecMins, const Vector2D &vecMaxs )
{
	Vector2D vecCenter = ( vecMins + vecMaxs ) * 0.5f;
	Vector2D vecHalfExtents = ( vecMaxs - vecMins ) * 0.5f;

	int nCandidates[SHATTERGLASS_MAX_SHARDS];
	int nCount = m_Pattern.GatherShardsInRect( vecMins, vecMaxs, nCandidates, SHATTERGLASS_MAX_SHARDS );

	int nBroken = 0;
	for ( int c = 0; c < nCount; c++ )
	{
		int i = nCandidates[c];
		if ( IsBroken( i ) )
			continue;

		if ( CShatterGlassPattern::ShardOverlapsRect( m_Pattern.Get( i ), vecCenter, vecHalfExtents ) )
		{
			BreakShard( i );
			nBroken++;
		}
	}

	return nBroken;
}

//-----------------------------------------------------------------------------
// Purpose: 
//-----------------------------------------------------------------------------
void CShatterGlass::CrackAround( const Vector2D &uv, float flRadius )
{
	int nCandidates[SHATTERGLASS_MAX_SHARDS];
	int nCount = m_Pattern.GatherShardsInRect( uv - Vector2D( flRadius, flRadius ), uv + Vector2D( flRadius, flRadius ), nCandidates, SHATTERGLASS_MAX_SHARDS );
	for ( int c = 0; c < nCount; c++ )
	{
		if ( m_Pattern.Get( nCandidates[c] ).centroid.DistTo( uv ) < flRadius )
			CrackShard( nCandidates[c] );
	}
}

//-----------------------------------------------------------------------------
// Purpose: Flood fill from the anchored edges; anything unreachable falls out
//-----------------------------------------------------------------------------
int CShatterGlass::UpdateSupport( void )
{
	int nShards = m_Pattern.Count();
	if ( nShards <= 0 )
		return 0;

	unsigned char bVisited[SHATTERGLASS_MAX_SHARDS];
	short nQueue[SHATTERGLASS_MAX_SHARDS];
	int nHead = 0, nTail = 0;

	V_memset( bVisited, 0, sizeof( bVisited ) );

	for ( int i = 0; i < nShards; i++ )
	{
		if ( IsBroken( i ) )
			continue;

		if ( m_Pattern.Get( i ).nBorderAnchorBits & m_nAnchorBits )
		{
			bVisited[i] = 1;
			nQueue[nTail++] = (short)i;
		}
	}

	while ( nHead < nTail )
	{
		const CShatterGlassPattern::Shard_t &shard = m_Pattern.Get( nQueue[nHead++] );
		for ( int n = 0; n < shard.nNeighbors; n++ )
		{
			int nOther = shard.neighbors[n];
			if ( bVisited[nOther] || IsBroken( nOther ) )
				continue;

			bVisited[nOther] = 1;
			nQueue[nTail++] = (short)nOther;
		}
	}

	int nBroken = 0;
	for ( int i = 0; i < nShards; i++ )
	{
		if ( !bVisited[i] && !IsBroken( i ) )
		{
			BreakShard( i );
			nBroken++;
		}
	}

	return nBroken;
}

//-----------------------------------------------------------------------------
// Purpose: Common handling after shards were knocked out
//-----------------------------------------------------------------------------
void CShatterGlass::PostBreak( int nBroken, CBaseEntity *pActivator, const Vector2D &uv, bool bLoud )
{
	if ( nBroken <= 0 )
		return;

	if ( !m_bBroken )
	{
		m_bBroken = true;

		// From now on, physics objects pass through the holes and are handled in GlassThink()
		VPhysicsDestroyObject();

		m_OnBroken.FireOutput( pActivator ? pActivator : this, this );

		SetThink( &CShatterGlass::GlassThink );
		SetNextThink( gpGlobals->curtime + SHATTERGLASS_THINK_INTERVAL );
	}

	nBroken += UpdateSupport();

	if ( m_nBrokenCount < m_Pattern.Count() && m_flCollapseFraction < 1.0f && m_flBrokenArea >= m_flCollapseFraction * m_Pattern.TotalArea() )
	{
		// Too little glass left to hold together
		BreakAll( m_vecImpactForce, pActivator );
		return;
	}

	if ( bLoud || nBroken >= 4 )
		PlayBreakSound( PanelToWorld( uv ) );

	if ( m_nBrokenCount >= m_Pattern.Count() && !m_bFullyBroken )
	{
		m_bFullyBroken = true;
		m_takedamage = DAMAGE_NO;
		AddSolidFlags( FSOLID_NOT_SOLID );
		SetThink( NULL );

		m_OnFullyBroken.FireOutput( pActivator ? pActivator : this, this );
	}
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CShatterGlass::BreakAll( const Vector &vecForce, CBaseEntity *pActivator )
{
	if ( m_bFullyBroken )
		return;

	Vector2D vecCenter( m_Panel.m_flWidth * 0.5f, m_Panel.m_flHeight * 0.5f );
	if ( m_vecImpactPos.Get().x > 0.0f || m_vecImpactPos.Get().y > 0.0f )
		vecCenter.Init( m_vecImpactPos.Get().x, m_vecImpactPos.Get().y );

	SetImpact( vecCenter, vecForce, MAX( m_Panel.m_flWidth, m_Panel.m_flHeight ) );

	int nBroken = 0;
	for ( int i = 0; i < m_Pattern.Count(); i++ )
	{
		if ( !IsBroken( i ) )
		{
			BreakShard( i );
			nBroken++;
		}
	}

	// Make sure the pattern is considered fully broken even if it contained degenerate shards
	m_nBrokenCount = m_Pattern.Count();

	m_flNextBreakSound = 0.0f;
	PostBreak( MAX( nBroken, 1 ), pActivator, vecCenter, true );
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void CShatterGlass::TraceAttack( const CTakeDamageInfo &info, const Vector &vecDir, trace_t *ptr, CDmgAccumulator *pAccumulator )
{
	if ( m_takedamage == DAMAGE_NO || !PassesDamageFilter( info ) )
		return;

	int nType = info.GetDamageType();
	if ( !( nType & ( DMG_BULLET | DMG_BUCKSHOT | DMG_CLUB | DMG_SLASH ) ) )
	{
		// Let OnTakeDamage() deal with it
		BaseClass::TraceAttack( info, vecDir, ptr, pAccumulator );
		return;
	}

	if ( ( nType & ( DMG_BULLET | DMG_BUCKSHOT ) ) && HasSpawnFlags( SF_SHATTERGLASS_NO_BULLET_DAMAGE ) )
		return;

	Vector2D uv = WorldToPanel( ptr->endpos );
	float s = ShardSize();

	if ( nType & ( DMG_CLUB | DMG_SLASH ) )
	{
		// Melee knocks out a larger hole
		Fracture( uv, s * 1.25f + 2.0f, s * 4.0f, vecDir * 200.0f, info.GetAttacker(), true );
	}
	else
	{
		// Bullets punch a small hole and crack the glass around it
		float flScale = clamp( info.GetDamage() / 10.0f, 0.5f, 2.0f );
		Fracture( uv, s * 0.5f * flScale, s * 3.0f * sqrtf( flScale ), vecDir * 150.0f, info.GetAttacker(), false );
	}
}

//-----------------------------------------------------------------------------
// Purpose: Damage that didn't come in through TraceAttack()
//-----------------------------------------------------------------------------
int CShatterGlass::OnTakeDamage( const CTakeDamageInfo &info )
{
	if ( m_takedamage == DAMAGE_NO || !PassesDamageFilter( info ) )
		return 0;

	int nType = info.GetDamageType();
	float s = ShardSize();

	Vector vecPos = info.GetDamagePosition();
	if ( vecPos == vec3_origin )
	{
		CBaseEntity *pInflictor = info.GetInflictor();
		vecPos = pInflictor ? pInflictor->WorldSpaceCenter() : WorldSpaceCenter();
	}

	float flDistToPlane;
	Vector2D uv = WorldToPanel( vecPos, &flDistToPlane );

	// Closest point on the pane
	Vector2D uvClamped( clamp( uv.x, 0.0f, m_Panel.m_flWidth ), clamp( uv.y, 0.0f, m_Panel.m_flHeight ) );

	Vector vecNormal = WorldNormal();
	Vector vecAway = vecNormal * ( flDistToPlane > 0.0f ? -1.0f : 1.0f );

	if ( nType & ( DMG_BLAST | DMG_SONIC ) )
	{
		float flDist = ( PanelToWorld( uvClamped ) - vecPos ).Length();
		float flRadius = info.GetDamage() * 2.0f - flDist;
		if ( flRadius <= 0.0f )
			return 0;

		Vector vecForce = vecAway * ( 250.0f + info.GetDamage() * 2.0f );
		Fracture( uvClamped, flRadius, flRadius * 1.5f + s, vecForce, info.GetAttacker(), true );
		return 0;
	}

	if ( nType & DMG_CRUSH )
	{
		if ( HasSpawnFlags( SF_SHATTERGLASS_NO_PHYSICS_DAMAGE ) )
			return 0;

		float flRadius = clamp( info.GetDamage() * 0.4f, s * 0.75f, s * 6.0f );

		Vector vecForce = info.GetDamageForce();
		if ( vecForce.LengthSqr() < 1.0f )
			vecForce = vecAway;
		VectorNormalize( vecForce );

		Fracture( uvClamped, flRadius, flRadius * 2.5f, vecForce * 250.0f, info.GetAttacker(), true );
		return 0;
	}

	if ( ( nType & ( DMG_BULLET | DMG_BUCKSHOT ) ) && HasSpawnFlags( SF_SHATTERGLASS_NO_BULLET_DAMAGE ) )
		return 0;

	// Anything else
	float flRadius = s * 0.75f + info.GetDamage() * 0.1f;
	Fracture( uvClamped, flRadius, flRadius * 3.0f, vecAway * 150.0f, info.GetAttacker(), false );
	return 0;
}

//-----------------------------------------------------------------------------
// Purpose: Physics objects hitting the intact pane
//-----------------------------------------------------------------------------
void CShatterGlass::VPhysicsCollision( int index, gamevcollisionevent_t *pEvent )
{
	BaseClass::VPhysicsCollision( index, pEvent );

	if ( m_bBroken || HasSpawnFlags( SF_SHATTERGLASS_NO_PHYSICS_DAMAGE ) )
		return;

	int nDamageType = 0;
	float flDamage = CalculateDefaultPhysicsDamage( index, pEvent, 1.0f, false, nDamageType, AllocPooledString( "glass" ), true );
	if ( flDamage <= 0.0f )
		return;

	Vector vecNormal, vecDamagePos;
	pEvent->pInternalData->GetSurfaceNormal( vecNormal );
	if ( index == 0 )
		vecNormal *= -1.0f;
	pEvent->pInternalData->GetContactPoint( vecDamagePos );

	CBaseEntity *pInflictor = pEvent->pEntities[!index];

	if ( flDamage > 10.0f )
	{
		// HACKHACK: Reset mass to get correct collision response for the object breaking this
		pEvent->pObjects[index]->SetMass( 2.0f );

		CTakeDamageInfo info( pInflictor, pInflictor, vecNormal, vecDamagePos, flDamage, DMG_CRUSH );
		PhysCallbackDamage( this, info, *pEvent, index );
	}
	else
	{
		// Light knocks only crack the glass
		CrackAround( WorldToPanel( vecDamagePos ), ShardSize() * ( 1.0f + flDamage * 0.2f ) );
	}
}

//-----------------------------------------------------------------------------
// Purpose: Loose shards and objects passing through holes
//-----------------------------------------------------------------------------
void CShatterGlass::GlassThink( void )
{
	if ( m_bFullyBroken )
		return;

	SetNextThink( gpGlobals->curtime + SHATTERGLASS_THINK_INTERVAL );

	CheckObjectsPassingThrough();
	if ( m_bFullyBroken )
		return;

	if ( m_flLooseShardRate > 0.0f && !HasSpawnFlags( SF_SHATTERGLASS_NO_LOOSE_SHARDS ) )
	{
		// Cracked shards that are only hanging on by a single connection eventually fall out
		int nBroken = 0;
		float flChance = 0.12f * m_flLooseShardRate;

		for ( int i = 0; i < m_Pattern.Count(); i++ )
		{
			if ( IsBroken( i ) || !IsCracked( i ) )
				continue;

			const CShatterGlassPattern::Shard_t &shard = m_Pattern.Get( i );

			int nConnections = ( shard.nBorderAnchorBits & m_nAnchorBits ) ? 2 : 0;
			for ( int n = 0; n < shard.nNeighbors && nConnections < 2; n++ )
			{
				if ( !IsBroken( shard.neighbors[n] ) )
					nConnections++;
			}

			if ( nConnections <= 1 && RandomFloat() < flChance )
			{
				BreakShard( i );
				nBroken++;
			}
		}

		if ( nBroken > 0 )
		{
			// Falling shards just drop
			Vector2D uv( m_vecImpactPos.Get().x, m_vecImpactPos.Get().y );
			PostBreak( nBroken, this, uv, false );
		}
	}

	if ( sv_shatterglass_debug.GetBool() )
	{
		for ( int i = 0; i < m_Pattern.Count(); i++ )
		{
			if ( IsBroken( i ) )
				continue;

			const CShatterGlassPattern::Shard_t &shard = m_Pattern.Get( i );
			int r = IsCracked( i ) ? 255 : 0;
			int g = IsCracked( i ) ? 128 : 255;
			for ( int v = 0; v < shard.nVerts; v++ )
			{
				NDebugOverlay::Line( PanelToWorld( shard.verts[v] ), PanelToWorld( shard.verts[(v + 1) % shard.nVerts] ), r, g, 0, true, SHATTERGLASS_THINK_INTERVAL );
			}
		}
	}
}

//-----------------------------------------------------------------------------
// Purpose: Without a VPhysics object, objects fly straight through the pane.
//			Break whatever glass they pass through.
//-----------------------------------------------------------------------------
void CShatterGlass::CheckObjectsPassingThrough( void )
{
	if ( HasSpawnFlags( SF_SHATTERGLASS_NO_PHYSICS_DAMAGE ) )
		return;

	Vector vecMins, vecMaxs;
	CollisionProp()->WorldSpaceAABB( &vecMins, &vecMaxs );

	// Account for objects which will move a long way during one think
	Vector vecBloat( 64, 64, 64 );

	CBaseEntity *pList[32];
	int nCount = UTIL_EntitiesInBox( pList, ARRAYSIZE( pList ), vecMins - vecBloat, vecMaxs + vecBloat, 0 );

	for ( int e = 0; e < nCount; e++ )
	{
		CBaseEntity *pEnt = pList[e];
		if ( !pEnt || pEnt == this || pEnt->IsPlayer() || pEnt->MyNPCPointer() || pEnt->GetMoveType() != MOVETYPE_VPHYSICS )
			continue;

		IPhysicsObject *pPhys = pEnt->VPhysicsGetObject();
		if ( !pPhys || !pPhys->IsMoveable() || pPhys->IsAsleep() )
			continue;

		Vector vecVelocity;
		pPhys->GetVelocity( &vecVelocity, NULL );
		if ( vecVelocity.LengthSqr() < ( 100.0f * 100.0f ) )
			continue;

		// Sweep the object's bounds back over the last think
		Vector vecEntMins, vecEntMaxs;
		pEnt->CollisionProp()->WorldSpaceAABB( &vecEntMins, &vecEntMaxs );

		Vector vecBack = vecVelocity * -SHATTERGLASS_THINK_INTERVAL;
		Vector vecSweepMins = vecEntMins, vecSweepMaxs = vecEntMaxs;
		for ( int k = 0; k < 3; k++ )
		{
			vecSweepMins[k] = MIN( vecEntMins[k], vecEntMins[k] + vecBack[k] );
			vecSweepMaxs[k] = MAX( vecEntMaxs[k], vecEntMaxs[k] + vecBack[k] );
		}

		// Convert the swept box into pane space
		Vector2D uvMins( FLT_MAX, FLT_MAX ), uvMaxs( -FLT_MAX, -FLT_MAX );
		float flNMin = FLT_MAX, flNMax = -FLT_MAX;
		for ( int c = 0; c < 8; c++ )
		{
			Vector vecCorner( ( c & 1 ) ? vecSweepMaxs.x : vecSweepMins.x,
							  ( c & 2 ) ? vecSweepMaxs.y : vecSweepMins.y,
							  ( c & 4 ) ? vecSweepMaxs.z : vecSweepMins.z );
			float flN;
			Vector2D uv = WorldToPanel( vecCorner, &flN );
			uvMins.x = MIN( uvMins.x, uv.x ); uvMins.y = MIN( uvMins.y, uv.y );
			uvMaxs.x = MAX( uvMaxs.x, uv.x ); uvMaxs.y = MAX( uvMaxs.y, uv.y );
			flNMin = MIN( flNMin, flN );
			flNMax = MAX( flNMax, flN );
		}

		// Did it cross the plane?
		if ( flNMin > m_Panel.m_flThickness * 0.5f || flNMax < -m_Panel.m_flThickness * 0.5f )
			continue;

		// The swept box overestimates the footprint a lot for fast objects; use the box at the plane instead
		Vector2D uvCenter = ( uvMins + uvMaxs ) * 0.5f;
		Vector vecEntSize = ( vecEntMaxs - vecEntMins ) * 0.5f;
		float flHalf = MAX( vecEntSize.x, MAX( vecEntSize.y, vecEntSize.z ) );
		Vector2D uvHalf( MIN( flHalf, ( uvMaxs.x - uvMins.x ) * 0.5f ), MIN( flHalf, ( uvMaxs.y - uvMins.y ) * 0.5f ) );

		int nBroken = BreakRect( uvCenter - uvHalf, uvCenter + uvHalf );
		if ( nBroken > 0 )
		{
			SetImpact( uvCenter, vecVelocity * 0.5f, flHalf + ShardSize() );

			// Crack the glass around the hole too
			CrackAround( uvCenter, flHalf + ShardSize() * 2.0f );
			PostBreak( nBroken, pEnt, uvCenter, true );

			if ( m_bFullyBroken )
				return;
		}
	}
}

//-----------------------------------------------------------------------------
// Inputs
//-----------------------------------------------------------------------------
void CShatterGlass::InputBreak( inputdata_t &inputdata )
{
	if ( !m_bBroken )
		m_vecImpactPos = Vector( m_Panel.m_flWidth * 0.5f, m_Panel.m_flHeight * 0.5f, 0.0f );

	BreakAll( WorldNormal() * 50.0f, inputdata.pActivator );
}

// Compatible with func_breakable_surf: "x y radius", where x and y are 0-1 across the pane
void CShatterGlass::InputShatter( inputdata_t &inputdata )
{
	Vector vecInfo;
	inputdata.value.Vector3D( vecInfo );

	Vector2D uv( vecInfo.x * m_Panel.m_flWidth, vecInfo.y * m_Panel.m_flHeight );
	float flRadius = MAX( vecInfo.z, 1.0f ) / m_flFragility;

	Fracture( uv, flRadius, flRadius * 1.5f + ShardSize(), WorldNormal() * 150.0f, inputdata.pActivator, true );
}

// World position; acts like a bullet hitting the glass there
void CShatterGlass::InputBreakAt( inputdata_t &inputdata )
{
	Vector vecPos;
	inputdata.value.Vector3D( vecPos );

	float flDist;
	Vector2D uv = WorldToPanel( vecPos, &flDist );
	Vector vecForce = WorldNormal() * ( flDist > 0.0f ? -150.0f : 150.0f );

	Fracture( uv, ShardSize() * 0.5f, ShardSize() * 3.0f, vecForce, inputdata.pActivator, false );
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
int CShatterGlass::DrawDebugTextOverlays( void )
{
	int nOffset = BaseClass::DrawDebugTextOverlays();

	if ( m_debugOverlays & OVERLAY_TEXT_BIT )
	{
		char tempstr[512];
		Q_snprintf( tempstr, sizeof( tempstr ), "shards: %d broken / %d (%.0f%% area)", m_nBrokenCount, m_Pattern.Count(),
			m_Pattern.TotalArea() > 0.0f ? 100.0f * m_flBrokenArea / m_Pattern.TotalArea() : 0.0f );
		EntityText( nOffset++, tempstr, 0 );

		Q_snprintf( tempstr, sizeof( tempstr ), "pane: %.0f x %.0f x %.1f, shard size %.1f, seed %d", m_Panel.m_flWidth, m_Panel.m_flHeight,
			m_Panel.m_flThickness, ShardSize(), m_nSeed.Get() );
		EntityText( nOffset++, tempstr, 0 );
	}

	return nOffset;
}
