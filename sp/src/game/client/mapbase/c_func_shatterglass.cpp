//========= Mapbase - https://github.com/mapbase-source/source-sdk-2013 ============//
//
// Purpose: Client side of func_shatterglass, a Source 2-style breakable glass pane.
//
//			Once damaged, the pane is drawn shard by shard through a brush surface
//			renderer, which keeps the brush's own material, texture alignment and
//			lightmap. Shards which get knocked out fall as clientside debris that
//			tumbles, bounces and breaks up further when it lands.
//
// Author(s): HybridSource1
//
//=============================================================================//

#include "cbase.h"
#include "mapbase/shatterglass_shared.h"
#include "materialsystem/imaterialsystem.h"
#include "materialsystem/imaterial.h"
#include "materialsystem/imesh.h"
#include "materialsystem/MaterialSystemUtil.h"
#include "ivrenderview.h"
#include "model_types.h"
#include "c_recipientfilter.h"
#include "physics_shared.h"
#include "KeyValues.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

ConVar r_shatterglass_debris( "r_shatterglass_debris", "1", FCVAR_ARCHIVE, "Show falling func_shatterglass shards." );
ConVar r_shatterglass_debris_max( "r_shatterglass_debris_max", "256", FCVAR_ARCHIVE, "Maximum number of falling shards per func_shatterglass." );
ConVar r_shatterglass_debris_lifetime( "r_shatterglass_debris_lifetime", "5", FCVAR_ARCHIVE, "How long fallen func_shatterglass shards stay around." );
ConVar r_shatterglass_cracks( "r_shatterglass_cracks", "1", FCVAR_ARCHIVE, "Draw cracks on func_shatterglass panes." );

static ConVarRef s_sv_gravity( "sv_gravity" );

#define SHATTERGLASS_MAX_DEBRIS_GENERATION	2

//-----------------------------------------------------------------------------
// A falling piece of glass
//-----------------------------------------------------------------------------
struct ShatterGlassDebris_t
{
	Vector2D	uv[SHATTERGLASS_MAX_VERTS];	// Where this piece used to be on the pane (texture/lightmap lookup)
	int			nVerts;
	Vector2D	center;
	float		flArea;

	Vector		pos;		// World position of the center
	Vector		vel;
	QAngle		angVel;
	matrix3x4_t	rot;		// Columns are the world directions of the pane's U, V and N axes for this piece

	float		flDieTime;
	int			nGeneration;
	int			nBounces;
	bool		bResting;
};

//-----------------------------------------------------------------------------
// A vertex waiting to be drawn with the brush material
//-----------------------------------------------------------------------------
struct ShatterGlassVert_t
{
	Vector		pos;
	Vector		normal;
	Vector2D	tex;
	Vector2D	lightmap;
};

//-----------------------------------------------------------------------------
// Affine mapping from pane (u, v) to a face's texture or lightmap coordinates
//-----------------------------------------------------------------------------
struct ShatterGlassAffine_t
{
	float s[3];	// s = s[0] * u + s[1] * v + s[2]
	float t[3];

	Vector2D Map( const Vector2D &uv ) const
	{
		return Vector2D( s[0] * uv.x + s[1] * uv.y + s[2], t[0] * uv.x + t[1] * uv.y + t[2] );
	}
};

static bool SolveAffine( const Vector2D *pUV, const Vector2D *pValues, ShatterGlassAffine_t &out )
{
	// Solve [u v 1] * coeffs = value for three points
	float flDet = pUV[0].x * ( pUV[1].y - pUV[2].y ) - pUV[0].y * ( pUV[1].x - pUV[2].x ) + ( pUV[1].x * pUV[2].y - pUV[2].x * pUV[1].y );
	if ( fabs( flDet ) < 1e-6f )
		return false;

	float flInv = 1.0f / flDet;
	for ( int c = 0; c < 2; c++ )
	{
		float v0 = pValues[0][c], v1 = pValues[1][c], v2 = pValues[2][c];
		float a = ( v0 * ( pUV[1].y - pUV[2].y ) - pUV[0].y * ( v1 - v2 ) + ( v1 * pUV[2].y - v2 * pUV[1].y ) ) * flInv;
		float b = ( pUV[0].x * ( v1 - v2 ) - v0 * ( pUV[1].x - pUV[2].x ) + ( pUV[1].x * v2 - pUV[2].x * v1 ) ) * flInv;
		float d = ( pUV[0].x * ( pUV[1].y * v2 - pUV[2].y * v1 ) - pUV[0].y * ( pUV[1].x * v2 - pUV[2].x * v1 ) + v0 * ( pUV[1].x * pUV[2].y - pUV[2].x * pUV[1].y ) ) * flInv;

		float *pDest = ( c == 0 ) ? out.s : out.t;
		pDest[0] = a;
		pDest[1] = b;
		pDest[2] = d;
	}

	return true;
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class C_ShatterGlass : public C_BaseEntity, public IBrushRenderer
{
public:
	DECLARE_CLASS( C_ShatterGlass, C_BaseEntity );
	DECLARE_CLIENTCLASS();

	C_ShatterGlass();

	virtual void	OnDataChanged( DataUpdateType_t updateType );
	virtual void	ClientThink( void );

	virtual bool	TestCollision( const Ray_t &ray, unsigned int fContentsMask, trace_t &tr );

	virtual bool	IsTransparent( void );
	virtual int		DrawModel( int flags );
	virtual void	GetRenderBounds( Vector &theMins, Vector &theMaxs );

	// IBrushRenderer
	virtual bool	RenderBrushModelSurface( IClientEntity *pBaseEntity, IBrushSurface *pBrushSurface );

private:
	void			BuildPattern( void );
	void			CountStates( void );
	bool			IsBroken( int i ) const { return ShatterGlass_IsBitSet( m_nBrokenBits, i ); }
	bool			IsCracked( int i ) const { return ShatterGlass_IsBitSet( m_nCrackedBits, i ); }

	Vector			PanelToWorld( const Vector2D &uv, float n ) const;
	Vector			PanelDirToWorld( float u, float v, float n ) const;

	// Debris
	void			SpawnDebrisForShard( int nShard, bool bFromImpact );
	void			AddDebrisPiece( const Vector2D *pUV, int nVerts, const Vector &vecVelocity, const QAngle &angVel, int nGeneration, const matrix3x4_t &rot, const Vector *pPos );
	void			SplitDebris( const ShatterGlassDebris_t &debris, int nPieces, Vector2D *pOutUV, int *pOutCounts, int nMaxVertsPerPiece ) const;
	void			SimulateDebris( float flFrameTime );
	void			PlayShardSound( const Vector &vecPos, float flSpeed );
	void			UpdateDebrisBounds( void );

	// Drawing
	void			EmitPolygon( const Vector *pPos, const Vector2D *pUV, int nVerts, const Vector &vecFacing, const ShatterGlassAffine_t &tex, const ShatterGlassAffine_t &lm );
	void			FlushVerts( IMaterial *pMaterial );
	void			DrawExtras( const ShatterGlassAffine_t &tex, const ShatterGlassAffine_t &lm );
	void			DrawCracks( void );

	// Space the current brush surface's vertices are in
	Vector			ToDrawSpace( const Vector &vecWorld ) const;
	Vector			ToDrawSpaceDir( const Vector &vecWorld ) const;

private:
	// Networked
	int				m_nSeed;
	float			m_flShardSize;
	Vector			m_vecPanelMins;
	Vector			m_vecPanelMaxs;
	int				m_nBrokenBits[SHATTERGLASS_STATE_WORDS];
	int				m_nCrackedBits[SHATTERGLASS_STATE_WORDS];
	int				m_nImpactSerial;
	Vector			m_vecImpactPos;
	Vector			m_vecImpactForce;
	float			m_flImpactRadius;

	// Local
	ShatterGlassPanel_t		m_Panel;
	CShatterGlassPattern	m_Pattern;
	int				m_nBuiltSeed;
	float			m_flBuiltShardSize;
	Vector			m_vecBuiltMins, m_vecBuiltMaxs;

	int				m_nPrevBrokenBits[SHATTERGLASS_STATE_WORDS];
	int				m_nPrevImpactSerial;
	int				m_nBrokenCount;
	int				m_nCrackedCount;

	int				m_nContents;
	unsigned short	m_nSurfaceProps;
	const char		*m_pszShardSound;
	float			m_flNextShardSound;

	CUtlVector<ShatterGlassDebris_t>	m_Debris;
	Vector			m_vecDebrisMins, m_vecDebrisMaxs;	// Render space
	bool			m_bHasDebrisBounds;

	CUtlVector<ShatterGlassVert_t>		m_RenderVerts;
	CMaterialReference					m_CrackMaterial;

	// Per-draw state
	bool			m_bDrewExtras;
	bool			m_bDrewCracks;
	bool			m_bFrontIsCCW;
	bool			m_bDrawSpaceIsLocal;
	matrix3x4_t		m_matRender;
};

IMPLEMENT_CLIENTCLASS_DT( C_ShatterGlass, DT_ShatterGlass, CShatterGlass )
	RecvPropInt( RECVINFO( m_nSeed ) ),
	RecvPropFloat( RECVINFO( m_flShardSize ) ),
	RecvPropVector( RECVINFO( m_vecPanelMins ) ),
	RecvPropVector( RECVINFO( m_vecPanelMaxs ) ),
	RecvPropArray3( RECVINFO_ARRAY( m_nBrokenBits ), RecvPropInt( RECVINFO( m_nBrokenBits[0] ) ) ),
	RecvPropArray3( RECVINFO_ARRAY( m_nCrackedBits ), RecvPropInt( RECVINFO( m_nCrackedBits[0] ) ) ),
	RecvPropInt( RECVINFO( m_nImpactSerial ) ),
	RecvPropVector( RECVINFO( m_vecImpactPos ) ),
	RecvPropVector( RECVINFO( m_vecImpactForce ) ),
	RecvPropFloat( RECVINFO( m_flImpactRadius ) ),
END_RECV_TABLE()

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
C_ShatterGlass::C_ShatterGlass()
{
	m_nSeed = 0;
	m_flShardSize = SHATTERGLASS_DEFAULT_SHARD_SIZE;
	m_vecPanelMins.Init();
	m_vecPanelMaxs.Init();
	V_memset( m_nBrokenBits, 0, sizeof( m_nBrokenBits ) );
	V_memset( m_nCrackedBits, 0, sizeof( m_nCrackedBits ) );
	V_memset( m_nPrevBrokenBits, 0, sizeof( m_nPrevBrokenBits ) );
	m_nImpactSerial = m_nPrevImpactSerial = 0;
	m_vecImpactPos.Init();
	m_vecImpactForce.Init();
	m_flImpactRadius = 0.0f;

	m_nBuiltSeed = -1;
	m_flBuiltShardSize = -1.0f;
	m_vecBuiltMins.Init();
	m_vecBuiltMaxs.Init();

	m_nBrokenCount = m_nCrackedCount = 0;
	m_nContents = CONTENTS_SOLID;
	m_nSurfaceProps = 0;
	m_pszShardSound = NULL;
	m_flNextShardSound = 0.0f;
	m_bHasDebrisBounds = false;
	m_vecDebrisMins.Init();
	m_vecDebrisMaxs.Init();

	m_bDrewExtras = false;
	m_bDrewCracks = false;
	m_bFrontIsCCW = false;
	m_bDrawSpaceIsLocal = true;
	SetIdentityMatrix( m_matRender );
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void C_ShatterGlass::BuildPattern( void )
{
	m_Panel.Init( m_vecPanelMins, m_vecPanelMaxs );
	m_Pattern.Generate( m_Panel.m_flWidth, m_Panel.m_flHeight, m_flShardSize, m_nSeed );

	m_nBuiltSeed = m_nSeed;
	m_flBuiltShardSize = m_flShardSize;
	m_vecBuiltMins = m_vecPanelMins;
	m_vecBuiltMaxs = m_vecPanelMaxs;
}

void C_ShatterGlass::CountStates( void )
{
	m_nBrokenCount = 0;
	m_nCrackedCount = 0;
	for ( int i = 0; i < m_Pattern.Count(); i++ )
	{
		if ( IsBroken( i ) )
			m_nBrokenCount++;
		else if ( IsCracked( i ) )
			m_nCrackedCount++;
	}
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
void C_ShatterGlass::OnDataChanged( DataUpdateType_t updateType )
{
	BaseClass::OnDataChanged( updateType );

	bool bNewPattern = ( m_nBuiltSeed != m_nSeed || m_flBuiltShardSize != m_flShardSize ||
		m_vecBuiltMins != m_vecPanelMins || m_vecBuiltMaxs != m_vecPanelMaxs );

	if ( updateType == DATA_UPDATE_CREATED || bNewPattern )
	{
		BuildPattern();

		m_nContents = GetModel() ? modelinfo->GetModelContents( GetModelIndex() ) : CONTENTS_SOLID;
		if ( m_nContents == 0 )
			m_nContents = CONTENTS_SOLID;

		int nSurface = physprops->GetSurfaceIndex( "glass" );
		m_nSurfaceProps = (unsigned short)MAX( nSurface, 0 );

		const surfacedata_t *pSurface = physprops->GetSurfaceData( m_nSurfaceProps );
		m_pszShardSound = pSurface ? physprops->GetString( pSurface->sounds.impactHard ) : NULL;
		if ( m_pszShardSound && m_pszShardSound[0] )
			PrecacheScriptSound( m_pszShardSound );
		else
			m_pszShardSound = NULL;

		if ( !m_CrackMaterial.IsValid() )
		{
			const char *pszCrackMaterial = "__shatterglass_crack";
			if ( materials->IsMaterialLoaded( pszCrackMaterial ) )
			{
				m_CrackMaterial.Init( pszCrackMaterial, TEXTURE_GROUP_CLIENT_EFFECTS );
			}
			else
			{
				KeyValues *pVMT = new KeyValues( "UnlitGeneric" );
				pVMT->SetString( "$basetexture", "vgui/white" );
				pVMT->SetInt( "$vertexcolor", 1 );
				pVMT->SetInt( "$vertexalpha", 1 );
				pVMT->SetInt( "$additive", 1 );
				pVMT->SetInt( "$nocull", 1 );
				m_CrackMaterial.Init( pszCrackMaterial, TEXTURE_GROUP_CLIENT_EFFECTS, pVMT );
			}
		}

		// Don't throw debris for shards that were already gone when we got here
		V_memcpy( m_nPrevBrokenBits, m_nBrokenBits, sizeof( m_nPrevBrokenBits ) );
		m_nPrevImpactSerial = m_nImpactSerial;
		CountStates();
		return;
	}

	bool bFreshImpact = ( m_nImpactSerial != m_nPrevImpactSerial );
	m_nPrevImpactSerial = m_nImpactSerial;

	bool bChanged = false;
	for ( int w = 0; w < SHATTERGLASS_STATE_WORDS; w++ )
	{
		unsigned int nNew = ((unsigned int)m_nBrokenBits[w]) & ~((unsigned int)m_nPrevBrokenBits[w]);
		if ( nNew == 0 )
			continue;

		bChanged = true;
		if ( !r_shatterglass_debris.GetBool() )
			continue;

		for ( int b = 0; b < 32; b++ )
		{
			if ( nNew & ( 1u << b ) )
			{
				int nShard = w * 32 + b;
				if ( nShard < m_Pattern.Count() )
					SpawnDebrisForShard( nShard, bFreshImpact );
			}
		}
	}

	V_memcpy( m_nPrevBrokenBits, m_nBrokenBits, sizeof( m_nPrevBrokenBits ) );
	CountStates();

	if ( bChanged && m_Debris.Count() > 0 )
	{
		UpdateDebrisBounds();
		SetNextClientThink( CLIENT_THINK_ALWAYS );
	}
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
bool C_ShatterGlass::TestCollision( const Ray_t &ray, unsigned int fContentsMask, trace_t &tr )
{
	return ShatterGlass_TestCollision( ray, fContentsMask, tr, EntityToWorldTransform(), m_Panel, m_Pattern,
		m_nBrokenBits, m_nBrokenCount, m_nContents, m_nSurfaceProps );
}

//-----------------------------------------------------------------------------
// Coordinate helpers
//-----------------------------------------------------------------------------
Vector C_ShatterGlass::PanelToWorld( const Vector2D &uv, float n ) const
{
	Vector vecWorld;
	VectorTransform( m_Panel.LocalPoint( uv.x, uv.y, n ), EntityToWorldTransform(), vecWorld );
	return vecWorld;
}

Vector C_ShatterGlass::PanelDirToWorld( float u, float v, float n ) const
{
	Vector vecLocal;
	vecLocal[m_Panel.m_iU] = u;
	vecLocal[m_Panel.m_iV] = v;
	vecLocal[m_Panel.m_iN] = n;

	Vector vecWorld;
	VectorRotate( vecLocal, EntityToWorldTransform(), vecWorld );
	return vecWorld;
}

Vector C_ShatterGlass::ToDrawSpace( const Vector &vecWorld ) const
{
	if ( !m_bDrawSpaceIsLocal )
		return vecWorld;

	Vector vecLocal;
	VectorITransform( vecWorld, m_matRender, vecLocal );
	return vecLocal;
}

Vector C_ShatterGlass::ToDrawSpaceDir( const Vector &vecWorld ) const
{
	if ( !m_bDrawSpaceIsLocal )
		return vecWorld;

	Vector vecLocal;
	VectorIRotate( vecWorld, m_matRender, vecLocal );
	return vecLocal;
}

//-----------------------------------------------------------------------------
// Debris
//-----------------------------------------------------------------------------
void C_ShatterGlass::AddDebrisPiece( const Vector2D *pUV, int nVerts, const Vector &vecVelocity, const QAngle &angVel, int nGeneration, const matrix3x4_t &rot, const Vector *pPos )
{
	if ( nVerts < 3 )
		return;

	// Too many? Drop the oldest
	int nMax = MAX( r_shatterglass_debris_max.GetInt(), 0 );
	if ( nMax == 0 )
		return;
	while ( m_Debris.Count() >= nMax )
		m_Debris.Remove( 0 );

	ShatterGlassDebris_t &debris = m_Debris[m_Debris.AddToTail()];
	debris.nVerts = MIN( nVerts, SHATTERGLASS_MAX_VERTS );
	for ( int i = 0; i < debris.nVerts; i++ )
		debris.uv[i] = pUV[i];

	CShatterGlassPattern::ComputeAreaAndCentroid( debris.uv, debris.nVerts, debris.flArea, debris.center );

	MatrixCopy( rot, debris.rot );
	if ( pPos )
	{
		debris.pos = *pPos;
	}
	else
	{
		debris.pos = PanelToWorld( debris.center, m_Panel.m_flThickness * 0.5f );
	}

	debris.vel = vecVelocity;
	debris.angVel = angVel;
	debris.flDieTime = gpGlobals->curtime + r_shatterglass_debris_lifetime.GetFloat() * RandomFloat( 0.8f, 1.3f );
	debris.nGeneration = nGeneration;
	debris.nBounces = 0;
	debris.bResting = false;
}

void C_ShatterGlass::SplitDebris( const ShatterGlassDebris_t &debris, int nPieces, Vector2D *pOutUV, int *pOutCounts, int nMaxVertsPerPiece ) const
{
	// Cut the piece with lines through (roughly) its center
	float flJitter = 0.15f * sqrtf( debris.flArea );
	Vector2D vecCenter = debris.center + Vector2D( RandomFloat( -flJitter, flJitter ), RandomFloat( -flJitter, flJitter ) );
	float flBaseAngle = RandomFloat( 0.0f, M_PI_F );

	for ( int p = 0; p < nPieces; p++ )
	{
		// Wedge between two cut directions
		float flA0 = flBaseAngle + ( 2.0f * M_PI_F * p ) / nPieces + RandomFloat( -0.3f, 0.3f ) / nPieces;
		float flA1 = flBaseAngle + ( 2.0f * M_PI_F * ( p + 1 ) ) / nPieces;

		Vector2D bufA[SHATTERGLASS_MAX_VERTS], bufB[SHATTERGLASS_MAX_VERTS];
		int n = debris.nVerts;
		for ( int i = 0; i < n; i++ )
			bufA[i] = debris.uv[i];

		// Keep the side counter-clockwise of direction A0...
		Vector2D vecN0( sinf( flA0 ), -cosf( flA0 ) );
		n = CShatterGlassPattern::ClipPolygon( bufA, NULL, n, vecN0, DotProduct2D( vecCenter, vecN0 ), 0, bufB, NULL, SHATTERGLASS_MAX_VERTS );

		// ...and clockwise of direction A1
		if ( nPieces > 1 && n >= 3 )
		{
			Vector2D vecN1( -sinf( flA1 ), cosf( flA1 ) );
			n = CShatterGlassPattern::ClipPolygon( bufB, NULL, n, vecN1, DotProduct2D( vecCenter, vecN1 ), 0, bufA, NULL, SHATTERGLASS_MAX_VERTS );
		}
		else
		{
			for ( int i = 0; i < n; i++ )
				bufA[i] = bufB[i];
		}

		n = MIN( n, nMaxVertsPerPiece );
		pOutCounts[p] = ( n >= 3 ) ? n : 0;
		for ( int i = 0; i < pOutCounts[p]; i++ )
			pOutUV[p * nMaxVertsPerPiece + i] = bufA[i];
	}
}

void C_ShatterGlass::SpawnDebrisForShard( int nShard, bool bFromImpact )
{
	const CShatterGlassPattern::Shard_t &shard = m_Pattern.Get( nShard );
	if ( shard.nVerts < 3 )
		return;

	// The pane's axes in world space
	matrix3x4_t rot;
	Vector vecU = PanelDirToWorld( 1, 0, 0 );
	Vector vecV = PanelDirToWorld( 0, 1, 0 );
	Vector vecN = PanelDirToWorld( 0, 0, 1 );
	MatrixInitialize( rot, vec3_origin, vecU, vecV, vecN );

	Vector2D vecImpactUV( m_vecImpactPos.x, m_vecImpactPos.y );
	float flFalloff = 0.0f;
	Vector2D vecAway( 0, 0 );
	if ( bFromImpact )
	{
		float flDist = shard.centroid.DistTo( vecImpactUV );
		flFalloff = clamp( 1.0f - flDist / MAX( m_flImpactRadius * 2.5f, 1.0f ), 0.0f, 1.0f );
		vecAway = shard.centroid - vecImpactUV;
		Vector2DNormalize( vecAway );
	}

	// Big shards break up as they come loose
	ShatterGlassDebris_t temp;
	temp.nVerts = shard.nVerts;
	for ( int i = 0; i < shard.nVerts; i++ )
		temp.uv[i] = shard.verts[i];
	temp.center = shard.centroid;
	temp.flArea = shard.area;

	float flCellArea = m_Pattern.CellSize() * m_Pattern.CellSize();
	int nPieces = 1;
	if ( shard.area > flCellArea * 0.35f )
		nPieces = ( bFromImpact && flFalloff > 0.5f ) ? 3 : 2;
	if ( m_Debris.Count() > r_shatterglass_debris_max.GetInt() / 2 )
		nPieces = 1;

	Vector2D pieces[3 * SHATTERGLASS_MAX_VERTS];
	int nCounts[3];
	if ( nPieces > 1 )
	{
		SplitDebris( temp, nPieces, pieces, nCounts, SHATTERGLASS_MAX_VERTS );
	}
	else
	{
		nCounts[0] = shard.nVerts;
		for ( int i = 0; i < shard.nVerts; i++ )
			pieces[i] = shard.verts[i];
	}

	for ( int p = 0; p < nPieces; p++ )
	{
		if ( nCounts[p] < 3 )
			continue;

		Vector vecVelocity;
		QAngle angVel;
		if ( bFromImpact )
		{
			vecVelocity = m_vecImpactForce * ( 0.25f + 0.75f * flFalloff ) * RandomFloat( 0.6f, 1.2f );
			vecVelocity += ( vecU * vecAway.x + vecV * vecAway.y ) * ( 60.0f * flFalloff * RandomFloat( 0.5f, 1.0f ) );
			vecVelocity += RandomVector( -15.0f, 15.0f );
			angVel = RandomAngle( -360.0f, 360.0f ) * ( 0.3f + flFalloff );
		}
		else
		{
			// Loose shards mostly just drop out of the frame
			vecVelocity = RandomVector( -8.0f, 8.0f ) + vecN * RandomFloat( -20.0f, 20.0f );
			angVel = RandomAngle( -120.0f, 120.0f );
		}

		AddDebrisPiece( &pieces[p * SHATTERGLASS_MAX_VERTS], nCounts[p], vecVelocity, angVel, ( nPieces > 1 ) ? 1 : 0, rot, NULL );
	}
}

void C_ShatterGlass::PlayShardSound( const Vector &vecPos, float flSpeed )
{
	if ( !m_pszShardSound || flSpeed < 60.0f || gpGlobals->curtime < m_flNextShardSound )
		return;

	CLocalPlayerFilter filter;
	EmitSound( filter, entindex(), m_pszShardSound, &vecPos );
	m_flNextShardSound = gpGlobals->curtime + RandomFloat( 0.08f, 0.2f );
}

void C_ShatterGlass::SimulateDebris( float flFrameTime )
{
	if ( flFrameTime <= 0.0f )
		return;

	float flGravity = s_sv_gravity.IsValid() ? s_sv_gravity.GetFloat() : 600.0f;

	for ( int i = m_Debris.Count() - 1; i >= 0; i-- )
	{
		ShatterGlassDebris_t &debris = m_Debris[i];

		if ( gpGlobals->curtime >= debris.flDieTime )
		{
			m_Debris.Remove( i );
			continue;
		}

		if ( debris.bResting )
			continue;

		debris.vel.z -= flGravity * flFrameTime;

		Vector vecNewPos = debris.pos + debris.vel * flFrameTime;

		// Tumble
		matrix3x4_t spin, newRot;
		AngleMatrix( debris.angVel * flFrameTime, spin );
		ConcatTransforms( spin, debris.rot, newRot );
		MatrixCopy( newRot, debris.rot );

		trace_t tr;
		UTIL_TraceLine( debris.pos, vecNewPos, MASK_SOLID_BRUSHONLY, this, COLLISION_GROUP_DEBRIS, &tr );
		if ( tr.fraction >= 1.0f || tr.startsolid )
		{
			debris.pos = vecNewPos;
			continue;
		}

		debris.pos = tr.endpos + tr.plane.normal * 0.5f;

		float flSpeed = debris.vel.Length();
		PlayShardSound( debris.pos, flSpeed );

		Vector vecReflect = debris.vel - 2.0f * DotProduct( debris.vel, tr.plane.normal ) * tr.plane.normal;

		float flMinArea = 1.5f;
		if ( flSpeed > 150.0f && debris.nGeneration < SHATTERGLASS_MAX_DEBRIS_GENERATION && debris.flArea > flMinArea * 2.0f )
		{
			// Shatter on impact
			ShatterGlassDebris_t copy = debris;
			m_Debris.Remove( i );

			int nPieces = ( copy.flArea > 12.0f ) ? 3 : 2;
			Vector2D pieces[3 * SHATTERGLASS_MAX_VERTS];
			int nCounts[3];
			SplitDebris( copy, nPieces, pieces, nCounts, SHATTERGLASS_MAX_VERTS );

			for ( int p = 0; p < nPieces; p++ )
			{
				if ( nCounts[p] < 3 )
					continue;

				Vector vecVelocity = vecReflect * RandomFloat( 0.15f, 0.35f ) + RandomVector( -40.0f, 40.0f );
				vecVelocity += tr.plane.normal * RandomFloat( 10.0f, 50.0f );

				// Keep each piece where it was on the original shard
				Vector2D vecPieceCenter;
				float flPieceArea;
				CShatterGlassPattern::ComputeAreaAndCentroid( &pieces[p * SHATTERGLASS_MAX_VERTS], nCounts[p], flPieceArea, vecPieceCenter );
				Vector2D vecOffset = vecPieceCenter - copy.center;
				Vector vecLocalOffset( vecOffset.x, vecOffset.y, 0.0f ), vecWorldOffset;
				VectorRotate( vecLocalOffset, copy.rot, vecWorldOffset );
				Vector vecPos = copy.pos + vecWorldOffset + tr.plane.normal * 0.5f;

				AddDebrisPiece( &pieces[p * SHATTERGLASS_MAX_VERTS], nCounts[p], vecVelocity, RandomAngle( -540.0f, 540.0f ),
					copy.nGeneration + 1, copy.rot, &vecPos );
			}
			continue;
		}

		debris.vel = vecReflect * 0.3f;
		debris.angVel *= 0.5f;
		debris.nBounces++;

		if ( flSpeed < 40.0f || debris.nBounces > 3 || ( tr.plane.normal.z > 0.7f && flSpeed < 100.0f ) )
		{
			// Settle flat on the surface
			debris.bResting = true;
			debris.vel.Init();

			Vector vecNormal = tr.plane.normal;
			Vector vecU;
			MatrixGetColumn( debris.rot, 0, vecU );
			vecU -= vecNormal * DotProduct( vecU, vecNormal );
			if ( VectorNormalize( vecU ) < 0.01f )
			{
				Vector vecUp;
				VectorVectors( vecNormal, vecU, vecUp );
			}

			Vector vecV = CrossProduct( vecNormal, vecU );
			Vector vecOrigin = debris.pos + vecNormal * ( m_Panel.m_flThickness * 0.5f );
			MatrixInitialize( debris.rot, vec3_origin, vecU, vecV, vecNormal );
			debris.pos = vecOrigin;
		}
	}
}

void C_ShatterGlass::UpdateDebrisBounds( void )
{
	m_bHasDebrisBounds = false;
	if ( m_Debris.Count() == 0 )
		return;

	const matrix3x4_t &matRender = RenderableToWorldTransform();

	Vector vecMins( FLT_MAX, FLT_MAX, FLT_MAX ), vecMaxs( -FLT_MAX, -FLT_MAX, -FLT_MAX );
	float flPad = m_Pattern.CellSize() * 2.0f;
	for ( int i = 0; i < m_Debris.Count(); i++ )
	{
		Vector vecLocal;
		VectorITransform( m_Debris[i].pos, matRender, vecLocal );
		VectorMin( vecMins, vecLocal - Vector( flPad, flPad, flPad ), vecMins );
		VectorMax( vecMaxs, vecLocal + Vector( flPad, flPad, flPad ), vecMaxs );
	}

	m_vecDebrisMins = vecMins;
	m_vecDebrisMaxs = vecMaxs;
	m_bHasDebrisBounds = true;
}

void C_ShatterGlass::ClientThink( void )
{
	if ( m_Debris.Count() == 0 )
	{
		m_bHasDebrisBounds = false;
		SetNextClientThink( CLIENT_THINK_NEVER );
		ClientLeafSystem()->RenderableChanged( RenderHandle() );
		return;
	}

	SimulateDebris( MIN( gpGlobals->frametime, 0.1f ) );
	UpdateDebrisBounds();
	ClientLeafSystem()->RenderableChanged( RenderHandle() );
}

void C_ShatterGlass::GetRenderBounds( Vector &theMins, Vector &theMaxs )
{
	BaseClass::GetRenderBounds( theMins, theMaxs );

	if ( m_bHasDebrisBounds )
	{
		VectorMin( theMins, m_vecDebrisMins, theMins );
		VectorMax( theMaxs, m_vecDebrisMaxs, theMaxs );
	}
}

//-----------------------------------------------------------------------------
// Drawing
//-----------------------------------------------------------------------------
bool C_ShatterGlass::IsTransparent( void )
{
	// The pane has holes in it
	if ( m_nBrokenCount > 0 || m_nCrackedCount > 0 || m_Debris.Count() > 0 )
		return true;

	return BaseClass::IsTransparent();
}

int C_ShatterGlass::DrawModel( int flags )
{
	if ( !m_bReadyToDraw )
		return 0;

	bool bDamaged = ( m_nBrokenCount > 0 || m_nCrackedCount > 0 );
	if ( ( !bDamaged || !m_Pattern.IsValid() ) && m_Debris.Count() == 0 )
		return BaseClass::DrawModel( flags );

	if ( m_nBrokenCount >= m_Pattern.Count() && m_Debris.Count() == 0 )
		return 0;

	m_bDrewExtras = false;
	m_bDrewCracks = false;
	m_matRender = RenderableToWorldTransform();

	render->InstallBrushSurfaceRenderer( this );
	BaseClass::DrawModel( flags | STUDIO_TRANSPARENCY );
	render->InstallBrushSurfaceRenderer( NULL );

	return 1;
}

void C_ShatterGlass::EmitPolygon( const Vector *pPos, const Vector2D *pUV, int nVerts, const Vector &vecFacing, const ShatterGlassAffine_t &tex, const ShatterGlassAffine_t &lm )
{
	if ( nVerts < 3 )
		return;

	// Newell normal of the winding as given
	Vector vecWinding( 0, 0, 0 );
	for ( int i = 0; i < nVerts; i++ )
	{
		const Vector &a = pPos[i];
		const Vector &b = pPos[(i + 1) % nVerts];
		vecWinding.x += ( a.y - b.y ) * ( a.z + b.z );
		vecWinding.y += ( a.z - b.z ) * ( a.x + b.x );
		vecWinding.z += ( a.x - b.x ) * ( a.y + b.y );
	}

	bool bReverse = ( ( DotProduct( vecWinding, vecFacing ) > 0.0f ) != m_bFrontIsCCW );

	Vector vecNormal = vecFacing;
	VectorNormalize( vecNormal );

	for ( int i = 1; i < nVerts - 1; i++ )
	{
		int nIndices[3] = { 0, i, i + 1 };
		if ( bReverse )
		{
			nIndices[1] = i + 1;
			nIndices[2] = i;
		}

		for ( int k = 0; k < 3; k++ )
		{
			int v = nIndices[k];
			ShatterGlassVert_t &vert = m_RenderVerts[m_RenderVerts.AddToTail()];
			vert.pos = pPos[v];
			vert.normal = vecNormal;
			vert.tex = tex.Map( pUV[v] );
			vert.lightmap = lm.Map( pUV[v] );
		}
	}
}

void C_ShatterGlass::FlushVerts( IMaterial *pMaterial )
{
	if ( m_RenderVerts.Count() < 3 )
	{
		m_RenderVerts.RemoveAll();
		return;
	}

	CMatRenderContextPtr pRenderContext( materials );
	pRenderContext->Bind( pMaterial, (IClientRenderable *)this );

	const int nMaxTrisPerBatch = 2048;
	int nTotalTris = m_RenderVerts.Count() / 3;

	for ( int nFirstTri = 0; nFirstTri < nTotalTris; nFirstTri += nMaxTrisPerBatch )
	{
		int nTris = MIN( nMaxTrisPerBatch, nTotalTris - nFirstTri );

		IMesh *pMesh = pRenderContext->GetDynamicMesh();
		CMeshBuilder meshBuilder;
		meshBuilder.Begin( pMesh, MATERIAL_TRIANGLES, nTris );

		for ( int v = nFirstTri * 3; v < ( nFirstTri + nTris ) * 3; v++ )
		{
			const ShatterGlassVert_t &vert = m_RenderVerts[v];
			meshBuilder.Position3fv( vert.pos.Base() );
			meshBuilder.Normal3fv( vert.normal.Base() );
			meshBuilder.Color4ub( 255, 255, 255, 255 );
			meshBuilder.TexCoord2fv( 0, vert.tex.Base() );
			meshBuilder.TexCoord2fv( 1, vert.lightmap.Base() );
			meshBuilder.TexCoord2f( 2, 0.0f, 0.0f );
			meshBuilder.AdvanceVertex();
		}

		meshBuilder.End();
		pMesh->Draw();
	}

	m_RenderVerts.RemoveAll();
}

bool C_ShatterGlass::RenderBrushModelSurface( IClientEntity *pBaseEntity, IBrushSurface *pBrushSurface )
{
	int nFaceVerts = pBrushSurface->GetVertexCount();
	if ( nFaceVerts < 3 || !m_Pattern.IsValid() )
		return false;

	BrushVertex_t *pFaceVerts = (BrushVertex_t *)stackalloc( nFaceVerts * sizeof( BrushVertex_t ) );
	pBrushSurface->GetVertexData( pFaceVerts );

	// Figure out whether the brush's vertices are in entity space or world space
	// by checking which interpretation puts them on the pane's bounds
	const float flTolerance = 0.5f;
	Vector vecLocal0 = pFaceVerts[0].m_Pos;
	bool bLocalFits = true;
	for ( int k = 0; k < 3 && bLocalFits; k++ )
	{
		if ( vecLocal0[k] < m_vecPanelMins[k] - flTolerance || vecLocal0[k] > m_vecPanelMaxs[k] + flTolerance )
			bLocalFits = false;
	}
	m_bDrawSpaceIsLocal = bLocalFits;

	Vector vecFaceNormalLocal = pFaceVerts[0].m_Normal;
	if ( !m_bDrawSpaceIsLocal )
		VectorIRotate( pFaceVerts[0].m_Normal, m_matRender, vecFaceNormalLocal );

	// The engine's front face winding
	{
		Vector vecEdge1 = pFaceVerts[1].m_Pos - pFaceVerts[0].m_Pos;
		Vector vecEdge2 = pFaceVerts[2].m_Pos - pFaceVerts[0].m_Pos;
		m_bFrontIsCCW = DotProduct( CrossProduct( vecEdge1, vecEdge2 ), pFaceVerts[0].m_Normal ) > 0.0f;
	}

	// Only the two big faces of the pane are drawn; the rim is hidden in the frame
	float flAlong = vecFaceNormalLocal[m_Panel.m_iN];
	if ( fabs( flAlong ) < 0.9f )
		return false;

	// Map pane coordinates to this face's texture and lightmap coordinates
	Vector2D faceUV[3], faceTex[3], faceLM[3];
	{
		// Pick a well-shaped triangle from the face
		int i0 = 0, i1 = 1, i2 = 2;
		float flBestArea = -1.0f;
		for ( int a = 1; a < nFaceVerts - 1; a++ )
		{
			for ( int b = a + 1; b < nFaceVerts; b++ )
			{
				float flArea = CrossProduct( pFaceVerts[a].m_Pos - pFaceVerts[0].m_Pos, pFaceVerts[b].m_Pos - pFaceVerts[0].m_Pos ).LengthSqr();
				if ( flArea > flBestArea )
				{
					flBestArea = flArea;
					i1 = a;
					i2 = b;
				}
			}
		}

		int nIdx[3] = { i0, i1, i2 };
		for ( int k = 0; k < 3; k++ )
		{
			const BrushVertex_t &vert = pFaceVerts[nIdx[k]];
			Vector vecLocal = vert.m_Pos;
			if ( !m_bDrawSpaceIsLocal )
				VectorITransform( vert.m_Pos, m_matRender, vecLocal );

			m_Panel.LocalToPanel( vecLocal, faceUV[k] );
			faceTex[k] = vert.m_TexCoord;
			faceLM[k] = vert.m_LightmapCoord;
		}
	}

	ShatterGlassAffine_t tex, lm;
	if ( !SolveAffine( faceUV, faceTex, tex ) || !SolveAffine( faceUV, faceLM, lm ) )
		return false;

	// Which side of the pane this face is on
	Vector vecFaceLocal = pFaceVerts[0].m_Pos;
	if ( !m_bDrawSpaceIsLocal )
		VectorITransform( pFaceVerts[0].m_Pos, m_matRender, vecFaceLocal );
	float flFaceN = vecFaceLocal[m_Panel.m_iN] - m_vecPanelMins[m_Panel.m_iN];

	Vector vecFacingLocal( 0, 0, 0 );
	vecFacingLocal[m_Panel.m_iN] = ( flAlong > 0.0f ) ? 1.0f : -1.0f;

	Vector vecFacingWorld;
	VectorRotate( vecFacingLocal, m_matRender, vecFacingWorld );
	Vector vecFacing = m_bDrawSpaceIsLocal ? vecFacingLocal : vecFacingWorld;

	IMaterial *pMaterial = pBrushSurface->GetMaterial();

	// The shards still in place
	m_RenderVerts.RemoveAll();
	Vector polyPos[SHATTERGLASS_MAX_VERTS];
	for ( int i = 0; i < m_Pattern.Count(); i++ )
	{
		if ( IsBroken( i ) )
			continue;

		const CShatterGlassPattern::Shard_t &shard = m_Pattern.Get( i );
		if ( shard.nVerts < 3 )
			continue;

		for ( int v = 0; v < shard.nVerts; v++ )
		{
			Vector vecLocal = m_Panel.LocalPoint( shard.verts[v].x, shard.verts[v].y, flFaceN );
			if ( m_bDrawSpaceIsLocal )
				polyPos[v] = vecLocal;
			else
				VectorTransform( vecLocal, m_matRender, polyPos[v] );
		}

		EmitPolygon( polyPos, shard.verts, shard.nVerts, vecFacing, tex, lm );
	}

	// Hole edges and debris are drawn once per pass, with whichever face comes first
	if ( !m_bDrewExtras )
	{
		m_bDrewExtras = true;
		DrawExtras( tex, lm );
	}

	FlushVerts( pMaterial );

	// Cracks use a two-sided material, so once per pass is enough
	if ( !m_bDrewCracks && m_nCrackedCount > 0 && r_shatterglass_cracks.GetBool() )
	{
		m_bDrewCracks = true;
		DrawCracks();
	}

	// Decals don't line up with the holes
	return false;
}

void C_ShatterGlass::DrawExtras( const ShatterGlassAffine_t &tex, const ShatterGlassAffine_t &lm )
{
	const int iU = m_Panel.m_iU, iV = m_Panel.m_iV, iN = m_Panel.m_iN;
	float flThickness = m_Panel.m_flThickness;

	Vector quadPos[4];
	Vector2D quadUV[4];

	// Edges of the holes, so the glass has some thickness to it
	if ( m_nBrokenCount > 0 && flThickness > 0.1f )
	{
		for ( int i = 0; i < m_Pattern.Count(); i++ )
		{
			if ( IsBroken( i ) )
				continue;

			const CShatterGlassPattern::Shard_t &shard = m_Pattern.Get( i );
			for ( int v = 0; v < shard.nVerts; v++ )
			{
				int nTag = shard.edgeTag[v];
				if ( nTag < 0 || !IsBroken( nTag ) )
					continue;

				const Vector2D &a = shard.verts[v];
				const Vector2D &b = shard.verts[(v + 1) % shard.nVerts];

				Vector vecOutLocal( 0, 0, 0 );
				vecOutLocal[iU] = b.y - a.y;
				vecOutLocal[iV] = a.x - b.x;
				vecOutLocal[iN] = 0.0f;

				Vector vecLocal[4] =
				{
					m_Panel.LocalPoint( a.x, a.y, flThickness ),
					m_Panel.LocalPoint( b.x, b.y, flThickness ),
					m_Panel.LocalPoint( b.x, b.y, 0.0f ),
					m_Panel.LocalPoint( a.x, a.y, 0.0f ),
				};

				for ( int k = 0; k < 4; k++ )
				{
					if ( m_bDrawSpaceIsLocal )
						quadPos[k] = vecLocal[k];
					else
						VectorTransform( vecLocal[k], m_matRender, quadPos[k] );
				}
				quadUV[0] = a; quadUV[1] = b; quadUV[2] = b; quadUV[3] = a;

				Vector vecOut = vecOutLocal;
				if ( !m_bDrawSpaceIsLocal )
					VectorRotate( vecOutLocal, m_matRender, vecOut );

				EmitPolygon( quadPos, quadUV, 4, vecOut, tex, lm );
			}
		}
	}

	// Falling shards, both sides plus edges
	float flHalf = flThickness * 0.5f;
	Vector frontPos[SHATTERGLASS_MAX_VERTS], backPos[SHATTERGLASS_MAX_VERTS];
	for ( int d = 0; d < m_Debris.Count(); d++ )
	{
		const ShatterGlassDebris_t &debris = m_Debris[d];

		// Shrink away at the end of its life
		float flScale = clamp( ( debris.flDieTime - gpGlobals->curtime ) / 0.5f, 0.0f, 1.0f );
		if ( flScale <= 0.0f )
			continue;

		Vector vecU, vecV, vecN;
		MatrixGetColumn( debris.rot, 0, vecU );
		MatrixGetColumn( debris.rot, 1, vecV );
		MatrixGetColumn( debris.rot, 2, vecN );

		for ( int v = 0; v < debris.nVerts; v++ )
		{
			Vector2D vecOffset = ( debris.uv[v] - debris.center ) * flScale;
			Vector vecBase = debris.pos + vecU * vecOffset.x + vecV * vecOffset.y;
			frontPos[v] = ToDrawSpace( vecBase + vecN * ( flHalf * flScale ) );
			backPos[v] = ToDrawSpace( vecBase - vecN * ( flHalf * flScale ) );
		}

		Vector vecDrawN = ToDrawSpaceDir( vecN );
		EmitPolygon( frontPos, debris.uv, debris.nVerts, vecDrawN, tex, lm );
		EmitPolygon( backPos, debris.uv, debris.nVerts, -vecDrawN, tex, lm );

		if ( flThickness > 0.1f )
		{
			for ( int v = 0; v < debris.nVerts; v++ )
			{
				int w = ( v + 1 ) % debris.nVerts;
				const Vector2D &a = debris.uv[v];
				const Vector2D &b = debris.uv[w];

				quadPos[0] = frontPos[v]; quadPos[1] = frontPos[w];
				quadPos[2] = backPos[w]; quadPos[3] = backPos[v];
				quadUV[0] = a; quadUV[1] = b; quadUV[2] = b; quadUV[3] = a;

				Vector vecOut = ToDrawSpaceDir( vecU * ( b.y - a.y ) + vecV * ( a.x - b.x ) );
				EmitPolygon( quadPos, quadUV, 4, vecOut, tex, lm );
			}
		}
	}
}

void C_ShatterGlass::DrawCracks( void )
{
	if ( !m_CrackMaterial.IsValid() )
		return;

	// Count the crack lines first; the mesh needs an exact primitive count
	int nLines = 0;
	for ( int i = 0; i < m_Pattern.Count(); i++ )
	{
		if ( IsBroken( i ) || !IsCracked( i ) )
			continue;

		const CShatterGlassPattern::Shard_t &shard = m_Pattern.Get( i );
		for ( int v = 0; v < shard.nVerts; v++ )
		{
			int nTag = shard.edgeTag[v];
			if ( nTag < 0 || IsBroken( nTag ) || ( IsCracked( nTag ) && nTag < i ) )
				continue;
			nLines++;
		}
	}

	if ( nLines == 0 )
		return;

	CMatRenderContextPtr pRenderContext( materials );
	pRenderContext->Bind( m_CrackMaterial, (IClientRenderable *)this );
	IMesh *pMesh = pRenderContext->GetDynamicMesh();

	float flMid = m_Panel.m_flThickness * 0.5f;
	float flHalfWidth = clamp( m_Pattern.CellSize() * 0.02f, 0.08f, 0.25f );

	const int nMaxQuadsPerBatch = 2048;
	CMeshBuilder meshBuilder;
	int nInBatch = 0;
	int nRemaining = nLines;
	meshBuilder.Begin( pMesh, MATERIAL_QUADS, MIN( nRemaining, nMaxQuadsPerBatch ) );

	for ( int i = 0; i < m_Pattern.Count(); i++ )
	{
		if ( IsBroken( i ) || !IsCracked( i ) )
			continue;

		const CShatterGlassPattern::Shard_t &shard = m_Pattern.Get( i );
		for ( int v = 0; v < shard.nVerts; v++ )
		{
			int nTag = shard.edgeTag[v];
			if ( nTag < 0 || IsBroken( nTag ) || ( IsCracked( nTag ) && nTag < i ) )
				continue;

			const Vector2D &a = shard.verts[v];
			const Vector2D &b = shard.verts[(v + 1) % shard.nVerts];

			Vector2D vecSide( a.y - b.y, b.x - a.x );
			Vector2DNormalize( vecSide );
			vecSide *= flHalfWidth;

			// Cracks between two cracked shards are brighter than cracks running into intact glass
			unsigned char nAlpha = IsCracked( nTag ) ? 110 : 60;

			Vector2D corners[4] = { a - vecSide, b - vecSide, b + vecSide, a + vecSide };
			for ( int k = 0; k < 4; k++ )
			{
				Vector vecLocal = m_Panel.LocalPoint( corners[k].x, corners[k].y, flMid );
				Vector vecPos = vecLocal;
				if ( !m_bDrawSpaceIsLocal )
					VectorTransform( vecLocal, m_matRender, vecPos );

				meshBuilder.Position3fv( vecPos.Base() );
				meshBuilder.Color4ub( 255, 255, 255, nAlpha );
				meshBuilder.TexCoord2f( 0, 0.5f, 0.5f );
				meshBuilder.AdvanceVertex();
			}

			nInBatch++;
			nRemaining--;
			if ( nInBatch >= nMaxQuadsPerBatch && nRemaining > 0 )
			{
				meshBuilder.End();
				pMesh->Draw();
				pMesh = pRenderContext->GetDynamicMesh();
				meshBuilder.Begin( pMesh, MATERIAL_QUADS, MIN( nRemaining, nMaxQuadsPerBatch ) );
				nInBatch = 0;
			}
		}
	}

	meshBuilder.End();
	pMesh->Draw();
}
