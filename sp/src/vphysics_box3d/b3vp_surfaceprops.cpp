//=================================================================================================
//
// VPhysics Box3D - IPhysicsSurfaceProps
//
//=================================================================================================

#include "b3vp_base.h"
#include "b3vp_surfaceprops.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

CBox3DPhysicsSurfaceProps CBox3DPhysicsSurfaceProps::s_Instance;
EXPOSE_SINGLE_INTERFACE_GLOBALVAR( CBox3DPhysicsSurfaceProps, IPhysicsSurfaceProps, VPHYSICS_SURFACEPROPS_INTERFACE_VERSION, CBox3DPhysicsSurfaceProps::GetInstance() );

static std::string ToLower( const char *p )
{
	std::string s( p ? p : "" );
	for ( size_t i = 0; i < s.size(); i++ )
		s[i] = (char)tolower( (unsigned char)s[i] );
	return s;
}

CBox3DPhysicsSurfaceProps::CBox3DPhysicsSurfaceProps()
	: m_bHasWorldMaterials( false )
{
	// String 0 is the empty string, the game treats it as "no sound"
	m_Strings.push_back( "" );
	m_StringToIndex[""] = 0;

	// Index 0 is always "default"
	surfacedata_t def;
	memset( &def, 0, sizeof( def ) );
	def.physics.friction = 0.8f;
	def.physics.elasticity = 0.25f;
	def.physics.density = 2000.0f;
	def.physics.thickness = 0.0f;
	def.physics.dampening = 0.0f;
	def.audio.reflectivity = 0.66f;
	def.audio.hardnessFactor = 1.0f;
	def.audio.roughnessFactor = 1.0f;
	def.audio.roughThreshold = 0.5f;
	def.audio.hardThreshold = 0.5f;
	def.audio.hardVelocityThreshold = 0.0f;
	def.game.maxSpeedFactor = 1.0f;
	def.game.jumpFactor = 1.0f;
	def.game.material = 'C';
	m_Props.push_back( def );
	m_Names.push_back( "default" );
	m_NameToIndex["default"] = 0;

	for ( int i = 0; i < 128; i++ )
		m_WorldMaterials[i] = 0;
}

unsigned short CBox3DPhysicsSurfaceProps::AddString( const char *pString )
{
	std::map< std::string, unsigned short >::const_iterator it = m_StringToIndex.find( pString );
	if ( it != m_StringToIndex.end() )
		return it->second;

	const unsigned short index = (unsigned short)m_Strings.size();
	m_Strings.push_back( pString );
	m_StringToIndex[pString] = index;
	return index;
}

// A minimal tokenizer for the surfaceproperties format
static const char *NextToken( const char *p, std::string &out, bool &bBrace )
{
	out.clear();
	bBrace = false;
	for ( ;; )
	{
		while ( *p && ( (unsigned char)*p <= ' ' ) )
			p++;
		if ( p[0] == '/' && p[1] == '/' )
		{
			while ( *p && *p != '\n' )
				p++;
			continue;
		}
		break;
	}

	if ( !*p )
		return NULL;

	if ( *p == '{' || *p == '}' )
	{
		out = *p;
		bBrace = true;
		return p + 1;
	}

	if ( *p == '"' )
	{
		p++;
		while ( *p && *p != '"' )
			out += *p++;
		if ( *p == '"' )
			p++;
		return p;
	}

	while ( *p && (unsigned char)*p > ' ' && *p != '{' && *p != '}' && *p != '"' )
		out += *p++;
	return p;
}

int CBox3DPhysicsSurfaceProps::ParseSurfaceData( const char *pFilename, const char *pTextfile )
{
	if ( !pTextfile )
		return (int)m_Props.size();

	const char *p = pTextfile;
	std::string token, key, value;
	bool bBrace = false;

	while ( p )
	{
		// Surface name
		p = NextToken( p, token, bBrace );
		if ( !p )
			break;
		if ( bBrace )
			continue;

		const std::string name = token;

		p = NextToken( p, token, bBrace );
		if ( !p || !bBrace || token != "{" )
			break;

		const std::string lowerName = ToLower( name.c_str() );

		// Redefinitions update the existing entry, new ones start from "default"
		surfacedata_t data;
		int existing = -1;
		std::map< std::string, int >::const_iterator it = m_NameToIndex.find( lowerName );
		if ( it != m_NameToIndex.end() )
		{
			existing = it->second;
			data = m_Props[existing];
		}
		else
		{
			data = m_Props[0];
		}

		for ( ;; )
		{
			p = NextToken( p, key, bBrace );
			if ( !p || ( bBrace && key == "}" ) )
				break;
			if ( bBrace )
				continue;

			p = NextToken( p, value, bBrace );
			if ( !p )
				break;
			if ( bBrace )
			{
				if ( value == "}" )
					break;
				continue;
			}

			const char *k = key.c_str();
			const float f = (float)atof( value.c_str() );

			if ( !Q_stricmp( k, "base" ) )
			{
				const int baseIndex = GetSurfaceIndex( value.c_str() );
				if ( baseIndex >= 0 )
					data = m_Props[baseIndex];
			}
			else if ( !Q_stricmp( k, "friction" ) )					data.physics.friction = f;
			else if ( !Q_stricmp( k, "elasticity" ) )				data.physics.elasticity = f;
			else if ( !Q_stricmp( k, "density" ) )					data.physics.density = f;
			else if ( !Q_stricmp( k, "thickness" ) )				data.physics.thickness = f;
			else if ( !Q_stricmp( k, "dampening" ) )				data.physics.dampening = f;
			else if ( !Q_stricmp( k, "audioreflectivity" ) )		data.audio.reflectivity = f;
			else if ( !Q_stricmp( k, "audiohardnessfactor" ) )		data.audio.hardnessFactor = f;
			else if ( !Q_stricmp( k, "audioroughnessfactor" ) )		data.audio.roughnessFactor = f;
			else if ( !Q_stricmp( k, "scraperoughthreshold" ) )		data.audio.roughThreshold = f;
			else if ( !Q_stricmp( k, "impacthardthreshold" ) )		data.audio.hardThreshold = f;
			else if ( !Q_stricmp( k, "audiohardminvelocity" ) )		data.audio.hardVelocityThreshold = f;
			else if ( !Q_stricmp( k, "stepleft" ) )					data.sounds.stepleft = AddString( value.c_str() );
			else if ( !Q_stricmp( k, "stepright" ) )				data.sounds.stepright = AddString( value.c_str() );
			else if ( !Q_stricmp( k, "impactsoft" ) )				data.sounds.impactSoft = AddString( value.c_str() );
			else if ( !Q_stricmp( k, "impacthard" ) )				data.sounds.impactHard = AddString( value.c_str() );
			else if ( !Q_stricmp( k, "scrapesmooth" ) )				data.sounds.scrapeSmooth = AddString( value.c_str() );
			else if ( !Q_stricmp( k, "scraperough" ) )				data.sounds.scrapeRough = AddString( value.c_str() );
			else if ( !Q_stricmp( k, "bulletimpact" ) )				data.sounds.bulletImpact = AddString( value.c_str() );
			else if ( !Q_stricmp( k, "rolling" ) )					data.sounds.rolling = AddString( value.c_str() );
			else if ( !Q_stricmp( k, "break" ) )					data.sounds.breakSound = AddString( value.c_str() );
			else if ( !Q_stricmp( k, "strain" ) )					data.sounds.strainSound = AddString( value.c_str() );
			else if ( !Q_stricmp( k, "maxspeedfactor" ) )			data.game.maxSpeedFactor = f;
			else if ( !Q_stricmp( k, "jumpfactor" ) )				data.game.jumpFactor = f;
			else if ( !Q_stricmp( k, "climbable" ) )				data.game.climbable = (unsigned char)atoi( value.c_str() );
			else if ( !Q_stricmp( k, "gamematerial" ) )
			{
				// Either a single character ('C', 'M', ...) or a number
				if ( value.size() == 1 && !isdigit( (unsigned char)value[0] ) )
					data.game.material = (unsigned short)toupper( (unsigned char)value[0] );
				else
					data.game.material = (unsigned short)atoi( value.c_str() );
			}
		}

		if ( existing >= 0 )
		{
			m_Props[existing] = data;
		}
		else
		{
			m_NameToIndex[lowerName] = (int)m_Props.size();
			m_Props.push_back( data );
			m_Names.push_back( name );
		}
	}

	return (int)m_Props.size();
}

int CBox3DPhysicsSurfaceProps::SurfacePropCount( void ) const
{
	return (int)m_Props.size();
}

int CBox3DPhysicsSurfaceProps::GetSurfaceIndex( const char *pSurfacePropName ) const
{
	if ( !pSurfacePropName )
		return -1;

	std::map< std::string, int >::const_iterator it = m_NameToIndex.find( ToLower( pSurfacePropName ) );
	if ( it != m_NameToIndex.end() )
		return it->second;

	return -1;
}

int CBox3DPhysicsSurfaceProps::ClampIndex( int index ) const
{
	if ( index < 0 || index >= (int)m_Props.size() )
		return 0;
	return index;
}

void CBox3DPhysicsSurfaceProps::GetPhysicsProperties( int surfaceDataIndex, float *density, float *thickness, float *friction, float *elasticity ) const
{
	const surfacedata_t &data = m_Props[ClampIndex( surfaceDataIndex )];
	if ( density )		*density = data.physics.density;
	if ( thickness )	*thickness = data.physics.thickness;
	if ( friction )		*friction = data.physics.friction;
	if ( elasticity )	*elasticity = data.physics.elasticity;
}

surfacedata_t *CBox3DPhysicsSurfaceProps::GetSurfaceData( int surfaceDataIndex )
{
	return &m_Props[ClampIndex( surfaceDataIndex )];
}

const char *CBox3DPhysicsSurfaceProps::GetString( unsigned short stringTableIndex ) const
{
	if ( stringTableIndex >= m_Strings.size() )
		return NULL;
	return m_Strings[stringTableIndex].c_str();
}

const char *CBox3DPhysicsSurfaceProps::GetPropName( int surfaceDataIndex ) const
{
	if ( surfaceDataIndex < 0 || surfaceDataIndex >= (int)m_Names.size() )
		return NULL;
	return m_Names[surfaceDataIndex].c_str();
}

void CBox3DPhysicsSurfaceProps::SetWorldMaterialIndexTable( int *pMapArray, int mapSize )
{
	for ( int i = 0; i < 128; i++ )
		m_WorldMaterials[i] = ( pMapArray && i < mapSize ) ? MAX( 0, pMapArray[i] ) : 0;
	m_bHasWorldMaterials = pMapArray != NULL;
}

void CBox3DPhysicsSurfaceProps::GetPhysicsParameters( int surfaceDataIndex, surfacephysicsparams_t *pParamsOut ) const
{
	if ( pParamsOut )
		*pParamsOut = m_Props[ClampIndex( surfaceDataIndex )].physics;
}

int CBox3DPhysicsSurfaceProps::RemapWorldMaterial( int materialIndex7bits ) const
{
	if ( !m_bHasWorldMaterials || materialIndex7bits <= 0 || materialIndex7bits >= 128 )
		return -1;
	return m_WorldMaterials[materialIndex7bits];
}

float CBox3DPhysicsSurfaceProps::GetFriction( int surfaceDataIndex ) const
{
	return m_Props[ClampIndex( surfaceDataIndex )].physics.friction;
}

float CBox3DPhysicsSurfaceProps::GetElasticity( int surfaceDataIndex ) const
{
	return m_Props[ClampIndex( surfaceDataIndex )].physics.elasticity;
}

float CBox3DPhysicsSurfaceProps::GetDensity( int surfaceDataIndex ) const
{
	return m_Props[ClampIndex( surfaceDataIndex )].physics.density;
}
