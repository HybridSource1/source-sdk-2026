//=================================================================================================
//
// VPhysics Box3D - IVPhysicsKeyParser
//
// The format is a sequence of named blocks of quoted key/value pairs, with optional nested
// blocks (vehicle scripts):
//
//     solid { "index" "0" "name" "box" "mass" "10" }
//     vehicle { "wheelsperaxle" "2" "axle" { "wheel" { "radius" "16" } } }
//
//=================================================================================================

#include "b3vp_base.h"
#include "b3vp_parse.h"
#include "b3vp_surfaceprops.h"

// memdbgon must be the last include file in a .cpp file!!!
#include "tier0/memdbgon.h"

//-------------------------------------------------------------------------------------------------
// A tiny parse tree
//-------------------------------------------------------------------------------------------------

struct B3VPKeyNode_t
{
	std::string							key;
	std::string							value;
	bool								bBlock;
	std::vector< B3VPKeyNode_t * >		children;

	B3VPKeyNode_t() : bBlock( false ) {}
	~B3VPKeyNode_t()
	{
		for ( size_t i = 0; i < children.size(); i++ )
			delete children[i];
	}
};

class CB3VPTokenizer
{
public:
	CB3VPTokenizer( const char *pText ) : m_p( pText ? pText : "" ) {}

	enum TokenType_t { TOKEN_EOF, TOKEN_STRING, TOKEN_OPEN, TOKEN_CLOSE };

	TokenType_t Next( std::string &out )
	{
		out.clear();
		SkipWhitespaceAndComments();

		if ( !*m_p )
			return TOKEN_EOF;

		if ( *m_p == '{' ) { m_p++; return TOKEN_OPEN; }
		if ( *m_p == '}' ) { m_p++; return TOKEN_CLOSE; }

		if ( *m_p == '"' )
		{
			m_p++;
			while ( *m_p && *m_p != '"' )
			{
				out += *m_p;
				m_p++;
			}
			if ( *m_p == '"' )
				m_p++;
			return TOKEN_STRING;
		}

		while ( *m_p && !IsSpace( *m_p ) && *m_p != '{' && *m_p != '}' && *m_p != '"' )
		{
			out += *m_p;
			m_p++;
		}
		return TOKEN_STRING;
	}

private:
	static bool IsSpace( char c ) { return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\0' || (unsigned char)c < 32; }

	void SkipWhitespaceAndComments()
	{
		for ( ;; )
		{
			while ( *m_p && IsSpace( *m_p ) )
				m_p++;

			if ( m_p[0] == '/' && m_p[1] == '/' )
			{
				while ( *m_p && *m_p != '\n' )
					m_p++;
				continue;
			}
			break;
		}
	}

	const char *m_p;
};

// Parses the contents of a block (after its '{') into pParent until the matching '}'
static void ParseBlockContents( CB3VPTokenizer &tok, B3VPKeyNode_t *pParent, int depth )
{
	std::string key, value;
	for ( ;; )
	{
		CB3VPTokenizer::TokenType_t t = tok.Next( key );
		if ( t == CB3VPTokenizer::TOKEN_EOF || t == CB3VPTokenizer::TOKEN_CLOSE )
			return;
		if ( t == CB3VPTokenizer::TOKEN_OPEN )
		{
			// Anonymous block, parse and drop
			B3VPKeyNode_t dummy;
			ParseBlockContents( tok, &dummy, depth + 1 );
			continue;
		}

		CB3VPTokenizer::TokenType_t t2 = tok.Next( value );
		if ( t2 == CB3VPTokenizer::TOKEN_EOF )
			return;

		B3VPKeyNode_t *pNode = new B3VPKeyNode_t;
		pNode->key = key;
		pParent->children.push_back( pNode );

		if ( t2 == CB3VPTokenizer::TOKEN_OPEN )
		{
			pNode->bBlock = true;
			if ( depth < 32 )
				ParseBlockContents( tok, pNode, depth + 1 );
		}
		else if ( t2 == CB3VPTokenizer::TOKEN_CLOSE )
		{
			// Key with no value right before the end of the block
			return;
		}
		else
		{
			pNode->value = value;
		}
	}
}

// Vehicle scripts give engine speeds in MPH, the game expects them in inches/second
#define B3VP_MPH_TO_INS		17.6f

//-------------------------------------------------------------------------------------------------
// Value helpers
//-------------------------------------------------------------------------------------------------

static inline bool KeyIs( const B3VPKeyNode_t *pNode, const char *pName )
{
	return !Q_stricmp( pNode->key.c_str(), pName );
}

static inline float ToFloat( const B3VPKeyNode_t *pNode )	{ return (float)atof( pNode->value.c_str() ); }
static inline int ToInt( const B3VPKeyNode_t *pNode )		{ return atoi( pNode->value.c_str() ); }
static inline bool ToBool( const B3VPKeyNode_t *pNode )
{
	const char *p = pNode->value.c_str();
	return !Q_stricmp( p, "true" ) || !Q_stricmp( p, "yes" ) || atoi( p ) != 0;
}

static inline void ToVector( const B3VPKeyNode_t *pNode, Vector &out )
{
	out.Init();
	sscanf( pNode->value.c_str(), "%f %f %f", &out.x, &out.y, &out.z );
}

static inline void ToVector4D( const B3VPKeyNode_t *pNode, Vector4D &out )
{
	out.Init();
	sscanf( pNode->value.c_str(), "%f %f %f %f", &out.x, &out.y, &out.z, &out.w );
}

static inline int ToSurfaceProp( const B3VPKeyNode_t *pNode )
{
	return MAX( 0, CBox3DPhysicsSurfaceProps::GetInstance().GetSurfaceIndex( pNode->value.c_str() ) );
}

// Sends a whole block to an unknown key handler, nested blocks first announce their name
static void SendBlockToHandler( const B3VPKeyNode_t *pBlock, void *pData, IVPhysicsKeyHandler *pHandler )
{
	if ( !pHandler )
		return;

	for ( size_t i = 0; i < pBlock->children.size(); i++ )
	{
		const B3VPKeyNode_t *pChild = pBlock->children[i];
		if ( pChild->bBlock )
		{
			pHandler->ParseKeyValue( pData, pChild->key.c_str(), "" );
			SendBlockToHandler( pChild, pData, pHandler );
		}
		else
		{
			pHandler->ParseKeyValue( pData, pChild->key.c_str(), pChild->value.c_str() );
		}
	}
}

//-------------------------------------------------------------------------------------------------
// The parser
//-------------------------------------------------------------------------------------------------

class CBox3DKeyParser : public IVPhysicsKeyParser
{
public:
	CBox3DKeyParser( const char *pKeyData )
		: m_nCurrent( 0 )
	{
		CB3VPTokenizer tok( pKeyData );
		std::string name, next;
		for ( ;; )
		{
			CB3VPTokenizer::TokenType_t t = tok.Next( name );
			if ( t == CB3VPTokenizer::TOKEN_EOF )
				break;
			if ( t != CB3VPTokenizer::TOKEN_STRING )
				continue;

			CB3VPTokenizer::TokenType_t t2 = tok.Next( next );
			if ( t2 == CB3VPTokenizer::TOKEN_EOF )
				break;
			if ( t2 != CB3VPTokenizer::TOKEN_OPEN )
				continue; // stray key/value at the top level

			B3VPKeyNode_t *pBlock = new B3VPKeyNode_t;
			pBlock->key = name;
			pBlock->bBlock = true;
			ParseBlockContents( tok, pBlock, 1 );
			m_Blocks.push_back( pBlock );
		}
	}

	~CBox3DKeyParser()
	{
		for ( size_t i = 0; i < m_Blocks.size(); i++ )
			delete m_Blocks[i];
	}

	const char *GetCurrentBlockName( void )
	{
		if ( Finished() )
			return NULL;
		return m_Blocks[m_nCurrent]->key.c_str();
	}

	bool Finished( void )
	{
		return m_nCurrent >= m_Blocks.size();
	}

	void SkipBlock( void )
	{
		NextBlock();
	}

	void ParseSolid( solid_t *pSolid, IVPhysicsKeyHandler *unknownKeyHandler )
	{
		if ( unknownKeyHandler )
			unknownKeyHandler->SetDefaults( pSolid );
		else
			memset( pSolid, 0, sizeof( *pSolid ) );

		const B3VPKeyNode_t *pBlock = Current();
		if ( pBlock )
		{
			for ( size_t i = 0; i < pBlock->children.size(); i++ )
			{
				const B3VPKeyNode_t *pKey = pBlock->children[i];
				if ( KeyIs( pKey, "index" ) )					pSolid->index = ToInt( pKey );
				else if ( KeyIs( pKey, "name" ) )				Q_strncpy( pSolid->name, pKey->value.c_str(), sizeof( pSolid->name ) );
				else if ( KeyIs( pKey, "parent" ) )				Q_strncpy( pSolid->parent, pKey->value.c_str(), sizeof( pSolid->parent ) );
				else if ( KeyIs( pKey, "surfaceprop" ) )		Q_strncpy( pSolid->surfaceprop, pKey->value.c_str(), sizeof( pSolid->surfaceprop ) );
				else if ( KeyIs( pKey, "mass" ) )				pSolid->params.mass = ToFloat( pKey );
				else if ( KeyIs( pKey, "masscenteroverride" ) )
				{
					ToVector( pKey, pSolid->massCenterOverride );
					pSolid->params.massCenterOverride = &pSolid->massCenterOverride;
				}
				else if ( KeyIs( pKey, "inertia" ) )			pSolid->params.inertia = ToFloat( pKey );
				else if ( KeyIs( pKey, "damping" ) )			pSolid->params.damping = ToFloat( pKey );
				else if ( KeyIs( pKey, "rotdamping" ) )			pSolid->params.rotdamping = ToFloat( pKey );
				else if ( KeyIs( pKey, "drag" ) )				pSolid->params.dragCoefficient = ToFloat( pKey );
				else if ( KeyIs( pKey, "rotinertialimit" ) )	pSolid->params.rotInertiaLimit = ToFloat( pKey );
				else if ( KeyIs( pKey, "volume" ) )				pSolid->params.volume = ToFloat( pKey );
				else if ( KeyIs( pKey, "rollingdrag" ) )		{ /* not part of objectparams_t in this branch */ }
				else if ( unknownKeyHandler )
				{
					if ( pKey->bBlock )
						SendBlockToHandler( pKey, pSolid, unknownKeyHandler );
					else
						unknownKeyHandler->ParseKeyValue( pSolid, pKey->key.c_str(), pKey->value.c_str() );
				}
			}
		}

		NextBlock();
	}

	void ParseFluid( fluid_t *pFluid, IVPhysicsKeyHandler *unknownKeyHandler )
	{
		if ( unknownKeyHandler )
		{
			unknownKeyHandler->SetDefaults( pFluid );
		}
		else
		{
			memset( pFluid, 0, sizeof( *pFluid ) );
			Q_strncpy( pFluid->surfaceprop, "water", sizeof( pFluid->surfaceprop ) );
		}

		const B3VPKeyNode_t *pBlock = Current();
		if ( pBlock )
		{
			for ( size_t i = 0; i < pBlock->children.size(); i++ )
			{
				const B3VPKeyNode_t *pKey = pBlock->children[i];
				if ( KeyIs( pKey, "index" ) )					pFluid->index = ToInt( pKey );
				else if ( KeyIs( pKey, "surfaceprop" ) )		Q_strncpy( pFluid->surfaceprop, pKey->value.c_str(), sizeof( pFluid->surfaceprop ) );
				else if ( KeyIs( pKey, "damping" ) )			pFluid->params.damping = ToFloat( pKey );
				else if ( KeyIs( pKey, "surfaceplane" ) )		ToVector4D( pKey, pFluid->params.surfacePlane );
				else if ( KeyIs( pKey, "currentvelocity" ) )	ToVector( pKey, pFluid->params.currentVelocity );
				else if ( KeyIs( pKey, "contents" ) )			pFluid->params.contents = ToInt( pKey );
				else if ( unknownKeyHandler && !pKey->bBlock )
					unknownKeyHandler->ParseKeyValue( pFluid, pKey->key.c_str(), pKey->value.c_str() );
			}
		}

		NextBlock();
	}

	void ParseRagdollConstraint( constraint_ragdollparams_t *pConstraint, IVPhysicsKeyHandler *unknownKeyHandler )
	{
		if ( unknownKeyHandler )
		{
			unknownKeyHandler->SetDefaults( pConstraint );
		}
		else
		{
			memset( pConstraint, 0, sizeof( *pConstraint ) );
			pConstraint->childIndex = -1;
			pConstraint->parentIndex = -1;
		}

		// The text data specifies clockwise rotations
		pConstraint->useClockwiseRotations = true;

		const B3VPKeyNode_t *pBlock = Current();
		if ( pBlock )
		{
			for ( size_t i = 0; i < pBlock->children.size(); i++ )
			{
				const B3VPKeyNode_t *pKey = pBlock->children[i];
				if ( KeyIs( pKey, "parent" ) )				pConstraint->parentIndex = ToInt( pKey );
				else if ( KeyIs( pKey, "child" ) )			pConstraint->childIndex = ToInt( pKey );
				else if ( KeyIs( pKey, "xmin" ) )			pConstraint->axes[0].minRotation = ToFloat( pKey );
				else if ( KeyIs( pKey, "ymin" ) )			pConstraint->axes[1].minRotation = ToFloat( pKey );
				else if ( KeyIs( pKey, "zmin" ) )			pConstraint->axes[2].minRotation = ToFloat( pKey );
				else if ( KeyIs( pKey, "xmax" ) )			pConstraint->axes[0].maxRotation = ToFloat( pKey );
				else if ( KeyIs( pKey, "ymax" ) )			pConstraint->axes[1].maxRotation = ToFloat( pKey );
				else if ( KeyIs( pKey, "zmax" ) )			pConstraint->axes[2].maxRotation = ToFloat( pKey );
				else if ( KeyIs( pKey, "xfriction" ) )		{ pConstraint->axes[0].torque = ToFloat( pKey ); pConstraint->axes[0].angularVelocity = 0; }
				else if ( KeyIs( pKey, "yfriction" ) )		{ pConstraint->axes[1].torque = ToFloat( pKey ); pConstraint->axes[1].angularVelocity = 0; }
				else if ( KeyIs( pKey, "zfriction" ) )		{ pConstraint->axes[2].torque = ToFloat( pKey ); pConstraint->axes[2].angularVelocity = 0; }
				else if ( unknownKeyHandler && !pKey->bBlock )
					unknownKeyHandler->ParseKeyValue( pConstraint, pKey->key.c_str(), pKey->value.c_str() );
			}
		}

		NextBlock();
	}

	void ParseSurfaceTable( int *table, IVPhysicsKeyHandler *unknownKeyHandler )
	{
		const B3VPKeyNode_t *pBlock = Current();
		if ( pBlock && table )
		{
			for ( size_t i = 0; i < pBlock->children.size(); i++ )
			{
				const B3VPKeyNode_t *pKey = pBlock->children[i];
				const int tableIndex = ToInt( pKey );
				if ( tableIndex >= 0 && tableIndex < 128 )
					table[tableIndex] = CBox3DPhysicsSurfaceProps::GetInstance().GetSurfaceIndex( pKey->key.c_str() );
			}
		}

		NextBlock();
	}

	void ParseCustom( void *pCustom, IVPhysicsKeyHandler *unknownKeyHandler )
	{
		if ( unknownKeyHandler )
			unknownKeyHandler->SetDefaults( pCustom );

		const B3VPKeyNode_t *pBlock = Current();
		if ( pBlock )
			SendBlockToHandler( pBlock, pCustom, unknownKeyHandler );

		NextBlock();
	}

	void ParseVehicle( vehicleparams_t *pVehicle, IVPhysicsKeyHandler *unknownKeyHandler )
	{
		if ( unknownKeyHandler )
			unknownKeyHandler->SetDefaults( pVehicle );
		else
			memset( pVehicle, 0, sizeof( *pVehicle ) );

		const B3VPKeyNode_t *pBlock = Current();
		if ( pBlock )
		{
			for ( size_t i = 0; i < pBlock->children.size(); i++ )
			{
				const B3VPKeyNode_t *pKey = pBlock->children[i];
				if ( KeyIs( pKey, "wheelsperaxle" ) )		pVehicle->wheelsPerAxle = ToInt( pKey );
				else if ( KeyIs( pKey, "body" ) )			ParseVehicleBody( pKey, pVehicle->body );
				else if ( KeyIs( pKey, "engine" ) )			ParseVehicleEngine( pKey, pVehicle->engine );
				else if ( KeyIs( pKey, "steering" ) )		ParseVehicleSteering( pKey, pVehicle->steering );
				else if ( KeyIs( pKey, "axle" ) )
				{
					if ( pVehicle->axleCount < VEHICLE_MAX_AXLE_COUNT )
					{
						ParseVehicleAxle( pKey, pVehicle->axles[pVehicle->axleCount] );
						pVehicle->axleCount++;
					}
				}
				else if ( unknownKeyHandler && !pKey->bBlock )
					unknownKeyHandler->ParseKeyValue( pVehicle, pKey->key.c_str(), pKey->value.c_str() );
			}
		}

		NextBlock();
	}

private:
	const B3VPKeyNode_t *Current() const
	{
		return m_nCurrent < m_Blocks.size() ? m_Blocks[m_nCurrent] : NULL;
	}

	void NextBlock()
	{
		if ( m_nCurrent < m_Blocks.size() )
			m_nCurrent++;
	}

	static void ParseVehicleBody( const B3VPKeyNode_t *pBlock, vehicle_bodyparams_t &body )
	{
		for ( size_t i = 0; i < pBlock->children.size(); i++ )
		{
			const B3VPKeyNode_t *pKey = pBlock->children[i];
			if ( KeyIs( pKey, "masscenteroverride" ) )			ToVector( pKey, body.massCenterOverride );
			else if ( KeyIs( pKey, "massoverride" ) )			body.massOverride = ToFloat( pKey );
			else if ( KeyIs( pKey, "addgravity" ) )				body.addGravity = ToFloat( pKey );
			else if ( KeyIs( pKey, "tiltforce" ) )				body.tiltForce = ToFloat( pKey );
			else if ( KeyIs( pKey, "tiltforceheight" ) )		body.tiltForceHeight = ToFloat( pKey );
			else if ( KeyIs( pKey, "countertorquefactor" ) )	body.counterTorqueFactor = ToFloat( pKey );
			else if ( KeyIs( pKey, "keepuprighttorque" ) )		body.keepUprightTorque = ToFloat( pKey );
			else if ( KeyIs( pKey, "maxangularvelocity" ) )		body.maxAngularVelocity = ToFloat( pKey );
		}
	}

	static void ParseVehicleEngine( const B3VPKeyNode_t *pBlock, vehicle_engineparams_t &engine )
	{
		for ( size_t i = 0; i < pBlock->children.size(); i++ )
		{
			const B3VPKeyNode_t *pKey = pBlock->children[i];
			if ( KeyIs( pKey, "horsepower" ) )					engine.horsepower = ToFloat( pKey );
			else if ( KeyIs( pKey, "maxspeed" ) )				engine.maxSpeed = ToFloat( pKey ) * B3VP_MPH_TO_INS;
			else if ( KeyIs( pKey, "maxreversespeed" ) )		engine.maxRevSpeed = ToFloat( pKey ) * B3VP_MPH_TO_INS;
			else if ( KeyIs( pKey, "maxrpm" ) )					engine.maxRPM = ToFloat( pKey );
			else if ( KeyIs( pKey, "axleratio" ) )				engine.axleRatio = ToFloat( pKey );
			else if ( KeyIs( pKey, "throttletime" ) )			engine.throttleTime = ToFloat( pKey );
			else if ( KeyIs( pKey, "autotransmission" ) )		engine.isAutoTransmission = ToBool( pKey );
			else if ( KeyIs( pKey, "shiftuprpm" ) )				engine.shiftUpRPM = ToFloat( pKey );
			else if ( KeyIs( pKey, "shiftdownrpm" ) )			engine.shiftDownRPM = ToFloat( pKey );
			else if ( KeyIs( pKey, "autobrakespeedgain" ) )		engine.autobrakeSpeedGain = ToFloat( pKey );
			else if ( KeyIs( pKey, "autobrakespeedfactor" ) )	engine.autobrakeSpeedFactor = ToFloat( pKey );
			else if ( KeyIs( pKey, "gear" ) )
			{
				if ( engine.gearCount < VEHICLE_MAX_GEAR_COUNT )
					engine.gearRatio[engine.gearCount++] = ToFloat( pKey );
			}
			else if ( KeyIs( pKey, "boost" ) && pKey->bBlock )
			{
				for ( size_t j = 0; j < pKey->children.size(); j++ )
				{
					const B3VPKeyNode_t *pBoost = pKey->children[j];
					if ( KeyIs( pBoost, "force" ) )				engine.boostForce = ToFloat( pBoost );
					else if ( KeyIs( pBoost, "duration" ) )		engine.boostDuration = ToFloat( pBoost );
					else if ( KeyIs( pBoost, "delay" ) )		engine.boostDelay = ToFloat( pBoost );
					else if ( KeyIs( pBoost, "maxspeed" ) )		engine.boostMaxSpeed = ToFloat( pBoost ) * B3VP_MPH_TO_INS;
					else if ( KeyIs( pBoost, "torqueboost" ) )	engine.torqueBoost = ToBool( pBoost );
				}
			}
		}
	}

	static void ParseVehicleSteering( const B3VPKeyNode_t *pBlock, vehicle_steeringparams_t &steering )
	{
		for ( size_t i = 0; i < pBlock->children.size(); i++ )
		{
			const B3VPKeyNode_t *pKey = pBlock->children[i];
			if ( KeyIs( pKey, "degreesslow" ) )								steering.degreesSlow = ToFloat( pKey );
			else if ( KeyIs( pKey, "degreesfast" ) )						steering.degreesFast = ToFloat( pKey );
			else if ( KeyIs( pKey, "degreesboost" ) )						steering.degreesBoost = ToFloat( pKey );
			else if ( KeyIs( pKey, "slowcarspeed" ) )						steering.speedSlow = ToFloat( pKey );
			else if ( KeyIs( pKey, "fastcarspeed" ) )						steering.speedFast = ToFloat( pKey );
			else if ( KeyIs( pKey, "slowsteeringrate" ) )					steering.steeringRateSlow = ToFloat( pKey );
			else if ( KeyIs( pKey, "faststeeringrate" ) )					steering.steeringRateFast = ToFloat( pKey );
			else if ( KeyIs( pKey, "steeringrestrateslow" ) )				steering.steeringRestRateSlow = ToFloat( pKey );
			else if ( KeyIs( pKey, "steeringrestratefast" ) )				steering.steeringRestRateFast = ToFloat( pKey );
			else if ( KeyIs( pKey, "throttlesteeringrestratefactor" ) )		steering.throttleSteeringRestRateFactor = ToFloat( pKey );
			else if ( KeyIs( pKey, "booststeeringrestratefactor" ) )		steering.boostSteeringRestRateFactor = ToFloat( pKey );
			else if ( KeyIs( pKey, "booststeeringratefactor" ) )			steering.boostSteeringRateFactor = ToFloat( pKey );
			else if ( KeyIs( pKey, "steeringexponent" ) )					steering.steeringExponent = ToFloat( pKey );
			else if ( KeyIs( pKey, "turnthrottlereduceslow" ) )				steering.turnThrottleReduceSlow = ToFloat( pKey );
			else if ( KeyIs( pKey, "turnthrottlereducefast" ) )				steering.turnThrottleReduceFast = ToFloat( pKey );
			else if ( KeyIs( pKey, "brakesteeringratefactor" ) )			steering.brakeSteeringRateFactor = ToFloat( pKey );
			else if ( KeyIs( pKey, "powerslideaccel" ) )					steering.powerSlideAccel = ToFloat( pKey );
			else if ( KeyIs( pKey, "skidallowed" ) )						steering.isSkidAllowed = ToBool( pKey );
			else if ( KeyIs( pKey, "dustcloud" ) )							steering.dustCloud = ToBool( pKey );
		}
	}

	static void ParseVehicleAxle( const B3VPKeyNode_t *pBlock, vehicle_axleparams_t &axle )
	{
		for ( size_t i = 0; i < pBlock->children.size(); i++ )
		{
			const B3VPKeyNode_t *pKey = pBlock->children[i];
			if ( KeyIs( pKey, "offset" ) )							ToVector( pKey, axle.offset );
			else if ( KeyIs( pKey, "wheeloffset" ) )				ToVector( pKey, axle.wheelOffset );
			else if ( KeyIs( pKey, "raytracecenteroffset" ) )		ToVector( pKey, axle.raytraceCenterOffset );
			else if ( KeyIs( pKey, "raytraceoffset" ) )				ToVector( pKey, axle.raytraceOffset );
			else if ( KeyIs( pKey, "torquefactor" ) )				axle.torqueFactor = ToFloat( pKey );
			else if ( KeyIs( pKey, "brakefactor" ) )				axle.brakeFactor = ToFloat( pKey );
			else if ( KeyIs( pKey, "wheel" ) && pKey->bBlock )
			{
				for ( size_t j = 0; j < pKey->children.size(); j++ )
				{
					const B3VPKeyNode_t *pWheel = pKey->children[j];
					vehicle_wheelparams_t &wheel = axle.wheels;
					if ( KeyIs( pWheel, "radius" ) )							wheel.radius = ToFloat( pWheel );
					else if ( KeyIs( pWheel, "mass" ) )							wheel.mass = ToFloat( pWheel );
					else if ( KeyIs( pWheel, "inertia" ) || KeyIs( pWheel, "intertia" ) )	wheel.inertia = ToFloat( pWheel );
					else if ( KeyIs( pWheel, "damping" ) )						wheel.damping = ToFloat( pWheel );
					else if ( KeyIs( pWheel, "rotdamping" ) )					wheel.rotdamping = ToFloat( pWheel );
					else if ( KeyIs( pWheel, "frictionscale" ) )				wheel.frictionScale = ToFloat( pWheel );
					else if ( KeyIs( pWheel, "material" ) )						wheel.materialIndex = ToSurfaceProp( pWheel );
					else if ( KeyIs( pWheel, "skidmaterial" ) )					wheel.skidMaterialIndex = ToSurfaceProp( pWheel );
					else if ( KeyIs( pWheel, "brakematerial" ) )				wheel.brakeMaterialIndex = ToSurfaceProp( pWheel );
					else if ( KeyIs( pWheel, "springadditionallength" ) )		wheel.springAdditionalLength = ToFloat( pWheel );
				}
			}
			else if ( KeyIs( pKey, "suspension" ) && pKey->bBlock )
			{
				for ( size_t j = 0; j < pKey->children.size(); j++ )
				{
					const B3VPKeyNode_t *pSusp = pKey->children[j];
					vehicle_suspensionparams_t &susp = axle.suspension;
					if ( KeyIs( pSusp, "springconstant" ) )					susp.springConstant = ToFloat( pSusp );
					else if ( KeyIs( pSusp, "springdamping" ) )				susp.springDamping = ToFloat( pSusp );
					else if ( KeyIs( pSusp, "stabilizerconstant" ) )		susp.stabilizerConstant = ToFloat( pSusp );
					else if ( KeyIs( pSusp, "springdampingcompression" ) )	susp.springDampingCompression = ToFloat( pSusp );
					else if ( KeyIs( pSusp, "maxbodyforce" ) )				susp.maxBodyForce = ToFloat( pSusp );
				}
			}
		}
	}

	std::vector< B3VPKeyNode_t * >	m_Blocks;
	size_t							m_nCurrent;
};

IVPhysicsKeyParser *B3VP_CreateKeyParser( const char *pKeyData )
{
	return new CBox3DKeyParser( pKeyData );
}

void B3VP_DestroyKeyParser( IVPhysicsKeyParser *pParser )
{
	delete static_cast< CBox3DKeyParser * >( pParser );
}
