//=================================================================================================
//
// VPhysics Box3D - IPhysicsSurfaceProps (scripts/surfaceproperties*.txt)
//
//=================================================================================================

#ifndef B3VP_SURFACEPROPS_H
#define B3VP_SURFACEPROPS_H

#ifdef _WIN32
#pragma once
#endif

#include "b3vp_base.h"

class CBox3DPhysicsSurfaceProps : public IPhysicsSurfaceProps
{
public:
	CBox3DPhysicsSurfaceProps();

	static CBox3DPhysicsSurfaceProps &GetInstance() { return s_Instance; }

	int				ParseSurfaceData( const char *pFilename, const char *pTextfile );
	int				SurfacePropCount( void ) const;
	int				GetSurfaceIndex( const char *pSurfacePropName ) const;
	void			GetPhysicsProperties( int surfaceDataIndex, float *density, float *thickness, float *friction, float *elasticity ) const;
	surfacedata_t	*GetSurfaceData( int surfaceDataIndex );
	const char		*GetString( unsigned short stringTableIndex ) const;
	const char		*GetPropName( int surfaceDataIndex ) const;
	void			SetWorldMaterialIndexTable( int *pMapArray, int mapSize );
	void			GetPhysicsParameters( int surfaceDataIndex, surfacephysicsparams_t *pParamsOut ) const;

	// Maps a 7-bit material index stored in world (BSP) collision data to a surfaceprop index
	int				RemapWorldMaterial( int materialIndex7bits ) const;

	// Friction / restitution Box3D should use for a surface
	float			GetFriction( int surfaceDataIndex ) const;
	float			GetElasticity( int surfaceDataIndex ) const;
	float			GetDensity( int surfaceDataIndex ) const;

private:
	int				ClampIndex( int index ) const;
	unsigned short	AddString( const char *pString );

	static CBox3DPhysicsSurfaceProps s_Instance;

	std::vector< surfacedata_t >			m_Props;
	std::vector< std::string >				m_Names;
	std::map< std::string, int >			m_NameToIndex;		// lowercase names
	std::vector< std::string >				m_Strings;
	std::map< std::string, unsigned short >	m_StringToIndex;
	int										m_WorldMaterials[128];
	bool									m_bHasWorldMaterials;
};

#endif // B3VP_SURFACEPROPS_H
