//=================================================================================================
//
// VPhysics Box3D - IVPhysicsKeyParser (text blocks that follow the solids in .phy files,
// vehicle scripts, etc.)
//
//=================================================================================================

#ifndef B3VP_PARSE_H
#define B3VP_PARSE_H

#ifdef _WIN32
#pragma once
#endif

class IVPhysicsKeyParser;

IVPhysicsKeyParser *B3VP_CreateKeyParser( const char *pKeyData );
void B3VP_DestroyKeyParser( IVPhysicsKeyParser *pParser );

#endif // B3VP_PARSE_H
