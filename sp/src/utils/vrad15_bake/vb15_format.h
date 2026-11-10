// vb15_format.h - exchange format between 32-bit VRAD and the 64-bit bake helper.
// Plain C layout, little-endian, written and read sequentially. Keep it free of
// engine types: both programs include this file.
#ifndef VB15_FORMAT_H
#define VB15_FORMAT_H

#define VB15_SCENE_MAGIC  0x4E435342u // "BSCN"
#define VB15_RESULT_MAGIC 0x53455242u // "BRES"
#define VB15_VERSION      1u

enum { VB15_TRI_WORLD = 0, VB15_TRI_SKY = 1, VB15_TRI_PROP = 2 };

enum
{
	VB15_SAMPLE_BUMPED  = 1, // bump[] is valid and bumped results are wanted
	VB15_SAMPLE_INVALID = 2  // sample is inside solid: do not trace, fill from neighbours
};

enum { VB15_OPT_DENOISE = 1 };

#pragma pack(push, 1)
struct Vb15SceneHeader
{
	unsigned int magic, version;
	unsigned int numTris, numSamples;
	unsigned int raysPerSample, maxBounces;
	unsigned int options;
	int          threads; // <= 0 means all hardware threads
	unsigned int seed;
};

struct Vb15Tri
{
	float        v[9];
	unsigned int kind;
};

struct Vb15Sample
{
	float        pos[3];
	float        normal[3];  // hemisphere axis; matches the normal stock radiosity gathers about
	float        bump[3][3]; // bumped lightmap basis, world space
	float        direct[3];  // style 0 flat direct lighting, in lightmap units
	float        albedo[3];
	float        radius;     // approximate luxel spacing, world units
	unsigned int face;
	int          s, t;       // luxel coordinates on the face
	unsigned int flags;
};

struct Vb15ResultHeader
{
	unsigned int magic, version;
	unsigned int numSamples;
};

struct Vb15Result
{
	float light[4][3]; // indirect only: flat, then bump 0..2
};
#pragma pack(pop)

#endif
