//========= Copyright Valve Corporation, All rights reserved. ============//


#if defined( _X360 )

void GetBaseTextureAndNormal( sampler base, sampler base2, sampler bump, bool bBase2, bool bBump, float3 coords, float3 vWeights,
								out float4 vResultBase, out float4 vResultBase2, out float4 vResultBump )
{
	vResultBase = 0;
	vResultBase2 = 0;
	vResultBump = 0;

	if ( !bBump )
	{
		vResultBump = float4(0, 0, 1, 1);
	}

#if SEAMLESS

	vWeights = max( vWeights - 0.3, 0 );

	vWeights *= 1.0f / dot( vWeights, float3(1,1,1) );

	[branch]
	if (vWeights.x > 0)
	{
		vResultBase  += vWeights.x * tex2D( base,  coords.zy );

		if ( bBase2 )
		{
			vResultBase2 += vWeights.x * tex2D( base2, coords.zy );
		}

		if ( bBump )
		{
			vResultBump  += vWeights.x * tex2D( bump,  coords.zy );
		}
	}

	[branch]
	if (vWeights.y > 0)
	{
		vResultBase  += vWeights.y * tex2D( base,  coords.xz );

		if ( bBase2 )
		{
			vResultBase2 += vWeights.y * tex2D( base2, coords.xz );
		}
		if ( bBump )
		{
			vResultBump  += vWeights.y * tex2D( bump,  coords.xz );
		}
	}

	[branch]
	if (vWeights.z > 0)
	{
		vResultBase  += vWeights.z * tex2D( base,  coords.xy );
		if ( bBase2 )
		{
			vResultBase2 += vWeights.z * tex2D( base2, coords.xy );
		}

		if ( bBump )
		{
			vResultBump  += vWeights.z * tex2D( bump,  coords.xy );
		}
	}

#else  // not seamless

	vResultBase = tex2D( base, coords.xy );

	if ( bBase2 )
	{
		vResultBase2 = tex2D( base2, coords.xy );
	}

	if ( bBump )
	{
		vResultBump  = tex2D( bump, coords.xy );
	}

#endif


}

#else // PC

void GetBaseTextureAndNormal( sampler base, sampler base2, sampler bump, bool bBase2, bool bBump, float3 coords, float3 vWeights,
							 out float4 vResultBase, out float4 vResultBase2, out float4 vResultBump )
{
	vResultBase = 0;
	vResultBase2 = 0;
	vResultBump = 0;

	if ( !bBump )
	{
		vResultBump = float4(0, 0, 1, 1);
	}

#if SEAMLESS

	vResultBase  += vWeights.x * tex2D( base, coords.zy );
	if ( bBase2 )
	{
		vResultBase2 += vWeights.x * tex2D( base2, coords.zy );
	}
	if ( bBump )
	{
		vResultBump  += vWeights.x * tex2D( bump, coords.zy );
	}

	vResultBase  += vWeights.y * tex2D( base, coords.xz );
	if ( bBase2 )
	{
		vResultBase2 += vWeights.y * tex2D( base2, coords.xz );
	}
	if ( bBump )
	{
		vResultBump  += vWeights.y * tex2D( bump, coords.xz );
	}

	vResultBase  += vWeights.z * tex2D( base, coords.xy );
	if ( bBase2 )
	{
		vResultBase2 += vWeights.z * tex2D( base2, coords.xy );
	}
	if ( bBump )
	{
		vResultBump  += vWeights.z * tex2D( bump, coords.xy );
	}

#else  // not seamless

	vResultBase  = tex2D( base, coords.xy );
	if ( bBase2 )
	{
		vResultBase2 = tex2D( base2, coords.xy );
	}
	if ( bBump )
	{
		vResultBump  = tex2D( bump, coords.xy );
	}
#endif

}

#endif




#if defined( SHADER_MODEL_PS_3_0 )
// Cubic B-spline lightmap filtering, built from four bilinear fetches
// (GPU Gems 2, chapter 20). vTexSize is ( 1/width, 1/height, width, height ) of the
// lightmap page. The filter reads one luxel beyond the bilinear footprint, so the map
// must be compiled with VRAD's -bicubiclightmaps, which pads each lightmap by one luxel.
float3 LightMapSampleBicubic( sampler LightmapSampler, float2 vTexCoord, float4 vTexSize )
{
	float2 p = vTexCoord * vTexSize.zw - 0.5f;
	float2 f = frac( p );
	float2 cell = p - f;

	float2 f2 = f * f;
	float2 f3 = f2 * f;
	float2 w0 = ( 1.0f - 3.0f * f + 3.0f * f2 - f3 ) * ( 1.0f / 6.0f );
	float2 w1 = ( 4.0f - 6.0f * f2 + 3.0f * f3 ) * ( 1.0f / 6.0f );
	float2 w2 = ( 1.0f + 3.0f * f + 3.0f * f2 - 3.0f * f3 ) * ( 1.0f / 6.0f );
	float2 w3 = f3 * ( 1.0f / 6.0f );

	// g0 and g1 are never below 1/6, so the divisions are safe.
	float2 g0 = w0 + w1;
	float2 g1 = w2 + w3;
	float2 lo = ( cell - 0.5f + w1 / g0 ) * vTexSize.xy;
	float2 hi = ( cell + 1.5f + w3 / g1 ) * vTexSize.xy;

	// tex2Dlod, because this runs under a dynamic branch. Lightmaps have no mips.
	float3 s00 = tex2Dlod( LightmapSampler, float4( lo.x, lo.y, 0, 0 ) ).rgb;
	float3 s10 = tex2Dlod( LightmapSampler, float4( hi.x, lo.y, 0, 0 ) ).rgb;
	float3 s01 = tex2Dlod( LightmapSampler, float4( lo.x, hi.y, 0, 0 ) ).rgb;
	float3 s11 = tex2Dlod( LightmapSampler, float4( hi.x, hi.y, 0, 0 ) ).rgb;

	return g0.y * ( g0.x * s00 + g1.x * s10 ) + g1.y * ( g0.x * s01 + g1.x * s11 );
}
#endif

float3 LightMapSample( sampler LightmapSampler, float2 vTexCoord )
{
#	if ( !defined( _X360 ) || !defined( USE_32BIT_LIGHTMAPS_ON_360 ) )
	{
		float3 sample = tex2D( LightmapSampler, vTexCoord );

		return sample;
	}
#	else
	{
#		if 0 //1 for cheap sampling, 0 for accurate scaling from the individual samples
		{
			float4 sample = tex2D( LightmapSampler, vTexCoord );

			return sample.rgb * sample.a;
		}
#		else
		{
			float4 Weights;
			float4 samples_0; //no arrays allowed in inline assembly
			float4 samples_1;
			float4 samples_2;
			float4 samples_3;
			
			asm {
				tfetch2D samples_0, vTexCoord.xy, LightmapSampler, OffsetX = -0.5, OffsetY = -0.5, MinFilter=point, MagFilter=point, MipFilter=keep, UseComputedLOD=false
				tfetch2D samples_1, vTexCoord.xy, LightmapSampler, OffsetX =  0.5, OffsetY = -0.5, MinFilter=point, MagFilter=point, MipFilter=keep, UseComputedLOD=false
				tfetch2D samples_2, vTexCoord.xy, LightmapSampler, OffsetX = -0.5, OffsetY =  0.5, MinFilter=point, MagFilter=point, MipFilter=keep, UseComputedLOD=false
				tfetch2D samples_3, vTexCoord.xy, LightmapSampler, OffsetX =  0.5, OffsetY =  0.5, MinFilter=point, MagFilter=point, MipFilter=keep, UseComputedLOD=false

				getWeights2D Weights, vTexCoord.xy, LightmapSampler
			};

			Weights = float4( (1-Weights.x)*(1-Weights.y), Weights.x*(1-Weights.y), (1-Weights.x)*Weights.y, Weights.x*Weights.y );

			float3 result;
			result.rgb  = samples_0.rgb * (samples_0.a * Weights.x);
			result.rgb += samples_1.rgb * (samples_1.a * Weights.y);
			result.rgb += samples_2.rgb * (samples_2.a * Weights.z);
			result.rgb += samples_3.rgb * (samples_3.a * Weights.w);
		
			return result;
		}
#		endif
	}
#	endif
}

