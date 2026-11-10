# vrad15_bake

64-bit helper that replaces VRAD's patch radiosity with path-traced bounces when
VRAD runs with `-pathtrace`. Direct lighting, props and BSP output stay in VRAD.

## Build and install
1. Build the helper: `build_windows.ps1 -BinDir <folder holding vrad.exe>`
2. Rebuild `vrad_dll` from `maptools.sln` (run `createmaptoolsprojects.bat` first,
   because `vrad_dll.vpc` gained a file) and copy `vrad_dll.dll` to that same folder.

## Use
`vrad -pathtrace -both -game <game> <map>`

| Option | Default | Meaning |
|---|---|---|
| `-pathsamples N` | 256 | Rays per lightmap sample |
| `-pathbounces N` | 8 | Maximum bounces |
| `-pathnodenoise` | off | Skip denoising |

`-mpi` is not supported. Without vis data VRAD does direct lighting only, as stock does.

## Known limits
- Static props block bounced light but do not reflect it, the same as stock VRAD.
- Alpha-tested prop shadows are ignored for bounces, also the same as stock.
- `func_brush` and similar entities without `vrad_brush_cast_shadows` do not block or
  reflect bounced light, because they are not in VRAD's ray tracing scene.
- Bounce light from light styles other than 0 is not computed, as in stock.

## Tests
`test_bake.py <path to vrad15_bake>` checks the helper against scenes with known answers.

## Third-party
Embree and Open Image Denoise are Apache 2.0. Include their license files if you
redistribute the DLLs.
