# Oafish Grass Runtime (OGR) v0.2.0

Standalone WED-authored animated grass runtime for X-Plane 11/12.

OGR is deliberately separate from SSA. WED authors the grass area; OGR reads the exported scenery and draws/animates the real grass instances.

## What OGR keeps from SSA Grass

- X-Plane weather wind direction and speed
- actual propeller RPM + thrust response for prop aircraft
- N1/throttle/thrust response for jet aircraft
- per-engine prop wash / jet blast influence
- engine position and height attenuation
- camera/world-relative active grass ring
- 3000 ft (914.4 m) grass LOD
- optional aircraft AGL cutoff at 3000 ft
- random A/B/C grass model variants
- near animated / far static optimization
- no-shadow OBJ assets
- upward normals to avoid black grass cards
- strict WED polygon edge margin so grass cards do not spill onto asphalt

## New in v0.2.0: direct WED DSF reading

The normal workflow no longer requires `BUILD_GRASS_AREAS.cmd`.

After WED exports the scenery pack, OGR scans the pack's exported `Earth nav data/**/*.dsf`, finds forest polygons whose resource is exactly `OafishGrass/ogr_grass.for`, decodes their polygon windings/holes and WED density, then feeds those areas into the existing proven runtime automatically.

WED encodes forest density in the polygon parameter and OGR decodes that value directly. v0.2 reads WED **Area** forest placements; line/point forest modes are ignored.

The old `ogr_areas.json` format remains as an internal compatibility hand-off to the stable runtime. OGR regenerates it automatically from DSF when direct parsing succeeds. `BUILD_GRASS_AREAS.cmd` remains in the starter pack only as an optional fallback for compressed/unsupported DSFs.

### Direct DSF compatibility

- uncompressed DSF master version 1: direct reader enabled
- normal WED custom scenery export: intended v0.2 path
- 7z-compressed DSF: detected and left to the legacy JSON/CMD fallback
- parser failure: existing legacy `ogr_areas.json` is preserved rather than overwritten

## Replay-aware grass effects

OGR keeps the v0.1.1 replay system. It records a lightweight 20 Hz history for up to about 20 minutes containing:

- per-engine wash power and RPM ratio
- engine position relative to the aircraft
- reverse/forward wake heading
- weather wind vector
- aircraft transform used to align replay samples

When `sim/time/is_in_replay` is active, OGR restores the recorded engine wash and wind state, so prop wash, jet blast and weather wind remain visible while watching or scrubbing X-Plane Replay.

## WED workflow

1. Install `OafishGrassRuntime.xpl` in `X-Plane 11/Resources/plugins/OafishGrassRuntime/win_x64/`.
2. Copy `WED-Starter/OafishGrass` into the scenery pack, for example `Custom Scenery/WIDN Tanjung Pinang/OafishGrass/`.
3. Open the scenery in WED.
4. Select the **Forest** polygon tool and choose `OafishGrass/ogr_grass.for`.
5. Draw the exact grass area. Holes and WED forest density are supported.
6. **Save and Export Scenery Pack** in WED.
7. Start X-Plane, or use `Plugins > Oafish Grass Runtime > Reload WED/DSF grass areas`.

That is the normal v0.2 workflow. No Python/CMD step is required after a normal uncompressed WED export.

The `.for` file remains a valid XP11 forest resource for WED authoring. Lines beginning with `#OGR_` are OGR metadata comments used for models and runtime tuning.

## OGR extension metadata

Edit `OafishGrass/ogr_grass.for` to tune a scenery pack. Examples:

```text
#OGR_MODEL grass_A.obj
#OGR_MODEL grass_B.obj
#OGR_MODEL grass_C.obj
#OGR_TILE_SIZE_M 2.4
#OGR_DRAW_DISTANCE_M 914.4
#OGR_ANIMATED_DISTANCE_M 600.0
#OGR_BOUNDARY_MARGIN_M 1.4
#OGR_WIND_FULL_BEND_KT 45.0
#OGR_ENGINE_WASH 1
#OGR_ENGINE_WASH_STRENGTH 1.3
#OGR_ENGINE_WASH_RANGE_M 35.0
#OGR_MAX_ACTIVE_TILES 1600
#OGR_MAX_TOTAL_TILES 8000
#OGR_HIDE_AIRCRAFT_AGL_FT 3000
```

`DRAW_DISTANCE_M` is metric because X-Plane object LOD is metric. `914.4 m = 3000 ft`.

## Performance profile

The default profile stays conservative for older iGPU systems:

- at most 1600 active instances around the camera
- at most 8000 candidate placements per scenery pack
- animation at 20 Hz (`0.05 s`)
- active-set refresh every `0.35 s`
- far grass stops receiving animation updates
- replay stores engine/environment inputs instead of per-blade state
- DSF parsing happens on load/reload, not every frame

## Build

Requires CMake 3.21+, C++17 and the X-Plane SDK.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DXPLANE_SDK_ROOT="path/to/SDK"
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
python tests/test_wed_extract.py
```

The C++ test suite includes grass/engine-wash math, replay history and direct DSF polygon decoding.

## Current limitations

- One distinct OGR `.for` resource per scenery pack. You may draw many WED polygons/holes using it.
- Direct v0.2 reading targets uncompressed DSF v1 and WED Area forest placements.
- 7z-compressed DSFs currently use the legacy `ogr_areas.json` / `BUILD_GRASS_AREAS.cmd` fallback.
- Replay effects are available inside OGR's in-memory history window (up to about 20 minutes).
