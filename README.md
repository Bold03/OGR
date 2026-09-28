# Oafish Grass Runtime (OGR) v0.1.1

Standalone WED-authored animated grass runtime for X-Plane 11/12.

OGR is deliberately separate from SSA. WED is responsible for authoring the grass area; the OGR plugin is responsible for drawing and animating the real grass instances.

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
- strict WED polygon edge margin so the ~2.6 m grass cards do not spill onto asphalt

## New in v0.1.1: replay-aware grass effects

OGR now records a lightweight 20 Hz history of the environmental inputs that matter to grass animation. The recorder keeps up to 20 minutes of:

- per-engine wash power and RPM ratio
- engine position relative to the aircraft
- reverse/forward wake heading
- weather wind vector
- aircraft transform used to align replay samples

When `sim/time/is_in_replay` becomes active, OGR stops depending on the live engine state for the final grass bend and selects the matching historical sample using X-Plane's replay time. This lets Cessna prop wash, jet/N1 blast and weather wind remain visible while the player watches or scrubs a replay. If the replay is moved outside OGR's recorded history window, the effect is deliberately neutral instead of applying stale wash to unrelated footage.

The replay recorder stores engine state, not every grass blade, so the memory/CPU cost stays small enough for older systems.

## WED workflow

1. Install the plugin in `X-Plane 11/Resources/plugins/OafishGrassRuntime/`.
2. Copy `WED-Starter/OafishGrass` into the scenery pack you are editing, for example:
   `Custom Scenery/WIDN Tanjung Pinang/OafishGrass/`.
3. Open the scenery in WED.
4. Select the **Forest** polygon tool and choose `OafishGrass/ogr_grass.for`.
5. Draw the exact grass area. WED holes are supported. WED's forest density value is also used by OGR.
6. Save the WED project. Export the scenery as usual.
7. Run `OafishGrass/BUILD_GRASS_AREAS.cmd` once after changing the grass mapping.
8. Start X-Plane, or use `Plugins > Oafish Grass Runtime > Reload WED grass areas` after rebuilding the area file.

The small `.for` file is a normal XP11 forest resource so WED can author it. Its native fallback is effectively invisible. Lines beginning with `#OGR_` are OGR metadata comments; the runtime exporter reads them, while normal WED/X-Plane forest parsing ignores them.

## Why v0.1 has BUILD_GRASS_AREAS.cmd

X-Plane's plugin SDK does not expose an API that enumerates arbitrary forest polygons from loaded scenery. v0.1 therefore converts WED's `earth.wed.xml` into `OafishGrass/ogr_areas.json` with one click. The runtime itself does not depend on SSA or `ssa.json`.

A future version can replace this bridge with direct exported-DSF parsing; the WED authoring workflow and `.for` resource can remain the same.

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

`DRAW_DISTANCE_M` is in metres because X-Plane's object LOD is metric. `914.4 m = 3000 ft`.

## Performance profile

The default profile is deliberately conservative for older iGPU systems:

- at most 1600 active instances around the camera
- at most 8000 candidate placements per scenery pack
- animation updates at 20 Hz (`0.05 s`)
- camera active-set refresh every `0.35 s`
- far grass stops receiving animation updates
- above the configured aircraft AGL cutoff, instances are destroyed only once and stay suspended until descent
- replay history records environment/engine inputs at 20 Hz rather than storing per-grass animation state

## Build

Requires CMake 3.21+, C++17 and the X-Plane SDK.

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DXPLANE_SDK_ROOT="path/to/SDK"
cmake --build build --config Release --parallel
ctest --test-dir build -C Release --output-on-failure
python tests/test_wed_extract.py
```

The included GitHub Actions workflow builds Windows and publishes the ZIP directly to **GitHub Releases**, so it does not use Actions artifact storage.

## Current limitations

- One distinct OGR `.for` resource per scenery pack. You may draw as many WED forest polygons/holes with that resource as needed.
- Run `BUILD_GRASS_AREAS.cmd` after changing WED grass polygons.
- Replay effects are available only inside OGR's current in-memory history window (up to 20 minutes).
- This is a new standalone runtime; test it in a copy of the scenery before distribution.
