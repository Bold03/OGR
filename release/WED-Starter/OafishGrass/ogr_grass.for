A
800
FOREST

# Native XP11/WED fallback: one nearly invisible transparent billboard at an
# extremely large spacing. OGR itself renders the real animated grass.
TEXTURE ogr_marker.png
NO_SHADOW
LOD 50
SCALE_X 1
SCALE_Y 1
SPACING 10000 10000
RANDOM 0 0
TREE 0 0 1 1 0.5 100 0.01 0.01 1 0 OGR_marker

# Oafish Grass Runtime extension metadata. X-Plane/WED ignore these comments;
# OGR reads them from the scenery resource.
# v0.5 uses a 3-stage runtime LOD: near animated, mid static/cheap, far sparse,
# with deterministic density cross-fades so the outer edge does not pop at once.
#OGR_VERSION 1
#OGR_MODEL grass_A.obj
#OGR_MODEL grass_B.obj
#OGR_MODEL grass_C.obj
#OGR_TILE_SIZE_M 2.4
#OGR_DRAW_DISTANCE_M 2000.0
#OGR_ANIMATED_DISTANCE_M 600.0
#OGR_BOUNDARY_MARGIN_M 1.4
#OGR_GROUND_OFFSET_M 0.0
#OGR_WEATHER_WIND 1
#OGR_WIND_STRENGTH 1.0
#OGR_WIND_FULL_BEND_KT 45.0
#OGR_ENGINE_WASH 1
#OGR_ENGINE_WASH_STRENGTH 1.3
#OGR_ENGINE_WASH_RANGE_M 35.0
#OGR_ENGINE_WASH_HALF_ANGLE_DEG 18.0
#OGR_ENGINE_WASH_BASE_HALF_WIDTH_M 2.5
#OGR_TRAFFIC_WASH 1
#OGR_TRAFFIC_WASH_STRENGTH 0.90
#OGR_MAX_TRAFFIC_TARGETS 24
#OGR_TRAFFIC_UPDATE_INTERVAL_S 0.10
#OGR_MAX_ACTIVE_TILES 1600
#OGR_MAX_TOTAL_TILES 10000
#OGR_REFRESH_INTERVAL_S 0.35
#OGR_ANIMATION_INTERVAL_S 0.05
#OGR_HIDE_AIRCRAFT_AGL_FT 3000
#OGR_SCATTER_MODE uniform
