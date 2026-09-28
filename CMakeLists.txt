cmake_minimum_required(VERSION 3.21)
project(OafishGrassRuntime VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT XPLANE_SDK_ROOT)
  message(FATAL_ERROR "Set XPLANE_SDK_ROOT to the extracted X-Plane SDK directory")
endif()

include(FetchContent)
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

add_library(OafishGrass SHARED
  src/plugin.cpp
  src/datarefs.cpp
  src/runtime.cpp
)

target_include_directories(OafishGrass PRIVATE
  include
  "${XPLANE_SDK_ROOT}/CHeaders/XPLM"
  "${XPLANE_SDK_ROOT}/CHeaders/Widgets"
)
target_link_libraries(OafishGrass PRIVATE nlohmann_json::nlohmann_json)
target_compile_definitions(OafishGrass PRIVATE XPLM303 XPLM301 XPLM300 XPLM210 XPLM200)
# XPLM 3.0 package layout expects the binary name to match the plugin package
# name: OafishGrassRuntime/win_x64/OafishGrassRuntime.xpl.
set_target_properties(OafishGrass PROPERTIES
  PREFIX ""
  SUFFIX ".xpl"
  OUTPUT_NAME "OafishGrassRuntime"
)

if(WIN32)
  target_compile_definitions(OafishGrass PRIVATE IBM=1 NOMINMAX WIN32_LEAN_AND_MEAN)
  target_link_libraries(OafishGrass PRIVATE "${XPLANE_SDK_ROOT}/Libraries/Win/XPLM_64.lib" user32)
elseif(APPLE)
  target_compile_definitions(OafishGrass PRIVATE APL=1)
  target_link_options(OafishGrass PRIVATE "-F${XPLANE_SDK_ROOT}/Libraries/Mac" "-framework" "XPLM")
elseif(UNIX)
  target_compile_definitions(OafishGrass PRIVATE LIN=1)
  target_link_options(OafishGrass PRIVATE "-Wl,--unresolved-symbols=ignore-all")
endif()

if(MSVC)
  target_compile_options(OafishGrass PRIVATE /W4 /EHsc /utf-8)
else()
  target_compile_options(OafishGrass PRIVATE -Wall -Wextra -Wpedantic)
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(ogr_grass_math_test tests/grass_math_test.cpp)
  target_include_directories(ogr_grass_math_test PRIVATE include)
  if(MSVC)
    target_compile_options(ogr_grass_math_test PRIVATE /W4 /EHsc /utf-8)
  else()
    target_compile_options(ogr_grass_math_test PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  add_test(NAME ogr_grass_geometry_and_engine_wash COMMAND ogr_grass_math_test)
endif()
