#include "ogr/grass_math.hpp"
#include "ogr/grass_motion.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

using ogr::grass_math::Point2;

static void require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}

int main() {
  try {
    const std::vector<Point2> outer{{0,0},{20,0},{20,20},{0,20}};
    const std::vector<std::vector<Point2>> holes{{{8,8},{12,8},{12,12},{8,12}}};
    require(ogr::grass_math::point_in_area_margin({4,4}, outer, holes, 1.0f),
            "valid grass point rejected");
    require(!ogr::grass_math::point_in_area_margin({0.4f,4}, outer, holes, 1.0f),
            "boundary margin allowed grass to leak outside visual footprint");
    require(!ogr::grass_math::point_in_area_margin({8.4f,10}, outer, holes, 1.0f),
            "hole margin did not exclude grass near hole edge");

    const auto aft = ogr::grass_math::engine_wash({0,10},{0,0},0.0f,1.0f,35.0f,18.0f,2.5f);
    const auto ahead = ogr::grass_math::engine_wash({0,-10},{0,0},0.0f,1.0f,35.0f,18.0f,2.5f);
    require(std::hypot(aft.x,aft.z) > 0.05f, "engine wash missing aft of aircraft");
    require(std::hypot(ahead.x,ahead.z) < 0.0001f, "engine wash incorrectly reaches ahead of aircraft");

    const float idle = ogr::grass_math::prop_wash_power(0.25f, 0.5f, 150.0f, true);
    const float high = ogr::grass_math::prop_wash_power(0.90f, 0.5f, 900.0f, true);
    require(high > idle, "prop wash did not increase with actual RPM/thrust");

    std::cout << "OGR grass math tests PASS\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << "OGR grass math tests FAIL: " << e.what() << "\n";
    return 1;
  }
}
