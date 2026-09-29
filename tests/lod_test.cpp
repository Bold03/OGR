#include "ogr/lod.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

int main() {
  using namespace ogr::lod;

  const auto stream = split_stream_budget(10000);
  assert(stream.near_dense == 4500);
  assert(stream.near_outer == 1500);
  assert(stream.mid == 2500);
  assert(stream.far == 1500);
  assert(stream.near_dense + stream.near_outer + stream.mid + stream.far == 10000);

  const auto active = split_active_budget(1600);
  assert(active.near == 896);
  assert(active.mid == 448);
  assert(active.far == 256);
  assert(active.near + active.mid + active.far == 1600);

  const float mid = mid_radius(600.0f, 2000.0f);
  assert(std::abs(mid - 1200.0f) < 0.01f);

  assert(ring_keep_probability(900.0f, 700.0f, 1200.0f, 100.0f, 120.0f) > 0.99f);
  assert(ring_keep_probability(700.0f, 700.0f, 1200.0f, 100.0f, 120.0f) == 0.0f);
  assert(ring_keep_probability(1200.0f, 700.0f, 1200.0f, 100.0f, 120.0f) == 0.0f);
  const float fading = ring_keep_probability(1150.0f, 700.0f, 1200.0f, 100.0f, 120.0f);
  assert(fading > 0.0f && fading < 1.0f);

  std::cout << "OGR three-stage LOD math test passed\n";
  return 0;
}
