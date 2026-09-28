#include "ogr/replay.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

int main() {
  ogr::ReplayHistory history(1200.0f, 24000);

  for (int i = 0; i < 200; ++i) {
    ogr::ReplayFrame frame;
    frame.running_time = i * 0.05f;
    frame.flight_time = i * 0.05f;
    frame.aircraft_x = i * 0.5f;
    frame.aircraft_y = 10.0f;
    frame.aircraft_z = -20.0f;
    frame.aircraft_heading = 90.0f;
    frame.wind_x_mps = 4.0f;
    frame.wind_z_mps = 1.0f;
    frame.engine_count = 1;
    frame.engines[0].power = i / 199.0f;
    frame.engines[0].rpm_ratio = 0.2f + 0.8f * i / 199.0f;
    history.push(frame);
  }

  assert(history.size() == 200);
  ogr::ReplayFrame sample;
  const bool ok = history.sample(5.0f, 5.0f, 50.0f, 10.0f, -20.0f, 90.0f, sample);
  assert(ok);
  assert(std::abs(sample.running_time - 5.0f) < 0.11f);
  assert(sample.engine_count == 1);
  assert(sample.engines[0].power > 0.45f && sample.engines[0].power < 0.60f);

  // A new live-flight timeline must replace stale history.
  ogr::ReplayFrame reset;
  reset.running_time = 20.0f;
  reset.flight_time = 0.0f;
  history.push(reset);
  assert(history.size() == 1);

  // Outside both recorded time ranges should deliberately fail rather than
  // applying a stale engine wash to unrelated replay footage.
  ogr::ReplayFrame miss;
  assert(!history.sample(1000.0f, 1000.0f, 0.0f, 0.0f, 0.0f, 0.0f, miss));

  std::cout << "OGR replay history tests passed\n";
  return 0;
}
