#include "ogr/traffic.hpp"

#include <cassert>
#include <cmath>

int main() {
  using namespace ogr::traffic;

  const auto c172 = profile_for_icao("C172");
  const auto at76 = profile_for_icao("AT76");
  const auto b738 = profile_for_icao("B738");
  const auto b77w = profile_for_icao("B77W");

  assert(c172.kind == Kind::light_prop);
  assert(at76.kind == Kind::turboprop);
  assert(b738.kind == Kind::jet);
  assert(b77w.kind == Kind::heavy_jet);
  assert(b77w.range_m > b738.range_m);
  assert(b738.range_m > c172.range_m);

  const auto c172_gear = gear_footprint(c172.kind);
  const auto at76_gear = gear_footprint(at76.kind);
  const auto b738_gear = gear_footprint(b738.kind);
  const auto b77w_gear = gear_footprint(b77w.kind);
  assert(c172_gear.nose_forward_m < at76_gear.nose_forward_m);
  assert(at76_gear.nose_forward_m < b738_gear.nose_forward_m);
  assert(b738_gear.nose_forward_m < b77w_gear.nose_forward_m);
  assert(c172_gear.main_half_track_m < b738_gear.main_half_track_m);
  assert(b738_gear.main_half_track_m < b77w_gear.main_half_track_m);
  assert(c172_gear.contact_radius_m < b77w_gear.contact_radius_m);

  const float parked = estimated_power(0.0f, 0.0f, true, b738);
  const float taxi = estimated_power(0.0f, 7.0f, true, b738);
  const float takeoff = estimated_power(0.0f, 55.0f, true, b738);
  const float commanded = estimated_power(0.85f, 2.0f, true, b738);
  const float airborne = estimated_power(0.0f, 55.0f, false, b738);

  assert(std::abs(parked) < 0.001f);
  assert(taxi > parked);
  assert(takeoff > taxi);
  assert(commanded > taxi);
  assert(airborne < takeoff);

  return 0;
}
