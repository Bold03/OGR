#include "ogr/rotor.hpp"

#include <cmath>
#include <iostream>

namespace {

bool close_to(float a, float b, float eps = 0.001f) {
  return std::abs(a - b) <= eps;
}

int fail(const char* message) {
  std::cerr << "OGR rotor test failed: " << message << '\n';
  return 1;
}

} // namespace

int main() {
  const float radius = ogr::rotor::radius_from_disc_area(ogr::grass_math::pi * 25.0f);
  if (!close_to(radius, 5.0f, 0.01f))
    return fail("disc area must convert back to rotor radius");

  const float rpm_style = ogr::rotor::rotor_reference_rad_s(300.0f);
  const float expected_rad_s = 300.0f * 2.0f * ogr::grass_math::pi / 60.0f;
  if (!close_to(rpm_style, expected_rad_s, 0.02f))
    return fail("obvious RPM-style rotor redline must convert to rad/s");
  if (!close_to(ogr::rotor::rotor_reference_rad_s(32.0f), 32.0f, 0.01f))
    return fail("rad/s-style rotor redline must remain unchanged");

  const float xp11_ratio = ogr::rotor::rotor_ratio_from_speed(32.0f, 2700.0f, 45.0f);
  if (!(xp11_ratio > 0.60f && xp11_ratio < 0.90f))
    return fail("XP11 POINT_tacrad rotor must survive an unrelated high prop redline");

  const float idle = ogr::rotor::power_from_state(1.0f, 0.0f);
  const float hover = ogr::rotor::power_from_state(1.0f, 0.65f);
  const float high_collective = ogr::rotor::power_from_state(1.0f, 1.0f);
  if (!(idle > 0.0f && hover > idle && high_collective > hover))
    return fail("collective must increase rotor downwash at governed RPM");

  const auto wash = ogr::rotor::ground_wash(
      {4.0f, 0.0f}, {0.0f, 0.0f}, 3.0f, 5.0f, hover, 1.0f, 1.0f, 35.0f);
  if (!(wash.influence > 0.0f && wash.bend.x > 0.0f))
    return fail("grass east of rotor should receive outward ground flow");
  if (!(std::abs(wash.bend.z) > 0.0f))
    return fail("rotor ground flow should include a small rotational swirl");

  const auto near_center = ogr::rotor::ground_wash(
      {0.7f, 0.0f}, {0.0f, 0.0f}, 3.0f, 5.0f, hover, 1.0f, 1.0f, 35.0f);
  if (!(near_center.influence > 0.05f))
    return fail("main rotor field must not have a broad dead zone near the hub");

  const float local_dominance = ogr::rotor::radial_dominance(8.0f, 5.0f, 3.0f, 1.0f);
  const float far_dominance = ogr::rotor::radial_dominance(60.0f, 5.0f, 3.0f, 1.0f);
  if (!(local_dominance > 0.5f && close_to(far_dominance, 0.0f)))
    return fail("radial rotor field must suppress the old local rear cone but not distant animation");

  const auto reverse_swirl = ogr::rotor::ground_wash(
      {4.0f, 0.0f}, {0.0f, 0.0f}, 3.0f, 5.0f, hover, -1.0f, 1.0f, 35.0f);
  if (!(wash.bend.z * reverse_swirl.bend.z < 0.0f))
    return fail("reversing rotor direction must reverse swirl direction");

  const auto high_hover = ogr::rotor::ground_wash(
      {4.0f, 0.0f}, {0.0f, 0.0f}, 18.0f, 5.0f, hover, 1.0f, 1.0f, 35.0f);
  if (!(high_hover.influence < wash.influence))
    return fail("rotor ground effect must weaken as hover height increases");

  const auto too_high = ogr::rotor::ground_wash(
      {4.0f, 0.0f}, {0.0f, 0.0f}, 40.0f, 5.0f, hover, 1.0f, 1.0f, 35.0f);
  if (!close_to(too_high.influence, 0.0f))
    return fail("rotor wash must stop beyond the configured maximum height");

  const auto far = ogr::rotor::ground_wash(
      {100.0f, 0.0f}, {0.0f, 0.0f}, 3.0f, 5.0f, hover, 1.0f, 1.0f, 35.0f);
  if (!close_to(far.influence, 0.0f))
    return fail("far grass must not receive rotor wash");

  const ogr::grass_math::Vector2 old_dir{1.0f, 0.0f};
  const ogr::grass_math::Vector2 flipped{-1.0f, 0.0f};
  const auto stable = ogr::rotor::stabilize_crush_direction(old_dir, flipped, true);
  if (!(stable.x > 0.95f && std::abs(stable.z) < 0.05f))
    return fail("hard wheel-crush direction reversal must remain locked");

  const ogr::grass_math::Vector2 gentle_turn{0.8f, -0.6f};
  const auto steered = ogr::rotor::stabilize_crush_direction(old_dir, gentle_turn, true);
  if (!(steered.x > 0.0f && steered.z < 0.0f))
    return fail("non-reversing wheel-crush direction should steer gradually");

  std::cout << "OGR helicopter rotor wash tests passed\n";
  return 0;
}
