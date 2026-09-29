#include "ogr/wheel.hpp"

#include <cmath>
#include <iostream>

namespace {

bool close_to(float a, float b, float eps = 0.0001f) {
  return std::abs(a - b) <= eps;
}

int fail(const char* message) {
  std::cerr << "OGR wheel test failed: " << message << '\n';
  return 1;
}

} // namespace

int main() {
  if (!close_to(ogr::wheel::contact_weight(0.0f, 1.5f), 1.0f))
    return fail("wheel center must fully flatten grass");
  if (!close_to(ogr::wheel::contact_weight(1.5f, 1.5f), 0.0f))
    return fail("outside the tire influence radius must be untouched");

  const float half = ogr::wheel::contact_weight(0.75f, 1.5f);
  if (!(half > 0.45f && half < 0.55f))
    return fail("contact falloff should be smooth around half radius");

  float amount = 1.0f;
  amount = ogr::wheel::recover_linear(amount, 2.75f, 5.5f);
  if (!close_to(amount, 0.5f, 0.001f))
    return fail("grass should recover linearly over the configured time");
  amount = ogr::wheel::recover_linear(amount, 2.75f, 5.5f);
  if (!close_to(amount, 0.0f, 0.001f))
    return fail("grass should return upright after recovery time");

  float x = 0.0f;
  float z = 0.0f;
  ogr::wheel::normalize_or(3.0f, 4.0f, 0.0f, -1.0f, x, z);
  if (!close_to(x, 0.6f) || !close_to(z, 0.8f))
    return fail("travel direction normalization is incorrect");

  ogr::wheel::normalize_or(0.0f, 0.0f, 0.0f, -1.0f, x, z);
  if (!close_to(x, 0.0f) || !close_to(z, -1.0f))
    return fail("stationary wheel fallback direction is incorrect");

  std::cout << "OGR wheel flatten test passed\n";
  return 0;
}
