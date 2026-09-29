#pragma once

#include <algorithm>
#include <cmath>

namespace ogr::wheel {

inline float smoothstep01(float value) {
  const float t = std::clamp(value, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

inline float contact_weight(float distance_m, float radius_m) {
  if (!std::isfinite(distance_m) || !std::isfinite(radius_m) || radius_m <= 0.01f)
    return 0.0f;
  if (distance_m >= radius_m) return 0.0f;
  return smoothstep01(1.0f - std::max(0.0f, distance_m) / radius_m);
}

inline float recover_linear(float amount, float dt, float recovery_s) {
  amount = std::clamp(amount, 0.0f, 1.0f);
  if (amount <= 0.0f || !std::isfinite(dt) || dt <= 0.0f) return amount;
  const float seconds = std::max(0.25f, recovery_s);
  return std::max(0.0f, amount - dt / seconds);
}

inline void normalize_or(float x, float z, float fallback_x, float fallback_z,
                         float& out_x, float& out_z) {
  const float mag = std::hypot(x, z);
  if (std::isfinite(mag) && mag > 0.001f) {
    out_x = x / mag;
    out_z = z / mag;
    return;
  }
  const float fallback_mag = std::hypot(fallback_x, fallback_z);
  if (std::isfinite(fallback_mag) && fallback_mag > 0.001f) {
    out_x = fallback_x / fallback_mag;
    out_z = fallback_z / fallback_mag;
  } else {
    out_x = 0.0f;
    out_z = -1.0f;
  }
}

// Runtime hooks are implemented in runtime_wheel.cpp. Explicit initialization
// lets the plugin report wheel-dataref availability at startup, even before the
// first grass instance happens to be drawn.
void initialize_runtime();
void reset_runtime();

} // namespace ogr::wheel
