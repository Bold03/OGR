#pragma once

#include <algorithm>
#include <cstddef>
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

// A tire that remains on the grass continuously holds at least the current
// spatial contact pressure. This is intentionally independent of groundspeed:
// a parked aircraft still has weight on its tires and must keep the blades down.
inline float apply_contact(float amount, float contact) {
  const float current = std::clamp(std::isfinite(amount) ? amount : 0.0f, 0.0f, 1.0f);
  const float pressure = std::clamp(std::isfinite(contact) ? contact : 0.0f, 0.0f, 1.0f);
  return std::max(current, pressure);
}

inline float recover_linear(float amount, float dt, float recovery_s) {
  amount = std::clamp(amount, 0.0f, 1.0f);
  if (amount <= 0.0f || !std::isfinite(dt) || dt <= 0.0f) return amount;
  const float seconds = std::max(0.25f, recovery_s);
  return std::max(0.0f, amount - dt / seconds);
}

// X-Plane 11's rain_percent is normally a ratio, while some weather providers
// and future/alternate datarefs may expose percentage-style values. Normalize
// both forms conservatively so 25 means 25% rather than instantly saturating.
inline float normalize_precipitation(float raw) {
  if (!std::isfinite(raw) || raw <= 0.0f) return 0.0f;
  if (raw > 1.5f) raw *= 0.01f;
  return std::clamp(raw, 0.0f, 1.0f);
}

inline float wetness_target(float precipitation) {
  const float p = normalize_precipitation(precipitation);
  if (p <= 0.005f) return 0.0f;
  return std::clamp(0.15f + p * 0.85f, 0.0f, 1.0f);
}

// Wetness builds quickly once rain starts but dries much more slowly after the
// rain stops. Exponential approach keeps the result stable across frame rates.
inline float approach_wetness(float current, float target, float dt,
                              float wetting_s = 18.0f,
                              float drying_s = 240.0f) {
  current = std::clamp(std::isfinite(current) ? current : 0.0f, 0.0f, 1.0f);
  target = std::clamp(std::isfinite(target) ? target : 0.0f, 0.0f, 1.0f);
  if (!std::isfinite(dt) || dt <= 0.0f) return current;
  const float tau = std::max(0.25f, target > current ? wetting_s : drying_s);
  const float alpha = 1.0f - std::exp(-std::min(dt, 5.0f) / tau);
  return std::clamp(current + (target - current) * alpha, 0.0f, 1.0f);
}

// Fully wet blades keep about 82% of the dry bend magnitude and respond through
// a longer low-pass time constant, making wind/prop/jet motion feel heavier.
inline float wet_bend_scale(float wetness) {
  const float wet = std::clamp(std::isfinite(wetness) ? wetness : 0.0f, 0.0f, 1.0f);
  return 1.0f - 0.18f * wet;
}

inline float wet_filter_seconds(float wetness) {
  const float wet = std::clamp(std::isfinite(wetness) ? wetness : 0.0f, 0.0f, 1.0f);
  return 0.02f + 0.22f * wet;
}

inline float wet_track_recovery_seconds(float wetness,
                                        float dry_seconds = 5.5f,
                                        float wet_seconds = 32.0f) {
  const float wet = std::clamp(std::isfinite(wetness) ? wetness : 0.0f, 0.0f, 1.0f);
  const float dry = std::max(0.25f, dry_seconds);
  const float soaked = std::max(dry, wet_seconds);
  return dry + (soaked - dry) * wet;
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

// X-Plane TCAS traffic exposes aircraft position/heading, not individual tire
// coordinates. traffic.cpp supplies an estimated aircraft-level footprint and
// runtime_wheel.cpp expands it into one nose + two main wheel contacts.
struct TrafficAircraftContact {
  int id{};
  float x{};
  float z{};
  float heading_deg{};
  float vx{};
  float vz{};
  float nose_forward_m{};
  float main_aft_m{};
  float main_half_track_m{};
  float contact_radius_m{1.5f};
  bool on_ground{};
};

// Replaces the current cached set. Passing nullptr/0 clears online/AI wheel
// contacts immediately; user-aircraft wheel contacts remain independent.
void update_traffic_aircraft_contacts(const TrafficAircraftContact* contacts,
                                      std::size_t count);

// Runtime hooks are implemented in runtime_wheel.cpp. Explicit initialization
// lets the plugin report wheel-dataref availability at startup, even before the
// first grass instance happens to be drawn.
void initialize_runtime();
void reset_runtime();

} // namespace ogr::wheel
