#pragma once

#include "ogr/grass_math.hpp"

#include <algorithm>
#include <cmath>

namespace ogr::rotor {

struct GroundWash {
  grass_math::Vector2 bend{};
  float speed_mps{};
  float influence{};
};

inline float smoothstep01(float value) {
  const float t = std::clamp(value, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

inline grass_math::Vector2 normalized_or(grass_math::Vector2 value,
                                          grass_math::Vector2 fallback = {0.0f, -1.0f}) {
  const float length = std::hypot(value.x, value.z);
  if (std::isfinite(length) && length > 0.0001f)
    return {value.x / length, value.z / length};
  const float fallback_length = std::hypot(fallback.x, fallback.z);
  if (std::isfinite(fallback_length) && fallback_length > 0.0001f)
    return {fallback.x / fallback_length, fallback.z / fallback_length};
  return {0.0f, -1.0f};
}

// A crushed grass instance can sit exactly between two gear contacts. Small
// gear-position jitter used to make the selected contact alternate and flip the
// blade direction left/right each update. Lock hard reversals while the same
// instance remains crushed, but still allow gradual steering changes.
inline grass_math::Vector2 stabilize_crush_direction(grass_math::Vector2 previous,
                                                       grass_math::Vector2 current,
                                                       bool has_previous) {
  const auto now = normalized_or(current);
  if (!has_previous) return now;
  const auto old = normalized_or(previous, now);
  const float dot = old.x * now.x + old.z * now.z;
  if (dot < -0.10f)
    return old;
  const float follow = dot < 0.45f ? 0.08f : 0.18f;
  return normalized_or({old.x * (1.0f - follow) + now.x * follow,
                        old.z * (1.0f - follow) + now.z * follow}, old);
}

inline float radius_from_disc_area(float area_m2, float fallback_m = 5.5f) {
  if (!std::isfinite(area_m2) || area_m2 <= 1.0f)
    return std::clamp(fallback_m, 2.0f, 12.0f);
  return std::clamp(std::sqrt(area_m2 / grass_math::pi), 2.0f, 12.0f);
}

// Aircraft files/plugins are not perfectly consistent about the practical unit
// exposed by acf_RSC_redline_prp. Accept either rad/s-style values or obvious
// RPM-style values so XP11 helicopters do not end up with a tiny RPM ratio.
inline float rotor_reference_rad_s(float redline_raw, float fallback_rad_s = 45.0f) {
  if (!std::isfinite(redline_raw) || redline_raw <= 5.0f)
    return std::max(5.0f, fallback_rad_s);
  if (redline_raw > 140.0f)
    return std::max(5.0f, redline_raw * 2.0f * grass_math::pi / 60.0f);
  return redline_raw;
}

inline float rotor_ratio_from_speed(float speed_rad_s, float redline_raw,
                                    float fallback_rad_s = 45.0f) {
  const float speed = std::abs(std::isfinite(speed_rad_s) ? speed_rad_s : 0.0f);
  if (speed <= 0.01f) return 0.0f;
  float reference = rotor_reference_rad_s(redline_raw, fallback_rad_s);
  float ratio = speed / std::max(5.0f, reference);
  // Some XP11 helicopters expose an engine/prop redline that is unrelated to
  // POINT_tacrad's geared main-rotor speed. If that would make an obviously
  // spinning 10..100 rad/s rotor look nearly stopped, use the helicopter-scale
  // reference instead of suppressing rotor wash entirely.
  if (speed >= 10.0f && speed <= 100.0f && ratio < 0.30f)
    ratio = speed / std::max(20.0f, fallback_rad_s);
  return std::clamp(ratio, 0.0f, 1.20f);
}

inline float power_from_state(float rpm_ratio, float collective_ratio) {
  const float rpm = std::clamp(std::isfinite(rpm_ratio) ? rpm_ratio : 0.0f, 0.0f, 1.20f);
  const float collective = std::clamp(
      std::abs(std::isfinite(collective_ratio) ? collective_ratio : 0.0f), 0.0f, 1.0f);

  const float loading = 0.24f + 0.76f * collective;
  return std::clamp(rpm * rpm * loading, 0.0f, 1.25f);
}

// While a helicopter main rotor is active, the old fixed-wing/propeller wake
// must not leave a fake one-directional cone behind the fuselage. This gives the
// main rotor authority over a bounded local region and fades that authority out
// before the normal airport wind/traffic field resumes.
inline float radial_dominance(float distance_m, float rotor_radius_m,
                              float rotor_height_m, float power) {
  if (!std::isfinite(distance_m) || !std::isfinite(rotor_radius_m) ||
      !std::isfinite(rotor_height_m) || !std::isfinite(power) || power <= 0.0f)
    return 0.0f;
  const float radius = std::clamp(rotor_radius_m, 2.0f, 12.0f);
  const float height = std::max(0.0f, rotor_height_m);
  const float inner = std::max(radius * 1.55f, 8.0f);
  const float outer = std::clamp(radius * 4.9f + height * 0.75f, 30.0f, 52.0f);
  if (distance_m <= inner)
    return std::clamp(power * 1.25f, 0.0f, 1.0f);
  if (distance_m >= outer)
    return 0.0f;
  const float t = (distance_m - inner) / std::max(0.1f, outer - inner);
  return std::clamp(power * 1.25f, 0.0f, 1.0f) * (1.0f - smoothstep01(t));
}

inline GroundWash ground_wash(grass_math::Point2 grass,
                              grass_math::Point2 rotor_center,
                              float rotor_height_m,
                              float rotor_radius_m,
                              float power,
                              float swirl_sign,
                              float strength = 1.0f,
                              float max_height_m = 35.0f) {
  GroundWash out;
  if (!std::isfinite(rotor_height_m) || !std::isfinite(rotor_radius_m) ||
      !std::isfinite(power) || !std::isfinite(strength))
    return out;

  const float radius = std::clamp(rotor_radius_m, 2.0f, 12.0f);
  const float height = std::max(0.0f, rotor_height_m);
  const float max_height = std::max(radius, max_height_m);
  if (power <= 0.002f || strength <= 0.0f || height > max_height)
    return out;

  const float dx = grass.x - rotor_center.x;
  const float dz = grass.z - rotor_center.z;
  const float distance = std::hypot(dx, dz);

  // Ground flow extends beyond the geometric disc after the downwash turns
  // outward at the surface. Keep the footprint local but visibly 360-degree.
  const float footprint = std::clamp(radius * 1.55f + height * 0.85f,
                                     radius * 1.35f, radius * 4.50f);
  if (distance >= footprint || distance < 0.02f)
    return out;

  const float normalized = distance / footprint;
  const float inner = smoothstep01(normalized / 0.035f);
  const float outer = smoothstep01((1.0f - normalized) / 0.45f);
  const float radial_shape = inner * outer;
  if (radial_shape <= 0.0001f)
    return out;

  const float height_ratio = height / std::max(1.0f, radius);
  const float height_factor = 1.0f / (1.0f + 0.58f * height_ratio * height_ratio);
  const float influence = std::clamp(power * strength * height_factor * radial_shape,
                                     0.0f, 1.35f);

  const float inv_distance = 1.0f / distance;
  const float radial_x = dx * inv_distance;
  const float radial_z = dz * inv_distance;
  const float tangent_x = -radial_z * (swirl_sign < 0.0f ? -1.0f : 1.0f);
  const float tangent_z = radial_x * (swirl_sign < 0.0f ? -1.0f : 1.0f);

  constexpr float swirl = 0.12f;
  out.bend.x = (radial_x + tangent_x * swirl) * influence;
  out.bend.z = (radial_z + tangent_z * swirl) * influence;
  out.speed_mps = (10.0f + 34.0f * std::clamp(power, 0.0f, 1.25f)) *
                  height_factor * radial_shape;
  out.influence = influence;
  return out;
}

} // namespace ogr::rotor
