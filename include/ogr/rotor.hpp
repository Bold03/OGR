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

inline float radius_from_disc_area(float area_m2, float fallback_m = 5.5f) {
  if (!std::isfinite(area_m2) || area_m2 <= 1.0f)
    return std::clamp(fallback_m, 2.0f, 12.0f);
  return std::clamp(std::sqrt(area_m2 / grass_math::pi), 2.0f, 12.0f);
}

// Aircraft files/plugins are not perfectly consistent about the practical unit
// exposed by acf_RSC_redline_prp. Accept either rad/s-style values or obvious
// RPM-style values so native XP11 helicopters do not end up with a tiny RPM
// ratio and an almost invisible main-rotor field.
inline float rotor_reference_rad_s(float redline_raw, float fallback_rad_s = 45.0f) {
  if (!std::isfinite(redline_raw) || redline_raw <= 5.0f)
    return std::max(5.0f, fallback_rad_s);
  if (redline_raw > 140.0f)
    return std::max(5.0f, redline_raw * 2.0f * grass_math::pi / 60.0f);
  return redline_raw;
}

inline float power_from_state(float rpm_ratio, float collective_ratio) {
  const float rpm = std::clamp(std::isfinite(rpm_ratio) ? rpm_ratio : 0.0f, 0.0f, 1.20f);
  const float collective = std::clamp(
      std::abs(std::isfinite(collective_ratio) ? collective_ratio : 0.0f), 0.0f, 1.0f);

  const float loading = 0.22f + 0.78f * collective;
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
  const float inner = std::max(radius * 1.35f, 7.0f);
  const float outer = std::clamp(radius * 4.8f + height * 0.55f, 28.0f, 48.0f);
  if (distance_m <= inner)
    return std::clamp(power, 0.0f, 1.0f);
  if (distance_m >= outer)
    return 0.0f;
  const float t = (distance_m - inner) / std::max(0.1f, outer - inner);
  return std::clamp(power, 0.0f, 1.0f) * (1.0f - smoothstep01(t));
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

  const float footprint = std::clamp(radius * 1.28f + height * 0.68f,
                                     radius * 1.15f, radius * 4.0f);
  if (distance >= footprint || distance < 0.05f)
    return out;

  const float normalized = distance / footprint;
  // Keep the physical stagnation point tiny. The first build used a broad
  // inner dead zone, which made the main rotor visually weak while the legacy
  // directional engine wake remained obvious behind the helicopter.
  const float inner = smoothstep01(normalized / 0.08f);
  const float outer = smoothstep01((1.0f - normalized) / 0.40f);
  const float radial_shape = inner * outer;
  if (radial_shape <= 0.0001f)
    return out;

  const float height_ratio = height / std::max(1.0f, radius);
  const float height_factor = 1.0f / (1.0f + 0.66f * height_ratio * height_ratio);
  const float influence = std::clamp(power * strength * height_factor * radial_shape,
                                     0.0f, 1.20f);

  const float inv_distance = 1.0f / distance;
  const float radial_x = dx * inv_distance;
  const float radial_z = dz * inv_distance;
  const float tangent_x = -radial_z * (swirl_sign < 0.0f ? -1.0f : 1.0f);
  const float tangent_z = radial_x * (swirl_sign < 0.0f ? -1.0f : 1.0f);

  constexpr float swirl = 0.14f;
  out.bend.x = (radial_x + tangent_x * swirl) * influence;
  out.bend.z = (radial_z + tangent_z * swirl) * influence;
  out.speed_mps = (9.0f + 31.0f * std::clamp(power, 0.0f, 1.25f)) *
                  height_factor * radial_shape;
  out.influence = influence;
  return out;
}

} // namespace ogr::rotor
