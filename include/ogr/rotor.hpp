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

inline float power_from_state(float rpm_ratio, float collective_ratio) {
  const float rpm = std::clamp(std::isfinite(rpm_ratio) ? rpm_ratio : 0.0f, 0.0f, 1.20f);
  const float collective = std::clamp(
      std::abs(std::isfinite(collective_ratio) ? collective_ratio : 0.0f), 0.0f, 1.0f);

  const float loading = 0.22f + 0.78f * collective;
  return std::clamp(rpm * rpm * loading, 0.0f, 1.25f);
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

  const float footprint = std::clamp(radius * 1.20f + height * 0.62f,
                                     radius * 1.10f, radius * 3.80f);
  if (distance >= footprint || distance < 0.05f)
    return out;

  const float normalized = distance / footprint;
  const float inner = smoothstep01(normalized / 0.22f);
  const float outer = smoothstep01((1.0f - normalized) / 0.42f);
  const float radial_shape = inner * outer;
  if (radial_shape <= 0.0001f)
    return out;

  const float height_ratio = height / std::max(1.0f, radius);
  const float height_factor = 1.0f / (1.0f + 0.72f * height_ratio * height_ratio);
  const float influence = std::clamp(power * strength * height_factor * radial_shape,
                                     0.0f, 1.15f);

  const float inv_distance = 1.0f / distance;
  const float radial_x = dx * inv_distance;
  const float radial_z = dz * inv_distance;
  const float tangent_x = -radial_z * (swirl_sign < 0.0f ? -1.0f : 1.0f);
  const float tangent_z = radial_x * (swirl_sign < 0.0f ? -1.0f : 1.0f);

  constexpr float swirl = 0.16f;
  out.bend.x = (radial_x + tangent_x * swirl) * influence;
  out.bend.z = (radial_z + tangent_z * swirl) * influence;
  out.speed_mps = (8.0f + 27.0f * std::clamp(power, 0.0f, 1.25f)) *
                  height_factor * radial_shape;
  out.influence = influence;
  return out;
}

} // namespace ogr::rotor
