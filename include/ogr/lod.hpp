#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace ogr::lod {

struct StreamBudget {
  std::size_t near_dense{};
  std::size_t near_outer{};
  std::size_t mid{};
  std::size_t far{};
};

struct ActiveBudget {
  std::size_t near{};
  std::size_t mid{};
  std::size_t far{};
};

inline StreamBudget split_stream_budget(std::size_t total) {
  total = std::max<std::size_t>(100, total);
  StreamBudget out;
  out.near_dense = static_cast<std::size_t>(std::floor(total * 0.45));
  out.near_outer = static_cast<std::size_t>(std::floor(total * 0.15));
  out.mid = static_cast<std::size_t>(std::floor(total * 0.25));
  out.far = total - out.near_dense - out.near_outer - out.mid;
  return out;
}

inline ActiveBudget split_active_budget(std::size_t total) {
  total = std::max<std::size_t>(3, total);
  ActiveBudget out;
  out.near = static_cast<std::size_t>(std::floor(total * 0.56));
  out.mid = static_cast<std::size_t>(std::floor(total * 0.28));
  out.far = total - out.near - out.mid;
  out.near = std::max<std::size_t>(1, out.near);
  out.mid = std::max<std::size_t>(1, out.mid);
  out.far = std::max<std::size_t>(1, out.far);
  return out;
}

inline float mid_radius(float animated_radius, float draw_radius) {
  draw_radius = std::max(30.0f, draw_radius);
  animated_radius = std::clamp(animated_radius, 15.0f, draw_radius);
  if (draw_radius <= animated_radius + 1.0f) return draw_radius;
  const float preferred = std::max(animated_radius + 250.0f, draw_radius * 0.60f);
  return std::clamp(preferred, animated_radius + 50.0f, draw_radius);
}

inline float smoothstep01(float value) {
  const float t = std::clamp(value, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

// Deterministic density cross-fade. The caller compares a stable per-tile
// random number against this probability. That avoids a hard radial wall
// without requiring alpha-blending or an extra per-instance fade dataref.
inline float ring_keep_probability(float distance_m, float inner_m, float outer_m,
                                   float fade_in_m, float fade_out_m,
                                   float base_probability = 1.0f) {
  if (outer_m <= inner_m || distance_m < inner_m || distance_m > outer_m) return 0.0f;
  float keep = std::clamp(base_probability, 0.0f, 1.0f);
  if (inner_m > 0.0f && fade_in_m > 0.0f)
    keep *= smoothstep01((distance_m - inner_m) / fade_in_m);
  if (fade_out_m > 0.0f)
    keep *= smoothstep01((outer_m - distance_m) / fade_out_m);
  return std::clamp(keep, 0.0f, 1.0f);
}

} // namespace ogr::lod
