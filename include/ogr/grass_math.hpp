#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace ogr::grass_math {

constexpr float pi = 3.14159265358979323846f;

struct Point2 { float x{}, z{}; };
struct Vector2 { float x{}, z{}; };

inline float clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

inline bool point_in_polygon(const Point2& p, const std::vector<Point2>& polygon) {
  if (polygon.size() < 3) return false;
  bool inside = false;
  size_t j = polygon.size() - 1;
  for (size_t i = 0; i < polygon.size(); j = i++) {
    const auto& a = polygon[i];
    const auto& b = polygon[j];
    const bool crosses = ((a.z > p.z) != (b.z > p.z));
    if (!crosses) continue;
    const float denom = b.z - a.z;
    if (std::abs(denom) < 1.0e-7f) continue;
    const float edge_x = (b.x - a.x) * (p.z - a.z) / denom + a.x;
    if (p.x < edge_x) inside = !inside;
  }
  return inside;
}

inline float point_segment_distance(const Point2& p, const Point2& a, const Point2& b) {
  const float vx = b.x - a.x;
  const float vz = b.z - a.z;
  const float wx = p.x - a.x;
  const float wz = p.z - a.z;
  const float vv = vx * vx + vz * vz;
  const float t = vv > 1.0e-8f ? std::clamp((wx * vx + wz * vz) / vv, 0.0f, 1.0f) : 0.0f;
  const float dx = p.x - (a.x + vx * t);
  const float dz = p.z - (a.z + vz * t);
  return std::hypot(dx, dz);
}

inline float distance_to_edges(const Point2& p, const std::vector<Point2>& polygon) {
  if (polygon.size() < 2) return 0.0f;
  float best = 1.0e30f;
  for (size_t i = 0; i < polygon.size(); ++i)
    best = std::min(best, point_segment_distance(p, polygon[i], polygon[(i + 1) % polygon.size()]));
  return best;
}

inline bool point_in_area_margin(const Point2& p, const std::vector<Point2>& outer,
                                 const std::vector<std::vector<Point2>>& holes,
                                 float margin_m) {
  if (!point_in_polygon(p, outer)) return false;
  if (margin_m > 0.0f && distance_to_edges(p, outer) < margin_m) return false;
  for (const auto& hole : holes) {
    if (point_in_polygon(p, hole)) return false;
    if (margin_m > 0.0f && distance_to_edges(p, hole) < margin_m) return false;
  }
  return true;
}

inline uint32_t mix_hash(uint32_t x) {
  x ^= x >> 16;
  x *= 0x7feb352du;
  x ^= x >> 15;
  x *= 0x846ca68bu;
  x ^= x >> 16;
  return x;
}

inline float hash01(uint32_t seed) {
  return static_cast<float>(mix_hash(seed) & 0x00ffffffu) /
         static_cast<float>(0x01000000u);
}

inline bool clustered_keep(Point2 p, float cluster_spacing_m, float cluster_radius_m,
                           float cluster_probability, uint32_t area_seed) {
  cluster_spacing_m = std::max(0.25f, cluster_spacing_m);
  cluster_radius_m = std::max(0.05f, cluster_radius_m);
  cluster_probability = clamp01(cluster_probability);
  if (cluster_probability <= 0.0f) return false;

  const int cell_x = static_cast<int>(std::floor(p.x / cluster_spacing_m));
  const int cell_z = static_cast<int>(std::floor(p.z / cluster_spacing_m));
  float best_distance2 = 1.0e30f;
  float best_radius = 0.0f;
  for (int dz = -1; dz <= 1; ++dz) {
    for (int dx = -1; dx <= 1; ++dx) {
      const int cx = cell_x + dx;
      const int cz = cell_z + dz;
      uint32_t seed = area_seed;
      seed ^= static_cast<uint32_t>(cx) * 73856093u;
      seed ^= static_cast<uint32_t>(cz) * 19349663u;
      seed = mix_hash(seed);
      if (hash01(seed ^ 0x51ed270bu) > cluster_probability) continue;
      const float ox = 0.12f + 0.76f * hash01(seed ^ 0xa341316cu);
      const float oz = 0.12f + 0.76f * hash01(seed ^ 0xc8013ea4u);
      const float center_x = (static_cast<float>(cx) + ox) * cluster_spacing_m;
      const float center_z = (static_cast<float>(cz) + oz) * cluster_spacing_m;
      const float ddx = p.x - center_x;
      const float ddz = p.z - center_z;
      const float d2 = ddx * ddx + ddz * ddz;
      if (d2 < best_distance2) {
        best_distance2 = d2;
        best_radius = cluster_radius_m * (0.72f + 0.56f * hash01(seed ^ 0xad90777du));
      }
    }
  }
  return best_radius > 0.0f && best_distance2 <= best_radius * best_radius;
}

inline Vector2 world_to_object(Vector2 world, float heading_deg) {
  const float h = heading_deg * pi / 180.0f;
  const float c = std::cos(h);
  const float s = std::sin(h);
  return {world.x * c + world.z * s, -world.x * s + world.z * c};
}

inline Vector2 engine_wash(Point2 tile, Point2 engine, float heading_deg,
                           float power, float max_range_m,
                           float half_angle_deg, float base_half_width_m) {
  power = clamp01(power);
  if (power <= 0.001f || max_range_m <= 0.1f) return {};
  const float h = heading_deg * pi / 180.0f;
  const Vector2 aft{-std::sin(h), std::cos(h)};
  const Vector2 right{std::cos(h), std::sin(h)};
  const float dx = tile.x - engine.x;
  const float dz = tile.z - engine.z;
  const float aft_distance = dx * aft.x + dz * aft.z;
  if (aft_distance <= 0.0f) return {};
  const float effective_range = max_range_m * (0.25f + 0.75f * power);
  if (aft_distance >= effective_range) return {};
  const float lateral = std::abs(dx * right.x + dz * right.z);
  const float cone_half_width = std::max(
      0.5f, base_half_width_m + aft_distance *
                                 std::tan(std::clamp(half_angle_deg, 1.0f, 60.0f) * pi / 180.0f));
  if (lateral >= cone_half_width) return {};
  const float range_falloff = 1.0f - aft_distance / effective_range;
  const float cone_falloff = 1.0f - lateral / cone_half_width;
  const float strength = clamp01(power * range_falloff * range_falloff *
                                 cone_falloff * cone_falloff * (3.0f - 2.0f * cone_falloff));
  return {aft.x * strength, aft.z * strength};
}

} // namespace ogr::grass_math
