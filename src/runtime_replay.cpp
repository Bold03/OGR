#include "ogr/runtime.hpp"
#include "ogr/lod.hpp"

// Reuse the proven runtime implementation for loading, geometry generation,
// terrain probing and object handling. Its original update() body is kept as
// update_live(), while the public wrapper below uses a spatially indexed hot
// path plus replay recording/playback.
#define update update_live
#include "runtime.cpp"
#undef update

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <unordered_map>
#include <vector>

namespace ogr {
namespace {

float heading_delta_deg(float value, float reference) {
  float d = std::fmod(value - reference, 360.0f);
  if (d > 180.0f) d -= 360.0f;
  if (d < -180.0f) d += 360.0f;
  return d;
}

float finite_or_zero(float value) {
  return std::isfinite(value) ? value : 0.0f;
}

uint64_t spatial_key(int x, int z) {
  return (static_cast<uint64_t>(static_cast<uint32_t>(x)) << 32) |
         static_cast<uint32_t>(z);
}

int spatial_coord(float value, float cell) {
  return static_cast<int>(std::floor(value / std::max(1.0f, cell)));
}

uint64_t mix64(uint64_t value) {
  value ^= value >> 30;
  value *= 0xbf58476d1ce4e5b9ULL;
  value ^= value >> 27;
  value *= 0x94d049bb133111ebULL;
  value ^= value >> 31;
  return value;
}

uint64_t stream_candidate_key(size_t area_index, int ix, int iz, uint32_t tier) {
  uint64_t value = static_cast<uint64_t>(area_index + 1) * 0x9e3779b97f4a7c15ULL;
  value ^= static_cast<uint64_t>(static_cast<uint32_t>(ix)) << 1;
  value ^= static_cast<uint64_t>(static_cast<uint32_t>(iz)) << 33;
  value ^= static_cast<uint64_t>(tier) * 0xd6e8feb86659fd93ULL;
  value = mix64(value);
  return value ? value : 1ULL;
}

} // namespace

void Runtime::ensure_spatial_index(Pack& pack) {
  if (!pack.spatial_bins.empty() || pack.tiles.empty()) return;

  // 256 m maximum cells keep the 2 km lookup bounded on old CPUs. The exact
  // radial test below still determines visibility.
  pack.spatial_cell_m = std::clamp(pack.settings.draw_distance_m * 0.25f, 64.0f, 256.0f);
  pack.spatial_bins.clear();
  pack.spatial_bins.reserve(std::max<size_t>(16, pack.tiles.size() / 24));
  if (pack.active_indices.capacity() < static_cast<size_t>(pack.settings.max_active_tiles))
    pack.active_indices.reserve(static_cast<size_t>(pack.settings.max_active_tiles));
  pack.active_epoch = 1;

  for (size_t i = 0; i < pack.tiles.size(); ++i) {
    const auto& tile = pack.tiles[i];
    const int cx = spatial_coord(tile.x, pack.spatial_cell_m);
    const int cz = spatial_coord(tile.z, pack.spatial_cell_m);
    pack.spatial_bins[spatial_key(cx, cz)].push_back(i);
  }

  if (!pack.spatial_log_done) {
    log("Spatial grass index ready for " + pack.name + ": " +
        std::to_string(pack.tiles.size()) + " candidate(s) in " +
        std::to_string(pack.spatial_bins.size()) + " bin(s)");
    pack.spatial_log_done = true;
  }
}

void Runtime::rebuild_stream_window(Pack& pack, float camera_x, float camera_z) {
  if (pack.objects.empty() || pack.local_areas.empty()) return;

  const float recenter = std::clamp(pack.settings.tile_size_m * 24.0f, 48.0f, 90.0f);
  pack.stream_recenter_m = recenter;
  if (pack.stream_initialized &&
      distance2(camera_x, camera_z, pack.stream_center_x, pack.stream_center_z) <= recenter * recenter)
    return;

  const bool first_stream = !pack.stream_initialized;
  const size_t budget = static_cast<size_t>(std::max(100, pack.settings.max_total_tiles));
  const auto stream_budget = lod::split_stream_budget(budget);

  struct StreamCandidate {
    Tile tile;
    float d2{};
    uint64_t priority{};
  };

  std::array<std::vector<StreamCandidate>, 4> passes;
  passes[0].reserve(stream_budget.near_dense * 2u);
  passes[1].reserve(stream_budget.near_outer * 2u);
  passes[2].reserve(stream_budget.mid * 2u);
  passes[3].reserve(stream_budget.far * 2u);

  const uint32_t pack_seed = grass_math::mix_hash(
      static_cast<uint32_t>(std::hash<std::string>{}(pack.name)));
  const float draw_radius = std::max(30.0f, pack.settings.draw_distance_m);
  const float animated_radius = std::clamp(pack.settings.animated_distance_m, 15.0f, draw_radius);
  const float mid_radius = lod::mid_radius(animated_radius, draw_radius);

  // Keep a genuinely dense patch around the camera, then spread cheaper grass
  // through the rest of the animated, mid and far rings. This is what lets a
  // fixed 1,600-instance budget visually reach 2 km instead of being consumed
  // within the first few dozen metres.
  const float dense_radius = std::min(
      animated_radius,
      std::clamp(pack.settings.tile_size_m *
                     std::sqrt(static_cast<float>(stream_budget.near_dense) / grass_math::pi) * 1.15f,
                 90.0f, 220.0f));

  const float near_outer_inner = dense_radius * 0.72f;
  const float mid_inner = std::max(dense_radius, animated_radius - 140.0f);
  const float mid_outer = std::min(draw_radius, mid_radius + 140.0f);
  const float far_inner = std::max(animated_radius, mid_radius - 170.0f);

  auto emit_pass = [&](size_t pass_index, float radius, float inner_radius,
                       float spacing_multiplier, uint8_t lod_tier, uint32_t key_tier,
                       float fade_in_m, float fade_out_m, float base_keep) {
    if (radius <= inner_radius || pass_index >= passes.size()) return;
    const float radius2 = radius * radius;
    const float inner2 = inner_radius * inner_radius;

    for (size_t area_index = 0; area_index < pack.local_areas.size(); ++area_index) {
      const auto& area = pack.local_areas[area_index];
      if (area.outer.size() < 3) continue;

      float area_min_x = area.outer.front().x;
      float area_max_x = area.outer.front().x;
      float area_min_z = area.outer.front().z;
      float area_max_z = area.outer.front().z;
      for (const auto& point : area.outer) {
        area_min_x = std::min(area_min_x, point.x);
        area_max_x = std::max(area_max_x, point.x);
        area_min_z = std::min(area_min_z, point.z);
        area_max_z = std::max(area_max_z, point.z);
      }
      if (area_max_x < camera_x - radius || area_min_x > camera_x + radius ||
          area_max_z < camera_z - radius || area_min_z > camera_z + radius)
        continue;

      const float density = std::clamp(pack.areas[area_index].density, 0.03f, 1.0f);
      const float base_spacing = pack.settings.tile_size_m /
                                 std::sqrt(std::max(0.03f, density));
      const float spacing = std::max(0.35f, base_spacing * spacing_multiplier);

      const float scan_min_x = std::max(camera_x - radius, area_min_x - spacing);
      const float scan_max_x = std::min(camera_x + radius, area_max_x + spacing);
      const float scan_min_z = std::max(camera_z - radius, area_min_z - spacing);
      const float scan_max_z = std::min(camera_z + radius, area_max_z + spacing);
      const int ix0 = static_cast<int>(std::floor(scan_min_x / spacing));
      const int ix1 = static_cast<int>(std::ceil(scan_max_x / spacing));
      const int iz0 = static_cast<int>(std::floor(scan_min_z / spacing));
      const int iz1 = static_cast<int>(std::ceil(scan_max_z / spacing));

      const uint32_t area_seed = pack_seed ^
          static_cast<uint32_t>(area_index + 1) * 2654435761u ^
          key_tier * 2246822519u;

      for (int iz = iz0; iz <= iz1; ++iz) {
        for (int ix = ix0; ix <= ix1; ++ix) {
          uint32_t seed = area_seed ^
                          static_cast<uint32_t>(ix) * 73856093u ^
                          static_cast<uint32_t>(iz) * 19349663u;
          const float jitter_x = (grass_math::hash01(seed) - 0.5f) * spacing * 0.58f;
          const float jitter_z = (grass_math::hash01(seed ^ 0x9e3779b9u) - 0.5f) * spacing * 0.58f;
          const grass_math::Point2 p{ix * spacing + jitter_x, iz * spacing + jitter_z};
          const float d2 = distance2(p.x, p.z, camera_x, camera_z);
          if (d2 > radius2 || d2 < inner2) continue;
          const float distance = std::sqrt(d2);
          const float keep = lod::ring_keep_probability(
              distance, inner_radius, radius, fade_in_m, fade_out_m, base_keep);
          if (grass_math::hash01(seed ^ 0xc2b2ae35u) > keep) continue;
          if (!grass_math::point_in_area_margin(p, area.outer, area.holes,
                                                pack.settings.boundary_margin_m))
            continue;
          if (pack.settings.scatter_mode == "clustered" &&
              !grass_math::clustered_keep(
                  p, pack.settings.cluster_spacing_m, pack.settings.cluster_radius_m,
                  pack.settings.cluster_probability,
                  area_seed ^ 0x85ebca6bu))
            continue;

          Tile tile;
          tile.x = p.x;
          tile.z = p.z;
          tile.heading = grass_math::hash01(seed ^ 0xa5a5a5a5u) * 360.0f;
          tile.phase = grass_math::hash01(seed ^ 0x36ef3720u) * grass_math::pi * 2.0f;
          tile.model_variant = static_cast<size_t>(
              grass_math::mix_hash(seed ^ 0x6c8e9cf5u)) %
              std::max<size_t>(1, pack.objects.size());
          tile.stream_key = stream_candidate_key(area_index, ix, iz, key_tier);
          tile.lod_tier = lod_tier;
          passes[pass_index].push_back({std::move(tile), d2,
                                        mix64(stream_candidate_key(area_index, ix, iz, key_tier) ^
                                              0xa0761d6478bd642fULL)});
        }
      }
    }
  };

  // Stage 1a: full-density near detail. No fade at the camera side.
  emit_pass(0, dense_radius, 0.0f, 1.0f, 0, 1u,
            0.0f, std::min(35.0f, dense_radius * 0.22f), 1.0f);

  // Stage 1b: still inside the configured animated radius, but substantially
  // cheaper. It overlaps the dense patch and fades into stage 2.
  emit_pass(1, animated_radius, near_outer_inner, 3.0f, 0, 2u,
            std::max(20.0f, dense_radius * 0.25f), 95.0f, 1.0f);

  // Stage 2: static/slow ring. At the v0.5 defaults this occupies roughly
  // 460-1340 m, with density cross-fades at both ends.
  emit_pass(2, mid_outer, mid_inner, 6.0f, 1, 3u,
            120.0f, 150.0f, 0.92f);

  // Stage 3: sparse far grass. A long outer fade avoids the obvious circular
  // pop wall that the old 914.4 m OBJ LOD produced.
  emit_pass(3, draw_radius, far_inner, 11.0f, 2, 4u,
            180.0f, std::min(300.0f, draw_radius * 0.16f), 0.82f);

  auto cap_pass = [](std::vector<StreamCandidate>& pass, size_t pass_budget,
                     bool nearest) {
    if (pass.size() <= pass_budget) return;
    auto comp = [nearest](const StreamCandidate& a, const StreamCandidate& b) {
      return nearest ? (a.d2 < b.d2) : (a.priority < b.priority);
    };
    std::nth_element(pass.begin(), pass.begin() + static_cast<std::ptrdiff_t>(pass_budget),
                     pass.end(), comp);
    pass.resize(pass_budget);
  };

  cap_pass(passes[0], stream_budget.near_dense, true);
  cap_pass(passes[1], stream_budget.near_outer, false);
  cap_pass(passes[2], stream_budget.mid, false);
  cap_pass(passes[3], stream_budget.far, false);

  std::vector<StreamCandidate> candidates;
  candidates.reserve(budget);
  for (auto& pass : passes)
    for (auto& candidate : pass)
      candidates.push_back(std::move(candidate));

  // Preserve overlapping streamed tiles, including their XPLM instance,
  // terrain height and animation phase. Only tiles entering/leaving the moving
  // window are created/destroyed, which avoids a mass pop during recentering.
  std::unordered_map<uint64_t, Tile> old_tiles;
  old_tiles.reserve(pack.tiles.size() * 2u + 1u);
  for (auto& tile : pack.tiles) {
    if (!tile.stream_key) {
      if (tile.instance) XPLMDestroyInstance(tile.instance);
      continue;
    }
    auto [it, inserted] = old_tiles.emplace(tile.stream_key, std::move(tile));
    if (!inserted && tile.instance) XPLMDestroyInstance(tile.instance);
  }

  std::vector<Tile> next_tiles;
  next_tiles.reserve(candidates.size());
  for (auto& candidate : candidates) {
    auto it = old_tiles.find(candidate.tile.stream_key);
    if (it != old_tiles.end()) {
      Tile reused = std::move(it->second);
      old_tiles.erase(it);
      reused.x = candidate.tile.x;
      reused.z = candidate.tile.z;
      reused.heading = candidate.tile.heading;
      reused.phase = candidate.tile.phase;
      reused.model_variant = candidate.tile.model_variant;
      reused.stream_key = candidate.tile.stream_key;
      reused.lod_tier = candidate.tile.lod_tier;
      reused.active_epoch = 0;
      next_tiles.push_back(std::move(reused));
    } else {
      next_tiles.push_back(std::move(candidate.tile));
    }
  }

  for (auto& [key, tile] : old_tiles) {
    (void)key;
    if (tile.instance) XPLMDestroyInstance(tile.instance);
  }

  pack.tiles.swap(next_tiles);
  pack.spatial_bins.clear();
  pack.active_indices.clear();
  pack.active_indices.reserve(static_cast<size_t>(pack.settings.max_active_tiles));
  for (size_t i = 0; i < pack.tiles.size(); ++i)
    if (pack.tiles[i].instance) pack.active_indices.push_back(i);
  pack.active_epoch = 1;
  pack.stream_center_x = camera_x;
  pack.stream_center_z = camera_z;
  pack.stream_initialized = true;
  ++pack.stream_rebuild_count;

  ensure_spatial_index(pack);
  if (first_stream) {
    log("3-stage grass LOD ready for " + pack.name + ": animated " +
        std::to_string(static_cast<int>(animated_radius)) + " m, mid " +
        std::to_string(static_cast<int>(mid_radius)) + " m, far " +
        std::to_string(static_cast<int>(draw_radius)) +
        " m with deterministic density fade; up to " + std::to_string(budget) +
        " streamed candidates");
  }
}

void Runtime::refresh_active_set_fast(Pack& pack, float camera_x, float camera_z) {
  if (pack.objects.empty() || pack.tiles.empty()) return;
  ensure_spatial_index(pack);

  struct Candidate {
    size_t index{};
    float d2{};
    uint64_t priority{};
  };
  std::array<std::vector<Candidate>, 3> tiers;
  const auto active_budget = lod::split_active_budget(
      static_cast<size_t>(std::max(3, pack.settings.max_active_tiles)));
  tiers[0].reserve(active_budget.near * 3u);
  tiers[1].reserve(active_budget.mid * 4u);
  tiers[2].reserve(active_budget.far * 5u);

  const float draw = pack.settings.draw_distance_m;
  const float draw2 = draw * draw;
  const int cx0 = spatial_coord(camera_x - draw, pack.spatial_cell_m);
  const int cx1 = spatial_coord(camera_x + draw, pack.spatial_cell_m);
  const int cz0 = spatial_coord(camera_z - draw, pack.spatial_cell_m);
  const int cz1 = spatial_coord(camera_z + draw, pack.spatial_cell_m);

  for (int cz = cz0; cz <= cz1; ++cz) {
    for (int cx = cx0; cx <= cx1; ++cx) {
      const auto it = pack.spatial_bins.find(spatial_key(cx, cz));
      if (it == pack.spatial_bins.end()) continue;
      for (const size_t index : it->second) {
        const auto& tile = pack.tiles[index];
        const float d2 = distance2(tile.x, tile.z, camera_x, camera_z);
        if (d2 > draw2) continue;
        const size_t tier = std::min<size_t>(2, tile.lod_tier);
        tiers[tier].push_back({index, d2, mix64(tile.stream_key ^ 0xe7037ed1a0b428dbULL)});
      }
    }
  }

  std::vector<Candidate> candidates;
  candidates.reserve(static_cast<size_t>(pack.settings.max_active_tiles));

  auto append_by_priority = [&](std::vector<Candidate>& src, size_t amount) {
    if (amount == 0 || src.empty()) return size_t{0};
    amount = std::min(amount, src.size());
    if (src.size() > amount) {
      std::nth_element(src.begin(), src.begin() + static_cast<std::ptrdiff_t>(amount), src.end(),
                       [](const Candidate& a, const Candidate& b) {
                         return a.priority < b.priority;
                       });
      src.resize(amount);
    }
    for (const auto& item : src) candidates.push_back(item);
    return src.size();
  };

  // Near budget: keep most instances close to the camera for density, but
  // reserve roughly 30% for a spread of animated grass farther out. Without
  // this reservation, nearest-N selection consumes the whole budget at ~50 m.
  {
    auto& near = tiers[0];
    const float core_radius = std::min(170.0f,
        std::max(65.0f, pack.settings.animated_distance_m * 0.24f));
    const float core2 = core_radius * core_radius;
    std::vector<Candidate> core;
    std::vector<Candidate> outer;
    core.reserve(near.size());
    outer.reserve(near.size());
    for (const auto& item : near) {
      if (item.d2 <= core2) core.push_back(item);
      else outer.push_back(item);
    }

    const size_t near_budget = active_budget.near;
    size_t core_target = std::min(core.size(),
        static_cast<size_t>(std::floor(static_cast<double>(near_budget) * 0.70)));
    if (core.size() > core_target) {
      std::nth_element(core.begin(), core.begin() + static_cast<std::ptrdiff_t>(core_target), core.end(),
                       [](const Candidate& a, const Candidate& b) { return a.d2 < b.d2; });
      core.resize(core_target);
    }
    for (const auto& item : core) candidates.push_back(item);

    size_t remaining = near_budget - core.size();
    const size_t outer_added = append_by_priority(outer, remaining);
    remaining -= std::min(remaining, outer_added);

    // If the scenery has little grass in the outer animated ring, spend the
    // unused share on additional nearby detail rather than leaving FPS unused.
    if (remaining > 0 && near.size() > core.size() + outer_added) {
      std::vector<Candidate> fallback;
      fallback.reserve(near.size());
      for (const auto& item : near) {
        if (item.d2 > core2) continue;
        bool already = false;
        for (const auto& chosen : core) {
          if (chosen.index == item.index) { already = true; break; }
        }
        if (!already) fallback.push_back(item);
      }
      if (fallback.size() > remaining) {
        std::nth_element(fallback.begin(), fallback.begin() + static_cast<std::ptrdiff_t>(remaining), fallback.end(),
                         [](const Candidate& a, const Candidate& b) { return a.d2 < b.d2; });
        fallback.resize(remaining);
      }
      for (const auto& item : fallback) candidates.push_back(item);
    }
  }

  append_by_priority(tiers[1], active_budget.mid);
  append_by_priority(tiers[2], active_budget.far);

  // If a ring contains no grass because of pavement/holes, let the remaining
  // instance budget be filled by the closest not-yet-selected candidates.
  if (candidates.size() < static_cast<size_t>(pack.settings.max_active_tiles)) {
    const size_t target = static_cast<size_t>(pack.settings.max_active_tiles);
    std::vector<Candidate> spare;
    spare.reserve(pack.tiles.size());
    for (const auto& tier : tiers) {
      for (const auto& item : tier) {
        bool selected = false;
        for (const auto& chosen : candidates) {
          if (chosen.index == item.index) { selected = true; break; }
        }
        if (!selected) spare.push_back(item);
      }
    }
    const size_t needed = std::min(target - candidates.size(), spare.size());
    if (needed > 0 && spare.size() > needed) {
      std::nth_element(spare.begin(), spare.begin() + static_cast<std::ptrdiff_t>(needed), spare.end(),
                       [](const Candidate& a, const Candidate& b) { return a.d2 < b.d2; });
      spare.resize(needed);
    }
    for (const auto& item : spare) candidates.push_back(item);
  }

  ++pack.active_epoch;
  if (pack.active_epoch == 0) {
    for (auto& tile : pack.tiles) tile.active_epoch = 0;
    pack.active_epoch = 1;
  }
  const uint32_t epoch = pack.active_epoch;
  for (const auto& candidate : candidates)
    pack.tiles[candidate.index].active_epoch = epoch;

  // Only inspect instances that were active last refresh. This removes the
  // old O(total candidates) destroy pass from the recurring camera refresh.
  for (const size_t index : pack.active_indices) {
    if (index >= pack.tiles.size()) continue;
    auto& tile = pack.tiles[index];
    if (tile.active_epoch == epoch) continue;
    if (tile.instance) XPLMDestroyInstance(tile.instance);
    tile.instance = nullptr;
    tile.positioned_once = false;
    tile.animated_last = false;
  }

  static const char* datarefs[] = {
      "oafish/ogr/grass/bend_x_0", "oafish/ogr/grass/bend_z_0",
      "oafish/ogr/grass/bend_x_1", "oafish/ogr/grass/bend_z_1",
      "oafish/ogr/grass/bend_x_2", "oafish/ogr/grass/bend_z_2",
      "oafish/ogr/grass/bend_x_3", "oafish/ogr/grass/bend_z_3", nullptr};

  std::vector<size_t> next_active;
  next_active.reserve(candidates.size());
  const float fallback_y = local_y_ref_ ? XPLMGetDataf(local_y_ref_) : 0.0f;

  for (const auto& candidate : candidates) {
    const size_t index = candidate.index;
    auto& tile = pack.tiles[index];
    if (!tile.instance) {
      XPLMObjectRef object = pack.objects[tile.model_variant % pack.objects.size()];
      tile.instance = XPLMCreateInstance(object, datarefs);
      if (!tile.instance) continue;
      tile.y = terrain_y(tile.x, tile.z, fallback_y, pack.settings.ground_offset_m);
      XPLMDrawInfo_t draw{};
      draw.structSize = sizeof(draw);
      draw.x = tile.x; draw.y = tile.y; draw.z = tile.z;
      draw.pitch = 0.0f; draw.heading = tile.heading; draw.roll = 0.0f;
      float neutral[8]{};
      XPLMInstanceSetPosition(tile.instance, &draw, neutral);
      tile.positioned_once = true;
      tile.animated_last = false;
    }
    next_active.push_back(index);
  }

  pack.active_indices.swap(next_active);
}

void Runtime::update_live_fast(float elapsed_seconds, float aircraft_heading_deg,
                               float aircraft_agl_m) {
  if (packs_.empty()) return;
  const float elapsed = std::clamp(elapsed_seconds, 0.0f, 0.5f);
  const float aircraft_x = local_x_ref_ ? XPLMGetDataf(local_x_ref_) : 0.0f;
  const float aircraft_z = local_z_ref_ ? XPLMGetDataf(local_z_ref_) : 0.0f;
  float camera_x = aircraft_x;
  float camera_z = aircraft_z;
  XPLMCameraPosition_t camera{};
  XPLMReadCameraPosition(&camera);
  if (std::isfinite(camera.x) && std::isfinite(camera.z)) {
    camera_x = camera.x;
    camera_z = camera.z;
  }

  engine_clock_ += elapsed;
  if (engine_clock_ >= 0.02f) {
    update_engines(engine_clock_, aircraft_x, aircraft_z, aircraft_heading_deg);
    engine_clock_ = 0.0f;
  }

  for (auto& pack : packs_) {
    const float hide_m = pack.settings.hide_aircraft_agl_ft * 0.3048f;
    if (pack.settings.hide_aircraft_agl_ft > 0.0f && aircraft_agl_m >= hide_m) {
      if (!pack.altitude_suspended) {
        destroy_pack_instances(pack);
        pack.active_indices.clear();
        pack.altitude_suspended = true;
      }
      pack.refresh_clock = 0.0f;
      pack.animation_clock = 0.0f;
      continue;
    }
    if (pack.altitude_suspended) {
      pack.altitude_suspended = false;
      pack.refresh_clock = pack.settings.refresh_interval_s;
      pack.animation_clock = pack.settings.animation_interval_s;
    }

    pack.refresh_clock += elapsed;
    pack.animation_clock += elapsed;
    if (pack.refresh_clock >= pack.settings.refresh_interval_s) {
      pack.refresh_clock = 0.0f;
      rebuild_stream_window(pack, camera_x, camera_z);
      refresh_active_set_fast(pack, camera_x, camera_z);
    }
    if (pack.animation_clock < pack.settings.animation_interval_s) continue;

    const float dt = pack.animation_clock;
    pack.animation_clock = 0.0f;
    cached_wind_ = ambient_wind_world(pack.settings.wind_strength,
                                      pack.settings.wind_full_bend_kt,
                                      pack.settings.weather_wind,
                                      &cached_wind_speed_mps_);

    // Animation is O(active grass), not O(all candidates). Mid/far candidates
    // are positioned once and then naturally remain static outside the
    // animated_distance_m test in update_tile().
    for (const size_t index : pack.active_indices) {
      if (index >= pack.tiles.size()) continue;
      auto& tile = pack.tiles[index];
      if (tile.instance)
        update_tile(pack, tile, camera_x, camera_z, aircraft_x, aircraft_z, dt);
    }
  }
}

void Runtime::ensure_replay_datarefs() {
  if (replay_refs_initialized_) return;
  replay_ref_ = XPLMFindDataRef("sim/time/is_in_replay");
  running_time_ref_ = XPLMFindDataRef("sim/time/total_running_time_sec");
  flight_time_ref_ = XPLMFindDataRef("sim/time/total_flight_time_sec");
  replay_refs_initialized_ = true;
  log("Replay recorder armed: 20 Hz, up to 20 minutes of wind + per-engine wash history");
}

void Runtime::record_replay_frame(float aircraft_x, float aircraft_y, float aircraft_z,
                                  float aircraft_heading_deg) {
  ReplayFrame frame;
  frame.running_time = running_time_ref_ ? XPLMGetDataf(running_time_ref_) : 0.0f;
  frame.flight_time = flight_time_ref_ ? XPLMGetDataf(flight_time_ref_) : frame.running_time;
  frame.aircraft_x = aircraft_x;
  frame.aircraft_y = aircraft_y;
  frame.aircraft_z = aircraft_z;
  frame.aircraft_heading = aircraft_heading_deg;

  float wx = wind_x_ref_ ? finite_or_zero(XPLMGetDataf(wind_x_ref_)) : 0.0f;
  float wz = wind_z_ref_ ? finite_or_zero(XPLMGetDataf(wind_z_ref_)) : 0.0f;
  float speed = std::hypot(wx, wz);
  if (speed < 0.01f && wind_dir_ref_ && wind_speed_ref_) {
    const float dir = finite_or_zero(XPLMGetDataf(wind_dir_ref_)) * grass_math::pi / 180.0f;
    speed = std::max(0.0f, finite_or_zero(XPLMGetDataf(wind_speed_ref_))) * 0.514444f;
    wx = -std::sin(dir) * speed;
    wz = std::cos(dir) * speed;
  }
  frame.wind_x_mps = wx;
  frame.wind_z_mps = wz;

  frame.engine_count = engine_count_ref_ ? std::clamp(XPLMGetDatai(engine_count_ref_), 0, 16) : 0;
  const float h = aircraft_heading_deg * grass_math::pi / 180.0f;
  const float c = std::cos(h);
  const float s = std::sin(h);

  for (int i = 0; i < frame.engine_count; ++i) {
    const auto& wake = engine_wakes_[static_cast<std::size_t>(i)];
    const float dx = wake.x - aircraft_x;
    const float dz = wake.z - aircraft_z;
    auto& state = frame.engines[static_cast<std::size_t>(i)];
    state.rel_x = dx * c + dz * s;
    state.rel_z = -dx * s + dz * c;
    state.rel_y = wake.y - aircraft_y;
    state.heading_delta = heading_delta_deg(wake.heading, aircraft_heading_deg);
    state.power = wake.power;
    state.rpm_ratio = wake.rpm_ratio;
  }

  replay_history_.push(frame);
}

bool Runtime::apply_replay_frame(float aircraft_x, float aircraft_y, float aircraft_z,
                                 float aircraft_heading_deg) {
  ReplayFrame frame;
  const float running = running_time_ref_ ? XPLMGetDataf(running_time_ref_) : 0.0f;
  const float flight = flight_time_ref_ ? XPLMGetDataf(flight_time_ref_) : running;
  if (!replay_history_.sample(running, flight, aircraft_x, aircraft_y, aircraft_z,
                              aircraft_heading_deg, frame)) {
    return false;
  }

  replay_wind_x_mps_ = frame.wind_x_mps;
  replay_wind_z_mps_ = frame.wind_z_mps;

  const float h = aircraft_heading_deg * grass_math::pi / 180.0f;
  const float c = std::cos(h);
  const float s = std::sin(h);
  const int count = std::clamp(frame.engine_count, 0, 16);

  for (int i = 0; i < 16; ++i) {
    auto& wake = engine_wakes_[static_cast<std::size_t>(i)];
    if (i >= count) {
      wake = {};
      continue;
    }
    const auto& state = frame.engines[static_cast<std::size_t>(i)];
    wake.x = aircraft_x + state.rel_x * c - state.rel_z * s;
    wake.z = aircraft_z + state.rel_x * s + state.rel_z * c;
    wake.y = aircraft_y + state.rel_y;
    wake.heading = aircraft_heading_deg + state.heading_delta;
    wake.power = state.power;
    wake.rpm_ratio = state.rpm_ratio;
  }
  return true;
}

grass_math::Vector2 Runtime::replay_wind_world(const Settings& settings,
                                                 float* speed_mps) const {
  if (speed_mps) *speed_mps = 0.0f;
  if (!settings.weather_wind) return {};
  const float speed = std::hypot(replay_wind_x_mps_, replay_wind_z_mps_);
  if (speed_mps) *speed_mps = speed;
  if (speed < 0.01f) return {};

  const float full_bend_mps = std::max(1.0f, settings.wind_full_bend_kt * 0.514444f);
  const float normalized = grass_math::clamp01(speed / full_bend_mps);
  const float response = std::pow(normalized, 0.62f);
  const float amount = response * settings.wind_strength;
  return {replay_wind_x_mps_ / speed * amount,
          replay_wind_z_mps_ / speed * amount};
}

void Runtime::update(float elapsed_seconds, float aircraft_heading_deg, float aircraft_agl_m) {
  if (packs_.empty()) return;
  ensure_replay_datarefs();

  const float elapsed = std::clamp(elapsed_seconds, 0.0f, 0.5f);
  const float aircraft_x = local_x_ref_ ? XPLMGetDataf(local_x_ref_) : 0.0f;
  const float aircraft_y = local_y_ref_ ? XPLMGetDataf(local_y_ref_) : 0.0f;
  const float aircraft_z = local_z_ref_ ? XPLMGetDataf(local_z_ref_) : 0.0f;
  const bool in_replay = replay_ref_ && XPLMGetDatai(replay_ref_) != 0;

  if (!in_replay) {
    if (replay_active_) {
      log("Replay ended; live RPM/N1, thrust and weather wind resumed");
      replay_active_ = false;
      replay_sample_missing_logged_ = false;
      engine_clock_ = 0.0f;
    }

    update_live_fast(elapsed_seconds, aircraft_heading_deg, aircraft_agl_m);

    replay_record_clock_ += elapsed;
    if (replay_record_clock_ >= 0.05f) {
      replay_record_clock_ = std::fmod(replay_record_clock_, 0.05f);
      record_replay_frame(aircraft_x, aircraft_y, aircraft_z, aircraft_heading_deg);
    }
    return;
  }

  if (!replay_active_) {
    replay_active_ = true;
    replay_sample_missing_logged_ = false;
    log("Replay detected; grass now follows recorded prop RPM / jet power / wind history (" +
        std::to_string(static_cast<int>(replay_history_.seconds_available())) + " s available)");
  }

  // Keep camera culling, terrain and LOD active in replay, but stop live engine
  // sampling from replacing the historical wake before the correction pass.
  engine_clock_ = -elapsed;
  update_live_fast(elapsed_seconds, aircraft_heading_deg, aircraft_agl_m);

  const bool have_frame = apply_replay_frame(aircraft_x, aircraft_y, aircraft_z,
                                             aircraft_heading_deg);
  if (!have_frame) {
    engine_wakes_ = {};
    replay_wind_x_mps_ = 0.0f;
    replay_wind_z_mps_ = 0.0f;
    if (!replay_sample_missing_logged_) {
      log("Replay position is outside OGR history; grass replay effects are neutral until the timeline re-enters the recorded window");
      replay_sample_missing_logged_ = true;
    }
  } else {
    replay_sample_missing_logged_ = false;
  }

  float camera_x = aircraft_x;
  float camera_z = aircraft_z;
  XPLMCameraPosition_t camera{};
  XPLMReadCameraPosition(&camera);
  if (std::isfinite(camera.x) && std::isfinite(camera.z)) {
    camera_x = camera.x;
    camera_z = camera.z;
  }

  // A zero-dt correction pass replaces only final bend inputs with historical
  // wind/engine state. Iterate only active grass so replay remains cheap too.
  for (auto& pack : packs_) {
    if (pack.altitude_suspended) continue;
    cached_wind_ = have_frame ? replay_wind_world(pack.settings, &cached_wind_speed_mps_)
                              : grass_math::Vector2{};
    if (!have_frame) cached_wind_speed_mps_ = 0.0f;
    for (const size_t index : pack.active_indices) {
      if (index >= pack.tiles.size()) continue;
      auto& tile = pack.tiles[index];
      if (tile.instance)
        update_tile(pack, tile, camera_x, camera_z, aircraft_x, aircraft_z, 0.0f);
    }
  }
}

} // namespace ogr
