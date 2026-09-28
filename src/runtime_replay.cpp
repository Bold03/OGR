#include "ogr/runtime.hpp"

// Reuse the proven runtime implementation for loading, geometry generation,
// terrain probing and object handling. Its original update() body is kept as
// update_live(), while the public wrapper below uses a spatially indexed hot
// path plus replay recording/playback.
#define update update_live
#include "runtime.cpp"
#undef update

#include <algorithm>
#include <cmath>
#include <cstdint>

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

} // namespace

void Runtime::ensure_spatial_index(Pack& pack) {
  if (!pack.spatial_bins.empty() || pack.tiles.empty()) return;

  // Large cells keep hash lookups low; the exact draw-radius test below still
  // determines visibility. The range is conservative for old CPUs/iGPUs.
  pack.spatial_cell_m = std::clamp(pack.settings.draw_distance_m * 0.25f, 64.0f, 256.0f);
  pack.spatial_bins.clear();
  pack.spatial_bins.reserve(std::max<size_t>(16, pack.tiles.size() / 24));
  pack.active_indices.clear();
  pack.active_indices.reserve(static_cast<size_t>(pack.settings.max_active_tiles));
  pack.active_epoch = 1;

  for (size_t i = 0; i < pack.tiles.size(); ++i) {
    const auto& tile = pack.tiles[i];
    const int cx = spatial_coord(tile.x, pack.spatial_cell_m);
    const int cz = spatial_coord(tile.z, pack.spatial_cell_m);
    pack.spatial_bins[spatial_key(cx, cz)].push_back(i);
  }

  log("Spatial grass index ready for " + pack.name + ": " +
      std::to_string(pack.tiles.size()) + " candidate(s) in " +
      std::to_string(pack.spatial_bins.size()) + " bin(s)");
}

void Runtime::refresh_active_set_fast(Pack& pack, float camera_x, float camera_z) {
  if (pack.objects.empty() || pack.tiles.empty()) return;
  ensure_spatial_index(pack);

  struct Candidate { size_t index; float d2; };
  std::vector<Candidate> candidates;
  candidates.reserve(static_cast<size_t>(pack.settings.max_active_tiles) * 2u);

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
        if (d2 <= draw2) candidates.push_back({index, d2});
      }
    }
  }

  if (static_cast<int>(candidates.size()) > pack.settings.max_active_tiles) {
    std::nth_element(candidates.begin(),
                     candidates.begin() + pack.settings.max_active_tiles,
                     candidates.end(),
                     [](const Candidate& a, const Candidate& b) { return a.d2 < b.d2; });
    candidates.resize(static_cast<size_t>(pack.settings.max_active_tiles));
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
    ensure_spatial_index(pack);
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
      refresh_active_set_fast(pack, camera_x, camera_z);
    }
    if (pack.animation_clock < pack.settings.animation_interval_s) continue;

    const float dt = pack.animation_clock;
    pack.animation_clock = 0.0f;
    cached_wind_ = ambient_wind_world(pack.settings.wind_strength,
                                      pack.settings.wind_full_bend_kt,
                                      pack.settings.weather_wind,
                                      &cached_wind_speed_mps_);

    // Animation is now O(active grass), not O(all candidates).
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
