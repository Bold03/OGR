#include "ogr/runtime.hpp"

// Reuse the proven v0.1 runtime implementation, but compile its update() body
// under a private name. The public wrapper below records live engine/wind state
// and re-applies that state while X-Plane is in replay.
#define update update_live
#include "runtime.cpp"
#undef update

#include <algorithm>
#include <cmath>

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

} // namespace

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
    // Inverse of the aircraft-local -> X-Plane-local transform used by
    // update_engines(). Recording relative coordinates survives local-origin
    // shifts and makes replay seeking robust around a scenery pack.
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

    update_live(elapsed_seconds, aircraft_heading_deg, aircraft_agl_m);

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

  // Keep the normal camera culling, LOD, terrain and altitude suspension logic.
  // Prevent its live engine sampler from replacing the replay wake before the
  // correction pass below.
  engine_clock_ = -elapsed;
  update_live(elapsed_seconds, aircraft_heading_deg, aircraft_agl_m);

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

  // update_live() already advanced the phase and managed instances this frame.
  // A zero-dt correction pass replaces only the final bend inputs with the
  // historical wind/engine state, so replay does not double-speed animation.
  for (auto& pack : packs_) {
    if (pack.altitude_suspended) continue;
    cached_wind_ = have_frame ? replay_wind_world(pack.settings, &cached_wind_speed_mps_)
                              : grass_math::Vector2{};
    if (!have_frame) cached_wind_speed_mps_ = 0.0f;
    for (auto& tile : pack.tiles) {
      if (tile.instance)
        update_tile(pack, tile, camera_x, camera_z, aircraft_x, aircraft_z, 0.0f);
    }
  }
}

} // namespace ogr
