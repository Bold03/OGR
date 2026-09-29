#include "ogr/runtime.hpp"
#include "ogr/traffic.hpp"

#include <XPLMDataAccess.h>
#include <XPLMCamera.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <string_view>
#include <vector>

namespace ogr {
namespace {

constexpr int kTcasSlots = 64;
constexpr int kTypeBytesPerSlot = 8;

float finite_or(float value, float fallback = 0.0f) {
  return std::isfinite(value) ? value : fallback;
}

float horizontal_distance2(float ax, float az, float bx, float bz) {
  const float dx = ax - bx;
  const float dz = az - bz;
  return dx * dx + dz * dz;
}

float heading_from_velocity(float vx, float vz) {
  if (!std::isfinite(vx) || !std::isfinite(vz) || std::hypot(vx, vz) < 0.2f)
    return 0.0f;
  float deg = std::atan2(vx, -vz) * 180.0f / grass_math::pi;
  if (deg < 0.0f) deg += 360.0f;
  return deg;
}

} // namespace

void Runtime::ensure_traffic_datarefs() {
  if (traffic_refs_initialized_) return;

  traffic_mode_s_ref_ = XPLMFindDataRef("sim/cockpit2/tcas/targets/modeS_id");
  traffic_x_ref_ = XPLMFindDataRef("sim/cockpit2/tcas/targets/position/x");
  traffic_y_ref_ = XPLMFindDataRef("sim/cockpit2/tcas/targets/position/y");
  traffic_z_ref_ = XPLMFindDataRef("sim/cockpit2/tcas/targets/position/z");
  traffic_vx_ref_ = XPLMFindDataRef("sim/cockpit2/tcas/targets/position/vx");
  traffic_vy_ref_ = XPLMFindDataRef("sim/cockpit2/tcas/targets/position/vy");
  traffic_vz_ref_ = XPLMFindDataRef("sim/cockpit2/tcas/targets/position/vz");
  traffic_psi_ref_ = XPLMFindDataRef("sim/cockpit2/tcas/targets/position/psi");
  traffic_wow_ref_ = XPLMFindDataRef("sim/cockpit2/tcas/targets/position/weight_on_wheels");
  traffic_throttle_ref_ = XPLMFindDataRef("sim/cockpit2/tcas/targets/position/throttle");
  traffic_icao_type_ref_ = XPLMFindDataRef("sim/cockpit2/tcas/targets/icao_type");

  traffic_refs_available_ = traffic_mode_s_ref_ && traffic_x_ref_ && traffic_y_ref_ && traffic_z_ref_;
  traffic_refs_initialized_ = true;

  if (traffic_refs_available_) {
    log("TCAS/online traffic grass interaction armed (read-only; IVAO/VATSIM/traffic-plugin compatible when targets are published to X-Plane TCAS)");
  } else {
    log("TCAS traffic datarefs unavailable; online traffic grass interaction disabled on this X-Plane build");
  }
}

void Runtime::update_traffic_wakes(float dt, float aircraft_x, float aircraft_z) {
  ensure_traffic_datarefs();

  int target_cap = 0;
  float update_interval = 0.10f;
  float strength = 0.0f;
  bool enabled = false;
  for (const auto& pack : packs_) {
    if (!pack.settings.traffic_wash) continue;
    enabled = true;
    target_cap = std::max(target_cap, pack.settings.max_traffic_targets);
    update_interval = std::min(update_interval, pack.settings.traffic_update_interval_s);
    strength = std::max(strength, pack.settings.traffic_wash_strength);
  }
  target_cap = std::clamp(target_cap, 0, static_cast<int>(traffic_wakes_.size()));
  update_interval = std::clamp(update_interval, 0.05f, 0.50f);

  const int user_engine_count = engine_count_ref_ ?
      std::clamp(XPLMGetDatai(engine_count_ref_), 0, static_cast<int>(engine_wakes_.size())) : 0;

  if (!enabled || !traffic_refs_available_ || target_cap <= 0) {
    traffic_wakes_ = {};
    return;
  }

  traffic_clock_ += std::max(0.0f, dt);
  const bool refresh = traffic_clock_ >= update_interval;
  if (refresh) {
    const float sample_dt = traffic_clock_;
    traffic_clock_ = std::fmod(traffic_clock_, update_interval);

    std::array<int, kTcasSlots> ids{};
    std::array<int, kTcasSlots> wow{};
    std::array<float, kTcasSlots> x{}, y{}, z{}, vx{}, vy{}, vz{}, psi{}, throttle{};
    std::array<char, kTcasSlots * kTypeBytesPerSlot> types{};

    const int id_count = XPLMGetDatavi(traffic_mode_s_ref_, ids.data(), 0, kTcasSlots);
    if (id_count <= 1) {
      traffic_wakes_ = {};
    } else {
      XPLMGetDatavf(traffic_x_ref_, x.data(), 0, kTcasSlots);
      XPLMGetDatavf(traffic_y_ref_, y.data(), 0, kTcasSlots);
      XPLMGetDatavf(traffic_z_ref_, z.data(), 0, kTcasSlots);
      if (traffic_vx_ref_) XPLMGetDatavf(traffic_vx_ref_, vx.data(), 0, kTcasSlots);
      if (traffic_vy_ref_) XPLMGetDatavf(traffic_vy_ref_, vy.data(), 0, kTcasSlots);
      if (traffic_vz_ref_) XPLMGetDatavf(traffic_vz_ref_, vz.data(), 0, kTcasSlots);
      if (traffic_psi_ref_) XPLMGetDatavf(traffic_psi_ref_, psi.data(), 0, kTcasSlots);
      if (traffic_throttle_ref_) XPLMGetDatavf(traffic_throttle_ref_, throttle.data(), 0, kTcasSlots);
      if (traffic_wow_ref_) XPLMGetDatavi(traffic_wow_ref_, wow.data(), 0, kTcasSlots);
      if (traffic_icao_type_ref_)
        XPLMGetDatab(traffic_icao_type_ref_, types.data(), 0, static_cast<int>(types.size()));

      float camera_x = aircraft_x;
      float camera_z = aircraft_z;
      XPLMCameraPosition_t camera{};
      XPLMReadCameraPosition(&camera);
      if (std::isfinite(camera.x) && std::isfinite(camera.z)) {
        camera_x = camera.x;
        camera_z = camera.z;
      }

      struct Candidate {
        TrafficWake wake;
        float distance2{};
      };
      std::vector<Candidate> candidates;
      candidates.reserve(static_cast<size_t>(target_cap) * 2u);

      const int count = std::min(id_count, kTcasSlots);
      constexpr float kInterestRadiusM = 2600.0f;
      constexpr float kInterestRadius2 = kInterestRadiusM * kInterestRadiusM;

      for (int i = 1; i < count; ++i) { // TCAS slot 0 is the user aircraft
        if (ids[i] == 0) continue;
        if (!std::isfinite(x[i]) || !std::isfinite(y[i]) || !std::isfinite(z[i])) continue;

        const float d_aircraft = horizontal_distance2(x[i], z[i], aircraft_x, aircraft_z);
        const float d_camera = horizontal_distance2(x[i], z[i], camera_x, camera_z);
        const float d2 = std::min(d_aircraft, d_camera);
        if (d2 > kInterestRadius2) continue;

        std::string_view type_view;
        if (traffic_icao_type_ref_) {
          const char* slot = types.data() + i * kTypeBytesPerSlot;
          type_view = std::string_view(slot, kTypeBytesPerSlot);
        }
        const auto profile = traffic::profile_for_icao(type_view);

        const float speed = std::hypot(finite_or(vx[i]), finite_or(vz[i]));
        const bool on_ground = traffic_wow_ref_ ? wow[i] != 0 : std::abs(finite_or(vy[i])) < 1.5f;
        const float target_power = traffic::estimated_power(
            traffic_throttle_ref_ ? throttle[i] : 0.0f, speed, on_ground, profile);

        TrafficWake wake;
        wake.mode_s_id = ids[i];
        wake.x = x[i];
        wake.y = y[i];
        wake.z = z[i];
        wake.heading = traffic_psi_ref_ && std::isfinite(psi[i])
                           ? psi[i]
                           : heading_from_velocity(vx[i], vz[i]);
        wake.power = target_power;
        wake.rpm_ratio = profile.rpm_ratio;
        wake.range_m = profile.range_m;
        wake.half_angle_deg = profile.half_angle_deg;
        wake.base_half_width_m = profile.base_half_width_m;
        wake.on_ground = on_ground;

        for (const auto& previous : traffic_wakes_) {
          if (previous.mode_s_id != wake.mode_s_id) continue;
          wake.power = grass_math::approach(previous.power, target_power, sample_dt,
                                            target_power > previous.power ? 0.25f : 0.65f);
          break;
        }

        if (wake.power < 0.005f && speed < 0.5f) continue;
        candidates.push_back({wake, d2});
      }

      if (static_cast<int>(candidates.size()) > target_cap) {
        std::nth_element(candidates.begin(), candidates.begin() + target_cap, candidates.end(),
                         [](const Candidate& a, const Candidate& b) {
                           return a.distance2 < b.distance2;
                         });
        candidates.resize(static_cast<size_t>(target_cap));
      }

      traffic_wakes_ = {};
      for (size_t i = 0; i < candidates.size() && i < traffic_wakes_.size(); ++i)
        traffic_wakes_[i] = candidates[i].wake;

      if (!traffic_detected_logged_ && !candidates.empty()) {
        log("TCAS traffic detected: " + std::to_string(candidates.size()) +
            " nearby target(s) can now disturb OGR grass");
        traffic_detected_logged_ = true;
      }
    }
  }

  // Feed cached online targets into the same proven engine-wash field used by
  // the user aircraft. This keeps the hot grass loop unchanged and cheap.
  int slot = user_engine_count;
  for (const auto& traffic : traffic_wakes_) {
    if (slot >= static_cast<int>(engine_wakes_.size())) break;
    if (!traffic.mode_s_id || traffic.power < 0.005f) continue;

    auto& wake = engine_wakes_[static_cast<size_t>(slot++)];
    wake.x = traffic.x;
    wake.y = traffic.y;
    wake.z = traffic.z;
    wake.heading = traffic.heading;
    wake.power = grass_math::clamp01(traffic.power * strength * (traffic.on_ground ? 1.0f : 0.35f));
    wake.rpm_ratio = traffic.rpm_ratio;
  }
}

grass_math::Vector2 Runtime::traffic_wash_at(const Settings& settings, const Tile& tile,
                                               float* wash_speed) const {
  if (wash_speed) *wash_speed = 0.0f;
  if (!settings.traffic_wash) return {};

  grass_math::Vector2 world{};
  float speed_sum = 0.0f;
  for (const auto& traffic : traffic_wakes_) {
    if (!traffic.mode_s_id || traffic.power < 0.005f) continue;

    const float height = traffic.y - tile.y;
    if (height < -4.0f || height > 28.0f) continue;

    const float h = traffic.heading * grass_math::pi / 180.0f;
    const float aft_distance = -(tile.x - traffic.x) * std::sin(h) +
                                (tile.z - traffic.z) * std::cos(h);
    const auto wash = grass_math::engine_wash(
        {tile.x, tile.z}, {traffic.x, traffic.z}, traffic.heading, traffic.power,
        traffic.range_m, traffic.half_angle_deg, traffic.base_half_width_m);

    float airborne_factor = 1.0f;
    if (!traffic.on_ground) {
      airborne_factor = grass_math::clamp01((16.0f - std::max(0.0f, height)) / 12.0f) * 0.45f;
      if (airborne_factor <= 0.0f) continue;
    }

    const float attenuation = settings.traffic_wash_strength * airborne_factor *
        grass_math::wake_ground_factor(height, aft_distance);
    world.x += wash.x * attenuation;
    world.z += wash.z * attenuation;
    speed_sum += std::hypot(wash.x, wash.z) * attenuation *
                 (12.0f + 30.0f * traffic.rpm_ratio);
  }

  if (wash_speed) *wash_speed = speed_sum;
  return world;
}

} // namespace ogr
