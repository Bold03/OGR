#include "ogr/rotor.hpp"

#include <XPLMDataAccess.h>
#include <XPLMInstance.h>
#include <XPLMUtilities.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>

namespace {

constexpr float kRotorSampleIntervalS = 0.02f;
constexpr float kFallbackRotorRadiusM = 5.5f;
constexpr float kFallbackRotorRedlineRadS = 45.0f;
constexpr float kRotorMaxHeightM = 35.0f;
constexpr float kRotorStrength = 1.35f;

struct RotorHookState {
  bool initialized{};
  bool available{};
  bool availability_logged{};
  bool active_logged{};
  XPLMDataRef time_ref{};
  XPLMDataRef replay_ref{};
  XPLMDataRef clutch_ref{};
  XPLMDataRef local_x_ref{};
  XPLMDataRef local_y_ref{};
  XPLMDataRef local_z_ref{};
  XPLMDataRef heading_ref{};
  XPLMDataRef prop_speed_ref{};
  XPLMDataRef redline_ref{};
  XPLMDataRef disc_area_ref{};
  XPLMDataRef prop_dir_ref{};
  XPLMDataRef collective_ref{};
  float last_sample_time{-1000.0f};
  float center_x{};
  float center_y{};
  float center_z{};
  float radius_m{kFallbackRotorRadiusM};
  float rpm_ratio{};
  float collective{};
  float power{};
  float swirl_sign{1.0f};
  bool active{};
};

RotorHookState g_rotor;

float rotor_finite_or(float value, float fallback = 0.0f) {
  return std::isfinite(value) ? value : fallback;
}

double rotor_finite_or(double value, double fallback = 0.0) {
  return std::isfinite(value) ? value : fallback;
}

void rotor_log(const std::string& text) {
  const std::string line = "[OGR] " + text + "\n";
  XPLMDebugString(line.c_str());
}

void ensure_rotor_refs() {
  if (g_rotor.initialized) return;
  g_rotor.initialized = true;

  g_rotor.time_ref = XPLMFindDataRef("sim/time/total_running_time_sec");
  g_rotor.replay_ref = XPLMFindDataRef("sim/time/is_in_replay");
  g_rotor.clutch_ref = XPLMFindDataRef("sim/aircraft/artstability/acf_has_clutch");
  g_rotor.local_x_ref = XPLMFindDataRef("sim/flightmodel/position/local_x");
  g_rotor.local_y_ref = XPLMFindDataRef("sim/flightmodel/position/local_y");
  g_rotor.local_z_ref = XPLMFindDataRef("sim/flightmodel/position/local_z");
  g_rotor.heading_ref = XPLMFindDataRef("sim/flightmodel/position/psi");
  g_rotor.prop_speed_ref = XPLMFindDataRef("sim/flightmodel2/engines/prop_rotation_speed_rad_sec");
  g_rotor.redline_ref = XPLMFindDataRef("sim/aircraft/controls/acf_RSC_redline_prp");
  g_rotor.disc_area_ref = XPLMFindDataRef("sim/aircraft/prop/acf_discarea");
  g_rotor.prop_dir_ref = XPLMFindDataRef("sim/aircraft/prop/acf_prop_dir");
  g_rotor.collective_ref = XPLMFindDataRef("sim/cockpit2/engine/actuators/prop_ratio");

  g_rotor.available = g_rotor.clutch_ref && g_rotor.local_x_ref &&
                      g_rotor.local_y_ref && g_rotor.local_z_ref &&
                      g_rotor.heading_ref && g_rotor.prop_speed_ref;

  if (!g_rotor.availability_logged) {
    if (g_rotor.available) {
      rotor_log("Helicopter rotor-wash detector armed: rotor clutch + RPM datarefs found; collective/disc geometry are used when available");
    } else {
      rotor_log("Helicopter rotor wash unavailable: required native X-Plane rotor datarefs were not found");
    }
    g_rotor.availability_logged = true;
  }
}

float rotor_now() {
  ensure_rotor_refs();
  return g_rotor.time_ref ? rotor_finite_or(XPLMGetDataf(g_rotor.time_ref)) : 0.0f;
}

void sample_rotor_if_needed() {
  ensure_rotor_refs();
  if (!g_rotor.available) return;

  const float now = rotor_now();
  if (now >= g_rotor.last_sample_time &&
      now - g_rotor.last_sample_time < kRotorSampleIntervalS)
    return;

  float dt = kRotorSampleIntervalS;
  if (g_rotor.last_sample_time > -900.0f && now >= g_rotor.last_sample_time)
    dt = std::clamp(now - g_rotor.last_sample_time, 0.001f, 0.50f);
  if (now + 0.25f < g_rotor.last_sample_time)
    dt = kRotorSampleIntervalS;
  g_rotor.last_sample_time = now;

  const bool replay = g_rotor.replay_ref && XPLMGetDatai(g_rotor.replay_ref) != 0;
  const bool helicopter = XPLMGetDatai(g_rotor.clutch_ref) != 0;

  float target_power = 0.0f;
  if (!replay && helicopter) {
    std::array<float, 16> speed{};
    std::array<float, 16> area{};
    std::array<float, 16> direction{};
    std::array<float, 16> collective{};

    const int speed_count = XPLMGetDatavf(g_rotor.prop_speed_ref, speed.data(), 0, 16);
    const int area_count = g_rotor.disc_area_ref
        ? XPLMGetDatavf(g_rotor.disc_area_ref, area.data(), 0, 16) : 0;
    const int dir_count = g_rotor.prop_dir_ref
        ? XPLMGetDatavf(g_rotor.prop_dir_ref, direction.data(), 0, 16) : 0;
    const int collective_count = g_rotor.collective_ref
        ? XPLMGetDatavf(g_rotor.collective_ref, collective.data(), 0, 16) : 0;

    int rotor_index = 0;
    float largest_area = 0.0f;
    for (int i = 0; i < area_count; ++i) {
      const float candidate = std::max(0.0f, rotor_finite_or(area[static_cast<std::size_t>(i)]));
      if (candidate > largest_area) {
        largest_area = candidate;
        rotor_index = i;
      }
    }

    g_rotor.radius_m = ogr::rotor::radius_from_disc_area(largest_area, kFallbackRotorRadiusM);
    const float redline_raw = g_rotor.redline_ref
        ? rotor_finite_or(XPLMGetDataf(g_rotor.redline_ref)) : 0.0f;
    const float redline = ogr::rotor::rotor_reference_rad_s(redline_raw, kFallbackRotorRedlineRadS);
    const float rotor_speed = rotor_index < speed_count
        ? std::abs(rotor_finite_or(speed[static_cast<std::size_t>(rotor_index)])) : 0.0f;
    g_rotor.rpm_ratio = std::clamp(rotor_speed / redline, 0.0f, 1.20f);
    g_rotor.collective = rotor_index < collective_count
        ? std::clamp(std::abs(rotor_finite_or(collective[static_cast<std::size_t>(rotor_index)])), 0.0f, 1.0f)
        : 0.65f;
    g_rotor.swirl_sign = rotor_index < dir_count && rotor_finite_or(direction[static_cast<std::size_t>(rotor_index)]) < 0.0f
        ? -1.0f : 1.0f;

    target_power = ogr::rotor::power_from_state(g_rotor.rpm_ratio, g_rotor.collective);

    const double aircraft_x = rotor_finite_or(XPLMGetDatad(g_rotor.local_x_ref));
    const double aircraft_y = rotor_finite_or(XPLMGetDatad(g_rotor.local_y_ref));
    const double aircraft_z = rotor_finite_or(XPLMGetDatad(g_rotor.local_z_ref));
    g_rotor.center_x = static_cast<float>(aircraft_x);
    g_rotor.center_z = static_cast<float>(aircraft_z);

    const float hub_above_cg = std::clamp(g_rotor.radius_m * 0.24f, 1.1f, 3.0f);
    g_rotor.center_y = static_cast<float>(aircraft_y) + hub_above_cg;
  }

  const float tau = target_power > g_rotor.power ? 0.12f : 0.32f;
  const float alpha = 1.0f - std::exp(-dt / tau);
  g_rotor.power += (target_power - g_rotor.power) * alpha;
  if (g_rotor.power < 0.001f) g_rotor.power = 0.0f;
  g_rotor.active = helicopter && !replay && g_rotor.power > 0.015f;

  if (g_rotor.active && !g_rotor.active_logged) {
    rotor_log("Helicopter rotor wash active: main rotor estimated at " +
              std::to_string(static_cast<int>(std::lround(g_rotor.radius_m * 2.0f))) +
              " m diameter; radial main-rotor flow now overrides the old one-directional rear prop/engine cone nearby");
    g_rotor.active_logged = true;
  }
}

void ogr_rotor_instance_set_position(XPLMInstanceRef instance,
                                     const XPLMDrawInfo_t* position,
                                     const float* data) {
  if (!instance || !position) {
    XPLMInstanceSetPosition(instance, position, data);
    return;
  }

  sample_rotor_if_needed();
  if (!g_rotor.active) {
    XPLMInstanceSetPosition(instance, position, data);
    return;
  }

  // Wheel flattening still wins. Wheel datarefs intentionally exceed the normal
  // authored range in order to reach approximately 90 degrees.
  if (data) {
    float max_abs = 0.0f;
    for (int i = 0; i < 8; ++i)
      max_abs = std::max(max_abs, std::abs(rotor_finite_or(data[i])));
    if (max_abs > 1.55f) {
      XPLMInstanceSetPosition(instance, position, data);
      return;
    }
  }

  const float rotor_height = g_rotor.center_y - position->y;
  const float dx = position->x - g_rotor.center_x;
  const float dz = position->z - g_rotor.center_z;
  const float distance = std::hypot(dx, dz);
  const float dominance = ogr::rotor::radial_dominance(
      distance, g_rotor.radius_m, rotor_height, g_rotor.power);

  const auto wash = ogr::rotor::ground_wash(
      {position->x, position->z}, {g_rotor.center_x, g_rotor.center_z},
      rotor_height, g_rotor.radius_m, g_rotor.power,
      g_rotor.swirl_sign, kRotorStrength, kRotorMaxHeightM);

  if (wash.influence <= 0.0001f && dominance <= 0.0001f) {
    XPLMInstanceSetPosition(instance, position, data);
    return;
  }

  float modified[8]{};
  if (data) {
    for (int i = 0; i < 8; ++i) modified[i] = rotor_finite_or(data[i]);
  }

  // runtime.cpp's legacy prop/engine estimator interprets helicopter powerplants
  // like forward-facing propulsors. That is what produced the screenshot where
  // only grass behind the S-76 moved. Inside the rotor-dominant region, fade that
  // base vector away before adding the true radial field. Farther away, normal
  // weather and traffic animation resumes unchanged.
  const float base_scale = 1.0f - std::clamp(dominance * 0.96f, 0.0f, 0.96f);
  for (float& value : modified) value *= base_scale;

  const float now = rotor_now();
  const float spatial_phase = position->x * 0.23f + position->z * 0.17f;
  for (int group = 0; group < 4; ++group) {
    const float group_phase = spatial_phase + static_cast<float>(group) * 1.57f;
    const float flutter = 0.90f + 0.10f * std::sin(now * 16.0f + group_phase);
    const float variation = 0.94f + static_cast<float>(group) * 0.025f;
    const int xi = group * 2;
    const int zi = xi + 1;
    modified[xi] = std::clamp(modified[xi] + wash.bend.x * flutter * variation,
                              -1.25f, 1.25f);
    modified[zi] = std::clamp(modified[zi] + wash.bend.z * flutter * variation,
                              -1.25f, 1.25f);
  }

  XPLMInstanceSetPosition(instance, position, modified);
}

} // namespace

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4005)
#endif
#define XPLMInstanceSetPosition ogr_rotor_instance_set_position
#include "runtime_wheel.cpp"
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
#undef XPLMInstanceSetPosition
