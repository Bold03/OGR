#include "ogr/rotor.hpp"

#include <XPLMDataAccess.h>
#include <XPLMInstance.h>
#include <XPLMUtilities.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <unordered_map>

namespace {

constexpr float kRotorSampleIntervalS = 0.02f;
constexpr float kFallbackRotorRadiusM = 5.5f;
constexpr float kFallbackRotorRedlineRadS = 45.0f;
constexpr float kRotorMaxHeightM = 35.0f;
constexpr float kRotorStrength = 1.65f;
constexpr float kWheelCrushDetect = 1.55f;
constexpr float kWheelCrushRelease = 1.20f;

struct CrushVisualState {
  ogr::grass_math::Vector2 direction{0.0f, -1.0f};
  bool has_direction{};
};

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
  XPLMDataRef point_speed_ref{};
  XPLMDataRef prop_rpm_ref{};
  XPLMDataRef engine_n2_ref{};
  XPLMDataRef redline_ref{};
  XPLMDataRef disc_area_ref{};
  XPLMDataRef point_xyz_ref{};
  XPLMDataRef prop_dir_ref{};
  XPLMDataRef collective_ref{};
  XPLMDataRef collective_all_ref{};
  float last_sample_time{-1000.0f};
  float center_x{};
  float center_y{};
  float center_z{};
  float radius_m{kFallbackRotorRadiusM};
  float rpm_ratio{};
  float collective{};
  float power{};
  float swirl_sign{1.0f};
  int rotor_index{};
  std::string speed_source{"none"};
  bool active{};
};

RotorHookState g_rotor;
std::unordered_map<XPLMInstanceRef, CrushVisualState> g_crush_visuals;

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

  // POINT_tacrad is the flight-model prop/rotor angular speed and is the most
  // reliable XP11 source for the default helicopters. Keep newer flightmodel2
  // and cockpit RPM sources as fallbacks for third-party aircraft.
  g_rotor.point_speed_ref = XPLMFindDataRef("sim/flightmodel/engine/POINT_tacrad");
  g_rotor.prop_speed_ref = XPLMFindDataRef("sim/flightmodel2/engines/prop_rotation_speed_rad_sec");
  g_rotor.prop_rpm_ref = XPLMFindDataRef("sim/cockpit2/engine/indicators/prop_speed_rpm");
  g_rotor.engine_n2_ref = XPLMFindDataRef("sim/flightmodel/engine/ENGN_N2_");

  g_rotor.redline_ref = XPLMFindDataRef("sim/aircraft/controls/acf_RSC_redline_prp");
  g_rotor.disc_area_ref = XPLMFindDataRef("sim/aircraft/prop/acf_discarea");
  g_rotor.point_xyz_ref = XPLMFindDataRef("sim/flightmodel/engine/POINT_XYZ");
  g_rotor.prop_dir_ref = XPLMFindDataRef("sim/aircraft/prop/acf_prop_dir");
  g_rotor.collective_ref = XPLMFindDataRef("sim/cockpit2/engine/actuators/prop_ratio");
  g_rotor.collective_all_ref = XPLMFindDataRef("sim/cockpit2/engine/actuators/prop_ratio_all");

  g_rotor.available = g_rotor.clutch_ref && g_rotor.local_x_ref &&
                      g_rotor.local_y_ref && g_rotor.local_z_ref &&
                      g_rotor.heading_ref &&
                      (g_rotor.point_speed_ref || g_rotor.prop_speed_ref ||
                       g_rotor.prop_rpm_ref || g_rotor.engine_n2_ref);

  if (!g_rotor.availability_logged) {
    if (g_rotor.available) {
      rotor_log("Helicopter rotor-wash detector armed: XP11 POINT_tacrad + flightmodel2/cockpit RPM fallbacks available; rotor disc geometry is used when present");
    } else {
      rotor_log("Helicopter rotor wash unavailable: required helicopter/rotor datarefs were not found");
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
    std::array<float, 16> point_speed{};
    std::array<float, 16> speed{};
    std::array<float, 16> rpm{};
    std::array<float, 16> n2{};
    std::array<float, 16> area{};
    std::array<float, 16> direction{};
    std::array<float, 16> collective{};
    std::array<float, 48> point_xyz{};

    const int point_speed_count = g_rotor.point_speed_ref
        ? XPLMGetDatavf(g_rotor.point_speed_ref, point_speed.data(), 0, 16) : 0;
    const int speed_count = g_rotor.prop_speed_ref
        ? XPLMGetDatavf(g_rotor.prop_speed_ref, speed.data(), 0, 16) : 0;
    const int rpm_count = g_rotor.prop_rpm_ref
        ? XPLMGetDatavf(g_rotor.prop_rpm_ref, rpm.data(), 0, 16) : 0;
    const int n2_count = g_rotor.engine_n2_ref
        ? XPLMGetDatavf(g_rotor.engine_n2_ref, n2.data(), 0, 16) : 0;
    const int area_count = g_rotor.disc_area_ref
        ? XPLMGetDatavf(g_rotor.disc_area_ref, area.data(), 0, 16) : 0;
    const int xyz_count = g_rotor.point_xyz_ref
        ? XPLMGetDatavf(g_rotor.point_xyz_ref, point_xyz.data(), 0, 48) : 0;
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
    g_rotor.rotor_index = rotor_index;
    g_rotor.radius_m = ogr::rotor::radius_from_disc_area(largest_area, kFallbackRotorRadiusM);

    const float redline_raw = g_rotor.redline_ref
        ? rotor_finite_or(XPLMGetDataf(g_rotor.redline_ref)) : 0.0f;

    float rotor_speed_rad_s = 0.0f;
    g_rotor.speed_source = "none";
    if (rotor_index < point_speed_count) {
      rotor_speed_rad_s = std::abs(rotor_finite_or(point_speed[static_cast<std::size_t>(rotor_index)]));
      if (rotor_speed_rad_s > 1.0f) g_rotor.speed_source = "POINT_tacrad";
    }
    if (rotor_speed_rad_s <= 1.0f && rotor_index < speed_count) {
      rotor_speed_rad_s = std::abs(rotor_finite_or(speed[static_cast<std::size_t>(rotor_index)]));
      if (rotor_speed_rad_s > 1.0f) g_rotor.speed_source = "flightmodel2";
    }
    if (rotor_speed_rad_s <= 1.0f && rotor_index < rpm_count) {
      const float rotor_rpm = std::abs(rotor_finite_or(rpm[static_cast<std::size_t>(rotor_index)]));
      if (rotor_rpm > 10.0f) {
        rotor_speed_rad_s = rotor_rpm * 2.0f * ogr::grass_math::pi / 60.0f;
        g_rotor.speed_source = "cockpit RPM";
      }
    }

    g_rotor.rpm_ratio = ogr::rotor::rotor_ratio_from_speed(
        rotor_speed_rad_s, redline_raw, kFallbackRotorRedlineRadS);

    // Final XP11 fallback: turbine N2 is not a rotor-speed measurement, but with
    // the rotor clutch engaged it is a much better indication of an operating
    // helicopter than producing no radial field at all on an aircraft whose
    // custom model does not publish a usable prop-speed array.
    if (g_rotor.rpm_ratio < 0.12f && n2_count > 0) {
      float max_n2 = 0.0f;
      for (int i = 0; i < n2_count; ++i)
        max_n2 = std::max(max_n2, std::abs(rotor_finite_or(n2[static_cast<std::size_t>(i)])));
      if (max_n2 > 15.0f) {
        g_rotor.rpm_ratio = std::clamp(max_n2 / 100.0f, 0.0f, 1.15f);
        g_rotor.speed_source = "N2 fallback";
      }
    }

    float collective_ratio = 0.0f;
    if (rotor_index < collective_count)
      collective_ratio = std::abs(rotor_finite_or(collective[static_cast<std::size_t>(rotor_index)]));
    if (g_rotor.collective_all_ref)
      collective_ratio = std::max(collective_ratio,
          std::abs(rotor_finite_or(XPLMGetDataf(g_rotor.collective_all_ref))));
    g_rotor.collective = std::clamp(collective_ratio, 0.0f, 1.0f);

    target_power = ogr::rotor::power_from_state(g_rotor.rpm_ratio, g_rotor.collective);

    const double aircraft_x = rotor_finite_or(XPLMGetDatad(g_rotor.local_x_ref));
    const double aircraft_y = rotor_finite_or(XPLMGetDatad(g_rotor.local_y_ref));
    const double aircraft_z = rotor_finite_or(XPLMGetDatad(g_rotor.local_z_ref));
    const float heading_deg = rotor_finite_or(XPLMGetDataf(g_rotor.heading_ref));
    const float h = heading_deg * ogr::grass_math::pi / 180.0f;
    const float c = std::cos(h);
    const float s = std::sin(h);

    bool used_point_xyz = false;
    const int xyz_base = rotor_index * 3;
    if (xyz_base >= 0 && xyz_base + 2 < xyz_count) {
      const float local_x = rotor_finite_or(point_xyz[static_cast<std::size_t>(xyz_base)]);
      const float local_y = rotor_finite_or(point_xyz[static_cast<std::size_t>(xyz_base + 1)]);
      const float local_z = rotor_finite_or(point_xyz[static_cast<std::size_t>(xyz_base + 2)]);
      if (std::abs(local_x) < 80.0f && std::abs(local_y) < 80.0f && std::abs(local_z) < 80.0f) {
        g_rotor.center_x = static_cast<float>(aircraft_x) + local_x * c - local_z * s;
        g_rotor.center_z = static_cast<float>(aircraft_z) + local_x * s + local_z * c;
        g_rotor.center_y = static_cast<float>(aircraft_y) + local_y;
        used_point_xyz = true;
      }
    }

    if (!used_point_xyz) {
      g_rotor.center_x = static_cast<float>(aircraft_x);
      g_rotor.center_z = static_cast<float>(aircraft_z);
      const float hub_above_cg = std::clamp(g_rotor.radius_m * 0.24f, 1.1f, 3.0f);
      g_rotor.center_y = static_cast<float>(aircraft_y) + hub_above_cg;
    }

    g_rotor.swirl_sign = rotor_index < dir_count &&
        rotor_finite_or(direction[static_cast<std::size_t>(rotor_index)]) < 0.0f
        ? -1.0f : 1.0f;
  }

  const float tau = target_power > g_rotor.power ? 0.10f : 0.30f;
  const float alpha = 1.0f - std::exp(-dt / tau);
  g_rotor.power += (target_power - g_rotor.power) * alpha;
  if (g_rotor.power < 0.001f) g_rotor.power = 0.0f;
  g_rotor.active = helicopter && !replay && g_rotor.power > 0.010f;

  if (g_rotor.active && !g_rotor.active_logged) {
    rotor_log("Helicopter rotor wash active: main rotor source=" + g_rotor.speed_source +
              ", index=" + std::to_string(g_rotor.rotor_index) +
              ", RPM ratio=" + std::to_string(g_rotor.rpm_ratio) +
              ", collective=" + std::to_string(g_rotor.collective) +
              ", diameter=" +
              std::to_string(static_cast<int>(std::lround(g_rotor.radius_m * 2.0f))) +
              " m; radial 360-degree flow overrides the legacy rear cone nearby");
    g_rotor.active_logged = true;
  }
}

bool stabilize_crushed_grass(XPLMInstanceRef instance,
                             const XPLMDrawInfo_t* position,
                             const float* data) {
  if (!instance || !position || !data) return false;

  float max_abs = 0.0f;
  ogr::grass_math::Vector2 average{};
  for (int group = 0; group < 4; ++group) {
    const float x = rotor_finite_or(data[group * 2]);
    const float z = rotor_finite_or(data[group * 2 + 1]);
    average.x += x;
    average.z += z;
    max_abs = std::max(max_abs, std::max(std::abs(x), std::abs(z)));
  }

  if (max_abs <= kWheelCrushDetect) {
    if (max_abs < kWheelCrushRelease)
      g_crush_visuals.erase(instance);
    return false;
  }

  average.x *= 0.25f;
  average.z *= 0.25f;
  auto& visual = g_crush_visuals[instance];
  visual.direction = ogr::rotor::stabilize_crush_direction(
      visual.direction, average, visual.has_direction);
  visual.has_direction = true;

  float modified[8]{};
  for (int group = 0; group < 4; ++group) {
    const int xi = group * 2;
    const int zi = xi + 1;
    const float x = rotor_finite_or(data[xi]);
    const float z = rotor_finite_or(data[zi]);
    const float magnitude = std::hypot(x, z);
    modified[xi] = visual.direction.x * magnitude;
    modified[zi] = visual.direction.z * magnitude;
  }

  XPLMInstanceSetPosition(instance, position, modified);
  return true;
}

void ogr_rotor_instance_set_position(XPLMInstanceRef instance,
                                     const XPLMDrawInfo_t* position,
                                     const float* data) {
  if (!instance || !position) {
    XPLMInstanceSetPosition(instance, position, data);
    return;
  }

  // This visual lock is independent of helicopter detection. It removes the
  // left/right alternating crush seen when a grass clump is equally close to
  // two user-gear contacts, while preserving the full ~90-degree magnitude.
  if (stabilize_crushed_grass(instance, position, data))
    return;

  sample_rotor_if_needed();
  if (!g_rotor.active) {
    XPLMInstanceSetPosition(instance, position, data);
    return;
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

  // The screenshot bug was caused by the fixed-wing engine estimator still
  // dominating behind the helicopter. Within the main-rotor field, remove that
  // directional base motion before adding a rotor-centered radial vector.
  const float base_scale = 1.0f - std::clamp(dominance, 0.0f, 1.0f);
  for (float& value : modified) value *= base_scale;

  const float now = rotor_now();
  const float spatial_phase = position->x * 0.23f + position->z * 0.17f;
  for (int group = 0; group < 4; ++group) {
    const float group_phase = spatial_phase + static_cast<float>(group) * 1.57f;
    const float flutter = 0.91f + 0.09f * std::sin(now * 15.0f + group_phase);
    const float variation = 0.96f + static_cast<float>(group) * 0.018f;
    const int xi = group * 2;
    const int zi = xi + 1;
    modified[xi] = std::clamp(modified[xi] + wash.bend.x * flutter * variation,
                              -1.35f, 1.35f);
    modified[zi] = std::clamp(modified[zi] + wash.bend.z * flutter * variation,
                              -1.35f, 1.35f);
  }

  XPLMInstanceSetPosition(instance, position, modified);
}

void ogr_rotor_destroy_instance(XPLMInstanceRef instance) {
  if (instance) g_crush_visuals.erase(instance);
  XPLMDestroyInstance(instance);
}

} // namespace

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4005)
#endif
#define XPLMInstanceSetPosition ogr_rotor_instance_set_position
#define XPLMDestroyInstance ogr_rotor_destroy_instance
#include "runtime_wheel.cpp"
#undef XPLMDestroyInstance
#if defined(_MSC_VER)
#pragma warning(pop)
#endif
#undef XPLMInstanceSetPosition
