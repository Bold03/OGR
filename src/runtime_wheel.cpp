#include "ogr/runtime.hpp"
#include "ogr/wheel.hpp"

#include <XPLMDataAccess.h>
#include <XPLMGraphics.h>
#include <XPLMInstance.h>
#include <XPLMScenery.h>
#include <XPLMUtilities.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <unordered_map>

namespace {

constexpr int kWheelSlots = 10;
constexpr float kSampleIntervalS = 0.02f;
constexpr float kRecoveryS = 5.5f;
// The exported grass blades reach roughly 32 degrees when a bend dataref has
// magnitude 1. X-Plane extrapolates OBJ animation keyframes outside that range,
// so a magnitude near 90/32 gives an actual ~90 degree blade rotation while
// still pivoting every blade around its own authored base instead of tilting
// the whole grass tile.
constexpr float kAuthoredFullBendDeg = 32.0f;
constexpr float kWheelFlatDeg = 90.0f;
constexpr float kWheelFlatDataref = kWheelFlatDeg / kAuthoredFullBendDeg;
constexpr float kCrushSinkM = 0.015f;

struct WheelContact {
  bool active{};
  float x{};
  float z{};
  float dir_x{};
  float dir_z{-1.0f};
  float radius_m{1.35f};
};

struct InstanceFlattenState {
  float amount{};
  float dir_x{};
  float dir_z{-1.0f};
  float last_time{-1.0f};
};

struct WheelRuntimeState {
  bool initialized{};
  bool log_written{};
  bool contact_logged{};
  bool available{};
  XPLMDataRef time_ref{};
  XPLMDataRef aircraft_x_ref{};
  XPLMDataRef aircraft_z_ref{};
  XPLMDataRef heading_ref{};
  XPLMDataRef on_ground_ref{};
  XPLMDataRef deflection_ref{};
  XPLMDataRef deploy_ref{};
  XPLMDataRef gear_x_ref{};
  XPLMDataRef gear_z_ref{};
  XPLMDataRef tire_radius_ref{};
  float last_sample_time{-1000.0f};
  std::array<float, kWheelSlots> previous_x{};
  std::array<float, kWheelSlots> previous_z{};
  std::array<float, kWheelSlots> previous_dir_x{};
  std::array<float, kWheelSlots> previous_dir_z{};
  std::array<unsigned char, kWheelSlots> previous_valid{};
  std::array<WheelContact, kWheelSlots> contacts{};
  std::unordered_map<XPLMInstanceRef, InstanceFlattenState> instance_states;
};

WheelRuntimeState g_wheels;

float finite_or(float value, float fallback = 0.0f) {
  return std::isfinite(value) ? value : fallback;
}

void write_log(const char* message) {
  XPLMDebugString(message);
}

void ensure_wheel_refs() {
  if (g_wheels.initialized) return;
  g_wheels.initialized = true;

  g_wheels.time_ref = XPLMFindDataRef("sim/time/total_running_time_sec");
  g_wheels.aircraft_x_ref = XPLMFindDataRef("sim/flightmodel/position/local_x");
  g_wheels.aircraft_z_ref = XPLMFindDataRef("sim/flightmodel/position/local_z");
  g_wheels.heading_ref = XPLMFindDataRef("sim/flightmodel/position/psi");
  g_wheels.on_ground_ref = XPLMFindDataRef("sim/flightmodel2/gear/on_ground");
  g_wheels.deflection_ref = XPLMFindDataRef("sim/flightmodel2/gear/tire_vertical_deflection_mtr");
  g_wheels.deploy_ref = XPLMFindDataRef("sim/flightmodel2/gear/deploy_ratio");
  g_wheels.gear_x_ref = XPLMFindDataRef("sim/aircraft/parts/acf_gear_xnodef");
  g_wheels.gear_z_ref = XPLMFindDataRef("sim/aircraft/parts/acf_gear_znodef");
  g_wheels.tire_radius_ref = XPLMFindDataRef("sim/aircraft/parts/acf_gear_tirrad");

  g_wheels.available = g_wheels.aircraft_x_ref && g_wheels.aircraft_z_ref &&
                       g_wheels.heading_ref && g_wheels.gear_x_ref &&
                       g_wheels.gear_z_ref &&
                       (g_wheels.on_ground_ref || g_wheels.deflection_ref);

  if (!g_wheels.log_written) {
    if (g_wheels.available) {
      write_log("[OGR] Wheel-track grass interaction armed: touching gear holds contacted grass near 90 degrees even while parked; recovery starts after the wheel leaves\n");
    } else {
      write_log("[OGR] Wheel-track grass interaction unavailable: required X-Plane gear datarefs were not found\n");
    }
    g_wheels.log_written = true;
  }
}

float current_time() {
  ensure_wheel_refs();
  return g_wheels.time_ref ? finite_or(XPLMGetDataf(g_wheels.time_ref), 0.0f) : 0.0f;
}

void sample_wheels_if_needed() {
  ensure_wheel_refs();
  if (!g_wheels.available) return;

  const float now = current_time();
  if (now >= g_wheels.last_sample_time &&
      now - g_wheels.last_sample_time < kSampleIntervalS)
    return;

  if (now + 0.25f < g_wheels.last_sample_time) {
    g_wheels.previous_valid.fill(0);
    g_wheels.instance_states.clear();
  }
  g_wheels.last_sample_time = now;

  std::array<int, kWheelSlots> on_ground{};
  std::array<float, kWheelSlots> deflection{};
  std::array<float, kWheelSlots> deploy{};
  std::array<float, kWheelSlots> gear_x{};
  std::array<float, kWheelSlots> gear_z{};
  std::array<float, kWheelSlots> tire_radius{};

  const int ground_count = g_wheels.on_ground_ref
      ? XPLMGetDatavi(g_wheels.on_ground_ref, on_ground.data(), 0, kWheelSlots) : 0;
  const int deflection_count = g_wheels.deflection_ref
      ? XPLMGetDatavf(g_wheels.deflection_ref, deflection.data(), 0, kWheelSlots) : 0;
  const int deploy_count = g_wheels.deploy_ref
      ? XPLMGetDatavf(g_wheels.deploy_ref, deploy.data(), 0, kWheelSlots) : 0;
  const int x_count = XPLMGetDatavf(g_wheels.gear_x_ref, gear_x.data(), 0, kWheelSlots);
  const int z_count = XPLMGetDatavf(g_wheels.gear_z_ref, gear_z.data(), 0, kWheelSlots);
  const int radius_count = g_wheels.tire_radius_ref
      ? XPLMGetDatavf(g_wheels.tire_radius_ref, tire_radius.data(), 0, kWheelSlots) : 0;

  const float aircraft_x = finite_or(XPLMGetDataf(g_wheels.aircraft_x_ref));
  const float aircraft_z = finite_or(XPLMGetDataf(g_wheels.aircraft_z_ref));
  const float heading_deg = finite_or(XPLMGetDataf(g_wheels.heading_ref));
  const float h = heading_deg * ogr::grass_math::pi / 180.0f;
  const float c = std::cos(h);
  const float s = std::sin(h);
  const float fallback_forward_x = s;
  const float fallback_forward_z = -c;
  int active_contacts = 0;

  for (int i = 0; i < kWheelSlots; ++i) {
    auto& contact = g_wheels.contacts[static_cast<std::size_t>(i)];
    contact = {};
    const std::size_t index = static_cast<std::size_t>(i);

    if (i >= x_count || i >= z_count) {
      g_wheels.previous_valid[index] = 0;
      continue;
    }

    bool touching = false;
    if (i < ground_count) touching = on_ground[index] != 0;
    if (!touching && i < deflection_count)
      touching = finite_or(deflection[index]) > 0.001f;
    if (i < deploy_count && finite_or(deploy[index], 1.0f) < 0.45f)
      touching = false;

    if (!touching) {
      g_wheels.previous_valid[index] = 0;
      continue;
    }

    const float local_x = finite_or(gear_x[index]);
    const float local_z = finite_or(gear_z[index]);
    const float world_x = aircraft_x + local_x * c - local_z * s;
    const float world_z = aircraft_z + local_x * s + local_z * c;

    float dir_x = fallback_forward_x;
    float dir_z = fallback_forward_z;
    if (g_wheels.previous_valid[index]) {
      const float dx = world_x - g_wheels.previous_x[index];
      const float dz = world_z - g_wheels.previous_z[index];
      const float travel = std::hypot(dx, dz);
      if (travel > 0.002f && travel < 25.0f) {
        ogr::wheel::normalize_or(dx, dz, fallback_forward_x, fallback_forward_z,
                                 dir_x, dir_z);
        g_wheels.previous_dir_x[index] = dir_x;
        g_wheels.previous_dir_z[index] = dir_z;
      } else {
        ogr::wheel::normalize_or(g_wheels.previous_dir_x[index],
                                 g_wheels.previous_dir_z[index],
                                 fallback_forward_x, fallback_forward_z,
                                 dir_x, dir_z);
      }
    } else {
      g_wheels.previous_dir_x[index] = fallback_forward_x;
      g_wheels.previous_dir_z[index] = fallback_forward_z;
    }

    g_wheels.previous_x[index] = world_x;
    g_wheels.previous_z[index] = world_z;
    g_wheels.previous_valid[index] = 1;

    const float tire = i < radius_count ? std::abs(finite_or(tire_radius[index])) : 0.35f;
    contact.active = true;
    contact.x = world_x;
    contact.z = world_z;
    contact.dir_x = dir_x;
    contact.dir_z = dir_z;
    contact.radius_m = std::clamp(1.15f + tire * 0.70f, 1.15f, 1.85f);
    ++active_contacts;
  }

  if (active_contacts > 0 && !g_wheels.contact_logged) {
    const std::string line = "[OGR] Wheel contact detected: " +
        std::to_string(active_contacts) +
        " gear contact(s) will keep grass flattened while the tires remain on it\n";
    XPLMDebugString(line.c_str());
    g_wheels.contact_logged = true;
  }
}

void real_instance_set_position(XPLMInstanceRef instance,
                                const XPLMDrawInfo_t* position,
                                const float* data) {
  XPLMInstanceSetPosition(instance, position, data);
}

void real_destroy_instance(XPLMInstanceRef instance) {
  XPLMDestroyInstance(instance);
}

void ogr_instance_set_position(XPLMInstanceRef instance,
                               const XPLMDrawInfo_t* position,
                               const float* data) {
  if (!instance || !position) {
    real_instance_set_position(instance, position, data);
    return;
  }

  sample_wheels_if_needed();
  if (!g_wheels.available) {
    real_instance_set_position(instance, position, data);
    return;
  }

  const float now = current_time();
  auto& state = g_wheels.instance_states[instance];
  if (state.last_time >= 0.0f) {
    if (now + 0.25f < state.last_time) {
      state = {};
      state.last_time = now;
    } else {
      const float dt = std::clamp(now - state.last_time, 0.0f, 0.5f);
      state.amount = ogr::wheel::recover_linear(state.amount, dt, kRecoveryS);
      state.last_time = now;
    }
  } else {
    state.last_time = now;
  }

  float best = 0.0f;
  const WheelContact* best_contact = nullptr;
  for (const auto& contact : g_wheels.contacts) {
    if (!contact.active) continue;
    const float dx = position->x - contact.x;
    const float dz = position->z - contact.z;
    const float weight = ogr::wheel::contact_weight(std::hypot(dx, dz), contact.radius_m);
    if (weight > best) {
      best = weight;
      best_contact = &contact;
    }
  }

  if (best_contact && best > 0.0f) {
    if (best >= state.amount * 0.70f) {
      state.dir_x = best_contact->dir_x;
      state.dir_z = best_contact->dir_z;
    }
    // Static tire pressure is a continuous contact, not a one-shot track.
    // Re-applying the spatial contact weight every update keeps the grass down
    // indefinitely while the wheel remains over it. Recovery only wins after
    // the wheel moves away or loses ground contact.
    state.amount = std::max(state.amount, best);
  }

  if (state.amount <= 0.001f) {
    if (state.amount <= 0.0f && best <= 0.0f)
      g_wheels.instance_states.erase(instance);
    real_instance_set_position(instance, position, data);
    return;
  }

  float modified[8]{};
  if (data) {
    for (int i = 0; i < 8; ++i) modified[i] = finite_or(data[i]);
  }

  // Use the OBJ's own per-blade bend pivots for wheel flattening. Full contact
  // takes the blade to roughly 90 degrees whether the aircraft is rolling or
  // parked. The last meaningful wheel-travel direction is retained when the
  // aircraft stops, avoiding direction flicker from tiny position jitter.
  const ogr::grass_math::Vector2 wheel_world{state.dir_x, state.dir_z};
  const auto wheel_local = ogr::grass_math::world_to_object(wheel_world, position->heading);
  const float amount = std::clamp(state.amount, 0.0f, 1.0f);
  const float wheel_mix = ogr::wheel::smoothstep01(amount);
  const float target_x = wheel_local.x * kWheelFlatDataref * amount;
  const float target_z = wheel_local.z * kWheelFlatDataref * amount;

  for (int group = 0; group < 4; ++group) {
    const int x_index = group * 2;
    const int z_index = x_index + 1;
    modified[x_index] = modified[x_index] * (1.0f - wheel_mix) + target_x * wheel_mix;
    modified[z_index] = modified[z_index] * (1.0f - wheel_mix) + target_z * wheel_mix;
    modified[x_index] = std::clamp(modified[x_index], -kWheelFlatDataref, kWheelFlatDataref);
    modified[z_index] = std::clamp(modified[z_index], -kWheelFlatDataref, kWheelFlatDataref);
  }

  XPLMDrawInfo_t crushed = *position;
  crushed.y -= amount * kCrushSinkM;
  real_instance_set_position(instance, &crushed, modified);
}

void ogr_destroy_instance(XPLMInstanceRef instance) {
  if (instance) g_wheels.instance_states.erase(instance);
  real_destroy_instance(instance);
}

} // namespace

namespace ogr::wheel {

void initialize_runtime() {
  ensure_wheel_refs();
}

void reset_runtime() {
  g_wheels = WheelRuntimeState{};
}

} // namespace ogr::wheel

// OGR's existing runtime is deliberately included as one translation unit so
// replay can wrap the proven live runtime. v0.6.5 treats wheel contact as real
// tire pressure: rolling or parked gear keeps grass flat until contact ends.
#define XPLMInstanceSetPosition ogr_instance_set_position
#define XPLMDestroyInstance ogr_destroy_instance
#include "runtime_replay.cpp"
#undef XPLMDestroyInstance
#undef XPLMInstanceSetPosition
