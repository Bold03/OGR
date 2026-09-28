#pragma once
#include "ogr/grass_math.hpp"
#include <array>

namespace ogr::grass_math {

inline float prop_wash_power(float rpm_ratio, float throttle, float thrust_n,
                             bool thrust_available) {
  rpm_ratio = std::clamp(rpm_ratio, 0.0f, 1.2f);
  if (rpm_ratio < 0.01f) return 0.0f;
  if (thrust_available) {
    if (thrust_n <= 0.0f) return 0.0f;
    return clamp01(std::pow(rpm_ratio, 1.25f) *
                   std::sqrt(thrust_n / (thrust_n + 400.0f)));
  }
  return clamp01(rpm_ratio * rpm_ratio * (0.25f + 0.75f * clamp01(throttle)));
}

inline float approach(float current, float target, float dt, float tau) {
  return current + (target - current) * (1.0f - std::exp(-dt / tau));
}

inline float wake_ground_factor(float height_m, float aft_distance_m) {
  const float vertical_spread = 1.8f + 0.09f * std::max(0.0f, aft_distance_m);
  const float h = std::max(0.0f, height_m) / vertical_spread;
  return std::exp(-0.5f * h * h);
}

struct MotionPhase { float sway{}, flutter{}, gust{}; };

inline std::array<float, 3> motion_hz(float speed_mps, float sample_dt) {
  const float activity = 1.0f - std::exp(-std::max(0.0f, speed_mps) / 12.0f);
  const float sample_limit = 0.22f / std::max(0.02f, sample_dt);
  return {std::min(0.25f + 1.75f * activity, sample_limit),
          std::min(0.70f + 2.90f * activity, sample_limit),
          std::min(0.12f + 0.50f * activity, sample_limit)};
}

inline void advance_motion(MotionPhase& phase, float speed_mps, float dt) {
  const auto hz = motion_hz(speed_mps, dt);
  const float step = 2.0f * pi * std::min(0.25f, std::max(0.0f, dt));
  phase.sway = std::fmod(phase.sway + step * hz[0], 2.0f * pi);
  phase.flutter = std::fmod(phase.flutter + step * hz[1], 2.0f * pi);
  phase.gust = std::fmod(phase.gust + step * hz[2], 2.0f * pi);
}

inline Vector2 moving_bend(Vector2 flow, const MotionPhase& phase,
                           float seed, int group, float speed_mps) {
  constexpr float offsets[4] = {0.0f, 1.37f, 2.71f, 4.19f};
  const float offset = offsets[group];
  const float activity = 1.0f - std::exp(-std::max(0.0f, speed_mps) / 12.0f);
  const float pulse = std::clamp(
      0.66f + 0.23f * std::sin(phase.sway + seed + offset) +
      (0.05f + 0.20f * activity) * std::sin(phase.flutter + seed * 1.7f + offset) +
      0.14f * activity * std::sin(phase.gust + seed * 0.63f + offset * 0.7f),
      0.12f, 1.12f);
  const float side = 0.12f * activity * std::sin(phase.flutter + seed * 1.7f + offset + 0.9f);
  Vector2 bend{flow.x * pulse - flow.z * side, flow.z * pulse + flow.x * side};
  const float magnitude = std::hypot(bend.x, bend.z);
  if (magnitude > 1.0f) { bend.x /= magnitude; bend.z /= magnitude; }
  return bend;
}

} // namespace ogr::grass_math
