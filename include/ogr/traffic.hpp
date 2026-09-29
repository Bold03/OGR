#pragma once

#include "ogr/grass_math.hpp"
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <string>
#include <string_view>

namespace ogr::traffic {

enum class Kind {
  light_prop,
  turboprop,
  jet,
  heavy_jet,
};

struct Profile {
  Kind kind{Kind::jet};
  float range_m{58.0f};
  float half_angle_deg{20.0f};
  float base_half_width_m{4.5f};
  float power_scale{1.0f};
  float rpm_ratio{0.85f};
};

inline std::string normalize_icao(std::string_view raw) {
  std::string out;
  out.reserve(raw.size());
  for (char ch : raw) {
    if (ch == '\0' || std::isspace(static_cast<unsigned char>(ch))) break;
    out.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(ch))));
  }
  return out;
}

inline bool one_of(std::string_view code, std::initializer_list<std::string_view> values) {
  for (const auto value : values)
    if (code == value) return true;
  return false;
}

inline Profile profile_for_icao(std::string_view raw) {
  const std::string code = normalize_icao(raw);

  if (one_of(code, {"C150","C152","C170","C172","C177","C182","C185","C206","C208",
                    "PA18","PA28","PA32","PA34","PA44","DA20","DA40","DA42","SR20","SR22",
                    "BE36","BE58","P28A","P28R","P28T","M20P","M20T","PC6T","BN2P"})) {
    return {Kind::light_prop, 30.0f, 24.0f, 2.4f, 0.72f, 0.95f};
  }

  if (one_of(code, {"AT43","AT45","AT72","AT73","AT75","AT76","DH8A","DH8B","DH8C","DH8D",
                    "SF34","F50","F27","JS32","JS41","BE20","BE30","B190","D228","D328",
                    "PC12","C27J","C130","A400"})) {
    return {Kind::turboprop, 46.0f, 23.0f, 4.0f, 0.92f, 1.0f};
  }

  if (one_of(code, {"A332","A333","A338","A339","A343","A345","A346","A359","A35K","A388",
                    "B742","B743","B744","B748","B772","B773","B77L","B77W","B788","B789","B78X",
                    "DC10","MD11","L101","A124","A225"})) {
    return {Kind::heavy_jet, 92.0f, 18.0f, 8.0f, 1.18f, 1.0f};
  }

  return {Kind::jet, 64.0f, 19.0f, 5.5f, 1.0f, 0.9f};
}

inline float estimated_power(float throttle, float horizontal_speed_mps, bool on_ground,
                             const Profile& profile) {
  const float lever = std::isfinite(throttle) ? grass_math::clamp01(throttle) : 0.0f;
  const float speed = std::max(0.0f, std::isfinite(horizontal_speed_mps) ? horizontal_speed_mps : 0.0f);

  // Online clients do not always populate a useful throttle value. Ground
  // speed is therefore a conservative fallback: taxi produces a small wake,
  // while a takeoff roll ramps strongly even when throttle is unavailable.
  const float taxi = grass_math::clamp01(speed / 18.0f) * 0.28f;
  const float takeoff = grass_math::clamp01((speed - 12.0f) / 43.0f);
  float inferred = std::max(taxi, takeoff * takeoff);

  if (!on_ground) {
    // Keep low airborne traffic capable of disturbing grass, but much weaker;
    // tile-specific height attenuation will remove it once it is clear of the ground.
    inferred *= 0.40f;
  }

  float commanded = lever * lever;
  if (profile.kind == Kind::light_prop || profile.kind == Kind::turboprop)
    commanded = std::pow(lever, 1.35f);

  return grass_math::clamp01(std::max(commanded, inferred) * profile.power_scale);
}

} // namespace ogr::traffic
