#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <limits>

namespace ogr {

struct ReplayEngineState {
  float rel_x{};
  float rel_y{};
  float rel_z{};
  float heading_delta{};
  float power{};
  float rpm_ratio{};
};

struct ReplayFrame {
  float running_time{};
  float flight_time{};
  float aircraft_x{};
  float aircraft_y{};
  float aircraft_z{};
  float aircraft_heading{};
  float wind_x_mps{};
  float wind_z_mps{};
  int engine_count{};
  std::array<ReplayEngineState, 16> engines{};
};

class ReplayHistory {
public:
  ReplayHistory(float max_age_seconds = 1200.0f, std::size_t max_samples = 24000)
      : max_age_seconds_(std::max(30.0f, max_age_seconds)),
        max_samples_(std::max<std::size_t>(100, max_samples)) {}

  void clear() { frames_.clear(); }
  std::size_t size() const { return frames_.size(); }
  bool empty() const { return frames_.empty(); }

  float seconds_available() const {
    if (frames_.size() < 2) return 0.0f;
    return std::max(0.0f, frames_.back().running_time - frames_.front().running_time);
  }

  void push(const ReplayFrame& frame) {
    if (!frames_.empty()) {
      const auto& last = frames_.back();
      // A new flight resets total_flight_time_sec. Some X-Plane replay paths can
      // also move time backwards; we only record while not in replay, so a
      // backwards jump here means the old history no longer belongs to the
      // current live timeline.
      if (frame.flight_time + 1.0f < last.flight_time ||
          frame.running_time + 1.0f < last.running_time) {
        frames_.clear();
      }
    }

    frames_.push_back(frame);
    while (frames_.size() > max_samples_) frames_.pop_front();

    while (frames_.size() > 2) {
      const float age = frames_.back().running_time - frames_.front().running_time;
      if (age < 0.0f || age <= max_age_seconds_) break;
      frames_.pop_front();
    }
  }

  bool sample(float running_time, float flight_time,
              float aircraft_x, float aircraft_y, float aircraft_z,
              float aircraft_heading, ReplayFrame& out) const {
    if (frames_.empty()) return false;

    enum class Clock { Running, Flight };
    Clock clock = Clock::Running;
    auto in_range = [](float value, float a, float b) {
      const float lo = std::min(a, b) - 1.5f;
      const float hi = std::max(a, b) + 1.5f;
      return value >= lo && value <= hi;
    };

    if (!in_range(running_time, frames_.front().running_time, frames_.back().running_time)) {
      if (in_range(flight_time, frames_.front().flight_time, frames_.back().flight_time))
        clock = Clock::Flight;
      else
        return false;
    }

    const auto get_time = [clock](const ReplayFrame& f) {
      return clock == Clock::Running ? f.running_time : f.flight_time;
    };
    const float wanted_time = clock == Clock::Running ? running_time : flight_time;

    auto lower = std::lower_bound(
        frames_.begin(), frames_.end(), wanted_time,
        [&](const ReplayFrame& f, float value) { return get_time(f) < value; });

    std::size_t center = lower == frames_.end()
                             ? frames_.size() - 1
                             : static_cast<std::size_t>(std::distance(frames_.begin(), lower));
    constexpr std::size_t window = 64; // ~3.2 s either side at the 20 Hz recorder rate.
    const std::size_t begin = center > window ? center - window : 0;
    const std::size_t end = std::min(frames_.size(), center + window + 1);

    float best_score = std::numeric_limits<float>::max();
    std::size_t best = center;
    for (std::size_t i = begin; i < end; ++i) {
      const auto& f = frames_[i];
      const float dt = std::abs(get_time(f) - wanted_time);
      const float dx = f.aircraft_x - aircraft_x;
      const float dy = f.aircraft_y - aircraft_y;
      const float dz = f.aircraft_z - aircraft_z;
      float dh = std::fmod(std::abs(f.aircraft_heading - aircraft_heading), 360.0f);
      if (dh > 180.0f) dh = 360.0f - dh;

      // Time is primary. Position/heading break ties when XP11 exposes a
      // quantized replay clock or the aircraft is being scrubbed on the replay
      // timeline.
      const float score = dt * 30.0f + std::hypot(dx, dz) * 0.03f +
                          std::abs(dy) * 0.005f + dh * 0.01f;
      if (score < best_score) {
        best_score = score;
        best = i;
      }
    }

    out = frames_[best];
    return true;
  }

private:
  float max_age_seconds_{};
  std::size_t max_samples_{};
  std::deque<ReplayFrame> frames_;
};

} // namespace ogr
