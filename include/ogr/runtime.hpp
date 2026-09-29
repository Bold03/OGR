#pragma once

#include "ogr/grass_math.hpp"
#include "ogr/grass_motion.hpp"
#include "ogr/replay.hpp"
#include <XPLMCamera.h>
#include <XPLMDataAccess.h>
#include <XPLMInstance.h>
#include <XPLMScenery.h>
#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace ogr {

struct GeoPoint {
  double latitude{};
  double longitude{};
};

struct Area {
  std::string id;
  std::string name;
  float density{1.0f};
  std::vector<GeoPoint> outer;
  std::vector<std::vector<GeoPoint>> holes;
};

struct Settings {
  float tile_size_m{2.4f};
  float draw_distance_m{2000.0f};
  float animated_distance_m{600.0f};
  float boundary_margin_m{1.4f};
  float ground_offset_m{0.0f};
  float wind_strength{1.0f};
  float wind_full_bend_kt{45.0f};
  bool weather_wind{true};
  bool engine_wash{true};
  float engine_wash_strength{1.3f};
  float engine_wash_range_m{35.0f};
  float engine_wash_half_angle_deg{18.0f};
  float engine_wash_base_half_width_m{2.5f};
  bool traffic_wash{true};
  float traffic_wash_strength{0.90f};
  int max_traffic_targets{24};
  float traffic_update_interval_s{0.10f};
  int max_active_tiles{1600};
  int max_total_tiles{10000};
  float refresh_interval_s{0.35f};
  float animation_interval_s{0.05f};
  float hide_aircraft_agl_ft{3000.0f};
  std::string scatter_mode{"uniform"};
  float cluster_spacing_m{12.0f};
  float cluster_radius_m{5.0f};
  float cluster_probability{0.8f};
};

class Runtime {
public:
  Runtime();
  ~Runtime();
  Runtime(const Runtime&) = delete;
  Runtime& operator=(const Runtime&) = delete;

  bool load_all(const std::string& xplane_root);
  bool reload();
  void unload();
  void update(float elapsed_seconds, float aircraft_heading_deg, float aircraft_agl_m);

  size_t pack_count() const { return packs_.size(); }
  size_t tile_count() const;
  size_t active_tile_count() const;
  const std::string& status() const { return status_; }

private:
  struct LocalArea {
    std::vector<grass_math::Point2> outer;
    std::vector<std::vector<grass_math::Point2>> holes;
  };

  struct Tile {
    float x{}, y{}, z{};
    float heading{};
    float phase{};
    grass_math::MotionPhase motion{};
    size_t model_variant{};
    uint64_t stream_key{};
    uint32_t active_epoch{};
    uint8_t lod_tier{}; // 0 = near, 1 = mid, 2 = far
    bool positioned_once{};
    bool animated_last{};
    XPLMInstanceRef instance{};
  };

  struct Pack {
    std::filesystem::path scenery_root;
    std::filesystem::path asset_root;
    std::filesystem::path config_path;
    std::string name;
    Settings settings;
    std::vector<std::string> model_paths;
    std::vector<XPLMObjectRef> objects;
    std::vector<Area> areas;
    std::vector<LocalArea> local_areas;
    std::vector<Tile> tiles;

    // Candidate grass can be numerous, but only a small camera-local subset is
    // active. Spatial bins avoid scanning every candidate on each refresh and
    // active_indices avoids scanning inactive tiles on each animation tick.
    std::unordered_map<uint64_t, std::vector<size_t>> spatial_bins;
    std::vector<size_t> active_indices;
    float spatial_cell_m{128.0f};
    uint32_t active_epoch{1};
    bool spatial_log_done{};

    // v0.2.2+: the candidate window follows the camera. v0.5 keeps the same
    // camera-local stream, but distributes it across near/mid/far LOD tiers so
    // the 10k candidate budget is not exhausted only around the camera.
    float stream_center_x{};
    float stream_center_z{};
    float stream_recenter_m{60.0f};
    uint32_t stream_rebuild_count{};
    bool stream_initialized{};

    float refresh_clock{};
    float animation_clock{};
    bool altitude_suspended{};
  };

  struct EngineWake {
    float x{}, y{}, z{}, heading{}, power{}, rpm_ratio{};
  };

  struct TrafficWake {
    int mode_s_id{};
    float x{}, y{}, z{};
    float heading{};
    float power{};
    float rpm_ratio{};
    float range_m{};
    float half_angle_deg{};
    float base_half_width_m{};
    bool on_ground{};
  };

  void update_live(float elapsed_seconds, float aircraft_heading_deg, float aircraft_agl_m);
  void update_live_fast(float elapsed_seconds, float aircraft_heading_deg, float aircraft_agl_m);
  void ensure_spatial_index(Pack& pack);
  void rebuild_stream_window(Pack& pack, float camera_x, float camera_z);
  void refresh_active_set_fast(Pack& pack, float camera_x, float camera_z);

  void load_datarefs();
  std::vector<std::filesystem::path> active_scenery_roots() const;
  bool load_pack(const std::filesystem::path& scenery_root,
                 const std::filesystem::path& config_path, Pack& out);
  bool load_pack_objects(Pack& pack);
  void rebuild_pack_tiles(Pack& pack);
  void destroy_pack_instances(Pack& pack);
  void unload_pack(Pack& pack);
  void refresh_active_set(Pack& pack, float camera_x, float camera_z);
  void update_tile(Pack& pack, Tile& tile, float camera_x, float camera_z,
                   float aircraft_x, float aircraft_z, float dt);
  float terrain_y(float x, float z, float fallback, float ground_offset_m) const;
  void update_engines(float dt, float aircraft_x, float aircraft_z, float heading_deg);
  grass_math::Vector2 ambient_wind_world(float wind_strength, float full_bend_kt,
                                         bool enabled, float* speed_mps) const;
  void log(const std::string& text) const;

  void ensure_traffic_datarefs();
  void update_traffic_wakes(float dt, float aircraft_x, float aircraft_z);
  grass_math::Vector2 traffic_wash_at(const Settings& settings, const Tile& tile,
                                      float* wash_speed) const;

  void ensure_replay_datarefs();
  void record_replay_frame(float aircraft_x, float aircraft_y, float aircraft_z,
                           float aircraft_heading_deg);
  bool apply_replay_frame(float aircraft_x, float aircraft_y, float aircraft_z,
                          float aircraft_heading_deg);
  grass_math::Vector2 replay_wind_world(const Settings& settings,
                                        float* speed_mps) const;

  std::filesystem::path xplane_root_;
  std::vector<Pack> packs_;
  XPLMProbeRef probe_{};
  std::string status_{"OGR not loaded"};

  XPLMDataRef local_x_ref_{};
  XPLMDataRef local_y_ref_{};
  XPLMDataRef local_z_ref_{};
  XPLMDataRef wind_x_ref_{};
  XPLMDataRef wind_z_ref_{};
  XPLMDataRef wind_dir_ref_{};
  XPLMDataRef wind_speed_ref_{};
  XPLMDataRef engine_count_ref_{};
  XPLMDataRef engine_throttle_ref_{};
  XPLMDataRef engine_n1_ref_{};
  XPLMDataRef engine_running_ref_{};
  XPLMDataRef engine_type_ref_{};
  XPLMDataRef prop_speed_ref_{};
  XPLMDataRef engine_speed_ref_{};
  XPLMDataRef prop_redline_ref_{};
  XPLMDataRef engine_thrust_ref_{};
  XPLMDataRef engine_mode_ref_{};
  XPLMDataRef engine_x_ref_{};
  XPLMDataRef engine_y_ref_{};
  XPLMDataRef engine_z_ref_{};

  XPLMDataRef traffic_mode_s_ref_{};
  XPLMDataRef traffic_x_ref_{};
  XPLMDataRef traffic_y_ref_{};
  XPLMDataRef traffic_z_ref_{};
  XPLMDataRef traffic_vx_ref_{};
  XPLMDataRef traffic_vy_ref_{};
  XPLMDataRef traffic_vz_ref_{};
  XPLMDataRef traffic_psi_ref_{};
  XPLMDataRef traffic_wow_ref_{};
  XPLMDataRef traffic_throttle_ref_{};
  XPLMDataRef traffic_icao_type_ref_{};
  bool traffic_refs_initialized_{};
  bool traffic_refs_available_{};
  bool traffic_detected_logged_{};
  float traffic_clock_{};
  std::array<TrafficWake, 32> traffic_wakes_{};

  XPLMDataRef replay_ref_{};
  XPLMDataRef running_time_ref_{};
  XPLMDataRef flight_time_ref_{};
  bool replay_refs_initialized_{};
  bool replay_active_{};
  bool replay_sample_missing_logged_{};
  float replay_record_clock_{};
  float replay_wind_x_mps_{};
  float replay_wind_z_mps_{};
  ReplayHistory replay_history_{1200.0f, 24000};

  std::array<EngineWake, 16> engine_wakes_{};
  grass_math::Vector2 cached_wind_{};
  float cached_wind_speed_mps_{};
  float engine_clock_{};
};

} // namespace ogr
