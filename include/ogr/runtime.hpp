#pragma once

#include "ogr/grass_math.hpp"
#include "ogr/grass_motion.hpp"
#include <XPLMCamera.h>
#include <XPLMDataAccess.h>
#include <XPLMInstance.h>
#include <XPLMScenery.h>
#include <array>
#include <filesystem>
#include <string>
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
  float draw_distance_m{914.4f};
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
  int max_active_tiles{1600};
  int max_total_tiles{8000};
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
    bool positioned_once{};
    bool animated_last{};
    XPLMInstanceRef instance{};
  };

  struct Pack {
    std::filesystem::path scenery_root;
    std::filesystem::path config_path;
    std::string name;
    Settings settings;
    std::vector<std::string> model_paths;
    std::vector<XPLMObjectRef> objects;
    std::vector<Area> areas;
    std::vector<LocalArea> local_areas;
    std::vector<Tile> tiles;
    float refresh_clock{};
    float animation_clock{};
    bool altitude_suspended{};
  };

  struct EngineWake {
    float x{}, y{}, z{}, heading{}, power{}, rpm_ratio{};
  };

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

  std::array<EngineWake, 16> engine_wakes_{};
  grass_math::Vector2 cached_wind_{};
  float cached_wind_speed_mps_{};
  float engine_clock_{};
};

} // namespace ogr
