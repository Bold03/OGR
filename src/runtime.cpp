#include "ogr/runtime.hpp"

#include <XPLMGraphics.h>
#include <XPLMUtilities.h>
#include <nlohmann/json.hpp>
#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <fstream>
#include <functional>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_set>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace ogr {
namespace {
constexpr float pi = grass_math::pi;

std::string trim(std::string value) {
  auto not_space = [](unsigned char c) { return !std::isspace(c); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  if (value.size() >= 2 && value.front() == '"' && value.back() == '"')
    value = value.substr(1, value.size() - 2);
  return value;
}

bool safe_relative_path(const fs::path& path) {
  if (path.empty() || path.is_absolute()) return false;
  const std::string text = path.generic_string();
  if (text.find(':') != std::string::npos) return false;
  for (const auto& part : path)
    if (part == "..") return false;
  return true;
}

float distance2(float ax, float az, float bx, float bz) {
  const float dx = ax - bx;
  const float dz = az - bz;
  return dx * dx + dz * dz;
}

float clamp_density(float value) { return std::clamp(value, 0.03f, 1.0f); }

bool valid_geo(double lat, double lon) {
  return std::isfinite(lat) && std::isfinite(lon) && lat >= -90.0 && lat <= 90.0 &&
         lon >= -180.0 && lon <= 180.0;
}

std::string generic_path(fs::path path) {
  return path.lexically_normal().generic_string();
}

} // namespace

Runtime::Runtime() = default;
Runtime::~Runtime() { unload(); }

void Runtime::log(const std::string& text) const {
  const std::string line = "[OGR] " + text + "\n";
  XPLMDebugString(line.c_str());
}

void Runtime::load_datarefs() {
  local_x_ref_ = XPLMFindDataRef("sim/flightmodel/position/local_x");
  local_y_ref_ = XPLMFindDataRef("sim/flightmodel/position/local_y");
  local_z_ref_ = XPLMFindDataRef("sim/flightmodel/position/local_z");
  wind_x_ref_ = XPLMFindDataRef("sim/weather/wind_now_x_msc");
  wind_z_ref_ = XPLMFindDataRef("sim/weather/wind_now_z_msc");
  wind_dir_ref_ = XPLMFindDataRef("sim/weather/wind_direction_degt");
  wind_speed_ref_ = XPLMFindDataRef("sim/weather/wind_speed_kt");
  engine_count_ref_ = XPLMFindDataRef("sim/aircraft/engine/acf_num_engines");
  engine_throttle_ref_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_thro");
  engine_n1_ref_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_N1_");
  engine_running_ref_ = XPLMFindDataRef("sim/flightmodel/engine/ENGN_running");
  engine_type_ref_ = XPLMFindDataRef("sim/aircraft/prop/acf_en_type");
  prop_speed_ref_ = XPLMFindDataRef("sim/flightmodel2/engines/prop_rotation_speed_rad_sec");
  engine_speed_ref_ = XPLMFindDataRef("sim/flightmodel2/engines/engine_rotation_speed_rad_sec");
  prop_redline_ref_ = XPLMFindDataRef("sim/aircraft/controls/acf_RSC_redline_prp");
  engine_thrust_ref_ = XPLMFindDataRef("sim/flightmodel/engine/POINT_thrust");
  engine_mode_ref_ = XPLMFindDataRef("sim/cockpit2/engine/actuators/prop_mode");
  engine_x_ref_ = XPLMFindDataRef("sim/flightmodel2/engines/location_x_mtr");
  engine_y_ref_ = XPLMFindDataRef("sim/flightmodel2/engines/location_y_mtr");
  engine_z_ref_ = XPLMFindDataRef("sim/flightmodel2/engines/location_z_mtr");
}

std::vector<fs::path> Runtime::active_scenery_roots() const {
  std::vector<fs::path> roots;
  std::unordered_set<std::string> seen;
  const fs::path custom = xplane_root_ / "Custom Scenery";
  const fs::path ini = custom / "scenery_packs.ini";
  std::ifstream input(ini);
  if (input) {
    std::string line;
    while (std::getline(input, line)) {
      constexpr const char* prefix = "SCENERY_PACK ";
      if (line.rfind(prefix, 0) != 0) continue;
      std::string relative = trim(line.substr(std::char_traits<char>::length(prefix)));
      if (relative.empty() || relative[0] == '*') continue;
      std::replace(relative.begin(), relative.end(), '\\', '/');
      fs::path path = xplane_root_ / fs::path(relative);
      if (!fs::exists(path) || !fs::is_directory(path)) continue;
      const std::string key = generic_path(path);
      if (seen.insert(key).second) roots.push_back(std::move(path));
    }
    return roots;
  }

  std::error_code ec;
  if (!fs::exists(custom, ec)) return roots;
  for (const auto& entry : fs::directory_iterator(custom, ec)) {
    if (ec) break;
    if (!entry.is_directory(ec)) continue;
    const std::string key = generic_path(entry.path());
    if (seen.insert(key).second) roots.push_back(entry.path());
  }
  return roots;
}

bool Runtime::load_pack(const fs::path& scenery_root, const fs::path& config_path, Pack& out) {
  try {
    std::ifstream input(config_path);
    if (!input) return false;
    const json root = json::parse(input);
    if (root.value("format", std::string()) != "OGR_AREA_V1") {
      log("Ignoring unsupported area file: " + config_path.string());
      return false;
    }

    out.scenery_root = scenery_root;
    out.config_path = config_path;
    out.name = root.value("name", scenery_root.filename().string());

    const json s = root.contains("settings") && root.at("settings").is_object()
                       ? root.at("settings") : json::object();
    out.settings.tile_size_m = std::clamp(s.value("tile_size_m", 2.4f), 0.35f, 20.0f);
    out.settings.draw_distance_m = std::clamp(s.value("draw_distance_m", 914.4f), 30.0f, 5000.0f);
    out.settings.animated_distance_m = std::clamp(
        s.value("animated_distance_m", 600.0f), 15.0f, out.settings.draw_distance_m);
    out.settings.boundary_margin_m = std::clamp(s.value("boundary_margin_m", 1.4f), 0.0f, 10.0f);
    out.settings.ground_offset_m = std::clamp(s.value("ground_offset_m", 0.0f), -0.5f, 1.0f);
    out.settings.wind_strength = std::clamp(s.value("wind_strength", 1.0f), 0.0f, 3.0f);
    out.settings.wind_full_bend_kt = std::clamp(s.value("wind_full_bend_kt", 45.0f), 20.0f, 150.0f);
    out.settings.weather_wind = s.value("weather_wind", true);
    out.settings.engine_wash = s.value("engine_wash", true);
    out.settings.engine_wash_strength = std::clamp(s.value("engine_wash_strength", 1.3f), 0.0f, 3.0f);
    out.settings.engine_wash_range_m = std::clamp(s.value("engine_wash_range_m", 35.0f), 5.0f, 150.0f);
    out.settings.engine_wash_half_angle_deg = std::clamp(
        s.value("engine_wash_half_angle_deg", 18.0f), 3.0f, 60.0f);
    out.settings.engine_wash_base_half_width_m = std::clamp(
        s.value("engine_wash_base_half_width_m", 2.5f), 0.5f, 12.0f);
    out.settings.max_active_tiles = std::clamp(s.value("max_active_tiles", 1600), 50, 20000);
    out.settings.max_total_tiles = std::clamp(s.value("max_total_tiles", 8000), 100, 500000);
    out.settings.refresh_interval_s = std::clamp(s.value("refresh_interval_s", 0.35f), 0.1f, 2.0f);
    out.settings.animation_interval_s = std::clamp(s.value("animation_interval_s", 0.05f), 0.02f, 0.5f);
    out.settings.hide_aircraft_agl_ft = std::clamp(s.value("hide_aircraft_agl_ft", 3000.0f), 0.0f, 30000.0f);
    out.settings.scatter_mode = s.value("scatter_mode", std::string("uniform"));
    std::transform(out.settings.scatter_mode.begin(), out.settings.scatter_mode.end(),
                   out.settings.scatter_mode.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (out.settings.scatter_mode != "uniform" && out.settings.scatter_mode != "clustered")
      out.settings.scatter_mode = "uniform";
    out.settings.cluster_spacing_m = std::clamp(s.value("cluster_spacing_m", 12.0f), 2.0f, 80.0f);
    out.settings.cluster_radius_m = std::clamp(s.value("cluster_radius_m", 5.0f), 0.5f, 40.0f);
    out.settings.cluster_probability = std::clamp(s.value("cluster_probability", 0.8f), 0.05f, 1.0f);

    if (root.contains("models") && root.at("models").is_array()) {
      for (const auto& item : root.at("models")) {
        if (!item.is_string()) continue;
        const std::string path = item.get<std::string>();
        if (path.empty()) continue;
        if (std::find(out.model_paths.begin(), out.model_paths.end(), path) == out.model_paths.end())
          out.model_paths.push_back(path);
      }
    }
    if (out.model_paths.empty()) {
      log("Area file has no OGR models: " + config_path.string());
      return false;
    }

    if (root.contains("areas") && root.at("areas").is_array()) {
      for (const auto& item : root.at("areas")) {
        if (!item.is_object()) continue;
        Area area;
        area.id = item.value("id", std::string("area"));
        area.name = item.value("name", area.id);
        area.density = clamp_density(item.value("density", 1.0f));
        if (item.contains("outer") && item.at("outer").is_array()) {
          for (const auto& point : item.at("outer")) {
            if (!point.is_object()) continue;
            const double lat = point.value("latitude", 999.0);
            const double lon = point.value("longitude", 999.0);
            if (valid_geo(lat, lon)) area.outer.push_back({lat, lon});
          }
        }
        if (item.contains("holes") && item.at("holes").is_array()) {
          for (const auto& ring : item.at("holes")) {
            if (!ring.is_array()) continue;
            std::vector<GeoPoint> hole;
            for (const auto& point : ring) {
              if (!point.is_object()) continue;
              const double lat = point.value("latitude", 999.0);
              const double lon = point.value("longitude", 999.0);
              if (valid_geo(lat, lon)) hole.push_back({lat, lon});
            }
            if (hole.size() >= 3) area.holes.push_back(std::move(hole));
          }
        }
        if (area.outer.size() >= 3) out.areas.push_back(std::move(area));
      }
    }
    if (out.areas.empty()) {
      log("No OGR forest areas in: " + config_path.string());
      return false;
    }
    return true;
  } catch (const std::exception& e) {
    log(std::string("Cannot parse ") + config_path.string() + ": " + e.what());
    return false;
  }
}

bool Runtime::load_pack_objects(Pack& pack) {
  for (const auto& relative_text : pack.model_paths) {
    const fs::path relative(relative_text);
    if (!safe_relative_path(relative)) {
      log("Unsafe OGR model path ignored: " + relative_text);
      return false;
    }
    const fs::path object_path = pack.scenery_root / relative;
    if (!fs::exists(object_path)) {
      log("Missing OGR model: " + object_path.string());
      return false;
    }
    XPLMObjectRef object = XPLMLoadObject(object_path.string().c_str());
    if (!object) {
      log("X-Plane could not load OGR model: " + object_path.string());
      return false;
    }
    pack.objects.push_back(object);
  }
  return !pack.objects.empty();
}

void Runtime::rebuild_pack_tiles(Pack& pack) {
  destroy_pack_instances(pack);
  pack.tiles.clear();
  pack.local_areas.clear();
  pack.local_areas.reserve(pack.areas.size());

  for (const auto& area : pack.areas) {
    LocalArea local;
    for (const auto& point : area.outer) {
      double x{}, y{}, z{};
      XPLMWorldToLocal(point.latitude, point.longitude, 0.0, &x, &y, &z);
      local.outer.push_back({static_cast<float>(x), static_cast<float>(z)});
    }
    for (const auto& geo_hole : area.holes) {
      std::vector<grass_math::Point2> hole;
      for (const auto& point : geo_hole) {
        double x{}, y{}, z{};
        XPLMWorldToLocal(point.latitude, point.longitude, 0.0, &x, &y, &z);
        hole.push_back({static_cast<float>(x), static_cast<float>(z)});
      }
      if (hole.size() >= 3) local.holes.push_back(std::move(hole));
    }
    pack.local_areas.push_back(std::move(local));
  }

  for (size_t area_index = 0; area_index < pack.local_areas.size() &&
                              static_cast<int>(pack.tiles.size()) < pack.settings.max_total_tiles;
       ++area_index) {
    const auto& area = pack.local_areas[area_index];
    if (area.outer.size() < 3) continue;
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();
    for (const auto& p : area.outer) {
      min_x = std::min(min_x, p.x); max_x = std::max(max_x, p.x);
      min_z = std::min(min_z, p.z); max_z = std::max(max_z, p.z);
    }

    const float density = clamp_density(pack.areas[area_index].density);
    const float spacing = pack.settings.tile_size_m / std::sqrt(std::max(0.03f, density));
    const int ix0 = static_cast<int>(std::floor(min_x / spacing));
    const int ix1 = static_cast<int>(std::ceil(max_x / spacing));
    const int iz0 = static_cast<int>(std::floor(min_z / spacing));
    const int iz1 = static_cast<int>(std::ceil(max_z / spacing));
    const uint32_t pack_seed = grass_math::mix_hash(static_cast<uint32_t>(std::hash<std::string>{}(pack.name)));

    for (int iz = iz0; iz <= iz1 && static_cast<int>(pack.tiles.size()) < pack.settings.max_total_tiles; ++iz) {
      for (int ix = ix0; ix <= ix1 && static_cast<int>(pack.tiles.size()) < pack.settings.max_total_tiles; ++ix) {
        uint32_t seed = pack_seed ^ static_cast<uint32_t>(area_index + 1) * 2654435761u ^
                        static_cast<uint32_t>(ix) * 73856093u ^
                        static_cast<uint32_t>(iz) * 19349663u;
        const float jitter_x = (grass_math::hash01(seed) - 0.5f) * spacing * 0.58f;
        const float jitter_z = (grass_math::hash01(seed ^ 0x9e3779b9u) - 0.5f) * spacing * 0.58f;
        const grass_math::Point2 p{ix * spacing + jitter_x, iz * spacing + jitter_z};
        if (!grass_math::point_in_area_margin(p, area.outer, area.holes,
                                              pack.settings.boundary_margin_m))
          continue;
        if (pack.settings.scatter_mode == "clustered" &&
            !grass_math::clustered_keep(p, pack.settings.cluster_spacing_m,
                                        pack.settings.cluster_radius_m,
                                        pack.settings.cluster_probability,
                                        pack_seed ^ static_cast<uint32_t>(area_index + 1) * 2246822519u))
          continue;

        Tile tile;
        tile.x = p.x;
        tile.z = p.z;
        tile.heading = grass_math::hash01(seed ^ 0xa5a5a5a5u) * 360.0f;
        tile.phase = grass_math::hash01(seed ^ 0x36ef3720u) * pi * 2.0f;
        tile.model_variant = static_cast<size_t>(grass_math::mix_hash(seed ^ 0x6c8e9cf5u)) %
                             std::max<size_t>(1, pack.objects.size());
        pack.tiles.push_back(tile);
      }
    }
  }
}

void Runtime::destroy_pack_instances(Pack& pack) {
  for (auto& tile : pack.tiles) {
    if (tile.instance) XPLMDestroyInstance(tile.instance);
    tile.instance = nullptr;
    tile.positioned_once = false;
    tile.animated_last = false;
  }
}

void Runtime::unload_pack(Pack& pack) {
  destroy_pack_instances(pack);
  for (auto object : pack.objects)
    if (object) XPLMUnloadObject(object);
  pack.objects.clear();
  pack.tiles.clear();
  pack.local_areas.clear();
  pack.areas.clear();
}

bool Runtime::load_all(const std::string& xplane_root) {
  unload();
  xplane_root_ = fs::path(xplane_root);
  load_datarefs();
  probe_ = XPLMCreateProbe(xplm_ProbeY);
  if (!probe_) {
    status_ = "Cannot create terrain probe";
    log(status_);
    return false;
  }

  const auto roots = active_scenery_roots();
  size_t found_configs = 0;
  for (const auto& scenery_root : roots) {
    const fs::path config = scenery_root / "OafishGrass" / "ogr_areas.json";
    if (!fs::exists(config)) continue;
    ++found_configs;
    Pack pack;
    if (!load_pack(scenery_root, config, pack)) continue;
    if (!load_pack_objects(pack)) {
      unload_pack(pack);
      continue;
    }
    rebuild_pack_tiles(pack);
    log("Loaded " + pack.name + ": " + std::to_string(pack.areas.size()) +
        " WED area(s), " + std::to_string(pack.tiles.size()) + " grass candidate(s)");
    packs_.push_back(std::move(pack));
  }

  if (packs_.empty()) {
    status_ = found_configs ? "OGR area files found, but none could be loaded"
                            : "No OafishGrass/ogr_areas.json in active scenery packs";
    log(status_);
    return false;
  }
  status_ = "OGR ready: " + std::to_string(packs_.size()) + " scenery pack(s), " +
            std::to_string(tile_count()) + " candidate(s)";
  log(status_);
  return true;
}

bool Runtime::reload() {
  if (xplane_root_.empty()) return false;
  const std::string root = xplane_root_.string();
  return load_all(root);
}

void Runtime::unload() {
  for (auto& pack : packs_) unload_pack(pack);
  packs_.clear();
  if (probe_) XPLMDestroyProbe(probe_);
  probe_ = nullptr;
  engine_wakes_ = {};
  cached_wind_ = {};
  cached_wind_speed_mps_ = 0.0f;
  engine_clock_ = 0.0f;
  status_ = "OGR not loaded";
}

size_t Runtime::tile_count() const {
  size_t count = 0;
  for (const auto& pack : packs_) count += pack.tiles.size();
  return count;
}

size_t Runtime::active_tile_count() const {
  size_t count = 0;
  for (const auto& pack : packs_)
    for (const auto& tile : pack.tiles)
      if (tile.instance) ++count;
  return count;
}

float Runtime::terrain_y(float x, float z, float fallback, float ground_offset_m) const {
  if (!probe_) return fallback;
  XPLMProbeInfo_t info{};
  info.structSize = sizeof(info);
  const auto result = XPLMProbeTerrainXYZ(probe_, x, fallback + 1200.0f, z, &info);
  return result == xplm_ProbeHitTerrain ? info.locationY + ground_offset_m : fallback;
}

void Runtime::update_engines(float dt, float aircraft_x, float aircraft_z, float heading_deg) {
  const int count = engine_count_ref_ ? std::clamp(XPLMGetDatai(engine_count_ref_), 0, 16) : 0;
  using Floats = std::array<float, 16>;
  using Ints = std::array<int, 16>;
  Floats throttle{}, n1{}, prop{}, crank{}, thrust{}, x{}, y{}, z{};
  Ints running{}, type{}, mode{};
  auto read_f = [count](XPLMDataRef ref, Floats& values) {
    return ref && count ? XPLMGetDatavf(ref, values.data(), 0, count) : 0;
  };
  auto read_i = [count](XPLMDataRef ref, Ints& values) {
    return ref && count ? XPLMGetDatavi(ref, values.data(), 0, count) : 0;
  };
  read_f(engine_throttle_ref_, throttle);
  read_f(engine_n1_ref_, n1);
  const int prop_count = read_f(prop_speed_ref_, prop);
  const int crank_count = read_f(engine_speed_ref_, crank);
  const int thrust_count = read_f(engine_thrust_ref_, thrust);
  const int type_count = read_i(engine_type_ref_, type);
  const int running_count = read_i(engine_running_ref_, running);
  const int mode_count = read_i(engine_mode_ref_, mode);
  read_f(engine_x_ref_, x); read_f(engine_y_ref_, y); read_f(engine_z_ref_, z);
  auto finite = [](float value) { return std::isfinite(value) ? value : 0.0f; };
  const float redline = prop_redline_ref_ ? finite(XPLMGetDataf(prop_redline_ref_)) : 0.0f;
  const float reference_speed = redline > 1.0f ? redline : 2700.0f * pi / 30.0f;
  const float h = heading_deg * pi / 180.0f;
  const float c = std::cos(h), s = std::sin(h);
  const float aircraft_y = local_y_ref_ ? XPLMGetDataf(local_y_ref_) : 0.0f;

  for (int i = 0; i < 16; ++i) {
    auto& wake = engine_wakes_[i];
    float target = 0.0f;
    float ratio = 0.0f;
    bool reverse = false;
    if (i < count) {
      const bool piston = type_count > i && (type[i] == 0 || type[i] == 1);
      const bool is_prop = type_count > i ?
          (piston || type[i] == 2 || type[i] == 3 || type[i] == 8 ||
           type[i] == 9 || type[i] == 10)
          : (prop_count > i && std::abs(prop[i]) > 1.0f);
      const bool rpm_available = prop_count > i || (piston && crank_count > i);
      const float speed = std::abs(finite(prop_count > i ? prop[i] :
                                         (piston && crank_count > i ? crank[i] : 0.0f)));
      ratio = std::clamp(speed / reference_speed, 0.0f, 1.2f);
      const bool on = running_count <= i || running[i] != 0;
      reverse = mode_count > i && mode[i] == 3;
      const bool feathered = is_prop && mode_count > i && mode[i] == 0;
      const float signed_thrust = finite(thrust[i]);
      const float driving_thrust = std::max(0.0f, reverse ? -signed_thrust : signed_thrust);
      const bool has_thrust = thrust_count > i && std::isfinite(thrust[i]);
      const float lever = grass_math::clamp01(finite(throttle[i]));
      if (on && !feathered) {
        if (is_prop && rpm_available) {
          target = grass_math::prop_wash_power(ratio, lever, driving_thrust, has_thrust);
        } else {
          const float spool = std::max(lever, grass_math::clamp01(finite(n1[i]) / 100.0f));
          ratio = spool;
          target = spool * spool;
          if (has_thrust && driving_thrust <= 0.0f) target = 0.0f;
        }
      }
      wake.x = aircraft_x + finite(x[i]) * c - finite(z[i]) * s;
      wake.z = aircraft_z + finite(x[i]) * s + finite(z[i]) * c;
      wake.y = aircraft_y + finite(y[i]);
      wake.heading = heading_deg + (reverse ? 180.0f : 0.0f);
    }
    wake.power = grass_math::approach(wake.power, target, dt, target > wake.power ? 0.20f : 0.45f);
    wake.rpm_ratio = grass_math::approach(wake.rpm_ratio, ratio, dt, 0.20f);
    if (i >= count) wake = {};
  }
}

grass_math::Vector2 Runtime::ambient_wind_world(float wind_strength, float full_bend_kt,
                                                  bool enabled, float* speed_mps) const {
  if (speed_mps) *speed_mps = 0.0f;
  if (!enabled) return {};
  float wx = wind_x_ref_ ? XPLMGetDataf(wind_x_ref_) : 0.0f;
  float wz = wind_z_ref_ ? XPLMGetDataf(wind_z_ref_) : 0.0f;
  if (!std::isfinite(wx) || !std::isfinite(wz)) { wx = 0.0f; wz = 0.0f; }
  float speed = std::hypot(wx, wz);
  if (speed < 0.01f && wind_dir_ref_ && wind_speed_ref_) {
    const float dir = XPLMGetDataf(wind_dir_ref_) * pi / 180.0f;
    speed = std::max(0.0f, XPLMGetDataf(wind_speed_ref_)) * 0.514444f;
    wx = -std::sin(dir) * speed;
    wz = std::cos(dir) * speed;
  }
  if (speed_mps) *speed_mps = speed;
  if (speed < 0.01f) return {};
  const float full_bend_mps = std::max(1.0f, full_bend_kt * 0.514444f);
  const float normalized = grass_math::clamp01(speed / full_bend_mps);
  const float response = std::pow(normalized, 0.62f);
  const float amount = response * wind_strength;
  return {wx / speed * amount, wz / speed * amount};
}

void Runtime::refresh_active_set(Pack& pack, float camera_x, float camera_z) {
  if (pack.objects.empty()) return;
  struct Candidate { size_t index; float d2; };
  std::vector<Candidate> candidates;
  const float draw2 = pack.settings.draw_distance_m * pack.settings.draw_distance_m;
  for (size_t i = 0; i < pack.tiles.size(); ++i) {
    const float d2 = distance2(pack.tiles[i].x, pack.tiles[i].z, camera_x, camera_z);
    if (d2 <= draw2) candidates.push_back({i, d2});
  }
  if (static_cast<int>(candidates.size()) > pack.settings.max_active_tiles) {
    std::nth_element(candidates.begin(),
                     candidates.begin() + pack.settings.max_active_tiles,
                     candidates.end(),
                     [](const Candidate& a, const Candidate& b) { return a.d2 < b.d2; });
    candidates.resize(static_cast<size_t>(pack.settings.max_active_tiles));
  }
  std::vector<unsigned char> wanted(pack.tiles.size(), 0);
  for (const auto& candidate : candidates) wanted[candidate.index] = 1;

  static const char* datarefs[] = {
      "oafish/ogr/grass/bend_x_0", "oafish/ogr/grass/bend_z_0",
      "oafish/ogr/grass/bend_x_1", "oafish/ogr/grass/bend_z_1",
      "oafish/ogr/grass/bend_x_2", "oafish/ogr/grass/bend_z_2",
      "oafish/ogr/grass/bend_x_3", "oafish/ogr/grass/bend_z_3", nullptr};

  for (size_t i = 0; i < pack.tiles.size(); ++i) {
    auto& tile = pack.tiles[i];
    if (!wanted[i]) {
      if (tile.instance) XPLMDestroyInstance(tile.instance);
      tile.instance = nullptr;
      tile.positioned_once = false;
      tile.animated_last = false;
      continue;
    }
    if (tile.instance) continue;
    XPLMObjectRef object = pack.objects[tile.model_variant % pack.objects.size()];
    tile.instance = XPLMCreateInstance(object, datarefs);
    if (!tile.instance) continue;
    const float fallback_y = local_y_ref_ ? XPLMGetDataf(local_y_ref_) : 0.0f;
    tile.y = terrain_y(tile.x, tile.z, fallback_y, pack.settings.ground_offset_m);
    XPLMDrawInfo_t draw{};
    draw.structSize = sizeof(draw);
    draw.x = tile.x; draw.y = tile.y; draw.z = tile.z;
    draw.pitch = 0.0f; draw.heading = tile.heading; draw.roll = 0.0f;
    float neutral[8]{};
    XPLMInstanceSetPosition(tile.instance, &draw, neutral);
    tile.positioned_once = true;
    tile.animated_last = false;
  }
}

void Runtime::update_tile(Pack& pack, Tile& tile, float camera_x, float camera_z,
                          float, float, float dt) {
  if (!tile.instance) return;
  const bool animate = distance2(tile.x, tile.z, camera_x, camera_z) <=
                       pack.settings.animated_distance_m * pack.settings.animated_distance_m;
  if (!animate && tile.positioned_once && !tile.animated_last) return;

  grass_math::Vector2 world = animate ? cached_wind_ : grass_math::Vector2{};
  float wash_speed = 0.0f;
  if (animate && pack.settings.engine_wash) {
    for (const auto& engine : engine_wakes_) {
      if (engine.power < 0.001f) continue;
      const float h = engine.heading * pi / 180.0f;
      const float aft_distance = -(tile.x - engine.x) * std::sin(h) +
                                  (tile.z - engine.z) * std::cos(h);
      const auto wash = grass_math::engine_wash(
          {tile.x, tile.z}, {engine.x, engine.z}, engine.heading, engine.power,
          pack.settings.engine_wash_range_m, pack.settings.engine_wash_half_angle_deg,
          pack.settings.engine_wash_base_half_width_m);
      const float attenuation = pack.settings.engine_wash_strength *
          grass_math::wake_ground_factor(engine.y - tile.y, aft_distance);
      world.x += wash.x * attenuation;
      world.z += wash.z * attenuation;
      wash_speed += std::hypot(wash.x, wash.z) * attenuation *
                    (10.0f + 28.0f * engine.rpm_ratio);
    }
  }
  const float magnitude = std::hypot(world.x, world.z);
  if (magnitude > 1.25f) {
    world.x *= 1.25f / magnitude;
    world.z *= 1.25f / magnitude;
  }
  const auto local = grass_math::world_to_object(world, tile.heading);
  float data[8]{};
  if (animate) {
    const float local_speed = std::hypot(cached_wind_speed_mps_, wash_speed);
    grass_math::advance_motion(tile.motion, local_speed, dt);
    for (int group = 0; group < 4; ++group) {
      const auto bend = grass_math::moving_bend(local, tile.motion, tile.phase, group, local_speed);
      data[group * 2] = bend.x;
      data[group * 2 + 1] = bend.z;
    }
  }
  XPLMDrawInfo_t draw{};
  draw.structSize = sizeof(draw);
  draw.x = tile.x; draw.y = tile.y; draw.z = tile.z;
  draw.pitch = 0.0f; draw.heading = tile.heading; draw.roll = 0.0f;
  XPLMInstanceSetPosition(tile.instance, &draw, data);
  tile.positioned_once = true;
  tile.animated_last = animate;
}

void Runtime::update(float elapsed_seconds, float aircraft_heading_deg, float aircraft_agl_m) {
  if (packs_.empty()) return;
  const float elapsed = std::clamp(elapsed_seconds, 0.0f, 0.5f);
  const float aircraft_x = local_x_ref_ ? XPLMGetDataf(local_x_ref_) : 0.0f;
  const float aircraft_z = local_z_ref_ ? XPLMGetDataf(local_z_ref_) : 0.0f;
  float camera_x = aircraft_x;
  float camera_z = aircraft_z;
  XPLMCameraPosition_t camera{};
  XPLMReadCameraPosition(&camera);
  if (std::isfinite(camera.x) && std::isfinite(camera.z)) {
    camera_x = camera.x;
    camera_z = camera.z;
  }

  engine_clock_ += elapsed;
  if (engine_clock_ >= 0.02f) {
    update_engines(engine_clock_, aircraft_x, aircraft_z, aircraft_heading_deg);
    engine_clock_ = 0.0f;
  }

  for (auto& pack : packs_) {
    const float hide_m = pack.settings.hide_aircraft_agl_ft * 0.3048f;
    if (pack.settings.hide_aircraft_agl_ft > 0.0f && aircraft_agl_m >= hide_m) {
      // Do the expensive destroy pass only once while the aircraft remains
      // above the cutoff. This matters on low-end CPUs with thousands of
      // candidate grass tiles.
      if (!pack.altitude_suspended) {
        destroy_pack_instances(pack);
        pack.altitude_suspended = true;
      }
      pack.refresh_clock = 0.0f;
      pack.animation_clock = 0.0f;
      continue;
    }
    if (pack.altitude_suspended) {
      pack.altitude_suspended = false;
      // Rebuild the camera-visible ring immediately after descent.
      pack.refresh_clock = pack.settings.refresh_interval_s;
      pack.animation_clock = pack.settings.animation_interval_s;
    }

    pack.refresh_clock += elapsed;
    pack.animation_clock += elapsed;
    if (pack.refresh_clock >= pack.settings.refresh_interval_s) {
      pack.refresh_clock = 0.0f;
      refresh_active_set(pack, camera_x, camera_z);
    }
    if (pack.animation_clock < pack.settings.animation_interval_s) continue;
    const float dt = pack.animation_clock;
    pack.animation_clock = 0.0f;
    cached_wind_ = ambient_wind_world(pack.settings.wind_strength,
                                      pack.settings.wind_full_bend_kt,
                                      pack.settings.weather_wind,
                                      &cached_wind_speed_mps_);
    for (auto& tile : pack.tiles)
      if (tile.instance)
        update_tile(pack, tile, camera_x, camera_z, aircraft_x, aircraft_z, dt);
  }
}

} // namespace ogr
