#include "ogr/dsf_bridge.hpp"
#include "ogr/dsf_reader.hpp"

#include <nlohmann/json.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <unordered_set>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace ogr {
namespace {

constexpr const char* kOgrResource = "OafishGrass/ogr_grass.for";

std::string trim(std::string value) {
  auto not_space = [](unsigned char c) { return !std::isspace(c); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  if (value.size() >= 2 && value.front() == '"' && value.back() == '"')
    value = value.substr(1, value.size() - 2);
  return value;
}

std::string normalized_resource(std::string value) {
  std::replace(value.begin(), value.end(), '\\', '/');
  while (value.rfind("./", 0) == 0) value.erase(0, 2);
  return value;
}

std::vector<fs::path> scenery_roots(const fs::path& xplane_root) {
  std::vector<fs::path> roots;
  std::unordered_set<std::string> seen;
  const fs::path custom = xplane_root / "Custom Scenery";
  const fs::path ini = custom / "scenery_packs.ini";
  std::ifstream input(ini);
  if (input) {
    std::string line;
    while (std::getline(input, line)) {
      constexpr const char* prefix = "SCENERY_PACK ";
      if (line.rfind(prefix, 0) != 0) continue;
      std::string relative = trim(line.substr(std::char_traits<char>::length(prefix)));
      if (relative.empty()) continue;
      std::replace(relative.begin(), relative.end(), '\\', '/');
      fs::path path = xplane_root / fs::path(relative);
      std::error_code ec;
      if (!fs::is_directory(path, ec)) continue;
      const std::string key = path.lexically_normal().generic_string();
      if (seen.insert(key).second) roots.push_back(std::move(path));
    }
    return roots;
  }

  std::error_code ec;
  if (!fs::is_directory(custom, ec)) return roots;
  for (const auto& entry : fs::directory_iterator(custom, ec)) {
    if (ec) break;
    if (!entry.is_directory(ec)) continue;
    const std::string key = entry.path().lexically_normal().generic_string();
    if (seen.insert(key).second) roots.push_back(entry.path());
  }
  return roots;
}

bool parse_bool(const std::string& value) {
  std::string lowered = value;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return lowered != "0" && lowered != "false" && lowered != "off" && lowered != "no";
}

struct ForMetadata {
  bool valid{};
  std::vector<std::string> models;
  json settings = json::object();
};

ForMetadata parse_for_metadata(const fs::path& for_path, const fs::path& scenery_root) {
  ForMetadata out;
  std::ifstream input(for_path);
  if (!input) return out;

  std::string line;
  while (std::getline(input, line)) {
    line = trim(line);
    if (line.rfind("#OGR_", 0) != 0) continue;
    const std::string payload = trim(line.substr(5));
    if (payload.empty()) continue;
    const auto split = payload.find_first_of(" \t");
    std::string key = split == std::string::npos ? payload : payload.substr(0, split);
    const std::string value = split == std::string::npos ? std::string{} : trim(payload.substr(split + 1));
    std::transform(key.begin(), key.end(), key.begin(), [](unsigned char ch) {
      return static_cast<char>(std::toupper(ch));
    });

    if (key == "VERSION") {
      out.valid = true;
      continue;
    }
    if (key == "MODEL" && !value.empty()) {
      std::error_code ec;
      const fs::path full = fs::weakly_canonical(for_path.parent_path() / fs::path(value), ec);
      const fs::path root = fs::weakly_canonical(scenery_root, ec);
      if (ec) continue;
      const fs::path rel = full.lexically_relative(root);
      const std::string model = rel.generic_string();
      if (model.empty() || model == ".." || model.rfind("../", 0) == 0) continue;
      if (std::find(out.models.begin(), out.models.end(), model) == out.models.end())
        out.models.push_back(model);
      continue;
    }

    auto put_float = [&](const char* json_key) {
      try { out.settings[json_key] = std::stof(value); } catch (...) {}
    };
    auto put_int = [&](const char* json_key) {
      try { out.settings[json_key] = std::stoi(value); } catch (...) {}
    };

    if (key == "TILE_SIZE_M") put_float("tile_size_m");
    else if (key == "DRAW_DISTANCE_M") put_float("draw_distance_m");
    else if (key == "ANIMATED_DISTANCE_M") put_float("animated_distance_m");
    else if (key == "BOUNDARY_MARGIN_M") put_float("boundary_margin_m");
    else if (key == "GROUND_OFFSET_M") put_float("ground_offset_m");
    else if (key == "WIND_STRENGTH") put_float("wind_strength");
    else if (key == "WIND_FULL_BEND_KT") put_float("wind_full_bend_kt");
    else if (key == "WEATHER_WIND") out.settings["weather_wind"] = parse_bool(value);
    else if (key == "ENGINE_WASH") out.settings["engine_wash"] = parse_bool(value);
    else if (key == "ENGINE_WASH_STRENGTH") put_float("engine_wash_strength");
    else if (key == "ENGINE_WASH_RANGE_M") put_float("engine_wash_range_m");
    else if (key == "ENGINE_WASH_HALF_ANGLE_DEG") put_float("engine_wash_half_angle_deg");
    else if (key == "ENGINE_WASH_BASE_HALF_WIDTH_M") put_float("engine_wash_base_half_width_m");
    else if (key == "MAX_ACTIVE_TILES") put_int("max_active_tiles");
    else if (key == "MAX_TOTAL_TILES") put_int("max_total_tiles");
    else if (key == "REFRESH_INTERVAL_S") put_float("refresh_interval_s");
    else if (key == "ANIMATION_INTERVAL_S") put_float("animation_interval_s");
    else if (key == "HIDE_AIRCRAFT_AGL_FT") put_float("hide_aircraft_agl_ft");
    else if (key == "SCATTER_MODE") out.settings["scatter_mode"] = value;
    else if (key == "CLUSTER_SPACING_M") put_float("cluster_spacing_m");
    else if (key == "CLUSTER_RADIUS_M") put_float("cluster_radius_m");
    else if (key == "CLUSTER_PROBABILITY") put_float("cluster_probability");
  }
  return out;
}

struct LibraryProvider {
  bool valid{};
  fs::path root;
  fs::path for_path;
  ForMetadata meta;
};

bool find_library_provider(const std::vector<fs::path>& roots, LibraryProvider& out) {
  for (const auto& root : roots) {
    const fs::path library_txt = root / "library.txt";
    std::ifstream input(library_txt);
    if (!input) continue;

    std::string line;
    while (std::getline(input, line)) {
      line = trim(line);
      if (line.empty() || line[0] == '#') continue;
      std::istringstream row(line);
      std::string command, virtual_path, physical_path;
      row >> command >> virtual_path >> physical_path;
      if (command != "EXPORT" && command != "EXPORT_BACKUP" && command != "EXPORT_EXTEND")
        continue;
      if (normalized_resource(virtual_path) != kOgrResource || physical_path.empty()) continue;

      const fs::path for_path = root / fs::path(normalized_resource(physical_path));
      std::error_code ec;
      if (!fs::is_regular_file(for_path, ec)) continue;
      ForMetadata meta = parse_for_metadata(for_path, root);
      if (!meta.valid || meta.models.empty()) continue;

      out.valid = true;
      out.root = root;
      out.for_path = for_path;
      out.meta = std::move(meta);
      return true;
    }
  }
  return false;
}

double ring_area(const std::vector<dsf::Point>& points) {
  if (points.size() < 3) return 0.0;
  double mean_lat = 0.0;
  for (const auto& p : points) mean_lat += p.latitude;
  mean_lat /= static_cast<double>(points.size());
  const double cos_lat = std::cos(mean_lat * 3.14159265358979323846 / 180.0);
  double total = 0.0;
  for (std::size_t i = 0; i < points.size(); ++i) {
    const auto& p = points[i];
    const auto& q = points[(i + 1) % points.size()];
    total += (p.longitude * cos_lat) * q.latitude - (q.longitude * cos_lat) * p.latitude;
  }
  return 0.5 * total;
}

json ring_json(const std::vector<dsf::Point>& ring) {
  json out = json::array();
  for (const auto& p : ring)
    out.push_back({{"latitude", p.latitude}, {"longitude", p.longitude}});
  return out;
}

void append_polygons(const fs::path& scenery_root, const fs::path& dsf_path,
                     const dsf::ReadResult& read, json& areas, std::size_t& area_index) {
  for (const auto& polygon : read.polygons) {
    const unsigned fill_mode = polygon.parameter / 256u;
    if (fill_mode != 0u || polygon.windings.empty()) continue;
    std::size_t outer_index = 0;
    double best = -1.0;
    for (std::size_t i = 0; i < polygon.windings.size(); ++i) {
      const double a = std::abs(ring_area(polygon.windings[i]));
      if (a > best) { best = a; outer_index = i; }
    }
    if (polygon.windings[outer_index].size() < 3) continue;

    json holes = json::array();
    for (std::size_t i = 0; i < polygon.windings.size(); ++i) {
      if (i != outer_index && polygon.windings[i].size() >= 3)
        holes.push_back(ring_json(polygon.windings[i]));
    }
    const float density = std::clamp(static_cast<float>(polygon.parameter % 256u) / 255.0f,
                                     0.03f, 1.0f);
    const std::string id = "dsf_" + std::to_string(++area_index);
    areas.push_back({
        {"id", id},
        {"name", scenery_root.filename().string() + " / " + dsf_path.stem().string()},
        {"density", density},
        {"outer", ring_json(polygon.windings[outer_index])},
        {"holes", std::move(holes)},
    });
  }
}

bool write_direct_json(const fs::path& scenery_root, const ForMetadata& meta,
                       const json& areas, const std::string& source, std::string& error) {
  const fs::path output = scenery_root / "OafishGrass" / "ogr_areas.json";
  std::error_code ec;
  fs::create_directories(output.parent_path(), ec);
  if (ec) { error = "cannot create " + output.parent_path().string(); return false; }

  json root = {
      {"format", "OGR_AREA_V1"},
      {"name", scenery_root.filename().string()},
      {"source", source},
      {"resource", kOgrResource},
      {"models", meta.models},
      {"settings", meta.settings},
      {"areas", areas},
  };
  std::ofstream file(output, std::ios::binary | std::ios::trunc);
  if (!file) { error = "cannot write " + output.string(); return false; }
  file << root.dump(2) << '\n';
  if (!file) { error = "failed while writing " + output.string(); return false; }
  return true;
}

void remove_generated_local_jsons(const std::vector<fs::path>& roots,
                                  const fs::path& provider_root,
                                  DsfRefreshStats& stats) {
  for (const auto& root : roots) {
    if (root.lexically_normal() == provider_root.lexically_normal()) continue;
    const fs::path path = root / "OafishGrass" / "ogr_areas.json";
    std::ifstream input(path);
    if (!input) continue;
    try {
      const json existing = json::parse(input);
      const std::string source = existing.value("source", std::string());
      if (source.rfind("direct-dsf", 0) != 0 && source != "earth.wed.xml") continue;
      std::error_code ec;
      fs::remove(path, ec);
      if (!ec)
        stats.messages.push_back("Shared library migration: removed generated local area cache from " +
                                 root.filename().string());
    } catch (...) {
      // Never delete a malformed/unknown user file.
    }
  }
}

DsfRefreshStats refresh_shared_library(const fs::path& xplane_root,
                                       const std::vector<fs::path>& roots,
                                       const LibraryProvider& provider) {
  DsfRefreshStats stats;
  stats.packs_seen = 1;
  json areas = json::array();
  std::size_t area_index = 0;
  std::size_t dsf_count = 0;
  std::size_t skipped_compressed = 0;
  std::size_t skipped_errors = 0;

  for (const auto& scenery_root : roots) {
    if (scenery_root.lexically_normal() == provider.root.lexically_normal()) continue;
    const fs::path earth_nav = scenery_root / "Earth nav data";
    std::error_code ec;
    if (!fs::is_directory(earth_nav, ec)) continue;

    for (fs::recursive_directory_iterator it(earth_nav, ec), end; it != end && !ec; it.increment(ec)) {
      if (ec || !it->is_regular_file(ec) || it->path().extension() != ".dsf") continue;
      ++dsf_count;
      const auto read = dsf::read_forest_polygons(it->path(), kOgrResource);
      if (read.compressed) {
        ++stats.compressed_dsfs;
        ++skipped_compressed;
        continue;
      }
      if (!read.ok) {
        ++stats.parse_errors;
        ++skipped_errors;
        continue;
      }
      append_polygons(scenery_root, it->path(), read, areas, area_index);
    }
    if (ec) {
      ++stats.parse_errors;
      ++skipped_errors;
    }
  }

  if (dsf_count == 0) {
    stats.messages.push_back("Shared OGR library found, but no scenery DSFs were available to scan");
    return stats;
  }

  if (areas.empty()) {
    if (skipped_compressed || skipped_errors) {
      stats.messages.push_back("Shared OGR library scan found no readable OGR areas; existing cache preserved");
      return stats;
    }
    const fs::path old_json = provider.root / "OafishGrass" / "ogr_areas.json";
    std::error_code ec;
    fs::remove(old_json, ec);
    stats.messages.push_back("Shared OGR library found no OGR Area forests in active scenery");
    return stats;
  }

  std::string error;
  if (!write_direct_json(provider.root, provider.meta, areas, "shared-library-direct-dsf-v0.3", error)) {
    ++stats.parse_errors;
    stats.messages.push_back("Shared OGR library write error: " + error);
    return stats;
  }

  remove_generated_local_jsons(roots, provider.root, stats);
  ++stats.packs_updated;
  stats.areas_written = areas.size();
  stats.messages.push_back("Shared OGR library rebuilt from active scenery: " +
                           std::to_string(areas.size()) + " grass area(s)");
  if (skipped_compressed)
    stats.messages.push_back("Shared OGR library skipped " + std::to_string(skipped_compressed) +
                             " compressed DSF(s)");
  if (skipped_errors)
    stats.messages.push_back("Shared OGR library skipped " + std::to_string(skipped_errors) +
                             " unreadable DSF(s)");
  return stats;
}

} // namespace

DsfRefreshStats refresh_direct_dsf_areas(const fs::path& xplane_root) {
  const auto roots = scenery_roots(xplane_root);

  LibraryProvider provider;
  if (find_library_provider(roots, provider)) {
    auto stats = refresh_shared_library(xplane_root, roots, provider);
    stats.messages.insert(stats.messages.begin(),
                          "Shared OGR library provider: " + provider.root.filename().string());
    return stats;
  }

  DsfRefreshStats stats;
  for (const auto& scenery_root : roots) {
    const fs::path for_path = scenery_root / "OafishGrass" / "ogr_grass.for";
    std::error_code ec;
    if (!fs::is_regular_file(for_path, ec)) continue;
    ++stats.packs_seen;

    const ForMetadata meta = parse_for_metadata(for_path, scenery_root);
    if (!meta.valid || meta.models.empty()) {
      ++stats.parse_errors;
      stats.messages.push_back("Direct DSF skipped " + scenery_root.filename().string() +
                               ": invalid #OGR metadata in OafishGrass/ogr_grass.for");
      continue;
    }

    const fs::path earth_nav = scenery_root / "Earth nav data";
    if (!fs::is_directory(earth_nav, ec)) {
      stats.messages.push_back("Direct DSF skipped " + scenery_root.filename().string() +
                               ": Earth nav data folder not found");
      continue;
    }

    json areas = json::array();
    bool scan_failed = false;
    std::size_t area_index = 0;
    std::size_t dsf_count = 0;
    for (fs::recursive_directory_iterator it(earth_nav, ec), end; it != end && !ec; it.increment(ec)) {
      if (ec || !it->is_regular_file(ec)) continue;
      if (it->path().extension() != ".dsf") continue;
      ++dsf_count;
      const auto read = dsf::read_forest_polygons(it->path(), kOgrResource);
      if (read.compressed) {
        ++stats.compressed_dsfs;
        scan_failed = true;
        stats.messages.push_back("Direct DSF fallback for " + scenery_root.filename().string() +
                                 ": compressed DSF " + it->path().filename().string());
        break;
      }
      if (!read.ok) {
        ++stats.parse_errors;
        scan_failed = true;
        stats.messages.push_back("Direct DSF parse error in " + it->path().string() + ": " + read.error);
        break;
      }
      append_polygons(scenery_root, it->path(), read, areas, area_index);
    }
    if (ec) {
      ++stats.parse_errors;
      scan_failed = true;
      stats.messages.push_back("Direct DSF directory scan error in " + earth_nav.string());
    }
    if (scan_failed) continue;
    if (dsf_count == 0) continue;

    if (areas.empty()) {
      const fs::path old_json = scenery_root / "OafishGrass" / "ogr_areas.json";
      fs::remove(old_json, ec);
      stats.messages.push_back("Direct DSF found no OGR Area forests in " + scenery_root.filename().string());
      continue;
    }

    std::string error;
    if (!write_direct_json(scenery_root, meta, areas, "direct-dsf-v0.3", error)) {
      ++stats.parse_errors;
      stats.messages.push_back("Direct DSF write error: " + error);
      continue;
    }
    ++stats.packs_updated;
    stats.areas_written += areas.size();
    stats.messages.push_back("Direct DSF rebuilt " + scenery_root.filename().string() + ": " +
                             std::to_string(areas.size()) + " grass area(s)");
  }
  return stats;
}

} // namespace ogr
