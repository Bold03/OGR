#pragma once

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace ogr::dsf {

struct Point {
  double longitude{};
  double latitude{};
};

struct Polygon {
  std::uint16_t parameter{};
  std::vector<std::vector<Point>> windings;
};

struct ReadResult {
  bool ok{};
  bool compressed{};
  std::string error;
  std::vector<Polygon> polygons;
};

// Reads uncompressed DSF v1 polygon commands for one exact polygon resource.
// X-Plane 10+ can also read 7z-compressed DSFs; those are reported as
// compressed=true so the caller can keep a compatibility fallback.
ReadResult read_forest_polygons(const std::filesystem::path& dsf_path,
                                const std::string& resource_path);

} // namespace ogr::dsf
