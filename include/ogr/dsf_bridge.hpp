#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace ogr {

struct DsfRefreshStats {
  std::size_t packs_seen{};
  std::size_t packs_updated{};
  std::size_t areas_written{};
  std::size_t compressed_dsfs{};
  std::size_t parse_errors{};
  std::vector<std::string> messages;
};

// Rebuilds OafishGrass/ogr_areas.json directly from exported WED DSFs.
// The JSON remains an internal compatibility hand-off to the proven v0.1
// runtime; scenery authors no longer need to run BUILD_GRASS_AREAS.cmd.
DsfRefreshStats refresh_direct_dsf_areas(const std::filesystem::path& xplane_root);

} // namespace ogr
