#include "ogr/dsf_reader.hpp"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {
using Bytes = std::vector<std::uint8_t>;

std::uint32_t fourcc(char a, char b, char c, char d) {
  return (std::uint32_t(std::uint8_t(a)) << 24) | (std::uint32_t(std::uint8_t(b)) << 16) |
         (std::uint32_t(std::uint8_t(c)) << 8) | std::uint32_t(std::uint8_t(d));
}
void u8(Bytes& b, std::uint8_t v) { b.push_back(v); }
void u16(Bytes& b, std::uint16_t v) { b.push_back(v & 255); b.push_back((v >> 8) & 255); }
void u32(Bytes& b, std::uint32_t v) {
  for (int i = 0; i < 4; ++i) b.push_back((v >> (i * 8)) & 255);
}
void f32(Bytes& b, float v) {
  std::uint32_t bits{}; std::memcpy(&bits, &v, sizeof(v)); u32(b, bits);
}
Bytes atom(std::uint32_t id, const Bytes& payload) {
  Bytes out; u32(out, id); u32(out, static_cast<std::uint32_t>(payload.size() + 8));
  out.insert(out.end(), payload.begin(), payload.end()); return out;
}
void append(Bytes& dst, const Bytes& src) { dst.insert(dst.end(), src.begin(), src.end()); }

Bytes make_dsf() {
  Bytes poly_defs;
  const std::string resource = "OafishGrass/ogr_grass.for";
  poly_defs.insert(poly_defs.end(), resource.begin(), resource.end()); poly_defs.push_back(0);
  Bytes defn; append(defn, atom(fourcc('P','O','L','Y'), poly_defs));

  // Eight points: outer square (0..3), inner square (4..7). Longitudes and
  // latitudes use RLE-differenced encoding to exercise the real WED-style pool decoder.
  const std::uint16_t lon_abs[8] = {1000, 3000, 3000, 1000, 1600, 2400, 2400, 1600};
  const std::uint16_t lat_abs[8] = {1000, 1000, 3000, 3000, 1600, 1600, 2400, 2400};
  auto append_diff_rle = [](Bytes& p, const std::uint16_t* abs, int count) {
    u8(p, 3); // RLE + differenced
    u8(p, static_cast<std::uint8_t>(count)); // literal run
    std::uint16_t prev = 0;
    for (int i = 0; i < count; ++i) {
      const std::uint16_t d = static_cast<std::uint16_t>(abs[i] - prev);
      u16(p, d); prev = abs[i];
    }
  };
  Bytes pool; u32(pool, 8); u8(pool, 2); append_diff_rle(pool, lon_abs, 8); append_diff_rle(pool, lat_abs, 8);
  Bytes scal; f32(scal, 1.0f); f32(scal, 104.0f); f32(scal, 1.0f); f32(scal, 0.0f);
  Bytes geod; append(geod, atom(fourcc('P','O','O','L'), pool)); append(geod, atom(fourcc('S','C','A','L'), scal));

  Bytes cmds;
  u8(cmds, 1); u16(cmds, 0);     // pool 0
  u8(cmds, 3); u8(cmds, 0);      // definition 0
  u8(cmds, 14); u16(cmds, 255);  // nested area, 100% density
  u8(cmds, 2);                    // two windings
  u8(cmds, 4); for (int i = 0; i < 4; ++i) u16(cmds, i);
  u8(cmds, 4); for (int i = 4; i < 8; ++i) u16(cmds, i);
  u8(cmds, 15); u16(cmds, 128);  // nested range, about 50% density
  u8(cmds, 2);                    // two windings
  u16(cmds, 0); u16(cmds, 4); u16(cmds, 8);

  Bytes file;
  const char cookie[] = "XPLNEDSF"; file.insert(file.end(), cookie, cookie + 8); u32(file, 1);
  append(file, atom(fourcc('D','E','F','N'), defn));
  append(file, atom(fourcc('G','E','O','D'), geod));
  append(file, atom(fourcc('C','M','D','S'), cmds));
  file.resize(file.size() + 16, 0); // MD5 footer isn't validated by OGR.
  return file;
}

} // namespace

int main() {
  const auto path = std::filesystem::temp_directory_path() / "ogr_dsf_reader_test.dsf";
  const auto bytes = make_dsf();
  { std::ofstream out(path, std::ios::binary); out.write(reinterpret_cast<const char*>(bytes.data()), bytes.size()); }

  const auto result = ogr::dsf::read_forest_polygons(path, "OafishGrass/ogr_grass.for");
  std::filesystem::remove(path);
  assert(result.ok);
  assert(!result.compressed);
  assert(result.polygons.size() == 2);
  assert(result.polygons[0].parameter == 255);
  assert(result.polygons[0].windings.size() == 2);
  assert(result.polygons[1].parameter == 128);
  assert(result.polygons[1].windings.size() == 2);
  assert(result.polygons[0].windings[0].size() == 4);
  const auto& p = result.polygons[0].windings[0][0];
  assert(std::abs(p.longitude - (104.0 + 1000.0 / 65535.0)) < 1e-6);
  assert(std::abs(p.latitude - (1000.0 / 65535.0)) < 1e-6);

  std::cout << "OGR direct DSF reader test passed\n";
  return 0;
}
