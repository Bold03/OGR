#include "ogr/dsf_reader.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace ogr::dsf {
namespace {

constexpr std::uint32_t fourcc(char a, char b, char c, char d) {
  return (static_cast<std::uint32_t>(static_cast<unsigned char>(a)) << 24) |
         (static_cast<std::uint32_t>(static_cast<unsigned char>(b)) << 16) |
         (static_cast<std::uint32_t>(static_cast<unsigned char>(c)) << 8) |
         static_cast<std::uint32_t>(static_cast<unsigned char>(d));
}

constexpr std::uint32_t kDEFN = fourcc('D','E','F','N');
constexpr std::uint32_t kPOLY = fourcc('P','O','L','Y');
constexpr std::uint32_t kGEOD = fourcc('G','E','O','D');
constexpr std::uint32_t kPOOL = fourcc('P','O','O','L');
constexpr std::uint32_t kSCAL = fourcc('S','C','A','L');
constexpr std::uint32_t kCMDS = fourcc('C','M','D','S');
constexpr double kRecip65535 = 1.0 / 65535.0;

struct Cursor {
  const std::vector<std::uint8_t>* bytes{};
  std::size_t pos{};
  std::size_t end{};

  bool can(std::size_t n) const { return bytes && pos <= end && n <= end - pos; }

  std::uint8_t u8() {
    if (!can(1)) throw std::runtime_error("unexpected end of DSF");
    return (*bytes)[pos++];
  }
  std::uint16_t u16() {
    if (!can(2)) throw std::runtime_error("unexpected end of DSF");
    const auto* p = bytes->data() + pos;
    pos += 2;
    return static_cast<std::uint16_t>(p[0] | (static_cast<std::uint16_t>(p[1]) << 8));
  }
  std::uint32_t u32() {
    if (!can(4)) throw std::runtime_error("unexpected end of DSF");
    const auto* p = bytes->data() + pos;
    pos += 4;
    return static_cast<std::uint32_t>(p[0]) |
           (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) |
           (static_cast<std::uint32_t>(p[3]) << 24);
  }
  float f32() {
    const std::uint32_t bits = u32();
    float value{};
    static_assert(sizeof(value) == sizeof(bits));
    std::memcpy(&value, &bits, sizeof(value));
    return value;
  }
  void skip(std::size_t n) {
    if (!can(n)) throw std::runtime_error("unexpected end of DSF");
    pos += n;
  }
};

struct Atom {
  std::uint32_t id{};
  std::size_t begin{};
  std::size_t end{};
};

std::vector<Atom> atoms_in(const std::vector<std::uint8_t>& bytes,
                           std::size_t begin, std::size_t end) {
  std::vector<Atom> result;
  Cursor c{&bytes, begin, end};
  while (c.pos < c.end) {
    if (!c.can(8)) throw std::runtime_error("truncated DSF atom header");
    const std::uint32_t id = c.u32();
    const std::uint32_t size = c.u32();
    if (size < 8) throw std::runtime_error("invalid DSF atom size");
    const std::size_t payload_begin = c.pos;
    const std::size_t payload_end = payload_begin + static_cast<std::size_t>(size - 8);
    if (payload_end < payload_begin || payload_end > end)
      throw std::runtime_error("DSF atom exceeds container");
    result.push_back({id, payload_begin, payload_end});
    c.pos = payload_end;
  }
  return result;
}

const Atom* first_atom(const std::vector<Atom>& atoms, std::uint32_t id) {
  for (const auto& atom : atoms) if (atom.id == id) return &atom;
  return nullptr;
}

std::string normalize_resource(std::string value) {
  std::replace(value.begin(), value.end(), '\\', '/');
  while (value.rfind("./", 0) == 0) value.erase(0, 2);
  while (!value.empty() && value.front() == '/') value.erase(value.begin());
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  return value;
}

std::vector<std::string> string_table(const std::vector<std::uint8_t>& bytes,
                                      const Atom& atom) {
  std::vector<std::string> strings;
  std::size_t start = atom.begin;
  for (std::size_t i = atom.begin; i < atom.end; ++i) {
    if (bytes[i] != 0) continue;
    strings.emplace_back(reinterpret_cast<const char*>(bytes.data() + start), i - start);
    start = i + 1;
  }
  if (start != atom.end) throw std::runtime_error("unterminated DSF string table");
  return strings;
}

struct RawPool {
  std::uint32_t count{};
  std::uint8_t depth{};
  std::vector<std::vector<std::uint16_t>> planes;
};

std::vector<std::uint16_t> decode_plane(Cursor& c, std::uint32_t count,
                                        std::uint8_t encoding) {
  if (encoding > 3) throw std::runtime_error("unknown DSF planar encoding");
  std::vector<std::uint16_t> values;
  values.reserve(count);
  const bool rle = (encoding & 2u) != 0;
  const bool differenced = (encoding & 1u) != 0;

  if (!rle) {
    while (values.size() < count) values.push_back(c.u16());
  } else {
    while (values.size() < count) {
      const std::uint8_t control = c.u8();
      const std::size_t n = static_cast<std::size_t>(control & 0x7fu);
      if (n == 0) throw std::runtime_error("invalid zero-length DSF RLE run");
      if ((control & 0x80u) != 0) {
        const std::uint16_t value = c.u16();
        if (n > count - values.size()) throw std::runtime_error("DSF RLE run exceeds plane");
        values.insert(values.end(), n, value);
      } else {
        if (n > count - values.size()) throw std::runtime_error("DSF RLE literal exceeds plane");
        for (std::size_t i = 0; i < n; ++i) values.push_back(c.u16());
      }
    }
  }

  if (differenced) {
    std::uint16_t previous = 0;
    for (auto& value : values) {
      value = static_cast<std::uint16_t>(previous + value); // uint16 wrap is intentional.
      previous = value;
    }
  }
  return values;
}

RawPool parse_pool(const std::vector<std::uint8_t>& bytes, const Atom& atom) {
  Cursor c{&bytes, atom.begin, atom.end};
  RawPool pool;
  pool.count = c.u32();
  pool.depth = c.u8();
  if (pool.depth < 2) throw std::runtime_error("DSF point pool has fewer than two planes");
  pool.planes.reserve(pool.depth);
  for (std::uint8_t plane = 0; plane < pool.depth; ++plane) {
    const std::uint8_t encoding = c.u8();
    pool.planes.push_back(decode_plane(c, pool.count, encoding));
  }
  if (c.pos != c.end) throw std::runtime_error("unexpected bytes at end of DSF POOL atom");
  return pool;
}

std::vector<float> parse_scale(const std::vector<std::uint8_t>& bytes, const Atom& atom) {
  Cursor c{&bytes, atom.begin, atom.end};
  std::vector<float> out;
  while (c.pos < c.end) out.push_back(c.f32());
  return out;
}

struct Pool {
  std::uint32_t count{};
  std::uint8_t depth{};
  std::vector<std::vector<std::uint16_t>> raw;
  std::vector<float> scale;

  Point point(std::uint32_t index) const {
    if (index >= count || depth < 2 || raw.size() < 2 || scale.size() < static_cast<std::size_t>(depth) * 2)
      throw std::runtime_error("DSF polygon coordinate index out of range");
    auto decode = [&](std::size_t plane) {
      const double multiplier = scale[plane * 2];
      const double offset = scale[plane * 2 + 1];
      const double value = raw[plane][index];
      return multiplier != 0.0 ? value * multiplier * kRecip65535 + offset : value;
    };
    return {decode(0), decode(1)}; // DSF polygon pools are longitude, latitude.
  }
};

std::vector<std::uint8_t> read_file(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("cannot open DSF");
  input.seekg(0, std::ios::end);
  const auto end = input.tellg();
  if (end < 0) throw std::runtime_error("cannot size DSF");
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(end));
  input.seekg(0, std::ios::beg);
  if (!bytes.empty()) input.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
  if (!input && !bytes.empty()) throw std::runtime_error("cannot read DSF");
  return bytes;
}

std::vector<Point> points_from_indices(const Pool& pool,
                                       const std::vector<std::uint16_t>& indices) {
  std::vector<Point> points;
  points.reserve(indices.size());
  for (const auto index : indices) points.push_back(pool.point(index));
  return points;
}

} // namespace

ReadResult read_forest_polygons(const std::filesystem::path& dsf_path,
                                const std::string& resource_path) {
  ReadResult result;
  try {
    const auto bytes = read_file(dsf_path);
    static constexpr std::array<std::uint8_t, 6> k7z{{0x37,0x7a,0xbc,0xaf,0x27,0x1c}};
    if (bytes.size() >= k7z.size() && std::equal(k7z.begin(), k7z.end(), bytes.begin())) {
      result.compressed = true;
      result.error = "7z-compressed DSF is not supported by the direct reader yet";
      return result;
    }
    if (bytes.size() < 12 + 16 || std::memcmp(bytes.data(), "XPLNEDSF", 8) != 0) {
      result.error = "not an uncompressed X-Plane DSF";
      return result;
    }
    Cursor header{&bytes, 8, bytes.size()};
    if (header.u32() != 1u) {
      result.error = "unsupported DSF master version";
      return result;
    }

    const auto roots = atoms_in(bytes, 12, bytes.size() - 16);
    const Atom* defn = first_atom(roots, kDEFN);
    const Atom* geod = first_atom(roots, kGEOD);
    const Atom* cmds = first_atom(roots, kCMDS);
    if (!defn || !geod || !cmds) throw std::runtime_error("DSF is missing DEFN, GEOD or CMDS atom");

    const auto definitions = atoms_in(bytes, defn->begin, defn->end);
    const Atom* poly_atom = first_atom(definitions, kPOLY);
    if (!poly_atom) throw std::runtime_error("DSF has no polygon definition table");
    const auto polygon_defs = string_table(bytes, *poly_atom);
    const std::string target = normalize_resource(resource_path);
    std::unordered_set<std::uint32_t> target_defs;
    for (std::size_t i = 0; i < polygon_defs.size(); ++i) {
      if (normalize_resource(polygon_defs[i]) == target)
        target_defs.insert(static_cast<std::uint32_t>(i));
    }
    if (target_defs.empty()) {
      result.ok = true;
      return result;
    }

    const auto geo_atoms = atoms_in(bytes, geod->begin, geod->end);
    std::vector<RawPool> raw_pools;
    std::vector<std::vector<float>> scales;
    for (const auto& atom : geo_atoms) {
      if (atom.id == kPOOL) raw_pools.push_back(parse_pool(bytes, atom));
      else if (atom.id == kSCAL) scales.push_back(parse_scale(bytes, atom));
    }
    if (raw_pools.size() != scales.size()) throw std::runtime_error("POOL/SCAL count mismatch");
    std::vector<Pool> pools;
    pools.reserve(raw_pools.size());
    for (std::size_t i = 0; i < raw_pools.size(); ++i) {
      if (scales[i].size() != static_cast<std::size_t>(raw_pools[i].depth) * 2)
        throw std::runtime_error("SCAL plane count does not match POOL");
      Pool pool;
      pool.count = raw_pools[i].count;
      pool.depth = raw_pools[i].depth;
      pool.raw = std::move(raw_pools[i].planes);
      pool.scale = std::move(scales[i]);
      pools.push_back(std::move(pool));
    }

    Cursor c{&bytes, cmds->begin, cmds->end};
    std::uint32_t current_definition = std::numeric_limits<std::uint32_t>::max();
    std::uint16_t current_pool = std::numeric_limits<std::uint16_t>::max();

    auto active_pool = [&]() -> const Pool& {
      if (current_pool >= pools.size()) throw std::runtime_error("polygon uses invalid DSF point pool");
      return pools[current_pool];
    };
    auto emit_single = [&](std::uint16_t parameter, const std::vector<std::uint16_t>& indices) {
      if (!target_defs.count(current_definition) || indices.size() < 3) return;
      Polygon polygon;
      polygon.parameter = parameter;
      polygon.windings.push_back(points_from_indices(active_pool(), indices));
      result.polygons.push_back(std::move(polygon));
    };
    auto emit_nested = [&](std::uint16_t parameter,
                           const std::vector<std::vector<std::uint16_t>>& windings) {
      if (!target_defs.count(current_definition)) return;
      Polygon polygon;
      polygon.parameter = parameter;
      for (const auto& indices : windings)
        if (indices.size() >= 3) polygon.windings.push_back(points_from_indices(active_pool(), indices));
      if (!polygon.windings.empty()) result.polygons.push_back(std::move(polygon));
    };

    while (c.pos < c.end) {
      const std::uint8_t cmd = c.u8();
      switch (cmd) {
        case 0: throw std::runtime_error("reserved DSF command 0");
        case 1: current_pool = c.u16(); break;
        case 2: c.skip(4); break;
        case 3: current_definition = c.u8(); break;
        case 4: current_definition = c.u16(); break;
        case 5: current_definition = c.u32(); break;
        case 6: c.skip(1); break;
        case 7: c.skip(2); break;
        case 8: c.skip(4); break;
        case 9: { const auto n = c.u8(); c.skip(static_cast<std::size_t>(n) * 2); break; }
        case 10: c.skip(4); break;
        case 11: { const auto n = c.u8(); c.skip(static_cast<std::size_t>(n) * 4); break; }
        case 12: {
          const auto param = c.u16();
          const auto n = c.u8();
          std::vector<std::uint16_t> indices; indices.reserve(n);
          for (std::uint8_t i = 0; i < n; ++i) indices.push_back(c.u16());
          emit_single(param, indices);
          break;
        }
        case 13: {
          const auto param = c.u16();
          const auto first = c.u16();
          const auto last = c.u16();
          if (last < first) throw std::runtime_error("invalid DSF polygon range");
          std::vector<std::uint16_t> indices; indices.reserve(last - first);
          for (std::uint32_t i = first; i < last; ++i) indices.push_back(static_cast<std::uint16_t>(i));
          emit_single(param, indices);
          break;
        }
        case 14: {
          const auto param = c.u16();
          const auto winding_count = c.u8();
          std::vector<std::vector<std::uint16_t>> windings;
          windings.reserve(winding_count);
          for (std::uint8_t w = 0; w < winding_count; ++w) {
            const auto n = c.u8();
            std::vector<std::uint16_t> indices; indices.reserve(n);
            for (std::uint8_t i = 0; i < n; ++i) indices.push_back(c.u16());
            windings.push_back(std::move(indices));
          }
          emit_nested(param, windings);
          break;
        }
        case 15: {
          const auto param = c.u16();
          const auto winding_count = c.u8();
          // DSF stores one starting index plus one ending index per winding,
          // i.e. winding_count + 1 range boundaries.
          std::vector<std::uint16_t> cuts; cuts.reserve(static_cast<std::size_t>(winding_count) + 1);
          for (std::size_t i = 0; i < static_cast<std::size_t>(winding_count) + 1; ++i)
            cuts.push_back(c.u16());
          std::vector<std::vector<std::uint16_t>> windings;
          windings.reserve(winding_count);
          for (std::size_t w = 0; w < winding_count; ++w) {
            if (cuts[w + 1] < cuts[w]) throw std::runtime_error("invalid DSF nested polygon range");
            std::vector<std::uint16_t> indices;
            indices.reserve(cuts[w + 1] - cuts[w]);
            for (std::uint32_t i = cuts[w]; i < cuts[w + 1]; ++i)
              indices.push_back(static_cast<std::uint16_t>(i));
            windings.push_back(std::move(indices));
          }
          emit_nested(param, windings);
          break;
        }
        case 16: break;
        case 17: c.skip(1); break;
        case 18: c.skip(9); break;
        case 23: case 26: case 29: {
          const auto n = c.u8(); c.skip(static_cast<std::size_t>(n) * 2); break;
        }
        case 24: case 27: case 30: {
          const auto n = c.u8(); c.skip(static_cast<std::size_t>(n) * 4); break;
        }
        case 25: case 28: case 31: c.skip(4); break;
        case 32: { const auto n = c.u8(); c.skip(n); break; }
        case 33: { const auto n = c.u16(); c.skip(n); break; }
        case 34: { const auto n = c.u32(); c.skip(n); break; }
        default: throw std::runtime_error("unsupported DSF command id " + std::to_string(cmd));
      }
    }

    result.ok = true;
  } catch (const std::exception& e) {
    result.error = e.what();
  }
  return result;
}

} // namespace ogr::dsf
