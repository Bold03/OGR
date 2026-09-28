#!/usr/bin/env python3
"""Extract Oafish Grass Runtime forest areas from WED's earth.wed.xml.

OGR uses a normal X-Plane 11 .for file as a WED marker. The .for file remains
valid to WED/X-Plane, while #OGR_* comment lines carry runtime metadata.
This script converts matching WED Forest placements into OafishGrass/ogr_areas.json.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Iterable, List, Tuple

FORMAT = "OGR_AREA_V1"

SETTING_KEYS = {
    "TILE_SIZE_M": ("tile_size_m", float),
    "DRAW_DISTANCE_M": ("draw_distance_m", float),
    "ANIMATED_DISTANCE_M": ("animated_distance_m", float),
    "BOUNDARY_MARGIN_M": ("boundary_margin_m", float),
    "GROUND_OFFSET_M": ("ground_offset_m", float),
    "WIND_STRENGTH": ("wind_strength", float),
    "WIND_FULL_BEND_KT": ("wind_full_bend_kt", float),
    "WEATHER_WIND": ("weather_wind", lambda v: v.lower() not in {"0", "false", "off", "no"}),
    "ENGINE_WASH": ("engine_wash", lambda v: v.lower() not in {"0", "false", "off", "no"}),
    "ENGINE_WASH_STRENGTH": ("engine_wash_strength", float),
    "ENGINE_WASH_RANGE_M": ("engine_wash_range_m", float),
    "ENGINE_WASH_HALF_ANGLE_DEG": ("engine_wash_half_angle_deg", float),
    "ENGINE_WASH_BASE_HALF_WIDTH_M": ("engine_wash_base_half_width_m", float),
    "MAX_ACTIVE_TILES": ("max_active_tiles", int),
    "MAX_TOTAL_TILES": ("max_total_tiles", int),
    "REFRESH_INTERVAL_S": ("refresh_interval_s", float),
    "ANIMATION_INTERVAL_S": ("animation_interval_s", float),
    "HIDE_AIRCRAFT_AGL_FT": ("hide_aircraft_agl_ft", float),
    "SCATTER_MODE": ("scatter_mode", str),
    "CLUSTER_SPACING_M": ("cluster_spacing_m", float),
    "CLUSTER_RADIUS_M": ("cluster_radius_m", float),
    "CLUSTER_PROBABILITY": ("cluster_probability", float),
}


def normalize_resource(value: str) -> str:
    return value.replace("\\", "/").lstrip("./")


def parse_for_metadata(path: Path, scenery_root: Path) -> dict | None:
    if not path.is_file():
        return None
    version = None
    models: List[str] = []
    settings = {}
    try:
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return None

    for raw in lines:
        line = raw.strip()
        if not line.startswith("#OGR_"):
            continue
        payload = line[5:].strip()
        if not payload:
            continue
        parts = payload.split(None, 1)
        key = parts[0].upper()
        value = parts[1].strip() if len(parts) > 1 else ""
        if key == "VERSION":
            version = value or "1"
        elif key == "MODEL" and value:
            model_path = (path.parent / value).resolve()
            try:
                rel = model_path.relative_to(scenery_root.resolve())
            except ValueError:
                raise ValueError(f"OGR model escapes scenery pack: {value}")
            models.append(rel.as_posix())
        elif key in SETTING_KEYS and value:
            out_key, parser = SETTING_KEYS[key]
            settings[out_key] = parser(value)

    if version is None:
        return None
    if not models:
        raise ValueError(f"{path}: OGR resource has no #OGR_MODEL lines")
    return {"version": version, "models": models, "settings": settings}


def children_ids(obj: ET.Element) -> List[str]:
    children = obj.find("children")
    if children is None:
        return []
    return [child.attrib.get("id", "") for child in children.findall("child") if child.attrib.get("id")]


def node_data(obj: ET.Element) -> dict | None:
    point = obj.find("point")
    if point is None:
        return None
    try:
        lat = float(point.attrib["latitude"])
        lon = float(point.attrib["longitude"])
    except (KeyError, ValueError):
        return None
    return {
        "lat": lat,
        "lon": lon,
        "lat_lo": float(point.attrib.get("ctrl_latitude_lo", 0.0)),
        "lon_lo": float(point.attrib.get("ctrl_longitude_lo", 0.0)),
        "lat_hi": float(point.attrib.get("ctrl_latitude_hi", 0.0)),
        "lon_hi": float(point.attrib.get("ctrl_longitude_hi", 0.0)),
    }


def cubic(a: float, b: float, c: float, d: float, t: float) -> float:
    u = 1.0 - t
    return u * u * u * a + 3.0 * u * u * t * b + 3.0 * u * t * t * c + t * t * t * d


def sample_ring(ring: ET.Element, objects: Dict[str, ET.Element], bezier_steps: int) -> List[dict]:
    nodes = []
    for node_id in children_ids(ring):
        obj = objects.get(node_id)
        if obj is None:
            continue
        data = node_data(obj)
        if data is not None:
            nodes.append(data)
    if len(nodes) < 3:
        return []

    result: List[dict] = []
    for i, current in enumerate(nodes):
        nxt = nodes[(i + 1) % len(nodes)]
        curved = any(abs(current[k]) > 1e-12 for k in ("lat_hi", "lon_hi")) or \
                 any(abs(nxt[k]) > 1e-12 for k in ("lat_lo", "lon_lo"))
        steps = bezier_steps if curved else 1
        c1_lat = current["lat"] + current["lat_hi"]
        c1_lon = current["lon"] + current["lon_hi"]
        c2_lat = nxt["lat"] + nxt["lat_lo"]
        c2_lon = nxt["lon"] + nxt["lon_lo"]
        for step in range(steps):
            t = step / steps
            lat = cubic(current["lat"], c1_lat, c2_lat, nxt["lat"], t)
            lon = cubic(current["lon"], c1_lon, c2_lon, nxt["lon"], t)
            result.append({"latitude": lat, "longitude": lon})
    return result


def ring_area(points: List[dict]) -> float:
    if len(points) < 3:
        return 0.0
    mean_lat = sum(p["latitude"] for p in points) / len(points)
    cos_lat = math.cos(math.radians(mean_lat))
    total = 0.0
    for i, p in enumerate(points):
        q = points[(i + 1) % len(points)]
        x1, y1 = p["longitude"] * cos_lat, p["latitude"]
        x2, y2 = q["longitude"] * cos_lat, q["latitude"]
        total += x1 * y2 - x2 * y1
    return 0.5 * total


def hierarchy_name(obj: ET.Element, fallback: str) -> str:
    hierarchy = obj.find("hierarchy")
    if hierarchy is None:
        return fallback
    return hierarchy.attrib.get("name", fallback)


def extract(scenery_root: Path, bezier_steps: int = 8) -> dict:
    wed_path = scenery_root / "earth.wed.xml"
    if not wed_path.is_file():
        raise FileNotFoundError(f"WED project not found: {wed_path}")
    tree = ET.parse(wed_path)
    objects = {obj.attrib.get("id", ""): obj for obj in tree.findall(".//object") if obj.attrib.get("id")}

    areas = []
    selected_meta = None
    selected_resource = None
    ignored_resources = set()

    for obj in objects.values():
        if obj.attrib.get("class") != "WED_ForestPlacement":
            continue
        placement = obj.find("forest_placement")
        if placement is None or placement.attrib.get("closed", "Area") != "Area":
            continue
        resource = normalize_resource(placement.attrib.get("resource", ""))
        if not resource or resource.startswith("lib/"):
            continue
        for_path = scenery_root / Path(resource)
        meta = parse_for_metadata(for_path, scenery_root)
        if meta is None:
            continue

        if selected_resource is None:
            selected_resource = resource
            selected_meta = meta
        elif resource != selected_resource:
            ignored_resources.add(resource)
            continue

        rings = []
        for ring_id in children_ids(obj):
            ring_obj = objects.get(ring_id)
            if ring_obj is None or ring_obj.attrib.get("class") != "WED_ForestRing":
                continue
            points = sample_ring(ring_obj, objects, bezier_steps)
            if len(points) >= 3:
                rings.append(points)
        if not rings:
            continue

        # WED normally stores outer first. Picking the largest ring by area also
        # makes the exporter resilient to hand-edited/reordered XML.
        outer_index = max(range(len(rings)), key=lambda i: abs(ring_area(rings[i])))
        outer = rings[outer_index]
        holes = [ring for i, ring in enumerate(rings) if i != outer_index]
        density = float(placement.attrib.get("density", 1.0))
        density = min(1.0, max(0.03, density))
        object_id = obj.attrib.get("id", "unknown")
        areas.append({
            "id": f"wed_{object_id}",
            "name": hierarchy_name(obj, f"WED Forest {object_id}"),
            "density": density,
            "outer": outer,
            "holes": holes,
        })

    if selected_meta is None:
        raise RuntimeError(
            "No WED Forest placement using a local .for file with #OGR_VERSION metadata was found."
        )
    if ignored_resources:
        print(
            "WARNING: v0.1 supports one OGR .for resource per scenery pack. Ignored: " +
            ", ".join(sorted(ignored_resources)), file=sys.stderr
        )

    return {
        "format": FORMAT,
        "name": scenery_root.name,
        "source": "earth.wed.xml",
        "resource": selected_resource,
        "models": selected_meta["models"],
        "settings": selected_meta["settings"],
        "areas": areas,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Build OGR grass areas from WED earth.wed.xml")
    parser.add_argument("scenery_root", nargs="?", default=".", help="Scenery pack containing earth.wed.xml")
    parser.add_argument("--output", help="Output JSON path (default: OafishGrass/ogr_areas.json)")
    parser.add_argument("--bezier-steps", type=int, default=8, help="Samples per curved WED segment")
    args = parser.parse_args()

    scenery_root = Path(args.scenery_root).expanduser().resolve()
    output = Path(args.output).expanduser().resolve() if args.output else scenery_root / "OafishGrass" / "ogr_areas.json"
    try:
        data = extract(scenery_root, max(2, min(32, args.bezier_steps)))
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    except Exception as exc:
        print(f"OGR exporter ERROR: {exc}", file=sys.stderr)
        return 1

    print(f"OGR: wrote {len(data['areas'])} grass area(s) -> {output}")
    print(f"OGR: models: {', '.join(data['models'])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
