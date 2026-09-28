#!/usr/bin/env python3
import json
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "ogr_extract_wed.py"

FOR_TEXT = """A
800
FOREST
TEXTURE ogr_marker.png
NO_SHADOW
LOD 50
SCALE_X 1
SCALE_Y 1
SPACING 10000 10000
RANDOM 0 0
TREE 0 0 1 1 0.5 100 0.01 0.01 1 0 OGR_marker
#OGR_VERSION 1
#OGR_MODEL grass_A.obj
#OGR_MODEL grass_B.obj
#OGR_TILE_SIZE_M 2.4
#OGR_DRAW_DISTANCE_M 914.4
#OGR_BOUNDARY_MARGIN_M 1.4
#OGR_ENGINE_WASH 1
"""

WED_XML = """<?xml version="1.0" encoding="UTF-8"?>
<doc><objects>
<object class="WED_ForestPlacement" id="100" parent_id="1">
  <children><child id="101"/><child id="102"/></children>
  <hierarchy hidden="0" locked="0" name="OGR grass"/>
  <forest_placement closed="Area" density="0.8" resource="OafishGrass/ogr_grass.for"/>
</object>
<object class="WED_ForestRing" id="101" parent_id="100">
 <children><child id="110"/><child id="111"/><child id="112"/><child id="113"/></children>
 <hierarchy name="outer"/>
</object>
<object class="WED_ForestRing" id="102" parent_id="100">
 <children><child id="120"/><child id="121"/><child id="122"/><child id="123"/></children>
 <hierarchy name="hole"/>
</object>
<object class="WED_SimpleBoundaryNode" id="110" parent_id="101"><point latitude="-4.000" longitude="122.000"/></object>
<object class="WED_SimpleBoundaryNode" id="111" parent_id="101"><point latitude="-4.000" longitude="122.010"/></object>
<object class="WED_SimpleBoundaryNode" id="112" parent_id="101"><point latitude="-4.010" longitude="122.010"/></object>
<object class="WED_SimpleBoundaryNode" id="113" parent_id="101"><point latitude="-4.010" longitude="122.000"/></object>
<object class="WED_SimpleBoundaryNode" id="120" parent_id="102"><point latitude="-4.003" longitude="122.003"/></object>
<object class="WED_SimpleBoundaryNode" id="121" parent_id="102"><point latitude="-4.003" longitude="122.006"/></object>
<object class="WED_SimpleBoundaryNode" id="122" parent_id="102"><point latitude="-4.006" longitude="122.006"/></object>
<object class="WED_SimpleBoundaryNode" id="123" parent_id="102"><point latitude="-4.006" longitude="122.003"/></object>
</objects></doc>
"""

with tempfile.TemporaryDirectory(prefix="ogr-wed-test-") as td:
    scenery = Path(td) / "Test Airport"
    grass = scenery / "OafishGrass"
    grass.mkdir(parents=True)
    (scenery / "earth.wed.xml").write_text(WED_XML, encoding="utf-8")
    (grass / "ogr_grass.for").write_text(FOR_TEXT, encoding="utf-8")
    (grass / "grass_A.obj").write_text("A\n800\nOBJ\n", encoding="utf-8")
    out = grass / "ogr_areas.json"
    subprocess.run([sys.executable, str(SCRIPT), str(scenery)], check=True)
    data = json.loads(out.read_text(encoding="utf-8"))
    assert data["format"] == "OGR_AREA_V1"
    assert data["resource"] == "OafishGrass/ogr_grass.for"
    assert data["models"] == ["OafishGrass/grass_A.obj", "OafishGrass/grass_B.obj"]
    assert data["settings"]["draw_distance_m"] == 914.4
    assert len(data["areas"]) == 1
    assert abs(data["areas"][0]["density"] - 0.8) < 1e-6
    assert len(data["areas"][0]["outer"]) == 4
    assert len(data["areas"][0]["holes"]) == 1
print("OGR WED extractor test PASS")
