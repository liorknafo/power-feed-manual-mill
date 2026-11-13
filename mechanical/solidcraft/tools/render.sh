#!/bin/sh
# Renders of the assemblies with SolidCraft's headless renderer, into mechanical/solidcraft/renders/.
# Run from anywhere: sh mechanical/solidcraft/tools/render.sh
# The feed unit's world is Y-up (Z runs along the leadscrew), so its cameras carry a roll that puts +Y up.
set -e
cd "$(dirname "$0")/.."
snap() { solidcraft-cli snapshot "$@" --width 1400 --height 1000 --no-grid --no-axes > /dev/null; }
ISO='{"target": [0.0, -60.0, 150.0], "yaw": -2.181522291184105, "pitch": 0.7334529191006443, "distance": 650.0, "roll": -2.009015703935208}'
SIDE='{"target": [0.0, -60.0, 130.0], "yaw": -1.5708963267945633, "pitch": 0.0, "distance": 620.0, "roll": -1.5707963267948966}'
BACK='{"target": [0.0, -60.0, 150.0], "yaw": 2.1112158270654806, "pitch": 0.6572724238356391, "distance": 650.0, "roll": 1.9221610963005697}'
TRAY='{"target": [0.0, -130.0, 180.0], "yaw": -2.3036114285814033, "pitch": 0.7283514008925277, "distance": 420.0, "roll": -2.1105362942852386}'
see_through="--body shell:opacity=0.15 --body hood:opacity=0.15 --body end_cap:opacity=0.15 --body door:opacity=0.15"
snap clutch.sc --out renders/clutch_iso.png --view iso
snap clutch.sc --out renders/clutch_section.png --view back --section XZ
snap feed_unit.sc --out renders/feed_unit_iso.png --camera "$ISO"
snap feed_unit.sc --out renders/feed_unit_back.png --camera "$BACK"
snap feed_unit.sc --out renders/feed_unit_inside.png --camera "$ISO" $see_through
snap feed_unit.sc --out renders/feed_unit_section.png --camera "$SIDE" --section YZ
snap parts/mains/mains_tray.json --out renders/mains_tray.png --camera "$TRAY"
echo "renders written to $(pwd)/renders"
