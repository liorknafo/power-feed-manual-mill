#!/bin/sh
# Builds the 3D viewer's PCB model and serves the viewer.
#   sh mechanical/viewer/build.sh          -> then open http://localhost:8765/mechanical/viewer/
# The STL parts come from the CAD scripts' out/ folders (run clutch.py and feed_unit.py first).
set -e
cd "$(dirname "$0")/../.."
KC=${KICAD_CLI:-/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli}
mkdir -p mechanical/viewer/out
"$KC" pcb export glb -f --subst-models --include-silkscreen --include-soldermask \
    -o mechanical/viewer/out/pcb.glb hardware/kicad/power-feed.kicad_pcb
echo "serving on http://localhost:8765/mechanical/viewer/"
exec python3 -m http.server 8765 --bind 127.0.0.1
