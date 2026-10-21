#!/bin/sh
# Serves the 3D viewer from the repo root (the viewer loads ../feed_unit/out, ../clutch/out), from any directory.
#   sh mechanical/viewer/serve.sh                  -> http://localhost:8765/mechanical/viewer/
#   sh mechanical/viewer/serve.sh 100.x.y.z    -> on that address (e.g. your Tailscale IP)
# Rebuild the PCB model and component list first with build.sh if the board changed.
cd "$(dirname "$0")/../.."
ADDR=${1:-127.0.0.1}
echo "serving on http://$ADDR:8765/mechanical/viewer/"
exec python3 -m http.server 8765 --bind "$ADDR"
