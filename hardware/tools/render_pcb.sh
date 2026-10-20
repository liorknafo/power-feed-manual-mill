#!/bin/zsh
# Render the board to PNGs for review: render_pcb.sh <outdir>
set -e
OUT=${1:-/tmp/pcb_render}
K=/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli
PCB=$(cd "$(dirname "$0")/../kicad" && pwd)/power-feed.kicad_pcb
mkdir -p "$OUT"; rm -rf "$OUT"/*(N)
$K pcb export svg --mode-single --fit-page-to-board --exclude-drawing-sheet -l "Edge.Cuts,F.Courtyard,F.Fab,F.SilkS" -o "$OUT/place.svg" "$PCB" >/dev/null 2>&1
$K pcb export svg --mode-single --fit-page-to-board --exclude-drawing-sheet -l "Edge.Cuts,F.Cu" -o "$OUT/fcu.svg" "$PCB" >/dev/null 2>&1
$K pcb export svg --mode-single --fit-page-to-board --exclude-drawing-sheet -l "Edge.Cuts,B.Cu" -o "$OUT/bcu.svg" "$PCB" >/dev/null 2>&1
cd "$OUT" && for f in *.svg; do qlmanage -t -s 1800 -o . "$f" >/dev/null 2>&1; done
ls "$OUT"
