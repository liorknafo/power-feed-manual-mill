# Hardware — controller PCB

```
hardware/
  tools/build_schematic.py   circuit description (source of truth for now)
  tools/kicad_gen.py         generator: Python parts/nets -> KiCad 9 schematic
  tools/check_schematic.py   ERC + netlist-vs-intent + footprint check
  tools/sexp.py              S-expression reader/writer
  kicad/                     generated KiCad project (open power-feed.kicad_pro)
```

## Regenerate and check

```sh
python3 hardware/tools/check_schematic.py    # builds, runs KiCad ERC, compares netlist
```

The check:
- runs KiCad's ERC and fails on any error not in the reviewed allow-list;
- exports the netlist and requires every net to match the intended connections exactly;
- lists parts whose footprint is missing.

## How the generated sheets look

Every pin gets a short wire stub and a net label; there are no long wires. Connectivity therefore comes only from net names, which keeps the generated schematic correct by construction. Labels used on more than one sheet are global labels; power nets use power symbols.

## Workflow

While the circuit is still changing, edit `build_schematic.py` and regenerate. Edits made in the KiCad GUI are overwritten on regeneration. Once the circuit is frozen for layout, stop regenerating, and from then on edit the schematic in KiCad directly.

Sheets: power input and 5 V buck · TMC5160 driver · Pico and E-stop interlock · clutch driver · panel I/O. See `docs/design.md` §4 for the pin map and §7.6 for the clutch driver.

## PCB

```sh
KPY=/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3
$KPY hardware/tools/build_pcb.py --route   # place, planes, Freerouting, power pours, GND stitching
$KPY hardware/tools/fix_planes.py          # vias to planes for anything the router left
$KPY hardware/tools/finish_pcb.py          # bridge pours for the last same-net gaps
$KPY hardware/tools/manual_fixes.py        # hand re-routes of the spots the scripts could not close (documented inside)
$KPY hardware/tools/silk_cleanup.py        # passives' refs to F.Fab, relocate the rest, connector labels + title
python3 hardware/tools/check_pcb.py        # KiCad DRC + schematic parity
python3 hardware/tools/export_fab.py       # Gerbers, drills, BOM, CPL, zip -> hardware/fab/
hardware/tools/render_pcb.sh               # 2D layer PNGs; 3D: kicad-cli pcb render
```

The saved `kicad/power-feed.kicad_pcb` is the master now: `build_pcb.py` rebuilds from scratch and throws away
the routing, so do not rerun it unless the schematic changes. `manual_fixes.py` and `silk_cleanup.py` are
idempotent and only touch the items they describe.

- **Board:** 100×100 mm, 4 layers: F.Cu signals + GND pour, In1 GND plane, In2 split +36V (power area) / +3V3, B.Cu signals + GND pour.
- **Autorouter:** Freerouting 2.4.1, run on Java 25. Set `FREEROUTING_JAR` to point at your copy.
- **Order** with 2 oz outer copper if possible (motor phases ~3.5 A RMS).
