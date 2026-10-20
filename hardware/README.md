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
