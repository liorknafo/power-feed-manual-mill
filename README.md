<img src="assets/icon.svg" width="64" height="64" alt="">

# power-feed-manual-mill

A stepper-motor X-axis power feed for a **SIEG X3.5D** mill.

## Features
- **Speed:** a speed dial and an OLED showing mm/min.
- **Mode switch** (3 positions): LEFT, STOP, RIGHT.
- **Rapid traverse button.**
- **Clutch:** a home-made **electromagnetic tooth clutch**. With the switch on STOP the handwheel turns freely, as it does with no power.
- **Safety:**
  - a hardware E-stop interlock;
  - limit switches;
  - stall detection from an encoder on the leadscrew.

![PCB](hardware/renders/pcb_iso.png)

## What is where

| Folder | What | Start here |
|---|---|---|
| `docs/` | The spec, plus notes about the mill | `docs/design.md`, `docs/mill.md` |
| `hardware/` | KiCad 9 schematic + 4-layer 100×100 mm PCB, order files | `hardware/README.md`, `hardware/fab/README.md` |
| `firmware/` | Rust firmware for the RP2040 (Embassy). Core logic is unit-tested on the PC | `firmware/README.md` |
| `mechanical/clutch/` | Tooth clutch: parametric CAD + machining sheet | `mechanical/clutch/MACHINING.md` |
| `mechanical/feed_unit/` | The whole power feed in one box at the left table end: plate, 2:1 belt, leadscrew stub, encoder, clutch module, motor, controller PCB and control panel | `mechanical/feed_unit/FEED_UNIT.md` |
| `mechanical/viewer/` | Interactive 3D view of everything (`sh mechanical/viewer/build.sh`) | `mechanical/README.md` |

## Status

| Part | State |
|---|---|
| Schematic | Done. KiCad ERC clean, netlist checked, independently reviewed |
| PCB | Routed. DRC 0 errors / 0 warnings with schematic parity. Gerbers, BOM, CPL in `hardware/fab/` |
| Firmware | Builds for the RP2040. Core state machine has 47 passing tests. Needs bench commissioning (`firmware/README.md`) |
| Clutch CAD | Ready to machine, after you measure the motor shaft length. Independently reviewed (interference checked engaged and released). Magnetic sizing must be bench-calibrated |
| Feed unit CAD | Structure reviewed: Oldham drive, circlip bearing stack, encoder adapter, guard, drawbolt tensioner. **Mill-side dimensions are placeholders until measured** |
| Housing (one box: drive + PCB + controls) | Modelled: aluminium panels and bars on the feed-unit plate, PCB on the partition, controls on the front panel. Interference-checked; see FEED_UNIT.md → "Housing" |

## What you need to do

1. **Measure the mill** (`docs/mill.md` → "Unknown"):
   - leadscrew pitch;
   - the left leadscrew end and its drive slot;
   - where the plate can bolt to the table end;
   - clearances at full travel.

   Enter the values in `mechanical/feed_unit/feed_unit.py` (class `F`) and re-run it.
2. **Measure the motor:** shaft length beyond the flange and the pilot diameter. Set them in `mechanical/clutch/clutch.py` (class `P`).
3. **Order the PCB:**
   - upload `hardware/fab/power-feed-gerbers.zip` to JLCPCB: 4 layers, 1.6 mm, 2 oz outer copper if possible;
   - stencil plus the BOM/CPL for assembly, or reflow it yourself;
   - everything is SMD on the top side, including the RP2040 (QFN-56, 0.4 mm pitch: use the stencil).
4. **Make the clutch** (`MACHINING.md`). Then calibrate it on the bench: pull force and cam-out torque against hold current.
5. **Flash and commission:** follow `firmware/README.md` → *Commissioning*, in order, before mounting on the mill.

## Rebuilding things
```sh
python3 hardware/tools/check_schematic.py                 # schematic: ERC + netlist check
python3 hardware/tools/check_pcb.py                       # PCB: DRC + schematic parity
(cd firmware/feed-core && cargo test)                     # logic tests
(cd firmware/feed-app && cargo build --release)           # firmware
mechanical/.venv/bin/python mechanical/clutch/clutch.py   # clutch CAD (+ machining sheet)
mechanical/.venv/bin/python mechanical/feed_unit/feed_unit.py
mechanical/.venv/bin/python mechanical/tools/check_clutch.py      # interference, engaged + released
mechanical/.venv/bin/python mechanical/tools/check_feed_unit.py   # interference, incl. bolt heads over the tension slot
```

Create the Python environment for the CAD scripts first:

```sh
python3 -m venv mechanical/.venv
mechanical/.venv/bin/pip install build123d trimesh pyvista shapely networkx
```
