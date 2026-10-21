# Mechanical models (build123d)

All models are parametric Python (a dataclass `P` of parameters, one function per part). Run them with the
bundled venv; each writes STEP + STL per part and shaded renders into its own `out/` folder
(`out/` is git-ignored except `render_*.png`). The venv needs build123d 0.13 and bd_warehouse 0.4
(`mechanical/.venv/bin/pip install build123d==0.13.0 bd_warehouse==0.4.0`): every screw, nut, washer, insert and
tapped-hole thread is an ISO part from bd_warehouse (`mechanical/fasteners.py`), placed in the models with real threads
for the viewer and as plain shanks for the checks.

```
mechanical/.venv/bin/python mechanical/clutch/clutch.py        # tooth clutch (+ MACHINING.md)
mechanical/.venv/bin/python mechanical/feed_unit/feed_unit.py  # the whole power feed: drive, housing, PCB, controls (+ FEED_UNIT.md)
```

There is ONE box: the motor, clutch, belt drive, encoder, controller PCB and the controls all live in the feed-unit
housing on the table end (the separate controller box and pendant were retired; see git history before 2026-10-20).

## Clutch (`mechanical/clutch/clutch.py`)

The DIY electromagnetic tooth clutch, modelled engaged (docs/design.md §7).
- **Run:** `mechanical/.venv/bin/python mechanical/clutch/clutch.py`.
- **Output:** STEP/STL for each part, renders, and `MACHINING.md`, the dimension and fit tables for the lathe and CNC, generated from the same parameters.
- **Measure first:** set `shaft_len` to your motor's actual shaft length, and check the pilot diameter.

## X feed unit (`mechanical/feed_unit/feed_unit.py`)

The mount for the left table end of the SIEG X3.5D:
- standoffs from the table end;
- a belt chamber closed by a printed guard cup;
- a 28 mm aluminium plate holding a leadscrew stub on 2× 6002 bearings (through bore, circlips, spacer tube);
- a steel or bronze Oldham disc between the stub and the leadscrew slot, so the plate needn't sit exactly on the leadscrew axis;
- HTD-5M 20T/40T pulleys with a 295 mm belt (2:1, centre distance 70.71 mm);
- the clutch module bolted to the plate through its cover's front wall, slotted for belt tension, with a drawbolt tension block;
- the output encoder on an adapter disc and three posts, coupled to the stub's outer end;
- **the housing around all of it, compact and rounded, 3D printed (PETG, fits a Prusa MK3S):** the PCB lies flat
  above the motor; a shell print (walls + PCB rails, heat-set inserts) screws onto the plate edges, and a hood print
  (top + control strip: E-stop, LEFT/STOP/RIGHT, RAPID, OLED, SPEED dial) lifts off to expose the PCB.
  USB-C and the cable glands are in the end wall. Details in `FEED_UNIT.md` → "Housing".

`feed_unit.py` re-uses the clutch model, so the module always matches `clutch.py`. Interference checks:
`mechanical/tools/check_clutch.py` (engaged and released) and `mechanical/tools/check_feed_unit.py`; re-run them after
changing `P` or `F`.

**Run:** `mechanical/.venv/bin/python mechanical/feed_unit/feed_unit.py`. It writes STEP/STL, renders and `FEED_UNIT.md`.

**The mill-side dimensions are PLACEHOLDERS:** the leadscrew-end slot, the standoff positions on the table end, and the motor angle. See `FEED_UNIT.md` and docs/mill.md.

## Interactive 3D view
`sh mechanical/viewer/build.sh` exports the PCB to `mechanical/viewer/out/pcb.glb` and serves the repo; open
http://localhost:8765/mechanical/viewer/ to orbit the power feed on the mill (with the routed PCB inside) and the
exploded clutch. **Click any part** to see what it is, what it's made of and what it does; on the PCB, click a
component to get its reference, value, circuit, MPN and LCSC number (`pcb_parts.py` builds that list from the board
and the schematic). Run the CAD scripts first (it loads their `out/*.stl`).

## Renders
`mechanical/tools/render.py`: VTK off-screen shaded renders of STL files. Options:
- `--cut` with `--cut-normal x,y,z` for half sections;
- `--up` to set the camera's up vector;
- `part.stl:color:dz` for exploded views.
