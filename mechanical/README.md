# Mechanical models (build123d)

All models are parametric Python (a dataclass `P` of parameters, one function per part). Run them with the
bundled venv; each writes STEP + STL per part and shaded renders into its own `out/` folder
(`out/` is git-ignored except `render_*.png`).

```
mechanical/.venv/bin/python mechanical/enclosure/enclosure.py          # controller enclosure (PETG)
mechanical/.venv/bin/python mechanical/enclosure/enclosure.py --alu    # 3 mm aluminium variant (*_alu.step/stl, no renders)
mechanical/.venv/bin/python mechanical/panel/panel.py                  # front panel / pendant box
```

Renders use `mechanical/tools/render.py`. Each script also runs an interference check of the shell parts against
the placeholder bodies (PCB parts / controls / ribbon) and prints `interference check: clean`.

`clutch/` is a separate model, not described here.

---

## 1. Controller enclosure (`enclosure/enclosure.py`)

Frame: X = PCB x, Y = -PCB y (the connector edge, PCB y = 100, faces -Y), Z up, floor underside at Z = 0, PCB
top-left corner at (0, 0). `W(x, y, z)` converts PCB coordinates from the spec.

Parts: `base`, `lid`, `fuse_hatch` (+ `pcb_placeholder`, render only).

| Item | Value |
|---|---|
| Outer size (PETG) | 117.6 x 107 x 41.1 mm (lid on); 147.6 wide with the ears |
| Wall / floor | 2.5 mm (`wall`); `P.alu()` = 3.0 mm |
| PCB position | 1 mm gap to the top and connector walls (`pcb_clear`), 6.3 mm to the left / right walls (`side_clear`, room for the lid bosses), PCB underside 6 mm above the floor top (z 8.5..10.1) |
| Inside height | 35.6 mm = 6 + 1.6 + 25 + 3 (`cap_clear`); the caps top out at z 35.1, lid underside at 38.1 |
| Standoffs | 4 x Ø7 at the PCB holes, Ø2.6 x 8 pilot (M3 self-tapping, PETG) |
| Lid | 3 mm plate, 3 mm inner lip (0.25 mm clearance), 4 x M3 counterbored, vents, fuse opening |
| Lid screws | 4 hanging bosses on the left / right walls (Ø7, 9 mm tall, 45 degree cone underneath so they print without support), Ø2.6 x 8 pilot. They sit entirely outside the 100 x 100 board outline (0.8 mm from the PCB edge, 8.4 mm from the nearest PCB screw axis), so the board drops straight in and its screws stay reachable |
| Vents | 8 slots 2.2 x 31 mm, 4.5 mm pitch, over x 25..60, y 50..85 |
| Mounting ears | 2 x (16 x 20 x 3 mm) on the left / right wall at PCB y = 50, Ø5.5 x 9.5 slot (M5) |

Connector openings (the +y wall):
- **J1..J3**: with the requested 1 mm clearance the three cut-outs overlap (gap between bodies is only 1.2 mm), so
  they are one window, x 11.0..65.2, z 9.1..23.1 (connector top + 1 mm).
- **J5 and J4**: slot x 65.0..89.45 (J5 8 mm wide centred 69.0, J4 22 mm wide centred 78.45; they merge), open from
  z 9.1 (PCB top - 1) to the top of the wall, so the cables come out upward and over the wall edge.
- The wall above the J1..J3 window is part of the **lid**: a skirt (x 11.2..65.2, z 23.1 up) closes it. This avoids
  a 54 mm wall bridge that is only held at one end, and it prints with no supports.
- **Micro-USB**: 14 x 9.5 mm hole in the +x wall at PCB y = 15, centred 2.4 mm above the PCB top. Because the
  side margin is 6.3 mm, a 5.3 mm thick pad (PCB y 7.5..27) brings the wall inner face back to the 1 mm PCB gap
  around the USB hole, so the plug reaches the connector as before.
- **Fuse holder** (x 0..11, y 59..87): a 10 x 26 mm opening in the lid above it plus a separate `fuse_hatch` plate
  (rebate 1.2 mm, fixed by 2 x M3x8 into two small bosses under the lid). Pull the fuse out through the opening.

**PCB insertion check**: `insertion_sweep()` (run with the script) lifts the PCB placeholder with all its parts
straight up from its seat in 1 mm steps to 45 mm and intersects it with the base at each of the 46 positions: result
0.0000 mm3 everywhere (CLEAR). Check it by hand if you change `boss_gap`, `boss_r`, or the PCB footprint.

Print notes (PETG): base floor-down; lid upside down (plate on the bed, skirt/lip/bosses up); hatch flat.
0.2 mm layers, 3+ perimeters, 20 % infill. No supports needed. Heat-set inserts (M3, Ø4.0 hole) can replace the
self-tapping pilots if you prefer: change `pilot_d` / `boss_hole`.

**Aluminium variant** (`P.alu()`): 3 mm walls and floor, 6 mm outer / 3 mm inner corner radius (6 mm end mill),
M3 tap drill 2.5 mm, no printed support cones. The geometry is made to be machined from block or plate; if you
fold or weld it from 3 mm sheet, only the overall dimensions and the cut-out positions carry over.

Hardware: 4 x M3x10 pan head (PCB into standoffs), 4 x M3x8 pan head (lid), 2 x M3x8 pan head (hatch),
2 x M5 for the ears, PETG about 100 g.

---

## 2. Front panel / pendant box (`panel/panel.py`)

Frame: origin at the box centre, X right and Y up as seen from the front, Z = depth; shell rim at Z = 0, front
face at Z = 60 (`depth`); the back cover is Z -4..0.

Parts: `shell` (front plate + walls), `back_cover` (+ `hardware_placeholder`, render only).

| Item | Value |
|---|---|
| Box | 140 x 100 mm, 60 mm deep (`depth`, +4 mm cover), 3 mm walls and front plate, cavity 57 mm (E-stop body 45 mm, 12 mm spare for its rear terminals and wiring). **Measure your E-stop** (body depth incl. contact blocks) and set `depth` to suit |
| E-stop | Ø22.5 hole at (44, 22), right column; head Ø40 9 mm clear of the dial knob |
| OLED | window 30 x 16 at (0, 30.5); module centre (0, 28); 4 posts Ø5 x 2.5 mm at 30.4 x 28.5, M2 self-tapping pilots Ø1.7 |
| Dial (EC11) | Ø7.4 hole at (0, -1) + Ø3.2 anti-rotation hole 7.8 mm above it; Ø30 knob. Panel 3 mm + M7 nut 2 mm + 0.5 mm spare = 5.5 mm, so it needs a bushing of at least 5.5 mm (typical EC11 6 mm): `enc_bushing_len`. If your encoder has a shorter bushing or you thicken the panel, the script thins the panel from behind (Ø11 pocket) automatically |
| Toggle | Ø12.4 hole at (-44, -33); mount it with the lever swinging left-right (LEFT / STOP / RIGHT labels above it) |
| Rapid button | Ø16.3 hole at (0, -34) |
| Cable entry | 20 x 4 mm slot in the bottom wall at x = -22, z = 21; a zip-tie bridge inside it (two 1.8 x 4.2 slots at x -22 +-11.5, for 2.5-3.6 mm ties) is the strain relief; its face toward the print bed is a 45 degree chamfer, so it prints without supports |
| Back cover | 4 x M3 clearance with Ø6.2 x 2 counterbore, 2 mm locating lip, 4 x Ø12.3 x 3.2 magnet pockets open to the inside (0.8 mm skin outside), 4 adapter-board posts |
| Adapter board | 45 x 28 mm placeholder with the 2x7 IDC header, 4 posts 5 mm tall at 39 x 22 mm (M3 self-tapping) |

Engraved labels (0.6 mm deep, 4 mm Arial): SPEED, RAPID, LEFT, STOP, RIGHT, E-STOP. They are cut with build123d
`Text` and the font worked here; if it fails on your machine the script prints a warning and skips that label, and
the label list is `P.labels` (position of each text on the front face).

Press-fit holes are sized for printing (nominal + 0.2..0.5 mm: 22.5 / 12.4 / 16.3 / 7.4 / 3.2); the Ø12.3 magnet
pockets and M2/M3 pilots likewise. Test-fit one hole before printing the whole shell and adjust the `*_hole` values.

Print notes (PETG): shell front face on the bed (the engraving then prints as a clean top surface, so use a 0.12
mm layer height or an ironing pass for the text); cover flat, inside face up. Magnets: glue in before closing the
cover, polarity all the same way. No supports.

Hardware: 22 mm mushroom E-stop (NC, 40 mm head), ON-OFF-ON 12 mm toggle, 16 mm momentary push-button, EC11
encoder with push + Ø30 knob, 1.3" SH1106 OLED, 4 x M3x10 pan head (cover), 4 x M2x6 (OLED), 4 x M3x8 (adapter
board), 4 x Ø12x3 N35 magnets, 2 zip ties, 14-way ribbon with IDC plug, small adapter board with a 2x7 IDC header.

---

## Assumptions to verify before printing
- Heights that were not given are guesses and only matter for the placeholder bodies in the renders: J5 8.5 mm,
  J4 10 mm, fuse holder 13 mm, J1..J3 depth 11 mm. The enclosure openings only use the given 12 mm for J1..J3.
- Pico USB centre line is assumed 2.4 mm above the PCB top (a 14 x 9.5 mm hole covers a typical overmoulded plug).
- OLED post height 2.5 mm assumes the glass stands about 1.5-2 mm proud of the module PCB: check your module.
- E-stop plate/legend ring (typically Ø60 yellow) is not modelled; it would still fit in the right column.
- Panel strain relief clamps by friction (zip tie around the ribbon); it is not a mechanical stop.


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
- the output encoder on an adapter disc and three posts, coupled to the stub's outer end.

`feed_unit.py` re-uses the clutch model, so the module always matches `clutch.py`. Interference checks:
`mechanical/tools/check_clutch.py` (engaged and released) and `mechanical/tools/check_feed_unit.py`; re-run them after
changing `P` or `F`.

**Run:** `mechanical/.venv/bin/python mechanical/feed_unit/feed_unit.py`. It writes STEP/STL, renders and `FEED_UNIT.md`.

**The mill-side dimensions are PLACEHOLDERS:** the leadscrew-end slot, the standoff positions on the table end, and the motor angle. See `FEED_UNIT.md` and docs/mill.md.

## Interactive 3D view
`sh mechanical/viewer/build.sh` exports the PCB to `mechanical/viewer/out/pcb.glb` and serves the repo; open
http://localhost:8765/mechanical/viewer/ to orbit the feed unit on the mill, the exploded clutch, the controller box
with the routed PCB inside, and the panel. Run the CAD scripts first (it loads their `out/*.stl`).

## Renders
`mechanical/tools/render.py`: VTK off-screen shaded renders of STL files. Options:
- `--cut` with `--cut-normal x,y,z` for half sections;
- `--up` to set the camera's up vector;
- `part.stl:color:dz` for exploded views.
