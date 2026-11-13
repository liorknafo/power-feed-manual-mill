# Mechanical CAD in SolidCraft

The whole power feed, modelled in [SolidCraft](https://solidcraft.io) (0.40): every part its own parametric design,
two assemblies that link them. Built through SolidCraft's MCP server, part by part, from the build123d model this
replaced (git history before the migration) and checked against it (see "Ported from build123d" below).

```
parts/clutch/*.json      the clutch's made parts (clutch frame)
parts/feed_unit/*.json   plate, drive, encoder disc, PCB stand-in, ... (feed-unit world)
parts/housing/*.json     the printed housing: shell, hood, door, end cap (feed-unit world)
parts/mains/*.json       the folded aluminium mains tray and its lid (sheet metal)
parts/bought/*.json      bought parts, each from its listing's drawing (own frame)
clutch.sc                assembly: the clutch on its motor
feed_unit.sc             assembly: the whole unit on the table end, the clutch as a nested `clutch_module`
clutch.build.json        the commands that build clutch.sc from the parts (solidcraft-cli run)
feed_unit.build.json     the same for feed_unit.sc
flat/                    the tray's and lid's flat patterns (DXF) for the CNC, exported by SolidCraft
renders/                 pictures (tools/render.sh)
tools/check.py           assembly checks (interference, mains separation, door swing, print-bed fit)
tools/render.sh          the renders
tools/scmcp.py           a small client that keeps a headless SolidCraft MCP session behind a socket
```

Parts are plain-JSON designs (`.json`): they open in the app like `.sc` files and diff in git. The assemblies are
`.sc` containers (they embed a copy of each linked part, so they are binary and large).

## Working on it
- **Edit a part:** open `parts/<group>/<name>.json` in SolidCraft, change its parameters (Parameters panel; each one's
  comment says what it is and its range) or features, save. Then open the assemblies and update the links
  (`component.update_links`, or right-click the linked component), save them, and run the checks.
- **Rebuild an assembly from scratch** (after renaming or adding parts, or to get back to a clean state):
  ```sh
  cd mechanical/solidcraft
  solidcraft-cli run clutch.build.json --save clutch.sc
  solidcraft-cli run feed_unit.build.json --save feed_unit.sc
  ```
  The scripts place every part, insert every library fastener and make the joints. Placements are plain numbers
  (from the build123d model): a part that moves because of a parameter change needs its placement edited here too.
- **Check:** `python3 mechanical/solidcraft/tools/check.py` (about 30 s). It fails on any overlap that is not an intended
  fit listed in `ALLOWED`, and on anything inside the earthed mains tray but the PSU, the inlet and the tray's own
  screws.
- **Renders:** `sh mechanical/solidcraft/tools/render.sh`.
- **Export** a part for the shop or the slicer: File > Export in the app, or
  `solidcraft-cli run --in parts/clutch/cover.json --out cover.step` (`.step`, `.stl`, `.3mf`).
- **Flat patterns** of the tray and lid: open the part, `sheet.flat_pattern` / Sheet Metal > Export DXF, and copy the
  DXF into `flat/`.
- **With an AI agent:** `claude mcp add solidcraft -- solidcraft-cli mcp --allow-path "$PWD/mechanical/solidcraft"`
  gives Claude Code SolidCraft's tools. Without that, `tools/scmcp.py` drives the same server from a shell:
  `python3 tools/scmcp.py serve --sock /tmp/sc.sock &`, then `python3 tools/scmcp.py call --sock /tmp/sc.sock batch @steps.json`.

## Frames
- **clutch frame** (`parts/clutch`, `clutch.sc`): axis Z, the motor flange face at z = 0, output toward +z, modelled
  ENGAGED. Each clutch part is modelled where it sits.
- **feed-unit world** (`parts/feed_unit`, `parts/housing`, `parts/mains`, `feed_unit.sc`): Z (= W) along the leadscrew,
  out of the table; Y up; X toward the operator; z = 0 at the face of the table's leadscrew bracket. The clutch module
  goes in flipped: `clutch_module:1` sits at (0, MY, W0) = (0, −70.71, 125.85), turned 180° about X.
- **own frame** (`parts/bought`, and the repeated small parts: pins, bushes, springs, clips, circlips, standoffs,
  inserts, rivnuts, washers): described in each design's parameters and placed by the assembly.

## Measure first (the PLACEHOLDERS)
The mill-side and motor values aren't known yet (docs/mill.md "Unknown"). Each lives in the part files that use it,
as a parameter of the same name: change it in every file listed, update the links, run the checks.

| Value | Parameter | In |
|---|---|---|
| leadscrew Ø, its end slot width and depth, end face position | `screw_d`, `screw_end_slot_w`, `screw_end_slot_d`, `screw_end_z` | `feed_unit/oldham_disc`, `feed_unit/stub_shaft`, `feed_unit/mill_reference` |
| bracket size (reference only) | `bracket_w`, `bracket_h` | `feed_unit/mill_reference` |
| where the standoffs land on the table end | `standoff_x`, `standoff_y_hi`, `standoff_y_lo` | `feed_unit/plate`, and the standoffs' and M6 screws' placements in `feed_unit.build.json` |
| motor shaft length beyond the flange | `shaft_len` | `clutch/rotor` (bore depth), `bought/motor_23hs45` |
| motor pilot Ø | `pilot_d` | `clutch/field_housing`, `bought/motor_23hs45` |

Values that run through many parts (the belt-chamber depth `chamber`, the plate thickness `plate_t`, the housing's
`cap_cut`, ...) are parameters in each part that uses them, with the same name; `grep -l '"name": "chamber"' parts/*/*.json`
lists them.

## The assemblies
- **clutch.sc**: the stationary block (motor, field housing, coil, cover, bearings, spacer tube, circlips, motor and
  cover screws) is one rigid group; `rotor_turn` and `output_turn` turn the rotor and the output (hub, shaft, bushes,
  springs, spacer, circlip, set screws) about Z; `armature_stroke` slides the armature with its pins, washers and
  E-clips along the hub's pins, 0 (engaged) to 1.1 mm (released, against the hub). Drive it to see the release.
  The springs are modelled at their engaged length and don't stretch.
- **feed_unit.sc**: everything that doesn't move is one rigid group with the table end (`mill_reference`, grounded),
  the clutch module included. `door_hinge` swings the bottom door down (negative angles; take the end cap off first,
  its scarf holds the door's far end). `pulley_20T_turn` turns the 20T pulley; a belt link (ratio 0.5, same
  direction) turns `leadscrew_stub` (stub shaft, 40T pulley, Oldham disc).
- **Fasteners, bearings:** SolidCraft's standard library (ISO 4762, 7380, 10642, 4026, 4032, 7089; 6001, 6002), one
  component per size, with modelled threads. The made parts keep their tap-drill holes, so a screw's thread overlaps
  its tapped hole: the checks allow exactly those pairs. Circlips, E-clips, heat-set inserts, rivnuts and the special
  washers are their own part files.
- Materials are assigned in each part, so Properties gives masses (the coil is one ring at a winding density of
  3.67 g/cm³, ≈ 85 g as wound).

## Checks
`tools/check.py` covers:
- every pair of bodies in both assemblies, the clutch engaged and released;
- mains separation: nothing but the PSU, the inlet and plug and the tray's own screws inside the tray, the 36 mm
  wiring room between the PSU terminals and the inlet body empty;
- the door's swing on its hinge, and the hinge pin's clearance in the door's knuckles at full swing;
- the four housing prints on the Prusa MK3S bed in their print orientations.

The build123d checks also covered these, which SolidCraft does not check yet (do them by eye in the app, or
with `inspect.clearance` / `joint.sweep` / `inspect.section`, after a change in that area):
print overhangs and islands starting in mid-air, material round each heat-set insert, the module bolt heads over the
whole tension slot, panel parts turned through their play, slide-on paths (shell, end cap) and loading the module and
tray from below, tool access to the PCB screws, plug/wire and USB-C path envelopes (the envelopes are in
`parts/feed_unit/clearance_envelopes.json` and in the assembly, so the interference check does see anything that
enters them), the hinge pin's insertion path, the tray's inlet margins and the lid's screw lengths, and sharp internal
corners a cutter can't make.

## Ported from build123d
Each part was rebuilt natively (sketches, extrudes, revolves, lofts, coils, sheet metal, holes, patterns) and
compared with the build123d part exported as STEP: xor (the volume of the difference) 0 for nearly all. The
deliberate differences:
- clutch teeth: true r 32 / r 38 arcs where the build123d sections were 4-segment polylines (0.94 mm³ per part; the
  flanks are identical);
- belt guard and belt band: true arcs and tangents instead of 48/144-sided hulls (+68 mm³, +141 mm³);
- plate: the tension slots as true slots instead of a row of circles 0.5 mm apart (5 mm³);
- motor: the pilot is a ring round the shaft (the two bodies no longer overlap);
- hood and end cap labels: DejaVu Sans without kerning (letters shift up to 0.09 mm);
- mains tray and lid: real 1.5 mm bends (the build123d model had sharp corners); same flat sizes, holes and reliefs.
