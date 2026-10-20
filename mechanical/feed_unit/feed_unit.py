"""X-axis feed unit for the SIEG X3.5D: mount plate, 2:1 belt drive, leadscrew stub, encoder.

Run:  mechanical/.venv/bin/python mechanical/feed_unit/feed_unit.py
Out:  mechanical/feed_unit/out/*.step|*.stl, renders; FEED_UNIT.md (parts, placeholders).

Layout (docs/mill.md: the feed goes on the LEFT table end, where SIEG's own SX3.5P feeder mounts):
  World Z = leadscrew axis, pointing OUT of the table (away from the mill). Z = 0 is the face of the
  table's left leadscrew bracket. Y = up. The motor/clutch module sits below the leadscrew.

  table end | belt chamber (standoffs, guard) | 20 mm plate | clutch module | motor
            |  40T pulley on the stub   20T on the clutch output   |
  The leadscrew stub runs in 2x 6002 in the plate; its tongue drives the leadscrew end; the output
  encoder sits on the stub's outer end (after the belt, so it also sees belt slip).

Every value marked PLACEHOLDER must be checked on the machine before cutting metal.
"""

import math
import os
import sys
from dataclasses import dataclass

from build123d import (Align, Box, Compound, Cylinder, Polyline, Pos, Rot, export_step, export_stl, extrude,
                       make_face)

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "clutch"))
import clutch  # noqa: E402

OUT = os.path.join(HERE, "out")
MIN = (Align.CENTER, Align.CENTER, Align.MIN)


@dataclass
class F:
    # --- mill side: ALL PLACEHOLDERS until measured (docs/mill.md "Unknown")
    screw_d: float = 16.0            # PLACEHOLDER leadscrew OD
    screw_end_slot_w: float = 4.0    # PLACEHOLDER drive slot across the leadscrew end
    screw_end_slot_d: float = 5.0    # PLACEHOLDER slot depth
    screw_end_z: float = -1.0        # PLACEHOLDER leadscrew end face relative to the bracket face
    bracket_w: float = 70.0          # PLACEHOLDER left bracket width (for the reference model)
    bracket_h: float = 60.0
    # where the standoffs land on the table end (x, y) relative to the leadscrew axis; PLACEHOLDER:
    # drill and tap M6 there, or use the bracket's own screws if they line up
    standoffs: tuple = ((-46.0, 12.0), (46.0, 12.0), (-46.0, -60.0), (46.0, -60.0))
    # --- drive
    pitch_belt: float = 5.0          # HTD 5M
    teeth_small: int = 20
    teeth_big: int = 40
    belt_len: float = 295.0          # 295-5M-15 (59 teeth)
    belt_w: float = 15.0
    pulley_w: float = 21.0
    motor_angle_deg: float = -90.0   # module below the leadscrew; rotate if it hits the saddle or base
    # --- structure
    plate_t: float = 20.0
    chamber: float = 36.0            # bracket face -> plate (belt plane in between)
    standoff_d: float = 14.0
    stub_d: float = 15.0             # at the 6002 bearings (15x32x9)
    stub_enc_d: float = 8.0
    bearing_od: float = 32.0
    bearing_w: float = 9.0
    tension_slot: float = 3.0        # +- adjustment of the module along the centre line
    encoder_standoff: float = 28.0


f = F()


def pd(teeth):
    return teeth * f.pitch_belt / math.pi


def centre_distance():
    """Solve the belt-length equation for C."""
    d1, d2 = pd(f.teeth_small), pd(f.teeth_big)
    c = 70.0
    for _ in range(60):
        L = 2 * c + math.pi * (d1 + d2) / 2 + (d2 - d1) ** 2 / (4 * c)
        dL = 2 - (d2 - d1) ** 2 / (4 * c * c)
        c -= (L - f.belt_len) / dL
    return c


C = centre_distance()
ANG = math.radians(f.motor_angle_deg)
MX, MY = C * math.cos(ANG), C * math.sin(ANG)       # motor/clutch axis in the XY plane
Z_PLATE_IN, Z_PLATE_OUT = f.chamber, f.chamber + f.plate_t


def ring(r_out, r_in, h, z0=0.0):
    s = Cylinder(r_out, h, align=MIN)
    if r_in > 0:
        s = s - Cylinder(r_in, h, align=MIN)
    return Pos(0, 0, z0) * s


def hull(points):
    pts = sorted(set(points))
    if len(pts) < 3:
        return pts

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for p_ in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], p_) <= 0:
            lower.pop()
        lower.append(p_)
    for p_ in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], p_) <= 0:
            upper.pop()
        upper.append(p_)
    return lower[:-1] + upper[:-1]


def stadium(r1, r2, z0, h, n=48):
    """Solid hull of a circle r1 at the leadscrew and r2 at the motor axis."""
    pts = [(round(r1 * math.cos(2 * math.pi * k / n), 4), round(r1 * math.sin(2 * math.pi * k / n), 4)) for k in range(n)]
    pts += [(round(MX + r2 * math.cos(2 * math.pi * k / n), 4), round(MY + r2 * math.sin(2 * math.pi * k / n), 4)) for k in range(n)]
    h_ = hull(pts)
    face = make_face(Polyline(*[(x, y, z0) for x, y in h_], close=True))
    return extrude(face, amount=h)


# ------------------------------------------------------------------ module placement
# clutch local Z (motor face = 0, output toward +z) maps to world W = W0 - z (output points at the table)
CP = clutch.p
Z_FRONT = clutch.Z_HUB + CP.hub_flange_t + CP.hub_boss_len + 3.0
Z_FRONT_OUT = Z_FRONT + CP.cover_front_t
BOSS_LEN = 2 * CP.bearing_w + CP.bearing_gap + 4
W0 = Z_PLATE_OUT + Z_FRONT_OUT
W_BOSS_END = W0 - (Z_FRONT_OUT + BOSS_LEN)
W_SHAFT_END = W0 - (clutch.Z_HUB + 1.0 + CP.out_shaft_len)
W_PULLEY = (W_BOSS_END + W_SHAFT_END) / 2           # small pulley centre plane
assert W_SHAFT_END > 1.0, "clutch output shaft would hit the table end: increase chamber"
assert W_BOSS_END > W_PULLEY + f.pulley_w / 2 - 0.1, "pulley overlaps the bearing boss"


def place_module(shape):
    """Clutch-local shape -> world: flip so the output points at the table, then move to the motor axis."""
    return Pos(MX, MY, W0) * Rot(180, 0, 0) * shape


# ------------------------------------------------------------------ parts
def plate():
    w_side = 55.0
    pts_top = max(y for _, y in f.standoffs) + 14
    pts_bot = MY - (CP.housing_od / 2 + CP.cover_wall) - 8
    s = Pos(0, (pts_top + pts_bot) / 2, Z_PLATE_IN) * Box(2 * w_side, pts_top - pts_bot, f.plate_t, align=MIN)
    # stub bearings: 2x 6002 seats from both faces, shaft clearance between
    s = s - Pos(0, 0, Z_PLATE_IN) * Cylinder(f.bearing_od / 2, f.bearing_w, align=MIN)
    s = s - Pos(0, 0, Z_PLATE_OUT - f.bearing_w) * Cylinder(f.bearing_od / 2, f.bearing_w, align=MIN)
    s = s - Pos(0, 0, Z_PLATE_IN) * Cylinder(f.stub_d / 2 + 1.5, f.plate_t, align=MIN)
    # clutch boss: slot along the centre line for belt tension
    ux, uy = math.cos(ANG), math.sin(ANG)
    boss_r = CP.bearing_od / 2 + 5 + 1.0
    for t in [i * 0.5 for i in range(int(-2 * f.tension_slot), int(2 * f.tension_slot) + 1)]:
        s = s - Pos(MX + ux * t, MY + uy * t, Z_PLATE_IN) * Cylinder(boss_r, f.plate_t, align=MIN)
        for k in range(4):
            a = math.radians(45 + 90 * k)
            x = MX + CP.cover_bolt_r * math.cos(a) + ux * t
            y = MY + CP.cover_bolt_r * math.sin(a) + uy * t
            s = s - Pos(x, y, Z_PLATE_IN) * Cylinder(2.75, f.plate_t, align=MIN)
    for x, y in f.standoffs:
        s = s - Pos(x, y, Z_PLATE_IN) * Cylinder(3.3, f.plate_t, align=MIN)
    for k in range(3):  # encoder standoffs, M3 on r=15
        a = math.radians(90 + 120 * k)
        s = s - Pos(15 * math.cos(a), 15 * math.sin(a), Z_PLATE_IN) * Cylinder(1.25, f.plate_t, align=MIN)
    return s


def standoffs():
    return [Pos(x, y, 0) * ring(f.standoff_d / 2, 3.3, f.chamber) for x, y in f.standoffs]


def stub_shaft():
    tongue = Pos(0, 0, f.screw_end_z - f.screw_end_slot_d) * Box(f.screw_d - 2, f.screw_end_slot_w - 0.2,
                                                               f.screw_end_slot_d - f.screw_end_z + 0.01,
                                                               align=MIN)
    body = Cylinder(f.stub_d / 2, Z_PLATE_OUT + 1.0, align=MIN)
    enc = Pos(0, 0, Z_PLATE_OUT + 1.0) * Cylinder(f.stub_enc_d / 2, 14.0, align=MIN)
    return tongue + body + enc


def pulleys():
    small_od = pd(f.teeth_small) - 1.14
    big_od = pd(f.teeth_big) - 1.14
    z0 = W_PULLEY - f.pulley_w / 2
    big = ring(big_od / 2, f.stub_d / 2, f.pulley_w, z0) + ring(big_od / 2 + 2, f.stub_d / 2, 1.0, z0 - 1.0) + \
        ring(big_od / 2 + 2, f.stub_d / 2, 1.0, z0 + f.pulley_w)
    small = Pos(MX, MY, 0) * (ring(small_od / 2, CP.out_shaft_d / 2, f.pulley_w, z0)
                              + ring(small_od / 2 + 2, CP.out_shaft_d / 2, 1.0, z0 - 1.0)
                              + ring(small_od / 2 + 2, CP.out_shaft_d / 2, 1.0, z0 + f.pulley_w))
    return big, small


def belt():
    r1, r2 = pd(f.teeth_big) / 2, pd(f.teeth_small) / 2
    z0 = W_PULLEY - f.belt_w / 2
    return stadium(r1 + 1.2, r2 + 1.2, z0, f.belt_w) - stadium(r1 - 0.6, r2 - 0.6, z0 - 1, f.belt_w + 2)


def guard():
    """Printed belt guard wall around the chamber (PETG), between the table end and the plate."""
    r1 = pd(f.teeth_big) / 2 + 6
    r2 = pd(f.teeth_small) / 2 + 6
    outer = stadium(r1 + 2.0, r2 + 2.0, 0.5, f.chamber - 1.0)
    inner = stadium(r1, r2, 0.0, f.chamber)
    g = outer - inner
    g = g - Pos(0, 0, 0) * Cylinder(f.screw_d / 2 + 3, f.chamber, align=MIN)   # leadscrew pass-through
    return g


def encoder():
    """LPD3806-600BM: Ø38 x 36 body, Ø6 shaft; on 3 standoffs from the plate, flexible coupling 6-8."""
    z0 = Z_PLATE_OUT + f.encoder_standoff
    body = Pos(0, 0, z0) * Cylinder(19.0, 36.0, align=MIN)
    shaft = Pos(0, 0, z0 - 13) * Cylinder(3.0, 13.0, align=MIN)
    coupling = Pos(0, 0, Z_PLATE_OUT + 3) * Cylinder(9.5, 20.0, align=MIN)
    posts = [Pos(15 * math.cos(math.radians(90 + 120 * k)), 15 * math.sin(math.radians(90 + 120 * k)), Z_PLATE_OUT)
             * Cylinder(3.0, f.encoder_standoff, align=MIN) for k in range(3)]
    return body + shaft, coupling, posts


def mill_reference():
    """Table end + leadscrew stand-in (reference only)."""
    bracket = Pos(0, -10, -40) * Box(f.bracket_w, f.bracket_h, 40, align=MIN)
    table = Pos(0, 45, -120) * Box(180, 70, 120, align=MIN)
    screw = Pos(0, 0, -120) * Cylinder(f.screw_d / 2, 120 + f.screw_end_z, align=MIN)
    screw = screw - Pos(0, 0, f.screw_end_z - f.screw_end_slot_d) * Box(f.screw_d + 1, f.screw_end_slot_w, f.screw_end_slot_d + 1, align=MIN)
    return bracket + table + screw


def build():
    os.makedirs(OUT, exist_ok=True)
    cov, _ = clutch.cover()
    module = {
        "clutch_field_housing": clutch.field_housing(),
        "clutch_coil": clutch.coil(),
        "clutch_rotor": clutch.rotor(),
        "clutch_armature": clutch.armature(),
        "clutch_hub": clutch.output_hub(),
        "clutch_output_shaft": clutch.output_shaft(),
        "clutch_cover": cov,
        "motor": clutch.motor_stub() + Pos(0, 0, -114) * Box(57, 57, 104, align=MIN),
    }
    module = {k: place_module(v) for k, v in module.items()}
    big, small = pulleys()
    enc_body, coupling, posts = encoder()
    parts = {
        "plate": plate(),
        "standoffs": Compound(children=standoffs()),
        "stub_shaft": stub_shaft(),
        "pulley_40T": big,
        "pulley_20T": small,
        "belt": belt(),
        "belt_guard": guard(),
        "encoder": enc_body,
        "encoder_coupling": coupling,
        "encoder_posts": Compound(children=posts),
        "bearings_6002": Compound(children=[ring(f.bearing_od / 2, f.stub_d / 2, f.bearing_w, Z_PLATE_IN),
                                            ring(f.bearing_od / 2, f.stub_d / 2, f.bearing_w, Z_PLATE_OUT - f.bearing_w)]),
        "mill_reference": mill_reference(),
    }
    parts.update(module)
    for name, shape in parts.items():
        export_stl(shape, os.path.join(OUT, name + ".stl"))
        if not name.startswith(("mill_reference", "motor", "clutch_", "belt", "bearings")):
            export_step(shape, os.path.join(OUT, name + ".step"))
    export_step(Compound(children=list(parts.values())), os.path.join(OUT, "feed_unit_assembly.step"))
    return parts


def notes():
    d1, d2 = pd(f.teeth_small), pd(f.teeth_big)
    out_len = W0 + 114
    return f"""# X feed unit — SIEG X3.5D left table end (generated by feed_unit.py)

**Status: concept with PLACEHOLDER mill dimensions.** Measure the items in docs/mill.md, edit the
`F` dataclass at the top of `feed_unit.py`, re-run, and check the renders before cutting metal.

## Drive
| Item | Value |
|---|---|
| Belt | HTD 5M, {f.belt_len:.0f} mm ({f.belt_len / f.pitch_belt:.0f} T), {f.belt_w:.0f} mm wide |
| Pulleys | {f.teeth_small} T (PD {d1:.2f}) on the clutch output Ø{CP.out_shaft_d:.0f}; {f.teeth_big} T (PD {d2:.2f}) on the Ø{f.stub_d:.0f} stub → {f.teeth_big / f.teeth_small:.1f}:1 |
| Centre distance | {C:.2f} mm, tension slot ±{f.tension_slot:.0f} mm along the centre line |
| Module position | {C:.1f} mm from the leadscrew at {f.motor_angle_deg:.0f}° (below) |
| Belt plane | {W_PULLEY:.1f} mm out from the bracket face |
| Overall | ≈{out_len:.0f} mm out from the table end (motor tail) |

Firmware: `belt_ratio = {f.teeth_big / f.teeth_small:.1f}`, `encoder_counts = 2400` (per leadscrew turn; the encoder is on the stub).

## Parts to make
| Part | Material | Notes |
|---|---|---|
| Plate | 6061, {f.plate_t:.0f} mm | 2× 6002 seats (Ø{f.bearing_od:.0f} K6, {f.bearing_w:.0f} deep) from both faces, concentric; slotted Ø{2 * (CP.bearing_od / 2 + 6):.0f} hole + 4× M5 slots for the clutch module; {len(f.standoffs)}× Ø6.6 for the standoffs; 3× M3 for the encoder posts |
| Standoffs | steel or 6061 | {len(f.standoffs)}× Ø{f.standoff_d:.0f} × {f.chamber:.0f}, Ø6.6 thru (M6 screws into the table end) |
| Leadscrew stub | steel, Ø{f.stub_d:.0f} ground | tongue {f.screw_d - 2:.0f} × {f.screw_end_slot_w - 0.2:.1f} into the leadscrew slot; Ø{f.stub_enc_d:.0f} encoder end; pulley set screws on a flat |
| Belt guard | PETG print | closes the chamber between the table end and the plate |
| Encoder posts | 3× M3 standoffs, {f.encoder_standoff:.0f} mm | |

## Buy
- HTD 5M pulleys {f.teeth_small} T and {f.teeth_big} T for {f.belt_w:.0f} mm belt (re-bore to Ø{CP.out_shaft_d:.0f} / Ø{f.stub_d:.0f} H7).
- Belt {f.belt_len:.0f}-5M-{f.belt_w:.0f}.
- 2× 6002-2RS.
- Flexible coupling 6 × 8 mm (encoder).
- LPD3806-600BM-G5-24C encoder.
- M6 screws for the standoffs, 4× M5×30 for the clutch module, M3 for the encoder.

## PLACEHOLDERS (measure, then edit `F`)
- `screw_d`, `screw_end_slot_w`, `screw_end_slot_d`, `screw_end_z`: the left leadscrew end and its drive slot.
  - If there is no slot, use a pinned sleeve coupling instead of the tongue.
- `standoffs`: where M6 can go into the table end or the left bracket.
- `motor_angle_deg`: rotate the module if it hits the saddle, the base or the DRO cable at full travel.
- `chamber`: the minimum is set by the clutch output shaft; the script asserts it fits.
"""


if __name__ == "__main__":
    parts = build()
    with open(os.path.join(HERE, "FEED_UNIT.md"), "w") as fh:
        fh.write(notes())
    print("centre distance %.2f mm, belt plane W=%.1f, module motor face W=%.1f" % (C, W_PULLEY, W0))
    for name, shape in parts.items():
        bb = shape.bounding_box()
        print("%-22s %6.0f x %6.0f x %6.0f" % (name, bb.size.X, bb.size.Y, bb.size.Z))
