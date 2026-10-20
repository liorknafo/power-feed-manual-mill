"""X-axis feed unit for the SIEG X3.5D: mount plate, 2:1 belt drive, leadscrew stub, encoder.

Run:  mechanical/.venv/bin/python mechanical/feed_unit/feed_unit.py
Out:  mechanical/feed_unit/out/*.step|*.stl, renders; FEED_UNIT.md (parts, placeholders).

Layout (docs/mill.md: the feed goes on the LEFT table end, where SIEG's own SX3.5P feeder mounts):
  World Z = leadscrew axis, pointing OUT of the table (away from the mill). Z = 0 is the face of the
  table's left leadscrew bracket. Y = up. The motor/clutch module sits below the leadscrew.

  table end | belt chamber (standoffs, guard) | 20 mm plate | clutch module | motor
            |  40T pulley on the stub   20T on the clutch output   |
  The leadscrew stub runs in 2x 6002 in the plate (through bore, circlips, spacer tube); it drives the
  leadscrew end through an Oldham disc, so the plate need not be exactly on the leadscrew axis. The
  output encoder sits on the stub's outer end (after the belt, so it also sees belt slip).

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
    plate_t: float = 25.0             # 2x 6002 (9) + circlip insets + 3.6 spacer tube
    chamber: float = 38.0            # bracket face -> plate: Oldham disc, then the belt plane
    standoff_d: float = 14.0
    stub_d: float = 15.0             # at the 6002 bearings (15x32x9)
    stub_shoulder_d: float = 18.0    # locates the stub on the inner bearing's inner race
    stub_enc_d: float = 8.0
    bearing_od: float = 32.0
    bearing_w: float = 9.0
    circlip_groove: float = 1.3      # DIN 472-32 in the plate, DIN 471-15 on the stub
    tension_slot: float = 3.0        # +- adjustment of the module along the centre line
    # --- Oldham coupling (acetal disc): tongue into the leadscrew slot, cross slot for the stub tongue
    oldham_t: float = 8.0
    oldham_gap: float = 0.5          # each side: leadscrew end -> disc, disc -> stub end
    stub_tongue_w: float = 4.8
    stub_tongue_h: float = 3.0
    # --- encoder (LPD3806: 3x M3 on r=15 in its front face) on an adapter disc
    enc_post_r: float = 23.0         # outside the Ø32 bearing bore
    enc_post_len: float = 21.0
    enc_disc_d: float = 50.0
    enc_disc_t: float = 5.0
    enc_disc_flat: float = 20.0      # flat toward the clutch module (its cover top is ~27 below the stub)


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
GUARD_R1_ = pd(f.teeth_big) / 2 + 6
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
GUARD_R1 = GUARD_R1_
PLATE_BOT = MY - (CP.housing_od / 2 + CP.cover_wall) - f.tension_slot - 22   # room for the tension block


def bearing_zs():
    """6002 seats in the plate's straight-through Ø32 bore, inset past the circlip grooves."""
    inset = f.circlip_groove + 0.4
    return Z_PLATE_IN + inset, Z_PLATE_OUT - inset - f.bearing_w


def plate():
    w_side = 55.0
    top = max(max(y for _, y in f.standoffs) + 14, GUARD_R1 + 2.0 + 4.0)   # covers the guard's open face
    s = Pos(0, (top + PLATE_BOT) / 2, Z_PLATE_IN) * Box(2 * w_side, top - PLATE_BOT, f.plate_t, align=MIN)
    # stub bearings: straight-through Ø32 H7 bore (one setup), DIN 472-32 grooves at both faces
    s = s - Pos(0, 0, Z_PLATE_IN) * Cylinder(f.bearing_od / 2, f.plate_t, align=MIN)
    for zg in (Z_PLATE_IN + 0.2, Z_PLATE_OUT - 0.2 - f.circlip_groove):
        s = s - Pos(0, 0, zg) * Cylinder(33.7 / 2, f.circlip_groove, align=MIN)
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
    for k in range(3):  # encoder posts, M3 tapped 8 deep from the outer face
        a = math.radians(90 + 120 * k)
        s = s - Pos(f.enc_post_r * math.cos(a), f.enc_post_r * math.sin(a), Z_PLATE_OUT - 8) * Cylinder(1.25, 8, align=MIN)
    for x in (-15.0, 15.0):  # tension block, 2x M5 tapped from the outer face
        s = s - Pos(MX + x, TB_Y, Z_PLATE_OUT - 12) * Cylinder(2.1, 12, align=MIN)
    return s


TB_Y = MY - (CP.housing_od / 2 + CP.cover_wall) - f.tension_slot - 10   # tension block centre (below the module)


def tension_block():
    """Steel block on the plate's outer face, below the module. An M5 drawbolt through it into a radial
    tapped hole in the cover's front wall pulls the module away from the leadscrew: belt tension."""
    b = Pos(MX, TB_Y, Z_PLATE_OUT) * Box(44, 14, 12, align=MIN)
    for x in (-15.0, 15.0):
        b = b - Pos(MX + x, TB_Y, Z_PLATE_OUT) * Cylinder(2.75, 12, align=MIN)
    b = b - Pos(MX, TB_Y, Z_PLATE_OUT + 4) * Rot(90, 0, 0) * Cylinder(2.75, 20)          # drawbolt clearance
    return b


def standoffs():
    return [Pos(x, y, 0) * ring(f.standoff_d / 2, 3.3, f.chamber) for x, y in f.standoffs]


Z_OLDHAM = max(f.screw_end_z, 0.0) + f.oldham_gap           # disc, leadscrew side (outside the bracket face)
Z_STUB = Z_OLDHAM + f.oldham_t + f.oldham_gap               # stub end face (tongue tip is in the disc)


def oldham():
    """Acetal disc: a tongue (along X) into the leadscrew slot, a cross slot (along Y) for the stub tongue."""
    d = Pos(0, 0, Z_OLDHAM) * Cylinder(f.screw_d / 2, f.oldham_t, align=MIN)
    z_t = f.screw_end_z - f.screw_end_slot_d + 0.5                     # tongue tip, 0.5 off the slot bottom
    d = d + Pos(0, 0, z_t) * Box(f.screw_d - 2, f.screw_end_slot_w - 0.2, Z_OLDHAM - z_t + 0.01, align=MIN)
    d = d - Pos(0, 0, Z_OLDHAM + f.oldham_t - f.stub_tongue_h - 0.5) * Box(f.stub_tongue_w + 0.2, f.screw_d + 1, 10, align=MIN)
    return d


def stub_shaft():
    tongue = Pos(0, 0, Z_STUB - f.oldham_gap - f.stub_tongue_h) * Box(f.stub_tongue_w, f.screw_d - 2,
                                                                   f.stub_tongue_h + f.oldham_gap + 0.01, align=MIN)
    a, b = bearing_zs()
    body = Pos(0, 0, Z_STUB) * Cylinder(f.stub_d / 2, Z_PLATE_OUT + 1.0 - Z_STUB, align=MIN)
    body = body + Pos(0, 0, a - 2.0) * Cylinder(f.stub_shoulder_d / 2, 2.0, align=MIN)      # shoulder on the inner race
    zg = b + f.bearing_w + 0.1                                                           # DIN 471-15 groove
    body = body - Pos(0, 0, zg) * (Cylinder(f.stub_d / 2 + 1, 1.1, align=MIN) - Cylinder(14.3 / 2, 1.1, align=MIN))
    flat = (Align.CENTER, Align.CENTER, Align.MIN)
    body = body - Pos(f.stub_d / 2 - 0.6 + 5, 0, Z_STUB) * Box(10, 20, a - 2.0 - Z_STUB, align=flat)  # pulley set-screw flat
    enc = Pos(0, 0, Z_PLATE_OUT + 1.0) * Cylinder(f.stub_enc_d / 2, 10.0, align=MIN)
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
    """Printed belt-guard cup (PETG): closed back against the table end, open face covered by the plate.
    Captured between the two; the back has a pass-through for the leadscrew end and the Oldham disc."""
    r1 = GUARD_R1
    r2 = pd(f.teeth_small) / 2 + 6
    back = 2.0
    outer = stadium(r1 + 2.0, r2 + 2.0, 0.5, f.chamber - 1.0)
    inner = stadium(r1, r2, 0.5 + back, f.chamber)
    g = outer - inner
    g = g - Pos(0, 0, -1) * Cylinder(f.screw_d / 2 + 4, back + 3, align=MIN)          # leadscrew end + Oldham
    return g


def encoder():
    """LPD3806-600BM: Ø38 x 36 body, Ø6 shaft x 13. Screwed to an adapter disc (3x M3 on r=15), the disc
    sits on 3 posts at r=23 (outside the bearing bore); flexible coupling 6-8 between them."""
    zd = Z_PLATE_OUT + f.enc_post_len
    z0 = zd + f.enc_disc_t
    body = Pos(0, 0, z0) * Cylinder(19.0, 36.0, align=MIN)
    shaft = Pos(0, 0, z0 - 13) * Cylinder(3.0, 13.0, align=MIN)
    coupling = Pos(0, 0, Z_PLATE_OUT + 3) * Cylinder(9.5, 20.0, align=MIN)
    posts = [Pos(f.enc_post_r * math.cos(math.radians(90 + 120 * k)), f.enc_post_r * math.sin(math.radians(90 + 120 * k)),
                 Z_PLATE_OUT) * Cylinder(3.0, f.enc_post_len, align=MIN) for k in range(3)]
    disc = Pos(0, 0, zd) * Cylinder(f.enc_disc_d / 2, f.enc_disc_t, align=MIN)
    disc = disc - Pos(0, -f.enc_disc_flat - 20, zd) * Box(80, 40, f.enc_disc_t, align=MIN)
    disc = disc - Pos(0, 0, zd) * Cylinder(11.0, f.enc_disc_t, align=MIN)                   # encoder pilot + coupling access
    for k in range(3):
        a = math.radians(90 + 120 * k)
        disc = disc - Pos(f.enc_post_r * math.cos(a), f.enc_post_r * math.sin(a), zd) * Cylinder(1.7, f.enc_disc_t, align=MIN)
        disc = disc - Pos(15 * math.cos(a + math.pi / 3), 15 * math.sin(a + math.pi / 3), zd) * Cylinder(1.7, f.enc_disc_t, align=MIN)
    return body + shaft, coupling, posts, disc


def mill_reference():
    """Table end + leadscrew stand-in (reference only)."""
    bracket = Pos(0, -10, -40) * Box(f.bracket_w, f.bracket_h, 40, align=MIN) - Cylinder(f.screw_d / 2 + 2, 100)
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
    enc_body, coupling, posts, disc = encoder()
    a, b = bearing_zs()
    parts = {
        "plate": plate(),
        "standoffs": Compound(children=standoffs()),
        "stub_shaft": stub_shaft(),
        "oldham_disc": oldham(),
        "tension_block": tension_block(),
        "encoder_disc": disc,
        "pulley_40T": big,
        "pulley_20T": small,
        "belt": belt(),
        "belt_guard": guard(),
        "encoder": enc_body,
        "encoder_coupling": coupling,
        "encoder_posts": Compound(children=posts),
        "bearings_6002": Compound(children=[ring(f.bearing_od / 2, f.stub_d / 2, f.bearing_w, a),
                                            ring(f.bearing_od / 2, f.stub_d / 2, f.bearing_w, b),
                                            ring(f.bearing_od / 2 - 0.05, 14.0, b - a - f.bearing_w, a + f.bearing_w)]),
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
| Plate | 6061, {f.plate_t:.0f} mm | **Ø{f.bearing_od:.0f} H7 straight through** (one setup) with DIN 472-32 grooves (Ø33.7 × {f.circlip_groove}) 0.2 in from each face; slotted Ø{2 * (CP.bearing_od / 2 + 6):.0f} hole + 4× Ø5.5 slots (±{f.tension_slot:.0f}) for the clutch module; {len(f.standoffs)}× Ø6.6 for the standoffs; 3× M3 × 8 deep on r={f.enc_post_r:.0f} (encoder posts, outer face); 2× M5 × 12 for the tension block. Top edge {max(max(y for _, y in f.standoffs) + 14, GUARD_R1 + 6):.0f} above the leadscrew so it covers the guard |
| Standoffs | steel or 6061 | {len(f.standoffs)}× Ø{f.standoff_d:.0f} × {f.chamber:.0f}, Ø6.6 thru (M6 screws into the table end) |
| Leadscrew stub | steel, Ø{f.stub_d:.0f} ground | from the table side: tongue {f.stub_tongue_w} × {f.screw_d - 2:.0f} × {f.stub_tongue_h:.0f} (across the Oldham slot), Ø{f.stub_d:.0f} with a 0.6 flat for the 40T set screws, **Ø{f.stub_shoulder_d:.0f} × 2 shoulder** on the inner bearing's inner race, Ø{f.stub_d:.0f} k6 at the bearings, DIN 471-15 groove (Ø14.3 × 1.1) outboard of the outer bearing, Ø{f.stub_enc_d:.0f} × 10 encoder end |
| Bearing spacer tube | steel or 6061 | Ø{f.bearing_od - 0.05:.2f} / Ø28 × {bearing_zs()[1] - bearing_zs()[0] - f.bearing_w:.1f}, between the outer races. With the two circlips it takes the axial load both ways |
| Oldham disc | acetal (POM) | Ø{f.screw_d:.0f} × {f.oldham_t:.0f}: tongue {f.screw_d - 2:.0f} × {f.screw_end_slot_w - 0.2:.1f} on one face (into the leadscrew slot), cross slot {f.stub_tongue_w + 0.2:.1f} wide × {f.stub_tongue_h + 0.5:.1f} deep on the other. Takes up to ~1 mm offset between the plate and the leadscrew, and end float |
| Encoder disc | 6061, {f.enc_disc_t:.0f} mm | Ø{f.enc_disc_d:.0f} with a flat {f.enc_disc_flat:.0f} from centre toward the clutch module; Ø22 centre hole; 3× Ø3.4 on r={f.enc_post_r:.0f} (posts) and 3× Ø3.4 on r=15, 60° offset (encoder, **check your encoder's hole circle**) |
| Encoder posts | 3× M3 hex standoffs F-F, {f.enc_post_len:.0f} mm | |
| Tension block | steel | 44 × 14 × 12 on the plate's outer face below the module, 2× M5 to the plate; M5 drawbolt through it into a radial M5 tapped 7 deep in the cover's front wall (bottom, between two module bolts). Tighten the drawbolt to tension the belt, then lock the 4 module bolts |
| Belt guard | PETG print | a cup: closed back against the table end (Ø{f.screw_d + 8:.0f} pass-through for the leadscrew and Oldham disc), open face covered by the plate; captured between the two |

## Assembly
1. Plate: inner circlip, inner 6002, spacer tube, outer 6002, outer circlip.
2. Stub in from the chamber side until the shoulder stops on the inner bearing; DIN 471-15 on the outside.
3. Encoder to its disc, coupling on the encoder shaft, disc onto the posts, coupling onto the stub end.
4. 40T pulley on the stub (set screws on the flat), Oldham disc on the stub tongue, guard into the chamber, then
   the plate onto the standoffs: the Oldham tongue goes into the leadscrew slot.
5. Clutch module through the plate's slotted hole, 20T pulley and belt, 4 module bolts finger-tight, drawbolt to tension, lock.

## Buy
- HTD 5M pulleys {f.teeth_small} T and {f.teeth_big} T for {f.belt_w:.0f} mm belt (re-bore to Ø{CP.out_shaft_d:.0f} / Ø{f.stub_d:.0f} H7).
- Belt {f.belt_len:.0f}-5M-{f.belt_w:.0f}.
- 2× 6002-2RS, 2× DIN 472-32, 1× DIN 471-15.
- Flexible coupling 6 × 8 mm (encoder).
- LPD3806-600BM-G5-24C encoder.
- 4× M6 for the standoffs; 4× M5×35 + washers for the clutch module ({f.plate_t:.0f} plate + 8 wall: the tips stop clear of the hub); 2× M5×20 + 1× M5×30 (tension block, drawbolt); 3× M3 hex standoffs {f.enc_post_len:.0f} mm and 6× M3×8.

## Notes
- The module hangs ≈{out_len - Z_PLATE_OUT:.0f} mm off the plate on 4 M5 into the cover. The cover's spigot over the clutch housing is
  {CP.housing_len - CP.cover_spigot_z:.0f} mm long with {CP.cover_screws} radial M4, so the motor is held stiffly. A strap from the motor's tail to the
  table end is optional if the mill vibrates a lot.
- The chamber ({f.chamber:.0f} mm) is set by the Oldham disc plus the belt plane: the 40T pulley's inner flange clears the stub tongue.

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
