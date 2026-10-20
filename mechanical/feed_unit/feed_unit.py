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

from build123d import (Align, Box, Compound, Cylinder, Plane, Polyline, Pos, Rot, Solid, Text, export_step, export_stl,
                       extrude, make_face)

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
    standoffs: tuple = ((-46.0, 12.0), (46.0, 12.0), (-46.0, -78.0), (46.0, -78.0))  # lower pair clear of the module bolt heads
    # --- drive
    pitch_belt: float = 5.0          # HTD 5M
    teeth_small: int = 20
    teeth_big: int = 40
    belt_len: float = 295.0          # 295-5M-15 (59 teeth)
    belt_w: float = 15.0
    pulley_w: float = 21.0
    motor_angle_deg: float = -90.0   # module below the leadscrew; rotate if it hits the saddle or base
    # --- structure
    plate_t: float = 28.0            # 2x 6002 (9) + 2x (land + groove) + 2.0 spacer tube
    chamber: float = 35.0            # bracket face -> plate: Oldham disc, then the belt plane
    standoff_d: float = 14.0
    stub_d: float = 15.0             # at the 6002 bearings (15x32x9)
    stub_shoulder_d: float = 18.0    # locates the stub on the inner bearing's inner race
    stub_enc_d: float = 8.0
    bearing_od: float = 32.0
    bearing_w: float = 9.0
    circlip_groove: float = 1.3      # DIN 472-32 in the plate, DIN 471-15 on the stub
    circlip_land: float = 2.5        # material outside each plate groove (DIN 472-32 edge margin)
    tension_slot: float = 3.0        # +- adjustment of the module along the centre line
    # --- Oldham coupling (steel/bronze disc): tongue into the leadscrew slot, cross slot for the stub tongue
    oldham_d: float = 24.0           # outside the bracket, so not tied to the leadscrew diameter
    oldham_t: float = 9.0
    oldham_gap: float = 0.5          # each side: leadscrew end -> disc, disc -> stub end
    stub_tongue_w: float = 4.8
    stub_tongue_h: float = 4.0       # 14 x 4 bearing faces: ~45 MPa at 9 N·m cam-out -> steel or bronze disc
    # --- encoder (LPD3806: 3x M3 on r=15 in its front face) on an adapter disc
    enc_post_r: float = 23.0         # outside the Ø32 bearing bore
    enc_post_len: float = 21.0
    enc_disc_d: float = 50.0
    enc_disc_t: float = 5.0
    enc_disc_flat: float = 20.0      # flat toward the clutch module (its cover top is ~27 below the stub)
    # --- housing: ONE box for the drive, the electronics and the controls (see housing section below)
    panel_t: float = 3.0             # printed shell wall (PETG; ASA if the motor runs hot)
    front_t: float = 4.0             # printed front (control) panel
    rail: float = 10.0               # printed corner rails / bosses carrying M3 heat-set inserts
    lap: float = 13.0                # the shell overlaps the plate edges by this much (screws into the edges);
                                     # keeps the print height under the Prusa MK3S's 210 mm
    x_back: float = -55.0            # back edge of the plate = inside of the back wall
    x_part: float = 50.0             # partition, motor-bay face (clutch cover OD is at 44)
    bay: float = 61.0                # electronics bay depth, partition -> front panel (E-stop body 50 + spare)
    tail_clear: float = 6.0          # motor tail -> end panel
    pcb_standoff: float = 6.0        # printed bosses on the partition, M3 heat-set inserts
    pcb_w0: float = 180.0            # W of the PCB's x=0 edge; PCB x runs toward the plate (-W)
    pcb_y0: float = 28.0             # world y of the PCB's y=0 edge; the connector edge (y=100) is at the bottom
    col_w: float = 211.0             # control column centre (W): E-stop, toggle, rapid
    disp_w: float = 135.0            # OLED + dial column centre (W), over the PCB


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
    inset = f.circlip_land + f.circlip_groove + 0.2
    return Z_PLATE_IN + inset, Z_PLATE_OUT - inset - f.bearing_w


def module_bolts(t=0.0):
    """World XY of the 4 module bolts (clutch-local angles, Y flipped by place_module), at tension offset t."""
    ux, uy = math.cos(ANG), math.sin(ANG)
    return [(MX + CP.cover_bolt_r * math.cos(math.radians(a)) + ux * t,
             MY - CP.cover_bolt_r * math.sin(math.radians(a)) + uy * t) for a in CP.cover_bolt_deg]


def bolt_heads(t=0.0):
    """M5 SHCS heads (Ø8.5 x 5) on the plate's chamber face; for the interference check and renders."""
    return [Pos(x, y, Z_PLATE_IN - 5.0) * Cylinder(4.25, 5.0, align=MIN) for x, y in module_bolts(t)]


PLATE_TOP = max(max(y for _, y in f.standoffs) + 14, GUARD_R1 + 2.0 + 4.0)   # covers the guard's open face
X_PART = f.x_part + f.panel_t                       # partition, electronics-bay face
X_FRONT = X_PART + f.bay                            # front edge of the plate = inside of the front (control) panel


def plate():
    """Back wall of the whole unit: carries the stub bearings and the clutch module, and is widened to the operator
    side (+X) to close the electronics bay. The bay part is pocketed from the table side down to 6 mm."""
    s = Pos((f.x_back + X_FRONT) / 2, (PLATE_TOP + PLATE_BOT) / 2, Z_PLATE_IN) * Box(
        X_FRONT - f.x_back, PLATE_TOP - PLATE_BOT, f.plate_t, align=MIN)
    s = s - Pos((X_PART + 7 + X_FRONT - 8) / 2, (PLATE_TOP + PLATE_BOT) / 2, Z_PLATE_IN) * Box(
        X_FRONT - 8 - X_PART - 7, PLATE_TOP - PLATE_BOT - 16, f.plate_t - 6, align=MIN)
    # stub bearings: straight-through Ø32 H7 bore (one setup), DIN 472-32 grooves at both faces
    s = s - Pos(0, 0, Z_PLATE_IN) * Cylinder(f.bearing_od / 2, f.plate_t, align=MIN)
    for zg in (Z_PLATE_IN + f.circlip_land, Z_PLATE_OUT - f.circlip_land - f.circlip_groove):
        s = s - Pos(0, 0, zg) * Cylinder(33.7 / 2, f.circlip_groove, align=MIN)
    # clutch boss: slot along the centre line for belt tension
    ux, uy = math.cos(ANG), math.sin(ANG)
    boss_r = CP.bearing_od / 2 + 5 + 1.0
    for t in [i * 0.5 for i in range(int(-2 * f.tension_slot), int(2 * f.tension_slot) + 1)]:
        s = s - Pos(MX + ux * t, MY + uy * t, Z_PLATE_IN) * Cylinder(boss_r, f.plate_t, align=MIN)
        for x, y in module_bolts(t):
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
    """Steel or bronze disc: a tongue (along X) into the leadscrew slot, a cross slot (along Y) for the stub tongue."""
    d = Pos(0, 0, Z_OLDHAM) * Cylinder(f.oldham_d / 2, f.oldham_t, align=MIN)
    z_t = f.screw_end_z - f.screw_end_slot_d + 0.5                     # tongue tip, 0.5 off the slot bottom
    d = d + Pos(0, 0, z_t) * Box(f.screw_d - 2, f.screw_end_slot_w - 0.2, Z_OLDHAM - z_t + 0.01, align=MIN)
    d = d - Pos(0, 0, Z_OLDHAM + f.oldham_t - f.stub_tongue_h - 0.5) * Box(f.stub_tongue_w + 0.2, f.oldham_d + 1, 10, align=MIN)
    return d


def stub_shaft():
    tongue = Pos(0, 0, Z_STUB - f.oldham_gap - f.stub_tongue_h) * Box(f.stub_tongue_w, f.screw_d - 2,
                                                                   f.stub_tongue_h + f.oldham_gap + 0.01, align=MIN)
    a, b = bearing_zs()
    body = Pos(0, 0, Z_STUB) * Cylinder(f.stub_d / 2, Z_PLATE_OUT + 1.0 - Z_STUB, align=MIN)
    body = body + Pos(0, 0, a - 2.0) * Cylinder(f.stub_shoulder_d / 2, 2.0, align=MIN)      # shoulder on the inner race
    zg = a + f.bearing_w + 0.1                     # DIN 471-15 groove: outboard of the inner (locating) bearing
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
    g = g - Pos(0, 0, -1) * Cylinder(f.oldham_d / 2 + 2, back + 3, align=MIN)         # leadscrew end + Oldham
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


# ------------------------------------------------------------------ housing: everything in one box
# +X points at the operator: at the LEFT table end, Y (up) x Z (out of the table, i.e. to the operator's left) = toward
# the front of the mill. So the electronics bay and the control panel are on +X, and as the operator sees the front
# panel, W increases to the LEFT.
#   motor bay        x_back .. x_part      clutch module, motor, encoder
#   partition        x_part .. X_PART      PCB on standoffs on its bay face, grommet for the motor/clutch/encoder wires
#   electronics bay  X_PART .. X_FRONT     PCB, E-stop/toggle/rapid column, OLED + dial over the PCB
# The housing is PRINTED (it carries no load; the aluminium plate carries the bearings, the module and the belt):
#   shell        back + top + bottom + end walls + partition in ONE print, standing on its end wall (no supports),
#                overlapping the plate edges by `lap` and screwed into them; printed rails/bosses take M3 heat-set inserts
#   front panel  the control panel, printed face down, screwed to the shell rails and the plate's front edge
# The 36 V PSU stays outside (mains on a moving table, and its size): 36 V and the limit switches come in through
# two cable glands in the bottom of the bay.
T = f.panel_t
Y0, Y1 = PLATE_BOT, PLATE_TOP
W_TAIL = W0 + 114.0                                 # motor tail (motor_stub flange + 104 body)
W_END = W_TAIL + f.tail_clear + T                   # outer face of the end wall
W_SHELL0 = Z_PLATE_OUT - f.lap                      # open rim of the shell, on the plate edges
XB = f.x_back
RAIL = f.rail
INSERT_D, INSERT_L = 4.0, 6.0                       # M3 x 5.7 heat-set insert hole


def bx(x0, x1, y0, y1, w0, w1):
    return Pos(x0, y0, w0) * Box(x1 - x0, y1 - y0, w1 - w0, align=(Align.MIN, Align.MIN, Align.MIN))


def pcb_to_world(px, py, h):
    """PCB coordinates (x right, y down, h up from the board's underside) -> feed-unit world, PCB facing the operator."""
    return (X_PART + f.pcb_standoff + h, f.pcb_y0 - py, f.pcb_w0 - px)


def pcb_box(px0, px1, py0, py1, h0, h1):
    a, b = pcb_to_world(px0, py0, h0), pcb_to_world(px1, py1, h1)
    return bx(min(a[0], b[0]), max(a[0], b[0]), min(a[1], b[1]), max(a[1], b[1]), min(a[2], b[2]), max(a[2], b[2]))


# PCB bodies from the KiCad layout (the same numbers the old enclosure used); heights are assumptions
PCB_HOLES = ((4, 4), (96, 4), (4, 96), (96, 96))


def pcb_placeholder():
    parts = [pcb_box(0, 100, 0, 100, 0, 1.6)]
    for cx, cy in ((15.5, 37.0), (19.0, 74.0)):                                   # 12.5 x 25 bulk caps
        x, y, w = pcb_to_world(cx, cy, 1.6)
        parts.append(Pos(x, y, w) * Rot(0, 90, 0) * Cylinder(6.25, 25.0, align=MIN))
    parts.append(pcb_box(37, 91, 3, 27, 1.6, 5.6))                                # Pico
    for x0, x1 in ((12.0, 25.3), (26.5, 49.9), (50.9, 64.2)):                      # J1..J3
        parts.append(pcb_box(x0, x1, 89, 100, 1.6, 13.6))
    parts.append(pcb_box(65.6, 72.4, 77, 98, 1.6, 10.1))                          # J5
    parts.append(pcb_box(73.0, 83.9, 66.4, 92.9, 1.6, 11.6))                      # J4
    parts.append(pcb_box(0.0, 11.0, 59, 87, 1.6, 14.6))                           # fuse holder
    return parts


def pcb_bosses():
    """Printed bosses on the partition's bay face for the PCB (M3 heat-set inserts). Each has a 45-degree gusset on its
    +W side, which is 'down' while the shell prints standing on its end wall."""
    out = []
    for px, py in PCB_HOLES:
        _, y, w = pcb_to_world(px, py, 0)
        boss = bx(X_PART, X_PART + f.pcb_standoff, y - 4, y + 4, w - 4, w + 4)
        gusset = extrude(make_face(Polyline((X_PART, y - 4, w + 4), (X_PART + f.pcb_standoff, y - 4, w + 4),
                                            (X_PART, y - 4, w + 4 + f.pcb_standoff), close=True)), amount=8)
        boss = boss + gusset - xcyl(INSERT_D / 2, X_PART + f.pcb_standoff - INSERT_L, X_PART + f.pcb_standoff + 1, y, w)
        out.append(boss)
    return out


# front (control) panel layout, in (W, y) on the panel; X_FP = outer face
X_FP = X_FRONT + f.front_t
CTRL = {
    "estop": (f.col_w, 0.0), "toggle": (f.col_w, -55.0), "rapid": (f.col_w, -95.0),
    "oled": (f.disp_w, 8.0), "dial": (f.disp_w, -35.0), "adapter": (105.0, -100.0),
}
LABELS = (("SPEED", f.disp_w, -58.0), ("LEFT", f.col_w + 16, -41.0), ("STOP", f.col_w, -41.0),
          ("RIGHT", f.col_w - 17, -41.0), ("RAPID", f.col_w, -113.0))


def xcyl(r, x0, x1, y, w):
    return Pos(x0, y, w) * Rot(0, 90, 0) * Cylinder(r, x1 - x0, align=MIN)


def shell():
    """One print: back, top, bottom and end walls, the partition, the front corner rails and the PCB bosses."""
    x0, x1 = XB - T, X_FP                              # outer x extent (the front panel sits between top and bottom)
    w = (W_SHELL0, W_END)
    top = bx(x0, x1, Y1, Y1 + T, *w)
    bot = bx(x0, x1, Y0 - T, Y0, *w)
    back = bx(x0, XB, Y0 - T, Y1 + T, *w)
    end = bx(x0, x1, Y0 - T, Y1 + T, W_END - T, W_END)
    part = bx(f.x_part, X_PART, Y0, Y1, Z_PLATE_OUT, W_END - T)
    rails = [bx(X_FRONT - RAIL, X_FRONT, Y1 - RAIL, Y1, Z_PLATE_OUT, W_END - T),       # top-front
             bx(X_FRONT - RAIL, X_FRONT, Y0, Y0 + RAIL, Z_PLATE_OUT, W_END - T),       # bottom-front
             bx(X_FRONT - RAIL, X_FRONT, Y0, Y1, W_END - T - RAIL, W_END - T)]         # end-front
    s = top
    for p_ in [bot, back, end, part] + rails + pcb_bosses():
        s = s + p_
    # bottom: drawbolt access under the tension block, 2x M16 cable glands (36 V, limit switches) under J1 / J5,
    # vent slots under the motor and under the driver; end wall: vent slots high up (chimney flow, out of the chips)
    s = s - Pos(MX, Y0 - T - 1, Z_PLATE_OUT + 4) * Rot(-90, 0, 0) * Cylinder(6.0, T + 2, align=MIN)
    for wg in (110.0, 160.0):
        s = s - Pos(84.0, Y0 - T - 1, wg) * Rot(-90, 0, 0) * Cylinder(8.1, T + 2, align=MIN)
    for i in range(11):
        s = s - bx(-35 + 7 * i - 1.5, -35 + 7 * i + 1.5, Y0 - T - 1, Y0 + 1, 150, 215)
    for i in range(4):
        s = s - bx(68 + 8 * i - 1.5, 68 + 8 * i + 1.5, Y0 - T - 1, Y0 + 1, 122, 148)
    for xa, xb_ in ((-38, 38), (66, 100)):
        for i in range(4):
            s = s - bx(xa, xb_, 2 + 7 * i, 5 + 7 * i, W_END - T - 1, W_END + 1)
    # partition: grommet for the motor, clutch and encoder wires (below the PCB)
    s = s - xcyl(10.0, f.x_part - 1, X_PART + 1, -100.0, 120.0)
    return s


def front_panel():
    fp = bx(X_FRONT, X_FP, Y0, Y1, W_SHELL0, W_END - T)
    w, y = CTRL["dial"]                     # thin the panel to 3 mm round the EC11 so its bushing takes the nut
    fp = fp - xcyl(6.5, X_FRONT - 1, X_FRONT + f.front_t - 3.0, y, w)
    return control_cutouts(fp)


def control_cutouts(fp):
    w, y = CTRL["estop"]; fp = fp - xcyl(11.25, X_FRONT - 1, X_FP + 1, y, w)
    w, y = CTRL["toggle"]; fp = fp - xcyl(6.2, X_FRONT - 1, X_FP + 1, y, w)
    w, y = CTRL["rapid"]; fp = fp - xcyl(8.15, X_FRONT - 1, X_FP + 1, y, w)
    w, y = CTRL["dial"]; fp = fp - xcyl(3.7, X_FRONT - 1, X_FP + 1, y, w) - xcyl(1.6, X_FRONT - 1, X_FRONT + 1.0, y + 7.8, w)
    w, y = CTRL["oled"]; fp = fp - bx(X_FRONT - 1, X_FP + 1, y - 8, y + 8, w - 15, w + 15)
    for dw, dy in ((-15.2, -14.25), (15.2, -14.25), (-15.2, 14.25), (15.2, 14.25)):   # OLED M2 tapped
        fp = fp - xcyl(0.8, X_FRONT - 1, X_FRONT + 2.0, y - 2 + dy, w + dw)
    w, y = CTRL["adapter"]
    for dw, dy in ((-19.5, -11), (19.5, -11), (-19.5, 11), (19.5, 11)):              # adapter board M3 tapped
        fp = fp - xcyl(1.25, X_FRONT - 1, X_FRONT + 2.0, y + dy, w + dw)
    for txt, w, y in LABELS:                                                       # engraved 0.6 deep (3 layers)
        pl = Plane(origin=(X_FP, y, w), x_dir=(0, 0, -1), z_dir=(1, 0, 0))
        try:
            fp = fp - extrude(pl * Text(txt, 4.5), amount=-0.6)
        except Exception as e:  # noqa: BLE001 - a missing font must not stop the build
            print("label %s skipped: %s" % (txt, e))
    return fp


def controls():
    """Placeholder bodies for the checks and renders (not machined)."""
    c = []
    w, y = CTRL["estop"]
    c += [xcyl(11.0, X_FRONT - 50, X_FRONT, y, w), xcyl(20.0, X_FP + 1, X_FP + 26, y, w), xcyl(30.0, X_FP, X_FP + 1, y, w)]
    w, y = CTRL["toggle"]
    c += [bx(X_FRONT - 25, X_FRONT, y - 6.5, y + 6.5, w - 6.5, w + 6.5), xcyl(2.0, X_FP, X_FP + 20, y, w)]
    w, y = CTRL["rapid"]
    c += [xcyl(8.0, X_FRONT - 25, X_FRONT, y, w), xcyl(10.0, X_FP, X_FP + 10, y, w)]
    w, y = CTRL["dial"]
    c += [bx(X_FRONT - 15, X_FRONT, y - 6, y + 6, w - 6, w + 6), xcyl(15.0, X_FP + 2, X_FP + 17, y, w)]
    w, y = CTRL["oled"]
    c += [bx(X_FRONT - 7.5, X_FRONT - 2.5, y - 2 - 17, y - 2 + 17, w - 18, w + 18)]
    for dw, dy in ((-15.2, -14.25), (15.2, -14.25), (-15.2, 14.25), (15.2, 14.25)):
        c.append(xcyl(1.25, X_FRONT - 2.5, X_FRONT, y - 2 + dy, w + dw))
    w, y = CTRL["adapter"]
    c += [bx(X_FRONT - 6.6, X_FRONT - 5, y - 14, y + 14, w - 22.5, w + 22.5), bx(X_FRONT - 15.6, X_FRONT - 6.6, y - 5, y + 5, w - 18, w + 18)]
    for dw, dy in ((-19.5, -11), (19.5, -11), (-19.5, 11), (19.5, 11)):
        c.append(xcyl(2.5, X_FRONT - 5, X_FRONT, y + dy, w + dw))
    return c


def screw_joints():
    """(part screwed, target, outer point, inward unit vector, kind). 'tap' = M3 tapped 8 deep in the aluminium plate,
    'insert' = M3 heat-set insert in a printed rail. Countersunk M3 through the printed wall/panel."""
    j = []
    wl = (W_SHELL0 + Z_PLATE_OUT) / 2                 # mid-lap: into the plate edges
    for x in (-35.0, 10.0, 60.0, 100.0):
        j += [("shell", "plate", (x, Y1 + T, wl), (0, -1, 0), "tap"), ("shell", "plate", (x, Y0 - T, wl), (0, 1, 0), "tap")]
    for y in (-110.0, -50.0, 10.0):
        j.append(("shell", "plate", (XB - T, y, wl), (1, 0, 0), "tap"))
    for y in (-90.0, -40.0, 10.0):
        j.append(("panel_front", "plate", (X_FP, y, wl), (-1, 0, 0), "tap"))
    for w in (100.0, 160.0):
        j += [("panel_front", "shell", (X_FP, Y1 - RAIL / 2, w), (-1, 0, 0), "insert"),
              ("panel_front", "shell", (X_FP, Y0 + RAIL / 2, w), (-1, 0, 0), "insert")]
    for y in (-120.0, -75.0, 25.0):     # end rail: clear of the E-stop legend ring and the toggle labels
        j.append(("panel_front", "shell", (X_FP, y, W_END - T - RAIL / 2), (-1, 0, 0), "insert"))
    return j


def housing():
    return {"shell": shell(), "panel_front": front_panel()}


def apply_screws(parts):
    for part_, target, p, d, kind in screw_joints():
        th = T if part_ == "shell" else f.front_t
        pl = Plane(origin=p, z_dir=d)
        parts[part_] = parts[part_] - Solid.make_cylinder(1.7, th + 0.01, pl) - Solid.make_cone(3.25, 1.7, 1.55, pl)
        inner = Plane(origin=tuple(p[i] + d[i] * th for i in range(3)), z_dir=d)
        r, depth = (1.25, 8.0) if kind == "tap" else (INSERT_D / 2, INSERT_L)
        parts[target] = parts[target] - Solid.make_cylinder(r, depth, inner)
    return parts


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
        "module_bolt_heads": Compound(children=bolt_heads()),
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
    parts.update(housing())
    parts = apply_screws(parts)
    parts["pcb_placeholder"] = Compound(children=pcb_placeholder())
    parts["controls"] = Compound(children=controls())
    for name, shape in parts.items():
        export_stl(shape, os.path.join(OUT, name + ".stl"))
        if not name.startswith(("mill_reference", "motor", "clutch_", "belt", "bearings", "module_bolt", "pcb_", "controls")):
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
| Overall | ≈{W_END:.0f} mm out from the table end (housing end panel) |

Firmware: `belt_ratio = {f.teeth_big / f.teeth_small:.1f}`, `encoder_counts = 2400` (per leadscrew turn; the encoder is on the stub).

## Parts to make
| Part | Material | Notes |
|---|---|---|
| Plate | 6061, {f.plate_t:.0f} mm | **Ø{f.bearing_od:.0f} H7 straight through** (one setup) with DIN 472-32 grooves (Ø33.7 × {f.circlip_groove}) leaving a **{f.circlip_land} land** at each face (keep the entry chamfer smaller); slotted Ø{2 * (CP.bearing_od / 2 + 6):.0f} hole + 4× Ø5.5 slots (±{f.tension_slot:.0f}) for the clutch module; {len(f.standoffs)}× Ø6.6 for the standoffs; 3× M3 × 8 deep on r={f.enc_post_r:.0f} (encoder posts, outer face); 2× M5 × 12 for the tension block. Top edge {max(max(y for _, y in f.standoffs) + 14, GUARD_R1 + 6):.0f} above the leadscrew so it covers the guard |
| Standoffs | steel or 6061 | {len(f.standoffs)}× Ø{f.standoff_d:.0f} × {f.chamber:.0f}, Ø6.6 thru (M6 screws into the table end) |
| Leadscrew stub | steel, Ø{f.stub_d:.0f} ground | from the table side: tongue {f.stub_tongue_w} × {f.screw_d - 2:.0f} × {f.stub_tongue_h:.0f} (across the Oldham slot), Ø{f.stub_d:.0f} with a 0.6 flat for the 40T set screws, **Ø{f.stub_shoulder_d:.0f} × 2 shoulder** on the inner bearing's inner race, Ø{f.stub_d:.0f} k6 at the inner (locating) bearing and h6 at the outer (floating) one, DIN 471-15 groove (Ø14.3 × 1.1) just outboard of the inner bearing, Ø{f.stub_enc_d:.0f} × 10 encoder end |
| Bearing spacer tube | steel or 6061 | Ø{f.bearing_od - 0.05:.2f} / Ø28 × {bearing_zs()[1] - bearing_zs()[0] - f.bearing_w:.1f}, between the outer races. The inner bearing locates the stub (shoulder + DIN 471-15 either side of it); the outer one floats on the stub |
| Oldham disc | steel (1045) or bronze (CuSn12), greased — acetal creeps at the ~45 MPa of a stall/cam-out | Ø{f.oldham_d:.0f} × {f.oldham_t:.0f}: tongue {f.screw_d - 2:.0f} × {f.screw_end_slot_w - 0.2:.1f} on one face (into the leadscrew slot), cross slot {f.stub_tongue_w + 0.2:.1f} wide × {f.stub_tongue_h + 0.5:.1f} deep on the other. Takes up to ~1 mm offset between the plate and the leadscrew, and end float |
| Encoder disc | 6061, {f.enc_disc_t:.0f} mm | Ø{f.enc_disc_d:.0f} with a flat {f.enc_disc_flat:.0f} from centre toward the clutch module; Ø22 centre hole; 3× Ø3.4 on r={f.enc_post_r:.0f} (posts) and 3× Ø3.4 on r=15, 60° offset (encoder, **check your encoder's hole circle**) |
| Encoder posts | 3× M3 hex standoffs F-F, {f.enc_post_len:.0f} mm | |
| Tension block | steel | 44 × 14 × 12 on the plate's outer face below the module, 2× M5 to the plate; M5 drawbolt through it into a radial M5 tapped 7 deep in the cover's front wall (bottom, between two module bolts). Tighten the drawbolt to tension the belt, then lock the 4 module bolts |
| Belt guard | PETG print | a cup: closed back against the table end (Ø{f.oldham_d + 4:.0f} pass-through for the leadscrew and Oldham disc), open face covered by the plate; captured between the two |

## Assembly
1. Plate: outer circlip, outer 6002 (pressed in from the chamber side), spacer tube.
2. Stub: inner 6002 against the shoulder, DIN 471-15 behind it. Push the stub through the outer bearing from the
   chamber side (support the outer bearing's inner race) until the inner bearing seats, then the inner circlip.
3. Encoder to its disc, coupling on the encoder shaft, disc onto the posts, coupling onto the stub end.
4. 40T pulley on the stub (set screws on the flat), Oldham disc on the stub tongue, guard into the chamber, then
   the plate onto the standoffs: the Oldham tongue goes into the leadscrew slot.
5. Clutch module through the plate's slotted hole, 20T pulley and belt, 4 module bolts finger-tight, drawbolt to tension, lock.
   The bolt heads are on the chamber side, outside the guard outline: reach them from the side with a ball-end key.

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

## Housing: the whole power feed in one box (3D printed)
The drive, the controller PCB and the controls share one housing on the table end. Outside it there is only the
belt chamber (closed by the guard cup), the 36 V supply and the limit switches. The housing carries no load (the
aluminium plate carries the bearings, the module and the belt), so it is printed.

| | |
|---|---|
| Outer size | {X_FP - (XB - T):.0f} (front-back) × {Y1 - Y0 + 2 * T:.0f} (high) × {W_END - W_SHELL0:.0f} mm (along the table), plus the {f.chamber:.0f} mm belt chamber and the controls proud of the front |
| `shell` (one print) | back, top, bottom and end walls ({T:.0f} mm), the partition, the front corner rails and the PCB bosses. **Print standing on the end wall** (open rim up): {X_FP - (XB - T):.0f} × {Y1 - Y0 + 2 * T:.0f} on the bed, {W_END - W_SHELL0:.0f} mm tall, fits a Prusa MK3S (250 × 210 × 210), no supports. PETG, 4 perimeters, 20 % gyroid; ASA if the motor runs hot (see Heat) |
| `panel_front` (one print) | the control panel, {f.front_t:.0f} mm, **printed face down** ({Y1 - Y0:.0f} × {W_END - T - W_SHELL0:.0f}); labels engraved 0.6 mm (on the bed side, so they come out crisp); thinned to 3 mm round the dial so the EC11 bushing takes its nut |
| Back wall | the aluminium plate, widened to x = {X_FRONT:.0f} on the operator side; the bay part pocketed to 6 mm from the table side |
| Fixing | the shell overlaps the plate edges by {f.lap:.0f} mm: {sum(1 for j in screw_joints() if j[0] == "shell")}× M3×8 countersunk into tapped holes in the plate's top, bottom and back edges. Front panel: {sum(1 for j in screw_joints() if j[0] == "panel_front" and j[4] == "tap")}× M3×10 into the plate's front edge, {sum(1 for j in screw_joints() if j[4] == "insert")}× M3×8 into heat-set inserts (M3 × 5.7, Ø4.0 holes) in the printed rails |
| PCB | 4 printed bosses on the partition (heat-set inserts, M3×6 screws), components toward the operator, connector edge down; Ø20 grommet below the PCB for the motor, clutch and encoder wires |
| Electronics bay | {f.bay:.0f} deep (E-stop body 50 + spare); the controls hang from the front panel, the E-stop column beside the PCB |
| Front panel = control panel | E-stop, LEFT/STOP/RIGHT toggle and RAPID in a column at the outer end; OLED and SPEED dial over the PCB; adapter board (2×7 IDC) on M3 inserts, 150 mm ribbon to J4 so the panel can come off and hang beside the unit |
| Bottom | Ø12 access to the drawbolt; 2× M16 cable glands under J1 (36 V) and J5 (limit switches); vent slots under the motor and under the driver |
| End wall | vent slots high up (air in at the bottom, out at the top of the end face, away from falling chips) |

- **USB / fuse / BOOTSEL:** take the front panel off (the ribbon stays plugged in). The USB-C is on the PCB's
  connector edge, which faces down into the empty space above the glands.
- **Heat:** the motor is inside. Default run current is 3 A (~16 W in the coils), 1 A at standstill. PETG softens
  around 75–80 °C; the motor is 6 mm from the end wall and never touches the shell. On the bench, run it closed for
  30 min: if the motor case passes ~65 °C, lower `MOTOR mA` in the menu (light feeds need far less), reprint the
  shell in ASA, or fit a 40 mm fan on the end wall's vents.
- **Order:** plate (bearings, stub, module, encoder) → PCB into the shell's bosses → slide the shell over the
  module onto the plate edges and screw it on → wire through the grommet → front panel with its controls last.

## PLACEHOLDERS (measure, then edit `F`)
- `screw_d`, `screw_end_slot_w`, `screw_end_slot_d`, `screw_end_z`: the left leadscrew end and its drive slot.
  - If there is no slot, use a pinned sleeve coupling instead of the tongue.
- `standoffs`: where M6 can go into the table end or the left bracket.
- `motor_angle_deg`: rotate the module if it hits the saddle, the base or the DRO cable at full travel.
- The housing reaches {X_FP + 26 - 90:.0f} mm in front of the table's front edge (E-stop head included) and {-(Y0 - T):.0f} mm below the
  leadscrew: check both against the saddle, the DRO reader and the base at full right travel.
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
