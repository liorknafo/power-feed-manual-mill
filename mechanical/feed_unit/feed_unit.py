"""X-axis feed unit for the SIEG X3.5D: mount plate, 2:1 belt drive, leadscrew stub, encoder.

Run:  mechanical/.venv/bin/python mechanical/feed_unit/feed_unit.py
Out:  mechanical/feed_unit/out/*.step|*.stl, renders; FEED_UNIT.md (parts, placeholders).

Layout (docs/mill.md: the feed goes on the LEFT table end, where SIEG's own SX3.5P feeder mounts):
  World Z = leadscrew axis, pointing OUT of the table (away from the mill). Z = 0 is the face of the
  table's left leadscrew bracket. Y = up. The motor/clutch module sits below the leadscrew.

  table end | belt chamber (standoffs, guard) | 28 mm plate | clutch module | motor
            |  40T pulley on the stub   20T on the clutch output   |
  The leadscrew stub runs in 2x 6002 in the plate (through bore, circlips, spacer tube); it drives the
  leadscrew end through an Oldham disc, so the plate need not be exactly on the leadscrew axis. The
  output encoder sits on the stub's outer end (after the belt, so it also sees belt slip).

Every value marked PLACEHOLDER must be checked on the machine before cutting metal.
"""

import json
import math
import os
import sys
from dataclasses import dataclass

from build123d import (Align, Axis, Box, Compound, Cylinder, Plane, Polyline, Pos, Rot, Solid, Text, export_step,
                       export_stl, extrude, chamfer, fillet, make_face)

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
    circlip_land: float = 2.6        # material outside each plate groove (DIN 472-32 minimum edge margin)
    tension_slot: float = 3.0        # +- adjustment of the module along the centre line
    # --- built-in mains: an earthed, folded-aluminium MAINS TRAY under the clutch/motor module holds the 36 V supply
    # (Mean Well LRS-75-36: 36 V 2.1 A, 99 x 97 x 30, case 240A; terminals 1 L, 2 N, 3 FG, 4 -V, 5 +V along one 97 mm
    # end, toward the inlet; full load to 50 C ambient) and a snap-in IEC
    # C14 inlet (fuse + switch) clipped into the tray's own end face. All 230 V stays inside the earthed tray.
    psu_l: float = 99.0
    psu_w: float = 97.0
    psu_h: float = 30.0
    psu_w0: float = 82.0             # W of the PSU's plate-side end (the tray starts at the plate face, W 63)
    psu_top_gap: float = 3.0         # tray top under the clutch cover with the module at the bottom of its tension slot
    tray_t: float = 1.5              # 5052 / 1050 aluminium sheet, folded
    tray_side_gap: float = 2.0       # PSU side -> tray wall
    psu_holes: tuple = ((20.5, 45.5), (75.5, 45.5))   # base M3 (max 3 mm deep): from the terminal end, from one long edge
    term_zone_rise: float = 10.0     # tray walls, end face and lid are this much higher over the terminal/inlet zone
    inlet_cut: tuple = (47.5, 27.8)  # snap-in IEC C14 + fuse + rocker switch, rated for 1-2 mm panels (check yours)
    inlet_flange: tuple = (50.0, 31.0)
    inlet_depth: float = 35.0        # its body behind the panel
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
    # --- housing: ONE compact printed housing for the drive, the electronics and the controls (see below)
    panel_t: float = 3.0             # printed wall (PETG; ASA if the motor runs hot)
    rail: float = 10.0               # printed rails carrying M3 heat-set inserts
    lap: float = 8.0                 # the housing overlaps the plate edges by this much (screws into the edges)
    fit_gap: float = 0.25            # per side, housing cavity over the plate edges
    x_half: float = 55.0             # plate / housing inner half-width (x = -55 back ... +55 front)
    r_long: float = 12.0             # outer radius of the long (along-the-table) edges; inside = r_long - wall. The
                                     # top-front edge is a 45-degree chamfer of this size instead: it lies on the bed
                                     # when the hood prints, and a fillet there would droop
    r_end: float = 8.0               # 45-degree chamfer round the end wall (it is the shell's bed face)
    tail_clear: float = 19.0         # motor tail -> end wall: J1/J2/J5 plugs + ferrules + wires turning down past the board edge
    band_y: float = -26.0            # bottom of the hood's control strip (the strip runs up to the top)
    pcb_y: float = -33.0             # underside of the PCB, lying flat above the motor (clears it at tension -3)
    pcb_x0: float = -50.0            # world x of the PCB's x=0 edge
    pcb_w0: float = 129.0            # world W of the PCB's y=0 edge; the connector edge (y=100) faces the end wall


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
W_SHAFT_END = W0 - (clutch.Z_SHAFT0 + CP.out_shaft_len)
W_PULLEY = (W_BOSS_END + W_SHAFT_END) / 2           # small pulley centre plane
assert W_SHAFT_END > 1.0, "clutch output shaft would hit the table end: increase chamber"
assert W_BOSS_END > W_PULLEY + f.pulley_w / 2 - 0.1, "pulley overlaps the bearing boss"


def place_module(shape):
    """Clutch-local shape -> world: flip so the output points at the table, then move to the motor axis."""
    return Pos(MX, MY, W0) * Rot(180, 0, 0) * shape


# ------------------------------------------------------------------ parts
GUARD_R1 = GUARD_R1_
TRAY_TOP = MY - (CP.housing_od / 2 + CP.cover_wall) - f.tension_slot - f.psu_top_gap   # under the clutch cover
PSU_TOP = TRAY_TOP - 1.0
PSU_BOT = PSU_TOP - f.psu_h
TRAY_BOT = PSU_BOT - f.tray_t
PLATE_BOT = min(MY - (CP.housing_od / 2 + CP.cover_wall) - f.tension_slot - 22,           # room for the tension block
                TRAY_BOT - 0.5)                                                        # and the mains tray


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


CHAMFER_IN = 12.0 - 3.0 * (2 - math.sqrt(2))       # inside legs of the housing's top-front chamfer
PLATE_TOP = max(max(y for _, y in f.standoffs) + 14, GUARD_R1 + 2.0 + 4.0)   # covers the guard's open face


def plate():
    """Back wall of the whole unit: carries the stub bearings and the clutch module. Its four corners are rounded to
    the housing's inside radius so the housing can overlap its edges."""
    s = Pos(0, (PLATE_TOP + PLATE_BOT) / 2, Z_PLATE_IN) * Box(2 * f.x_half, PLATE_TOP - PLATE_BOT, f.plate_t, align=MIN)
    mid_y = (PLATE_TOP + PLATE_BOT) / 2
    corners = s.edges().filter_by(Axis.Z)
    s = fillet([e for e in corners if e.center().Y < mid_y], f.r_long - f.panel_t)
    s = chamfer([e for e in s.edges().filter_by(Axis.Z) if e.center().Y > mid_y and e.length > f.plate_t - 0.01],
                CHAMFER_IN + 0.3)                         # both top corners: under the housing's chamfered inside
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
    for xt in TRAY_BOLTS_X:                         # 2x M4 tapped 10 deep: the mains tray's flange (= its earth bond)
        s = s - Pos(xt, TRAY_FLANGE_Y, Z_PLATE_OUT - 10) * Cylinder(1.65, 10, align=MIN)
    for x in (-15.0, 15.0):  # tension block, 2x M5 tapped from the outer face
        s = s - Pos(MX + x, TB_Y, Z_PLATE_OUT - 12) * Cylinder(2.1, 12, align=MIN)
    return s


TB_Y = MY - (CP.housing_od / 2 + CP.cover_wall) - f.tension_slot - 10   # tension block centre (below the module)


def tension_block():
    """Steel block on the plate's outer face, below the module. An M4 drawbolt through it into a radial
    tapped hole in the cover's front wall pulls the module away from the leadscrew: belt tension."""
    b = Pos(MX, TB_Y, Z_PLATE_OUT) * Box(44, 14, 12, align=MIN)
    for x in (-15.0, 15.0):
        b = b - Pos(MX + x, TB_Y, Z_PLATE_OUT) * Cylinder(2.75, 12, align=MIN)
    b = b - Pos(MX, TB_Y, Z_PLATE_OUT + 4) * Rot(90, 0, 0) * Cylinder(2.25, 20)          # M4 drawbolt clearance
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


# ------------------------------------------------------------------ housing: everything in one compact box
# +X points at the operator: at the LEFT table end, Y (up) x Z (out of the table, i.e. to the operator's left) = toward
# the front of the mill. As the operator sees the front, W increases to the LEFT.
# Packing: the clutch module and the motor hang below the leadscrew; the encoder sits on the leadscrew axis next to
# the plate; the PCB lies FLAT above the motor (components up, connector edge toward the end wall); the controls sit
# in a strip along the upper front, their bodies reaching back over the PCB. Two prints (PETG, Prusa MK3S):
#   shell  back + bottom + end + lower front walls, PCB rails, insert rails; prints standing on its end wall
#   hood   top + control strip, one L-shaped piece; lift it off and the whole PCB is exposed from above
# All long edges are rounded (r_long), the end-wall edges too (r_end); the plate corners match the inside radius.
# Mains comes in through an IEC inlet in an earthed aluminium tray under the module (mains_tray), which also holds the
# 36 V supply; the limit-switch cable through one gland; USB-C, BOOT/RESET and SWD are on the end wall.
T = f.panel_t
Y0, Y1 = PLATE_BOT, PLATE_TOP
XB, XF = -f.x_half, f.x_half                        # inside faces of the back and front walls
X_FRONT, X_FP = XF, XF + T                          # control strip: inner and outer face
G = f.fit_gap
W_TAIL = W0 + 114.0                                 # motor tail (motor_stub flange + 104 body)
W_END = W_TAIL + f.tail_clear + T                   # outer face of the end wall
W_SHELL0 = Z_PLATE_OUT - f.lap                      # open rim, on the plate edges
RAIL = f.rail
INSERT_D, INSERT_L = 4.0, 6.0                       # M3 x 5.7 heat-set insert hole
R_IN = f.r_long - T                                 # inside radius of the long edges
R_BOTTOM = 8.5                                      # the two bottom long edges: tighter, so the mains tray fits the corners
W_END_SCREW = W_END - f.r_end - 4.0                 # end hood screws: countersunk heads fully on the flat, not the chamfer
TERM_TOP = TRAY_TOP + f.term_zone_rise     # tray top over the terminal/inlet zone (the motor is >= 7 mm above)
TERM_W0 = f.psu_w0 + f.psu_l - 6           # where the raised terminal/inlet zone starts (over the PSU's terminal end)
LID_FLANGE = 10.0                          # inward-folded wall flanges carrying the lid's M3 rivnuts
INLET_Y = (PSU_BOT + TERM_TOP) / 2         # IEC inlet centre, in the tray's end face
TRAY_HALF = f.psu_w / 2 + f.tray_side_gap + f.tray_t
TRAY_W1 = W_END - f.r_end - 1.0   # tray end face (inlet panel): clear of the shell's inside end chamfer; the inlet
                                  # sits recessed ~9 mm behind the end wall's opening (shielded from run-off coolant)
TRAY_WALL_W0 = Z_PLATE_OUT + 12.0                # side walls start past the belt-tension block
TRAY_BOLTS_X = (-45.0, -13.0)                 # flange bolts into the plate face: clear of the shell's plate-edge
                                              # screws (x -35/-22), the drawbolt (x 0) and its tool path
TRAY_FLANGE_Y = TRAY_BOT + 6.0
GROMMET = (-30.0, TRAY_W1 - 12.0)             # (x, W) of the Ø10 grommet in the tray lid: 36 V leads up to J1
USB_PORT = (15.0, 10.0)     # (x, y) on the end wall
SWD_PORT = (-15.0, 10.0)
BOOT_BTN = (-15.0, -9.0)    # 7 mm momentary buttons wired across the PCB's SW1 (BOOTSEL) and SW2 (RESET)
RESET_BTN = (15.0, -9.0)
INTAKE = dict(w0=145.0, w1=225.0, y0=-100.0, y1=-72.0)    # motor-air louvres, back and front walls
FAN = dict(w0=115.0, w1=155.0, y0=-12.0, y1=28.0, x_in=XB + 4.5)   # 40 mm fan / louvre field on the back wall
# service door in the floor, under the mains tray: the PSU's base screws and the belt drawbolt. It runs from the shell's
# rim (where 2x M3 hold it to the plate's bottom edge) to a pin hinge along x just past the PSU's far base screw, and
# swings down. The tray floor is only 0.75 mm above the printed floor, so the hinge knuckles hang below the floor.
DOOR_X = 25.0                              # door x -25..25 (the opening is 0.3 wider each side)
DOOR_HINGE_W = 172.0                       # hinge axis (along x)
DOOR_AXIS_Y = Y0 - T - 0.75                # 0.75 below the outer floor face: knuckle tops flush with the floor's inside
KNUCKLE_R = 3.5                            # Ø7 knuckles on a Ø3 x 50 steel pin
SHELL_KNUCKLES = ((-25.0, -17.0), (-4.0, 4.0), (17.0, 25.0))
DOOR_KNUCKLES = ((-16.5, -4.5), (4.5, 16.5))
DOOR_SCREWS_X = (-18.0, 18.0)              # into the plate's bottom edge, clear of the tray's M4 holes and the drawbolt


def bx(x0, x1, y0, y1, w0, w1):
    return Pos(x0, y0, w0) * Box(x1 - x0, y1 - y0, w1 - w0, align=(Align.MIN, Align.MIN, Align.MIN))


def xcyl(r, x0, x1, y, w):
    return Pos(x0, y, w) * Rot(0, 90, 0) * Cylinder(r, x1 - x0, align=MIN)


def ycyl(r, y0, y1, x, w):
    return Pos(x, y0, w) * Rot(-90, 0, 0) * Cylinder(r, y1 - y0, align=MIN)


def wcyl(r, w0, w1, x, y):
    return Pos(x, y, w0) * Cylinder(r, w1 - w0, align=MIN)


def pcb_to_world(px, py, h):
    """PCB coordinates (x right, y down, h up from the board's underside) -> feed-unit world. The board lies flat,
    components up; seen from above, PCB x runs toward the operator (+X) and PCB y toward the end wall (+W)."""
    return (f.pcb_x0 + px, f.pcb_y + h, f.pcb_w0 + py)


def pcb_box(px0, px1, py0, py1, h0, h1):
    a, b = pcb_to_world(px0, py0, h0), pcb_to_world(px1, py1, h1)
    return bx(min(a[0], b[0]), max(a[0], b[0]), min(a[1], b[1]), max(a[1], b[1]), min(a[2], b[2]), max(a[2], b[2]))


# PCB bodies from the KiCad layout (positions from the routed board, 7d76d3e); heights are assumptions.
# Connectors: J1 36 V, J2 motor, J3 SWD header, J4 USB-C, J5 clutch, J6 panel IDC, J7 encoder + limit switches.
PCB_HOLES = ((4, 4), (96, 4), (4, 96), (96, 96))


def pcb_placeholder():
    parts = [pcb_box(0, 100, 0, 100, 0, 1.6)]
    for cx, cy in ((15.5, 37.0), (19.0, 74.0)):                                   # 12.5 x 25 bulk caps
        x, y, w = pcb_to_world(cx, cy, 1.6)
        parts.append(ycyl(6.25, y, y + 25.0, x, w))
    parts.append(pcb_box(55.8, 73.7, 3.3, 25.0, 1.6, 3.0))                        # RP2040 U3, flash U4, crystal Y1
    for x0, x1, y0, y1 in ((32.5, 42.5, 0.8, 8.2), (77.0, 87.0, 4.3, 11.7)):       # RESET SW2, BOOTSEL SW1
        parts.append(pcb_box(x0, x1, y0, y1, 1.6, 5.1))
    parts.append(pcb_box(44.6, 52.5, 1.9, 4.5, -3.0, 10.1))                       # SWD header J3 (THT: pins below too)
    parts.append(pcb_box(83.1, 92.1, 92.9, 100.0, 1.6, 4.8))                      # USB-C J4, mouth at the edge
    for x0, x1 in ((12.0, 25.3), (26.5, 49.9), (50.9, 64.2)):                      # J1 J2 J5
        parts.append(pcb_box(x0, x1, 89, 100, 1.6, 13.6))
    parts.append(pcb_box(65.6, 72.4, 77, 98, 1.6, 10.1))                          # J7
    parts.append(pcb_box(73.0, 83.9, 66.4, 92.9, 1.6, 11.6))                      # J6
    parts.append(pcb_box(0.0, 11.0, 59, 87, 1.6, 14.6))                           # fuse holder
    for x0, x1 in ((12.0, 25.3), (26.5, 49.9), (50.9, 64.2)):                      # J1 J2 J5: MSTB plugs
        parts.append(pcb_box(x0, x1, 99.3, 99.3 + MSTB_PLUG, 1.6, 16.6))
    return parts


MSTB_PLUG = 13.0          # MSTB 2,5/..-ST-5,08 plug body beyond the header face (Phoenix drawing: ~12.5)
WIRE_RUN = 10.0           # ferrule (8) + 2 straight before the bend
WIRE_BEND = 7.0           # bend radius, 1.5 mm2 wire turning down past the board edge


def psu_placeholder():
    """LRS-75-36 body and the IEC inlet module's body behind the tray's end face (for the checks and renders)."""
    psu = bx(-f.psu_w / 2, f.psu_w / 2, PSU_BOT, PSU_TOP, f.psu_w0, f.psu_w0 + f.psu_l)
    iw, ih = f.inlet_cut
    inlet = bx(-iw / 2 + 1, iw / 2 - 1, INLET_Y - ih / 2 + 1, INLET_Y + ih / 2 - 1, TRAY_W1 - f.tray_t - f.inlet_depth,
               TRAY_W1 - f.tray_t)
    return psu, inlet


def psu_screws():
    """(x, W) of the PSU's two base screws (from Mean Well's drawing)."""
    w_term = f.psu_w0 + f.psu_l
    return [(-f.psu_w / 2 + e, w_term - d) for d, e in f.psu_holes]


def mains_tray():
    """Folded aluminium tray: floor + side walls + end face (inlet cut-out) + 2 tabs bent up against the plate face
    (2x M4: fixed AND earthed there), side walls and end face 10 mm higher over the terminal/inlet zone with inward
    folded lid flanges, and a separate lid over that zone. The PSU screws through the floor into its base holes. Only
    the 36 V leads leave, through a grommet in the lid. No vents in the floor."""
    t, hw = f.tray_t, TRAY_HALF
    w0 = Z_PLATE_OUT
    wz = TERM_W0                                      # start of the raised terminal/inlet zone
    floor = bx(-hw, hw, TRAY_BOT, TRAY_BOT + t, w0, TRAY_W1)
    walls = None
    for sx in (-1, 1):
        x0, x1 = (-hw, -hw + t) if sx < 0 else (hw - t, hw)
        wall = bx(x0, x1, TRAY_BOT, TRAY_TOP, TRAY_WALL_W0, wz) + bx(x0, x1, TRAY_BOT, TERM_TOP, wz, TRAY_W1)
        lx0, lx1 = (-hw, -hw + LID_FLANGE) if sx < 0 else (hw - LID_FLANGE, hw)   # folded inward at the top (lid screws)
        lip = bx(lx0, lx1, TERM_TOP - t, TERM_TOP, wz + t, TRAY_W1 - t)   # starts past the lid's folded-down edge
        walls = wall + lip if walls is None else walls + wall + lip
    end = bx(-hw, hw, TRAY_BOT, TERM_TOP, TRAY_W1 - t, TRAY_W1)
    tray = floor + walls + end
    for xt in TRAY_BOLTS_X:                          # two tabs bent up against the plate face
        tray = tray + bx(xt - 7, xt + 7, TRAY_BOT, TRAY_BOT + 13, w0, w0 + t)
        tray = tray - Pos(xt, TRAY_FLANGE_Y, w0 - 1) * Cylinder(2.2, t + 2, align=MIN)
    iw, ih = f.inlet_cut
    tray = tray - bx(-iw / 2, iw / 2, INLET_Y - ih / 2, INLET_Y + ih / 2, TRAY_W1 - t - 1, TRAY_W1 + 1)
    tray = tray - ycyl(6.0, TRAY_BOT - 1, TRAY_BOT + t + 1, MX, Z_PLATE_OUT + 4)    # drawbolt access (outside the walls)
    for xp, wp in psu_screws():                      # PSU base screws: M3 x 4 button head (ISO 7380) from below
        tray = tray - ycyl(1.7, TRAY_BOT - 1, TRAY_BOT + t + 1, xp, wp)
    for xs in (-hw + LID_FLANGE / 2, hw - LID_FLANGE / 2):   # lid screws: M3 into closed-end rivnuts in the flanges
        for ws in (wz + 8, TRAY_W1 - 8):
            tray = tray - ycyl(1.7, TERM_TOP - t - 1, TERM_TOP + 1, xs, ws)
    lid = bx(-hw, hw, TERM_TOP, TERM_TOP + t, wz, TRAY_W1)
    # its -W edge is folded down to just above the PSU: closes the step between the low and the raised part
    lid = lid + bx(-hw + t, hw - t, TRAY_TOP, TERM_TOP + t, wz, wz + t)
    lid = lid - ycyl(5.0, TERM_TOP - 1, TERM_TOP + t + 1, GROMMET[0], GROMMET[1])
    for xs in (-hw + LID_FLANGE / 2, hw - LID_FLANGE / 2):
        for ws in (wz + 8, TRAY_W1 - 8):
            lid = lid - ycyl(1.7, TERM_TOP - 1, TERM_TOP + t + 1, xs, ws)
    return tray, lid


def psu_screw_heads():
    """ISO 7380 M3 button heads (Ø5.7 x 1.65) under the tray floor, for the checks."""
    return [ycyl(2.85, TRAY_BOT - 1.65, TRAY_BOT, xp, wp) for xp, wp in psu_screws()]


def plug_wires():
    """Wire envelopes out of the J1/J2/J5 plugs: straight run toward the end wall, then a bend down past the board edge.
    Checked against the housing, so the plugs can be wired and unplugged in place."""
    out = []
    for x0, x1 in ((12.0, 25.3), (26.5, 49.9), (50.9, 64.2)):
        y_end = 99.3 + MSTB_PLUG
        out.append(pcb_box(x0 + 1, x1 - 1, y_end, y_end + WIRE_RUN + WIRE_BEND, 3.0, 12.0))
        out.append(pcb_box(x0 + 1, x1 - 1, y_end + WIRE_RUN, y_end + WIRE_RUN + WIRE_BEND, -40.0, 12.0))
    return out



# control strip layout, (W, y) on the strip's outer face; as the operator sees it, W increases to the LEFT
CTRL = {
    "estop": (214.0, 2.0), "toggle": (160.0, 18.0), "rapid": (160.0, -6.0),
    "oled": (85.0, 20.0), "dial": (88.0, -6.0), "adapter": (128.0, 20.0),
}
LABELS = (("LEFT", 175.0, 28.0), ("STOP", 160.0, 28.0), ("RIGHT", 145.0, 28.0),
          ("RAPID", 140.0, -6.0), ("SPEED", 113.0, -6.0))


def _rounded(x0, x1, y0, y1, w0, w1, r, c_tf, c_end, chamfer_back_top=False, r_bottom=None):
    """Box with its long edges filleted r, except the top-front one (and the top-back one if asked), which are 45-degree
    chamfers c_tf; the +W end edges chamfered c_end."""
    body = bx(x0, x1, y0, y1, w0, w1)
    xm, ym = (x0 + x1) / 2, (y0 + y1) / 2

    def is_ch(e):
        return e.center().Y > ym and (e.center().X > xm or chamfer_back_top)
    body = chamfer([e for e in body.edges().filter_by(Axis.Z) if is_ch(e)], c_tf)
    longs = [e for e in body.edges().filter_by(Axis.Z) if not is_ch(e) and e.length > (w1 - w0) - 0.01]
    rb = r if r_bottom is None else r_bottom
    tops = [e for e in longs if e.center().Y > ym]
    if tops:
        body = fillet(tops, r)
    body = fillet([e for e in body.edges().filter_by(Axis.Z) if e.center().Y < ym and e.length > (w1 - w0) - 0.01], rb)
    if c_end > 0:
        body = chamfer(body.edges().group_by(Axis.Z)[-1], c_end)
    return body


def outer_body():
    """The whole closed outer volume of the housing (before it is split into shell and hood). Every edge that lies on
    the print bed is a 45-degree chamfer (end wall for the shell, top-front edge for the hood); the rest are fillets."""
    out = _rounded(XB - T, X_FP, Y0 - T, Y1 + T, W_SHELL0, W_END, f.r_long, f.r_long, f.r_end, r_bottom=R_BOTTOM)
    cav = _rounded(XB - G, XF + G, Y0 - G, Y1 + G, W_SHELL0 - 1, W_END - T, R_IN + G, CHAMFER_IN, max(f.r_end - T, 1.0),
                   chamfer_back_top=True, r_bottom=R_BOTTOM - T + G)
    return out - cav


def hood_region():
    """Top slab + control strip, plus both long top corners (the hood's back edge ends on the back wall's tangent
    line at full wall thickness, not tapered)."""
    return bx(XB - T - 1, X_FP + 1, Y1, Y1 + T + 1, W_SHELL0 - 1, W_END + 1) + \
        bx(XF, X_FP + 1, f.band_y, Y1 + T + 1, W_SHELL0 - 1, W_END - T) + \
        bx(XF - CHAMFER_IN - 1, XF + G, Y1 - CHAMFER_IN - 1, Y1 + G, W_SHELL0 - 1, W_END - T) + \
        bx(XB - T - 1, XB + CHAMFER_IN, Y1 - CHAMFER_IN, Y1 + G, W_SHELL0 - 1, W_END - T)


def windows(body, x_wall, x_far, y0, y1):
    """Cut 12 mm tool-access windows over the PCB's mounting holes out of a rail (and its web). The window's low-W
    side, which is the underside of the rail segment above it while the shell prints standing on its end wall, rises
    at 45 degrees from the wall, so that segment prints without support."""
    d = abs(x_far - x_wall)
    for py in (4, 96):
        w = pcb_to_world(0, py, 0)[2]
        body = body - xw_prism([(x_wall, w - 6), (x_wall, w + 6), (x_far, w + 6), (x_far, w - 6 - d)], y0, y1)
    return body


def xy_prism(pts_xy, w0, w1):
    """Prism from a polygon in the (x, y) plane, extruded along W from w0 to w1."""
    face = make_face(Polyline(*[(x, y, w0) for x, y in pts_xy], close=True))
    return extrude(face, amount=w1 - w0, dir=(0, 0, 1))


def xw_prism(pts_xw, y0, y1):
    """Prism from a polygon in the (x, W) plane, extruded along y from y0 to y1."""
    face = make_face(Polyline(*[(x, y0, w) for x, w in pts_xw], close=True))
    return extrude(face, amount=y1 - y0, dir=(0, 1, 0))


def pcb_bosses():
    """4 bosses under the PCB's holes (heat-set inserts), open between them so air can rise past the board."""
    out = []
    _, yb, _ = pcb_to_world(0, 0, 0)
    for px, py in PCB_HOLES:
        x, _, w = pcb_to_world(px, py, 0)
        # joined to the side wall, with a 45-degree wedge on its +W side (that is "down" while the shell prints)
        wall, tip = (XB - G, x + 5) if x < 0 else (XF + G, x - 5)
        d = abs(tip - wall)
        out.append(xw_prism([(wall, w - 6), (tip, w - 6), (tip, w + 6), (wall, w + 6 + d)], yb - 8, yb))
    return out


def shell():
    """Back, bottom, end and lower-front walls, rails, PCB bosses, louvres: one print standing on the end wall."""
    s = outer_body() - hood_region()
    skin = _rounded(XB - T, X_FP, Y0 - T, Y1 + T, W_SHELL0, W_END, f.r_long, f.r_long, f.r_end, r_bottom=R_BOTTOM)  # trims rails
    rail_w = (Z_PLATE_OUT, W_END - T)
    # back-top rail (hood screws) on the flat part of the top, tied to the back wall by a 4 mm shelf under it (its
    # section lies in the bed plane while the shell prints, so any shape prints)
    xr = XB + CHAMFER_IN
    yr = Y1 - RAIL
    back = bx(xr, xr + RAIL, yr, Y1, *rail_w)
    back = back + xy_prism([(XB - G - 0.5, yr - 4), (XB - G - 0.5, Y1 - CHAMFER_IN), (xr, Y1 - CHAMFER_IN), (xr, yr),
                            (xr + RAIL, yr), (xr + RAIL, yr - 4)], *rail_w)
    back = windows(back, XB - G - 0.5, xr + RAIL + 0.5, yr - 5, Y1 + 1)
    # front rail (strip screws) behind the strip's lower edge, carried by a 45-degree web off the front wall below it
    xf0 = XF - RAIL
    front = bx(xf0, XF, f.band_y, f.band_y + RAIL, *rail_w)
    front = front + xy_prism([(XF + G + 0.5, f.band_y - (XF + G + 0.5 - xf0)), (XF + G + 0.5, f.band_y), (xf0, f.band_y)], *rail_w)
    front = windows(front, XF + G + 0.5, xf0 - 0.5, f.band_y - (XF + G + 0.5 - xf0) - 1, f.band_y + RAIL + 1)
    rails = [back, front,
             bx(XF - RAIL, XF + G + 0.5, f.band_y - RAIL, Y1 - CHAMFER_IN, W_END_SCREW - 5, W_END - T + 0.5),  # end-front
             bx(xr, XF - RAIL, Y1 - RAIL, Y1, W_END_SCREW - 5, W_END - T + 0.5)]                            # end-top
    for r in rails + pcb_bosses():
        s = s + (r & skin)
    s = s - hood_region()
    for px, py in PCB_HOLES:                                                              # PCB inserts
        x, y, w = pcb_to_world(px, py, 0)
        s = s - ycyl(INSERT_D / 2, y - INSERT_L, y + 0.1, x, w)
    # bottom: the service door's opening (the door carries the drawbolt access hole; no floor vents, the mains tray is
    # above the floor), the shell's hinge knuckles and notches for the door's
    s = s - bx(-DOOR_X - 0.3, DOOR_X + 0.3, Y0 - T - 1, Y0 - G + 0.01, W_SHELL0 - 1, DOOR_HINGE_W)
    for x0, x1 in DOOR_KNUCKLES:
        s = s - xcyl(KNUCKLE_R + 0.3, x0 - 0.5, x1 + 0.5, DOOR_AXIS_Y, DOOR_HINGE_W)
    for i, (x0, x1) in enumerate(SHELL_KNUCKLES):
        s = s + knuckle(x0, x1)
        # the steel pin presses into the two outer knuckles (Ø2.95) and turns in the rest (Ø3.2)
        s = s - xcyl(1.475 if i != 1 else 1.6, x0 - 0.01, x1 + 0.01, DOOR_AXIS_Y, DOOR_HINGE_W)
    # Motor air intake: slots along W in the back and front walls beside the motor (they face sideways, so falling
    # chips don't enter; in the print they run vertically),
    # louvred like the exhaust: each slot climbs 45 degrees inward through a 5 mm block, so chips and coolant spray
    # can't reach the mains tray below; the blocks have 45-degree undersides for printing
    for sx in (-1, 1):
        x_wall = XB - G if sx < 0 else XF + G
        x_in = x_wall - 5.0 * sx
        d = 5.0
        s = s + xw_prism([(x_wall, INTAKE["w0"]), (x_in, INTAKE["w0"]), (x_in, INTAKE["w1"]), (x_wall, INTAKE["w1"] + d)],
                         INTAKE["y0"], INTAKE["y1"])
        for k in range(4):
            y = INTAKE["y0"] + 5 + 6.0 * k
            slot = Pos(x_wall + 2.0 * sx, y, (INTAKE["w0"] + INTAKE["w1"]) / 2) * Rot(0, 0, -45 * sx) * Box(14, 2.4, INTAKE["w1"] - INTAKE["w0"] - 4)
            lim = bx(min(x_wall + 3.5 * sx, x_in) - 0.01, max(x_wall + 3.5 * sx, x_in) + 0.01, INTAKE["y0"] + 1, INTAKE["y1"] - 1,
                     INTAKE["w0"] + 2, INTAKE["w1"] - 2)
            s = s - (slot & lim)
    # back wall: louvred exhaust / 40 mm fan field, beside the encoder and the board's start, away from the fuse.
    # Each slot climbs 45 degrees inward through a 6 mm block, so a chip can't fall or bounce through. In the print
    # (W vertical) the slots lie in the horizontal plane: no overhang.
    fz = FAN
    d = fz["x_in"] - (XB - G)                           # block, with a 45-degree underside for printing
    s = s + xw_prism([(XB - G, fz["w0"]), (fz["x_in"], fz["w0"]), (fz["x_in"], fz["w1"]), (XB - G, fz["w1"] + d)],
                     fz["y0"], fz["y1"])
    for k in range(6):
        y = fz["y0"] + 4 + 5.5 * k
        slot = Pos(XB - 1.5, y, (fz["w0"] + fz["w1"]) / 2) * Rot(0, 0, 45) * Box(14, 2.4, fz["w1"] - fz["w0"] - 4)
        s = s - (slot & bx(XB - T - 1, fz["x_in"] + 0.01, fz["y0"] + 1, fz["y1"] - 1, fz["w0"] + 2, fz["w1"] - 2))
    for dy in (-16, 16):                                                                   # fan screws (self-tap M3)
        for dw in (-16, 16):
            s = s - xcyl(1.25, fz["x_in"] - 8, fz["x_in"] + 0.1, (fz["y0"] + fz["y1"]) / 2 + dy, (fz["w0"] + fz["w1"]) / 2 + dw)
    # end wall: M16 gland for the limit-switch cable, low beside the motor (its wires drop from J7 past the board's
    # edge, behind the motor tail); the locknut clears the inside chamfer
    s = s - wcyl(8.1, W_END - T - 1, W_END + 1, 36.0, -62.0)
    # mains: the inlet is clipped into the aluminium tray's end face; the printed end wall only has an opening round
    # its flange. Ribs under the tray's far end (the tray is bolted to the plate at its other end).
    fw, fh = f.inlet_flange
    s = s - bx(-fw / 2 - 1, fw / 2 + 1, INLET_Y - fh / 2 - 1, INLET_Y + fh / 2 + 1, W_END - T - 1, W_END + 1)
    for xr_ in (-30.0, 30.0):
        s = s + bx(xr_ - 1.5, xr_ + 1.5, Y0 - G - 0.5, TRAY_BOT - 0.3, TRAY_W1 - 60, TRAY_W1 - 6)
    for xd in (-15.0, 15.0):                         # drains: coolant that gets in behind the inlet runs out
        s = s - ycyl(2.0, Y0 - T - 8, Y0 + 6, xd, W_END - T - 2.5)
    # programming ports, usable with the box closed (above the plugs, clear of the end rails):
    #   USB-C: round nut-mount panel extension (e.g. Adafruit #6069, 12-18 mm hole), its male end left in J4
    #   SWD:   1x3 0.1" pin header pressed into a slot from outside, pins sunk in a recess; 3 wires inside to J3.
    #          A Raspberry Pi Debug Probe flashes and recovers the RP2040 through it whatever the firmware does.
    (ux, uy), (sx, sy) = USB_PORT, SWD_PORT
    s = s - wcyl(8.0, W_END - T - 1, W_END + 1, ux, uy)
    wi = W_END - T
    s = s + bx(sx - 8, sx + 8, sy - 5, sy + 5, wi - 8, wi + 0.5)                     # boss behind the port
    s = s - bx(sx - 6, sx + 6, sy - 3, sy + 3, W_END - 7, W_END + 1)                 # recess (pins 6 long: sunk)
    s = s - bx(sx - 3.95, sx + 3.95, sy - 1.4, sy + 1.4, wi - 9, W_END - 6.9)          # slot for the header body
    # BOOT and RESET: 7 mm panel pushbuttons, recessed 4 mm (Ø11 counterbore) so they can't be hit by accident;
    # a 5 mm boss inside leaves a 4 mm panel for the nut. Hold BOOT + tap RESET = RP2040 USB drive, box closed
    for bxx, byy in (BOOT_BTN, RESET_BTN):
        s = s + wcyl(8.0, wi - 5, wi + 0.5, bxx, byy)
        s = s - wcyl(5.5, W_END - 4, W_END + 1, bxx, byy) - wcyl(3.5, wi - 6, W_END + 1, bxx, byy)
    for txt, x, y in (("SWD", sx, sy + 9), ("USB", ux, uy + 12), ("BOOT", BOOT_BTN[0], BOOT_BTN[1] - 11),
                      ("RESET", RESET_BTN[0], RESET_BTN[1] - 11)):                      # engraved on the bed face
        try:
            s = s - extrude(Plane(origin=(x, y, W_END), x_dir=(1, 0, 0), z_dir=(0, 0, 1)) * Text(txt, 4.5), amount=-0.6)
        except Exception as e:  # noqa: BLE001 - a missing font must not stop the build
            print("label %s skipped: %s" % (txt, e))
    s = s - extrude(Plane(origin=(0, INLET_Y + f.inlet_cut[1] / 2 + 6, W_END), x_dir=(1, 0, 0), z_dir=(0, 0, 1))
                    * Text("230V~", 4.5), amount=-0.6)
    return s


def knuckle(x0, x1, r=KNUCKLE_R):
    """Shell hinge knuckle along x with a 45-degree drip toward +W: that side faces down while the shell prints on
    its end wall, so the part standing proud of the floor needs no support."""
    k = xcyl(r, x0, x1, DOOR_AXIS_Y, DOOR_HINGE_W)
    a, c = r / math.sqrt(2), r * math.sqrt(2)
    pts = [(DOOR_HINGE_W, DOOR_AXIS_Y), (DOOR_HINGE_W + a, DOOR_AXIS_Y - a), (DOOR_HINGE_W + c, DOOR_AXIS_Y),
           (DOOR_HINGE_W + a, DOOR_AXIS_Y + a)]
    face = make_face(Polyline(*[(x0, y, w) for w, y in pts], close=True))
    return k + extrude(face, amount=x1 - x0, dir=(1, 0, 0))


def door():
    """Service door in the floor: flush with it, hinged on its far edge (Ø3 steel pin), swings down; 2x M3 into the
    plate's bottom edge hold it shut. Drawbolt access hole, pockets for the PSU's base screw heads. One print, inner
    face down."""
    d = bx(-DOOR_X, DOOR_X, Y0 - T, Y0 - G, W_SHELL0, DOOR_HINGE_W - 0.3)
    for x0, x1 in SHELL_KNUCKLES:
        d = d - xcyl(KNUCKLE_R + 0.3, x0 - 0.5, x1 + 0.5, DOOR_AXIS_Y, DOOR_HINGE_W)
    for x0, x1 in DOOR_KNUCKLES:
        d = d + xcyl(KNUCKLE_R, x0, x1, DOOR_AXIS_Y, DOOR_HINGE_W)
    d = d - xcyl(1.6, -DOOR_X - 1, DOOR_X + 1, DOOR_AXIS_Y, DOOR_HINGE_W)
    d = d - ycyl(6.0, Y0 - T - 1, Y0 + 1, MX, Z_PLATE_OUT + 4)          # belt drawbolt, reachable with the door shut
    for xp, wp in psu_screws():                                       # the PSU's M3 button heads stand 0.9 into it
        d = d - ycyl(4.5, Y0 - G - 1.2, Y0 - G + 0.1, xp, wp)
    return d


def hood():
    """Top + control strip, one print (the strip face down on the bed, the top standing up)."""
    hd = outer_body() & hood_region()
    w, y = CTRL["dial"]  # nothing to thin: a 3 mm strip + the EC11's 2 mm nut fit its 6 mm bushing
    return control_cutouts(hd)


def control_cutouts(fp):
    w, y = CTRL["estop"]; fp = fp - xcyl(11.25, X_FRONT - 1, X_FP + 1, y, w)
    w, y = CTRL["toggle"]; fp = fp - xcyl(6.2, X_FRONT - 1, X_FP + 1, y, w)
    w, y = CTRL["rapid"]; fp = fp - xcyl(8.15, X_FRONT - 1, X_FP + 1, y, w)
    w, y = CTRL["dial"]; fp = fp - xcyl(3.7, X_FRONT - 1, X_FP + 1, y, w) - xcyl(1.6, X_FRONT - 1, X_FRONT + 1.0, y + 7.8, w)
    w, y = CTRL["oled"]; fp = fp - bx(X_FRONT - 1, X_FP + 1, y - 8, y + 8, w - 15, w + 15)
    for dw, dy in ((-15.2, -14.25), (15.2, -14.25), (-15.2, 14.25), (15.2, 14.25)):   # OLED M2 posts
        fp = fp + xcyl(2.5, X_FRONT - 2.5, XF + G + 0.5, y - 2 + dy, w + dw) - xcyl(0.8, X_FRONT - 2.6, X_FRONT + 1.0, y - 2 + dy, w + dw)
    w, y = CTRL["adapter"]
    for dw, dy in ((-19.5, -11), (19.5, -11), (-19.5, 11), (19.5, 11)):              # adapter board posts, M3 inserts
        fp = fp + xcyl(4.0, X_FRONT - 5, XF + G + 0.5, y + dy, w + dw) - xcyl(INSERT_D / 2, X_FRONT - 5.1, X_FRONT - 5 + INSERT_L, y + dy, w + dw)
    for txt, w, y in LABELS:                                                       # engraved 0.6 deep (3 layers)
        pl = Plane(origin=(X_FP, y, w), x_dir=(0, 0, -1), z_dir=(1, 0, 0))
        try:
            fp = fp - extrude(pl * Text(txt, 4.5), amount=-0.6)
        except Exception as e:  # noqa: BLE001 - a missing font must not stop the build
            print("label %s skipped: %s" % (txt, e))
    return fp


def controls():
    """Placeholder bodies for the checks and renders (not machined). Same order as CONTROL_NAMES."""
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
    w, y = CTRL["adapter"]
    c += [bx(X_FRONT - 6.6, X_FRONT - 5, y - 14, y + 14, w - 22.5, w + 22.5), bx(X_FRONT - 15.6, X_FRONT - 6.6, y - 5, y + 5, w - 18, w + 18)]
    return c


CONTROL_NAMES = ["estop_body", "estop_head", "estop_ring", "toggle_body", "toggle_lever", "rapid_body", "rapid_cap",
                 "dial_body", "dial_knob", "oled", "adapter_board", "adapter_header"]


def screw_joints():
    """(part screwed, target, outer point, inward unit vector, kind). 'tap' = M3 tapped 8 deep in the aluminium plate,
    'insert' = M3 heat-set insert in a printed rail. Countersunk M3 through the printed wall."""
    j = []
    wl = (W_SHELL0 + Z_PLATE_OUT) / 2                 # mid-lap: into the plate edges
    for x in (-30.0, 0.0, 30.0):
        j.append(("hood", "plate", (x, Y1 + T, wl), (0, -1, 0), "tap"))
    for x in (-35.0, 35.0):                          # beside the service door
        j.append(("shell", "plate", (x, Y0 - T, wl), (0, 1, 0), "tap"))
    for x in DOOR_SCREWS_X:                          # the door's two screws
        j.append(("door", "plate", (x, Y0 - T, wl), (0, 1, 0), "tap"))
    for y in (-110.0, -60.0, -10.0, 20.0):
        j.append(("shell", "plate", (XB - T, y, wl), (1, 0, 0), "tap"))
    for y in (-110.0, -60.0):
        j.append(("shell", "plate", (X_FP, y, wl), (-1, 0, 0), "tap"))
    for y in (0.0, 26.0):
        j.append(("hood", "plate", (X_FP, y, wl), (-1, 0, 0), "tap"))
    xr = XB + CHAMFER_IN + RAIL / 2
    for w in (100.0, 165.0, 192.0):                   # back-top rail, clear of the tool windows and their 45° tops
        j.append(("hood", "shell", (xr, Y1 + T, w), (0, -1, 0), "insert"))
    for x in (-20.0, 25.0):
        j.append(("hood", "shell", (x, Y1 + T, W_END_SCREW), (0, -1, 0), "insert"))
    for w in (110.0, 200.0):
        j.append(("hood", "shell", (X_FP, f.band_y + RAIL / 2, w), (-1, 0, 0), "insert"))
    for y in (-10.0, 20.0):
        j.append(("hood", "shell", (X_FP, y, W_END_SCREW), (-1, 0, 0), "insert"))
    return j


def housing():
    return {"shell": shell(), "hood": hood(), "door": door()}


def apply_screws(parts):
    for part_, target, p, d, kind in screw_joints():
        pl = Plane(origin=p, z_dir=d)
        parts[part_] = parts[part_] - Solid.make_cylinder(1.7, T + 0.01, pl) - Solid.make_cone(3.25, 1.7, 1.55, pl)
        inner = Plane(origin=tuple(p[i] + d[i] * T for i in range(3)), z_dir=d)
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
    # drive pins (+ washers, E-clips: they move with the armature) and return springs + bushes; children placed one by
    # one (booleans on a Compound moved as a whole see the children unmoved)
    springs, washers, clips = clutch.return_springs()
    module["clutch_pins"] = Compound(children=[place_module(x) for x in clutch.drive_pins() + washers + clips])
    module["clutch_springs"] = Compound(children=[place_module(x) for x in springs + clutch.bushes()])
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
    parts["plug_wires"] = Compound(children=plug_wires())
    parts["psu"], parts["mains_inlet"] = psu_placeholder()
    parts["mains_tray"], parts["mains_tray_lid"] = mains_tray()
    groups = {"estop": (0, 3), "toggle": (3, 5), "rapid": (5, 7), "dial": (7, 9), "oled": (9, 10), "adapter": (10, 12)}
    cs = controls()
    for nm, (i0, i1) in groups.items():                       # one STL per control, so the viewer can name them
        parts["control_" + nm] = Compound(children=cs[i0:i1])
    with open(os.path.join(OUT, "viewer_layout.json"), "w") as fh:  # the 3D viewer places the KiCad PCB from this
        json.dump({"pcb_x0": f.pcb_x0, "pcb_y": f.pcb_y, "pcb_w0": f.pcb_w0,
                   "door_axis_y": DOOR_AXIS_Y, "door_axis_w": DOOR_HINGE_W}, fh)
    for name, shape in parts.items():
        export_stl(shape, os.path.join(OUT, name + ".stl"))
        if not name.startswith(("mill_reference", "motor", "clutch_", "belt", "bearings", "module_bolt", "pcb_", "control", "plug_", "psu", "mains_inlet")):
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
| Plate | 6061, {f.plate_t:.0f} mm, CNC | **Ø{f.bearing_od:.0f} H7 straight through**: too big to swing on the lathe (300 mm needed), so bore it on the mill with a boring head. DIN 472-32 grooves (Ø33.7 × {f.circlip_groove}) leaving a **{f.circlip_land} land** at each face: keyseat cutter (head ≤ Ø28, 1.0–1.2 wide, two passes) by circular interpolation, one face, then flip for the other. Clutch-module hole slotted Ø{2 * (CP.bearing_od / 2 + 6):.0f} and 4× Ø5.5 slots (±{f.tension_slot:.0f}): 5×D deep, cut them from both faces. {len(f.standoffs)}× Ø6.6 for the standoffs; 3× M3 × 8 deep on r={f.enc_post_r:.0f} (encoder posts, outer face); 2× M5 × 12 for the tension block. Edge holes (M3 × 8 for the housing): stand the plate in the vise on its edge, or use a drill-guide block. Corners R{f.r_long - f.panel_t:.0f} |
| Standoffs | steel or 6061 | {len(f.standoffs)}× Ø{f.standoff_d:.0f} × {f.chamber:.0f}, Ø6.6 thru (M6 screws into the table end) |
| Leadscrew stub | steel, turned from **Ø20** stock | from the table side: tongue {f.stub_tongue_w} × {f.screw_d - 2:.0f} × {f.stub_tongue_h:.0f} (across the Oldham slot), Ø{f.stub_d:.0f} with a 0.6 flat for the 40T set screws, **Ø{f.stub_shoulder_d:.0f} × 2 shoulder** on the inner bearing's inner race (corner radius ≤ 0.3 so the bearing's chamfer clears it), Ø{f.stub_d:.0f} h6 at both bearings with **Loctite 641 on the inner one** (as on the clutch; easier than holding k6 and h6 bands on a manual lathe), DIN 471-15 groove (Ø14.3 × 1.1) just outboard of the inner bearing, Ø{f.stub_enc_d:.0f} × 10 encoder end |
| Bearing spacer tube | steel or 6061 | Ø{f.bearing_od - 0.05:.2f} / Ø28 × {bearing_zs()[1] - bearing_zs()[0] - f.bearing_w:.1f}, between the outer races. The inner bearing locates the stub (shoulder + DIN 471-15 either side of it); the outer one floats on the stub |
| Oldham disc | steel (1045) or bronze (CuSn12), greased — acetal creeps at the ~45 MPa of a stall/cam-out | Ø{f.oldham_d:.0f} × {f.oldham_t:.0f}: tongue {f.screw_d - 2:.0f} × {f.screw_end_slot_w - 0.2:.1f} on one face (into the leadscrew slot), cross slot {f.stub_tongue_w + 0.2:.1f} wide × {f.stub_tongue_h + 0.5:.1f} deep on the other. Takes up to ~1 mm offset between the plate and the leadscrew, and end float |
| Encoder disc | 6061, {f.enc_disc_t:.0f} mm | Ø{f.enc_disc_d:.0f} with a flat {f.enc_disc_flat:.0f} from centre toward the clutch module; Ø22 centre hole; 3× Ø3.4 on r={f.enc_post_r:.0f} (posts) and 3× Ø3.4 on r=15, 60° offset (encoder, **check your encoder's hole circle**) |
| Encoder posts | 3× M3 hex standoffs F-F, {f.enc_post_len:.0f} mm | |
| Tension block | steel | 44 × 14 × 12 on the plate's outer face below the module, 2× M5 to the plate; M4 drawbolt through it into a radial M4 tapped 7 deep in the cover's front wall (bottom, between two module bolts). Tighten the drawbolt to tension the belt, then lock the 4 module bolts |
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
- 4× M6 for the standoffs; 4× M5×35 + washers for the clutch module ({f.plate_t:.0f} plate + 8 wall: the tips stop clear of the hub); 2× M5×20 (tension block) + 1× M4×30 (drawbolt); 3× M3 hex standoffs {f.enc_post_len:.0f} mm and 6× M3×8.

## Notes
- The module hangs ≈{out_len - Z_PLATE_OUT:.0f} mm off the plate on 4 M5 into the cover. The cover's spigot over the clutch housing is
  {CP.housing_len - CP.cover_spigot_z:.0f} mm long with {CP.cover_screws} radial M4, so the motor is held stiffly. A strap from the motor's tail to the
  table end is optional if the mill vibrates a lot.
- The chamber ({f.chamber:.0f} mm) is set by the Oldham disc plus the belt plane: the 40T pulley's inner flange clears the stub tongue.

## Housing: the whole power feed in one compact box (3D printed)
The drive, the controller PCB, the controls and the 36 V supply share one rounded housing on the table end. Outside
it there is only the belt chamber (closed by the guard cup) and the limit switches. The housing carries no load
(the aluminium plate carries the bearings, the module and the belt), so it is printed.

Packing: the clutch and motor hang below the leadscrew, the encoder sits on the leadscrew next to the plate, the
**PCB lies flat above the motor** (components up, connector edge toward the end wall), and the controls sit in a
strip along the upper front with their bodies reaching back over the PCB.

| | |
|---|---|
| Outer size | {X_FP - (XB - T):.0f} (front-back) × {Y1 - Y0 + 2 * T:.0f} (high) × {W_END - W_SHELL0:.0f} mm (along the table); long edges R{f.r_long:.0f}, the top-front edge and the end wall chamfered 45° (they lie on the print bed); plus the {f.chamber:.0f} mm belt chamber and the controls proud of the front |
| `shell` (one print) | back, bottom, end and lower-front walls ({T:.0f} mm), the rails, the 4 PCB bosses, the louvre block. **Print standing on the end wall**, {W_END - W_SHELL0:.0f} mm tall (MK3S: 210). Every boss and block has a 45° underside, so **no supports**; only short bridges (≤ 12 mm: rail windows, louvre slots). PETG, 4 perimeters, 20 % gyroid |
| `door` (one print) | **service door** in the floor under the mains tray, {2 * DOOR_X:.0f} × {DOOR_HINGE_W - W_SHELL0:.0f} mm from the shell's rim to a pin hinge: it reaches the PSU's two base screws and the belt drawbolt (that hole is in the door, so belt tension is still set with it shut). Hinge: Ø3 × {2 * DOOR_X:.0f} steel pin (silver steel / drill blank) through 5 knuckles that hang 4.25 mm below the floor (the tray is only 0.75 above it, so they can't go inside); press the pin into the shell's two outer knuckles (Ø2.95), it turns in the rest. Shut with 2× M3×8 countersunk into tapped holes in the plate's bottom edge; it swings down 100° (checked). **Print inner face down**, no supports; the shell's knuckles have a 45° drip so they print standing |
| `hood` (one print) | top + control strip, one L-shaped piece. **Print with the strip face down**: the engraved labels come out crisp on the bed side; its back lip has a 45° inside so it prints without support. Lift it off and the whole PCB is exposed from above |
| Plate | aluminium, {2 * f.x_half:.0f} wide, bottom corners R{f.r_long - f.panel_t:.0f}, both top corners chamfered {CHAMFER_IN + 0.3:.1f} × 45° to sit under the housing's chamfered inside |
| Fixing | both prints overlap the plate edges by {f.lap:.0f} mm with {f.fit_gap} mm clearance per side, and screw into them ({sum(1 for j in screw_joints() if j[4] == "tap")}× M3×8 countersunk into tapped holes); the hood also takes {sum(1 for j in screw_joints() if j[4] == "insert")}× M3×8 into heat-set inserts (M3 × 5.7, Ø4.0 holes) in the shell's rails |
| PCB | on 4 printed bosses (heat-set inserts, M3×6), underside {f.pcb_y:.0f} mm below the leadscrew axis, ≥ 3 mm above the motor even with the motor raised 3 mm by the belt tensioner. The rails above it have 12 mm windows over all 4 holes, so a screwdriver and the insert iron go straight down (checked). Trim the SWD header's (J3) pins flush underneath. Air rises past the board between the bosses |
| Plugs | J1/J2/J5 MSTB plugs (≈13 mm) + ferrules: the wires run toward the end wall and turn down past the board's edge, behind the motor tail ({f.tail_clear:.0f} mm), checked with a plug + wire-bend envelope. Unplugging in place: slide the plug off, the wires have slack below |
| Controls (strip, left → right as you face it) | E-stop (Ø60 legend ring fits on the flat face), LEFT/STOP/RIGHT toggle over RAPID, SPEED dial with the OLED above it; adapter board (2×7 IDC) on the strip's inside, ~200 mm ribbon down to J6 so the hood can be lifted and set beside the unit |
| End wall | the mains inlet (recessed 9 mm, see Power), one M16 cable gland for the limit switches, low beside the motor. **Programming ports, usable with the box closed:** a Ø16 hole for a round nut-mount USB-C panel extension (e.g. Adafruit #6069; its male end stays in J4, plug body ≤ 12.4 × 6.6), **BOOT and RESET** buttons (7 mm momentary NO, recessed in Ø11 × 4 counterbores; solder 2 wires across the pads of SW1 / SW2 on the PCB, in parallel with them) and a recessed **SWD port** (1×3 0.1" header, SWCLK / GND / SWDIO like J3, pressed into the slot from outside with a drop of CA; solder 3 wires to its inner pins and run them to J3) |
| Vents | intake through louvre blocks on the back and front walls beside the motor (slots climb 45° inward, so chips and spray can't reach the mains tray); no floor vents; exhaust through a louvre block high on the back wall, beside the encoder (slots climb 45° inward through 7.5 mm, so chips can't fall in), away from the fuse. The block takes a 40 mm fan (4× M3 self-tapping, 32 mm square) on its inside |

- **Power, built in: the mains tray.** All 230 V lives in an earthed, folded **aluminium tray** under the clutch and
  motor ({2 * TRAY_HALF:.0f} wide, {TRAY_TOP - TRAY_BOT:.0f} high over the PSU and {TERM_TOP - TRAY_BOT:.0f} over the terminal/inlet zone, W {Z_PLATE_OUT:.0f}…{TRAY_W1:.0f}; 1.5 mm
  1050/5052 sheet: floor, two side walls with inward-folded lid flanges over the terminal zone, end face, and two tabs
  bent up against the plate). Fold drawing: `out/mains_tray.step` (+ `mains_tray_lid.step`). It holds:
  - a Mean Well **LRS-75-36** (36 V / 2.1 A, 99 × 97 × 30, case 240A), screwed through the tray floor into its two
    base holes: **M3 × 4 button head (ISO 7380)** — the case allows 3 mm of thread, the tray is 1.5 — reached through
    the service door; their heads sit in pockets in the door. Terminals 1 L, 2 N, 3 FG, 4 −V, 5 +V, toward the inlet;
  - a **snap-in IEC C14 inlet with fuse and rocker switch** clipped into the tray's end face (cut-out
    {f.inlet_cut[0]} × {f.inlet_cut[1]}; pick one rated for a 1–2 mm panel). It sits 9 mm behind the printed end wall's opening, so
    coolant running down the end wall doesn't reach it. Fit an IEC retaining clip (V-lock or a wire bail) so the cord
    can't half-unplug on a vibrating table;
  - a separate aluminium **lid** over the terminal/inlet zone, its plate-side edge folded down to just above the PSU
    (so the zone is closed all round): 4× M3 into **closed-end** M3 rivnuts in the walls' 10 mm folded flanges (no
    screw tips inside the mains zone); the screws also bond the lid to the tray. That folded edge ends 1 mm above the
    PSU, right over its terminal screws: on your PSU check ≥ 3 mm from the fold to the L/N screw heads, and line the
    inside of the fold (and the lid above the terminals) with 0.4 mm Nomex/Formex insulating sheet either way. A Ø10 grommet in it: the **only** thing that leaves the
    tray is the 36 V pair (1 mm², 300/500 V insulation since it runs inside the mains tray, to J1).
  - **Fixing and earth:** the tray's two tabs bolt to the plate's outer face (2× M4 into tapped holes; checked against
    every other hole in the plate): that holds it
    and earths it. PE: inlet ⏚ → an M4 stud on the tray (ring terminal, star washer) → PSU ⏚. Leave both earth
    leads longer than L/N so they pull out last. Don't anodise the plate or the tray, or scrape the bond faces.
    Through the plate the motor (4 M5), the clutch (cover + housing screws), the encoder posts, the stub and the mill
    table are all on the same earth.
  - **Wiring:** 0.75 mm² 300/500 V wire, insulated female spades on the inlet (fuse in **L**), crimp ferrules in the
    PSU. 35 mm of free wiring room between the PSU terminals and the inlet body (checked). Tie each mains wire next to
    its spade and its terminal, so one loose wire can't move. Wire the whole tray on the bench, lid on, before it
    goes in: the PSU terminals can't be reached with the motor above them.
  - **Fuse:** T 2 A ceramic (high breaking capacity) 5 × 20 for 230 V; T 3.15 A for 120 V.
  - **Power budget:** about 35–50 W at rapid traverse and about 60 W during the clutch pull-in (54 W for 50 ms with
    the motor at its 1 A hold current); 75 W covers normal use. Only a jam at rapid (motor at full torque, ~100 W) can
    trip the supply before stall detection acts. That is safe: if the LRS trips (overload, or the overvoltage latch,
    which needs a mains off/on to reset), the clutch springs release and the board shows `NO 36V`; nothing restarts
    on its own until the switch is centred.
  - **No floor vents** (the tray is above the floor). The motor breathes through louvred slots in the back and front
    walls beside it; the PSU convects up out of the open tray (its own perforated case is the cover there). Two Ø4
    drains in the floor behind the inlet let out any coolant that gets past the recess.
  - The cord moves with the table: give it a loop or a cable chain, a P-clip on the unit and strain relief at the
    stand. The rocker switch is the mains switch; the E-stop removes the drive enable and the clutch, not mains.
  - **Before first power-up (commissioning):** with the cord unplugged, measure from the cord's earth pin to the plate,
    the motor case, the clutch cover, the encoder body and the PSU case: each ≤ 0.1 Ω. Check L/N/PE continuity and
    that the fuse is in L. Only then plug in.
  - Your S-350-36 is no longer needed for this unit.
- **Programming with the box closed:**
  - **From your Mac, whatever the firmware does:** USB-C cable from the Mac to the end-wall USB-C. Hold **BOOT**, tap
    **RESET**, release BOOT: the RP2040 comes up as the `RPI-RP2` drive; copy `power-feed.uf2` onto it. (With the 36 V
    supply off it's even simpler: hold BOOT while plugging the cable in; USB powers the logic.)
  - **Same thing from the menu** when the firmware runs: settings (long-press the dial while stopped) → last item
    **USB FLASH** → hold.
  - **Debugging:** Raspberry Pi Debug Probe on the end-wall SWD port (its
    JST-to-0.1" cable), then `cargo run --release` in `firmware/feed-app` (probe-rs flashes, resets and shows logs).
    The 36 V supply must be on (the probe doesn't power the board), or USB-C power is enough for the logic.
- **Fuse:** lift the hood.
- **Heat:** the motor is inside and the PCB sits above it. Default run current 3 A (~16 W in the coils), 1 A at standstill.
  PETG's glass transition is ~80 °C; the walls are ≥ 5.8 mm from the motor. On the bench, run it closed for 30 min:
  if the motor case passes ~60 °C or the PSU case passes ~60 °C (the LRS gives full power only up to 50 °C ambient;
  it runs at about half load here), fit the 40 mm fan (exhaust), lower `MOTOR mA` in the menu (light feeds need far
  less), or reprint in ASA (needs an enclosure on the MK3S to print without warping).
- **Order:**
  1. Plate: bearings, stub, clutch/motor module, encoder.
  2. Mains tray on the bench: PSU screwed in, inlet clipped in, earth stud, L/N/PE wired and tied, 36 V pair out
     through the grommet, lid on. Bolt its flange to the plate (2× M4, star washers).
  3. Slide the shell over the module and the tray onto the plate edges (the door is off: the floor under the PSU screws
     is open, so nothing catches); screw it on. Heat-set inserts.
  4. PCB onto its bosses; plug the 36 V pair into J1, the motor, clutch and encoder leads behind the motor tail.
  5. Limit-switch cable through the gland; USB-C extension, BOOT/RESET and SWD wires to the PCB.
  6. Fan (optional; it covers the H1 hole's tool path, so after the PCB); hood with its controls last.
  7. Door: hold it in its knuckles, push the Ø3 pin through, close it, 2× M3 into the plate.
  8. Earth test (above) before the first plug-in.
  - **PSU screws or a look at the tray floor:** 2 screws, the door swings down. Nothing else comes off.
  - **Removal:** hood off; door open; unplug J1/J2/J5/J7 and the panel wires; PCB off; shell off; then the tray comes off the
    plate with its two bolts.
- **Possible next size cut:** the floor is set by the belt-tension block under the clutch; behind it (from W ≈ 130)
  the floor could step up ~35 mm, under the motor. Not done yet.

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
