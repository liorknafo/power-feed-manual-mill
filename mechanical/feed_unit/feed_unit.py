"""X-axis feed unit for the SIEG X3.5D: mount plate, 2:1 belt drive, leadscrew stub, encoder.

Run:  mechanical/.venv/bin/python mechanical/feed_unit/feed_unit.py
Out:  mechanical/feed_unit/out/*.step|*.stl; FEED_UNIT.md (parts, placeholders). Renders: tools/render_feed_unit.py.

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
sys.path.insert(0, os.path.join(HERE, ".."))
import bought as B  # noqa: E402
import clutch  # noqa: E402
import fasteners as FA  # noqa: E402

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
    # (Mean Well LRS-150-36: 36 V 4.3 A, 159 x 97 x 30, case 241A; terminals 1 L, 2 N, 3 FG, 4-5 -V, 6-7 +V along one
    # 97 mm end, toward the inlet; full load to 50 C ambient) and a snap-in IEC
    # C14 inlet (fuse + switch) clipped into the tray's own end face. All 230 V stays inside the earthed tray.
    psu_l: float = 159.0
    psu_w: float = 97.0
    psu_h: float = 30.0
    psu_w0: float = 82.0             # W of the PSU's plate-side end (the tray starts at the plate face, W 63; the
                                     # tension block's M5 heads stand to W 80)
    psu_top_gap: float = 3.0         # tray top under the clutch cover with the module at the bottom of its tension slot
    tray_t: float = 1.5              # 5052 / 1050 aluminium sheet, folded
    tray_side_gap: float = 2.0       # PSU side -> tray wall
    psu_holes: tuple = ((24.0, 65.0), (102.0, 65.0))  # base M3 (max 3 mm deep): from the terminal end, from the x=0 edge
    mains_wiring: float = 36.0       # free wiring room, PSU terminal end -> inlet body (the mains review's >= 35)
    term_zone_rise: float = 10.0     # tray walls, end face and lid are this much higher over the terminal/inlet zone
    inlet_cut: tuple = (47.2, 28.2)  # JOYNANO snap-in C14 + fuse + rocker for 0.8-1.5 mm panels (cut-out 47 x 28)
    inlet_flange: tuple = (50.0, 30.8)
    inlet_depth: float = 29.0        # behind the panel (32 overall - the 3 mm flange)
    # --- Oldham coupling (steel/bronze disc): tongue into the leadscrew slot, cross slot for the stub tongue
    oldham_d: float = 24.0           # outside the bracket, so not tied to the leadscrew diameter
    oldham_t: float = 9.0
    oldham_gap: float = 0.5          # each side: leadscrew end -> disc, disc -> stub end
    stub_tongue_w: float = 4.8
    stub_tongue_h: float = 4.0       # 14 x 4 bearing faces: ~45 MPa at 9 N·m cam-out -> steel or bronze disc
    # --- encoder (LPD3806: 3x M3 on r=15 in its front face) on an adapter disc
    enc_post_r: float = 23.0         # outside the Ø32 bearing bore
    enc_post_len: float = 30.0       # M3 x 30 hex standoffs: room for the 25 long coupling between the stub and the encoder
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
    r_end: float = 8.0               # 45-degree chamfer round the end wall (it is the end cap's bed face)
    tail_clear: float = 19.0         # motor tail -> end wall at least: J1/J2/J5 plugs + ferrules + wires turning down
                                     # past the board edge (the mains tray sets the length now: see W_END)
    cap_cut: float = 248.0           # W where the end cap starts: shell, hood and door end here (each prints on a MK3S)
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
    """M5 SHCS heads (Ø8.5 x 5) on Ø10 x 1 washers, on the plate's chamber face; for the tension-range check."""
    return [Pos(x, y, Z_PLATE_IN - 6.0) * Cylinder(5.0, 6.0, align=MIN) for x, y in module_bolts(t)]


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
    # the slot runs on down through the bottom edge: the module's bearing boss and output shaft slide up into it when
    # the module goes in from below (the tension block bridges the two legs afterwards)
    s = s - bx(MX - boss_r, MX + boss_r, PLATE_BOT - 1, MY, Z_PLATE_IN - 1, Z_PLATE_OUT + 1)
    for x, y in f.standoffs:   # M6 through, SHCS heads in Ø11 x 6.5 counterbores (flush: the low pair sits by the cover)
        s = s - Pos(x, y, Z_PLATE_IN) * Cylinder(3.3, f.plate_t, align=MIN)
        s = s - Pos(x, y, Z_PLATE_OUT - STANDOFF_CB) * Cylinder(5.5, STANDOFF_CB + 0.01, align=MIN)
    for k in range(3):  # encoder posts, M3 tapped 8 deep from the outer face
        a = math.radians(90 + 120 * k)
        s = s - Pos(f.enc_post_r * math.cos(a), f.enc_post_r * math.sin(a), Z_PLATE_OUT - 8) * Cylinder(1.25, 8, align=MIN)
    for xt in TRAY_TABS_X:      # the mains tray's floor tabs sit in 2 rebates in the bottom edge; M4 up into each, 10 deep
        s = s - tray_rebate(xt)  # (that is also the tray's earth bond)
        s = s - ycyl(1.65, PLATE_BOT + REBATE_H - 0.01, PLATE_BOT + REBATE_H + 10, xt, TRAY_SCREW_W)
    for x in TB_SCREWS_X:  # tension block, 2x M5 tapped from the outer face, one in each leg beside the slot
        s = s - Pos(MX + x, TB_Y, Z_PLATE_OUT - 12) * Cylinder(2.1, 12, align=MIN)
    return s


TB_Y = MY - (CP.housing_od / 2 + CP.cover_wall) - f.tension_slot - 10   # tension block centre (below the module)


def tray_rebate(xt):
    """2.2 deep pocket in the plate's bottom edge, open to the outer face, for one of the tray's floor tabs; R3 inner
    corners (a Ø6 end mill from the bottom edge)."""
    y0, y1, w_in = PLATE_BOT - 1, PLATE_BOT + REBATE_H, Z_PLATE_OUT - 13
    return (bx(xt - 7, xt + 7, y0, y1, w_in + 3, Z_PLATE_OUT + 1) + bx(xt - 4, xt + 4, y0, y1, w_in, w_in + 3) +
            ycyl(3, y0, y1, xt - 4, w_in + 3) + ycyl(3, y0, y1, xt + 4, w_in + 3))


KEY_TOP = MY - (CP.bearing_od / 2 + 5) - f.tension_slot - 0.5   # just under the clutch boss at its lowest tension


def tension_block():
    """6061 block on the plate's outer face, below the module, bridging the plate's slot, with a tongue that fills the
    slot from the plate's bottom edge up to just under the clutch boss: it closes the slot between the belt chamber
    and the box (chips, coolant) and ties the plate's two legs. An M4 drawbolt through it into a radial tapped hole in
    the cover's front wall pulls the module away from the leadscrew: belt tension. Goes on from below after the
    module."""
    b = Pos(MX, TB_Y, Z_PLATE_OUT) * Box(2 * TB_HALF, 14, 12, align=MIN)
    slot_half = CP.bearing_od / 2 + 6 - 0.1
    b = b + bx(MX - slot_half, MX + slot_half, PLATE_BOT, KEY_TOP, Z_PLATE_IN, Z_PLATE_OUT)
    for x in TB_SCREWS_X:
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
    """HTD 5M 40T (on the stub, Ø15 bore) and 20T (on the clutch shaft, Ø12 bore), both hubless (bought.py)."""
    big = Pos(0, 0, W_PULLEY) * B.htd5m_pulley(f.teeth_big, f.stub_d, 5)
    small = Pos(MX, MY, W_PULLEY) * B.htd5m_pulley(f.teeth_small, CP.out_shaft_d, 5)
    return big, small


def belt():
    """HTD 5M toothed belt: backing from the land (0.57 inside the pitch line) to the back (+1.17), teeth in the
    pulley grooves on the wraps and at 5 mm pitch on the spans (bought.py)."""
    r1, r2 = pd(f.teeth_big) / 2, pd(f.teeth_small) / 2
    z0 = W_PULLEY - f.belt_w / 2
    k = 1 / math.cos(math.pi / 144)                  # the inner hull's flats must not dip inside the pulley tips
    back = stadium(r1 + 1.17, r2 + 1.17, z0, f.belt_w) - stadium((r1 - 0.55) * k, (r2 - 0.55) * k, z0 - 1, f.belt_w + 2, n=144)
    teeth = B.htd5m_belt_teeth((0.0, 0.0), f.teeth_big, (MX, MY), f.teeth_small, f.belt_w, z0)
    return back + teeth                                # a Part (a bare Solid & X gives None when apart)


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
    """Aideepen 600P/R encoder (B.lpd3806). Screwed to an adapter disc (3x M3 on r=15), the disc sits on 3 posts
    at r=23 (outside the bearing bore); flexible coupling 6-8 between them. Posts: priaolker M3 x 30 F-F brass hex
    standoffs, 6 across flats (Amazon B09NDJFLG7), a flat toward the axis."""
    zd = Z_PLATE_OUT + f.enc_post_len
    z0 = zd + f.enc_disc_t
    body = Pos(0, 0, z0) * B.lpd3806()
    coupling = Pos(0, 0, Z_PLATE_OUT + 2) * Cylinder(9.5, 25.0, align=MIN)     # uxcell 6x8 beam coupling Ø19 x 25 (B0FN786WNM)
    hexagon = make_face(Polyline(*[(2 * 3.0 / math.sqrt(3) * math.cos(math.radians(60 * i + 30)),
                                    2 * 3.0 / math.sqrt(3) * math.sin(math.radians(60 * i + 30))) for i in range(6)],
                                  close=True))                                  # 6 AF, flats on +-x
    post = extrude(hexagon, amount=f.enc_post_len)
    posts = [Pos(f.enc_post_r * math.cos(math.radians(90 + 120 * k)), f.enc_post_r * math.sin(math.radians(90 + 120 * k)),
                 Z_PLATE_OUT) * Rot(0, 0, 90 + 120 * k) * post for k in range(3)]
    disc = Pos(0, 0, zd) * Cylinder(f.enc_disc_d / 2, f.enc_disc_t, align=MIN)
    disc = disc - Pos(0, -f.enc_disc_flat - 20, zd) * Box(80, 40, f.enc_disc_t, align=MIN)
    disc = disc - Pos(0, 0, zd) * Cylinder(11.0, f.enc_disc_t, align=MIN)                   # encoder pilot + coupling access
    for k in range(3):
        a = math.radians(90 + 120 * k)
        disc = disc - Pos(f.enc_post_r * math.cos(a), f.enc_post_r * math.sin(a), zd) * Cylinder(1.7, f.enc_disc_t, align=MIN)
        disc = disc - Pos(15 * math.cos(a + math.pi / 3), 15 * math.sin(a + math.pi / 3), zd) * Cylinder(1.7, f.enc_disc_t, align=MIN)
    return body, coupling, posts, disc


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
# in a strip along the upper front, their bodies reaching back over the PCB. Four prints (PETG, Prusa MK3S), the first
# three ending at W_CUT (the LRS-150 makes the box too long for one print):
#   shell    back + lower front walls, PCB rails, insert rails; prints standing on its cut face
#   hood     top + control strip, one L-shaped piece; lift it off and the whole PCB is exposed from above
#   door     the floor, hinged along the back bottom edge
#   end_cap  the last W_END - W_CUT of all of them with the end wall; slides on along -W last, screwed to the shell
#            through 3 blocks that reach into it, and holds the door's far end up on a 45-degree scarf
# All long edges are rounded (r_long), the end-wall edges too (r_end); the plate corners match the inside radius.
# Mains comes in through an IEC inlet in an earthed aluminium tray under the module (mains_tray), which also holds the
# 36 V supply; the limit-switch cable through one gland; USB-C, BOOT/RESET and SWD are on the end wall.
T = f.panel_t
Y0, Y1 = PLATE_BOT, PLATE_TOP
XB, XF = -f.x_half, f.x_half                        # inside faces of the back and front walls
X_FRONT, X_FP = XF, XF + T                          # control strip: inner and outer face
G = f.fit_gap
W_TAIL = W0 + 114.0                                 # motor tail (23HS45: 113 MAX behind the flange face, + 1)
# the mains tray sets the length: PSU, the free wiring room, the inlet body, the tray's end face, the end wall chamfer
TRAY_W1 = f.psu_w0 + f.psu_l + f.mains_wiring + f.inlet_depth + f.tray_t   # tray end face (inlet panel)
W_END = max(W_TAIL + f.tail_clear + T, TRAY_W1 + f.r_end + 1.0)          # outer face of the end wall
W_CUT = f.cap_cut                                   # shell / hood / door end here, the end cap starts
W_SHELL0 = Z_PLATE_OUT - f.lap                      # open rim, on the plate edges
RAIL = f.rail
INSERT_D, INSERT_L = 4.0, 6.0                       # M3 x 5 heat-set insert hole (Kadrick: Ø4.0, insert + 1 deep)
R_IN = f.r_long - T                                 # inside radius of the long edges
R_BOTTOM = 8.5                                      # the two bottom long edges: tighter, so the mains tray fits the corners
TERM_TOP = TRAY_TOP + f.term_zone_rise     # tray top over the terminal/inlet zone (the motor is >= 7 mm above)
TERM_W0 = f.psu_w0 + f.psu_l - 18          # where the raised terminal/inlet zone starts: over the whole terminal block
LID_FLANGE = 10.0                          # inward-folded wall flanges carrying the lid's M3 rivnuts
INLET_Y = (PSU_BOT + TERM_TOP) / 2         # IEC inlet centre, in the tray's end face
TRAY_HALF = f.psu_w / 2 + f.tray_side_gap + f.tray_t
# (TRAY_W1 above: clear of the end cap's inside end chamfer; the inlet sits recessed ~9 mm behind the end wall's
# opening, shielded from run-off coolant)
TRAY_WALL_W0 = Z_PLATE_OUT + 12.0                # side walls start past the belt-tension block
TRAY_TABS_X = (-34.0, 34.0)                   # the tray's 2 floor tabs, in rebates in the plate's bottom edge: beside
                                              # the plate's slot (x +-20), clear of the door screws (x +-44)
REBATE_H = 2.5                                # tab 1.5 thick, 0.5 above the plate's bottom edge, a 0.5 serrated
                                              # washer between the tab and the plate (the earth bond bites both)
STANDOFF_CB = 6.5                             # counterbore depth for the M6 standoff screws' heads
TRAY_TAB_W0 = Z_PLATE_OUT - 11.0
TRAY_SCREW_W = Z_PLATE_OUT - 6.0              # M4 x 8 button head (ISO 7380) up through each tab into the plate
GROMMET = (-30.0, TRAY_W1 - 12.0)             # (x, W) of the Ø10 grommet in the tray lid: 36 V leads up to J1
USB_PORT = (15.0, 13.0)     # (x, y) on the end wall: its Ø27.5 flange clears RESET's recess
SWD_PORT = (-15.0, 10.0)
BOOT_BTN = (-15.0, -11.0)   # 7 mm PBS-110 momentary buttons wired across the PCB's SW1 (BOOTSEL) and SW2 (RESET)
RESET_BTN = (15.0, -11.0)
BTN_RECESS = (8.25, 8.5)    # (radius, depth) of the recess in front of each: its nut (11.55 across corners), a 1/4"
                            # 10 mm socket (<= 16 OD) on it, and the cap,
                            # deep enough that the cap (8.2 above the floor) stays 0.3 below the wall face
BTN_FLOOR = 3.0             # the recess floor the button mounts in (PBS-110 thread: panel <= ~3.4)
INTAKE = dict(w0=145.0, w1=225.0, y0=-100.0, y1=-72.0)    # motor-air louvres, back and front walls
FAN = dict(w0=115.0, w1=155.0, y0=-12.0, y1=28.0, x_in=XB + 4.5)   # 40 mm fan / louvre field on the back wall
# the bottom door: everything below the bottom fillets' tops, up to the end cap, so the clutch/motor module and the
# mains tray go in and come out from below. Pin hinge along the BACK bottom edge, outside: open, the door hangs beside
# the box and the opening is clear. One screw on the front side into the plate's bottom edge shuts its plate end; the
# end cap's 45-degree scarf holds up its far end.
SPLIT_Y = Y0 - T + R_BOTTOM                # door / shell split on the back, front and end walls: top of the bottom fillets
HINGE_X = XB - T - 0.75                    # hinge axis (along W), 0.75 outside the back wall, on the split line
HINGE_Y = SPLIT_Y
KNUCKLE_R = 3.5                            # Ø7 knuckles on a Ø3 x 180 steel pin
SHELL_KNUCKLES = ((70.0, 100.0), (145.0, 175.0), (W_CUT - 30.0, W_CUT))   # W ranges (the last one ends on the shell's
DOOR_KNUCKLES = ((100.5, 144.5), (175.5, W_CUT - 30.5))                   # bed face)
DOOR_SCREWS_X = (44.0,)                    # front side, into the plate's bottom edge, outside the tray tab and the slot
SCARF_K = W_CUT - T - (Y0 - T)             # door / end cap joint below the split: the plane W - y = SCARF_K, at 45
                                           # degrees (the door's end leans over the cap's, which holds it up)
CAP_BLOCKS = ((-1, -60.0), (1, -60.0), (-1, 15.0))   # (side, y) of the end cap's blocks into the shell: M3 inserts,
CAP_BLOCK = (8.0, 14.0, 22.0)              # screwed from outside; (thickness from the wall, height, reach past W_CUT)
CAP_SCREW_W = W_CUT - CAP_BLOCK[2] + 6.0   # their screws, clear of the lap
LAP, LAP_T, LAP_G = 8.0, 1.2, 0.2          # the end cap's seam is lapped all round above the door: the hood's outer
                                           # 1.2 mm runs 8 mm on over a step in the cap's top, the cap's runs 8 mm on
                                           # over a step in the shell's walls (0.2 clear), so coolant can't get in
TB_HALF = 36.0                             # tension block 72 wide: its 2 M5 go into the plate's legs beside the slot
TB_SCREWS_X = (-30.0, 30.0)


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
    parts.append(pcb_box(72.0, 84.9, 64.4, 94.9, 11.6, 20.6))                     # its IDC ribbon plug
    parts.append(pcb_box(0.0, 11.0, 59, 87, 1.6, 14.6))                           # fuse holder
    for x0, x1 in ((12.0, 25.3), (26.5, 49.9), (50.9, 64.2)):                      # J1 J2 J5: MSTB plugs
        parts.append(pcb_box(x0, x1, 99.3, 99.3 + MSTB_PLUG, 1.6, 16.6))
    return parts


MSTB_PLUG = 13.0          # MSTB 2,5/..-ST-5,08 plug body beyond the header face (Phoenix drawing: ~12.5)
WIRE_RUN = 10.0           # ferrule (8) + 2 straight before the bend
WIRE_BEND = 7.0           # bend radius, 1.5 mm2 wire turning down past the board edge


PSU_LOC = Pos(-f.psu_w / 2, PSU_BOT, f.psu_w0 + f.psu_l) * Rot(-90, 0, 0)   # the LRS-150's frame in the unit


def psu_placeholder():
    """The LRS-150-36 (bought.py, terminal end toward the inlet, its 97 width across x) and the IEC inlet module clipped
    into the tray's end face (front face at TRAY_W1, flange toward the end wall)."""
    psu = PSU_LOC * B.lrs150()
    inlet = Pos(0, INLET_Y, TRAY_W1) * B.iec_inlet_snapin()
    return psu, inlet


def cord_plug():
    """A moulded C13 cord connector plugged into the inlet's C14 bay (to show how it sits in the end wall's opening)."""
    return Pos(-10.5, INLET_Y + 4.7, TRAY_W1 + 3.0) * B.c13_plug()


def psu_screws():
    """(x, W) of the PSU's two base screws (from Mean Well's drawing)."""
    w_term = f.psu_w0 + f.psu_l
    return [(-f.psu_w / 2 + e, w_term - d) for d, e in f.psu_holes]


def mains_tray():
    """Folded aluminium tray: floor + side walls + end face (inlet cut-out) + 2 flat floor tabs that sit in rebates in
    the plate's bottom edge (2x M4 up into the plate from below: fixed AND earthed there, so it goes in from below), side walls and end face 10 mm higher over the terminal/inlet zone with inward
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
    for xt in TRAY_TABS_X:                           # two floor tabs on under the plate, into its bottom-edge rebates
        tray = tray + bx(xt - 6, xt + 6, TRAY_BOT, TRAY_BOT + t, TRAY_TAB_W0, w0 + 0.01)
        tray = tray - ycyl(2.2, TRAY_BOT - 1, TRAY_BOT + t + 1, xt, TRAY_SCREW_W)
    iw, ih = f.inlet_cut
    tray = tray - bx(-iw / 2, iw / 2, INLET_Y - ih / 2, INLET_Y + ih / 2, TRAY_W1 - t - 1, TRAY_W1 + 1)
    tray = tray - ycyl(6.0, TRAY_BOT - 1, TRAY_BOT + t + 1, MX, Z_PLATE_OUT + 4)    # drawbolt access (outside the walls)
    for xp, wp in psu_screws():                      # PSU base screws: M3 x 4 button head (ISO 7380) from below
        tray = tray - ycyl(1.7, TRAY_BOT - 1, TRAY_BOT + t + 1, xp, wp)
    for xs, ws in lid_screws():                      # lid screws: M3 into open-end rivnuts (Ø5 holes) in the flanges
        tray = tray - ycyl(2.5, TERM_TOP - t - 1, TERM_TOP + 1, xs, ws)
    lid = bx(-hw, hw, TERM_TOP, TERM_TOP + t, wz, TRAY_W1)
    # its -W edge is folded down to just above the PSU: closes the step between the low and the raised part
    lid = lid + bx(-hw + t, hw - t, TRAY_TOP, TERM_TOP + t, wz, wz + t)
    lid = lid - ycyl(5.0, TERM_TOP - 1, TERM_TOP + t + 1, GROMMET[0], GROMMET[1])
    for xs, ws in lid_screws():                      # Ø8.5: clears the rivnuts' Ø8 x 0.8 flanges, so the lid lies flat
        lid = lid - ycyl(4.25, TERM_TOP - 1, TERM_TOP + t + 1, xs, ws)
    return tray, lid


def lid_screws():
    """(x, W) of the tray lid's 4 M3 screws, in the walls' folded flanges."""
    hw = TRAY_HALF
    return [(xs, ws) for xs in (-hw + LID_FLANGE / 2, hw - LID_FLANGE / 2) for ws in (TERM_W0 + 8, TRAY_W1 - 8)]


def psu_screw_heads():
    """ISO 7380 M3 button heads (Ø5.7 x 1.65) under the tray floor, for the checks."""
    return [ycyl(2.85, TRAY_BOT - 1.65, TRAY_BOT, xp, wp) for xp, wp in psu_screws()]


def usb_envelopes():
    """The USB-C path to J4, as space envelopes for the checks and renders. J4's mouth is only 29 mm from the end wall
    and the extension's straight male end is long (not on the listing: 38 allowed, overmold 30 + strain relief 8), so
    it can't plug in straight: a 90-degree USB-C adapter (rgzhihuifz up/down, Amazon B08P5HRH3Z: 25.6 x 14.1; male into
    J4, female facing up; USB-C is reversible) takes it from above. Plus the panel end's cable turning away behind its
    strain relief (the panel end itself is in end_wall_parts)."""
    ux, uy = USB_PORT
    return [pcb_box(81.4, 93.9, 100.6, 114.6, 1.6, 21.6),        # 90-degree adapter (25.6 x 14.1 x ~6)
            pcb_box(81.6, 93.7, 104.0, 112.0, 21.6, 59.6),       # the straight male plug, going down into it
            pcb_box(78.0, 97.0, 100.0, 115.0, 59.6, 72.0),       # its cable bending away at the top
            wcyl(7.0, W_END - 34.5 - 14.0, W_END - 34.5, ux, uy)]   # panel end's cable turning away (Ø4.5, R ~10)


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
    "estop": (214.0, 2.0), "toggle": (160.0, 18.0), "rapid": (160.0, -4.0),
    "oled": (85.0, 20.0), "dial": (85.0, -6.0), "adapter": (124.5, 20.0),
}
LABELS = (("LEFT", 175.0, 28.0), ("STOP", 160.0, 28.0), ("RIGHT", 145.0, 28.0),
          ("RAPID", 140.0, -4.0), ("SPEED", 113.0, -6.0))
STRIP_T = X_FP - X_FRONT - G     # the control strip's real thickness (its back is G off the shell): the panel parts' panel
DIAL_RIB = (6.4, 7.6, 3.0)       # PEC11R anti-rotation ribs either side of its 12.5 body: (inner, outer) offset, depth
ESTOP_PLATE_T = 1.0              # the printed yellow legend plate under the E-stop's collar


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


def _skin(d):
    """The outer body's solid, inset by d (no end chamfer, a little longer than the box): for the seam laps."""
    return _rounded(XB - T + d, X_FP - d, Y0 - T + d, Y1 + T - d, W_SHELL0 - 2, W_END + 2, f.r_long - d,
                    f.r_long - d * (2 - math.sqrt(2)), 0, r_bottom=R_BOTTOM - d)


def lap_layer(d):
    """The outer d of the walls."""
    return _skin(0.0) - _skin(d)


def side_region():
    """Shell walls above the door (and the hinge knuckles), outside the hood's region: the cap laps over these."""
    return bx(XB - T - 5, X_FP + 5, SPLIT_Y + KNUCKLE_R + 2, Y1 + T + 5, W_SHELL0 - 2, W_END + 2) - hood_region()


def hood_region():
    """Top slab + control strip, plus both long top corners (the hood's back edge ends on the back wall's tangent
    line at full wall thickness, not tapered)."""
    return bx(XB - T - 1, X_FP + 1, Y1, Y1 + T + 1, W_SHELL0 - 1, W_END + 1) + \
        bx(XF, X_FP + 1, f.band_y, Y1 + T + 1, W_SHELL0 - 1, W_END - T) + \
        bx(XF - CHAMFER_IN - 1, XF + G, Y1 - CHAMFER_IN - 1, Y1 + G, W_SHELL0 - 1, W_END - T) + \
        bx(XB - T - 1, XB + CHAMFER_IN, Y1 - CHAMFER_IN, Y1 + G, W_SHELL0 - 1, W_END - T)


def windows(body, x_wall, x_far, y0, y1):
    """Cut 12 mm tool-access windows over the PCB's mounting holes out of a rail (and its web). The window's low-W
    side, which is the underside of the rail segment above it while the shell prints standing on its cut face, rises
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


def yw_prism(pts_yw, x0, x1):
    """Prism from a polygon in the (y, W) plane, extruded along x from x0 to x1."""
    face = make_face(Polyline(*[(x0, y, w) for y, w in pts_yw], close=True))
    return extrude(face, amount=x1 - x0, dir=(1, 0, 0))


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
    """Back and lower-front walls, rails, PCB bosses, louvres, up to W_CUT: one print standing on its cut face."""
    s = outer_body() - hood_region()
    skin = _rounded(XB - T, X_FP, Y0 - T, Y1 + T, W_SHELL0, W_END, f.r_long, f.r_long, f.r_end, r_bottom=R_BOTTOM)  # trims rails
    rail_w = (Z_PLATE_OUT, W_CUT)
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
    for r in [back, front] + pcb_bosses():
        s = s + (r & skin)
    s = s - hood_region()
    for px, py in PCB_HOLES:                                                              # PCB inserts
        x, y, w = pcb_to_world(px, py, 0)
        s = s - ycyl(INSERT_D / 2, y - INSERT_L, y + 0.1, x, w)
    # bottom: the floor is the door (rounded bottom edges included); the shell carries the hinge knuckles and notches
    # round the door's
    s = s - door_region()
    for w0, w1 in DOOR_KNUCKLES:
        s = s - wcyl(KNUCKLE_R + 0.3, w0 - 0.5, w1 + 0.5, HINGE_X, HINGE_Y)
    for i, (w0, w1) in enumerate(SHELL_KNUCKLES):     # vertical while the shell prints: plain cylinders
        s = s + wcyl(KNUCKLE_R, w0, w1, HINGE_X, HINGE_Y)
        # the steel pin goes in from the end-wall end, slides through every knuckle (Ø3.2) and presses only its last
        # 30 mm into the plate-end knuckle (Ø2.95): a press fit at both ends would mean forcing it 150 mm through one
        s = s - wcyl(1.475 if i == 0 else 1.6, w0 - 0.01, w1 + 0.01, HINGE_X, HINGE_Y)
    s = s - pin_paths()
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
    # brace across the cut face: the back wall and the lower front wall meet nowhere else (the end wall is on the end
    # cap), so a 5 x 3 bar on the print bed ties them, under the PCB's plug wires and over the motor's tail
    s = s + bx(XB - G - 0.5, XF + G + 0.5, -45.0, -40.0, W_CUT - 3.0, W_CUT)
    # the end cap's lap: the walls' outer LAP_T + LAP_G stepped down over the last LAP + LAP_G, the step at ~45 degrees
    # (3 stairs) so it prints standing on the cut face
    dep = LAP_T + LAP_G
    for k in range(3):
        s = s - (lap_layer(dep * (3 - k) / 3) & side_region() &
                 bx(XB - T - 5, X_FP + 5, Y0 - T - 5, Y1 + T + 5, W_CUT - LAP - LAP_G - dep * k / 3, W_CUT + 1))
    return s & bx(XB - T - 5, X_FP + 5, Y0 - T - 10, Y1 + T + 10, W_SHELL0 - 1, W_CUT)


def end_wall(s):
    """The end wall's openings, bosses and labels (on the end cap)."""
    wi = W_END - T
    # end wall: M16 gland for the limit-switch cable, low beside the motor (its wires drop from J7 past the board's
    # edge, behind the motor tail); the locknut clears the inside chamfer
    s = s - wcyl(8.1, W_END - T - 1, W_END + 1, 36.0, -62.0)
    # mains: the inlet is clipped into the aluminium tray's end face; the printed end wall only has an opening round
    # its flange (down to just above the cap's own bottom band, which closes it there)
    fw, fh = f.inlet_flange
    s = s - bx(-fw / 2 - 1, fw / 2 + 1, SPLIT_Y - 1, INLET_Y + fh / 2 + 1, W_END - T - 1, W_END + 1)
    # programming ports, usable with the box closed (above the plugs):
    #   USB-C: round threaded panel-mount extension (QIANRENON, M21: Ø27.5 flange outside, nut and body inside); its
    #          straight male end goes into a 90-degree adapter in J4 (J4 is only 29 mm from the wall)
    #   SWD:   1x3 0.1" pin header pressed into a slot from outside, pins sunk in a recess; 3 wires inside to J3.
    #          A Raspberry Pi Debug Probe flashes and recovers the RP2040 through it whatever the firmware does.
    (ux, uy), (sx, sy) = USB_PORT, SWD_PORT
    s = s - wcyl(10.8, W_END - T - 1, W_END + 1, ux, uy)                          # Ø21.6 for the M21 thread
    s = s + bx(sx - 8, sx + 8, sy - 5, sy + 5, wi - 8, wi + 0.5)                     # boss behind the port
    s = s - bx(sx - 6, sx + 6, sy - 3, sy + 3, W_END - 7, W_END + 1)                 # recess (pins 6 long: sunk)
    s = s - bx(sx - 3.95, sx + 3.95, sy - 1.4, sy + 1.4, wi - 9, W_END - 6.9)          # slot for the header body
    # BOOT and RESET: PBS-110 panel pushbuttons in recesses (BTN_RECESS) so they can't be hit by accident;
    # hold BOOT + tap RESET = RP2040 USB drive, box closed
    for bxx, byy in (BOOT_BTN, RESET_BTN):   # PBS-110 mounts from behind, nut in front: a boss carries a 3 mm floor
        floor = W_END - BTN_RECESS[1] - BTN_FLOOR
        s = s + wcyl(9.6, floor, wi + 0.5, bxx, byy)
        s = s - wcyl(BTN_RECESS[0], W_END - BTN_RECESS[1], W_END + 1, bxx, byy) - wcyl(3.6, floor - 1, W_END + 1, bxx, byy)
    for txt, x, y in (("SWD", sx, sy + 9), ("USB", ux, uy + 17), ("BOOT", BOOT_BTN[0], BOOT_BTN[1] - 11),
                      ("RESET", RESET_BTN[0], RESET_BTN[1] - 11)):                      # engraved on the bed face
        try:
            s = s - extrude(Plane(origin=(x, y, W_END), x_dir=(1, 0, 0), z_dir=(0, 0, 1)) * Text(txt, 4.5), amount=-0.6)
        except Exception as e:  # noqa: BLE001 - a missing font must not stop the build
            print("label %s skipped: %s" % (txt, e))
    s = s - extrude(Plane(origin=(0, INLET_Y + f.inlet_cut[1] / 2 + 6, W_END), x_dir=(1, 0, 0), z_dir=(0, 0, 1))
                    * Text("230V~", 4.5), amount=-0.6)
    return s


def end_cap():
    """The last W_END - W_CUT of the box: end wall, back/front/top walls, control-strip end and bottom band, one print
    standing on the end wall. It goes on last, sliding along -W: its floor under the mains tray's far end (2 ribs carry
    that end), its bottom band's 45-degree scarf under the door's far end (holding it up: take the cap off before
    opening the door). 3 blocks reach W_CUT - CAP_BLOCK[2] into the shell along its back and front walls; an M3
    countersunk screw goes through the shell wall into a heat-set insert in each. The blocks' +W ends are 45-degree
    wedges off the wall (that is "down" while it prints)."""
    upper = bx(XB - T - 5, X_FP + 5, SPLIT_Y, Y1 + T + 5, W_CUT + 0.2, W_END + 1)
    ylo = Y0 - T - 5
    lower = yw_prism([(ylo, ylo + SCARF_K + 0.3), (ylo, W_END + 1), (SPLIT_Y, W_END + 1),
                      (SPLIT_Y, SPLIT_Y + SCARF_K + 0.3)], XB - T - 5, X_FP + 5)
    c = outer_body() & (upper + lower)
    y_r0, y_r1 = Y0 - G - 0.5, TRAY_BOT - 0.3
    for xr_ in (-30.0, 30.0):                           # ribs under the tray's far end (it is bolted at the plate end),
        c = c + yw_prism([(y_r0, W_CUT + 0.5), (y_r1 - 3.0, W_CUT + 0.5), (y_r1, W_CUT + 3.5), (y_r1, TRAY_W1 - 6),
                          (y_r0, TRAY_W1 - 6)], xr_ - 1.5, xr_ + 1.5)     # with a 45-degree lead-in as it slides on
    # the seam's laps: over the shell's stepped walls, and stepped itself under the hood's
    c = c + (lap_layer(LAP_T) & side_region() & bx(XB - T - 5, X_FP + 5, Y0 - T - 5, Y1 + T + 5, W_CUT - LAP, W_CUT + 0.3))
    c = c - (lap_layer(LAP_T + LAP_G) & hood_region() & bx(XB - T - 5, X_FP + 5, Y0 - T - 5, Y1 + T + 5, W_CUT - 1,
                                                          W_CUT + LAP + LAP_G))
    tb, hb, reach = CAP_BLOCK
    for side, yc in CAP_BLOCKS:
        wall = XB - G if side < 0 else XF + G           # the walls' inside face
        x_e, tip = wall + side * 0.5, wall - side * (tb + 0.25)
        d = abs(tip - x_e)
        blk = xw_prism([(x_e, W_CUT - reach), (tip, W_CUT - reach), (tip, W_CUT + 2), (x_e, W_CUT + 2 + d)],
                       yc - hb / 2, yc + hb / 2)
        x0, x1 = (wall - 5, wall + 0.3) if side < 0 else (wall - 0.3, wall + 5)   # 0.3 off the shell's wall where
        c = c + (blk - bx(x0, x1, yc - hb, yc + hb, W_CUT - reach - 1, W_CUT + 0.2))  # it reaches into the shell
    for xd in (-15.0, 15.0):                         # drains: coolant that gets in behind the inlet runs out
        c = c - ycyl(2.0, Y0 - T - 8, Y0 + 6, xd, W_END - T - 2.5)
    return end_wall(c) - pin_paths()


def door_knuckle(w0, w1, r=KNUCKLE_R):
    """Door hinge knuckle along W with a 45-degree drip toward -y: that side faces down while the door prints."""
    a, c = r / math.sqrt(2), r * math.sqrt(2)
    drip = [(HINGE_X, HINGE_Y), (HINGE_X - a, HINGE_Y - a), (HINGE_X, HINGE_Y - c), (HINGE_X + a, HINGE_Y - a)]
    return wcyl(r, w0, w1, HINGE_X, HINGE_Y) + xy_prism(drip[::-1], w0, w1)


def pin_paths():
    """The hinge pin's way in (end-wall end) and the punch's way in to push it out (plate end): the axis is only 0.75
    outside the back wall, so a Ø3.2 groove, 0.75 deep, runs along the back bottom edge beyond the end knuckles."""
    return wcyl(1.6, SHELL_KNUCKLES[-1][1] - 0.01, W_END + 1, HINGE_X, HINGE_Y) + \
        wcyl(1.6, W_SHELL0 - 1, SHELL_KNUCKLES[0][0] + 0.01, HINGE_X, HINGE_Y)


def door_region():
    """Everything below the split, the whole length of the shell."""
    return bx(XB - T - 1, X_FP + 1, Y0 - T - 6, SPLIT_Y, W_SHELL0 - 1, W_END + 1)


def door():
    """The bottom door: the floor with both rounded bottom edges up to the end cap, flush with the shell. Hinged along
    its back edge (Ø3 steel pin), it swings down and hangs beside the box; a screw on the front side into the plate shuts
    its plate end, the end cap's scarf holds up its far end. The drawbolt access hole, pockets for the PSU's and the
    tray tabs' screw heads. One print, outer face down: a 45-degree foot under each rounded edge, a 45-degree drip under
    each knuckle, and its far end leans out at 45 degrees (the scarf), so no supports."""
    d = outer_body() & bx(XB - T - 1, X_FP + 1, Y0 - T - 1, SPLIT_Y - 0.3, W_SHELL0, W_END + 1)
    for sx in (-1, 1):                                  # 45-degree feet: fill under the fillet's lower, flatter part
        cx = sx * (X_FP - R_BOTTOM)
        a = R_BOTTOM / math.sqrt(2)
        foot = [(cx, Y0 - T), (cx + sx * (a - (R_BOTTOM - a)), Y0 - T), (cx + sx * a, Y0 - T + R_BOTTOM - a),
                (cx, Y0 - T + R_BOTTOM - a)]
        d = d + xy_prism(foot if sx > 0 else foot[::-1], W_SHELL0, W_END - f.r_end)
    for w0, w1 in SHELL_KNUCKLES:
        d = d - wcyl(KNUCKLE_R + 0.3, w0 - 0.5, w1 + 0.5, HINGE_X, HINGE_Y)
    for w0, w1 in DOOR_KNUCKLES:
        d = d + door_knuckle(w0, w1)
        # 1.2 mm web from the knuckle's drip down to the bed: the knuckle stands outside the rounded edge, so without it
        # the knuckle would start printing in mid-air (trim it off after printing)
        d = d + bx(HINGE_X - 0.6, HINGE_X + 0.6, Y0 - T, HINGE_Y - KNUCKLE_R * math.sqrt(2) + 0.3, w0, w1)
    d = d - wcyl(1.6, SHELL_KNUCKLES[0][0] - 1, SHELL_KNUCKLES[-1][1] + 1, HINGE_X, HINGE_Y) - pin_paths()
    d = d - ycyl(6.0, Y0 - T - 1, Y0 + 1, MX, Z_PLATE_OUT + 4)          # belt drawbolt, reachable with the door shut
    for xp, wp in psu_screws():                                       # the PSU's M3 button heads stand 0.9 into it
        d = d - ycyl(4.5, Y0 - G - 1.2, Y0 - G + 0.1, xp, wp)
    for xt in TRAY_TABS_X:                                            # the tray tabs' M4 button heads, 1.45 into it
        d = d - ycyl(5.0, Y0 - G - 1.75, Y0 - G + 0.1, xt, TRAY_SCREW_W)
    ylo, yhi = Y0 - T - 10, SPLIT_Y + 10            # up to the scarf: W - y <= SCARF_K
    return d & yw_prism([(ylo, W_SHELL0 - 2), (ylo, ylo + SCARF_K), (yhi, yhi + SCARF_K), (yhi, W_SHELL0 - 2)],
                        XB - T - 10, X_FP + 10)


def hood():
    """Top + control strip up to W_CUT, one print (the strip face down on the bed, the top standing up)."""
    hd = outer_body() & hood_region() & bx(XB - T - 5, X_FP + 5, f.band_y - 5, Y1 + T + 5, W_SHELL0 - 1, W_CUT)
    w, y = CTRL["dial"]  # nothing to thin: a 3 mm strip + the EC11's 2 mm nut fit its 6 mm bushing
    hd = hd + (lap_layer(LAP_T) & hood_region() & bx(XB - T - 5, X_FP + 5, f.band_y - 5, Y1 + T + 5, W_CUT - 0.1, W_CUT + LAP))
    return control_cutouts(hd)


def control_cutouts(fp):
    w, y = CTRL["estop"]; fp = fp - xcyl(11.175, X_FRONT - 1, X_FP + 1, y, w)     # 22 mm E-stop: Ø22.35 + its key
    kw, kext = B.ESTOP_KEY                                                         # notch (3.5 wide, 24.1 over it), up
    fp = fp - bx(X_FRONT - 1, X_FP + 1, y, y - 11.175 + kext, w - kw / 2, w + kw / 2)
    w, y = CTRL["toggle"]; fp = fp - xcyl(6.1, X_FRONT - 1, X_FP + 1, y, w)      # Ø12.2 (M12, drawing: Ø12) + 1.9 key notch
    fp = fp - bx(X_FRONT - 1, X_FP + 1, y + 5.5, y + 7.1, w - 1.05, w + 1.05)
    w, y = CTRL["rapid"]; fp = fp - xcyl(8.15, X_FRONT - 1, X_FP + 1, y, w)
    w, y = CTRL["dial"]; fp = fp - xcyl(3.7, X_FRONT - 1, X_FP + 1, y, w)      # PEC11R: M7 bushing, nut in front, no lug
    for s_ in (-1, 1):                     # so two ribs either side of its body key it: it can't turn into the OLED
        w_in, w_out = sorted((w + s_ * DIAL_RIB[0], w + s_ * DIAL_RIB[1]))
        fp = fp + bx(X_FRONT + G - DIAL_RIB[2], X_FRONT + G + 0.5, y - 6.7, y + 6.7, w_in, w_out)
    w, y = CTRL["oled"]; fp = fp - bx(X_FRONT - 1, X_FP + 1, y - 8, y + 8, w - 15, w + 15)
    hx, hy = B.OLED_HOLES[0] / 2, B.OLED_HOLES[1] / 2
    for dw, dy in ((-hx, -hy), (hx, -hy), (-hx, hy), (hx, hy)):                    # OLED M2 posts (Ø4.4: clear of its
        yp = y + B.OLED_YC + dy                                                    # glass, which ends ~2.4 from the holes)
        fp = fp + xcyl(2.2, X_FRONT - 2.5, XF + G + 0.5, yp, w + dw) - xcyl(0.8, X_FRONT - 2.6, X_FRONT + 1.0, yp, w + dw)
    w, y = CTRL["adapter"]
    for dw, dy in ((-16.0, -11), (16.0, -11), (-16.0, 11), (16.0, 11)):              # adapter board posts, M3 inserts
        fp = fp + xcyl(4.0, X_FRONT - 5, XF + G + 0.5, y + dy, w + dw) - xcyl(INSERT_D / 2, X_FRONT - 5.1, X_FRONT - 5 + INSERT_L, y + dy, w + dw)
    for txt, w, y in LABELS:                                                       # engraved 0.6 deep (3 layers)
        pl = Plane(origin=(X_FP, y, w), x_dir=(0, 0, -1), z_dir=(1, 0, 0))
        try:
            fp = fp - extrude(pl * Text(txt, 4.5), amount=-0.6)
        except Exception as e:  # noqa: BLE001 - a missing font must not stop the build
            print("label %s skipped: %s" % (txt, e))
    return fp


def panel_part(shape, w, y, x=X_FP):
    """A bought panel part (bought.py frame: panel front at z = 0, z out of the panel) on the control strip at (W, y):
    its z along +x, its x along -W (so its long axis follows LEFT/RIGHT)."""
    loc = Plane(origin=(x, y, w), x_dir=(0, 0, -1), z_dir=(1, 0, 0)).location
    return Compound(children=[so.moved(loc) for so in shape.solids()])


def controls():
    """The real panel parts, placed (bought.py). Same order as CONTROL_NAMES."""
    c = []
    w, y = CTRL["estop"]
    c.append(panel_part(B.estop_zgszjhf22(STRIP_T + ESTOP_PLATE_T), w, y, x=X_FP + ESTOP_PLATE_T))   # on its plate
    c.append(panel_part(B.estop_legend_plate(), w, y))
    w, y = CTRL["toggle"]
    c.append(panel_part(B.toggle_eten1322(STRIP_T), w, y))
    w, y = CTRL["rapid"]
    c.append(panel_part(B.pushbutton_16mm(STRIP_T), w, y))
    w, y = CTRL["dial"]
    c.append(panel_part(B.encoder_pec11r(STRIP_T), w, y))
    w, y = CTRL["oled"]                                  # on its 2.5 mm M2 posts behind the window
    c.append(panel_part(B.oled_sh1106(), w, y, x=X_FRONT - 2.5))
    w, y = CTRL["adapter"]
    c += [bx(X_FRONT - 6.6, X_FRONT - 5, y - 14, y + 14, w - 20.0, w + 20.0), bx(X_FRONT - 15.6, X_FRONT - 6.6, y - 5, y + 5, w - 16, w + 16)]
    return c


CONTROL_NAMES = ["estop", "estop_plate", "toggle", "rapid", "dial", "oled", "adapter_board", "adapter_header"]


def end_wall_parts(threads=True):
    """BOOT/RESET pushbuttons, the SWD header, the USB-C panel end outside (thread, nut, dust cap) and the cable gland,
    placed on the end wall (bought.py frames: z along +W)."""
    out = []
    for bxx, byy in (BOOT_BTN, RESET_BTN):               # mounted from behind on the recess floor, nut in front
        out.append(Pos(bxx, byy, W_END - BTN_RECESS[1]) * B.pushbutton_pbs110(BTN_FLOOR))
    sx, sy = SWD_PORT                                    # 1x3 header pressed into its slot, pins sunk in the recess
    hdr = bx(sx - 3.81, sx + 3.81, sy - 1.27, sy + 1.27, W_END - 9.4, W_END - 6.9)
    for k in (-1, 0, 1):
        hdr = hdr + bx(sx + 2.54 * k - 0.32, sx + 2.54 * k + 0.32, sy - 0.32, sy + 0.32, W_END - 12.4, W_END - 1.0)
    out.append(hdr)
    ux, uy = USB_PORT                                    # flange outside, nut and body inside
    out.append(Pos(ux, uy, W_END) * B.usb_c_panel(T))
    out.append(Pos(36.0, -62.0, W_END) * B.gland_m16(T))
    return out


def fan():
    """The optional 40 mm fan on the louvre block's inside, blowing out through the back wall."""
    yc, wc = (FAN["y0"] + FAN["y1"]) / 2, (FAN["w0"] + FAN["w1"]) / 2
    loc = Plane(origin=(FAN["x_in"] + 10.0, yc, wc), x_dir=(0, 0, 1), z_dir=(-1, 0, 0)).location
    return Compound(children=[so.moved(loc) for so in B.fan_40x10().solids()])


def screw_joints():
    """(part screwed, target, outer point, inward unit vector, kind). 'tap' = M3 tapped 8 deep in the aluminium plate,
    'insert' = M3 heat-set insert in a printed rail. Countersunk M3 through the printed wall."""
    j = []
    wl = (W_SHELL0 + Z_PLATE_OUT) / 2                 # mid-lap: into the plate edges
    for x in (-30.0, 0.0, 30.0):
        j.append(("hood", "plate", (x, Y1 + T, wl), (0, -1, 0), "tap"))
    for x in DOOR_SCREWS_X:                          # the door's plate-end screw (the far one goes into an insert)
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
    j.append(("hood", "shell", (xr, Y1 + T, W_CUT - 8), (0, -1, 0), "insert"))          # the hood's far end
    for w in (110.0, 200.0, W_CUT - 8):
        j.append(("hood", "shell", (X_FP, f.band_y + RAIL / 2, w), (-1, 0, 0), "insert"))
    for side, y in CAP_BLOCKS:                        # end cap: through the shell's walls into its blocks
        j.append(("shell", "end_cap", (XB - T if side < 0 else X_FP, y, CAP_SCREW_W), (-side, 0, 0),
                  "insert"))
    return j


def housing():
    return {"shell": shell(), "hood": hood(), "door": door(), "end_cap": end_cap()}


def apply_screws(parts):
    for part_, target, p, d, kind in screw_joints():
        pl = Plane(origin=p, z_dir=d)
        # Ø3.4 clearance and a 90-degree countersink for an ISO 10642 M3 head (Ø6.72): Ø6.9 at the surface
        parts[part_] = parts[part_] - Solid.make_cylinder(1.7, T + 0.01, pl) - Solid.make_cone(3.45, 1.7, 1.75, pl)
        inner = Plane(origin=tuple(p[i] + d[i] * T for i in range(3)), z_dir=d)
        r, depth = (1.25, 8.0) if kind == "tap" else (INSERT_D / 2, INSERT_L)
        parts[target] = parts[target] - Solid.make_cylinder(r, depth, inner)
    return parts


def insert(point, direction, threads):
    """M3 brass heat-set insert, Kadrick M3 x 5 (Amazon B0FD88XTV4: knurl Ø4.5, lead-in Ø3.9; hole Ø4.0 x >= 6, wall
    >= 1.6), with its thread, mouth at point, going along direction (modelled at the hole's Ø4)."""
    loc = Plane(origin=point, z_dir=direction).location
    sleeve = (Cylinder(2.0, 5.0, align=MIN) - Cylinder(1.6, 5.0, align=MIN)).moved(loc)
    th = FA.at("thread", 3, 5.0, point, direction, threads)
    return Compound(children=[sleeve] + (list(th.solids()) if th is not None else []))


RIVNUT = (8.0, 0.8, 4.9, 9.0)   # HANGLIFE M3 flat-head rivnut (B0DF72LHVP): flange Ø, flange t, body Ø, overall length


def rivnut(x, w, threads):
    """Open-end M3 rivnut (HANGLIFE HLRN-M3-001, Amazon B0DF72LHVP drawing: Ø5 hole, body Ø4.9, flange Ø8 x 0.8, 9
    overall, grip 0.5-1.5; no closed-end M3 ships to Israel) in a lid flange: its flange on the flange's top, inside
    the lid's Ø8.5 clearance hole. The M3 x 8 lid screw ends 2.7 short of its open end, so no screw tip reaches into
    the mains zone."""
    top = TERM_TOP
    fd, ft, bd, ln = RIVNUT
    body = ycyl(fd / 2, top, top + ft, x, w) + ycyl(bd / 2, top + ft - ln, top, x, w) - ycyl(1.25, top + ft - ln - 0.1, top + ft + 0.1, x, w)
    th = FA.at("thread", 3, ln - ft - 1.0, (x, top + ft, w), (0, -1, 0), threads)
    return Compound(children=[body] + (list(th.solids()) if th is not None else []))


INTO = {"field_housing": "clutch_field_housing", "motor_reference": "motor", "motor": "motor", "output_hub": "clutch_hub",
        "output_shaft": "clutch_output_shaft", "cover": "clutch_cover", "rotor": "clutch_rotor"}


def all_fasteners(threads=True):
    """Every screw, nut, washer, set screw, circlip, insert and rivnut in the unit, placed, with the threads of the tapped
    holes: (name, shape, into, group). `into` is the part (or fastener) it threads into or seats in: the checks allow
    overlap only there. `group` sorts them into the viewer's files."""
    out = []
    for name, shape, _, into in clutch.fasteners(threads):        # the clutch module's own, as mounted
        moved = Compound(children=[place_module(so) for so in shape.solids()])
        out.append((name, moved, INTO.get(into, into), "clutch"))
    for x, y in module_bolts(0.0):                                 # module: 4x M5x35 + washers, from the belt chamber
        out.append(("module bolt washer", FA.at("washer", 5, 0, (x, y, Z_PLATE_IN), (0, 0, -1), threads), "plate", "plate"))
        out.append(("module bolt M5x35", FA.at("shcs", 5, 35, (x, y, Z_PLATE_IN - 1.0), (0, 0, 1), threads), "clutch_cover",
                    "plate"))
    out.append(("drawbolt M4x30", FA.at("shcs", 4, 30, (MX, TB_Y - 7.0, Z_PLATE_OUT + 4), (0, 1, 0), threads),
                "clutch_cover", "plate"))
    for x in TB_SCREWS_X:                                          # tension block: 2x M5x20 into the plate's legs
        out.append(("tension block M5x20", FA.at("shcs", 5, 20, (MX + x, TB_Y, Z_PLATE_OUT + 12), (0, 0, -1), threads),
                    "plate", "plate"))
        out.append(("plate thread M5", FA.at("thread", 5, 12, (MX + x, TB_Y, Z_PLATE_OUT), (0, 0, -1), threads), "plate",
                    "plate"))
    for x, y in f.standoffs:                                       # 4x M6x70 through plate + standoff into the table end
        out.append(("standoff M6x70", FA.at("shcs", 6, 70, (x, y, Z_PLATE_OUT - STANDOFF_CB), (0, 0, -1), threads),
                    "mill_reference", "plate"))
    zd = Z_PLATE_OUT + f.enc_post_len                              # encoder: disc onto the posts, encoder onto the disc
    for k in range(3):
        a = math.radians(90 + 120 * k)
        xp, yp = f.enc_post_r * math.cos(a), f.enc_post_r * math.sin(a)
        out.append(("encoder post screw M3x8", FA.at("button", 3, 8, (xp, yp, zd + f.enc_disc_t), (0, 0, -1), threads),
                    "encoder_posts", "plate"))
        out.append(("plate thread M3 (post)", FA.at("thread", 3, 8, (xp, yp, Z_PLATE_OUT), (0, 0, -1), threads), "plate",
                    "plate"))
        xe, ye = 15 * math.cos(a + math.pi / 3), 15 * math.sin(a + math.pi / 3)
        out.append(("encoder screw M3x8", FA.at("button", 3, 8, (xe, ye, zd), (0, 0, 1), threads), "encoder", "plate"))
    for (cx, cy), teeth, bore, d, ln in (((0.0, 0.0), f.teeth_big, f.stub_d, 5, 20),           # pulley set screws, radial
                                         ((MX, MY), f.teeth_small, CP.out_shaft_d, 5, 6)):   # through a tooth root onto
                                                                  # the shaft's flat / round (sunk below the root)
        for a_deg in B.pulley_set_screw_angles(teeth):
            c_, s_ = math.cos(math.radians(a_deg)), math.sin(math.radians(a_deg))
            r0 = bore / 2 + ln
            out.append(("pulley set screw M%dx%d" % (d, ln), FA.at("set", d, ln, (cx + r0 * c_, cy + r0 * s_, W_PULLEY),
                        (-c_, -s_, 0), threads), "pulley_40T" if teeth == f.teeth_big else "pulley_20T", "plate"))
    a_, b_ = bearing_zs()                                          # stub circlips: DIN 472-32 x2 in the plate, 471-15
    for zg in (Z_PLATE_IN + f.circlip_land, Z_PLATE_OUT - f.circlip_land - f.circlip_groove):
        out.append(("circlip DIN 472-32", FA.circlip(13.6, 16.8, 1.2, zg + 0.05), "plate", "plate"))
    out.append(("circlip DIN 471-15", FA.circlip(7.2, 9.4, 1.0, a_ + f.bearing_w + 0.15), "stub_shaft", "plate"))
    for part_, target, pt, d, kind in screw_joints():              # housing: countersunk M3x8 into the plate or inserts
        out.append(("housing screw M3x8", FA.at("csk", 3, 8, pt, d, threads), target, "housing"))
        inner = tuple(pt[i] + d[i] * T for i in range(3))
        if kind == "insert":
            out.append(("heat-set insert M3", insert(inner, d, threads), target, "housing"))
        else:
            out.append(("plate thread M3", FA.at("thread", 3, 8, inner, d, threads), "plate", "housing"))
    for px, py in PCB_HOLES:                                       # PCB: 4x M3x6 button into inserts in the bosses
        x, y, w = pcb_to_world(px, py, 0)
        out.append(("PCB screw M3x6", FA.at("button", 3, 6, (x, y + 1.6, w), (0, -1, 0), threads), "pcb_placeholder",
                    "housing"))
        out.append(("heat-set insert M3", insert((x, y, w), (0, -1, 0), threads), "shell", "housing"))
    for xt in TRAY_TABS_X:                                         # mains tray: 2x M4x8 button up into the plate,
        out.append(("tray screw M4x8", FA.at("button", 4, 8, (xt, TRAY_BOT, TRAY_SCREW_W), (0, 1, 0), threads), "plate",
                    "mains"))                                      # a serrated washer between tab and plate
        out.append(("serrated washer M4", ycyl(4.0, TRAY_BOT + f.tray_t, TRAY_BOT + f.tray_t + 0.5, xt, TRAY_SCREW_W) -
                    ycyl(2.2, TRAY_BOT, TRAY_BOT + 3, xt, TRAY_SCREW_W), "plate", "mains"))
        out.append(("plate thread M4", FA.at("thread", 4, 10, (xt, PLATE_BOT + REBATE_H, TRAY_SCREW_W), (0, 1, 0), threads),
                    "plate", "mains"))
    for xp, wp in psu_screws():                                    # PSU: 2x M3x4 button up through the tray floor
        out.append(("PSU screw M3x4", FA.at("button", 3, 4, (xp, TRAY_BOT, wp), (0, 1, 0), threads), "psu", "mains"))
    for xs, ws in lid_screws():                                    # lid: 4x M3x8 button + a Ø12 washer over the
        top = TERM_TOP + f.tray_t                                  # lid's Ø8.5 hole, into open-end rivnuts
        out.append(("lid washer M3x12", ycyl(6.0, top, top + 1.0, xs, ws) - ycyl(1.6, top - 0.1, top + 1.1, xs, ws),
                    "mains_tray_lid", "mains"))
        out.append(("lid screw M3x8", FA.at("button", 3, 8, (xs, top + 1.0, ws), (0, -1, 0), threads), "rivnut", "mains"))
        out.append(("rivnut M3", rivnut(xs, ws, threads), "mains_tray", "mains"))
    return [o for o in out if o[1] is not None]


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
        "motor": B.nema23_23hs45() + B.nema23_shaft(CP.shaft_len),
    }
    module = {k: place_module(v) for k, v in module.items()}
    # drive pins (+ washers, E-clips: they move with the armature) and return springs + bushes; children placed one by
    # one (booleans on a Compound moved as a whole see the children unmoved)
    springs, washers, clips = clutch.return_springs()
    module["clutch_pins"] = Compound(children=[place_module(x) for x in clutch.drive_pins() + washers + clips])
    module["clutch_springs"] = Compound(children=[place_module(x) for x in springs])
    module["clutch_bushes"] = Compound(children=[place_module(x) for x in clutch.bushes()])
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
    parts.update(housing())
    parts = apply_screws(parts)
    parts["pcb_placeholder"] = Compound(children=pcb_placeholder())
    parts["controls"] = Compound(children=controls())
    parts["plug_wires"] = Compound(children=plug_wires())
    parts["usb_path"] = Compound(children=usb_envelopes())
    groups_fx = {}                                     # every fastener, threaded, one file per group (viewer)
    for _, shape, _, grp in all_fasteners(threads=True):
        groups_fx.setdefault(grp, []).extend(shape.solids())
    for grp, sols in groups_fx.items():
        export_stl(Compound(children=sols), os.path.join(OUT, "fx_" + grp + ".stl"), tolerance=0.05, angular_tolerance=0.5)
    parts["psu"], parts["mains_inlet"] = psu_placeholder()
    parts["mains_cord_plug"] = cord_plug()
    parts["end_wall_parts"] = Compound(children=[so for p_ in end_wall_parts() for so in p_.solids()])
    parts["fan"] = fan()
    parts["mains_tray"], parts["mains_tray_lid"] = mains_tray()
    # the door's hinge pin: Ø3 silver steel, pressed into the shell's plate-end knuckle, the door turns on it
    parts["door_pin"] = wcyl(1.5, SHELL_KNUCKLES[0][0], SHELL_KNUCKLES[-1][1], HINGE_X, HINGE_Y)
    groups = {"estop": (0, 1), "estop_plate": (1, 2), "toggle": (2, 3), "rapid": (3, 4), "dial": (4, 5), "oled": (5, 6),
              "adapter": (6, 8)}
    cs = controls()
    for nm, (i0, i1) in groups.items():                       # one STL per control, so the viewer can name them
        parts["control_" + nm] = Compound(children=cs[i0:i1])
    with open(os.path.join(OUT, "viewer_layout.json"), "w") as fh:  # the 3D viewer places the KiCad PCB from this
        json.dump({"pcb_x0": f.pcb_x0, "pcb_y": f.pcb_y, "pcb_w0": f.pcb_w0,
                   "door_axis_x": HINGE_X, "door_axis_y": HINGE_Y,     # door hinge axis, along W
                   # clutch return springs (exported engaged): seated at this W, they stretch toward -W when released
                   "spring_base_w": W0 - (clutch.Z_HUB + CP.bush_len),
                   "spring_len_engaged": clutch.SPRING_LEN_RELEASED - CP.stroke, "stroke": CP.stroke}, fh)
    for name, shape in parts.items():
        export_stl(shape, os.path.join(OUT, name + ".stl"))
        if not name.startswith(("mill_reference", "motor", "clutch_", "belt", "bearings", "module_bolt", "pcb_", "control", "plug_", "psu", "mains_inlet", "door_pin", "usb_", "mains_cord", "end_wall_parts", "fan")):
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
| Plate | 6061, {f.plate_t:.0f} mm, CNC | **Ø{f.bearing_od:.0f} H7 straight through**: too big to swing on the lathe (300 mm needed), so bore it on the mill with a boring head. DIN 472-32 grooves (Ø33.7 × {f.circlip_groove}) leaving a **{f.circlip_land} land** at each face: keyseat cutter (head ≤ Ø28, 1.0–1.2 wide, two passes) by circular interpolation, one face, then flip for the other. Clutch-module hole slotted Ø{2 * (CP.bearing_od / 2 + 6):.0f} **and run on down through the bottom edge** (the module slides up into it from below; the two legs are {f.x_half - (CP.bearing_od / 2 + 6):.0f} wide, tied by the tension block) and 4× Ø5.5 slots (±{f.tension_slot:.0f}): 5×D deep, cut them from both faces. Bottom edge: 2 rebates {REBATE_H} deep × 14 wide × 13 in from the outer face at x ±{TRAY_TABS_X[1]:.0f} (Ø6 end mill from the edge, R3 inner corners) with an M4 × 10 tapped up into each, for the mains tray's tabs. {len(f.standoffs)}× Ø6.6 for the standoffs, each with an Ø11 × {STANDOFF_CB} counterbore in the outer face (M6×70 SHCS heads flush: the low pair sits beside the clutch cover); 3× M3 × 8 deep on r={f.enc_post_r:.0f} (encoder posts, outer face); 2× M5 × 12 for the tension block (x ±{TB_SCREWS_X[1]:.0f}). Edge holes (M3 × 8 for the housing): stand the plate in the vise on its edge, or use a drill-guide block. Corners R{f.r_long - f.panel_t:.0f} |
| Standoffs | steel or 6061 | {len(f.standoffs)}× Ø{f.standoff_d:.0f} × {f.chamber:.0f}, Ø6.6 thru (M6 screws into the table end) |
| Leadscrew stub | steel, turned from **Ø20** stock | from the table side: tongue {f.stub_tongue_w} × {f.screw_d - 2:.0f} × {f.stub_tongue_h:.0f} (across the Oldham slot), Ø{f.stub_d:.0f} with a 0.6 flat for the 40T set screws, **Ø{f.stub_shoulder_d:.0f} × 2 shoulder** on the inner bearing's inner race (corner radius ≤ 0.3 so the bearing's chamfer clears it), Ø{f.stub_d:.0f} h6 at both bearings with **Loctite 243 on the inner one** (as on the clutch; easier than holding k6 and h6 bands on a manual lathe), DIN 471-15 groove (Ø14.3 × 1.1) just outboard of the inner bearing, Ø{f.stub_enc_d:.0f} × 10 encoder end |
| Bearing spacer tube | steel or 6061 | Ø{f.bearing_od - 0.05:.2f} / Ø28 × {bearing_zs()[1] - bearing_zs()[0] - f.bearing_w:.1f}, between the outer races. The inner bearing locates the stub (shoulder + DIN 471-15 either side of it); the outer one floats on the stub |
| Oldham disc | steel (1045) or bronze (CuSn12), greased — acetal creeps at the ~45 MPa of a stall/cam-out | Ø{f.oldham_d:.0f} × {f.oldham_t:.0f}: tongue {f.screw_d - 2:.0f} × {f.screw_end_slot_w - 0.2:.1f} on one face (into the leadscrew slot), cross slot {f.stub_tongue_w + 0.2:.1f} wide × {f.stub_tongue_h + 0.5:.1f} deep on the other. Takes up to ~1 mm offset between the plate and the leadscrew, and end float |
| Encoder disc | 6061, {f.enc_disc_t:.0f} mm | Ø{f.enc_disc_d:.0f} with a flat {f.enc_disc_flat:.0f} from centre toward the clutch module; Ø22 centre hole; 3× Ø3.4 on r={f.enc_post_r:.0f} (posts) and 3× Ø3.4 on r=15, 60° offset (encoder, **check your encoder's hole circle**) |
| Encoder posts | 3× M3 hex standoffs F-F, {f.enc_post_len:.0f} mm, 6 across flats (priaolker brass, Amazon B09NDJFLG7) | |
| Tension block | 6061 | {2 * TB_HALF:.0f} × 14 × 12 on the plate's outer face below the module, bridging the plate's slot, 2× M5 into the plate's legs (x ±{TB_SCREWS_X[1]:.0f}), **with a {2 * (CP.bearing_od / 2 + 6 - 0.1):.1f} × {Z_PLATE_OUT - Z_PLATE_IN:.0f} tongue that fills the slot** from the plate's bottom edge up to {KEY_TOP - PLATE_BOT:.0f} (just under the clutch boss): it seals the belt chamber off from the box (chips, coolant) and ties the plate's legs (checked); M4 drawbolt through it into a radial M4 tapped 7 deep in the cover's front wall (bottom, between two module bolts). Tighten the drawbolt to tension the belt, then lock the 4 module bolts |
| Belt guard | PETG print | a cup: closed back against the table end (Ø{f.oldham_d + 4:.0f} pass-through for the leadscrew and Oldham disc), open face covered by the plate; captured between the two |

## Assembly
1. Plate: outer circlip, outer 6002 (pressed in from the chamber side), spacer tube.
2. Stub: inner 6002 against the shoulder, DIN 471-15 behind it. Push the stub through the outer bearing from the
   chamber side (support the outer bearing's inner race) until the inner bearing seats, then the inner circlip.
3. Encoder to its disc, coupling on the encoder shaft, disc onto the posts, coupling onto the stub end.
4. 40T pulley on the stub (set screws: one on the flat, one on the round), Oldham disc on the stub tongue, guard into the chamber, then
   the plate onto the standoffs: the Oldham tongue goes into the leadscrew slot.
5. Clutch module through the plate's slotted hole (motor bolted to the clutch with its leads toward the front wall, not up
   toward the PCB: checked), 20T pulley and belt, 4 module bolts finger-tight, drawbolt to tension, lock.
   The bolt heads are on the chamber side, outside the guard outline: reach them from the side with a ball-end key.

## Buy
Every bought part ships to Israel from Amazon.com (checked with an Israeli address); the Amazon list "Mill power feed
(SIEG X3.5D)" has them all, and `bought.py` models each from its listing's drawing.
- HTD 5M pulleys, both hubless (flange, teeth, flange): **{f.teeth_small} T Saelno 20-5M-15-8mm** (Amazon B0FNQTJZMX: Ø35 flanges, 16
  between them, 20 overall, Ø8 bore, 2× M5 through the teeth 90° apart) and **{f.teeth_big} T uxcell 5M-40T-1/2B** (B0G5YDQS1N:
  Ø67 flanges, 22 overall, Ø12.7 bore with a keyway). Re-bore to Ø{CP.out_shaft_d:.0f} / Ø{f.stub_d:.0f} H7 (the 40T's keyway leaves a 0.25
  trace: harmless). Set screws 90° apart through a tooth root, one on the shaft's flat, one on the round, like the
  clutch hub: the 20T's own 2× M5 (M5×6), on the 40T 2× M5×20 (use its own holes if they are in a tooth root, else
  drill Ø4.2 and tap; the drill nicks the teeth either side: harmless, deburr).
- Belt {f.belt_len:.0f}-5M-{f.belt_w:.0f} (uxcell B0D5R5Y6VL).
- 2× 6002-2RS, 2× DIN 472-32, 1× DIN 471-15.
- Flexible coupling 6 × 8 mm, uxcell Ø19 × 25 beam type with set screws (B0FN786WNM).
- Encoder: Aideepen LPD3806-600BM-G5-24C, 600 P/R, NPN open collector (B01MZ4V1XP).
- Retaining compounds (Loctite 641 and 638 don't ship to Israel): **Loctite 243** (Amazon B004L439FE) on the bearing
  seats and the motor shaft (medium strength, comes off with hand tools), **Loctite 680** (B00KC541DA, seller ACTOOLS)
  for the clutch's drive pins. Both checked shipping to Israel.
- 4× M6 for the standoffs; 4× M5×35 + washers for the clutch module ({f.plate_t:.0f} plate + 8 wall: the tips stop clear of the hub); 2× M5×20 (tension block) + 1× M4×30 (drawbolt); 3× M3 hex standoffs {f.enc_post_len:.0f} mm F-F (room for the 25 mm coupling) and 6× M3×8.

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
| `shell` (one print) | back and lower-front walls ({T:.0f} mm) from the plate to W {W_CUT:.0f}, the rails, the 4 PCB bosses, the louvre blocks; **no floor** (that is the door), no end wall (that is the end cap). **Print standing on its cut face** (the plate-side rim up), {W_CUT - W_SHELL0:.0f} mm tall (MK3S: 210), on the 5 × 3 brace that ties its back and front walls across the cut face; above the brace they stand as two tall walls, so **use a brim** (5 mm). Every boss and block has a 45° underside, so **no supports**; only short bridges (≤ 12 mm: rail windows, louvre slots). PETG, 4 perimeters, 20 % gyroid |
| `end_cap` (one print) | the last {W_END - W_CUT:.0f} mm of the box: the end wall, the ends of the back, front and top walls and of the control strip, and the floor under the mains tray's far end (2 ribs carry that end). It exists because the LRS-150-36 (the only 36 V Mean Well on Amazon that ships to Israel) makes the box {W_END - W_SHELL0:.0f} mm long, too long for one print. **Print standing on the end wall**, {W_END - W_CUT + CAP_BLOCK[2]:.0f} mm tall with its blocks; the blocks' ends are 45° wedges, no supports. It goes on **last, sliding along the table axis** (checked: nothing in its way): its floor slides under the tray's far end, a 45° scarf on its bottom band slides under the door's far end and holds the door shut, and 3 blocks reach {CAP_BLOCK[2]:.0f} mm into the shell along its back and front walls: {len(CAP_BLOCKS)}× M3×8 countersunk through the shell's walls into heat-set inserts in them. The seam is **lapped over the top and the sides**: the cap's outer {LAP_T} mm runs {LAP:.0f} mm on over a step in the shell's walls, and the hood's runs {LAP:.0f} mm on over a step in the cap's top ({LAP_G} mm clear), so coolant on the top or the sides can't reach the plugs or the mains tray. Only the sides' bottom {KNUCKLE_R + 2:.1f} mm (beside the hinge knuckles) is a plain butt joint, which leads only to the floor under the earthed tray: a bead of silicone there, as on the door seam, if coolant gets in; the hood still lifts straight off, the cap still slides. Take it off (3 screws) before opening the door: it then hangs on the USB, SWD, button and limit-switch wires, so leave them ~150 mm of slack and set the cap down beside the box (or tie it up) rather than let it hang |
| `door` (one print) | **the floor** up to the end cap, both rounded bottom edges included ({X_FP - XB + T:.0f} × {W_CUT - W_SHELL0:.0f} mm, up to {SPLIT_Y - (Y0 - T):.1f} mm on the walls): open it and the clutch/motor module and the mains tray go in and come out **from below** (checked: both lift into place with nothing in the way). Hinged along the **back bottom edge** on a Ø3 × 180 steel pin (silver steel / drill blank) through 5 knuckles outside the back wall; push the pin in **from the end-wall end** (that line runs just outside the box; from the plate end it would hit the mill): it slides through every knuckle (Ø3.2) and only its last 30 mm press into the plate-end knuckle (Ø2.95, a drop of CA). To take the door off, punch the pin out from the plate end (there is room under the belt guard). Opened, it swings down and hangs beside the box, so the opening is clear (swing checked to 100°). Shut by one screw on the front side, M3×8 countersunk up into the plate's bottom edge (x {DOOR_SCREWS_X[0]:.0f}); its far end lies on the end cap's 45° scarf, which holds it up, so the cap comes off before the door opens and goes on after it shuts. It carries the belt drawbolt access hole (belt tension is set with it shut) and pockets for the PSU's and the tray tabs' screw heads. **Print outer face down**: 45° feet under the rounded edges, 45° drips and a 1.2 mm web from each door knuckle down to the bed (trim the webs off flush after printing), and its far end leans out at 45° (the scarf), no supports. The seam is a plain 0.3 mm butt joint: run a bead of silicone or a strip of foam tape along the front if coolant gets in |
| `hood` (one print) | top + control strip up to the end cap, one L-shaped piece. **Print with the strip face down**: the engraved labels come out crisp on the bed side; its back lip has a 45° inside so it prints without support. Lift it off and the whole PCB is exposed from above |
| Plate | aluminium, {2 * f.x_half:.0f} wide, bottom corners R{f.r_long - f.panel_t:.0f}, both top corners chamfered {CHAMFER_IN + 0.3:.1f} × 45° to sit under the housing's chamfered inside |
| Fixing | shell, hood and door overlap the plate edges by {f.lap:.0f} mm with {f.fit_gap} mm clearance per side, and screw into them ({sum(1 for j in screw_joints() if j[4] == "tap")}× M3×8 countersunk into tapped holes); the hood also takes {sum(1 for j in screw_joints() if j[4] == "insert" and j[0] == "hood")}× M3×8 into heat-set inserts (Kadrick M3 × 5, B0FD88XTV4: Ø4.0 × 6 holes) in the shell's rails, and the end cap {len(CAP_BLOCKS)} through the shell's walls into inserts in its blocks |
| PCB | on 4 printed bosses (heat-set inserts, M3×6), underside {f.pcb_y:.0f} mm below the leadscrew axis, ≥ 3 mm above the motor even with the motor raised 3 mm by the belt tensioner. The rails above it have 12 mm windows over all 4 holes, so a screwdriver and the insert iron go straight down (checked). Trim the SWD header's (J3) pins flush underneath. Air rises past the board between the bosses |
| Plugs | J1/J2/J5 MSTB plugs (≈13 mm) + ferrules: the wires run toward the end wall and turn down past the board's edge, behind the motor tail ({W_END - T - W_TAIL:.0f} mm), checked with a plug + wire-bend envelope. Unplugging in place: slide the plug off, the wires have slack below |
| Controls (strip, left → right as you face it) | E-stop **ZGSZJHF 22 mm ultra-short** (Amazon B0FZCN2HCP; 1 NO + 1 NC: the NC is the E-stop loop, ESTOP_SRC–ESTOP_RET; Ø22.35 hole with its key notch, 3.5 wide (its drawing: 3.3 +0.2), so it can't turn; Ø32.3 mushroom 25.7 proud, only 18.3 behind the panel to its pin tips, so it clears the J6 ribbon plug with room to spare; its ring nut is drawn but not dimensioned, modelled Ø30) on a **printed yellow Ø45 legend plate** (1 mm, keyed the same), LEFT/STOP/RIGHT toggle (DMWD ON-OFF-ON latching DPDT, B0BTSPYM34: Ø12 hole (Ø12.2 here) + key notch for its lock washer, body 29.7 × 19.8 × 28.7) over RAPID (Gebildet Ø16 stainless momentary, B081JJBN4G: 22 behind the panel), SPEED dial (EC11-type encoder; CAD = Bourns PEC11R: M7 bushing, nut in front, no lug hole; snip the lug if yours has one: two printed ribs either side of its body stop it turning into the OLED) with the uxcell Ø30 knurled aluminium handwheel (B0DGFB1K9F, hub toward the panel, its 2 set screws on the shaft), the Hosyond 1.3" SH1106 OLED (B0C3L7N917, holes on 30.3 × 28) on 4 M2 posts behind its window; adapter board 40 × 28 (2×7 IDC) on 4 posts on the strip's inside, ~200 mm ribbon down to J6 so the hood can be lifted and set beside the unit. All shapes from the Amazon listings' drawings (`bought.py`), checked against the strip, the PCB, its plugs and each other, the E-stop and toggle through their key play |
| End wall (on the end cap) | the mains inlet (recessed 9 mm, see Power), one M16 cable gland for the limit switches (uxcell nylon M16×1.5, B08K8HPHGM), low beside the motor. **Programming ports, usable with the box closed:** a Ø21.6 hole for a round threaded USB-C panel extension (QIANRENON 5 Gbps, B0FX2MZDTS: M21 thread, Ø27.5 flange outside, its nut inside; body and strain relief reach 34.5 in, 30 cm cable to a male plug; the Adafruit #6069 with its M12 thread doesn't ship to Israel), **plus a 90° USB-C adapter (male to female, "up/down" type, rgzhihuifz B08P5HRH3Z) in J4**: J4 is only 29 mm from the wall and the extension's straight male end is long (~38 mm allowed: the listing doesn't give it), so it goes down into the adapter from above (space checked, with the panel end's body and its cable turning away); **BOOT and RESET** buttons (Twidec PBS-110 7 mm momentary NO, pre-wired, B07RPS2ZY3, in Ø7.2 holes, recessed in Ø16.5 × 8.5 counterbores (room for a 1/4" 10 mm socket, ≤ 16 OD, on the 10 AF nut) so the caps stay 0.3 mm below the wall face and can't be pressed by a bump or a palm; a fingertip still reaches them; solder their 2 wires across the pads of SW1 / SW2 on the PCB, in parallel with them) and a recessed **SWD port** (1×3 0.1" header, SWCLK / GND / SWDIO like J3, pressed into the slot from outside with a drop of CA; solder 3 wires to its inner pins and run them to J3) |
| Vents | intake through louvre blocks on the back and front walls beside the motor (slots climb 45° inward, so chips and spray can't reach the mains tray); no floor vents; exhaust through a louvre block high on the back wall, beside the encoder (slots climb 45° inward through 7.5 mm, so chips can't fall in), away from the fuse. The block takes a 40 mm fan (4× M3 self-tapping, 32 mm square) on its inside |

- **Power, built in: the mains tray.** All 230 V lives in an earthed, folded **aluminium tray** under the clutch and
  motor ({2 * TRAY_HALF:.0f} wide, {TRAY_TOP - TRAY_BOT:.0f} high over the PSU and {TERM_TOP - TRAY_BOT:.0f} over the terminal/inlet zone, W {Z_PLATE_OUT:.0f}…{TRAY_W1:.0f}; 1.5 mm
  1050/5052 sheet: floor, two side walls with inward-folded lid flanges over the terminal zone, end face, and two flat floor tabs
  that reach under the plate). Fold drawing: `out/mains_tray.step` (+ `mains_tray_lid.step`). It holds:
  - a Mean Well **LRS-150-36** (36 V / 4.3 A, 159 × 97 × 30, case 241A; Amazon B019GYOQDW, PowerNex: no LRS-75-36 or
    LRS-100-36 listing ships to Israel), screwed through the tray floor into its two base holes (24 and 102 from the
    terminal end): **M3 × 4 button head (ISO 7380)** — the case allows 3 mm of thread, the tray is 1.5 — reached through
    the service door; their heads sit in pockets in the door. Terminals 1 L, 2 N, 3 FG, 4–5 −V, 6–7 +V, toward the inlet;
  - a **snap-in IEC C14 inlet with fuse and rocker switch** clipped into the tray's end face (cut-out
    {f.inlet_cut[0]} × {f.inlet_cut[1]}): the **JOYNANO 3-in-1** (Amazon B071VLCTPB), sold for 0.8–1.5 mm panels, so it grips the 1.5 mm
    tray sheet (most snap-ins are for 2 mm and would rattle); its listing has no drawing, so the CAD uses the same generic
    module's drawing (SINORS, B0H5TNSZ7L: flange 50 × 30.8 × 3, cut-out 47 × 27.5 R2, body 20.5 + tabs 10): C14 bay left,
    rocker right, fuse drawer under the bay. Swap its 5 A fuse for a T 2 A ceramic. The cord: buy it locally, an ordinary
    C13 cord with an Israeli (SI 32) plug, 3 × 0.75 mm² or more. The C13
    plug (LS-13 type: body 24.6 × 17, nose 23 × 15) reaches the inlet through the end wall's opening. It sits 9 mm behind the printed end wall's opening, so
    coolant running down the end wall doesn't reach it. Fit an IEC retaining clip (V-lock or a wire bail) so the cord
    can't half-unplug on a vibrating table;
  - a separate aluminium **lid** over the terminal/inlet zone, its plate-side edge folded down to just above the PSU
    (so the zone is closed all round): 4× M3×8 button head + Ø12 washer into **open-end M3 rivnuts** (HANGLIFE, B0DF72LHVP: Ø5 hole,
    grip 0.5–1.5, 9 long; no closed-end M3 rivnut ships to Israel) in the walls' 10 mm folded flanges: the lid's holes
    are Ø8.5 so it lies flat over the rivnuts' Ø8 × 0.8 flanges, the washer clamps it, and each M3×8 ends inside its
    rivnut (2.7 mm short of the open end as bought; setting shortens the body, so check with the screw in that the
    tip stays inside, else use M3×6), so no screw tip reaches into the mains zone; the screws also bond the lid to the tray. Its flat part lies 1 mm above the
    PSU case; over the terminals it rises with the terminal zone (checked: ≥ 3 mm to the modelled screw heads; the head
    height is inferred from the drawing, so check it on your PSU), and line the
    inside of the fold (and the lid above the terminals) with 0.4 mm Nomex/Formex insulating sheet either way. A Ø10 grommet in it: the **only** thing that leaves the
    tray is the 36 V pair (1 mm², 300/500 V insulation since it runs inside the mains tray, to J1).
  - **Fixing and earth:** the tray goes in from below. Its two floor tabs sit in rebates in the plate's bottom edge and
    2× **M4 × 8 button head (ISO 7380)** go up through them into the plate, with a **serrated lock washer (DIN 6798 J
    M4, 0.5 thick) between each tab and the plate**, in the rebate: it bites both bare metals (checked against every other
    hole in the plate); the heads sit in pockets in the door. That holds it and earths it; 2 ribs on the end cap
    carry its far end. PE: inlet ⏚ → an M4 stud on the tray (ring terminal, star washer) → PSU ⏚. Leave both earth
    leads longer than L/N so they pull out last. Don't anodise the plate or the tray, or scrape the bond faces.
    Through the plate the motor (4 M5), the clutch (cover + housing screws), the encoder posts, the stub and the mill
    table are all on the same earth.
  - **Wiring:** 0.75 mm² 300/500 V wire, insulated female spades on the inlet (fuse in **L**), crimp ferrules in the
    PSU. {f.mains_wiring:.0f} mm of free wiring room between the PSU terminals and the inlet body (checked). Tie each mains wire next to
    its spade and its terminal, so one loose wire can't move. Wire the whole tray on the bench, lid on, before it
    goes in: the PSU terminals can't be reached with the motor above them.
  - **Fuse:** T 2 A ceramic (high breaking capacity) 5 × 20 for 230 V; T 4 A for 120 V (the LRS-150 draws 3 A there at full load).
  - **Power budget:** about 35–50 W at rapid traverse and about 60 W during the clutch pull-in (54 W for 50 ms with
    the motor at its 1 A hold current); the LRS-150's 155 W covers all of it, a jam at rapid (motor at full torque,
    ~100 W) included. If the LRS ever trips (overload, or the overvoltage latch,
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
- **Fuse:** the inlet's fuse drawer, under its C14 bay, is reached from outside through the end wall's opening (cord unplugged).
- **Heat:** the motor is inside and the PCB sits above it. Default run current 3 A (~16 W in the coils), 1 A at standstill.
  PETG's glass transition is ~80 °C; the walls are ≥ 5.8 mm from the motor. On the bench, run it closed for 30 min:
  if the motor case passes ~60 °C or the PSU case passes ~60 °C (the LRS gives full power only up to 50 °C ambient;
  it runs at about a third of its load here), fit the 40 mm fan (exhaust), lower `MOTOR mA` in the menu (light feeds need far
  less), or reprint in ASA (needs an enclosure on the MK3S to print without warping).
- **Order** (on the bench, before the plate goes onto the mill; the box goes on first, the module and the tray go in
  from below, the PCB from above):
  1. Plate: stub bearings and stub, encoder. Put the 4 module bolts (M5×35) into the plate's slots from the chamber side
     now, tips just flush with the outer face (on the mill there is no room for them: the chamber is 35 deep). Shell onto
     the plate edges (back and front screws), heat-set inserts, door on its hinge (pin through the knuckles along the
     back), hanging open.
  2. **Clutch/motor module up through the bottom**: its bearing boss and output shaft slide up the plate's slot until the
     cover's front face is on the plate (checked: nothing in the way on the whole lift). Run the 4 bolts into the cover,
     then the 20T pulley, belt and guard in the chamber.
  3. **Tension block up from below**: its tongue slides up into the slot and closes it; 2× M5 into the plate's legs,
     drawbolt into the cover. Tension the belt, lock the 4 module bolts.
  4. Mains tray, fully built on the bench (PSU in, **inlet clipped into its end face**, earth stud, L/N/PE tied, 36 V pair
     out through the grommet, lid on): **up through the bottom** with the inlet in place (checked, flange included),
     tabs into the plate's rebates (a serrated washer on each tab's M4 hole first), 2× M4 up into the plate. Feed the 36 V pair up beside the motor.
     Recommended extra: a green/yellow lead from the tray's earth stud to an M4 tapped hole in the plate's outer face,
     so the tray stays bonded even with its two screws loose.
  5. Hood off: PCB onto its bosses; plug the 36 V pair into J1, the motor, clutch and encoder leads behind the motor tail.
  6. Limit-switch cable through the end cap's gland; USB-C extension, BOOT/RESET and SWD wires from the end cap to the
     PCB (the cap stays off, beside the box, on its wires).
  7. Fan (optional; it covers the H1 hole's tool path, so after the PCB); hood with its controls.
  8. Close the door: its screw on the front side. Then slide the end cap on along the table axis (its floor under the
     tray's end, its scarf under the door's end) and fit its 3 screws through the shell's walls. Earth test (above)
     before the first plug-in. Then the plate onto the mill.
  - **Service on the mill, from below:** **unplug the mains cord first** (the tray hangs on it otherwise). The end
    cap's 3 screws (slide it off; it hangs on its wires), the door's screw, and the door swings down to the back: the PSU's screws, the belt
    tension, and the mains tray (unplug J1, 2 screws: it drops out with its inlet). **Repeat the earth test every time
    the tray has been out.**
  - **The module can't come out on the mill:** its 20T pulley sits inside the belt guard, which is trapped between the
    plate and the table. Take the plate off the table first, then: door open, tray out, tension block off, belt off, back
    the 4 bolts out of the cover (they stay in the plate), and the module comes down out of the slot.
- **Possible next size cut:** the floor is set by the belt-tension block under the clutch; behind it (from W ≈ 130)
  the floor could step up ~35 mm, under the motor. Not done yet.

## PLACEHOLDERS (measure, then edit `F`)
- `screw_d`, `screw_end_slot_w`, `screw_end_slot_d`, `screw_end_z`: the left leadscrew end and its drive slot.
  - If there is no slot, use a pinned sleeve coupling instead of the tongue.
- `standoffs`: where M6 can go into the table end or the left bracket.
- `motor_angle_deg`: rotate the module if it hits the saddle, the base or the DRO cable at full travel.
- The housing reaches {X_FP + 22 - 90:.0f} mm in front of the table's front edge (the dial knob, 22 proud, included) and {-(Y0 - T):.0f} mm below the
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
