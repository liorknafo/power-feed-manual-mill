"""Electromagnetic tooth clutch for the X power feed: parametric build123d model.

Run:  mechanical/.venv/bin/python mechanical/clutch/clutch.py
Out:  mechanical/clutch/out/*.step|*.stl (one file per part + assembly) and preview SVGs.

Axis = Z. Motor flange face at z = 0, the motor is on the -Z side, output toward +Z.
The assembly is modelled ENGAGED (coil on). Released, the armature with its drive pins, washers and E-clips moves
+STROKE along Z (back against the hub), and the return springs lengthen by STROKE.

Stack (see docs/design.md §7):
  field housing (steel, stationary, bolted to the motor face, centred on the motor pilot)
  coil + bobbin inside it
  rotor (steel) on the motor shaft: hub inside the housing bore + pole face with banana slots
        and an outer tooth ring
  armature (steel) with mating teeth and 3 drive pins bonded into it; the pins slide in bronze bushes in the
  output hub (6061) on the output shaft (12 mm), carry the torque, and 3 compression springs on them pull the
  armature back against the hub when the coil is off. The output shaft runs in two 6001 bearings in
  cover (aluminium, non-magnetic), registered on the housing OD.
"""

import math
import os
from dataclasses import dataclass

from build123d import (Align, Axis, Box, Compound, Cylinder, ExportSVG, LineType, GeomType, Location, Plane, Polyline,
                       Pos, Rot, Solid, Unit, chamfer, export_step, export_stl, extrude, loft, make_face)

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
MIN = (Align.CENTER, Align.CENTER, Align.MIN)


@dataclass
class P:
    # motor (STEPPERONLINE 23HS45, NEMA 23)
    shaft_d: float = 10.0
    shaft_flat: float = 9.5          # across-flat of the motor's D-cut (the rotor bore is round; see below)
    shaft_len: float = 21.0          # shaft length beyond the flange face (measure yours!)
    pilot_d: float = 38.1
    pilot_h: float = 1.6
    bolt_square: float = 47.14
    # field housing
    housing_od: float = 80.0
    housing_len: float = 20.0
    back_plate: float = 5.0
    bore_d: float = 19.2             # inner pole bore (rotor hub runs inside, 0.3 radial gap)
    coil_r_in: float = 14.0
    coil_r_out: float = 27.0
    # rotor
    hub_od: float = 18.6
    face_gap: float = 0.3            # fixed axial gap housing outer wall -> rotor face
    face_t: float = 5.6
    rotor_od: float = 76.0
    slot_r_in: float = 19.0          # banana slots over the coil separate the inner/outer poles
    slot_r_out: float = 22.0
    slot_count: int = 3
    slot_span_deg: float = 112.0     # 8° bridges: thin enough to saturate quickly (little flux shorted)
    # teeth
    teeth: int = 30
    tooth_r_in: float = 32.0
    tooth_r_out: float = 38.0
    tooth_h: float = 0.9             # full (un-truncated) tooth height; flanks meet at 0.9 root-to-root
    tip_trunc: float = 0.1           # tips cut 0.1 lower: the flanks carry the load
    root_relief: float = 0.5         # root floor 0.5 below where the flanks meet (flanks continue down to it): the mating
                                     # tip sits 0.6 above the floor, clear of the rougher's and the cone's corners
    tooth_runout: float = 1.0        # turned run-out groove 1 mm inside the teeth: the Ø2 rougher and the chamfer mill
                                     # (0.75 cone radius at the top) pass r 32 into it (costs ~12 % of the outer pole face)
    flank_deg: float = 30.0          # flank angle from the axis, at the mean radius
    engaged_gap: float = 0.15        # pole-face gap when meshed
    stroke: float = 1.1              # armature travel; released pole gap = engaged_gap + stroke
    # armature + drive pins + return springs + hub
    armature_t: float = 6.0
    armature_bore: float = 11.5
    pin_r: float = 21.0              # between the pole rings (r 15 / r 27): there the armature only carries flux radially
    pin_d: float = 6.0               # Ø6 h6 silver steel (unhardened), bonded (Loctite 638) into the armature with the
    pin_glue: float = 0.17           # pins set through the hub's bushes, so the bushes line them up (no binding); the glue
                                     # gap (638 fills up to 0.25) takes the hole-pattern and sleeve tolerances
    pin_press: float = 5.5           # through holes; the pin ends 0.5 inside the armature's back face (excess glue exits there)
    pin_len: float = 25.5
    bush_od: float = 10.0            # sintered bronze plain bush 6 x 10 x 8 (ISO 2795), pressed into the hub flange
    bush_len: float = 8.0
    spring_od: float = 9.0           # return spring: 0.8 wire, OD 9, free length ~11.9, ~2.5 N/mm
    spring_wire: float = 0.8
    spring_free: float = 11.9
    spring_rate: float = 2.5
    washer_t: float = 1.6            # ISO 7089 M6 washer Ø6.4 / Ø12
    eclip_t: float = 0.7             # DIN 6799 RS5 E-clip, groove Ø5 x 0.74
    hub_start: float = 2.2           # rotor hub starts above the motor pilot (1.6) with clearance
    hub_flange_od: float = 76.0      # = armature OD: one Ø76 sleeve over both centres the armature at assembly
    hub_flange_t: float = 8.7        # its armature-side face is the released stop; its top is where the springs sit
    hub_boss_od: float = 24.0
    hub_boss_len: float = 10.0
    out_shaft_d: float = 12.0
    out_shaft_len: float = 78.0
    # cover
    cover_wall: float = 4.0
    cover_clear: float = 2.5         # radial clearance to the rotating parts
    bearing_od: float = 28.0         # 6001-2RS: 12 x 28 x 8
    bearing_w: float = 8.0
    bearing_gap: float = 8.0
    cover_front_t: float = 8.0       # front wall carries 4x M5 for bolting the module to the feed-unit plate
    cover_bolt_r: float = 36.0
    # module bolts, local angles. Mounted on the feed unit the module is flipped (local +Y = away from the
    # leadscrew), so 40/140 are the two low bolts either side of the drawbolt (wide enough apart to clear the plate's
    # slot, open to its bottom edge, that the module slides up into) and 190/350 sit just above the axis, outside the
    # belt-guard outline with their heads.
    cover_bolt_deg: tuple = (40.0, 140.0, 190.0, 350.0)
    cover_spigot_z: float = 2.0      # spigot over the housing OD from here to the housing front (18 mm)
    cover_cavity_d: float = 78.0     # = shoulder on the housing rim; clears the Ø76 rotor/armature by 1 mm
    cover_screws: int = 6
    circlip_groove: float = 1.3      # DIN 472-28 groove width
    circlip_land: float = 2.1        # material outside each bore groove (DIN 472-28 minimum edge margin)


p = P()


# ------------------------------------------------------------------ helpers
def ring(r_out, r_in, h, z0=0.0):
    s = Cylinder(r_out, h, align=MIN)
    if r_in > 0:
        s = s - Cylinder(r_in, h, align=MIN)
    return Pos(0, 0, z0) * s


def sector_face(r_in, r_out, a0, a1, z, n=16):
    pts = [(r_out * math.cos(a0 + (a1 - a0) * i / n), r_out * math.sin(a0 + (a1 - a0) * i / n), z) for i in range(n + 1)]
    pts += [(r_in * math.cos(a1 - (a1 - a0) * i / n), r_in * math.sin(a1 - (a1 - a0) * i / n), z) for i in range(n + 1)]
    return make_face(Polyline(*pts, close=True))


def sector_solid(r_in, r_out, a0, a1, z0, z1):
    return extrude(sector_face(r_in, r_out, a0, a1, z0), amount=z1 - z0)


def arc_slot(r_in, r_out, a0, a1, z0, z1):
    """Curved slot as an end mill cuts it: true arcs, full-round ends (end radius = half the slot width)."""
    h = z1 - z0
    ring_ = Pos(0, 0, z0) * (Cylinder(r_out, h, align=MIN) - Cylinder(r_in, h, align=MIN))
    rc, rr = (r_in + r_out) / 2, (r_out - r_in) / 2
    da = rr / rc                                        # trim the straight part so the round ends land on a0, a1
    wedge = sector_solid(0.5, r_out + 5, a0 + da, a1 - da, z0 - 1, z1 + 1)
    slot = ring_ & wedge
    for a in (a0 + da, a1 - da):
        slot = slot + Pos(rc * math.cos(a), rc * math.sin(a), z0) * Cylinder(rr, h, align=MIN)
    return slot


def tooth_ring(z_root, direction, phase):
    """Radial face teeth with FLAT flanks, as a 60-degree chamfer mill cuts them on a 3-axis CNC.

    Every flank is the plane through a radial line at the PITCH plane (half-way between where the flanks meet on the
    rotor and on the armature, so it is the same plane on both parts when engaged), tilted `flank_deg` from the axis.
    A rotor gap and the armature tooth that fills it share those radial lines, so their flanks coincide exactly;
    both parts are cut by the same program. `z_root` is where adjacent flanks meet; the root floor is `root_relief`
    deeper; tips point along `direction` (+1 or -1) and are truncated by `tip_trunc`."""
    pitch = 2 * math.pi / p.teeth
    t = math.tan(math.radians(p.flank_deg))
    z_p = z_root + direction * p.tooth_h / 2                  # pitch plane: tooth width = gap width = half a pitch
    z_floor = z_root - direction * p.root_relief
    z_tip = z_root + direction * (p.tooth_h - p.tip_trunc)

    def section(c, z):
        """Tooth cross-section at height z: between the two flank traces, clipped to the tooth band."""
        o = (z - z_p) * direction * t                         # >0 above the pitch plane: the tooth gets narrower
        f1, f2 = c - pitch / 4, c + pitch / 4                 # flank radial lines at the pitch plane
        n1 = (-math.sin(f1), math.cos(f1))                    # toward the tooth centre
        n2 = (math.sin(f2), -math.cos(f2))

        def on(f, n, r):                                      # point on a flank trace at radius r
            tt = math.sqrt(r * r - o * o)
            return (tt * math.cos(f) + o * n[0], tt * math.sin(f) + o * n[1])

        def ang(pt):                                          # unwrapped near the tooth centre (no jump at +-pi)
            return c + math.remainder(math.atan2(pt[1], pt[0]) - c, 2 * math.pi)
        pts = [on(f1, n1, p.tooth_r_in), on(f1, n1, p.tooth_r_out)]
        a0, a1 = ang(pts[-1]), ang(on(f2, n2, p.tooth_r_out))
        pts += [(p.tooth_r_out * math.cos(a0 + (a1 - a0) * i / 4), p.tooth_r_out * math.sin(a0 + (a1 - a0) * i / 4)) for i in range(1, 4)]
        pts += [on(f2, n2, p.tooth_r_out), on(f2, n2, p.tooth_r_in)]
        a0, a1 = ang(pts[-1]), ang(pts[0])
        pts += [(p.tooth_r_in * math.cos(a0 + (a1 - a0) * i / 4), p.tooth_r_in * math.sin(a0 + (a1 - a0) * i / 4)) for i in range(1, 4)]
        return make_face(Polyline(*[(x, y, z) for x, y in pts], close=True))

    out = None
    for k in range(p.teeth):
        c = phase + k * pitch
        tooth = loft([section(c, z_floor), section(c, z_tip)])  # straight flank traces at both ends -> planar flanks
        out = tooth if out is None else out + tooth
    return out


# ------------------------------------------------------------------ key Z levels (engaged)
Z_FACE_BACK = p.housing_len + p.face_gap            # rotor face, motor side
Z_POLE = Z_FACE_BACK + p.face_t                      # rotor pole face
Z_ARM = Z_POLE + p.engaged_gap                       # armature pole face (engaged)
Z_ROTOR_ROOT = Z_POLE - 0.5
Z_ARM_ROOT = Z_ROTOR_ROOT + p.tooth_h                # flanks meet when the roots are tooth_h apart
Z_ARM_FRONT = Z_ARM + p.armature_t
Z_HUB = Z_ARM_FRONT + p.stroke                       # hub underside = the released stop: the armature rests on it
Z_SHAFT0 = Z_HUB + p.hub_flange_t - 5.0              # output shaft's motor-side end, inside the hub bore
Z_PIN_END = Z_ARM_FRONT - p.pin_press + p.pin_len    # drive pins' free end (engaged)
SET_SCREW_DEG = (30.0, 150.0)                        # hub set screws, midway between the pins at 90/210/330
SPRING_LEN_RELEASED = Z_PIN_END + p.stroke - p.eclip_t - 1.0 - p.washer_t - (Z_HUB + p.bush_len)   # seated on the bush


Z_COVER_SCREW = (p.cover_spigot_z + p.housing_len) / 2 + 1.0


# ------------------------------------------------------------------ parts
def field_housing():
    s = ring(p.housing_od / 2, p.bore_d / 2, p.housing_len)
    # 0.5 x 45 edge breaks on the front face's OD and bore: they clear the lathe-nose radius in the cover's spigot/
    # shoulder corner and in the rotor's hub-to-disc corner (both turned, both <= 0.4)
    top = s.edges().filter_by(GeomType.CIRCLE).group_by(Axis.Z)[-1]
    s = chamfer(top, 0.5)
    s = s - Pos(0, 0, p.back_plate) * Cylinder(p.coil_r_out, p.housing_len, align=MIN)
    s = s + ring(p.coil_r_in, p.bore_d / 2, p.housing_len)                  # inner pole wall
    bore_edge = [e for e in s.edges().filter_by(GeomType.CIRCLE)
                 if abs(e.center().Z - p.housing_len) < 1e-6 and abs(e.radius - p.bore_d / 2) < 1e-3]
    s = chamfer(bore_edge, 0.5)                                             # clears the rotor's hub-to-disc corner
    s = s - Cylinder(p.pilot_d / 2 + 0.1, p.pilot_h + 0.4, align=MIN)       # locates on the motor pilot
    for k in range(4):                                                     # M5 through the motor flange
        a = math.radians(45 + 90 * k)
        r = p.bolt_square / math.sqrt(2)
        x, y = r * math.cos(a), r * math.sin(a)
        s = s - Pos(x, y, 0) * Cylinder(2.75, p.housing_len, align=MIN)
        s = s - Pos(x, y, 6.0) * Cylinder(4.5, p.housing_len, align=MIN)    # counterbore, head below the pole face
    # coil lead exit: hole through the back plate + radial groove on the motor side
    s = s - Pos(20.5, 0, 0) * Cylinder(1.6, p.back_plate + 1, align=MIN)
    s = s - Pos(20.5 + 10, 0, 0.75) * Box(20, 3.2, 1.5)
    # radial M4 for the cover spigot
    for k in range(p.cover_screws):
        s = s - Rot(0, 0, 360 / p.cover_screws * k + 30) * Pos(p.housing_od / 2 - 4, 0, Z_COVER_SCREW) * Rot(0, 90, 0) * Cylinder(1.65, 10)
    return s


def coil():
    b = ring(p.coil_r_out - 0.3, p.coil_r_in + 0.3, p.housing_len - p.back_plate - 0.5, p.back_plate + 0.3)
    return b


def rotor():
    s = ring(p.hub_od / 2, 0, Z_FACE_BACK - p.hub_start, p.hub_start)      # hub inside the housing bore
    s = s + ring(p.rotor_od / 2, 0, p.face_t, Z_FACE_BACK)                  # pole face disc
    # tooth ring: cut down to the root level, then add the teeth back
    s = s - ring(p.tooth_r_out + 1, p.tooth_r_in - p.tooth_runout, Z_POLE - Z_ROTOR_ROOT + p.root_relief + 1,
                 Z_ROTOR_ROOT - p.root_relief)
    s = s + tooth_ring(Z_ROTOR_ROOT, +1, 0.0)
    for k in range(p.slot_count):                                           # banana slots
        a0 = math.radians(k * 360 / p.slot_count)
        s = s - arc_slot(p.slot_r_in, p.slot_r_out, a0, a0 + math.radians(p.slot_span_deg), Z_FACE_BACK - 0.1, Z_POLE + 0.1)
    # bore: plain round Ø10 H7 to the shaft end (Loctite 641 on the motor's D-shaft: >10 N·m vs 3 N·m,
    # and removable without heating the whole motor), then M3 clearance + counterbore for the axial
    # screw that pulls the rotor's shoulder onto the shaft end. Bore depth = MEASURED shaft length -
    # hub_start: that is what sets the 0.3 mm face gap.
    s = s - Pos(0, 0, p.hub_start) * Cylinder(p.shaft_d / 2 + 0.005, p.shaft_len - p.hub_start, align=MIN)
    s = s - Cylinder(1.7, Z_POLE + 1, align=MIN)
    s = s - Pos(0, 0, Z_POLE - 3.2) * Cylinder(2.9, 4, align=MIN)
    return s


def armature():
    s = ring(p.rotor_od / 2, p.armature_bore / 2, p.armature_t, Z_ARM)
    s = s - ring(p.tooth_r_out + 1, p.tooth_r_in - p.tooth_runout, Z_ARM_ROOT + p.root_relief - Z_ARM + 0.01, Z_ARM - 0.01)
    s = s + tooth_ring(Z_ARM_ROOT, -1, math.pi / p.teeth)                  # half a pitch out of phase
    for x, y in pin_points():                                              # drive pins, bonded in
        s = s - Pos(x, y, Z_ARM - 0.01) * Cylinder((p.pin_d + p.pin_glue) / 2, p.armature_t + 0.02, align=MIN)  # through
    return s


def pin_points():
    return [(p.pin_r * math.cos(math.radians(90 + 120 * k)), p.pin_r * math.sin(math.radians(90 + 120 * k)))
            for k in range(3)]


def drive_pins(released=False):
    """3 drive pins (bonded into the armature, move with it), each with an E-clip groove 1 mm from its end."""
    dz = p.stroke if released else 0.0
    out = []
    for x, y in pin_points():
        pin = Pos(x, y, Z_ARM_FRONT - p.pin_press + dz) * Cylinder(p.pin_d / 2, p.pin_len, align=MIN)
        pin = pin - Pos(x, y, Z_PIN_END + dz - 1.0 - 0.74) * ring(p.pin_d / 2 + 1, 2.5, 0.74)
        out.append(pin)
    return out


def return_springs(released=False):
    """Per pin: compression spring (bush end -> washer), M6 washer, E-clip. The washer and clip ride on the pin, so the
    spring is SPRING_LEN_RELEASED long when released and STROKE shorter when engaged: it always pulls the armature
    back toward the hub."""
    dz = p.stroke if released else 0.0
    z_clip = Z_PIN_END + dz - 1.0 - p.eclip_t
    z_wash = z_clip - p.washer_t
    z_hub = Z_HUB + p.bush_len                    # spring seated on the bush end, in the 0.7 counterbore it leaves
    springs, washers, clips = [], [], []
    for x, y in pin_points():
        springs.append(Pos(x, y, 0) * ring(p.spring_od / 2, p.spring_od / 2 - p.spring_wire, z_wash - z_hub, z_hub))
        washers.append(Pos(x, y, 0) * ring(6.0, 3.2, p.washer_t, z_wash))
        clips.append(Pos(x, y, 0) * ring(5.5, 2.5, p.eclip_t, z_clip))
    return springs, washers, clips


def output_hub():
    s = ring(p.hub_flange_od / 2, p.out_shaft_d / 2, p.hub_flange_t, Z_HUB)
    s = s + ring(p.hub_boss_od / 2, p.out_shaft_d / 2, p.hub_boss_len, Z_HUB + p.hub_flange_t)
    for x, y in pin_points():                     # Ø10 H7 for the bronze bushes, flush with the stop face
        s = s - Pos(x, y, Z_HUB) * Cylinder(p.bush_od / 2, p.hub_flange_t, align=MIN)
    zs = Z_HUB + p.hub_flange_t + p.hub_boss_len / 2
    for a in SET_SCREW_DEG:  # 2x M5 set screws midway between the pins (a hex key reaches them), one on the shaft flat
        s = s - Pos(0, 0, zs) * Rot(0, 0, a) * Pos(p.hub_boss_od / 4, 0, 0) * Rot(0, 90, 0) * Cylinder(2.1, p.hub_boss_od / 2)
    return s


def bushes():
    """Sintered bronze plain bushes, pressed into the hub flange flush with its stop face (armature side); the 0.7 left
    on the spring side is a counterbore that locates each spring."""
    z0 = Z_HUB
    return [Pos(x, y, 0) * ring(p.bush_od / 2, p.pin_d / 2, p.bush_len, z0) for x, y in pin_points()]


def output_shaft():
    z0 = Z_SHAFT0
    s = Pos(0, 0, z0) * Cylinder(p.out_shaft_d / 2, p.out_shaft_len, align=MIN)
    tip = z0 + p.out_shaft_len
    flat = (Align.CENTER, Align.CENTER, Align.MIN)
    s = s - Pos(p.out_shaft_d / 2 - 0.6 + 5, 0, tip - 22) * Box(10, 20, 22, align=flat)                 # pulley flat
    zh = Z_HUB + p.hub_flange_t + p.hub_boss_len / 2                     # hub flat: spot-faced with a Ø10 end mill
    s = s - Rot(0, 0, SET_SCREW_DEG[0]) * Pos(p.out_shaft_d / 2 - 0.6, 0, zh) * Rot(0, 90, 0) * Cylinder(p.hub_boss_len / 2, 10, align=MIN)
    zg = bearing_zs()[0] + p.bearing_w + 0.1          # DIN 471-12 groove: outboard of the inner (locating) bearing
    s = s - Pos(0, 0, zg) * (Cylinder(p.out_shaft_d / 2 + 1, 1.1, align=MIN) - Cylinder(11.5 / 2, 1.1, align=MIN))
    return s


Z_FRONT = Z_HUB + p.hub_flange_t + p.hub_boss_len + 3.0
BOSS_LEN = 2 * p.bearing_w + p.bearing_gap + 4


def bearing_zs():
    """Bearing seats in the straight-through Ø28 bore: inner (locating via the stepped spacer) and outer."""
    z_bore0 = Z_FRONT
    z_bore1 = Z_FRONT + p.cover_front_t + BOSS_LEN
    inset = p.circlip_land + p.circlip_groove + 0.2
    return [z_bore0 + inset, z_bore1 - inset - p.bearing_w]


def cover():
    r_out = p.housing_od / 2 + p.cover_wall
    z_front = Z_FRONT
    s = ring(r_out, p.housing_od / 2, p.housing_len - p.cover_spigot_z, p.cover_spigot_z)   # 18 mm spigot, slide fit
    s = s + ring(r_out, p.cover_cavity_d / 2, z_front - p.housing_len, p.housing_len)   # shoulder sits on the housing rim
    s = s + ring(r_out, 0, p.cover_front_t, z_front)                                       # front wall
    s = s + ring(p.bearing_od / 2 + 5, 0, BOSS_LEN, z_front + p.cover_front_t)             # bearing boss
    # straight-through Ø28 bore (one chucking with the spigot), DIN 472-28 circlip grooves at both ends
    z_bore1 = z_front + p.cover_front_t + BOSS_LEN
    s = s - Pos(0, 0, z_front) * Cylinder(p.bearing_od / 2, z_bore1 - z_front, align=MIN)
    for zg in (z_front + p.circlip_land, z_bore1 - p.circlip_land - p.circlip_groove):
        s = s - Pos(0, 0, zg) * Cylinder(29.4 / 2, p.circlip_groove, align=MIN)
    for k in range(p.cover_screws):
        s = s - Rot(0, 0, 360 / p.cover_screws * k + 30) * Pos(r_out - 1, 0, Z_COVER_SCREW) * Rot(0, 90, 0) * Cylinder(2.25, 10)
    for deg in p.cover_bolt_deg:  # M5 tapped (Ø4.2) into the front wall, for the feed-unit plate
        a = math.radians(deg)
        s = s - Pos(p.cover_bolt_r * math.cos(a), p.cover_bolt_r * math.sin(a), z_front) * Cylinder(2.1, p.cover_front_t, align=MIN)
    # radial M5 at +Y for the feed unit's belt-tension drawbolt (+Y = away from the leadscrew when mounted)
    s = s - Pos(0, r_out - 7, z_front + p.cover_front_t / 2) * Rot(-90, 0, 0) * Cylinder(1.65, 8, align=MIN)   # M4
    return s, z_front


def bearings(z_front):
    a, b = bearing_zs()
    parts = [ring(p.bearing_od / 2, p.out_shaft_d / 2, p.bearing_w, z) for z in (a, b)]
    parts.append(ring(p.bearing_od / 2 - 0.05, 12.5, b - (a + p.bearing_w), a + p.bearing_w))      # outer-race spacer tube
    hub_top = Z_HUB + p.hub_flange_t + p.hub_boss_len
    parts.append(ring(15.0 / 2, p.out_shaft_d / 2, a - hub_top, hub_top))                     # stepped inner-race spacer (shim here)
    return parts


def motor_stub():
    """Reference geometry only: NEMA 23 flange, pilot and shaft."""
    flange = Pos(0, 0, -10) * Box(57, 57, 10, align=MIN)
    pilot = Cylinder(p.pilot_d / 2, p.pilot_h, align=MIN)
    shaft = Cylinder(p.shaft_d / 2, p.shaft_len, align=MIN)
    shaft = shaft - Pos(p.shaft_flat - p.shaft_d / 2 + 5, 0, 3) * Box(10, 20, 60, align=(Align.CENTER, Align.CENTER, Align.MIN))
    return flange + pilot + shaft


# ------------------------------------------------------------------ build + export
def build():
    os.makedirs(OUT, exist_ok=True)
    cov, z_front = cover()
    springs, washers, clips = return_springs()
    parts = {
        "field_housing": (field_housing(), "1018 steel (annealed)"),
        "coil_bobbin": (coil(), "~850 turns 0.32 mm enamelled Cu on a nylon/printed bobbin"),
        "rotor": (rotor(), "1018 steel (annealed)"),
        "armature": (armature(), "1018 steel (annealed)"),
        "drive_pins": (Compound(children=drive_pins()), "Ø6 h6 silver steel x 25, 3 pcs"),
        "return_springs": (Compound(children=springs + washers + clips),
                           "3x compression spring 0.8 wire OD 9, 3x ISO 7089 M6 washer, 3x DIN 6799 RS5 E-clip"),
        "bushes": (Compound(children=bushes()), "3x sintered bronze plain bush 6 x 10 x 8"),
        "output_hub": (output_hub(), "6061"),
        "output_shaft": (output_shaft(), "12 mm ground shaft"),
        "cover": (cov, "6061 aluminium (must be non-magnetic)"),
        "bearings": (Compound(children=bearings(z_front)), "2x 6001-2RS + spacer tube + inner-race spacer, 2x DIN 472-28, 1x DIN 471-12"),
    }
    for name, (shape, _) in parts.items():
        shape.label = name
        export_step(shape, os.path.join(OUT, name + ".step"))
        export_stl(shape, os.path.join(OUT, name + ".stl"))
    motor = motor_stub()
    motor.label = "motor_reference"
    export_stl(motor, os.path.join(OUT, "motor_reference.stl"))
    assy = Compound(children=[s for s, _ in parts.values()] + [motor])
    assy.label = "clutch_assembly"
    export_step(assy, os.path.join(OUT, "clutch_assembly.step"))
    return parts, motor, z_front


def preview(parts, motor):
    """Isometric line drawing and a half-section side view."""
    shapes = [s for s, _ in parts.values()] + [motor]
    whole = Compound(children=shapes)
    views = {"iso": (Location((180, -220, 160)), whole)}
    cut = Box(400, 400, 400, align=(Align.CENTER, Align.MIN, Align.CENTER))
    halves = []
    for s in shapes:
        try:
            h = s - Pos(0, 0, 50) * cut
            halves.append(h)
        except Exception:
            pass
    views["section"] = (Location((0, -300, 40)), Compound(children=halves))
    for name, (loc, shape) in views.items():
        vis, hid = shape.project_to_viewport(loc.position, viewport_up=(0, 0, 1))
        svg = ExportSVG(scale=4.0)
        svg.add_layer("hidden", line_color=(170, 170, 170), line_type=LineType.ISO_DOT)
        svg.add_layer("visible", line_weight=0.25)
        svg.add_shape(hid, layer="hidden")
        svg.add_shape(vis, layer="visible")
        svg.write(os.path.join(OUT, "preview_%s.svg" % name))


def dimension_sheet(z_front):
    """Machining data for the turned parts, generated from the same parameters as the model."""
    pitch_deg = 360 / p.teeth
    zb = bearing_zs()
    L = []
    w = L.append
    w("# Clutch — machining sheet (generated by mechanical/clutch/clutch.py)\n")
    w("All dimensions mm. **Bold** = critical for concentricity or the magnetic gaps: turn these in ONE chucking.")
    w("Concentricity budget: every bold diameter ≤ 0.03 TIR to its part's reference bore. After assembly, indicate the")
    w("rotor OD and the output shaft: ≤ 0.05 TIR each, or the 0.3 mm hub gap and the tooth mesh suffer.")
    w("Magnetic parts: low-carbon steel (1018/1020/12L14), anneal after machining if possible. Never 300-series stainless.\n")
    w("## Field housing (steel) — `field_housing.step`")
    w("| Feature | Dimension | Tolerance / note |\n|---|---|---|")
    w("| OD | **Ø%.1f** | h7 (cover spigot fits over it) |" % p.housing_od)
    w("| Length | %.1f | ±0.05 — sets the 0.3 mm rotor face gap |" % p.housing_len)
    w("| Motor pilot recess (back) | **Ø%.1f (measure your motor) × %.1f deep** | slide fit on the pilot, ≤0.02 play; the model shows +0.2 |" % (p.pilot_d, p.pilot_h + 0.4))
    w("| Inner pole bore | **Ø%.1f** through | concentric with OD and pilot within 0.03 |" % p.bore_d)
    w("| Coil cavity | Ø%.1f … Ø%.1f, %.1f deep from the front | **CNC:** circular pocket with a Ø8–10 end mill round the inner pole (a boring bar can't reach the inner wall), or a face-grooving tool on the lathe. Cut the 4 Ø9 counterbores FIRST: only 1.8 mm of wall is left between each and the cavity |" % (2 * p.coil_r_in, 2 * p.coil_r_out, p.housing_len - p.back_plate))
    w("| Back plate | %.1f thick | |" % p.back_plate)
    w("| Motor screws | 4× Ø5.5 thru on %.2f square, c'bore Ø9 from front to z=6 | M5×30 SHCS + nuts behind the motor flange |" % p.bolt_square)
    w("| Coil leads | Ø3.2 thru at r=20.5 + 3.2×1.5 groove on the back face | |")
    w("| Cover screws | %d× M4 tapped radial, every %.0f°, %.1f from the back face (motor side) | no 4th axis needed: fit the cover, then drill Ø3.3 through the cover's holes into the housing, and tap |\n" % (p.cover_screws, 360 / p.cover_screws, Z_COVER_SCREW))
    w("## Rotor (steel) — `rotor.step`")
    w("| Feature | Dimension | Tolerance / note |\n|---|---|---|")
    w("| Hub OD | **Ø%.1f** × %.1f long | runs in the Ø%.1f bore: 0.3 radial gap, keep runout < 0.05. Starts %.1f above the motor flange, clear of the %.1f pilot |" % (p.hub_od, Z_FACE_BACK - p.hub_start, p.bore_d, p.hub_start, p.pilot_h))
    w("| Shaft bore | **Ø%.0f H7**, **flat bottom %.2f ± 0.03 in front of the face disc's back face** (that is what sets the 0.3 face gap); depth from the hub end ≈ %.1f with the model's shaft length | finish the bottom with a boring bar (a reamer can't reach it); measure your motor's shaft length beyond the flange, the bottom must sit at that length + %.1f from the housing back. Loctite 641 on the D-shaft |" % (p.shaft_d, p.shaft_len - Z_FACE_BACK, p.shaft_len - p.hub_start, p.face_gap + p.housing_len - p.housing_len))
    w("| Retaining screw | Ø3.4 thru + c'bore Ø5.8 × 3.2 from the front | M3×8 SHCS (3.0 head, sits 0.2 below the pole face) into the tapped motor shaft end (drill Ø2.5 × 8, tap M3) |")
    w("| Face disc | **Ø%.1f** × %.1f thick | back face sits %.1f from the housing front; front face = pole face |" % (p.rotor_od, p.face_t, p.face_gap))
    w("| Banana slots | %d × %.0f° arcs, r %.1f … %.1f, through (%.0f° bridges), full-round ends | CNC, Ø3 end mill on the r 20.5 arc (slot = tool width) |" % (p.slot_count, p.slot_span_deg, p.slot_r_in, p.slot_r_out, 360 / p.slot_count - p.slot_span_deg))
    w("| Tooth ring | r %.1f … %.1f, %d teeth (%.1f° pitch), flank %.0f° from axis | flanks meet 0.5 below the pole face; root floor %.1f deeper (%.1f below the pole face, from r %.1f out to the OD); tips %.2f above where the flanks meet; see Teeth |\n" % (p.tooth_r_in, p.tooth_r_out, p.teeth, pitch_deg, p.flank_deg, p.root_relief, 0.5 + p.root_relief, p.tooth_r_in - p.tooth_runout, p.tooth_h - p.tip_trunc))
    w("## Armature (steel) — `armature.step`")
    w("| Feature | Dimension | Tolerance / note |\n|---|---|---|")
    w("| OD × thickness | **Ø%.1f h7 × %.1f** | OD, bore and the tooth-ring face in one chucking (≤ 0.02 TIR): the OD centres the part for milling the teeth and the pin holes; faces parallel within 0.02 |" % (p.rotor_od, p.armature_t))
    w("| Bore | Ø%.1f | clearance over the rotor's M3 head |" % p.armature_bore)
    w("| Tooth ring | as the rotor, rotated half a pitch (%.1f°) | flanks meet %.2f behind the pole face, root floor %.2f behind it (from r %.1f out), tips truncated %.1f |" % (pitch_deg / 2, Z_ARM_ROOT - Z_ARM, Z_ARM_ROOT - Z_ARM + p.root_relief, p.tooth_r_in - p.tooth_runout, p.tip_trunc))
    w("| Drive-pin holes | 3× **Ø%.2f–%.2f through** (glue gap for Loctite 638) at r=%.1f, 90°/210°/330°, position ±0.02 from the bore, 0.3 × 45° chamfer at the front-face mouth (glue trap) | same program and zero as the hub's bush holes. The pins are bonded in place through the bushes (assembly step 0), so the bushes line them up; through, so excess glue leaves at the back face (r %.0f faces the rotor's slot ring, between the poles) |\n" % (p.pin_d + 0.15, p.pin_d + 0.20, p.pin_r, p.pin_r))
    w("## Output hub (6061) — `output_hub.step`")
    w("| Feature | Dimension | Note |\n|---|---|---|")
    w("| Flange | **Ø%.1f h7** × **%.1f** | its armature-side face is the **released stop** the springs pull the armature against: face it flat and square to the bore within 0.02, and turn the OD, in the same chucking as the bore (the OD, = the armature's, centres the armature when its pins are bonded) |" % (p.hub_flange_od, p.hub_flange_t))
    w("| Boss | Ø%.1f × %.1f, bore **Ø%.0f H7** | 2× M5 set screws, mid-boss, at %.0f° and %.0f° (midway between the pins, so a hex key reaches them past the springs): one on the shaft flat, one on the round (+ Loctite 243) |" % (p.hub_boss_od, p.hub_boss_len, p.out_shaft_d, SET_SCREW_DEG[0], SET_SCREW_DEG[1]))
    w("| Bush holes | 3× **Ø%.0f H7** through at r=%.1f, 90°/210°/330°, position ±0.02 from the bore | same program and zero as the armature's pin holes; press the bronze bushes in flush with the **stop face** (the bush bore closes to about F7 on the pins); the %.1f left on the spring side locates the springs |\n" % (p.bush_od, p.pin_r, p.hub_flange_t - p.bush_len))
    w("## Cover (6061 — must be non-magnetic) — `cover.step`")
    w("| Feature | Dimension | Tolerance / note |\n|---|---|---|")
    w("| OD | Ø%.1f | |" % (p.housing_od + 2 * p.cover_wall))
    w("| Spigot bore (over the housing) | **Ø%.1f H7** × %.1f | slide fit on the housing OD, same chucking as the bearing bore |" % (p.housing_od, p.housing_len - p.cover_spigot_z))
    w("| Shoulder / inner cavity | Ø%.1f | the step from the spigot sits on the housing rim and locates the cover axially; clears the Ø%.0f rotor / armature |" % (p.cover_cavity_d, p.rotor_od))
    w("| Bearing bore | **Ø%.0f H7**, straight through the %.0f wall + %.0f boss (≈ 89 mm from the open face) | two setups: (1) grip the spigot OD, turn the boss and the front, rough the bore to Ø26, cut the OUTER circlip groove from the outer end; (2) grip the Ø38 boss (not the 4 mm spigot wall), bore the spigot, the Ø78 cavity, the Ø28 H7 bore and the INNER groove |" % (p.bearing_od, p.cover_front_t, BOSS_LEN))
    w("| Circlip grooves | 2× for DIN 472-28: Ø29.4 × %.1f, **%.1f land** from each end of the bore | the land takes the circlip load: keep the entry chamfer smaller than it |" % (p.circlip_groove, p.circlip_land))
    w("| Radial screws | %d× Ø4.5 every %.0f°, %.1f from the open end | M4×8 into the housing; drilled together with the housing (see there) |" % (p.cover_screws, 360 / p.cover_screws, Z_COVER_SCREW - p.cover_spigot_z))
    w("| Front wall | %.0f thick, 4× M5 tapped thru on r=%.0f at %s° (0° = +X, 90° = drawbolt) | bolts the module to the feed-unit plate |" % (p.cover_front_t, p.cover_bolt_r, "/".join("%.0f" % a for a in p.cover_bolt_deg)))
    w("| Drawbolt hole | M4 tapped 7 deep, radial, in the front wall at mid-thickness, between two of the M5 (M4 leaves 2 mm of wall each side in the 8 mm wall) | feed-unit belt tension; it points away from the leadscrew |\n")
    w("## Other parts")
    w("- Output shaft: Ø%.0f h6 **unhardened** ground rod (NOT hardened linear shaft: the flats and groove can't be cut in it) × %.0f. Flats 0.6 deep: × 22 at the pulley end; under the hub boss a Ø%.0f spot face (plunge a Ø10 end mill). DIN 471-12 groove (Ø11.5 × 1.1) at %.1f from its motor-side end, just outboard of the inner bearing." % (p.out_shaft_d, p.out_shaft_len, p.hub_boss_len, zb[0] + p.bearing_w + 0.1 - Z_SHAFT0))
    f_rel = p.spring_rate * (p.spring_free - SPRING_LEN_RELEASED)
    f_eng = p.spring_rate * (p.spring_free - SPRING_LEN_RELEASED + p.stroke)
    w("- Drive pins: 3× Ø%.0f h6 silver steel (unhardened ground rod) × %.1f, 0.5 × 45° lead chamfer at the armature end, bonded %.1f deep into the armature with Loctite 638 (0.5 short of its back face). E-clip groove Ø5 × 0.74 (DIN 6799 RS5), 1.0 from the free end. They slide in the hub's bronze bushes, carry the torque (≈ %.0f N per pin at the 4.5 N·m cam-out torque) and keep the armature centred." % (p.pin_d, p.pin_len, p.pin_press, 4.5e3 / (3 * p.pin_r)))
    w("- Return springs: 3× compression spring, music wire %.1f, OD %.0f (ID %.1f: clears the pin, sits on the bush end), free length ≈ %.1f, rate ≈ %.1f N/mm, solid ≤ 5. Installed %.1f long released (≈ %.0f N each, %.0f N total: the armature is pulled firmly against the hub) and %.1f long engaged (≈ %.0f N each, %.0f N total). Any catalogue spring near this works; the coil current is calibrated afterwards. On each pin: spring, M6 washer (ISO 7089, Ø6.4/12 × 1.6), E-clip." % (p.spring_wire, p.spring_od, p.spring_od - 2 * p.spring_wire, p.spring_free, p.spring_rate, SPRING_LEN_RELEASED, f_rel, 3 * f_rel, SPRING_LEN_RELEASED - p.stroke, f_eng, 3 * f_eng))
    w("- Bushes: 3× sintered (oil-impregnated) bronze plain bush %.0f × %.0f × %.0f, ISO 2795 form C." % (p.pin_d, p.bush_od, p.bush_len))
    w("- Bearing stack (cavity side → outer end): circlip, inner 6001, spacer tube Ø27.95/Ø25 × %.1f (outer races), outer 6001, circlip. The INNER bearing locates the shaft both ways: the stepped spacer Ø15/Ø12 × %.1f (hub boss → inner race) on one side, the DIN 471-12 on the shaft on the other (inside the spacer tube); the outer bearing floats on the shaft." % (zb[1] - zb[0] - p.bearing_w, zb[0] - (Z_HUB + p.hub_flange_t + p.hub_boss_len)))
    w("- Assembly (the hub and armature must go on the shaft first: once inside the cover nothing on them is reachable):")
    w("  0. **Armature on the hub; the bushes line the pins up.** Glue must never reach a bush (the pin-to-bush gap is")
    w("     0.01–0.03; Loctite 638 there would lock the pin), so in this order:")
    w("     a. Press the 3 bushes into the hub from the spring side, the stop face on a flat plate, so they finish flush with it.")
    w("        Wax both ends of each bush (clean the wax off after curing).")
    w("     b. Hub on the shaft with the shaft end **%.2f inside the stop face** (depth micrometer), then the set screws." % (Z_SHAFT0 - Z_HUB))
    w("     c. Armature on its own: a thin film of 638 on the walls of its 3 holes only, keeping the 1.5 mm at the front-face")
    w("        (hub-side) mouth of each dry.")
    w("     d. Armature pole face down on PE film on a surface plate (it stands on its tooth tips, ~0.55 proud: that gap lets")
    w("        the glue out); the hub on it, stop face on the armature's front face, its bush holes lined up over the armature's")
    w("        holes by eye; then a **centring sleeve** over both (any tube bored Ø%.0f H7, ~25 long). The first pin's chamfer" % p.rotor_od)
    w("        finds the rest of the alignment.")
    w("     e. Each pin: a Ø6 shaft collar (OD 12–16: it must bridge the Ø10 bush counterbore and clear the Ø24 boss) set %.1f" % (p.hub_flange_t + p.pin_press))
    w("        from its chamfered end. Push it, dry, straight **down** through its bush into the hole until the collar lands on the")
    w("        hub's spring-side face, and never lift it: the tip then sits 0.5 inside the armature's back face. Excess glue is")
    w("        pushed out of the open bottom of the hole onto the PE film; the 0.3 mouth chamfer catches any that creeps up.")
    w("     f. Clamp lightly (on the hub boss or the shaft end, never on the teeth), let it cure (24 h), collars and sleeve off,")
    w("        scrape the back face flush: nothing may stand proud of it (the engaged pole gap is 0.15). Each pin was set by its")
    w("        own bush, so three pins in three bushes can't bind. Slide the armature off the bushes, clean the wax off both")
    w("        stop faces and the bush ends, slide it back.")
    w("     g. On each pin, from the hub's far side: spring (into the counterbore on the bush), washer, E-clip (compress the")
    w("        spring with a slotted tube). Check: the armature slides 1.1+ mm toward the motor by hand, freely, and snaps back.")
    w("        Keep the stop faces dry (degrease): oil from the bushes on them can make the armature stick to the hub a little.")
    w("  1. Inner circlip into the empty cover. 2. Inner 6001 in from the outer end until it seats on that circlip.")
    w("  3. Shaft with hub, armature, pins, springs and stepped spacer already fitted: push it through the inner bearing from the cavity side,")
    w("     supporting the inner race with a tube from the outer end (Loctite 641 on that seat).")
    w("  4. DIN 471-12 down the bore into its groove (~%.0f deep from the outer end) with a cone on the shaft end and a push tube." % (Z_FRONT + p.cover_front_t + BOSS_LEN - (zb[0] + p.bearing_w + 0.1)))
    w("  5. Spacer tube, outer 6001 (slides on), outer circlip.")
    a_rot, b_arm = Z_POLE - p.housing_len, Z_ARM + p.stroke - p.housing_len
    w("  6. **Released-gap check.** The springs hold the armature against the hub, so the released gap is set by where the hub")
    w("     sits: the stepped spacer (hub boss → inner race) sets it. The gap is out of reach once the cover is on, so measure two")
    w("     depths before closing, with a depth micrometer:")
    w("     A = rotor pole face above the field housing's front face (rotor glued on, motor shaft in): %.2f;" % a_rot)
    w("     B = cover shoulder face (the step that sits on the housing rim) down to the armature's pole face: %.2f." % b_arm)
    w("     B − A is the released gap: %.2f ± 0.10. Too small: grind the stepped spacer shorter; too big: shim it. After closing,"
      % (p.engaged_gap + p.stroke))
    w("     turning the output by hand with the coil off must not tick (no tooth contact); coil on (1.5 A) the output locks to")
    w("     the motor, coil off it frees at once.")
    w("- Coil: ~850 turns 0.32 mm (AWG 28) enamelled Cu on a bobbin Ø%.1f…Ø%.1f × %.1f; ≈24 Ω. Leads out through the back plate." % (2 * p.coil_r_in + 0.6, 2 * p.coil_r_out - 0.6, p.housing_len - p.back_plate - 0.5))
    w("- Bearings: 2× 6001-2RS in an H7 bore. Shaft is h6 ground rod: the inner (locating) bearing is retained with Loctite 641, the outer one is a slip fit so it can float. Screws: 4× M5×30 + nuts (motor), %d× M4×8 (cover), 1× M3×8 (rotor), 2× M5×6 set screws (hub), 4× M5×35 + washers (module to the 28 mm feed-unit plate), 1× M4×30 drawbolt." % p.cover_screws)
    w("- Coil leads: sleeve them through the back plate, and pot the winding in the cavity (epoxy) so it cannot chafe or move.")
    w("## Teeth")
    w("- %d radial teeth, flanks **flat planes** at %.0f° from the axis: every flank is the plane through a radial line at the pitch" % (p.teeth, p.flank_deg))
    w("  plane, so a 60° chamfer mill cuts it exactly in one straight pass, and the rotor's gaps and the armature's teeth share those")
    w("  lines: both parts come off the SAME program and mate exactly.")
    w("- Engaged pole gap %.2f, stroke %.1f -> released gap %.2f." % (p.engaged_gap, p.stroke, p.engaged_gap + p.stroke))
    w("- Tips truncated %.1f (break them 0.1 × 45° after), roots relieved %.1f below where the flanks meet: a mating tip sits %.1f above" % (p.tip_trunc, p.root_relief, p.tip_trunc + p.root_relief))
    w("  the root floor, so the flanks carry all the load and the root corners never touch.")
    zt_r, zt_a = (Z_ROTOR_ROOT + p.tooth_h - p.tip_trunc) - Z_POLE, Z_ARM - (Z_ARM_ROOT - p.tooth_h + p.tip_trunc)
    w("- **Turning (before milling), both parts:**")
    w("  - tooth band r %.0f → OD: turn to the TIP height: **%.2f proud of the rotor pole face, %.2f proud of the armature pole face**;" % (p.tooth_r_in, zt_r, zt_a))
    w("  - run-out groove r %.1f … %.0f: turn down to the root floor, %.1f below the tip height (the cutter runs out into it);" % (p.tooth_r_in - p.tooth_runout, p.tooth_r_in, p.tooth_h - p.tip_trunc + p.root_relief))
    w("  - face the pole face inside r %.1f as dimensioned." % (p.tooth_r_in - p.tooth_runout))
    w("- **Milling (3-axis CNC, no indexer needed):**")
    w("  - Hold: rotor by its hub in a collet block, teeth up; armature on a fixture plate with a Ø11.5 spigot in its bore and")
    w("    3 Ø6 dowels in its pin holes (drill those first), clamped by a centre bolt through the spigot with a Ø24 washer on")
    w("    the inner pole face. Centre both on the Ø%.0f OD." % p.rotor_od)
    w("  - **Z zero on the turned TIP surface** (same on both parts). Pitch plane = %.2f below it; root floor = %.2f below it." % (p.tooth_h / 2 - p.tip_trunc, p.tooth_h - p.tip_trunc + p.root_relief))
    w("  1. Rough each gap with a Ø2 flat end mill to the root floor, radial passes with the **cutter centre from r %.0f** out past the" % p.tooth_r_in)
    w("     OD (starting further in notches the inner pole face), leaving ~0.1 on the flanks (the floor gap is only 2.25 wide at r %.0f)." % p.tooth_r_in)
    w("  2. Finish each flank with a **60° chamfer mill** (pointed or ≤ 0.2 flat), one straight radial pass per flank, **tip at the root")
    w("     floor (Z −%.2f)** (anywhere at or below Z −%.2f still covers the working flank): the cone's side on the flank line at the" % (p.tooth_h - p.tip_trunc + p.root_relief, p.tooth_h - p.tip_trunc - 0.1))
    w("     pitch plane, offset by the cone radius at that depth. %d flanks per part, same program for both." % (2 * p.teeth))
    w("  3. Measure the chamfer mill's actual angle and point first: each 0.01 mm of flank error on both parts moves the armature 0.035")
    w("     axially. **Acceptance:** mate the pair dry and check the pole gap with feeler gauges: 0.15 ± 0.05.")
    w("- Lap the pair together with fine compound after cutting; re-check the gap.")
    w("- `flank_deg = 45` suits a standard 90° chamfer mill, but the cam-out force then rises ~1.8x (expect ~1.3x more hold current;")
    w("  re-calibrate per design.md §7.4).\n")
    w("## Corners (what the tools leave, and why it doesn't matter)")
    w("- Every internal corner along a cutter's axis is modelled round (slot ends, pockets) or is a drilled/counterbored hole.")
    w("  `mechanical/tools/check_machinable.py` checks this on every machined part.")
    w("- Turned internal corners get the tool's nose radius. Keep it **≤ 0.3** where a part seats in the corner:")
    w("  - cover: spigot bore → Ø78 shoulder (the housing's front OD edge is chamfered 0.5 × 45° to clear it);")
    w("  - field housing: coil cavity floor corners (the bobbin has 0.3 clearance; chamfer its corners) and the motor-pilot recess")
    w("    floor (the motor's pilot edge is chamfered);")
    w("  - rotor: hub → face-disc corner (the housing bore's front edge is chamfered 0.5 × 45° to clear it).")
    w("- Rotor shaft bore: the motor shaft's end seats on its bottom, so the bottom must be **flat** and is dimensioned from the")
    w("  face disc (see Rotor): drill under size, bore to H7 and finish the bottom with the boring bar (a reamer can't reach it).")
    w("  A drill point there would set the rotor ~3 mm wrong.\n")
    return "\n".join(L) + "\n"


if __name__ == "__main__":
    parts, motor, z_front = build()
    for name, (shape, mat) in parts.items():
        bb = shape.bounding_box()
        print("%-15s %-58s  %.0f x %.0f x %.1f mm  vol %.1f cm3" % (name, mat, bb.size.X, bb.size.Y, bb.size.Z, shape.volume / 1000))
    preview(parts, motor)
    with open(os.path.join(os.path.dirname(OUT), "MACHINING.md"), "w") as f:
        f.write(dimension_sheet(z_front))
    print("Z engaged pole gap %.2f, released %.2f mm; overall length from motor face %.1f mm"
          % (p.engaged_gap, p.engaged_gap + p.stroke, max(s.bounding_box().max.Z for s, _ in parts.values())))
