"""Electromagnetic tooth clutch for the X power feed: parametric build123d model.

Run:  mechanical/.venv/bin/python mechanical/clutch/clutch.py
Out:  mechanical/clutch/out/*.step|*.stl (one file per part + assembly) and preview SVGs.

Axis = Z. Motor flange face at z = 0, the motor is on the -Z side, output toward +Z.
The assembly is modelled ENGAGED (coil on). Released, the armature, springs' free ends and
nothing else move +STROKE along Z.

Stack (see docs/design.md §7):
  field housing (steel, stationary, bolted to the motor face, centred on the motor pilot)
  coil + bobbin inside it
  rotor (steel) on the motor shaft: hub inside the housing bore + pole face with banana slots
        and an outer tooth ring
  armature (steel) with mating teeth, held by 3 tangential leaf springs to
  output hub (steel or alu) on the output shaft (12 mm), which runs in two 6001 bearings in
  cover (aluminium, non-magnetic), registered on the housing OD.
"""

import math
import os
from dataclasses import dataclass

from build123d import (Align, Axis, Box, Compound, Cylinder, ExportSVG, LineType, Location, Plane, Polyline,
                       Pos, Rot, Solid, Unit, export_step, export_stl, extrude, loft, make_face)

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")
MIN = (Align.CENTER, Align.CENTER, Align.MIN)


@dataclass
class P:
    # motor (STEPPERONLINE 23HS45, NEMA 23)
    shaft_d: float = 10.0
    shaft_flat: float = 9.5          # across-flat dimension of the D-cut
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
    slot_span_deg: float = 100.0
    # teeth
    teeth: int = 30
    tooth_r_in: float = 32.0
    tooth_r_out: float = 38.0
    tooth_h: float = 0.9             # tooth height (tip clearance 0.1 to the mating root)
    flank_deg: float = 30.0          # flank angle from the axis, at the mean radius
    engaged_gap: float = 0.15        # pole-face gap when meshed
    stroke: float = 1.1              # armature travel; released pole gap = engaged_gap + stroke
    # armature + springs + hub
    armature_t: float = 6.0
    armature_bore: float = 11.5
    spring_r: float = 31.0
    spring_twist_deg: float = 40.0
    spring_w: float = 8.0
    spring_t: float = 0.3
    spacer_h: float = 3.5
    hub_flange_od: float = 74.0
    hub_flange_t: float = 6.0
    hub_boss_od: float = 24.0
    hub_boss_len: float = 10.0
    out_shaft_d: float = 12.0
    out_shaft_len: float = 70.0
    # cover
    cover_wall: float = 4.0
    cover_clear: float = 2.5         # radial clearance to the rotating parts
    bearing_od: float = 28.0         # 6001-2RS: 12 x 28 x 8
    bearing_w: float = 8.0
    bearing_gap: float = 8.0


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


def tooth_ring(z_root, direction, phase):
    """Radial face teeth. Angular trapezoid profile; tips point along `direction` (+1 or -1)."""
    pitch = 2 * math.pi / p.teeth
    r_mean = (p.tooth_r_in + p.tooth_r_out) / 2
    flank = p.tooth_h * math.tan(math.radians(p.flank_deg)) / r_mean      # angular flank run
    land = (pitch - 2 * flank) / 2                                        # equal tip and root lands
    solids = []
    z_tip = z_root + direction * p.tooth_h
    for k in range(p.teeth):
        c = phase + k * pitch
        base = sector_face(p.tooth_r_in, p.tooth_r_out, c - land / 2 - flank, c + land / 2 + flank, z_root, n=4)
        tip = sector_face(p.tooth_r_in, p.tooth_r_out, c - land / 2, c + land / 2, z_tip, n=4)
        solids.append(loft([base, tip]))
    out = solids[0]
    for s in solids[1:]:
        out = out + s
    return out


# ------------------------------------------------------------------ key Z levels (engaged)
Z_FACE_BACK = p.housing_len + p.face_gap            # rotor face, motor side
Z_POLE = Z_FACE_BACK + p.face_t                      # rotor pole face
Z_ARM = Z_POLE + p.engaged_gap                       # armature pole face (engaged)
Z_ROTOR_ROOT = Z_POLE - 0.5
Z_ARM_ROOT = Z_ARM + 0.35
Z_ARM_FRONT = Z_ARM + p.armature_t
Z_SPRING = Z_ARM_FRONT + p.spacer_h
Z_HUB = Z_SPRING + p.spring_t


# ------------------------------------------------------------------ parts
def field_housing():
    s = ring(p.housing_od / 2, p.bore_d / 2, p.housing_len)
    s = s - Pos(0, 0, p.back_plate) * Cylinder(p.coil_r_out, p.housing_len, align=MIN)
    s = s + ring(p.coil_r_in, p.bore_d / 2, p.housing_len)                  # inner pole wall
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
    # 3x radial M4 for the cover
    for k in range(3):
        s = s - Rot(0, 0, 120 * k) * Pos(p.housing_od / 2 - 4, 0, 14) * Rot(0, 90, 0) * Cylinder(1.65, 10)
    return s


def coil():
    b = ring(p.coil_r_out - 0.3, p.coil_r_in + 0.3, p.housing_len - p.back_plate - 0.5, p.back_plate + 0.3)
    return b


def rotor():
    s = ring(p.hub_od / 2, 0, Z_FACE_BACK - 1.0, 1.0)                       # hub inside the housing bore
    s = s + ring(p.rotor_od / 2, 0, p.face_t, Z_FACE_BACK)                  # pole face disc
    # tooth ring: cut down to the root level, then add the teeth back
    s = s - ring(p.tooth_r_out + 1, p.tooth_r_in, Z_POLE - Z_ROTOR_ROOT + 1, Z_ROTOR_ROOT)
    s = s + tooth_ring(Z_ROTOR_ROOT, +1, 0.0)
    for k in range(p.slot_count):                                           # banana slots
        a0 = math.radians(k * 360 / p.slot_count)
        s = s - sector_solid(p.slot_r_in, p.slot_r_out, a0, a0 + math.radians(p.slot_span_deg), Z_FACE_BACK - 0.1, Z_POLE + 0.1)
    # bore: D-shaft to the shaft end, then an M3 clearance + counterbore for the retaining screw
    bore = Cylinder(p.shaft_d / 2 + 0.01, p.shaft_len - 1.0, align=MIN)
    bore = bore - Pos(p.shaft_flat - p.shaft_d / 2 + 5, 0, 0) * Box(10, 20, 60, align=(Align.CENTER, Align.CENTER, Align.MIN))
    s = s - Pos(0, 0, 1.0) * bore
    s = s - Cylinder(1.7, Z_POLE + 1, align=MIN)
    s = s - Pos(0, 0, Z_POLE - 3.2) * Cylinder(2.9, 4, align=MIN)
    return s


def armature():
    s = ring(p.rotor_od / 2, p.armature_bore / 2, p.armature_t, Z_ARM)
    s = s - ring(p.tooth_r_out + 1, p.tooth_r_in, Z_ARM_ROOT - Z_ARM + 0.01, Z_ARM - 0.01)
    s = s + tooth_ring(Z_ARM_ROOT, -1, math.pi / p.teeth)                  # half a pitch out of phase
    for k in range(3):                                                     # M3 for the springs
        a = math.radians(90 + 120 * k)
        s = s - Pos(p.spring_r * math.cos(a), p.spring_r * math.sin(a), Z_ARM_FRONT - 6) * Cylinder(1.25, 6.1, align=MIN)
    return s


def spring_points(k):
    a = math.radians(90 + 120 * k)
    b = a + math.radians(p.spring_twist_deg)
    return (p.spring_r * math.cos(a), p.spring_r * math.sin(a)), (p.spring_r * math.cos(b), p.spring_r * math.sin(b))


def leaf_springs():
    parts = []
    for k in range(3):
        (ax, ay), (bx, by) = spring_points(k)
        length = math.hypot(bx - ax, by - ay) + p.spring_w
        ang = math.degrees(math.atan2(by - ay, bx - ax))
        strap = Pos((ax + bx) / 2, (ay + by) / 2, Z_SPRING) * Rot(0, 0, ang) * Box(length, p.spring_w, p.spring_t, align=MIN)
        for (x, y) in ((ax, ay), (bx, by)):
            strap = strap - Pos(x, y, Z_SPRING - 1) * Cylinder(1.65, 3, align=MIN)
        spacer = Pos(ax, ay, Z_ARM_FRONT) * ring(3.5, 1.65, p.spacer_h)
        parts += [strap, spacer]
    return parts


def output_hub():
    s = ring(p.hub_flange_od / 2, p.out_shaft_d / 2, p.hub_flange_t, Z_HUB)
    s = s + ring(p.hub_boss_od / 2, p.out_shaft_d / 2, p.hub_boss_len, Z_HUB + p.hub_flange_t)
    for k in range(3):
        (ax, ay), (bx, by) = spring_points(k)
        s = s - Pos(ax, ay, Z_HUB) * Cylinder(5.0, p.hub_flange_t, align=MIN)   # clearance/access over the armature end
        s = s - Pos(bx, by, Z_HUB) * Cylinder(1.25, p.hub_flange_t, align=MIN)  # M3 tapped, spring hub end
    s = s - Pos(0, 0, Z_HUB + p.hub_flange_t + p.hub_boss_len / 2) * Rot(0, 90, 0) * Cylinder(2.1, p.hub_boss_od)  # M5 set screw
    return s


def output_shaft():
    z0 = Z_HUB + 1.0
    s = Pos(0, 0, z0) * Cylinder(p.out_shaft_d / 2, p.out_shaft_len, align=MIN)
    tip = z0 + p.out_shaft_len
    s = s - Pos(p.out_shaft_d / 2 - 0.6 + 5, 0, tip - 22) * Box(10, 20, 22, align=(Align.CENTER, Align.CENTER, Align.MIN))  # pulley flat
    return s


def cover():
    r_in = p.rotor_od / 2 + p.cover_clear
    r_out = p.housing_od / 2 + p.cover_wall
    z_spigot = 10.0
    z_front = Z_HUB + p.hub_flange_t + p.hub_boss_len + 3.0
    s = ring(r_out, p.housing_od / 2 + 0.05, p.housing_len - z_spigot, z_spigot)    # spigot over the housing OD
    s = s + ring(r_out, r_in, z_front - p.housing_len, p.housing_len)
    s = s + ring(r_out, 0, 4.0, z_front)                                            # front wall
    boss_len = 2 * p.bearing_w + p.bearing_gap + 4
    s = s + ring(p.bearing_od / 2 + 5, 0, boss_len, z_front + 4)                    # bearing boss
    s = s - Cylinder(p.out_shaft_d / 2 + 1.0, 500, align=MIN)
    s = s - Pos(0, 0, z_front + 4 + boss_len - p.bearing_w) * Cylinder(p.bearing_od / 2, p.bearing_w + 1, align=MIN)
    s = s - Pos(0, 0, z_front + 2) * Cylinder(p.bearing_od / 2, p.bearing_w + 2, align=MIN)
    for k in range(3):
        s = s - Rot(0, 0, 120 * k) * Pos(r_out - 1, 0, 14) * Rot(0, 90, 0) * Cylinder(2.25, 10)
    return s, z_front


def bearings(z_front):
    boss_len = 2 * p.bearing_w + p.bearing_gap + 4
    zs = [z_front + 2 + 1, z_front + 4 + boss_len - p.bearing_w]
    return [ring(p.bearing_od / 2, p.out_shaft_d / 2, p.bearing_w, z) for z in zs]


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
    springs = leaf_springs()
    parts = {
        "field_housing": (field_housing(), "1018 steel (annealed)"),
        "coil_bobbin": (coil(), "~850 turns 0.32 mm enamelled Cu on a nylon/printed bobbin"),
        "rotor": (rotor(), "1018 steel (annealed)"),
        "armature": (armature(), "1018 steel (annealed)"),
        "leaf_springs": (Compound(children=springs[0::2]), "0.3 mm spring steel (blue shim), 3 pcs"),
        "spring_spacers": (Compound(children=springs[1::2]), "steel or brass, 3 pcs"),
        "output_hub": (output_hub(), "steel or 6061"),
        "output_shaft": (output_shaft(), "12 mm ground shaft"),
        "cover": (cov, "6061 aluminium (must be non-magnetic)"),
        "bearings": (Compound(children=bearings(z_front)), "2x 6001-2RS (12x28x8), purchased"),
    }
    for name, (shape, _) in parts.items():
        shape.label = name
        export_step(shape, os.path.join(OUT, name + ".step"))
        if name not in ("bearings",):
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


if __name__ == "__main__":
    parts, motor, z_front = build()
    for name, (shape, mat) in parts.items():
        bb = shape.bounding_box()
        print("%-15s %-58s  %.0f x %.0f x %.1f mm  vol %.1f cm3" % (name, mat, bb.size.X, bb.size.Y, bb.size.Z, shape.volume / 1000))
    preview(parts, motor)
    print("Z engaged pole gap %.2f, released %.2f mm; overall length from motor face %.1f mm"
          % (p.engaged_gap, p.engaged_gap + p.stroke, max(s.bounding_box().max.Z for s, _ in parts.values())))
