"""Clutch interference check, engaged and released: every pair of parts, plus the clearances that matter.
Expected: only rotor x armature (flank contact, tessellation-level volume) when engaged.

usage: mechanical/.venv/bin/python mechanical/tools/check_clutch.py
"""
import sys, itertools
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "clutch"))
import clutch as C
from build123d import Pos, Compound
p = C.p
cov, zf = C.cover()
static = {"housing": C.field_housing(), "coil": C.coil(), "rotor": C.rotor(), "hub": C.output_hub(),
          "shaft": C.output_shaft(), "cover": cov, "motor": C.motor_stub()}
for i, b in enumerate(C.bearings(zf)): static["brg%d" % i] = b
def moving(released):
    dz = p.stroke if released else 0.0
    m = {"armature": Pos(0, 0, dz) * C.armature()}
    sp = C.leaf_springs(released)
    for k in range(3):
        m["strap%d" % k] = sp[2 * k]; m["spacer%d" % k] = sp[2 * k + 1]
    return m
# intended contacts (press fits / clamped faces), reported but not failures
ok = {("shaft", "brg0"), ("shaft", "brg1"), ("shaft", "hub"), ("rotor", "motor"), ("housing", "motor"), ("cover", "brg0"),
      ("cover", "brg1"), ("brg0", "brg2"), ("brg1", "brg2"), ("brg0", "brg3"), ("hub", "brg3"), ("shaft", "brg3"), ("cover", "brg2"),
      ("housing", "cover"), ("housing", "coil")}
for released in (False, True):
    allp = dict(static); allp.update(moving(released))
    print("== %s" % ("RELEASED" if released else "ENGAGED"))
    for a, b in itertools.combinations(allp, 2):
        if (a, b) in ok or (b, a) in ok: continue
        try:
            v = (allp[a] & allp[b]).volume
        except Exception as e:
            v = -1
        if abs(v) > 1e-3:
            print("  %-10s x %-10s %.4f mm3" % (a, b, v))
    # clearances that matter
    arm = allp["armature"]
    print("  armature<->rotor min dist %.3f" % arm.distance_to(allp["rotor"]))
    print("  armature<->hub   min dist %.3f" % arm.distance_to(allp["hub"]))
from build123d import Box, Cylinder as Cyl, Align as Al
pilot_flange = Pos(0, 0, -10) * Box(57, 57, 10, align=(Al.CENTER, Al.CENTER, Al.MIN)) + Cyl(p.pilot_d / 2, p.pilot_h, align=(Al.CENTER, Al.CENTER, Al.MIN))
print("rotor<->motor pilot/flange dist %.2f (the shaft itself sits in the bore)" % C.rotor().distance_to(pilot_flange))
print("hub start z", p.hub_start, "pilot h", p.pilot_h)
# hub-end screw heads (DIN 7984 low head M3: Ø5.5 x 2.0) under the strap, vs the armature
from build123d import Cylinder, Align
for released in (False, True):
    arm = Pos(0, 0, p.stroke if released else 0.0) * C.armature()
    for k in range(3):
        bx, by = C.spring_points(k)[1]
        head = Pos(bx, by, C.Z_HUB - p.spring_t - 2.0) * Cylinder(2.75, 2.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
        print("released" if released else "engaged", "hub-screw head %d to armature %.2f" % (k, head.distance_to(arm)))
