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
for i, b in enumerate(C.bushes()): static["bush%d" % i] = b
def moving(released):
    dz = p.stroke if released else 0.0
    m = {"armature": Pos(0, 0, dz) * C.armature()}
    springs, washers, clips = C.return_springs(released)
    for k, pin in enumerate(C.drive_pins(released)):
        m["pin%d" % k] = pin; m["spring%d" % k] = springs[k]; m["washer%d" % k] = washers[k]; m["clip%d" % k] = clips[k]
    return m
# intended contacts (press fits / clamped faces), reported but not failures
ok = {("shaft", "brg0"), ("shaft", "brg1"), ("shaft", "hub"), ("rotor", "motor"), ("housing", "motor"), ("cover", "brg0"),
      ("cover", "brg1"), ("brg0", "brg2"), ("brg1", "brg2"), ("brg0", "brg3"), ("hub", "brg3"), ("shaft", "brg3"), ("cover", "brg2"),
      ("housing", "cover"), ("housing", "coil")}
for k in range(3):                  # pressed pins and bushes, the pin sliding in its bush, spring/washer/clip stack-up
    ok |= {("pin%d" % k, "armature"), ("pin%d" % k, "bush%d" % k), ("bush%d" % k, "hub"), ("spring%d" % k, "hub"),
           ("spring%d" % k, "bush%d" % k), ("spring%d" % k, "washer%d" % k), ("washer%d" % k, "clip%d" % k),
           ("clip%d" % k, "pin%d" % k)}
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
    print("  armature<->hub   min dist %.3f  (released: resting on its stop = 0)" % arm.distance_to(allp["hub"]))
    tips = Compound(children=[allp["pin%d" % k] for k in range(3)] + [allp["clip%d" % k] for k in range(3)])
    print("  pins+clips<->cover %.2f, <->bearings %.2f, <->rotor %.2f" % (tips.distance_to(allp["cover"]),
          min(tips.distance_to(allp["brg%d" % i]) for i in range(4)), tips.distance_to(allp["rotor"])))
    sl = allp["washer0"].bounding_box().min.Z - allp["hub"].bounding_box().min.Z - p.hub_flange_t
    print("  spring length %.2f mm (solid <= 5), force %.1f N each" % (sl, p.spring_rate * (p.spring_free - sl)))
from build123d import Box, Cylinder as Cyl, Align as Al
pilot_flange = Pos(0, 0, -10) * Box(57, 57, 10, align=(Al.CENTER, Al.CENTER, Al.MIN)) + Cyl(p.pilot_d / 2, p.pilot_h, align=(Al.CENTER, Al.CENTER, Al.MIN))
print("rotor<->motor pilot/flange dist %.2f (the shaft itself sits in the bore)" % C.rotor().distance_to(pilot_flange))
print("hub start z", p.hub_start, "pilot h", p.pilot_h)
