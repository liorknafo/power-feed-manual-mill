"""Feed-unit interference check (rebuilds feed_unit.py): every pair of parts except intended fits.
Expected: only the clutch flank contact (rotor x armature). Re-run after changing the F dataclass.

usage: mechanical/.venv/bin/python mechanical/tools/check_feed_unit.py
"""
import sys, itertools
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "feed_unit"))
import feed_unit as FU
from build123d import Pos, Rot
parts = FU.build()
ok = {("stub_shaft","bearings_6002"),("plate","bearings_6002"),("stub_shaft","pulley_40T"),("stub_shaft","oldham_disc"),
      ("oldham_disc","mill_reference"),("plate","standoffs"),("standoffs","mill_reference"),("plate","encoder_posts"),
      ("encoder_posts","encoder_disc"),("encoder","encoder_disc"),("encoder","encoder_coupling"),("stub_shaft","encoder_coupling"),
      ("pulley_20T","clutch_output_shaft"),("belt","pulley_40T"),("belt","pulley_20T"),("plate","tension_block"),
      ("plate","clutch_cover"),("clutch_field_housing","clutch_cover"),("clutch_field_housing","clutch_coil"),
      ("clutch_rotor","motor"),("clutch_field_housing","motor"),("clutch_hub","clutch_output_shaft"),
      ("clutch_cover","clutch_output_shaft"),("belt_guard","mill_reference"),("belt_guard","plate"),
      ("plate","module_bolt_heads"),
      ("clutch_rotor","clutch_armature")}  # engaged tooth-flank contact (tessellation-level volume)
names = [n for n in parts if not n.startswith("control_")]   # control_* are per-control copies of "controls" for the viewer
bad = 0
for a, b in itertools.combinations(names, 2):
    if (a, b) in ok or (b, a) in ok: continue
    try:
        v = (parts[a] & parts[b]).volume
    except Exception:
        v = -1
    if abs(v) > 1e-2:
        bad += 1; print("  %-22s x %-22s %.3f mm3" % (a, b, v))
for a, b in (("belt_guard","mill_reference"),("belt_guard","plate"),("oldham_disc","mill_reference"),("belt","plate")):
    try: v = (parts[a] & parts[b]).volume
    except Exception: v = -1
    print("  [contact] %s x %s %.3f" % (a, b, v))
print("pulley40 <-> oldham", parts["pulley_40T"].distance_to(parts["oldham_disc"]))
print("pulley40 <-> stub tongue zone / mill", parts["pulley_40T"].distance_to(parts["mill_reference"]))
print("encoder_disc <-> clutch_cover", parts["encoder_disc"].distance_to(parts["clutch_cover"]))
print("encoder <-> clutch_cover", parts["encoder"].distance_to(parts["clutch_cover"]))
print("tension_block <-> clutch_cover", parts["tension_block"].distance_to(parts["clutch_cover"]))
print("guard <-> belt", parts["belt_guard"].distance_to(parts["belt"]))
print("guard <-> pulley_40T", parts["belt_guard"].distance_to(parts["pulley_40T"]))
print("plate top / guard top", parts["plate"].bounding_box().max.Y, parts["belt_guard"].bounding_box().max.Y)
print("bad pairs", bad)
# module bolt heads across the whole tension slot (they move with the module)
for t in (-FU.f.tension_slot, 0.0, FU.f.tension_slot):
    heads = FU.bolt_heads(t)
    for name in ("belt_guard", "pulley_40T", "belt", "standoffs", "stub_shaft", "tension_block", "encoder_disc"):
        v = sum((h & parts[name]).volume for h in heads)
        d = min(h.distance_to(parts[name]) for h in heads)
        if v > 1e-3:
            bad += 1
        print("  heads t=%+.0f vs %-12s overlap %.3f mm3, min dist %.2f" % (t, name, v, d))
print("bad pairs incl. bolt heads", bad)
# housing clearances: the clutch module moves +-tension_slot along the belt centre line, the housing does not
housing_names = [n for n in parts if n in ("shell", "hood", "psu", "mains_inlet", "mains_tray", "mains_tray_lid")]
ux, uy = FU.math.cos(FU.ANG), FU.math.sin(FU.ANG)
moving = ("motor", "clutch_cover", "clutch_field_housing", "pulley_20T")
worst = {}
for t in (-FU.f.tension_slot, 0.0, FU.f.tension_slot):
    for m in moving:
        sh = Pos(ux * t, uy * t, 0) * parts[m]
        for h in housing_names:
            d = sh.distance_to(parts[h])
            if d < worst.get(m, (99, ""))[0]:
                worst[m] = (d, "%s at t=%+.0f" % (h, t))
for m, (d, where) in worst.items():
    print("  housing clearance %-22s %.2f mm (%s)" % (m, d, where))
    if d < 2.0:
        bad += 1
for a, b in (("pcb_placeholder", "controls"), ("controls", "shell"), ("encoder", "shell"), ("encoder", "hood"),
             ("tension_block", "shell"), ("clutch_cover", "shell"), ("motor", "shell"), ("encoder_disc", "hood"),
             ("pcb_placeholder", "hood"), ("pcb_placeholder", "clutch_field_housing")):
    print("  clearance %-16s <-> %-20s %.2f mm" % (a, b, parts[a].distance_to(parts[b])))
# each control body against everything around it
for nm, c in zip(FU.CONTROL_NAMES, FU.controls()):
    d = min(c.distance_to(parts[n]) for n in ("shell", "pcb_placeholder", "encoder", "encoder_disc", "plate"))
    print("  control %-14s nearest %.2f mm" % (nm, d))
    if d < 1.0 and nm != "estop_ring":                # the legend ring lies ON the front face, contact is right
        bad += 1
# the PCB rests ON its printed rails; it must clear the motor (THT leads ~3 mm) over the whole tension slot
print("  pcb on rails: overlap %.3f mm3" % (parts["pcb_placeholder"] & parts["shell"]).volume)
for t in (-FU.f.tension_slot, FU.f.tension_slot):
    d = (Pos(ux * t, uy * t, 0) * parts["motor"]).distance_to(parts["pcb_placeholder"])
    print("  pcb <-> motor at t=%+.0f: %.2f mm" % (t, d))
    if d < 3.0:
        bad += 1
# every print must be ONE connected solid (no floating rails/posts)
for n in ("shell", "hood"):
    k = len(parts[n].solids())
    print("  %-5s is %d solid(s) %s" % (n, k, "OK" if k == 1 else "<- FLOATING PIECES"))
    if k != 1:
        bad += 1
# nothing pokes through the outer skin (rails trimmed to it)
skin = FU._rounded(FU.XB - FU.T, FU.X_FP, FU.Y0 - FU.T, FU.Y1 + FU.T, FU.W_SHELL0, FU.W_END, FU.f.r_long, FU.f.r_long, FU.f.r_end,
                  r_bottom=FU.R_BOTTOM)
for n in ("shell", "hood"):
    v = (parts[n] - skin).volume
    print("  %-5s outside the skin: %.3f mm3" % (n, v))
    if v > 1e-2:
        bad += 1
# every heat-set insert must sit in solid material: ring Ø4..Ø8 round each insert hole, over its depth, >= 95 % full
from build123d import Plane, Solid
for part_, target, pt, d, kind in FU.screw_joints():
    if kind != "insert":
        continue
    start = tuple(pt[i] + d[i] * (FU.T + 0.05) for i in range(3))
    pl = Plane(origin=start, z_dir=d)
    ring = Solid.make_cylinder(4.0, FU.INSERT_L - 0.1, pl) - Solid.make_cylinder(FU.INSERT_D / 2 + 0.05, FU.INSERT_L - 0.1, pl)
    full = (ring & parts[target]).volume / ring.volume
    if full < 0.95:
        bad += 1
        print("  insert seat at (%.1f, %.1f, %.1f) only %.0f %% material <- FIX" % (pt[0], pt[1], pt[2], 100 * full))
print("  insert seats checked: %d" % sum(1 for j in FU.screw_joints() if j[4] == "insert"))
# mains separation: nothing but the PSU and the inlet may enter the earthed tray's inside, and the wiring zone
# between the PSU terminals and the inlet body (>= 35 mm) must be empty
t, hw = FU.f.tray_t, FU.TRAY_HALF
inside = FU.bx(-hw + t, hw - t, FU.TRAY_BOT + t, FU.TRAY_TOP, FU.TRAY_WALL_W0, FU.TRAY_W1 - t)   # the walled part
mains_ok = ("mains_tray", "mains_tray_lid", "psu", "mains_inlet", "shell", "plate")
for n in parts:
    if n in mains_ok or n.startswith("control_"):
        continue
    v = (parts[n] & inside).volume
    if v > 1e-3:
        bad += 1
        print("  MAINS ZONE INTRUSION: %s %.2f mm3" % (n, v))
w_term = FU.f.psu_w0 + FU.f.psu_l
w_inlet = FU.TRAY_W1 - t - FU.f.inlet_depth
zone = FU.bx(-hw + t, hw - t, FU.TRAY_BOT + t, FU.TRAY_TOP, w_term, w_inlet)
occ = sum((zone & parts[n]).volume for n in parts if n not in ("mains_tray", "mains_tray_lid") and not n.startswith("control_"))
print("  mains wiring zone %.1f mm long, occupied %.3f mm3" % (w_inlet - w_term, occ))
if w_inlet - w_term < 35 or occ > 1e-3:
    bad += 1
# tapped/drilled holes in the plate must not run into each other (screws are not in the solid models)
from build123d import Cylinder as _Cy, Align as _Al
def _hole(axis, x, y, w, r, depth):
    if axis == "W":    # into the plate's outer face, toward -W
        return Pos(x, y, w - depth) * _Cy(r, depth, align=(_Al.CENTER, _Al.CENTER, _Al.MIN))
    pl = Plane(origin=(x, y, w), z_dir={"+Y": (0, 1, 0), "-Y": (0, -1, 0), "+X": (1, 0, 0), "-X": (-1, 0, 0)}[axis])
    return Solid.make_cylinder(r, depth, pl)
holes = []
for xt in FU.TRAY_BOLTS_X:
    holes.append(("tray bolt x%+.0f" % xt, _hole("W", xt, FU.TRAY_FLANGE_Y, FU.Z_PLATE_OUT, 1.75, 10)))
for xt in (-15.0, 15.0):
    holes.append(("tension-block M5 x%+.0f" % xt, _hole("W", FU.MX + xt, FU.TB_Y, FU.Z_PLATE_OUT, 2.1, 12)))
for part_, target, pt, d, kind in FU.screw_joints():
    if kind == "tap":
        axis = {(0, 1, 0): "+Y", (0, -1, 0): "-Y", (1, 0, 0): "+X", (-1, 0, 0): "-X"}[tuple(d)]
        start = tuple(pt[i] + d[i] * FU.T for i in range(3))
        holes.append(("%s screw at (%.0f, %.0f)" % (part_, pt[0], pt[1]), _hole(axis, *start, 1.45, 8.5)))
clashes = 0
for (na, ha), (nb, hb) in itertools.combinations(holes, 2):
    inter = ha & hb
    if inter is not None and inter.volume > 1e-3:
        clashes += 1
        print("  HOLE CLASH: %s x %s" % (na, nb))
print("  plate holes checked: %d, clashes %d" % (len(holes), clashes))
bad += clashes
# PSU base screw heads (ISO 7380 M3) sit in their floor pockets
v = sum(getattr(h & parts["shell"], "volume", 0.0) or 0.0 for h in FU.psu_screw_heads())
print("  PSU screw heads vs shell: %.3f mm3" % v)
if v > 1e-3:
    bad += 1
# the snap-in inlet needs flat sheet all round its flange: >= bend (t) + 1.5 mm to the floor and to the lid fold
t_ = FU.f.tray_t; fw, fh = FU.f.inlet_flange
m_low = (FU.INLET_Y - fh / 2) - (FU.TRAY_BOT + t_)
m_top = (FU.TERM_TOP - t_) - (FU.INLET_Y + fh / 2)
print("  inlet flange flat margin: bottom %.1f, top %.1f mm (need >= %.1f)" % (m_low, m_top, t_ + 1.5))
if min(m_low, m_top) < t_ + 1.5:
    bad += 1
# print-bed fit (Prusa MK3S 250 x 210 x 210) in the STATED orientations, keeping 5 mm off each bed edge:
#   shell standing on its end wall (print height = W extent), hood with the control strip face down (height = x extent)
bb = parts["shell"].bounding_box(); hb = parts["hood"].bounding_box()
fits = {"shell": (sorted([bb.size.X, bb.size.Y]), bb.size.Z), "hood": (sorted([hb.size.Y, hb.size.Z]), hb.size.X)}
for n, (foot, height) in fits.items():
    ok = foot[0] <= 200 and foot[1] <= 240 and height <= 207
    print("  print fit %-5s footprint %.0f x %.0f, height %.0f -> %s" % (n, foot[0], foot[1], height, "OK" if ok else "TOO BIG"))
    if not ok:
        bad += 1
# overhangs in those orientations: down-facing area steeper than 45 degrees, excluding the bed face and small holes
import numpy as np, trimesh
for n, up in (("shell", (0, 0, -1)), ("hood", (-1, 0, 0))):
    m = trimesh.load(os.path.join(FU.OUT, n + ".stl"))
    upv = np.array(up, float)
    h = m.vertices @ upv
    tri_h = m.triangles_center @ upv
    down = (m.face_normals @ upv) < -0.71
    above_bed = tri_h > h.min() + 0.3
    area = m.area_faces[down & above_bed]
    big = area.sum()
    print("  overhang >45deg %-5s %.0f mm2 (hole ceilings and bridges included)" % (n, big))
# tool access with the hood off: straight-down screwdriver (Ø6 shaft) and soldering iron (Ø8) onto each PCB hole
from build123d import Cylinder as Cy, Align as Al
for px, py in FU.PCB_HOLES:
    x, y, w = FU.pcb_to_world(px, py, 1.6 + 3.0)          # above an M3 pan head
    tool = Pos(x, y + 0.5, w) * Rot(-90, 0, 0) * Cy(4.0, 120, align=(Al.CENTER, Al.CENTER, Al.MIN))
    hit = (tool & parts["shell"]).volume
    print("  tool access PCB hole (%2d,%2d): %s" % (px, py, "clear" if hit < 1e-3 else "BLOCKED %.1f mm3" % hit))
    if hit >= 1e-3:
        bad += 1
# plug + wire envelopes (J1/J2/J5) against the housing and the drive
for n in ("shell", "hood", "motor"):
    v = (parts["plug_wires"] & parts[n]).volume
    print("  plug wires vs %-6s overlap %.3f mm3, min dist %.2f" % (n, v, parts["plug_wires"].distance_to(parts[n])))
    if v > 1e-3:
        bad += 1
print("bad incl. housing clearances", bad)
