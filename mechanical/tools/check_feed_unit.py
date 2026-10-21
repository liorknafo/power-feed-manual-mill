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
      ("door_pin","shell"),   # pressed into the shell's plate-end knuckle (Ø2.95 on a Ø3 pin)
      ("clutch_pins","clutch_armature"),("clutch_pins","clutch_springs"),("clutch_pins","clutch_bushes"),
      ("clutch_bushes","clutch_hub"),("clutch_springs","clutch_bushes"),
      ("clutch_cover","clutch_output_shaft"),("belt_guard","mill_reference"),("belt_guard","plate"),
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
for n in ("shell", "hood", "door"):
    k = len(parts[n].solids())
    print("  %-5s is %d solid(s) %s" % (n, k, "OK" if k == 1 else "<- FLOATING PIECES"))
    if k != 1:
        bad += 1
# nothing pokes through the outer skin (rails trimmed to it)
skin = FU._rounded(FU.XB - FU.T, FU.X_FP, FU.Y0 - FU.T, FU.Y1 + FU.T, FU.W_SHELL0, FU.W_END, FU.f.r_long, FU.f.r_long, FU.f.r_end,
                  r_bottom=FU.R_BOTTOM)
hinge_zone = FU.bx(FU.HINGE_X - 6, FU.XB - FU.T + 2.5, FU.Y0 - FU.T - 0.01, FU.HINGE_Y + 4.5, FU.W_SHELL0 - 1,
                   FU.W_END + 1)                      # the door hinge's knuckles stand outside the back wall on purpose
for sx in (-1, 1):                                    # and the door's 45-degree printing feet
    xa = sx * (FU.X_FP - FU.R_BOTTOM)
    hinge_zone = hinge_zone + FU.bx(min(xa, xa + sx * 9), max(xa, xa + sx * 9), FU.Y0 - FU.T - 0.01, FU.Y0 - FU.T + 2.6,
                                    FU.W_SHELL0 - 1, FU.W_END + 1)
for n in ("shell", "hood", "door"):
    v = (parts[n] - (skin + hinge_zone)).volume
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
# the door's far screw: M3 insert going up into the end wall's corner boss
pl = Plane(origin=(FU.END_SCREW[0], FU.SPLIT_Y + 0.05, FU.END_SCREW[1]), z_dir=(0, 1, 0))
ring = Solid.make_cylinder(4.0, FU.INSERT_L - 0.1, pl) - Solid.make_cylinder(FU.INSERT_D / 2 + 0.05, FU.INSERT_L - 0.1, pl)
full = (ring & parts["shell"]).volume / ring.volume
print("  door end-screw insert seat: %.0f %% material" % (100 * full))
if full < 0.95:
    bad += 1
print("  insert seats checked: %d" % (1 + sum(1 for j in FU.screw_joints() if j[4] == "insert")))
# mains separation: nothing but the PSU and the inlet may enter the earthed tray's inside, and the wiring zone
# between the PSU terminals and the inlet body (>= 35 mm) must be empty
t, hw = FU.f.tray_t, FU.TRAY_HALF
inside = FU.bx(-hw + t, hw - t, FU.TRAY_BOT + t, FU.TRAY_TOP, FU.TRAY_WALL_W0, FU.TRAY_W1 - t) + \
    FU.bx(-hw + t, hw - t, FU.TRAY_TOP, FU.TERM_TOP, FU.TERM_W0, FU.TRAY_W1 - t)                     # + raised zone
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
for xt in FU.TRAY_TABS_X:
    holes.append(("tray tab M4 x%+.0f" % xt, _hole("+Y", xt, FU.PLATE_BOT + FU.REBATE_H, FU.TRAY_SCREW_W, 1.75, 10)))
for xt in FU.TB_SCREWS_X:
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
# PSU base screw heads (ISO 7380 M3) sit in the door's pockets
for n in ("shell", "door"):
    v = sum(getattr(h & parts[n], "volume", 0.0) or 0.0 for h in FU.psu_screw_heads())
    print("  PSU screw heads vs %s: %.3f mm3" % (n, v))
    if v > 1e-3:
        bad += 1
# the door swings down on its hinge pin (axis along x) without touching the shell, the plate, the tray or the screw heads
from build123d import Axis as _Ax
hinge = _Ax((FU.HINGE_X, FU.HINGE_Y, 0), (0, 0, 1))
sign = -1 if parts["door"].rotate(hinge, -10).bounding_box().min.Y < parts["door"].bounding_box().min.Y else 1
others = ["shell", "plate", "mains_tray", "tension_block", "standoffs", "belt_guard", "mill_reference"]
worst_sw = 0.0
for ang in range(0, 101, 5):
    dr = parts["door"].rotate(hinge, sign * ang)
    v = sum(getattr(dr & parts[n], "volume", 0) or 0 for n in others) + \
        sum(getattr(dr & h, "volume", 0) or 0 for h in FU.psu_screw_heads())
    worst_sw = max(worst_sw, v)
    if v > 1e-2:
        print("  DOOR SWING blocked at %d deg: %.2f mm3" % (ang, v))
        bad += 1
        break
print("  service door swing 0..100 deg: worst overlap %.3f mm3" % worst_sw)
# the snap-in inlet needs flat sheet all round its flange: >= bend (t) + 1.5 mm to the floor and to the lid fold
t_ = FU.f.tray_t; fw, fh = FU.f.inlet_flange
m_low = (FU.INLET_Y - fh / 2) - (FU.TRAY_BOT + t_)
m_top = (FU.TERM_TOP - t_) - (FU.INLET_Y + fh / 2)
print("  inlet flange flat margin: bottom %.1f, top %.1f mm (need >= %.1f)" % (m_low, m_top, t_ + 1.5))
if min(m_low, m_top) < t_ + 1.5:
    bad += 1
# the raised terminal zone must be closed toward the PSU (lid's folded edge): the window between TRAY_TOP and TERM_TOP
# at TERM_W0 must be >= 95 % metal
win = FU.bx(-hw + t, hw - t, FU.TRAY_TOP, FU.TERM_TOP, FU.TERM_W0, FU.TERM_W0 + t)
cov = (getattr(win & parts["mains_tray_lid"], "volume", 0) or 0) + (getattr(win & parts["mains_tray"], "volume", 0) or 0)
print("  mains zone closed toward the PSU: %.0f %% metal" % (100 * cov / win.volume))
if cov / win.volume < 0.95:
    bad += 1
# slide-on: the shell goes on along -W over the bolted tray, module and PSU screw heads (PCB/hood not fitted yet)
inner = ["mains_tray", "mains_tray_lid", "psu", "mains_inlet", "motor", "clutch_cover", "clutch_field_housing",
         "tension_block", "encoder", "encoder_disc", "encoder_posts", "encoder_coupling"]
heads_c = FU.psu_screw_heads()
worst = 0.0
for dw in range(0, 205, 5):
    sh = Pos(0, 0, dw) * parts["shell"]
    v = sum(getattr(sh & parts[n], "volume", 0) or 0 for n in inner) + sum(getattr(sh & h, "volume", 0) or 0 for h in heads_c)
    worst = max(worst, v)
    if v > 1e-2:
        print("  SLIDE-ON blocked %d mm from home: %.2f mm3" % (dw, v))
        bad += 1
        break
print("  shell slide-on sweep 0..200 mm: worst overlap %.3f mm3" % worst)
# loading from below with the door open: the clutch/motor module rises into place (bearing boss and output shaft up the
# plate's slot; its bolts, the 20T pulley, belt and guard come after, from the chamber side), then the mains tray (the
# tension block fitted, the inlet not yet clipped in). Solids are moved one by one (a Compound moved as a whole
# confuses the booleans).
def _solids(names):
    return [so for n in names for so in parts[n].solids()]
def _lift(movers, fixed, d_max, step, label):
    obst = Compound(children=_solids(fixed))
    worst_ = 0.0
    for d in range(0, d_max + 1, step):
        moved = Compound(children=[Pos(0, -d, 0) * so for so in movers])
        hit = moved & obst
        v = hit.volume if hit is not None else 0.0
        worst_ = max(worst_, v)
        if v > 1e-2:
            print("  %s BLOCKED %d mm below home: %.2f mm3" % (label, d, v))
            return 1
    print("  %s from below (0..%d mm): worst overlap %.3f mm3" % (label, d_max, worst_))
    return 0
from build123d import Compound
module = ["motor", "clutch_cover", "clutch_field_housing", "clutch_coil", "clutch_rotor", "clutch_armature", "clutch_hub",
          "clutch_output_shaft", "clutch_pins", "clutch_springs", "clutch_bushes"]
around = ["shell", "plate", "stub_shaft", "bearings_6002", "oldham_disc", "pulley_40T", "encoder", "encoder_disc",
          "encoder_posts", "encoder_coupling", "standoffs", "mill_reference", "pcb_placeholder", "hood", "controls"]
mod_top = max(parts[n].bounding_box().max.Y for n in module)
bad += _lift(_solids(module), around, int(mod_top - (FU.Y0 - FU.T)) + 2, 4, "module")
tray = ["mains_tray", "mains_tray_lid", "psu", "mains_inlet"]          # the inlet is clipped in on the bench
tray_top = max(parts[n].bounding_box().max.Y for n in tray)
fw_, fh_ = FU.f.inlet_flange                                           # + the inlet's flange in front of the tray's end face
flange = FU.bx(-fw_ / 2, fw_ / 2, FU.INLET_Y - fh_ / 2, FU.INLET_Y + fh_ / 2, FU.TRAY_W1, FU.TRAY_W1 + 3.0)
bad += _lift(_solids(tray) + [flange], around + module + ["tension_block"], int(tray_top - (FU.Y0 - FU.T)) + 2, 3,
             "mains tray + inlet")
# the plate's open slot must be closed below the clutch boss (chips, coolant from the belt chamber): tension-block tongue
slot = FU.bx(FU.MX - 20, FU.MX + 20, FU.PLATE_BOT, FU.KEY_TOP, FU.Z_PLATE_IN, FU.Z_PLATE_OUT)
filled = 1 - (slot - parts["plate"] - parts["tension_block"]).volume / slot.volume
print("  plate slot below the clutch boss: %.1f %% filled (tongue 0.1 clear each side)" % (100 * filled))
if filled < 0.99:
    bad += 1
# the tray's tab screws (M4 ISO 7380, head Ø7.6 x 2.2) under the tabs vs the closed door
from build123d import Cylinder as _Cyl, Align as _Al2
tab_heads = [FU.ycyl(3.8, FU.TRAY_BOT - 2.2, FU.TRAY_BOT, xt, FU.TRAY_SCREW_W) for xt in FU.TRAY_TABS_X]
v = sum(getattr(h & parts["door"], "volume", 0) or 0 for h in tab_heads)
print("  tray tab screw heads vs door: %.3f mm3" % v)
if v > 1e-3:
    bad += 1
# the hinge pin goes in from the end-wall end: its whole insertion path beyond the box must be clear, it must slide
# through every knuckle but the plate-end one, and press only into that one
w_a, w_b = FU.SHELL_KNUCKLES[0][0], FU.SHELL_KNUCKLES[-1][1]
path = FU.wcyl(1.5, w_b, w_b + (w_b - w_a) + 10, FU.HINGE_X, FU.HINGE_Y) + \
    FU.wcyl(1.25, w_a - 60, w_a, FU.HINGE_X, FU.HINGE_Y)        # + a Ø2.5 punch from the plate end to push it out
hit = sum(getattr(path & parts[n], "volume", 0) or 0 for n in parts if not n.startswith("control_") and n != "door_pin")
slide = FU.wcyl(1.5, FU.SHELL_KNUCKLES[0][1], w_b, FU.HINGE_X, FU.HINGE_Y)
rub = sum(getattr(slide & parts[n], "volume", 0) or 0 for n in ("shell", "door"))
press = getattr(FU.wcyl(1.5, w_a, FU.SHELL_KNUCKLES[0][1], FU.HINGE_X, FU.HINGE_Y) & parts["shell"], "volume", 0) or 0
print("  hinge pin: insertion path %.3f mm3, sliding %.3f mm3, press fit at the plate-end knuckle %.1f mm3" % (hit, rub, press))
if hit > 1e-3 or rub > 1e-3 or press < 1:
    bad += 1
# every fastener (plain shanks: screws, nuts, washers, inserts, rivnuts, circlips) may overlap only what it threads
# into or seats in; heads, nuts and washers must clear everything else (e.g. the clutch cover's radial screws vs the tray)
fx = FU.all_fasteners(threads=False)
fx_parts = [n for n in parts if not n.startswith("control_") and n not in ("plug_wires", "usb_path")]
fx_bb = {n: parts[n].bounding_box() for n in fx_parts}
stray = 0
for name, shape, into, grp in fx:
    bb = shape.bounding_box()
    for n in fx_parts:
        pb = fx_bb[n]
        if n == into or bb.max.X < pb.min.X or bb.min.X > pb.max.X or bb.max.Y < pb.min.Y or bb.min.Y > pb.max.Y \
                or bb.max.Z < pb.min.Z or bb.min.Z > pb.max.Z:
            continue
        x = shape & parts[n]
        v = x.volume if x is not None else 0.0
        if v > 1e-2:
            stray += 1
            print("  FASTENER %s x %s: %.3f mm3" % (name, n, v))
print("  fasteners checked: %d, stray overlaps %d" % (len(fx), stray))
bad += stray
# print-bed fit (Prusa MK3S 250 x 210 x 210) in the STATED orientations, keeping 5 mm off each bed edge:
#   shell standing on its end wall (print height = W extent), hood with the control strip face down (height = x extent),
#   door outer face down (height = y extent)
bb = parts["shell"].bounding_box(); hb = parts["hood"].bounding_box()
db = parts["door"].bounding_box()
fits = {"shell": (sorted([bb.size.X, bb.size.Y]), bb.size.Z), "hood": (sorted([hb.size.Y, hb.size.Z]), hb.size.X),
        "door": (sorted([db.size.X, db.size.Z]), db.size.Y)}             # the door prints outer face down
for n, (foot, height) in fits.items():
    ok = foot[0] <= 200 and foot[1] <= 240 and height <= 207
    print("  print fit %-5s footprint %.0f x %.0f, height %.0f -> %s" % (n, foot[0], foot[1], height, "OK" if ok else "TOO BIG"))
    if not ok:
        bad += 1
# overhangs in those orientations: down-facing area steeper than 45 degrees, excluding the bed face and small holes
import numpy as np, trimesh
for n, up in (("shell", (0, 0, -1)), ("hood", (-1, 0, 0)), ("door", (0, 1, 0))):
    m = trimesh.load(os.path.join(FU.OUT, n + ".stl"))
    upv = np.array(up, float)
    h = m.vertices @ upv
    tri_h = m.triangles_center @ upv
    down = (m.face_normals @ upv) < -0.71
    above_bed = tri_h > h.min() + 0.3
    area = m.area_faces[down & above_bed]
    big = area.sum()
    print("  overhang >45deg %-5s %.0f mm2 (hole ceilings and bridges included)" % (n, big))
# nothing may start printing in mid-air: slice each print in its print orientation every 0.2 mm (one fixed 2D frame)
# and require every island to touch one in the layer below (a door without its knuckle webs fails this: 2 islands)
from trimesh.geometry import plane_transform
for n, up in (("shell", (0, 0, -1)), ("hood", (-1, 0, 0)), ("door", (0, 1, 0))):
    m = trimesh.load(os.path.join(FU.OUT, n + ".stl"))
    upv = np.array(up, float)
    h = m.vertices @ upv
    prev, floating = None, []
    for z in np.arange(h.min() + 0.1, h.max(), 0.2):
        sec = m.section(plane_origin=upv * z, plane_normal=upv)
        if sec is None:             # an empty layer: whatever starts above it starts in mid-air
            prev = []
            continue
        p2, _ = sec.to_2D(to_2D=plane_transform(upv * z, upv))
        polys = [pg for pg in p2.polygons_full if pg.area > 0.3]
        if prev is not None:
            floating += [z - h.min() for pg in polys if not any(pg.buffer(0.05).intersects(q) for q in prev)]
        prev = polys
    print("  %-5s islands starting in mid-air: %d %s" % (n, len(floating), ["%.1f" % f for f in floating[:5]]))
    if floating:
        bad += 1
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
# USB-C path (90-degree adapter in J4, the extension's male plug from above, its panel end behind the end wall)
for n in ("shell", "hood", "controls", "pcb_placeholder", "plug_wires", "motor", "clutch_cover", "mains_tray_lid"):
    x = parts["usb_path"] & parts[n]
    v = x.volume if x is not None else 0.0
    print("  USB path vs %-15s overlap %.3f mm3, min dist %.2f" % (n, v, parts["usb_path"].distance_to(parts[n])))
    if v > 1e-3:
        bad += 1
for n in ("shell", "hood", "motor"):
    v = (parts["plug_wires"] & parts[n]).volume
    print("  plug wires vs %-6s overlap %.3f mm3, min dist %.2f" % (n, v, parts["plug_wires"].distance_to(parts[n])))
    if v > 1e-3:
        bad += 1
print("bad incl. housing clearances", bad)
