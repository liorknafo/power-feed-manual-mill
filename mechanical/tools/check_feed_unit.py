"""Feed-unit interference check (rebuilds feed_unit.py): every pair of parts except intended fits.
Expected: only the clutch flank contact (rotor x armature). Re-run after changing the F dataclass.

usage: mechanical/.venv/bin/python mechanical/tools/check_feed_unit.py
"""
import sys, itertools
import os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "feed_unit"))
import feed_unit as FU
from build123d import Pos
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
names = list(parts)
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
