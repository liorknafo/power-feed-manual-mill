#!/usr/bin/env python3
"""Checks of the SolidCraft assemblies, run headless through SolidCraft's MCP server.

    python3 mechanical/solidcraft/tools/check.py            # both assemblies
    python3 mechanical/solidcraft/tools/check.py clutch     # one

clutch.sc:    every pair of bodies, ENGAGED and RELEASED (armature_stroke driven to the stroke): only the fits
              listed in ALLOWED may overlap.
feed_unit.sc: every pair of bodies (the clutch module included); mains separation (nothing but the PSU and the inlet
              inside the earthed tray, the wiring room empty); the bottom door swung open on its hinge; the
              four housing prints against the Prusa MK3S bed.

Rebuild the assemblies first if a part changed (README.md: component.update_links, or the *.build.json scripts).
Exit status 1 when a check fails.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from scmcp import Server  # noqa: E402

# Overlaps that are meant to be there, by component (the part file's name, or the library part's name). A library
# screw is modelled with its thread, the part it goes into with the tap-drill hole: the thread overlaps the hole wall.
ALLOWED = {
    # clutch: screws in their tapped holes
    ("ISO 10642 M4x8", "field_housing"): "cover screws in the housing's M4 tapped holes",
    ("ISO 4026 M5x6", "output_hub"): "hub set screws in their M5 tapped holes",
    ("ISO 4762 M3x8", "motor_23hs45"): "rotor screw in the motor shaft's M3 tapped end",
    # feed unit: screws in tapped holes, heat-set inserts and rivnuts
    ("ISO 10642 M3x8", "plate"): "housing screws in the plate edges' M3 tapped holes",
    ("ISO 10642 M3x8", "heat_set_insert_m3"): "housing screws in heat-set inserts",
    ("ISO 7380 M3x6", "heat_set_insert_m3"): "PCB screws in heat-set inserts",
    ("ISO 7380 M3x8", "rivnut_m3"): "tray lid screws in rivnuts",
    ("ISO 7380 M3x8", "hex_standoff_m3x30"): "encoder disc screws in the hex standoffs' threads",
    ("ISO 7380 M3x8", "encoder_lpd3806"): "encoder screws in its M3 face holes",
    ("ISO 7380 M3x4", "psu_lrs150_36"): "PSU base screws in its M3 holes",
    ("ISO 7380 M4x8", "plate"): "tray tab screws in the plate's M4 tapped holes",
    ("ISO 4762 M5x35", "clutch_module"): "module bolts in the clutch cover's M5 tapped holes",
    ("ISO 4762 M4x30", "clutch_module"): "belt-tension drawbolt in the clutch cover's M4 tapped hole",
    ("ISO 4762 M5x20", "plate"): "tension-block screws in the plate's M5 tapped holes",
    ("ISO 4762 M6x70", "mill_reference"): "standoff screws in the table end (M6 to be tapped there)",
    ("ISO 4026 M5x20", "pulley_htd5m_40t"): "40T pulley set screws in its tapped holes",
    ("ISO 4026 M5x6", "pulley_htd5m_20t"): "20T pulley set screws in its tapped holes",
    ("heat_set_insert_m3", "shell"): "inserts melted into the shell (modelled at the Ø4 hole)",
    ("door_pin", "shell"): "hinge pin pressed into the shell's plate-end knuckle",
    ("coupling_6x8", "stub_shaft"): "the encoder coupling clamps the stub's end",
    ("coupling_6x8", "encoder_lpd3806"): "the encoder coupling clamps the encoder's shaft",
    ("ISO 7380 M3x6", "pcb"): "PCB screws through the board (its holes are not modelled)",
}
MIN_MM3 = 0.01      # smaller overlaps are touching faces / meshing noise

# The mains tray's inside (parts/mains/mains_tray.json: tray_half, tray_bot, tray_t, wall_w0, term_w0, tray_w1 and the
# heights over the PSU and over the terminal zone), in the feed-unit world. Change these with the tray.
TRAY = dict(half=52.0, t=1.5, bot=-153.20883, top=-120.70883, term_top=-110.70883, wall_w0=75.0, term_w0=223.0,
            w1=307.5, w_term=241.0, inlet_depth=29.0)
MAINS_OK = {"mains_tray", "mains_tray_lid", "psu_lrs150_36", "iec_inlet_snapin", "c13_plug",
            "ISO 7380 M3x4",     # the PSU's base screws, up through the floor
            "rivnut_m3", "ISO 7380 M3x8"}   # the lid's rivnuts in the wall flanges, and their screws


def comp(occ_name):
    return occ_name.rsplit(":", 1)[0]


class Session:
    def __init__(self, design):
        self.srv = Server(design)

    def run(self, command, params=None):
        resp = self.srv.rpc("tools/call", {"name": "execute",
                                           "arguments": {"command": command, "params": params or {}}})
        res = resp.get("result", {})
        text = "".join(c.get("text", "") for c in res.get("content", []) if c.get("type") == "text")
        if res.get("isError") or "error" in resp:
            raise RuntimeError("%s: %s" % (command, text or resp.get("error")))
        return json.loads(text)

    def close(self):
        self.srv.proc.stdin.close()
        self.srv.proc.wait(timeout=60)


def interference(s, label):
    d = s.run("inspect.interference", {"budget_s": 1800})
    bad, ok = [], 0
    for i in d["interferences"]:
        v = i.get("volume_mm3", 0.0)
        pa, pb = i["a_part"]["path"], i["b_part"]["path"]
        # a part inside the clutch module counts as the module too (the module bolts thread into its cover)
        ca = {comp(x) for x in pa}
        cb = {comp(x) for x in pb}
        allowed = any((a, b) in ALLOWED or (b, a) in ALLOWED for a in ca for b in cb)
        same = pa == pb             # bodies of one multi-body part (the PCB's blocks, the envelopes) may overlap
        if v < MIN_MM3 or allowed or same:
            ok += 1
            continue
        bad.append((v, "/".join(pa), "/".join(pb)))
    print("== interference, %s: %d pairs overlap, %d of them intended fits" % (label, len(d["interferences"]), ok))
    for v, a, b in bad:
        print("   OVERLAP %10.3f mm3  %s  x  %s" % (v, a, b))
    if d.get("unchecked"):
        print("   NOT CHECKED: %d pairs, e.g. %s" % (len(d["unchecked"]), d["unchecked"][:3]))
    return len(bad) + len(d.get("unchecked", []))


def check_clutch():
    s = Session(os.path.join(ROOT, "clutch.sc"))
    try:
        bad = interference(s, "clutch ENGAGED")
        r = s.run("joint.drive", {"joint": "armature_stroke", "value": 1.1, "quiet": True})
        bad += 0 if r["moved"] else 1
        bad += interference(s, "clutch RELEASED (armature +1.1 against the hub)")
    finally:
        s.close()
    return bad


def mains_zones(s):
    """Mains separation: nothing but the PSU, the inlet and its plug (and the tray's own screws) may enter the earthed
    tray's inside, and the wiring room between the PSU terminals and the inlet body (>= 35 mm) must be empty. The two
    zones are boxes added to this session only (the design file is not saved)."""
    T = TRAY
    x = 2 * (T["half"] - T["t"])
    w_inlet = T["w1"] - T["t"] - T["inlet_depth"]
    boxes = {"mains_zone": (T["bot"] + T["t"], T["top"], T["wall_w0"], T["w1"] - T["t"]),
             "mains_zone_raised": (T["top"], T["term_top"], T["term_w0"], T["w1"] - T["t"]),
             "wiring_zone": (T["bot"] + T["t"], T["top"], T["w_term"], w_inlet)}
    for name, (y0, y1, w0, w1) in boxes.items():
        s.run("solid.box", {"length": x, "width": y1 - y0, "height": w1 - w0, "center": [0, (y0 + y1) / 2, (w0 + w1) / 2],
                            "operation": "new", "body_name": name, "name": name})
    d = s.run("inspect.interference", {"budget_s": 1800})
    bad = 0
    for i in d["interferences"]:
        if i.get("volume_mm3", 0) < MIN_MM3:
            continue
        for zone, other in ((i["a"], i["b_part"]), (i["b"], i["a_part"])):
            if not zone.startswith(("mains_zone", "wiring_zone")):
                continue
            who = {comp(p) for p in other["path"]} or {"(zone)"}
            if who == {"(zone)"}:
                continue
            if zone.startswith("mains_zone") and who & MAINS_OK:
                continue
            # the tension block's 2 M5x20 heads (earthed, on the plate) stand into the tray's plate end, W 75..80,
            # before the PSU starts at W 82 (the clutch's own M5x20s are inside clutch_module, so not these)
            if zone == "mains_zone" and other["path"] and len(other["path"]) == 1 and who == {"ISO 4762 M5x20"}:
                continue
            if zone == "wiring_zone" and who & {"mains_tray", "mains_tray_lid"}:
                continue
            bad += 1
            print("   MAINS %s INTRUDED BY %s (%.2f mm3)" % (zone, "/".join(other["path"]), i["volume_mm3"]))
    print("== mains separation: tray inside and the %.0f mm wiring room: %s" % (w_inlet - T["w_term"],
                                                                             "clear" if not bad else "INTRUDED"))
    return bad + (1 if w_inlet - T["w_term"] < 35 else 0)


def check_feed_unit():
    s = Session(os.path.join(ROOT, "feed_unit.sc"))
    bad = 0
    try:
        bad += interference(s, "feed unit")
        bad += mains_zones(s)
        # the bottom door swings open on its hinge: how far before it meets anything
        # (the end cap comes off first: its scarf holds the door's far end up)
        # The pin sits in the door's knuckles with 0.1 mm clearance, finer than the sweep's meshes: it is left out
        # of the sweep and checked exactly at the open angle instead. Opening is the negative direction.
        near = [o["name"] for o in s.run("component.list")["components"][0]["children"]
                if o["name"].split(":")[0] in ("shell", "plate", "mains_tray", "mains_tray_lid",
                                               "ISO 7380 M4x8", "ISO 7380 M3x4", "psu_lrs150_36")]
        r = s.run("joint.sweep", {"joint": "door_hinge", "from": 0, "to": -150, "occurrences": ["door:1"],
                                  "against": near})
        open_at = r.get("stopped_at")
        open_at = -150.0 if open_at is None else open_at
        by = r.get("stopped_by") or {}
        print("== door swing: opens to %.1f deg, stopped by %s" % (open_at, by.get("b") if isinstance(by, dict) else by))
        if open_at > -90:
            print("   FAIL: the door must swing down past 90 degrees to hang beside the box")
            bad += 1
        s.run("joint.drive", {"joint": "door_hinge", "value": open_at, "quiet": True})
        c = s.run("inspect.clearance", {"bodies": ["door"], "against": ["door_pin"], "clearance": 0.5})
        gap = min([n["distance_mm"] for n in c.get("near", [])] or [0.5])
        print("== door knuckles vs hinge pin at %.1f deg: overlaps %d, closest %.3f mm" % (open_at, len(c["overlaps"]), gap))
        bad += len(c["overlaps"])
        s.run("joint.drive", {"joint": "door_hinge", "value": 0, "quiet": True})
        # print-bed fit, Prusa MK3S (250 x 210 x 210, 5 mm off each edge) in the print orientations:
        # shell and end cap standing on W, hood on its control strip (height = x), door outer face down (height = y)
        m = s.run("inspect.measure", {"bodies": ["shell", "hood", "door", "end_cap"]})
        size = {b["name"]: [b["bbox"]["max"][k] - b["bbox"]["min"][k] for k in range(3)] for b in m["bodies"]}
        up = {"shell": 2, "end_cap": 2, "hood": 0, "door": 1}
        for n, k in up.items():
            foot = sorted(size[n][j] for j in range(3) if j != k)
            ok = foot[0] <= 200 and foot[1] <= 240 and size[n][k] <= 207
            print("== print fit %-7s footprint %.0f x %.0f, height %.0f: %s" % (n, foot[0], foot[1], size[n][k],
                                                                              "OK" if ok else "TOO BIG"))
            bad += 0 if ok else 1
    finally:
        s.close()
    return bad


if __name__ == "__main__":
    which = sys.argv[1:] or ["clutch", "feed_unit"]
    total = 0
    for w in which:
        total += {"clutch": check_clutch, "feed_unit": check_feed_unit}[w]()
    print("FAILED: %d problems" % total if total else "all checks passed")
    sys.exit(1 if total else 0)
