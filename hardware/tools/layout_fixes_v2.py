"""Second round of hand fixes on the routed board, after the independent layout review
(edits power-feed.kicad_pcb in place; never run build_pcb.py again, it would regenerate the routing).

Run with KiCad's Python after manual_fixes.py and silk_cleanup.py:
  /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3 \
      hardware/tools/layout_fixes_v2.py

Parts are named by their pre-RP2040 references (refmap.cur() translates them once rp2040_layout.py has renumbered
the board). Every edit is located by net + geometry, and old copper is removed by (net, layer, region) before the new
copper is added, so the script is rerunnable. Nothing touches the netlist: no part, net or footprint change,
only positions, tracks, vias and zones. The fixes, keyed to the review findings:

1. Input path. VIN_RAW (J1 -> F1) and VIN_F (F1 -> Q1 tab) were single 0.8 mm tracks, VIN_F through one
   via. Both become pours: VIN_RAW on F.Cu and B.Cu (10 mm wide, both THT pads), VIN_F on B.Cu from the
   fuse pad up to beside the Q1 tab (5 mm wide) joined to an F.Cu pour hugging the tab by nine 0.8/0.4 mm
   vias. Q1 pin 3 (+36V) reached the In2 plane through 8 mm of 0.3 mm track and one via in D2's pad; it
   now sits in a +36V F.Cu pour that also covers D2 pad 1, C4 pad 1 and runs up to the C1 + pad, with
   thirteen 0.8/0.4 mm vias into In2.

8. Plane split. The In2 +36V block under the clutch / op-amp logic (x 47-93, y 28-61) is removed; +36V on
   In2 is now only the input and power stage (x <= 64.5, y 28.5-86 plus the buck input corner) and a
   4 mm finger up to the clutch PTC F2 (x 64-68, y 33.5-53) so the PTC keeps a plane feed. The finger stops
   below the USB pair / I2C_SCL (which run east at y 31.6-32.5 and down at x >= 69.7, over +3V3) and above LIM_R's
   B.Cu drop at x 66.5, so no signal on B.Cu crosses the split. The rest of In2 is +3V3, one connected polygon.

2. FET decoupling. Each 2.2 uF/100 V cap (C20/C22/C24/C26) moves to directly below its half-bridge's drain
   island (pins 7/8 of Q10..Q13), +36V pad up against the island, GND pad tied to a column of 0.6/0.3 vias
   right beside it (three at C20 / C26, two at C22 / C24 where the next J2 pad is close). Each
   drain island is enlarged and gets a column of four 0.6/0.3 vias into In2 beside the cap. The phase
   fingers to J2 are redrawn (2.05 mm, F.Cu + B.Cu, four stitching vias each).

3/4. Sense network. The SNS_A sense line no longer wraps under J2: it leaves the SNS_A pour by a via next
   to R18 pad 1 and runs on B.Cu under the FET bodies to R19 pad 1. The sense-resistor GND pads (R18 / R21
   pad 2) sit in solid GND islands with six / seven 0.6/0.3 vias. NOT done: dedicated Kelvin returns for
   SRAL/SRBL. R20 / R23 pad 1 (the 47 ohm filters next to U2) still drop into the F.Cu GND pour beside U2: they
   are boxed in by SRAL, SRBL, LIM_L and the LIM_R / ENC_OUT_A / ENC_OUT_N B.Cu diagonals, and neither hand
   routing nor Freerouting found a corridor back to R18 / R21 pad 2 without rerouting that block.

5. Bridge returns. MOT_A1/MOT_B1/MOT_B2 to U2 pins 40/48/45 attach at the phase pours next to the FET
   source pins instead of at the J2 pins.

6. SNS_A node width: G_HA2 is rerouted and the SNS_A pour outline redrawn, which widens the Q11 pin 3 -> R18
   path from 0.8 mm to 0.85 mm at its one pinch (G_HA2 corner (33.4, 73.1) to G_HA1's via (32.99, 74.75), about
   0.5 mm long; everywhere else >= 1.2 mm). G_HA1's via cannot move further: MOT_A1 and the SNS_A sense line run
   under it on B.Cu and Q10 pin 1 bounds its F.Cu run.

7. 36 V clearance. power-feed.kicad_dru adds a 0.5 mm clearance rule for the 36 V nets (+36V, VIN_*, MOT_*,
   CLUTCH_VIN, COIL_P/N) on the outer layers; pads of the fine-pitch power parts (U1, U2, Q2-Q4) and the drivers'
   own gate / bootstrap nets are exempt (see the file). COIL_P and COIL_N are rerouted from the clutch driver to
   J3 (clutch connector): COIL_P straight down the free strip east of the bridge (its Q2 leg now drops at x 75
   to a via above D7, clear of the +5V bus), COIL_N down between J5 and J4 and under J5 along the bottom edge.
   ENC_OUT_N, which wove between R43, D7 and COIL_P, crosses under COIL_P on B.Cu. CLUTCH_ISENSE, which wrapped
   round D6 and ran 0.2 mm from the CLUTCH_VIN diagonal, drops to B.Cu between C35 (turned round) and R47 and
   comes back up between C32's pads. GND stitching vias that sat in the way are removed (fix10).

9. Nits. R44 taps CLUTCH_SHUNT at R42 pad 1 (Kelvin). C17 stays put (moving it put its pad on MOT_B1); the
   SNS_A sense line passes it on the east side instead.
"""

import math
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_pcb import PCB, mm  # noqa: E402
from manual_fixes import (F, B, IO, TMC, V, add_path, add_track, add_via, find_track, find_via,  # noqa: E402
                          near, net, remove_track, remove_via)
from refmap import cur  # noqa: E402

PWR = "/Power input, protection, 5 V buck/"
CLU = "/Clutch coil driver/"
IN1, IN2 = pcbnew.In1_Cu, pcbnew.In2_Cu
# the 36 V nets of power-feed.kicad_dru's hv_clearance rule (SNS_A/B and CLUTCH_SHUNT sit near GND: not listed)
HV = {"+36V", PWR + "VIN_RAW", PWR + "VIN_F", CLU + "CLUTCH_VIN", CLU + "COIL_P", CLU + "COIL_N"} | \
    {TMC + n for n in ("MOT_A1", "MOT_A2", "MOT_B1", "MOT_B2")}
HV_CLR = 0.5


# ----------------------------------------------------------------------------------------------- helpers

def poly_shape(pts):
    s = pcbnew.SHAPE_POLY_SET()
    s.NewOutline()
    for x, y in pts:
        s.Append(mm(x), mm(y))
    return s


def in_box(p, box):
    x0, y0, x1, y1 = box
    return mm(x0) <= p.x <= mm(x1) and mm(y0) <= p.y <= mm(y1)


def remove_tracks_in(board, netname, layer, box):
    """Delete the tracks of a net on a layer that have both ends inside the box (mm)."""
    n = 0
    for t in list(board.GetTracks()):
        if t.Type() == pcbnew.PCB_TRACE_T and t.GetNetname() == netname and t.GetLayer() == layer \
                and in_box(t.GetStart(), box) and in_box(t.GetEnd(), box):
            board.Delete(t)
            n += 1
    return n


def remove_vias_in(board, netname, box):
    n = 0
    for t in list(board.GetTracks()):
        if t.Type() == pcbnew.PCB_VIA_T and t.GetNetname() == netname and in_box(t.GetPosition(), box):
            board.Delete(t)
            n += 1
    return n


def remove_zones(board, netname, layer, box=None):
    """Delete the (non rule-area) zones of a net on a layer whose first vertex is inside the box."""
    n = 0
    for z in list(board.Zones()):
        if z.GetIsRuleArea() or z.GetNetname() != netname or z.GetLayer() != layer:
            continue
        if box is None or in_box(z.Outline().CVertex(0), box):
            board.Delete(z)
            n += 1
    return n


def add_zone(board, netname, layer, pts, priority, clearance=0.3, min_w=0.25, solid=False, name=""):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(net(board, netname))
    z.SetAssignedPriority(priority)
    z.SetLocalClearance(mm(clearance))
    z.SetMinThickness(mm(min_w))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL if solid else pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(mm(0.3))
    z.SetThermalReliefSpokeWidth(mm(0.5))
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    z.SetZoneName(name)
    o = z.Outline()
    o.NewOutline()
    for x, y in pts:
        o.Append(mm(x), mm(y))
    board.Add(z)
    return z


def add_keepout(board, layers, pts, name, pours=True, tracks=False, vias=False):
    """Rule area: by default only copper pours are kept out."""
    for z in list(board.Zones()):
        if z.GetIsRuleArea() and z.GetZoneName() == name:
            board.Delete(z)
    z = pcbnew.ZONE(board)
    z.SetIsRuleArea(True)
    z.SetDoNotAllowCopperPour(pours)
    z.SetDoNotAllowTracks(tracks)
    z.SetDoNotAllowVias(vias)
    z.SetDoNotAllowPads(False)
    z.SetDoNotAllowFootprints(False)
    ls = pcbnew.LSET()
    for l in layers:
        ls.addLayer(l)
    z.SetLayerSet(ls)
    z.SetZoneName(name)
    o = z.Outline()
    o.NewOutline()
    for x, y in pts:
        o.Append(mm(x), mm(y))
    board.Add(z)
    return z


def remove_zone_named(board, name):
    n = 0
    for z in list(board.Zones()):
        if not z.GetIsRuleArea() and z.GetZoneName() == name:
            board.Delete(z)
            n += 1
    return n


def rect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def band(a, b, half):
    """Rectangle of half-width `half` around the segment a-b (axis-aligned segments only)."""
    (x0, y0), (x1, y1) = a, b
    if abs(x0 - x1) < 1e-6:
        return rect(x0 - half, min(y0, y1) - half, x0 + half, max(y0, y1) + half)
    return rect(min(x0, x1) - half, y0 - half, max(x0, x1) + half, y0 + half)


def vias(board, netname, pts, d, drill):
    return sum(add_via(board, netname, p, d, drill) for p in pts)


def move_fp(board, ref, x, y, rot=None):
    fp = board.FindFootprintByReference(cur(board, ref))
    n = 0
    if not near(fp.GetPosition(), V(x, y)):
        fp.SetPosition(V(x, y))
        n += 1
    if rot is not None and abs(fp.GetOrientationDegrees() - rot) > 0.1:
        fp.SetOrientationDegrees(rot)
        n += 1
    return n


def pad(board, ref, num):
    for p in board.FindFootprintByReference(cur(board, ref)).Pads():
        if p.GetNumber() == str(num):
            return p
    raise KeyError((ref, num))


def pad_xy(board, ref, num):
    p = pad(board, ref, num).GetPosition()
    return p.x / 1e6, p.y / 1e6


def gnd_stitch_vias(board):
    """GND vias that have no track attached (the stitching grid)."""
    ends = set()
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_TRACE_T and t.GetNetname() == "GND":
            ends.add((t.GetStart().x, t.GetStart().y))
            ends.add((t.GetEnd().x, t.GetEnd().y))
    return [t for t in board.GetTracks() if t.Type() == pcbnew.PCB_VIA_T and t.GetNetname() == "GND"
            and (t.GetPosition().x, t.GetPosition().y) not in ends]


def remove_gnd_vias_in(board, pts, margin=0.6):
    """Remove GND stitching vias inside (or within `margin` mm of) a polygon: a pour goes there."""
    poly = poly_shape(pts)
    n = 0
    for v in gnd_stitch_vias(board):
        if poly.Collide(v.GetPosition(), mm(margin)):
            board.Delete(v)
            n += 1
    return n


# --------------------------------------------------------------------- 1. input path VIN_RAW / VIN_F / +36V

VIN_RAW_POUR = rect(3.3, 81.5, 18.3, 91.5)
VIN_F_F = rect(3.0, 45.3, 13.4, 52.3)                                  # beside + over the Q1 tab
VIN_F_B = [(3.0, 45.3), (13.4, 45.3), (13.4, 52.3), (8.0, 52.3), (8.0, 64.3), (3.0, 64.3)]
V36_IN = [(11.3, 35.3), (15.9, 35.3), (15.9, 39.0), (17.3, 39.0), (17.3, 54.6), (21.0, 54.6), (21.0, 57.4),
          (13.6, 57.4), (13.6, 60.9), (11.5, 60.9), (11.5, 57.4), (10.5, 57.4), (10.5, 53.3), (14.0, 53.3),
          (14.0, 39.0), (11.3, 39.0)]


def fix1_input_path(board):
    raw, vf = PWR + "VIN_RAW", PWR + "VIN_F"
    n = remove_tracks_in(board, raw, F, (3, 80, 19, 92))
    n += remove_tracks_in(board, vf, F, (3, 44, 14, 65)) + remove_tracks_in(board, vf, B, (3, 44, 14, 65))
    n += remove_vias_in(board, vf, (3, 44, 14, 65))
    for netname, layer in ((raw, F), (raw, B), (vf, F), (vf, B), ("+36V", F)):
        n += remove_zones(board, netname, layer, (0, 30, 22, 95))
    n += add_zone(board, raw, F, VIN_RAW_POUR, 3, HV_CLR, 0.3, solid=True, name="VIN_RAW") is not None
    n += add_zone(board, raw, B, VIN_RAW_POUR, 3, HV_CLR, 0.3, solid=True, name="VIN_RAW") is not None
    n += add_zone(board, vf, F, VIN_F_F, 3, HV_CLR, 0.3, solid=True, name="VIN_F") is not None
    n += add_zone(board, vf, B, VIN_F_B, 3, HV_CLR, 0.3, solid=True, name="VIN_F") is not None
    n += add_zone(board, "+36V", F, V36_IN, 3, HV_CLR, 0.3, solid=True, name="+36V input") is not None
    for pts in (VIN_RAW_POUR, VIN_F_F, VIN_F_B, V36_IN):
        n += remove_gnd_vias_in(board, pts)
    # VIN_F: F.Cu pour <-> B.Cu pour, nine vias beside the tab
    n += vias(board, vf, [(x, y) for x in (3.9, 5.0, 6.1) for y in (46.3, 48.4, 50.5)], 0.8, 0.4)
    # +36V pour -> In2 plane: 3x3 between Q1 pin 3 and D2, four more up the corridor to C1
    n += vias(board, "+36V", [(x, y) for x in (14.6, 15.7, 16.8) for y in (53.9, 55.3, 56.7)], 0.8, 0.4)
    n += vias(board, "+36V", [(15.65, y) for y in (41.0, 44.0, 47.0, 50.0)], 0.8, 0.4)
    # the old 0.3 mm feed from C27 to C20 (C20 moves to its bridge in fix 2)
    n += remove_track(board, "+36V", F, (16.5, 74.0), (16.5, 77.975))
    n += remove_track(board, "+36V", F, (16.5, 77.975), (22.275, 83.75))
    # R7 (VM_SENSE divider): the +36V feed came in diagonally past the VM_SENSE pad; go around it
    n += remove_track(board, "+36V", F, (9.075, 37.0), (7.5, 38.575))
    n += add_path(board, "+36V", F, [(9.075, 37.0), (9.075, 38.575), (7.5, 38.575)], 0.3)
    return n


# ------------------------------------------------------------------------------- 8. In2 plane split + F2 finger

IN2_36V = [(0.5, 6.5), (21.0, 6.5), (21.0, 28.5), (32.5, 28.5), (32.5, 51.5), (64.0, 51.5), (64.0, 33.5),
           (68.0, 33.5), (68.0, 53.0), (64.5, 53.0), (64.5, 86.0), (0.5, 86.0)]
IN2_3V3 = [(0.5, 0.5), (99.5, 0.5), (99.5, 99.5), (0.5, 99.5), (0.5, 86.5), (65.0, 86.5), (65.0, 52.5),
           (68.5, 52.5), (68.5, 33.0), (63.5, 33.0), (63.5, 51.0), (32.0, 51.0), (32.0, 28.0), (21.5, 28.0),
           (21.5, 6.0), (0.5, 6.0)]


def fix8_plane_split(board):
    n = remove_zones(board, "+36V", IN2) + remove_zones(board, "+3V3", IN2)
    add_zone(board, "+36V", IN2, IN2_36V, 1, 0.3, 0.25, name="In2 +36V")
    add_zone(board, "+3V3", IN2, IN2_3V3, 0, 0.3, 0.25, name="In2 +3V3")
    return n + 2


# ------------------------------------------------------------------ 2. half-bridge decoupling, islands, fingers

# bridge n: FET centre c, J2 pad x, cap. FET pins 1-4 at y 76.025, 5-8 at y 80.975; J2 pads y 87.0-90.6.
BRIDGES = [(29.5, 30.6, "C20", "MOT_A1"), (36.0, 35.68, "C22", "MOT_A2"), (42.5, 40.76, "C24", "MOT_B1"),
           (49.0, 45.84, "C26", "MOT_B2")]
VIA_ROWS = (80.6, 81.4, 82.2, 83.0)
SENSE_Y = 79.0      # B.Cu corridor under the FET bodies used by the SNS_A sense line (fix 4)


def fix2_bridges(board):
    n = 0
    # old islands, phase pours, cap feeds and the J2 links (B.Cu runs + F.Cu stubs) of all four bridges
    n += remove_zones(board, "+36V", F, (26, 79, 60, 84))
    for mot in ("MOT_A1", "MOT_A2", "MOT_B1", "MOT_B2"):
        n += remove_zones(board, TMC + mot, F, (25, 73, 53, 90)) + remove_zones(board, TMC + mot, B, (25, 73, 53, 92))
        n += remove_tracks_in(board, TMC + mot, B, (26, 61, 53, 92))
        n += remove_tracks_in(board, TMC + mot, F, (26, 64.3, 53, 92))
        n += remove_vias_in(board, TMC + mot, (26, 61, 53, 92))
    n += remove_tracks_in(board, "+36V", F, (21, 70, 60, 85)) + remove_vias_in(board, "+36V", (21, 70, 60, 85))
    for c, jx, cap, mot in BRIDGES:
        mot = TMC + mot
        # the 2.2 uF/100 V cap below the drain island: +36V pad up (y 82.25-83.4), GND pad down (85.2-86.35)
        n += move_fp(board, cap, c + 1.55, 83.95, -90)   # courtyards: Q bottom 81.75, J2 top 86.1
        # drain island (pins 7/8 + cap +36V pad) with a via column beside the cap, 4 x 0.6/0.3 into In2
        n += add_zone(board, "+36V", F, [(c + 0.2, 80.0), (c + 2.9, 80.0), (c + 2.9, 79.0), (c + 3.6, 79.0),
                                        (c + 3.6, 83.5), (c + 0.2, 83.5)], 7, 0.5, 0.3, solid=True,
                      name="island " + cap) is not None
        n += vias(board, "+36V", [(c + 3.25, y) for y in VIA_ROWS], 0.6, 0.3)
        for p in ((c + 1.0, 86.0), (c + 2.1, 86.0)):
            n += remove_via(board, "GND", p)
        # phase pour: pins 1/5/6 band under the body, 2.05 mm finger down to the J2 pad, on F.Cu and B.Cu
        xl, xr = min(c - 2.4, jx - 1.3), max(c - 0.35, jx + 1.3)
        finger = [(c - 0.35, 82.5), (c - 0.35, 87.1), (xr, 87.1), (xr, 90.8), (xl, 90.8), (xl, 87.1),
                  (c - 2.4, 87.1), (c - 2.4, 82.5)]
        n += add_zone(board, mot, F, [(c - 2.74, 74.75), (c + 2.74, 74.75), (c + 2.74, 79.5), (c - 0.35, 79.5),
                                      (c - 0.35, 82.5)] + finger[1:] + [(c - 2.4, 79.5), (c - 2.74, 79.5)],
                      5, 0.5, 0.3, solid=True, name="phase " + mot[-6:]) is not None
        n += add_zone(board, mot, B, [(c - 2.4, 80.3), (c - 0.35, 80.3)] + finger[1:], 5, 0.5, 0.3, solid=True,
                      name="phase " + mot[-6:]) is not None
        n += vias(board, mot, [(c - 1.9, 82.4), (c - 0.9, 82.4), (c - 1.9, 84.0), (c - 0.9, 84.0)], 0.6, 0.3)
        for pts in ([(c - 2.8, 73.5), (c + 3.7, 73.5), (c + 3.7, 86.6), (c - 2.8, 86.6)], finger):
            n += remove_gnd_vias_in(board, pts, 0.7)
        # cap GND pad (x c+0.2..c+2.9, y 84.85-86.0): 0.6/0.3 vias in a column right beside it (under the island's
        # via column, 0.5 mm from the next phase finger): two, plus a third below them where the next J2 pad leaves
        # room (C20, C26). Added after the stitching-via clean-up above, and every via has a track ending on its
        # centre, so no clean-up (here, fix 10, rp2040_layout) takes it for a stitching via.
        n += remove_track(board, "GND", F, (c + 2.5, 85.35), (c + 3.25, 85.35))
        for y in (84.9, 85.8):
            n += remove_via(board, "GND", (c + 3.25, y))
        gv = [(c + 3.25, 84.95), (c + 3.25, 85.8)]
        third = cap in ("C20", "C26")
        if third:
            gv.append((c + 3.25, 86.65))
        else:
            n += remove_track(board, "GND", F, (c + 3.25, 85.8), (c + 3.25, 86.65))
            n += remove_via(board, "GND", (c + 3.25, 86.65))
        n += vias(board, "GND", gv, 0.6, 0.3)
        n += add_track(board, "GND", F, (c + 2.5, 84.95), gv[0], 0.4)
        n += add_track(board, "GND", F, (c + 2.5, 85.8), gv[1], 0.4)
        if third:
            n += add_track(board, "GND", F, gv[1], gv[2], 0.4)
    return n


# ------------------------------------------------------------- 6. SNS_A node width at Q11 pin 3 (G_HA2 reroute)

def fix6_sns_a_neck(board):
    """G_HA2 crossed the SNS_A pour diagonally right between Q11 pin 4 and pin 3, pinching the pour to 0.8 mm.
    It now crosses higher (y 73.1) and drops vertically onto pin 2; the pour edge moves to x 36.1."""
    g = TMC + "G_HA2"
    n = remove_track(board, g, F, (32.062, 70.776), (36.635, 75.35)) + remove_track(board, g, F, (36.635, 75.35), (36.635, 76.025))
    n += add_path(board, g, F, [(32.062, 70.776), (32.062, 71.7), (33.4, 73.1), (36.1, 73.1), (36.635, 73.635),
                                (36.635, 76.025)], 0.3)
    for sns, pts in (("SNS_A", [(28.46, 68.03), (36.1, 68.03), (36.1, 76.03), (28.46, 76.03)]),
                     ("SNS_B", [(41.47, 68.03), (48.77, 68.03), (48.77, 76.03), (41.47, 76.03)])):
        n += remove_zones(board, TMC + sns, F)
        add_zone(board, TMC + sns, F, pts, 6, 0.2, 0.3, solid=True, name=sns)
    return n


# --------------------------------------------------------- 3/4/5. sense lines, bridge returns, U2 right side

def fix345_sense_and_returns(board):
    n = 0
    sa, sb = TMC + "SNS_A", TMC + "SNS_B"
    # all old SNS_A copper (the detour under J2) and SNS_B's 0.8 mm B.Cu run
    for layer in (F, B):
        n += remove_tracks_in(board, sa, layer, (26, 54, 55, 93)) + remove_tracks_in(board, sb, layer, (40, 45, 53, 78))
    n += remove_vias_in(board, sa, (26, 54, 55, 93)) + remove_vias_in(board, sb, (40, 45, 53, 78))
    # C17 (12VOUT 10 uF) stays where it was (an earlier 1 mm move put its pad on the MOT_B1 track)
    n += move_fp(board, "C17", 49.25, 61.5, 0)
    n += remove_tracks_in(board, TMC + "TMC_12V", F, (45, 60, 49, 62.5))
    n += add_path(board, TMC + "TMC_12V", F, [(45.496, 60.179), (46.817, 61.5), (47.775, 61.5)], 0.3)
    # SNS_A sense: R19 pad 1 -> F.Cu down east of C17 -> via at (49.77, 63.5) -> B.Cu column x 50.3 (the only gap
    # between the G_HB2 vias and the ENC_OUT_A bus) -> corridor under the FET bodies -> via in the SNS_A pour
    n += add_path(board, sa, F, [(50.75, 55.575), (51.7, 56.525), (51.7, 62.85), (51.05, 63.5), (49.77, 63.5)], 0.3)
    n += add_via(board, sa, (49.77, 63.5), 0.6, 0.3)
    n += add_path(board, sa, B, [(49.77, 63.5), (50.3, 64.03), (50.3, SENSE_Y), (31.0, SENSE_Y), (31.0, 72.6)], 0.3)
    n += add_via(board, sa, (31.0, 72.6), 0.6, 0.3)
    # SNS_B sense: the previous path (jog to x 50.2 past the ENC_OUT_N / G_HB1 vias) at 0.3 mm, shifted 0.26 mm
    # south-west so it keeps 0.5 mm from the ENC_OUT_A diagonal
    n += add_path(board, sb, F, [(43.175, 47.75), (43.175, 48.726), (43.428, 48.979)], 0.3)
    n += add_via(board, sb, (43.428, 48.979), 0.6, 0.3)
    # (jogs to x 48.0 under R21 so R21 pad 2 keeps its own GND via at (48.7, 67.3))
    n += add_path(board, sb, B, [(43.428, 48.979), (44.2, 48.2), (45.0, 48.2), (48.4, 51.6), (50.2, 53.4), (50.2, 61.0),
                                 (48.708, 62.49), (48.708, 66.2), (48.0, 66.908), (48.0, 72.6)], 0.3)
    n += add_via(board, sb, (48.0, 72.6), 0.6, 0.3)
    # sense-resistor GND pads (the bridge return, ~3.5 A RMS, and 0.6 W of heat each): a solid GND F.Cu island
    # around each pad, above the SNS pours and the board GND pour, with six (R18) / seven (R21) 0.6/0.3 vias into
    # In1. R21's island reaches east past the B.Cu G_HB2 / SNS_A / ENC_OUT_A columns, where its via column sits.
    n += remove_via(board, "GND", (35.716, 67.807))          # was 0.5/0.25
    r18 = [(35.716, 67.807), (36.9, 71.2), (37.8, 70.3), (38.75, 70.4), (38.75, 71.5), (37.85, 72.15)]
    r21 = [(48.7, 67.3), (48.8, 72.3)] + [(52.0, y) for y in (67.6, 68.6, 69.6, 70.6, 71.6)]
    n += vias(board, "GND", r18 + r21, 0.6, 0.3)
    n += add_track(board, "GND", F, (35.712, 71.2), (36.9, 71.2), 0.4)
    n += add_track(board, "GND", F, (35.712, 70.3), (37.8, 70.3), 0.4)
    n += add_track(board, "GND", F, (35.712, 68.3), (35.716, 67.807), 0.4)
    n += add_track(board, "GND", F, (48.712, 68.3), (48.7, 67.3), 0.4)
    n += add_track(board, "GND", F, (48.712, 71.7), (48.8, 72.3), 0.4)
    # the vias away from the pads get tracks ending on their centres too, so the stitching-via clean-ups (fix 10,
    # rp2040_layout) never drop them
    n += add_path(board, "GND", F, [(37.8, 70.3), (38.75, 70.4), (38.75, 71.5)], 0.4)
    n += add_track(board, "GND", F, (36.9, 71.2), (37.85, 72.15), 0.4)
    n += add_path(board, "GND", F, [(48.712, 69.6), (52.0, 69.6)], 0.4)
    n += add_path(board, "GND", F, [(52.0, 67.6), (52.0, 68.6), (52.0, 69.6), (52.0, 70.6), (52.0, 71.6)], 0.4)
    for name, pts in (("R18 GND", [(35.0, 67.45), (35.85, 67.45), (38.15, 69.75), (40.0, 69.75), (40.0, 72.7),
                                   (35.0, 72.7)]),
                      ("R21 GND", [(48.3, 66.6), (52.5, 66.6), (52.5, 73.0), (48.3, 73.0)])):
        n += remove_zone_named(board, name)
        add_zone(board, "GND", F, pts, 8, 0.2, 0.25, solid=True, name=name)
    # 5. bridge returns to U2 (pins 40/48/45/37) end in the phase pours at the FETs, not at J2
    a1, a2, b1, b2 = (TMC + m for m in ("MOT_A1", "MOT_A2", "MOT_B1", "MOT_B2"))
    n += add_via(board, a1, (37.983, 61.442), 0.6, 0.3)
    n += add_path(board, a1, B, [(37.983, 61.442), (37.983, 61.515), (35.503, 63.995), (35.503, 64.633), (32.169, 67.967),
                                 (32.169, 77.9)], 0.3)
    n += add_via(board, a1, (32.169, 77.9), 0.6, 0.3)
    n += add_path(board, a2, F, [(35.275, 64.25), (35.275, 65.133), (35.897, 65.755), (35.897, 66.616), (38.368, 69.087),
                                 (40.356, 69.087), (40.687, 69.418), (40.687, 73.557), (38.219, 76.025), (37.905, 76.025)], 0.3)
    n += add_via(board, b1, (44.173, 62.337), 0.6, 0.3)
    n += add_path(board, b1, B, [(44.173, 62.337), (41.75, 62.337), (41.75, 77.9)], 0.3)
    n += add_via(board, b1, (41.75, 77.9), 0.6, 0.3)
    n += add_path(board, b2, F, [(43.525, 64.25), (43.525, 64.282), (42.75, 65.057)], 0.3)
    n += add_via(board, b2, (42.75, 65.057), 0.6, 0.3)
    n += add_path(board, b2, B, [(42.75, 65.057), (42.75, 77.9), (46.6, 77.9)], 0.3)
    n += add_via(board, b2, (46.6, 77.9), 0.6, 0.3)
    # U2 pin 4 (+36V) -> C12 -> In2: the via sat 0.33 mm from C12's GND pad. C12 moves 0.25 mm right so the feed
    # passes its GND pad at 0.5 mm, and the via sits beyond C12 pad 1 (TMC_HB1's diagonal limits it to 0.5 mm)
    n += move_fp(board, "C12", 48.0, 58.0)
    n += remove_tracks_in(board, "+36V", F, (43.3, 56.9, 49.5, 59.5)) + remove_via(board, "+36V", (46.983, 58.183))
    # (0.23 mm beside the U2 pin 3 TMC_12V escape, as originally routed, then 0.3 mm)
    n += add_track(board, "+36V", F, (43.413, 57.25), (46.1, 57.25), 0.23)
    n += add_path(board, "+36V", F, [(46.1, 57.25), (46.6, 57.75), (46.6, 58.4), (47.15, 58.95), (48.9, 58.95)], 0.3)
    n += add_via(board, "+36V", (48.9, 58.95), 0.5, 0.25)
    # G_HB1's via sat on C12's (moved) GND pad: it moves 0.33 mm right / 0.2 mm down
    g = TMC + "G_HB1"
    n += remove_via(board, g, (49.017, 57.034)) + remove_track(board, g, B, (49.017, 57.581), (49.017, 57.034))
    n += remove_track(board, g, F, (49.017, 57.034), (49.294, 57.034)) + remove_track(board, g, F, (49.294, 57.034), (50.0, 57.74))
    n += add_via(board, g, (49.35, 57.25), 0.6, 0.3)
    n += add_path(board, g, F, [(49.35, 57.25), (49.84, 57.74), (50.0, 57.74)], 0.3)
    n += add_track(board, g, B, (49.35, 57.25), (49.017, 57.581), 0.3)
    # U2 pin 33 (+36V) -> C11 pad 1: the 45 deg feed passed C11's GND pad at 0.06 mm; go around it on the right
    n += remove_track(board, "+36V", F, (32.772, 57.3), (30.75, 59.322))
    n += add_path(board, "+36V", F, [(32.772, 57.3), (32.5, 57.3), (32.2, 57.6), (32.2, 59.0), (31.878, 59.322),
                                     (30.75, 59.322)], 0.3)
    return n


# ------------------------------------------------------------------------------ 7. COIL_P / COIL_N to J3

def fix7_coil(board):
    cp, cn = CLU + "COIL_P", CLU + "COIL_N"
    n = 0
    # COIL_P: old F.Cu diagonal through the I/O filters + the run beside Q13/C24
    n += remove_tracks_in(board, cp, F, (51, 58.0, 71, 90))
    # new: diagonal from the driver (x+y = 127.6, 0.5 mm from ENC_OUT_A's diagonal), a short B.Cu hop under
    # ENC_OUT_A's horizontal at y 74.2, then straight down x 56.3 to J3 pad 1
    n += add_path(board, cp, F, [(70.407, 58.325), (69.3, 58.325), (56.3, 71.325), (56.3, 72.6)], 0.8)
    n += add_via(board, cp, (56.3, 72.6), 0.9, 0.45)
    n += add_path(board, cp, B, [(56.3, 72.6), (56.3, 75.8)], 0.8)
    n += add_via(board, cp, (56.3, 75.8), 0.9, 0.45)
    n += add_path(board, cp, F, [(56.3, 75.8), (56.3, 87.5), (55.0, 88.8)], 0.8)
    # the D11 GND fan-out track that ran west across that line (D11 pin 2 keeps its pour connection)
    n += remove_track(board, "GND", F, (54.0, 79.525), (57.5, 79.525))
    # Q2 -> D7 leg: the B.Cu diagonal to a via at (71.18, 58.3) passed 0.3 mm from the +5V bus (x 70.27); it now
    # drops straight down x 75.0 to a via just above D7 pad 1
    n += remove_tracks_in(board, cp, B, (70, 41, 76, 59)) + remove_via(board, cp, (71.18, 58.325))
    n += add_path(board, cp, B, [(75.001, 42.0), (75.001, 55.6)], 0.8)
    n += add_via(board, cp, (75.001, 55.6), 0.9, 0.45)
    n += add_track(board, cp, F, (75.001, 55.6), (75.6, 56.9), 0.8)
    # ENC_OUT_N wove between R43, D7's pads and COIL_P at 0.2 mm; it now crosses under COIL_P on B.Cu at x 72.0
    eon = "ENC_OUT_N"
    n += remove_tracks_in(board, eon, F, (69.7, 54.0, 78.0, 65.1))
    n += add_path(board, eon, F, [(69.793, 54.592), (71.0, 54.592), (72.0, 55.592)], 0.2)
    n += add_via(board, eon, (72.0, 55.592), 0.6, 0.3)
    n += add_track(board, eon, B, (72.0, 55.592), (72.0, 60.5), 0.2)
    n += add_via(board, eon, (72.0, 60.5), 0.6, 0.3)
    n += add_path(board, eon, F, [(72.0, 60.5), (72.0, 63.775), (73.225, 65.0)], 0.2)
    # COIL_N: the B.Cu vertical moves from x 80.2 to x 81.0 (J4 pads), then instead of running down x 73.65 past
    # J5 and along the edge at y 96.5 (0.2 mm from J5 pin 1), it goes to y 98.8 under the connector bodies, up at
    # x 66.5 (clear of the MODE_R/RAPID diagonals) and into J3 pad 2 from the east along y 88.8.
    # The D6 -> Q4 link moves from x 83.3 (0.26 mm from the MODE_L bus) to x 82.6.
    n += remove_tracks_in(board, cn, B, (59, 38, 86, 99.5)) + remove_tracks_in(board, cn, F, (59, 38, 86, 99.5))
    n += remove_vias_in(board, cn, (82, 37, 84, 40))
    # Q4 pad 3 escape: 0.6 mm from the pad's top half, 0.55 mm above the 0.8 mm CLUTCH_SHUNT track under Q4
    n += add_path(board, cn, F, [(79.938, 50.75), (79.938, 50.263), (80.215, 49.986)], 0.6)
    # D6 pad 2 -> via at (82.9, 38.5): 0.5 mm from the CLUTCH_VIN diagonal and from the MODE_L bus (B.Cu x 84.0)
    n += add_track(board, cn, F, (84.6, 38.5), (82.9, 38.5), 0.8)
    n += add_via(board, cn, (82.9, 38.5), 0.9, 0.45)
    # the x 81 -> x 73.65 diagonal runs 0.5 mm clear of J4 pad 1
    n += add_path(board, cn, B, [(82.9, 38.5), (82.6, 38.8), (82.6, 47.6), (80.215, 49.986), (81.0, 50.771), (81.0, 65.73),
                                 (73.652, 73.078), (73.652, 98.8), (66.5, 98.8), (66.5, 88.8), (60.08, 88.8)], 0.8)
    # CLUTCH_ISENSE (C35/R47 -> MCU) wrapped round D6, between D6's pads and along the CLUTCH_VIN diagonal at 0.2 mm.
    # C35 turns round (GND pad west, away from the CLUTCH_VIN diagonal) and moves 0.5 mm down-left; ISENSE drops
    # to B.Cu between C35 and R47, runs north at x 81.3 (between I2C_SDA x 78.0 and COIL_N x 82.6), comes back up at y 34 and
    # runs west between C32's pads (0.8 mm each side) and round F2 pad 2 to the old trunk at (73.45, 30.05).
    ci = "CLUTCH_ISENSE"
    n += move_fp(board, "C35", 80.5, 41.6, 180)
    n += remove_tracks_in(board, ci, F, (73.4, 29.9, 87.0, 45.0))
    n += remove_tracks_in(board, ci, B, (73.4, 29.9, 87.0, 45.0)) + remove_vias_in(board, ci, (73.4, 29.9, 87.0, 45.0))
    n += add_path(board, ci, F, [(81.275, 41.6), (81.3, 42.55), (81.325, 43.5)], 0.2)
    n += add_via(board, ci, (81.3, 42.55), 0.6, 0.3)
    n += add_track(board, ci, B, (81.3, 42.55), (81.3, 34.0), 0.2)
    n += add_via(board, ci, (81.3, 34.0), 0.6, 0.3)
    n += add_path(board, ci, F, [(81.3, 34.0), (75.0, 34.0), (75.0, 30.6), (74.55, 30.15), (73.446, 30.047)], 0.2)
    # DIAL_B's via sat 0.26 mm from J3 pad 2; move it up-right
    db = "DIAL_B"
    n += remove_track(board, db, B, (61.043, 82.338), (61.043, 86.732)) + remove_via(board, db, (61.043, 86.732))
    n += remove_track(board, db, F, (61.043, 86.732), (67.993, 93.681))
    n += add_path(board, db, B, [(61.043, 82.338), (61.043, 85.643), (61.5, 86.1)], 0.2)
    n += add_via(board, db, (61.5, 86.1), 0.6, 0.3)
    n += add_path(board, db, F, [(61.5, 86.1), (62.5, 87.1), (62.5, 88.188), (67.993, 93.681)], 0.2)
    return n


# ------------------------------------------------------------------- 10. stitching vias in the way of new copper

def fix10_stitch_vias(board):
    """Delete GND stitching vias (no track attached) that end up closer than the rule to another net's track or
    via after the reroutes above: 0.5 mm to the 36 V nets, 0.2 mm otherwise."""
    others = [t for t in board.GetTracks() if t.GetNetname() != "GND"]
    n = 0
    for v in gnd_stitch_vias(board):
        vs = v.GetEffectiveShape(F)
        r = v.GetWidth(F) // 2
        for t in others:
            need = mm(HV_CLR if t.GetNetname() in HV else 0.2)
            p = v.GetPosition()
            if t.Type() == pcbnew.PCB_VIA_T:
                if (t.GetPosition() - p).EuclideanNorm() < r + t.GetWidth(F) // 2 + need:
                    break
            elif t.GetLayer() in (F, B) and t.GetEffectiveShape().Collide(vs, need):
                break
        else:
            continue
        board.Delete(v)
        n += 1
    return n


# ------------------------------------------------------------------------------------------------- main

FIXES = [fix1_input_path, fix8_plane_split, fix2_bridges, fix6_sns_a_neck, fix345_sense_and_returns, fix7_coil,
         fix10_stitch_vias]


def main():
    board = pcbnew.LoadBoard(PCB)
    for fix in FIXES:
        print("%-22s %d edits" % (fix.__name__, fix(board)))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(PCB)
    print("saved", PCB)


if __name__ == "__main__":
    main()
