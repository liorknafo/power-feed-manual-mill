"""Hand fixes for the last DRC errors on the routed board (edits power-feed.kicad_pcb in place).

Run with KiCad's Python after build_pcb.py --route, fix_planes.py and finish_pcb.py:
  /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3 \
      hardware/tools/manual_fixes.py

Every edit is located by net + geometry (not UUID) and skipped when already applied, so the script is
rerunnable. The fixes, in order:

1. +3V3 to U2 VCC_IO (pins 20/22, C18): the two vias the router dropped at (37.7, 49.8) and (37.7, 50.5)
   sit 1-2 mm below the In2 +3V3 corridor (which ended at y = 48.5) and land in the +36V plane instead,
   so they connect to nothing. Nothing on +36V needs In2 copper in x 32..47, y 49..51.5 (no vias, no
   pads), so the corridor is extended down to y = 51.0 (+36V edge to 51.5). The vias then join the plane
   and the C18 side already has plane vias at y < 48.5. The two extra vias punched through the middle of
   C18 pad 1 are removed (redundant, bad for assembly).

2. +3V3 to the clutch op-amp U6 (pin 5, C34): U6/C34 sit over the +36V part of In2 and are boxed in on
   F.Cu by OA_OUT, OA_FB and the C34 ground pad; going east on B.Cu is blocked by the vertical I2C_SCL
   and E_OK buses at x = 86.5 and 92.0. A 0.3 mm B.Cu track runs from the existing via at (85.64, 46.05)
   north at x = 85.3, in the free channel between the MODE_L (x = 84.0) and I2C_SCL buses, to a new via at
   (85.0, 27.6) inside the +3V3 top region of In2 (y < 28), clear of the Pico VSYS track. The redundant
   via through U6 pad 5 and the two redundant vias through C28 pad 1 (Pico decoupling) are removed.

3. TMC_CA1 (U2 pin 42 -> bootstrap cap C19): pin 40 (MOT_A1) and pin 42 (CA1) feed C19 whose pads were
   on the wrong sides (CA1 pad left, MOT_A1 pad right), so the two nets had to cross; the router gave up
   and left three dangling vias. C19 is rotated 180 deg. Then MOT_A1 goes straight down from pin 40 to
   pad 2, CA1 goes down from pin 42 and jogs right to pad 1, and HA1 (pin 41, between them) runs between
   the two C19 pads down to R10. The old HA1 loop around the right of C19 is removed. Gate/bootstrap
   tracks stay under 7 mm.

4. GND fill islands at D9 and D10 (USBLC6 ESD arrays): pin 2 (GND) sits mid-row with signal tracks on
   all four sides, so the F.Cu pour fragment on the pad is isolated. A 0.5/0.25 mm GND via is placed
   just off the pad end, overlapping the pad, with a short stub. At D9 the MODE_L_RAW track that passes
   under the body is moved up 0.25 mm to make room.

5. Dangling via clean-up: the TMC_LB2 via with nothing on B.Cu is removed.
"""

import math
import os
import subprocess
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_pcb import PCB, mm  # noqa: E402

TOL = mm(0.02)


def V(x, y):
    return pcbnew.VECTOR2I(mm(x), mm(y))


def near(a, b, tol=TOL):
    return math.hypot(a.x - b.x, a.y - b.y) <= tol


def net(board, name):
    n = board.FindNet(name)
    if n is None or n.GetNetCode() <= 0:
        raise KeyError(name)
    return n


def find_track(board, netname, layer, a, b):
    a, b = V(*a), V(*b)
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_TRACE_T and t.GetNetname() == netname and t.GetLayer() == layer:
            s, e = t.GetStart(), t.GetEnd()
            if (near(s, a) and near(e, b)) or (near(s, b) and near(e, a)):
                return t
    return None


def find_via(board, netname, pos):
    pos = V(*pos)
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_VIA_T and t.GetNetname() == netname and near(t.GetPosition(), pos):
            return t
    return None


def remove_track(board, netname, layer, a, b):
    t = find_track(board, netname, layer, a, b)
    if t is not None:
        board.Delete(t)
        return 1
    return 0


def remove_via(board, netname, pos):
    v = find_via(board, netname, pos)
    if v is not None:
        board.Delete(v)
        return 1
    return 0


def add_track(board, netname, layer, a, b, width):
    if find_track(board, netname, layer, a, b) is not None:
        return 0
    t = pcbnew.PCB_TRACK(board)
    t.SetStart(V(*a))
    t.SetEnd(V(*b))
    t.SetWidth(mm(width))
    t.SetLayer(layer)
    t.SetNet(net(board, netname))
    board.Add(t)
    return 1


def add_path(board, netname, layer, pts, width):
    return sum(add_track(board, netname, layer, pts[i], pts[i + 1], width) for i in range(len(pts) - 1))


def add_via(board, netname, pos, d, drill):
    if find_via(board, netname, pos) is not None:
        return 0
    v = pcbnew.PCB_VIA(board)
    v.SetPosition(V(*pos))
    v.SetWidth(mm(d))
    v.SetDrill(mm(drill))
    v.SetNet(net(board, netname))
    board.Add(v)
    return 1


def move_zone_edge(board, netname, layer, y_from, y_to):
    """Every outline vertex of the (net, layer) zone at y_from moves to y_to."""
    moved = 0
    for z in board.Zones():
        if z.GetIsRuleArea() or z.GetNetname() != netname or z.GetLayer() != layer:
            continue
        o = z.Outline().Outline(0)
        for i in range(o.PointCount()):
            p = o.CPoint(i)
            if abs(p.y - mm(y_from)) <= TOL:
                o.SetPoint(i, pcbnew.VECTOR2I(p.x, mm(y_to)))
                moved += 1
    return moved


F, B = pcbnew.F_Cu, pcbnew.B_Cu
TMC = "/TMC5160 stepper driver/"
IO = "/Panel and sensor I/O/"


def fix1_u2_vccio(board):
    n = move_zone_edge(board, "+36V", pcbnew.In2_Cu, 49.0, 51.5)
    n += move_zone_edge(board, "+3V3", pcbnew.In2_Cu, 48.5, 51.0)
    # vias through the middle of C18 pad 1 and their stubs
    n += remove_track(board, "+3V3", F, (35.975, 47.75), (36.071, 47.981))
    n += remove_via(board, "+3V3", (36.071, 47.981))
    n += remove_track(board, "+3V3", F, (34.725, 49.0), (35.475, 47.701))
    n += remove_via(board, "+3V3", (35.475, 47.701))
    return n


def fix2_u6_3v3(board):
    n = remove_track(board, "+3V3", F, (84.138, 46.05), (84.888, 46.05))  # stub to the in-pad via
    n += remove_via(board, "+3V3", (84.888, 46.05))
    n += add_path(board, "+3V3", B, [(85.638, 46.05), (85.3, 45.712), (85.3, 27.9), (85.0, 27.6)], 0.3)
    n += add_via(board, "+3V3", (85.0, 27.6), 0.6, 0.3)
    # C28 pad-1 via and the dangling via hanging off it
    n += remove_track(board, "+3V3", F, (80.225, 28.5), (79.938, 29.193))
    n += remove_via(board, "+3V3", (79.938, 29.193))
    n += remove_via(board, "+3V3", (80.225, 28.5))
    return n


def fix3_tmc_ca1(board):
    n = 0
    c19 = board.FindFootprintByReference("C19")
    if abs(c19.GetOrientationDegrees() - 180.0) > 0.1:
        c19.SetOrientationDegrees(180.0)
        n += 1
    ca1, ha1, mot = TMC + "TMC_CA1", TMC + "TMC_HA1", TMC + "MOT_A1"
    # dead CA1 vias + stubs left by the router / fix scripts
    for p in ((38.304, 64.25), (39.016, 64.962), (38.304, 65.0)):
        n += remove_via(board, ca1, p)
    n += remove_track(board, ca1, F, (38.225, 64.25), (38.304, 64.25))
    n += remove_track(board, ca1, F, (38.304, 64.25), (39.016, 64.962))
    n += remove_track(board, ca1, F, (38.304, 64.25), (38.304, 65.0))
    # old HA1 loop around the right of C19
    for a, b in (((38.5, 61.099), (38.5, 60.163)), ((38.667, 61.266), (38.5, 61.099)),
                 ((38.667, 61.84), (38.667, 61.266)), ((40.539, 63.712), (38.667, 61.84)),
                 ((40.539, 64.951), (40.539, 63.712)), ((39.24, 66.25), (40.539, 64.951))):
        n += remove_track(board, ha1, F, a, b)
    # old MOT_A1 diagonal to the (previous) right-hand pad
    n += remove_track(board, mot, F, (39.775, 64.25), (38.0, 62.475))
    # new routes (C19 pad 2 = MOT_A1 now at 38.225, pad 1 = CA1 at 39.775)
    n += add_path(board, mot, F, [(38.0, 62.475), (38.225, 64.25)], 0.3)
    n += add_path(board, ca1, F, [(39.0, 60.163), (39.0, 62.1), (39.775, 62.875), (39.775, 64.25)], 0.25)
    # HA1: 0.2 mm past the MOT_A1 via, 0.25 mm between the C19 pads (0.2 mm clearance each side)
    n += add_path(board, ha1, F, [(38.5, 60.163), (38.5, 60.95), (38.59, 61.04), (38.59, 62.45)], 0.2)
    n += add_path(board, ha1, F, [(38.59, 62.45), (39.0, 62.86), (39.0, 65.35)], 0.25)
    n += add_path(board, ha1, F, [(39.0, 65.35), (39.24, 66.25)], 0.3)
    return n


def fix4_gnd_islands(board):
    n = 0
    # D10 pin 2: via just above the pad end, 0.2 mm below the DIAL_B_RAW track at y = 87.05
    n += add_via(board, "GND", (86.25, 87.6), 0.5, 0.25)
    n += add_track(board, "GND", F, (86.25, 88.388), (86.25, 87.6), 0.25)
    # D9 pin 2: move MODE_L_RAW (runs under the body at y = 81.805) up to y = 81.55, then via above pad 2
    m = IO + "MODE_L_RAW"
    n += remove_track(board, m, F, (85.841, 81.805), (85.3, 82.346))
    n += remove_track(board, m, F, (88.87, 81.805), (85.841, 81.805))
    n += remove_track(board, m, F, (89.5, 81.175), (88.87, 81.805))
    n += add_path(board, m, F, [(85.3, 82.346), (85.3, 81.55), (88.87, 81.55), (89.245, 81.175), (89.5, 81.175)], 0.2)
    n += add_via(board, "GND", (86.25, 82.08), 0.5, 0.25)
    n += add_track(board, "GND", F, (86.25, 82.888), (86.25, 82.08), 0.25)
    return n


def fix5_dangling(board):
    return remove_via(board, TMC + "TMC_LB2", (46.8043, 66.9735))


def main():
    board = pcbnew.LoadBoard(PCB)
    for fix in (fix1_u2_vccio, fix2_u6_3v3, fix3_tmc_ca1, fix4_gnd_islands, fix5_dangling):
        print("%-18s %d edits" % (fix.__name__, fix(board)))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(PCB)
    print("saved", PCB)


if __name__ == "__main__":
    main()
