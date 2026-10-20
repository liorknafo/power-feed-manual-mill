"""Final clean-up pass on the routed board (run after build_pcb.py --route and fix_planes.py).

1. Connector GND pins get solid zone connections (no starved thermals).
2. Any remaining same-net gap between two items is closed with a small high-priority copper pour
   ("bridge zone") on the layer the items share; the zone filler routes it around everything else
   with full clearance, and is dropped again if it does not actually join the two pieces.
3. Reports isolated GND fill fragments.
"""

import json
import os
import subprocess
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_pcb import KCLI, PCB, mm  # noqa: E402

DRC = os.path.join(os.path.dirname(PCB), "drc.json")


def drc():
    subprocess.run([KCLI, "pcb", "drc", "--format", "json", "--severity-all", "-o", DRC, PCB], capture_output=True)
    return json.load(open(DRC))


def item_map(board):
    m = {}
    for fp in board.GetFootprints():
        for p in fp.Pads():
            m[p.m_Uuid.AsString()] = p
    for t in board.GetTracks():
        m[t.m_Uuid.AsString()] = t
    for z in board.Zones():
        m[z.m_Uuid.AsString()] = z
    return m


def bbox_of(obj):
    b = obj.GetBoundingBox()
    return b.GetLeft(), b.GetTop(), b.GetRight(), b.GetBottom()


def layers_of(obj):
    return [l for l in (pcbnew.F_Cu, pcbnew.B_Cu) if obj.IsOnLayer(l)]


def bridge_zone(board, a, b, margin):
    net = a.GetNet()
    ax0, ay0, ax1, ay1 = bbox_of(a)
    bx0, by0, bx1, by1 = bbox_of(b)
    x0, y0 = min(ax0, bx0) - mm(margin), min(ay0, by0) - mm(margin)
    x1, y1 = max(ax1, bx1) + mm(margin), max(ay1, by1) + mm(margin)
    common = [l for l in layers_of(a) if l in layers_of(b)]
    for layer in common:
        z = pcbnew.ZONE(board)
        z.SetLayer(layer)
        z.SetNet(net)
        z.SetAssignedPriority(20)
        z.SetLocalClearance(mm(0.2))
        z.SetMinThickness(mm(0.25))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        z.SetZoneName("bridge")
        o = z.Outline()
        o.NewOutline()
        for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
            o.Append(int(x), int(y))
        board.Add(z)
        yield z, layer


def via_near(board, items, holes, obj, anchor):
    """A via + F.Cu stub next to `anchor` (a point on obj), clearance-checked; returns the via or None."""
    from fix_planes import collides, VIA_D, VIA_DRILL
    import math
    net = obj.GetNetname()
    for r in [x * 0.25 for x in range(0, 13)]:
        for k in range(16 if r else 1):
            a = 2 * math.pi * k / 16
            c = pcbnew.VECTOR2I(anchor.x + int(mm(r) * math.cos(a)), anchor.y + int(mm(r) * math.sin(a)))
            if any(math.hypot(c.x - hx, c.y - hy) < hr + mm(VIA_DRILL / 2 + 0.35) for hx, hy, hr in holes):
                continue
            if collides(items, net, pcbnew.SHAPE_CIRCLE(c, mm(VIA_D / 2)), (pcbnew.F_Cu, pcbnew.B_Cu)):
                continue
            if r and collides(items, net, pcbnew.SHAPE_SEGMENT(anchor, c, mm(0.25)), (pcbnew.F_Cu,)):
                continue
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(c)
            v.SetWidth(mm(VIA_D))
            v.SetDrill(mm(VIA_DRILL))
            v.SetNet(obj.GetNet())
            board.Add(v)
            items.append(v)
            holes.append((c.x, c.y, mm(VIA_DRILL / 2)))
            if r:
                t = pcbnew.PCB_TRACK(board)
                t.SetStart(anchor)
                t.SetEnd(c)
                t.SetWidth(mm(0.25))
                t.SetLayer(pcbnew.F_Cu)
                t.SetNet(obj.GetNet())
                board.Add(t)
                items.append(t)
            return v
    return None


def anchor_of(obj, other):
    import math
    o = other.GetPosition() if not hasattr(other, "GetStart") or other.Type() == pcbnew.PCB_VIA_T else other.GetStart()
    if isinstance(obj, pcbnew.PAD) or obj.Type() == pcbnew.PCB_VIA_T:
        return obj.GetPosition()
    s, e = obj.GetStart(), obj.GetEnd()
    return s if math.hypot(s.x - o.x, s.y - o.y) < math.hypot(e.x - o.x, e.y - o.y) else e


def bottom_bridges(board, pairs):
    from fix_planes import copper_items
    items = copper_items(board)
    holes = []
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetDrillSize().x > 0:
                holes.append((p.GetPosition().x, p.GetPosition().y, p.GetDrillSize().x / 2))
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_VIA_T:
            holes.append((t.GetPosition().x, t.GetPosition().y, t.GetDrillValue() / 2))
    made = 0
    for a, b in pairs:
        va = via_near(board, items, holes, a, anchor_of(a, b))
        vb = via_near(board, items, holes, b, anchor_of(b, a))
        if va is None or vb is None:
            print("   no via room for", a.GetNetname())
            continue
        for z, layer in bridge_zone(board, va, vb, 1.5):
            if layer == pcbnew.B_Cu:
                made += 1
            else:
                board.Remove(z)
    return made


def report_islands(board):
    gnd = [(t.GetPosition().x, t.GetPosition().y) for t in board.GetTracks()
           if t.Type() == pcbnew.PCB_VIA_T and t.GetNetname() == "GND"]
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetDrillSize().x > 0 and p.GetNetname() == "GND":
                gnd.append((p.GetPosition().x, p.GetPosition().y))
    for z in board.Zones():
        if z.GetNetname() != "GND" or z.GetLayer() not in (pcbnew.F_Cu, pcbnew.B_Cu):
            continue
        polys = z.GetFilledPolysList(z.GetLayer())
        for i in range(polys.OutlineCount()):
            o = polys.Outline(i)
            if not any(o.PointInside(pcbnew.VECTOR2I(int(x), int(y))) for x, y in gnd):
                bb = o.BBox()
                print("   GND island on %s at (%.1f..%.1f, %.1f..%.1f) mm" % (
                    "F" if z.GetLayer() == pcbnew.F_Cu else "B", pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetRight()),
                    pcbnew.ToMM(bb.GetTop()), pcbnew.ToMM(bb.GetBottom())))


def main():
    board = pcbnew.LoadBoard(PCB)
    for ref in ("J1", "J2", "J3", "J4", "J5"):
        board.FindFootprintByReference(ref).SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(PCB)

    for margin in (1.0, 2.5, 3.5):
        d = drc()
        board = pcbnew.LoadBoard(PCB)
        items = item_map(board)
        pairs = []
        for v in d.get("unconnected_items", []):
            objs = [items.get(i.get("uuid")) for i in v["items"]]
            if len(objs) == 2 and all(o is not None for o in objs) and not any(isinstance(o, pcbnew.ZONE) for o in objs):
                pairs.append(objs)
        if not pairs:
            break
        print("margin %.1f mm: %d gaps" % (margin, len(pairs)))
        if margin < 2.0:
            for a, b in pairs:
                for z, layer in bridge_zone(board, a, b, margin):
                    pass
        else:
            print("   bottom-layer bridges:", bottom_bridges(board, pairs))
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        # drop bridge zones that produced no copper
        for z in list(board.Zones()):
            if z.GetZoneName() == "bridge" and not z.HasFilledPolysForLayer(z.GetLayer()):
                board.Remove(z)
            elif z.GetZoneName() == "bridge" and z.GetFilledPolysList(z.GetLayer()).OutlineCount() != 1:
                board.Remove(z)
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        board.Save(PCB)

    report_islands(board)
    d = drc()
    errs = [v for v in d["violations"] if v["severity"] == "error"] + d["unconnected_items"]
    print("remaining errors: %d" % len(errs))
    for v in errs:
        print("  ", v["type"], " | ".join("%s @(%.1f,%.1f)" % (i["description"], i["pos"]["x"], i["pos"]["y"]) for i in v["items"]))


if __name__ == "__main__":
    main()
