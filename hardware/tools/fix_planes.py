"""Connect pads/tracks the router left unconnected on plane nets (GND, +3V3, +36V) by dropping a via
to the right plane next to them. Every candidate via and its stub track are clearance-checked against
all copper of other nets on both outer layers; the via must lie inside the matching inner-plane outline.

Run with KiCad's Python after build_pcb.py --route:  .../python3 hardware/tools/fix_planes.py
"""

import json
import math
import os
import subprocess
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_pcb import KCLI, PCB, mm  # noqa: E402

DRC = os.path.join(os.path.dirname(PCB), "drc.json")
PLANES = {"GND": pcbnew.In1_Cu, "+3V3": pcbnew.In2_Cu, "+36V": pcbnew.In2_Cu}
VIA_D, VIA_DRILL, TRACK_W, CLEAR = 0.6, 0.3, 0.3, 0.2


def run_drc():
    subprocess.run([KCLI, "pcb", "drc", "--format", "json", "--severity-all", "-o", DRC, PCB], capture_output=True)
    return json.load(open(DRC))


def plane_outlines(board):
    out = {}
    for z in board.Zones():
        if z.GetIsRuleArea():
            continue
        n = z.GetNetname()
        if n in PLANES and z.GetLayer() == PLANES[n]:
            out.setdefault(n, []).append(z.Outline())
    return out


def copper_items(board):
    items = []
    for fp in board.GetFootprints():
        items += list(fp.Pads())
    items += list(board.GetTracks())
    return items


def collides(items, net, shape, layers):
    for it in items:
        if it.GetNetname() == net:
            continue
        for layer in layers:
            if not it.IsOnLayer(layer):
                continue
            if it.GetEffectiveShape(layer).Collide(shape, mm(CLEAR)):
                return True
    return False


def find_spot(board, items, holes, outlines, net, anchor, anchor_layer):
    for r in [x * 0.25 for x in range(4, 37)]:
        for k in range(24):
            a = 2 * math.pi * k / 24
            c = pcbnew.VECTOR2I(anchor.x + int(mm(r) * math.cos(a)), anchor.y + int(mm(r) * math.sin(a)))
            # the via must land in the plane's actual copper fill, not just inside its outline
            plane = [z for z in board.Zones() if z.GetNetname() == net and z.GetLayer() == PLANES[net]
                     and not z.GetIsRuleArea()]
            if not any(all(z.HitTestFilledArea(PLANES[net], pcbnew.VECTOR2I(c.x + dx, c.y + dy), 0)
                           for dx, dy in ((0, 0), (mm(0.5), 0), (-mm(0.5), 0), (0, mm(0.5)), (0, -mm(0.5))))
                       for z in plane):
                continue
            # keep inside the board
            if c.x < mm(1.5) or c.y < mm(1.5) or c.x > mm(98.5) or c.y > mm(98.5):
                continue
            if any(math.hypot(c.x - hx, c.y - hy) < hr + mm(VIA_DRILL / 2 + 0.35) for hx, hy, hr in holes):
                continue
            via_shape = pcbnew.SHAPE_CIRCLE(c, mm(VIA_D / 2))
            if collides(items, net, via_shape, (pcbnew.F_Cu, pcbnew.B_Cu)):
                continue
            seg = pcbnew.SHAPE_SEGMENT(anchor, c, mm(TRACK_W))
            if collides(items, net, seg, (anchor_layer,)):
                continue
            return c
    return None


def stitch_islands(board, holes, items):
    """Every outer-layer GND fill fragment without a GND via/PTH inside gets one."""
    gnd_holes = [(t.GetPosition().x, t.GetPosition().y) for t in board.GetTracks()
                 if t.Type() == pcbnew.PCB_VIA_T and t.GetNetname() == "GND"]
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetDrillSize().x > 0 and p.GetNetname() == "GND":
                gnd_holes.append((p.GetPosition().x, p.GetPosition().y))
    inner = [z for z in board.Zones() if z.GetNetname() == "GND" and z.GetLayer() == pcbnew.In1_Cu]
    added = 0
    for z in list(board.Zones()):
        if z.GetNetname() != "GND" or z.GetLayer() not in (pcbnew.F_Cu, pcbnew.B_Cu) or z.GetIsRuleArea():
            continue
        layer = z.GetLayer()
        polys = z.GetFilledPolysList(layer)
        for i in range(polys.OutlineCount()):
            outline = polys.Outline(i)
            if any(outline.PointInside(pcbnew.VECTOR2I(int(x), int(y))) for x, y in gnd_holes):
                continue
            bb = outline.BBox()
            best = None
            step = mm(0.25)
            y = bb.GetTop()
            while y <= bb.GetBottom() and best is None:
                x = bb.GetLeft()
                while x <= bb.GetRight():
                    c = pcbnew.VECTOR2I(int(x), int(y))
                    r = mm(0.25 + 0.2)
                    ring = [c] + [pcbnew.VECTOR2I(c.x + int(r * math.cos(a)), c.y + int(r * math.sin(a)))
                                  for a in [k * math.pi / 4 for k in range(8)]]
                    if all(outline.PointInside(q) for q in ring) and \
                       all(any(iz.HitTestFilledArea(pcbnew.In1_Cu, q, 0) for iz in inner) for q in ring) and \
                       not any(math.hypot(c.x - hx, c.y - hy) < hr + mm(0.125 + 0.35) for hx, hy, hr in holes) and \
                       not collides(items, "GND", pcbnew.SHAPE_CIRCLE(c, mm(0.25)), (pcbnew.F_Cu, pcbnew.B_Cu)):
                        best = c
                        break
                    x += step
                y += step
            if best is None:
                continue
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(best)
            v.SetWidth(mm(0.5))
            v.SetDrill(mm(0.25))
            v.SetNet(board.FindNet("GND"))
            board.Add(v)
            holes.append((best.x, best.y, mm(0.125)))
            gnd_holes.append((best.x, best.y))
            added += 1
    return added


def endpoints(obj):
    if isinstance(obj, pcbnew.PAD):
        return [obj.GetPosition()]
    if obj.Type() == pcbnew.PCB_VIA_T:
        return [obj.GetPosition()]
    return [obj.GetStart(), obj.GetEnd()]


def obj_layers(obj):
    if isinstance(obj, pcbnew.PAD) or obj.Type() == pcbnew.PCB_VIA_T:
        return [l for l in (pcbnew.F_Cu, pcbnew.B_Cu) if obj.IsOnLayer(l)]
    return [obj.GetLayer()]


def bridge(board, items, holes, a_obj, b_obj, width=0.25):
    """Join two fragments of the same net with a straight or L track (F.Cu or B.Cu, vias if needed)."""
    net = a_obj.GetNetname()
    best = None
    for pa in endpoints(a_obj):
        for pb in endpoints(b_obj):
            d = math.hypot(pa.x - pb.x, pa.y - pb.y)
            if best is None or d < best[0]:
                best = (d, pa, pb)
    _, pa, pb = best
    corners = [None, pcbnew.VECTOR2I(pa.x, pb.y), pcbnew.VECTOR2I(pb.x, pa.y)]
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        need_via_a = layer not in obj_layers(a_obj)
        need_via_b = layer not in obj_layers(b_obj)
        for c in corners:
            pts = [pa, pb] if c is None else [pa, c, pb]
            segs = [pcbnew.SHAPE_SEGMENT(pts[i], pts[i + 1], mm(width)) for i in range(len(pts) - 1)]
            if any(collides(items, net, sg, (layer,)) for sg in segs):
                continue
            vias_ok = True
            for need, pt in ((need_via_a, pa), (need_via_b, pb)):
                if need:
                    if collides(items, net, pcbnew.SHAPE_CIRCLE(pt, mm(VIA_D / 2)), (pcbnew.F_Cu, pcbnew.B_Cu)) or \
                       any(math.hypot(pt.x - hx, pt.y - hy) < hr + mm(VIA_DRILL / 2 + 0.35) for hx, hy, hr in holes):
                        vias_ok = False
            if not vias_ok:
                continue
            for i in range(len(pts) - 1):
                t = pcbnew.PCB_TRACK(board)
                t.SetStart(pts[i])
                t.SetEnd(pts[i + 1])
                t.SetWidth(mm(width))
                t.SetLayer(layer)
                t.SetNet(a_obj.GetNet())
                board.Add(t)
                items.append(t)
            for need, pt in ((need_via_a, pa), (need_via_b, pb)):
                if need:
                    v = pcbnew.PCB_VIA(board)
                    v.SetPosition(pt)
                    v.SetWidth(mm(VIA_D))
                    v.SetDrill(mm(VIA_DRILL))
                    v.SetNet(a_obj.GetNet())
                    board.Add(v)
                    items.append(v)
                    holes.append((pt.x, pt.y, mm(VIA_DRILL / 2)))
            return True
    return False


def main():
    board = pcbnew.LoadBoard(PCB)
    d = run_drc()
    by_uuid = {}
    for fp in board.GetFootprints():
        for p in fp.Pads():
            by_uuid[p.m_Uuid.AsString()] = p
    for t in board.GetTracks():
        by_uuid[t.m_Uuid.AsString()] = t
    outlines = plane_outlines(board)
    items = copper_items(board)
    holes = []  # (x, y, radius) of every drilled hole, any net
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetDrillSize().x > 0:
                holes.append((p.GetPosition().x, p.GetPosition().y, p.GetDrillSize().x / 2))
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_VIA_T:
            holes.append((t.GetPosition().x, t.GetPosition().y, t.GetDrillValue() / 2))
    done, failed = 0, []
    seen = set()
    bridged = 0
    for v in d.get("unconnected_items", []):
        objs = [by_uuid.get(it.get("uuid")) for it in v["items"]]
        if len(objs) == 2 and all(o is not None for o in objs) and objs[0].GetNetname() == objs[1].GetNetname():
            if bridge(board, items, holes, objs[0], objs[1]):
                bridged += 1
                seen.update(it["uuid"] for it in v["items"])
    print("bridged %d gaps" % bridged)
    for v in d.get("unconnected_items", []):
        for it in v["items"]:
            obj = by_uuid.get(it.get("uuid"))
            if obj is None or obj.GetNetname() not in PLANES or it["uuid"] in seen:
                continue
            seen.add(it["uuid"])
            net = obj.GetNetname()
            if isinstance(obj, pcbnew.PAD):
                anchor = obj.GetPosition()
                layer = pcbnew.F_Cu if obj.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
            else:
                anchor = obj.GetStart()
                layer = obj.GetLayer()
            spot = find_spot(board, items, holes, outlines, net, anchor, layer)
            if spot is None:
                failed.append(it["description"])
                continue
            via = pcbnew.PCB_VIA(board)
            via.SetPosition(spot)
            via.SetWidth(mm(VIA_D))
            via.SetDrill(mm(VIA_DRILL))
            via.SetNet(obj.GetNet())
            board.Add(via)
            tr = pcbnew.PCB_TRACK(board)
            tr.SetStart(anchor)
            tr.SetEnd(spot)
            tr.SetWidth(mm(TRACK_W))
            tr.SetLayer(layer)
            tr.SetNet(obj.GetNet())
            board.Add(tr)
            items += [via, tr]
            holes.append((spot.x, spot.y, mm(VIA_DRILL / 2)))
            done += 1
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    islands = stitch_islands(board, holes, items)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(PCB)
    print("ground islands stitched: %d" % islands)
    print("added %d plane vias; no room for %d:" % (done, len(failed)))
    for f in failed:
        print("   ", f)


if __name__ == "__main__":
    main()
