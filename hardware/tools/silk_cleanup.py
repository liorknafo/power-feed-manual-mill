"""Silkscreen clean-up (edits power-feed.kicad_pcb in place; rerunnable).

1. Reference designators of small passives (R*, C*, SOD-123 diodes, SOT-23-6 ESD arrays) move from
   F.Silkscreen to F.Fab: there is no room for them between 0402/0603 parts and they only generate
   silk-over-pad / overlap warnings. References stay on silk for ICs, the Pico, connectors, FETs and
   transistors, the switch, fuses, LEDs, the power diodes/TVS, the inductor and the mounting holes.
2. Every reference still on silk is checked against pads, other silk and the board edge; if it collides
   it is moved to the nearest free spot around its footprint courtyard.
3. Silk labels: connector pin names above the bottom-edge connectors, a "36V MAX" warning next to J1
   and the board title.

Run with KiCad's Python:
  /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3 \
      hardware/tools/silk_cleanup.py
"""

import os
import re
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_pcb import PCB, W, H, mm  # noqa: E402

KEEP_ON_SILK = re.compile(r"^(U\d+|A\d+|J\d+|Q\d+|SW\d+|F\d+|H\d+|L\d+|D2|D3|D6|D7|D8)$")
EDGE_INSET = 0.5   # mm, silk must stay this far inside the outline
PAD_MARGIN = 0.2   # mm, silk must stay this far from any F.Cu pad (mask opening)
SILK_MARGIN = 0.15


def V(x, y):
    return pcbnew.VECTOR2I(mm(x), mm(y))


def grow(bb, m):
    b = pcbnew.BOX2I(bb.GetPosition(), bb.GetSize())
    b.Inflate(mm(m))
    return b


class Obstacles:
    """Bounding boxes everything on front silk has to avoid."""

    def __init__(self, board):
        self.pads, self.silk, self.texts = [], [], {}
        for fp in board.GetFootprints():
            for p in fp.Pads():
                if p.IsOnLayer(pcbnew.F_Cu) and p.GetAttribute() != pcbnew.PAD_ATTRIB_NPTH:
                    self.pads.append(grow(p.GetBoundingBox(), PAD_MARGIN))
            for g in fp.GraphicalItems():
                if g.GetLayer() == pcbnew.F_SilkS:
                    self.silk.append(grow(g.GetBoundingBox(), SILK_MARGIN))
            r = fp.Reference()
            if r.IsVisible() and r.GetLayer() == pcbnew.F_SilkS:
                self.texts[fp.GetReference()] = grow(r.GetBoundingBox(), SILK_MARGIN)
        for d in board.GetDrawings():
            if d.Type() == pcbnew.PCB_TEXT_T and d.GetLayer() == pcbnew.F_SilkS:
                self.texts[d.GetText()] = grow(d.GetBoundingBox(), SILK_MARGIN)
        self.board = pcbnew.BOX2I(V(EDGE_INSET, EDGE_INSET), V(W - 2 * EDGE_INSET, H - 2 * EDGE_INSET))

    def conflicts(self, bb, me=None):
        if not self.board.Contains(bb):
            return "edge"
        for b in self.pads:
            if b.Intersects(bb):
                return "pad"
        for b in self.silk:
            if b.Intersects(bb):
                return "silk"
        for k, b in self.texts.items():
            if k != me and b.Intersects(bb):
                return "text " + k
        return None


def candidates(fp, inside=False):
    """Positions to try for a reference: a 0.25 mm grid around the courtyard, nearest to the part first
    (text centre at least 0.9 mm outside the courtyard so it clears the outline). With inside=True the
    grid covers the courtyard itself instead (connectors: between the pins and the plug outline)."""
    c = fp.GetCourtyard(pcbnew.F_CrtYd).BBox()
    cx, cy = pcbnew.ToMM(c.GetCenter().x), pcbnew.ToMM(c.GetCenter().y)
    x0, y0, x1, y1 = (pcbnew.ToMM(v) for v in (c.GetLeft(), c.GetTop(), c.GetRight(), c.GetBottom()))
    pts = []
    reach = 0 if inside else 5.0
    nx, ny = int((x1 - x0) / 0.5 + reach / 0.25) + 1, int((y1 - y0) / 0.5 + reach / 0.25) + 1
    for i in range(-nx, nx + 1):
        for j in range(-ny, ny + 1):
            x, y = cx + i * 0.25, cy + j * 0.25
            if (x0 - 0.9 < x < x1 + 0.9 and y0 - 0.9 < y < y1 + 0.9) != inside:
                continue
            dx = max(x0 - x, 0, x - x1)
            dy = max(y0 - y, 0, y - y1)
            pts.append((dx * dx + dy * dy, (x - cx) ** 2 + (y - cy) ** 2, x, y))
    for _, _, x, y in sorted(pts):
        yield x, y


def place_reference(fp, obs):
    r = fp.Reference()
    ref = fp.GetReference()
    if obs.conflicts(grow(r.GetBoundingBox(), SILK_MARGIN), me=ref) is None:
        return "ok"
    orig = r.GetPosition()
    for inside in (False, True):
        for x, y in candidates(fp, inside):
            r.SetPosition(V(x, y))
            bb = grow(r.GetBoundingBox(), SILK_MARGIN)
            if obs.conflicts(bb, me=ref) is None:
                obs.texts[ref] = bb
                return "moved to (%.1f, %.1f)%s" % (x, y, " inside outline" if inside else "")
    r.SetPosition(orig)
    return "NO FREE SPOT"


def add_text(board, obs, text, x, y, size=1.0, rot=0, thickness=0.15, bold=False):
    for d in board.GetDrawings():
        if d.Type() == pcbnew.PCB_TEXT_T and d.GetLayer() == pcbnew.F_SilkS and d.GetText() == text:
            return "exists"
    t = pcbnew.PCB_TEXT(board)
    t.SetText(text)
    t.SetLayer(pcbnew.F_SilkS)
    t.SetPosition(V(x, y))
    t.SetTextSize(pcbnew.VECTOR2I(mm(size), mm(size)))
    t.SetTextThickness(mm(thickness))
    t.SetTextAngleDegrees(rot)
    t.SetBold(bold)
    board.Add(t)
    bb = grow(t.GetBoundingBox(), SILK_MARGIN)
    c = obs.conflicts(bb)
    obs.texts[text] = bb
    return "added" + ("" if c is None else "  !! conflicts with " + c)


def main():
    board = pcbnew.LoadBoard(PCB)
    # 1. small passives: reference to F.Fab
    moved = 0
    for fp in board.GetFootprints():
        r = fp.Reference()
        if not KEEP_ON_SILK.match(fp.GetReference()) and r.GetLayer() == pcbnew.F_SilkS:
            r.SetLayer(pcbnew.F_Fab)
            moved += 1
    print("references moved to F.Fab:", moved)

    # 2. labels first, so the references are placed around them. y = 84.75 is the free strip between
    #    the FET / capacitor pads (bottom 83.55) and the connector outlines (top 85.83); 0.9 mm text.
    obs = Obstacles(board)
    labels = [("36V+", 15.9, 84.75), ("GND", 19.7, 84.75),
              ("A+", 30.6, 84.75), ("A-", 35.68, 84.75), ("B+", 40.76, 84.75), ("B-", 45.84, 84.75),
              ("COIL+", 54.3, 84.75), ("COIL-", 59.0, 84.75)]
    for text, x, y in labels:
        print("  label %-6s %s" % (text, add_text(board, obs, text, x, y, size=0.9)))
    print("  label 36V MAX", add_text(board, obs, "36V MAX", 11.0, 92.5, rot=90, bold=True))
    print("  title ", add_text(board, obs, "MILL POWER FEED v1", 22.5, 3.8, size=1.2, thickness=0.2, bold=True))

    # 3. remaining silk references: relocate the ones that collide
    for fp in sorted(board.GetFootprints(), key=lambda f: f.GetReference()):
        if fp.Reference().GetLayer() != pcbnew.F_SilkS:
            continue
        fp.Reference().SetTextAngleDegrees(0)
        res = place_reference(fp, obs)
        if res != "ok":
            print("  %-4s %s" % (fp.GetReference(), res))
    board.Save(PCB)
    print("saved", PCB)


if __name__ == "__main__":
    main()
