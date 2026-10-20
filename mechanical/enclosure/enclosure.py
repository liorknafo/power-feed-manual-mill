"""Controller enclosure for the power-feed PCB (100 x 100 x 1.6): parametric build123d model.

Run:  mechanical/.venv/bin/python mechanical/enclosure/enclosure.py [--alu]
Out:  mechanical/enclosure/out/*.step|*.stl (base, lid, fuse_hatch + PCB placeholder for renders)
      and render_iso.png / render_exploded.png / render_open.png (via mechanical/tools/render.py).
      --alu builds the 3 mm aluminium-plate variant (files get an _alu suffix).

Frame (world): X = PCB x (right), Y = -PCB y (the connector edge, PCB y = 100, faces -Y), Z up.
PCB top-left corner = (0, 0). Floor underside = Z 0.
  -> the PCB input coordinates from the spec are converted by W(x, y, z) = (x, -y, z).

Parts
  base       floor + 4 standoffs (PCB 6 mm up) + walls + connector notch + USB hole + 2 mounting ears
             + 4 hanging lid-screw bosses in the corners (M3 self-tapping, 8 mm deep).
  lid        plate with inner locating lip, 4 counterbored M3 holes, vent slots over the driver area,
             a fuse access opening + rebate, and a front skirt that closes the top of the J1..J3 window.
  fuse_hatch small plate that covers the fuse opening (2x M3x8 into lid-underside bosses).
"""

import math
import os
import subprocess
import sys
from dataclasses import dataclass

from build123d import (Align, Box, Compound, Cone, Cylinder, Pos, RectangleRounded, Rot, SlotOverall, export_step,
                       export_stl, extrude)

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
MIN = (Align.CENTER, Align.CENTER, Align.MIN)


@dataclass
class P:
    material: str = "petg"           # "petg" (3D print) or "alu" (3 mm plate, machined)
    # ---- PCB (spec, PCB coordinates, +y toward the connector edge)
    pcb_w: float = 100.0
    pcb_h: float = 100.0
    pcb_t: float = 1.6
    holes: tuple = ((4, 4), (96, 4), (4, 96), (96, 96))
    caps: tuple = ((15.5, 37.0), (19.0, 74.0))
    cap_d: float = 12.5
    cap_h: float = 25.0
    pico: tuple = (37.0, 3.0, 91.0, 27.0)       # x0, y0, x1, y1
    pico_h: float = 4.0
    usb_y: float = 15.0                          # micro-USB centre, faces +x
    # connectors on the +y edge: (name, x0, x1)
    low_conns: tuple = (("J1", 12.0, 25.3), ("J2", 26.5, 49.9), ("J3", 50.9, 64.2))
    conn_h: float = 12.0                         # height of J1..J3 above the PCB
    conn_y_len: float = 11.0                     # assumed body depth of J1..J3 (render placeholder only)
    j5: tuple = (65.6, 72.4, 77.0, 98.0)         # JST-XH 7-pin vertical x0,x1,y0,y1
    j5_h: float = 8.5                            # assumed height (placeholder only)
    j4: tuple = (73.0, 83.9, 66.4, 92.9)         # 2x7 IDC box header
    j4_h: float = 10.0                           # assumed height (placeholder only)
    fuse: tuple = (0.0, 59.0, 11.0, 87.0)        # 5x20 fuse holder x0,y0,x1,y1
    fuse_h: float = 13.0                         # assumed height (placeholder only)
    # ---- cut-out sizes
    cut_clear: float = 1.0                       # clearance around every connector body
    j5_slot_w: float = 8.0
    j4_slot_w: float = 22.0
    usb_w: float = 14.0                          # hole width (along y)
    usb_h: float = 9.5                           # hole height
    usb_z: float = 2.4                           # hole centre above PCB top (Pico USB centre line)
    # ---- shell
    wall: float = 2.5
    pcb_clear: float = 1.0                       # gap PCB edge -> inner wall
    standoff_h: float = 6.0                      # PCB underside above the floor
    standoff_d: float = 7.0
    pilot_d: float = 2.6                         # M3 self-tapping pilot in PETG (2.5 tap drill in alu)
    pilot_depth: float = 8.0
    cap_clear: float = 3.0                       # min gap cap top -> lid underside
    r_out: float = 4.0                           # outer vertical corner radius
    r_in: float = 1.5                            # inner pocket corner radius (alu: end-mill radius)
    # lid
    lid_t: float = 3.0
    lip_h: float = 3.0
    lip_t: float = 1.6
    lip_gap: float = 0.25                        # lip to inner wall clearance
    boss_r: float = 4.5                          # lid-screw boss radius (corner, hanging from the rim)
    boss_off: float = 3.5                        # boss centre distance from the two inner wall faces
    boss_h: float = 9.0
    boss_hole: float = 2.6
    boss_hole_depth: float = 8.0
    cbore_d: float = 6.4
    cbore_depth: float = 1.6
    vent_area: tuple = (25.0, 50.0, 60.0, 85.0)  # motor driver area x0,y0,x1,y1
    vent_w: float = 2.2
    vent_pitch: float = 4.5
    vent_margin_y: float = 2.0
    # fuse hatch
    hatch_open: tuple = (0.5, 60.0, 10.5, 86.0)  # opening in the lid (x0,y0,x1,y1)
    hatch_rebate: tuple = (-2.5, 52.5, 14.0, 93.5)
    hatch_rebate_depth: float = 1.2
    hatch_clear: float = 0.3
    hatch_screw: tuple = (5.75, 56.0, 90.0)      # x, y1, y2
    hatch_boss_r: float = 3.0
    hatch_boss_h: float = 6.0
    # mounting ears (on the left/right walls at PCB y = ear_y)
    ear_y: float = 50.0
    ear_len: float = 16.0
    ear_w: float = 20.0
    ear_t: float = 3.0
    ear_slot_d: float = 5.5
    ear_slot_stroke: float = 4.0

    @classmethod
    def alu(cls):
        """3 mm aluminium plate: thicker walls, inner corners = 3 mm radius (6 mm end mill), M3 tap drill 2.5."""
        return cls(material="alu", wall=3.0, ear_t=3.0, r_out=6.0, r_in=3.0, pilot_d=2.5, boss_hole=2.5)

    # ---- derived
    @property
    def z_floor(self):
        return self.wall

    @property
    def z_pcb_bot(self):
        return self.wall + self.standoff_h

    @property
    def z_pcb_top(self):
        return self.z_pcb_bot + self.pcb_t

    @property
    def inner_h(self):
        return self.standoff_h + self.pcb_t + self.cap_h + self.cap_clear

    @property
    def z_rim(self):
        return self.wall + self.inner_h

    @property
    def z_top(self):
        return self.z_rim + self.lid_t

    @property
    def inner_w(self):
        return self.pcb_w + 2 * self.pcb_clear

    @property
    def inner_d(self):
        return self.pcb_h + 2 * self.pcb_clear

    @property
    def outer_w(self):
        return self.inner_w + 2 * self.wall

    @property
    def outer_d(self):
        return self.inner_d + 2 * self.wall

    @property
    def centre(self):
        return (self.pcb_w / 2, -self.pcb_h / 2)

    # connector cut-out geometry (PCB x)
    @property
    def win_x(self):
        c = self.cut_clear
        return (min(x0 for _, x0, _ in self.low_conns) - c, max(x1 for _, _, x1 in self.low_conns) + c)

    @property
    def win_top(self):
        return self.z_pcb_top + self.conn_h + self.cut_clear

    @property
    def notch_bot(self):
        return self.z_pcb_top - self.cut_clear

    @property
    def slot_x(self):
        j5c = (self.j5[0] + self.j5[1]) / 2
        j4c = (self.j4[0] + self.j4[1]) / 2
        return (min(j5c - self.j5_slot_w / 2, j4c - self.j4_slot_w / 2), max(j5c + self.j5_slot_w / 2, j4c + self.j4_slot_w / 2))

    @property
    def notch_x(self):
        return (self.win_x[0], self.slot_x[1])

    @property
    def boss_pts(self):
        c, o = self.pcb_clear, self.boss_off
        xs = (-c + o, self.pcb_w + c - o)
        ys = (-c + o, self.pcb_h + c - o)
        return [(x, y) for y in ys for x in xs]       # PCB coordinates


p = P()


# ------------------------------------------------------------------ helpers
def W(x, y, z=0.0):
    """PCB coordinates -> world."""
    return (x, -y, z)


def at(x, y, z=0.0):
    return Pos(*W(x, y, z))


def rprism(w, d, r, z0, z1):
    """Rounded-rectangle prism centred on the PCB centre, between z0 and z1."""
    cx, cy = p.centre
    return Pos(cx, cy, z0) * extrude(RectangleRounded(w, d, r), amount=z1 - z0)


def pbox(x0, y0, x1, y1, z0, z1):
    """Axis-aligned box from PCB-coordinate corners."""
    return Pos((x0 + x1) / 2, -(y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def zcyl(x, y, z0, h, r):
    return at(x, y, z0) * Cylinder(r, h, align=MIN)


# ------------------------------------------------------------------ parts
def base():
    s = rprism(p.outer_w, p.outer_d, p.r_out, 0, p.z_rim)
    s = s - rprism(p.inner_w, p.inner_d, p.r_in, p.z_floor, p.z_rim + 1)
    # PCB standoffs
    for (x, y) in p.holes:
        so = zcyl(x, y, p.z_floor - 0.5, p.standoff_h + 0.5, p.standoff_d / 2)
        so = so - zcyl(x, y, p.z_floor + p.standoff_h - p.pilot_depth, p.pilot_depth + 1, p.pilot_d / 2)
        s = s + so
    # lid-screw bosses hanging from the rim in the four corners (45 degree cone underneath = printable)
    cone_h = p.boss_r - 1.0
    for (x, y) in p.boss_pts:
        b = zcyl(x, y, p.z_rim - p.boss_h, p.boss_h, p.boss_r)
        if p.material == "petg":
            b = b + at(x, y, p.z_rim - p.boss_h - cone_h) * Cone(1.0, p.boss_r, cone_h, align=MIN)
        b = b - zcyl(x, y, p.z_rim - p.boss_hole_depth, p.boss_hole_depth + 1, p.boss_hole / 2)
        s = s + b
    # mounting ears on the left / right walls
    cy = -p.ear_y
    for side in (-1, 1):
        x_out = -p.pcb_clear - p.wall if side < 0 else p.pcb_w + p.pcb_clear + p.wall
        x_c = x_out + side * (p.ear_len / 2 - 1.0)                      # 1 mm buried into the wall
        ear = Pos(x_c, cy, 0) * Box(p.ear_len, p.ear_w, p.ear_t, align=MIN)
        slot_c = x_out + side * (p.ear_len / 2 + 1.5)
        slot = Pos(slot_c, cy, -0.5) * extrude(SlotOverall(p.ear_slot_d + p.ear_slot_stroke, p.ear_slot_d), amount=p.ear_t + 1)
        s = s + ear - slot
    # connector notch in the +y wall: J1..J3 window, J5 + J4 cable slot (open to the lid)
    y_in = p.pcb_h + p.pcb_clear
    y_out = y_in + p.wall
    nx0, nx1 = p.notch_x
    s = s - pbox(nx0, y_in - 1.0, nx1, y_out + 1.0, p.notch_bot, p.z_rim + 1)
    # micro-USB hole in the +x wall
    xi = p.pcb_w + p.pcb_clear
    zc = p.z_pcb_top + p.usb_z
    s = s - pbox(xi - 1.0, p.usb_y - p.usb_w / 2, xi + p.wall + 1.0, p.usb_y + p.usb_w / 2, zc - p.usb_h / 2, zc + p.usb_h / 2)
    return s


def lid():
    c = p.pcb_clear
    s = rprism(p.outer_w, p.outer_d, p.r_out, p.z_rim, p.z_top)
    # locating lip inside the walls
    g = p.lip_gap
    lip = rprism(p.inner_w - 2 * g, p.inner_d - 2 * g, max(p.r_in - g, 0.5), p.z_rim - p.lip_h, p.z_rim)
    lip = lip - rprism(p.inner_w - 2 * g - 2 * p.lip_t, p.inner_d - 2 * g - 2 * p.lip_t, 0.5, p.z_rim - p.lip_h - 1, p.z_rim + 1)
    for (x, y) in p.boss_pts:                                    # relief around the corner bosses
        lip = lip - zcyl(x, y, p.z_rim - p.lip_h - 1, p.lip_h + 2, p.boss_r + 0.4)
    nx0, nx1 = p.notch_x
    y_in = p.pcb_h + c
    y_out = y_in + p.wall
    lip = lip - pbox(nx0 - 0.5, y_in - 3.0, nx1 + 0.5, y_in + 0.5, p.z_rim - p.lip_h - 1, p.z_rim + 1)   # front lip gap
    hx0, hy0, hx1, hy1 = p.hatch_rebate
    lip = lip - pbox(hx0 - 1.0, hy0, hx1 + 1.0, hy1, p.z_rim - p.lip_h - 1, p.z_rim + 1)               # left lip gap at the hatch
    s = s + lip
    # skirt that closes the top of the J1..J3 window (flush with the outer wall face)
    wx0, wx1 = p.win_x
    s = s + pbox(wx0 + 0.2, y_in, wx1, y_out, p.win_top, p.z_rim + 0.01)
    # lid screws
    for (x, y) in p.boss_pts:
        s = s - zcyl(x, y, p.z_rim - 1, p.lid_t + 2, 1.7)
        s = s - zcyl(x, y, p.z_top - p.cbore_depth, p.cbore_depth + 1, p.cbore_d / 2)
    # vents over the motor driver area
    vx0, vy0, vx1, vy1 = p.vent_area
    n = int((vx1 - vx0 - p.vent_w) / p.vent_pitch) + 1
    x_start = (vx0 + vx1) / 2 - (n - 1) * p.vent_pitch / 2
    length = (vy1 - vy0) - 2 * p.vent_margin_y
    for i in range(n):
        x = x_start + i * p.vent_pitch
        vent = at(x, (vy0 + vy1) / 2, p.z_rim - 0.5) * extrude(SlotOverall(length, p.vent_w, rotation=90), amount=p.lid_t + 1.0)
        s = s - vent
    # fuse access: opening, rebate for the hatch, 2 hanging pilot bosses
    ox0, oy0, ox1, oy1 = p.hatch_open
    rx0, ry0, rx1, ry1 = p.hatch_rebate
    hs_x, hs_y1, hs_y2 = p.hatch_screw
    for hy in (hs_y1, hs_y2):
        s = s + zcyl(hs_x, hy, p.z_rim - p.hatch_boss_h, p.hatch_boss_h + 0.01, p.hatch_boss_r)
    s = s - pbox(rx0, ry0, rx1, ry1, p.z_top - p.hatch_rebate_depth, p.z_top + 1)
    s = s - pbox(ox0, oy0, ox1, oy1, p.z_rim - 1, p.z_top + 1)
    for hy in (hs_y1, hs_y2):
        s = s - zcyl(hs_x, hy, p.z_rim - p.hatch_boss_h - 0.5, p.hatch_boss_h + p.lid_t - p.hatch_rebate_depth + 1.0, p.boss_hole / 2)
    return s


def fuse_hatch():
    c = p.hatch_clear
    rx0, ry0, rx1, ry1 = p.hatch_rebate
    t = p.hatch_rebate_depth
    s = pbox(rx0 + c, ry0 + c, rx1 - c, ry1 - c, p.z_top - t, p.z_top)
    hs_x, hs_y1, hs_y2 = p.hatch_screw
    for hy in (hs_y1, hs_y2):
        s = s - zcyl(hs_x, hy, p.z_top - t - 1, t + 2, 1.7)
    # finger notch to lever it out
    s = s - pbox(rx1 - c - 3.0, (ry0 + ry1) / 2 - 4.0, rx1 - c + 1.0, (ry0 + ry1) / 2 + 4.0, p.z_top - t * 0.5, p.z_top + 1)
    return s


# ------------------------------------------------------------------ PCB placeholder (render / interference checks only)
def pcb_placeholder():
    z0, z1 = p.z_pcb_bot, p.z_pcb_top
    board = pbox(0, 0, p.pcb_w, p.pcb_h, z0, z1)
    for (x, y) in p.holes:
        board = board - zcyl(x, y, z0 - 1, p.pcb_t + 2, 1.6)
    caps = Compound(children=[zcyl(x, y, z1, p.cap_h, p.cap_d / 2) for (x, y) in p.caps])
    px0, py0, px1, py1 = p.pico
    pico = pbox(px0, py0, px1, py1, z1, z1 + p.pico_h)
    pico = pico + pbox(px1 - 1.0, p.usb_y - 4.0, px1 + 1.5, p.usb_y + 4.0, z1 + 0.5, z1 + 0.5 + 3.5)     # micro-USB shell
    conns = []
    for name, x0, x1 in p.low_conns:
        conns.append(pbox(x0, 99.3 - p.conn_y_len, x1, 99.3, z1, z1 + p.conn_h))
    j5 = p.j5
    conns.append(pbox(j5[0], j5[2], j5[1], j5[3], z1, z1 + p.j5_h))
    j4 = p.j4
    conns.append(pbox(j4[0], j4[2], j4[1], j4[3], z1, z1 + p.j4_h))
    fz = p.fuse
    fuse = pbox(fz[0], fz[1], fz[2], fz[3], z1, z1 + p.fuse_h)
    return board, caps, pico, Compound(children=conns), fuse


# ------------------------------------------------------------------ build + export
def interference(parts):
    """Print intersection volumes between the PCB placeholder pieces and the shell parts (should all be ~0)."""
    board, caps, pico, conns, fuse = pcb_placeholder()
    pcb_all = [("board", board), ("caps", caps), ("pico", pico), ("connectors", conns), ("fuse", fuse)]
    bad = 0
    for sname, shell in parts.items():
        for pname, piece in pcb_all:
            try:
                v = (shell & piece).volume
            except Exception:
                v = 0.0
            if v > 0.05:
                bad += 1
                print("  INTERFERENCE %-10s x %-10s %.2f mm3" % (sname, pname, v))
    print("interference check:", "FAILED (%d)" % bad if bad else "clean")
    return bad


def build(suffix=""):
    os.makedirs(OUT, exist_ok=True)
    parts = {"base": base(), "lid": lid(), "fuse_hatch": fuse_hatch()}
    for name, shape in parts.items():
        shape.label = name + suffix
        export_step(shape, os.path.join(OUT, name + suffix + ".step"))
        export_stl(shape, os.path.join(OUT, name + suffix + ".stl"))
    board, caps, pico, conns, fuse = pcb_placeholder()
    pcb = Compound(children=[board, caps, pico, conns, fuse])
    pcb.label = "pcb_placeholder"
    export_step(pcb, os.path.join(OUT, "pcb_placeholder.step"))
    export_stl(pcb, os.path.join(OUT, "pcb_placeholder.stl"))
    for nm, sh in (("board", board), ("caps", caps), ("pico", pico), ("conns", conns), ("fuse", fuse)):
        export_stl(sh, os.path.join(OUT, "_pcb_%s.stl" % nm))
    return parts


RENDER_ROT = 90.0     # degrees about the box centre, only so the camera sees the connector + USB walls


def render(parts):
    """Shaded PNGs via mechanical/tools/render.py. Renders use rotated STL copies (_r_*.stl)."""
    cx, cy = p.centre
    pcb = pcb_placeholder()
    named = {"base": parts["base"], "lid": parts["lid"], "hatch": parts["fuse_hatch"], "pcb_board": pcb[0],
             "pcb_caps": pcb[1], "pcb_pico": pcb[2], "pcb_conns": pcb[3], "pcb_fuse": pcb[4]}
    for nm, sh in named.items():
        export_stl(Pos(cx, cy, 0) * Rot(0, 0, RENDER_ROT) * Pos(-cx, -cy, 0) * sh, os.path.join(OUT, "_r_%s.stl" % nm))
    col = {"base": "#8a9bab", "lid": "#a9b8c6", "hatch": "#d9822b", "pcb_board": "#1b7f3a", "pcb_caps": "#2255aa",
           "pcb_pico": "#b02020", "pcb_conns": "#1c1c1c", "pcb_fuse": "#d4a017"}
    tool = os.path.join(HERE, "..", "tools", "render.py")

    def spec(nm, dz=0.0):
        return "%s:%s:%g" % (os.path.join(OUT, "_r_%s.stl" % nm), col[nm], dz)

    def run(out, *specs):
        subprocess.run([sys.executable, tool, os.path.join(OUT, out), "--view", "iso", *specs], check=True)

    run("render_iso.png", spec("base"), spec("lid"), spec("hatch"))
    run("render_open.png", spec("base"), spec("pcb_board"), spec("pcb_caps"), spec("pcb_pico"), spec("pcb_conns"), spec("pcb_fuse"))
    run("render_exploded.png", spec("base"), spec("pcb_board", 14), spec("pcb_caps", 14), spec("pcb_pico", 14),
        spec("pcb_conns", 14), spec("pcb_fuse", 14), spec("lid", 60), spec("hatch", 85))


def info(parts):
    print("outer %.1f x %.1f x %.1f mm (lid on); inner height %.1f, rim at z=%.1f, PCB z %.1f..%.1f, cap top z %.1f, lid underside z %.1f"
          % (p.outer_w, p.outer_d, p.z_top, p.inner_h, p.z_rim, p.z_pcb_bot, p.z_pcb_top, p.z_pcb_top + p.cap_h, p.z_rim))
    print("cap clearance %.1f mm; J1..J3 window x %.1f..%.1f z %.1f..%.1f; J5/J4 slot x %.2f..%.2f; notch x %.2f..%.2f z>=%.1f"
          % (p.z_rim - (p.z_pcb_top + p.cap_h), *p.win_x, p.notch_bot, p.win_top, *p.slot_x, *p.notch_x, p.notch_bot))
    for name, shape in parts.items():
        bb = shape.bounding_box()
        print("%-11s %.1f x %.1f x %.1f mm  vol %.1f cm3  valid=%s" % (name, bb.size.X, bb.size.Y, bb.size.Z, shape.volume / 1000, shape.is_valid))


if __name__ == "__main__":
    if "--alu" in sys.argv:
        p = P.alu()
        parts = build("_alu")
    else:
        parts = build()
    info(parts)
    interference(parts)
    if "--alu" not in sys.argv:
        render(parts)
