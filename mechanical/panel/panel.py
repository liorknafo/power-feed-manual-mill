"""Front panel / pendant box for the power feed: parametric build123d model.

Run:  mechanical/.venv/bin/python mechanical/panel/panel.py
Out:  mechanical/panel/out/*.step|*.stl (shell, back_cover + hardware placeholder) and
      render_iso.png / render_front.png / render_exploded.png (via mechanical/tools/render.py).

Frame: origin = centre of the box outline, X right, Y up (as seen looking at the front face),
Z = depth. Back rim of the shell at Z = 0, front (operator) face at Z = depth. The back cover
sits below the rim (Z -cover_t .. 0) with a locating lip that goes inside the shell.

Front layout (x, y from the box centre, mm), 140 x 100 box:
    OLED window top-centre          (0, 30.5)   module centre (0, 28)
    SPEED dial (EC11)               (0, -1)
    ON-OFF-ON toggle                (-44, -33)  lever swings left/right, labels LEFT / STOP / RIGHT above it
    RAPID push-button               (0, -34)
    E-STOP mushroom                 (44, 22)    own column on the right, 9 mm clear of the dial knob
"""

import math
import os
import subprocess
import sys
from dataclasses import dataclass

from build123d import (Align, Box, Compound, Cylinder, Polyline, Pos, RectangleRounded, Rot, SlotOverall, Text, export_step,
                       export_stl, extrude, make_face)

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "out")
MIN = (Align.CENTER, Align.CENTER, Align.MIN)


@dataclass
class P:
    # ---- box
    box_w: float = 140.0
    box_h: float = 100.0
    wall: float = 3.0
    front_t: float = 3.0
    depth: float = 60.0                  # rim -> front face. E-stop body 45 mm + 12 mm for its rear terminals/wiring: MEASURE YOUR E-STOP
    r_out: float = 6.0
    r_in: float = 3.0
    # ---- back cover
    cover_t: float = 4.0                 # 3.2 magnet pocket + 0.8 skin
    lip_h: float = 3.0
    lip_t: float = 2.0
    lip_gap: float = 0.25
    boss_r: float = 4.0                  # corner screw bosses (M3 self-tapping, from the rim)
    boss_off: float = 3.5                # boss centre distance from the two inner wall faces
    boss_hole: float = 2.6
    boss_hole_depth: float = 14.0
    cbore_d: float = 6.2
    cbore_depth: float = 2.0
    # ---- controls (front face, x, y)
    estop_pos: tuple = (44.0, 22.0)
    estop_hole: float = 22.5             # 22 mm nominal; printed holes come out ~0.2-0.3 small
    estop_head_d: float = 40.0
    estop_head_h: float = 25.0
    estop_depth: float = 45.0            # body behind the panel
    toggle_pos: tuple = (-44.0, -33.0)
    toggle_hole: float = 12.4            # 12 mm bushing
    toggle_depth: float = 30.0
    rapid_pos: tuple = (0.0, -34.0)
    rapid_hole: float = 16.3             # 16 mm button
    rapid_depth: float = 25.0
    enc_pos: tuple = (0.0, -1.0)
    enc_hole: float = 7.4                # M7 bushing
    enc_pin: bool = True                 # anti-rotation pin hole
    enc_pin_d: float = 3.2
    enc_pin_off: float = 7.8
    enc_knob_d: float = 30.0
    enc_depth: float = 20.0
    enc_bushing_len: float = 6.0         # threaded bushing length of the EC11 (typ. 5-7 mm): check yours
    enc_nut_t: float = 2.0               # M7 nut
    enc_thread_spare: float = 0.5        # thread that must stick out past the nut
    enc_recess_d: float = 11.0           # rear pocket diameter if the panel has to be thinned around the encoder
    # ---- OLED 1.3" 128x64 SH1106 module
    oled_pos: tuple = (0.0, 28.0)        # module centre
    oled_pcb: tuple = (35.4, 33.5)
    oled_holes: tuple = (30.4, 28.5)     # M2 hole spacing x, y
    oled_win: tuple = (30.0, 16.0)
    oled_win_dy: float = 2.5             # visible area centre above the module centre
    oled_post_h: float = 2.5             # standoff height behind the panel (verify against your module's glass height)
    oled_post_d: float = 5.0
    oled_pilot_d: float = 1.7            # M2 self-tapping
    oled_pilot_depth: float = 4.0
    # ---- cable entry (bottom wall) + strain relief
    cable_x: float = -22.0
    cable_w: float = 20.0
    cable_h: float = 4.0
    cable_z: float = 21.0                # slot centre height above the rim
    tie_bridge_w: float = 30.0
    tie_bridge_z: tuple = (24.5, 32.0)   # z of the bridge at the wall: bottom face / top (bed side) edge; the bed-side face is a 45 degree chamfer
    tie_bridge_d: float = 5.0            # how far it sticks out from the inner wall face
    tie_slot: tuple = (1.8, 4.2)         # zip tie 3.6 mm wide: slot x, y
    tie_dx: float = 11.5
    # ---- adapter board on the back cover (IDC header + terminals)
    adapter_pos: tuple = (-22.0, -28.0)
    adapter_size: tuple = (45.0, 28.0, 1.6)
    adapter_holes: tuple = (39.0, 22.0)
    adapter_post_h: float = 5.0
    adapter_post_d: float = 6.0
    adapter_pilot_d: float = 2.6
    adapter_pilot_depth: float = 7.0
    # ---- magnets in the back cover (open to the inside, 0.8 mm skin outside)
    magnet_d: float = 12.0
    magnet_h: float = 3.0
    magnet_clear: float = 0.3
    magnet_pos: tuple = ((-52.0, -32.0), (52.0, -32.0), (-52.0, 32.0), (52.0, 32.0))
    # ---- engraving
    text_depth: float = 0.6
    text_size: float = 4.0
    font: str = "Arial"

    @property
    def cavity(self):                    # rim -> rear face of the front plate
        return self.depth - self.front_t

    @property
    def enc_recess(self):                # depth of the rear thinning around the encoder (0 for a 3 mm panel)
        return max(0.0, self.front_t + self.enc_nut_t + self.enc_thread_spare - self.enc_bushing_len)

    @property
    def z_plate(self):                   # rear face of the front plate
        return self.cavity

    @property
    def inner_w(self):
        return self.box_w - 2 * self.wall

    @property
    def inner_h(self):
        return self.box_h - 2 * self.wall

    @property
    def boss_pts(self):
        x = self.box_w / 2 - self.wall - self.boss_off
        y = self.box_h / 2 - self.wall - self.boss_off
        return [(sx * x, sy * y) for sy in (-1, 1) for sx in (-1, 1)]

    @property
    def labels(self):
        tx, ty = self.toggle_pos
        ex, ey = self.estop_pos
        rx, ry = self.rapid_pos
        nx, ny = self.enc_pos
        return [("SPEED", (nx, ny + 19.5)),
                ("RAPID", (rx + 20.0, ry)),
                ("LEFT", (tx - 16.0, ty + 12.5)),
                ("STOP", (tx, ty + 12.5)),
                ("RIGHT", (tx + 16.0, ty + 12.5)),
                ("E-STOP", (ex, ey - self.estop_head_d / 2 - 5.0))]


p = P()


# ------------------------------------------------------------------ helpers
def rprism(w, h, r, z0, z1):
    return Pos(0, 0, z0) * extrude(RectangleRounded(w, h, r), amount=z1 - z0)


def zcyl(x, y, z0, h, r):
    return Pos(x, y, z0) * Cylinder(r, h, align=MIN)


def box(x0, y0, z0, x1, y1, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def front_hole(pos, d):
    return zcyl(pos[0], pos[1], p.z_plate - 1, p.front_t + 2, d / 2)


# ------------------------------------------------------------------ parts
def shell():
    s = rprism(p.box_w, p.box_h, p.r_out, 0, p.depth)
    s = s - rprism(p.inner_w, p.inner_h, p.r_in, -1, p.z_plate)
    # corner screw bosses
    for (x, y) in p.boss_pts:
        b = zcyl(x, y, 0, p.z_plate + 0.01, p.boss_r)
        b = b - zcyl(x, y, -1, p.boss_hole_depth + 1, p.boss_hole / 2)
        s = s + b
    # OLED standoffs behind the panel
    ox, oy = p.oled_pos
    hx, hy = p.oled_holes
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = ox + sx * hx / 2, oy + sy * hy / 2
            post = zcyl(x, y, p.z_plate - p.oled_post_h, p.oled_post_h + 0.01, p.oled_post_d / 2)
            post = post - zcyl(x, y, p.z_plate - p.oled_post_h - 1, p.oled_pilot_depth + 1, p.oled_pilot_d / 2)
            s = s + post
    # zip-tie bridge for the ribbon strain relief (hangs off the bottom wall). The shell prints front-face-down, so
    # the layers build from z = depth toward z = 0 and the bridge face toward +z (the bed side) is the overhang:
    # it is a 45 degree chamfer from the wall (z = bz1) down to the tip (z = bz1 - d), so nothing is flat.
    yw = -p.box_h / 2 + p.wall
    bz0, bz1 = p.tie_bridge_z
    d = p.tie_bridge_d
    x0 = p.cable_x - p.tie_bridge_w / 2
    prof = [(x0, yw - 0.5, bz1), (x0, yw, bz1), (x0, yw + d, bz1 - d), (x0, yw + d, bz0), (x0, yw - 0.5, bz0)]
    br = extrude(make_face(Polyline(*prof, close=True)), amount=p.tie_bridge_w, dir=(1, 0, 0))
    for sx in (-1, 1):
        br = br - box(p.cable_x + sx * p.tie_dx - p.tie_slot[0] / 2, yw + d / 2 - p.tie_slot[1] / 2, bz0 - 1,
                      p.cable_x + sx * p.tie_dx + p.tie_slot[0] / 2, yw + d / 2 + p.tie_slot[1] / 2, bz1 + 1)
    s = s + br
    # ---- front plate cut-outs
    s = s - front_hole(p.estop_pos, p.estop_hole)
    s = s - front_hole(p.toggle_pos, p.toggle_hole)
    s = s - front_hole(p.rapid_pos, p.rapid_hole)
    s = s - front_hole(p.enc_pos, p.enc_hole)
    if p.enc_recess > 0:                 # thin the panel from behind so the M7 bushing still clears the nut
        s = s - zcyl(p.enc_pos[0], p.enc_pos[1], p.z_plate - 1, p.enc_recess + 1, p.enc_recess_d / 2)
    if p.enc_pin:
        s = s - front_hole((p.enc_pos[0], p.enc_pos[1] + p.enc_pin_off), p.enc_pin_d)
    s = s - Pos(ox, oy + p.oled_win_dy, p.z_plate - 1) * Box(*p.oled_win, p.front_t + 2, align=MIN)
    # ---- cable entry through the bottom wall
    yb = -p.box_h / 2
    zc = p.cable_z
    s = s - box(p.cable_x - p.cable_w / 2, yb - 1, zc - p.cable_h / 2, p.cable_x + p.cable_w / 2, yb + p.wall + 1, zc + p.cable_h / 2)
    # ---- engraved labels (front face)
    for txt, (x, y) in p.labels:
        try:
            t = Text(txt, p.text_size, font=p.font)
            eng = Pos(x, y, p.depth - p.text_depth) * extrude(t, amount=p.text_depth + 0.5)
            s = s - eng
        except Exception as e:                       # fonts can fail; labels are listed in the README
            print("text failed for %s: %s" % (txt, e))
    return s


def back_cover():
    s = rprism(p.box_w, p.box_h, p.r_out, -p.cover_t, 0)
    g = p.lip_gap
    lip = rprism(p.inner_w - 2 * g, p.inner_h - 2 * g, max(p.r_in - g, 0.5), 0, p.lip_h)
    lip = lip - rprism(p.inner_w - 2 * g - 2 * p.lip_t, p.inner_h - 2 * g - 2 * p.lip_t, 0.5, -1, p.lip_h + 1)
    for (x, y) in p.boss_pts:
        lip = lip - zcyl(x, y, -1, p.lip_h + 2, p.boss_r + 0.4)
    s = s + lip
    for (x, y) in p.boss_pts:                          # M3 clearance hole + counterbore for the pan head (not a countersink)
        s = s - zcyl(x, y, -p.cover_t - 1, p.cover_t + 2, 1.7)
        s = s - zcyl(x, y, -p.cbore_depth, p.cbore_depth + 1, p.cbore_d / 2)
    # magnet pockets: open on the inside, skin outside
    pocket_h = p.magnet_h + p.magnet_clear
    for (x, y) in p.magnet_pos:
        s = s - zcyl(x, y, -pocket_h, pocket_h + 0.01, (p.magnet_d + p.magnet_clear) / 2)
    # adapter board posts
    ax, ay = p.adapter_pos
    hx, hy = p.adapter_holes
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = ax + sx * hx / 2, ay + sy * hy / 2
            post = zcyl(x, y, -0.01, p.adapter_post_h + 0.01, p.adapter_post_d / 2)
            post = post - zcyl(x, y, p.adapter_post_h - p.adapter_pilot_depth, p.adapter_pilot_depth + 1, p.adapter_pilot_d / 2)
            s = s + post
    return s


# ------------------------------------------------------------------ hardware placeholders (render / interference checks only)
def hardware():
    zp = p.z_plate
    zf = p.depth
    h = {}
    ex, ey = p.estop_pos
    h["estop"] = Compound(children=[
        zcyl(ex, ey, zp - p.estop_depth, p.estop_depth + p.front_t, 11.0),               # neck/body through the hole
        zcyl(ex, ey, zp - p.estop_depth, p.estop_depth - 8, 15.0),                       # contact block, rear
        zcyl(ex, ey, zf, p.estop_head_h, p.estop_head_d / 2)])
    tx, ty = p.toggle_pos
    h["toggle"] = Compound(children=[
        zcyl(tx, ty, zp - p.toggle_depth, p.toggle_depth + p.front_t, 5.5),
        box(tx - 6.5, ty - 5.5, zp - 15.0, tx + 6.5, ty + 5.5, zp),
        zcyl(tx, ty, zf, 4.0, 8.0),
        Pos(tx, ty, zf + 4.0) * Rot(0, 0, 0) * Cylinder(1.8, 14.0, align=MIN)])
    rx, ry = p.rapid_pos
    h["rapid"] = Compound(children=[
        zcyl(rx, ry, zp - p.rapid_depth, p.rapid_depth + p.front_t, 7.5),
        zcyl(rx, ry, zf, 8.0, 9.5)])
    nx, ny = p.enc_pos
    h["encoder"] = Compound(children=[
        zcyl(nx, ny, zp - p.enc_depth, p.enc_depth + p.front_t + 6.0, 3.4),
        box(nx - 6.0, ny - 6.5, zp - 7.0, nx + 6.0, ny + 6.5, zp),
        zcyl(nx, ny, zf + 6.0, 15.0, p.enc_knob_d / 2)])
    ox, oy = p.oled_pos
    w, d = p.oled_pcb
    z_mod = zp - p.oled_post_h
    h["oled"] = Compound(children=[
        box(ox - w / 2, oy - d / 2, z_mod - 1.6, ox + w / 2, oy + d / 2, z_mod),
        box(ox - 15.0, oy + p.oled_win_dy - 7.0, z_mod, ox + 15.0, oy + p.oled_win_dy + 7.0, z_mod + 1.8)])   # glass
    ax, ay = p.adapter_pos
    aw, ad, at_ = p.adapter_size
    zb = p.adapter_post_h
    idc_y = ay - ad / 2 + 9.0
    h["adapter"] = Compound(children=[
        box(ax - aw / 2, ay - ad / 2, zb, ax + aw / 2, ay + ad / 2, zb + at_),
        box(p.cable_x - 9.0, idc_y - 2.5, zb + at_, p.cable_x + 9.0, idc_y + 2.5, zb + at_ + 10.0),          # IDC header
        box(p.cable_x - 11.0, idc_y - 4.5, zb + at_ + 10.0, p.cable_x + 11.0, idc_y + 4.5, zb + at_ + 19.5)])  # ribbon plug
    # ribbon cable from the plug out through the slot
    h["ribbon"] = box(p.cable_x - 8.9, -p.box_h / 2 - 45.0, p.cable_z - 0.5, p.cable_x + 8.9, idc_y, p.cable_z + 0.5)
    return h


COLORS = {"shell": "#9aa6b2", "cover": "#c4ccd4", "estop": "#d42a2a", "toggle": "#b0b0b0", "rapid": "#e6a21a",
          "encoder": "#303030", "oled": "#1a3a8a", "adapter": "#1b7f3a", "ribbon": "#8a8a8a"}


# ------------------------------------------------------------------ build + export
def interference(parts, hw):
    bad = 0
    for sname, sh in parts.items():
        for hname, piece in hw.items():
            try:
                v = (sh & piece).volume
            except Exception:
                v = 0.0
            if v > 0.05:
                bad += 1
                print("  INTERFERENCE %-11s x %-8s %.2f mm3" % (sname, hname, v))
    print("interference check:", "FAILED (%d)" % bad if bad else "clean")
    return bad


def build():
    os.makedirs(OUT, exist_ok=True)
    parts = {"shell": shell(), "back_cover": back_cover()}
    for name, sh in parts.items():
        sh.label = name
        export_step(sh, os.path.join(OUT, name + ".step"))
        export_stl(sh, os.path.join(OUT, name + ".stl"))
    hw = hardware()
    asm = Compound(children=list(hw.values()))
    asm.label = "hardware_placeholder"
    export_step(asm, os.path.join(OUT, "hardware_placeholder.step"))
    export_stl(asm, os.path.join(OUT, "hardware_placeholder.stl"))
    return parts, hw


def render(parts, hw):
    tool = os.path.join(HERE, "..", "tools", "render.py")
    items = {"shell": parts["shell"], "cover": parts["back_cover"], **hw}

    def dump(prefix, rot):
        for nm, sh in items.items():
            export_stl(rot * sh, os.path.join(OUT, "_%s_%s.stl" % (prefix, nm)))

    def spec(prefix, nm, dz=0.0):
        return "%s:%s:%g" % (os.path.join(OUT, "_%s_%s.stl" % (prefix, nm)), COLORS[nm], dz)

    def run(out, view, *specs):
        subprocess.run([sys.executable, tool, os.path.join(OUT, out), "--view", view, *specs], check=True)

    from build123d import Location
    dump("f", Location((0, 0, 0)))
    allf = [spec("f", n) for n in items if n != "cover"]
    run("render_front.png", "front", *allf)
    run("render_iso.png", "iso", *allf, spec("f", "cover"))
    # exploded: shell seen from behind (flipped 180 deg about X, front face down), the back cover with its adapter
    # board and ribbon laid beside it inside-face up, shifted +x (baked into the STL).
    dump("r", Rot(180, 0, 0))
    side = Pos(p.box_w + 30.0, 0, 0)
    for nm in ("cover", "adapter", "ribbon"):
        export_stl(side * items[nm], os.path.join(OUT, "_x_%s.stl" % nm))
    ex = [spec("r", n) for n in items if n not in ("cover", "adapter", "ribbon")]
    ex += [spec("x", n) for n in ("cover", "adapter", "ribbon")]
    run("render_exploded.png", "iso", *ex)
    # inside view: straight into the open shell from behind (hardware in place, cover and adapter removed)
    run("render_inside.png", "front", *[spec("r", n) for n in items if n not in ("cover", "adapter", "ribbon")])


def info(parts):
    print("encoder: panel %.1f + nut %.1f + %.1f spare = %.1f vs bushing %.1f -> rear recess %.2f mm"
          % (p.front_t, p.enc_nut_t, p.enc_thread_spare, p.front_t + p.enc_nut_t + p.enc_thread_spare, p.enc_bushing_len, p.enc_recess))
    print("box %.0f x %.0f, depth %.0f mm (+%.0f cover); cavity %.0f; E-stop body %.0f mm behind the plate -> %.0f mm spare"
          % (p.box_w, p.box_h, p.depth, p.cover_t, p.cavity, p.estop_depth, p.cavity - p.estop_depth))
    for name, sh in parts.items():
        bb = sh.bounding_box()
        print("%-11s %.1f x %.1f x %.1f mm  vol %.1f cm3  valid=%s" % (name, bb.size.X, bb.size.Y, bb.size.Z, sh.volume / 1000, sh.is_valid))


if __name__ == "__main__":
    parts, hw = build()
    info(parts)
    interference(parts, hw)
    render(parts, hw)
