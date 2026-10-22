"""The feed unit's committed renders (feed_unit/out/render_*.png) from the STLs feed_unit.py writes.

usage: render_feed_unit.py        (run feed_unit.py first)
Same iso camera as `render.py --view iso --up 0,1,0`; every bought part in its own colour. The section is cut through
the leadscrew axis (x = 0) and seen from the front.
"""

import os

import pyvista as pv

pv.OFF_SCREEN = True
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "feed_unit", "out")

COL = {
    "mill_reference": "#3d4147", "plate": "#b8c2cc", "standoffs": "#9aa0a6", "stub_shaft": "#cfd6dd", "oldham_disc": "#c87533",
    "pulley_40T": "#c0c6cc", "pulley_20T": "#c0c6cc", "belt": "#222222", "belt_guard": "#e0dccf", "bearings_6002": "#8c939b",
    "encoder": "#2b2b2b", "encoder_disc": "#9fb4c8", "encoder_posts": "#d4b24c", "encoder_coupling": "#3a6fb0",
    "tension_block": "#7d848c", "clutch_cover": "#b9c2cb", "clutch_field_housing": "#c87533", "clutch_hub": "#9aa7b5",
    "motor": "#34373c", "psu": "#5b8fd0", "mains_tray": "#a7b0ba", "mains_tray_lid": "#b9c2cb", "mains_inlet": "#1e1e1e",
    "mains_cord_plug": "#2e2e2e", "pcb_placeholder": "#1f6f3f", "fan": "#262626", "end_wall_parts": "#2a2a2a",
    "control_estop": "#c0392b", "control_estop_plate": "#f2c40f", "control_toggle": "#b8bec4", "control_rapid": "#8a9096",
    "control_dial": "#2a2a2a", "control_oled": "#203040", "control_adapter": "#2f7a3a",
    "shell": "#e8e4da", "hood": "#f0ece2", "door": "#ddd8ca", "door_pin": "#9aa0a6",
    "fx_plate": "#8c939b", "fx_housing": "#8c939b", "fx_mains": "#8c939b", "fx_clutch": "#8c939b",
}
DRIVE = ["mill_reference", "plate", "standoffs", "stub_shaft", "oldham_disc", "pulley_40T", "pulley_20T", "belt",
         "belt_guard", "encoder", "encoder_disc", "encoder_posts", "encoder_coupling", "tension_block", "clutch_cover",
         "clutch_field_housing", "motor", "fx_plate", "fx_clutch"]
MAINS = ["psu", "mains_tray", "mains_tray_lid", "mains_inlet", "mains_cord_plug", "fx_mains"]
INSIDE = ["pcb_placeholder", "fan"]
CONTROLS = ["control_estop", "control_estop_plate", "control_toggle", "control_rapid", "control_dial", "control_oled",
            "control_adapter"]
HOUSING = ["shell", "hood", "door", "door_pin", "end_wall_parts", "fx_housing"]

_cache = {}


def mesh(name):
    if name not in _cache:
        _cache[name] = pv.read(os.path.join(OUT, name + ".stl"))
    return _cache[name]


def shot(png, names, view="iso", cut=None, up=None, zoom=1.15):
    p = pv.Plotter(off_screen=True, window_size=(1400, 1400))
    p.set_background("white")
    p.enable_anti_aliasing("ssaa")
    for n in names:
        m = mesh(n)
        if cut is not None:
            m = m.clip(normal=cut, origin=(0, 0, 0), crinkle=False)
            if m.n_points == 0:
                continue
            m = m.extract_surface()
        p.add_mesh(m, color=COL.get(n, "#999999"), smooth_shading=False, specular=0.25)
        if cut is not None:
            edges = mesh(n).slice(normal=cut, origin=(0, 0, 0))
            if edges.n_points:
                p.add_mesh(edges, color="black", line_width=2)
    if view == "front":                 # from the operator's side (+x)
        p.view_yz(negative=False)
        p.camera.up = (0, 1, 0)
    elif view == "end":                 # from the end wall (+W)
        p.view_xy()
        p.camera.up = (0, 1, 0)
    elif view == "print":               # the shell standing on its end wall, as printed
        p.view_isometric()
        p.camera.up = (0, 0, -1)
        p.camera.azimuth = 30
    else:                               # from the front, above and the end-wall side (render.py --view iso --up 0,1,0)
        p.camera.position = (420, 260, 480)
        p.camera.focal_point = (0, -40, 60)
        p.camera.up = (0, 1, 0)
    if up:
        p.camera.up = up
    p.reset_camera()
    p.camera.zoom(zoom)
    p.screenshot(os.path.join(OUT, png))
    print("wrote", png)


def main():
    everything = DRIVE + MAINS + INSIDE + CONTROLS + HOUSING
    shot("render_iso.png", everything)
    shot("render_front.png", everything, view="front")
    shot("render_end.png", everything, view="end")
    shot("render_open.png", [n for n in everything if n not in ("hood", "fx_housing")])
    shot("render_section.png", [n for n in everything if not n.startswith("fx_")], cut=(1, 0, 0), view="front", zoom=1.0)
    shot("render_mains_tray.png", MAINS, zoom=1.3)
    shot("render_shell_print.png", ["shell"], view="print", zoom=1.1)


if __name__ == "__main__":
    main()
