"""Shaded PNG renders of STL parts with VTK (pyvista, off-screen).

usage: render.py <out.png> [--cut] [--view iso|side|front] part.stl:color[:dz] [...]
An optional :dz moves the part along Z (exploded views).
--cut clips away the y < 0 half (half-section) so the inside is visible.
"""

import argparse

import pyvista as pv

pv.OFF_SCREEN = True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("out")
    ap.add_argument("parts", nargs="+")
    ap.add_argument("--cut", action="store_true")
    ap.add_argument("--cut-normal", default="0,-1,0", help="clip-plane normal through the origin")
    ap.add_argument("--view", default="iso")
    ap.add_argument("--size", type=int, default=1400)
    ap.add_argument("--up", default="", help="camera up vector, e.g. 0,1,0")
    a = ap.parse_args()

    p = pv.Plotter(off_screen=True, window_size=(a.size, a.size))
    p.set_background("white")
    p.enable_anti_aliasing("ssaa")
    for spec in a.parts:
        bits = spec.split(":")
        path, color = bits[0], bits[1]
        dz = float(bits[2]) if len(bits) > 2 else 0.0
        m = pv.read(path).translate((0, 0, dz), inplace=False)
        if a.cut:
            m = m.clip(normal=tuple(float(v) for v in a.cut_normal.split(",")), origin=(0, 0, 0), crinkle=False)
            if m.n_points == 0:
                continue
            m = m.extract_surface()
        p.add_mesh(m, color=color, smooth_shading=False, specular=0.25, show_edges=False)
        if a.cut:
            # draw the cut outline so sectioned parts read clearly
            edges = pv.read(path).translate((0, 0, dz), inplace=False).slice(normal=tuple(float(v) for v in a.cut_normal.split(",")), origin=(0, 0, 0))
            if edges.n_points:
                p.add_mesh(edges, color="black", line_width=2)
    if a.view == "side":
        p.view_xz()
        p.camera.azimuth = 0
    elif a.view == "sidex":
        p.view_yz()
    elif a.view == "front":
        p.view_xy()
    else:
        p.view_isometric()
        p.camera.azimuth = -35
        p.camera.elevation = -10
    if a.up:
        p.camera.up = tuple(float(v) for v in a.up.split(","))
        if a.view == "iso":
            p.camera.position = (420, 260, 480)
            p.camera.focal_point = (0, -40, 60)
        p.reset_camera()
    p.camera.zoom(1.15)
    p.screenshot(a.out)


if __name__ == "__main__":
    main()
