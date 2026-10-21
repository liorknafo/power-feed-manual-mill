"""How the clutch works: half-sections through its axis and the tooth ring unrolled flat, released (coil off) and
engaged (coil on), cut from the real CAD. Writes mechanical/clutch/out/clutch_states.png.

usage: mechanical/.venv/bin/python mechanical/tools/clutch_states.py
"""
import os
import sys
import tempfile

import numpy as np
import trimesh
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MP, Rectangle
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
sys.path.insert(0, ROOT + "/clutch")
import clutch as C
from build123d import Compound, Pos, export_stl
p = C.p
OUT = os.path.join(ROOT, "clutch", "out")
TMP = tempfile.mkdtemp()
cov, z_front = C.cover()
common = {"motor": C.motor_stub(), "field_housing": C.field_housing(), "coil": C.coil(), "rotor": C.rotor(),
          "hub": C.output_hub(), "bushes": Compound(children=C.bushes()), "shaft": C.output_shaft(), "cover": cov,
          "bearings": Compound(children=C.bearings(z_front))}
arm = C.armature()


def state(released):
    springs, washers, clips = C.return_springs(released)
    return dict(armature=Pos(0, 0, p.stroke if released else 0.0) * arm, pins=Compound(children=C.drive_pins(released)),
                springs=Compound(children=springs), clips=Compound(children=washers + clips))


states = {"off": state(True), "on": state(False)}
COL = {"motor": "#5a5d63", "field_housing": "#8a8f99", "coil": "#c87533", "rotor": "#4f7fbf", "armature": "#d9534f",
       "pins": "#3b3f46", "springs": "#2f6fd6", "clips": "#d4b24c", "bushes": "#b5823a", "hub": "#9aa7b5", "shaft": "#cfd6dd",
       "cover": "#c9d1d9", "bearings": "#e0b050"}
T = np.array([[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 0, 1.0]])   # plane x = 0 -> (u, v) = (z, y)

def polys(shape, name):
    f = os.path.join(TMP, name + ".stl"); export_stl(shape, f, tolerance=0.01, angular_tolerance=0.05)
    m = trimesh.load(f)
    s = m.section(plane_origin=[0, 0, 0], plane_normal=[1, 0, 0])
    if s is None: return []
    p2, _ = s.to_2D(to_2D=T)
    return p2.polygons_full

def draw(ax, state, xlim, ylim, labels):
    parts = dict(common); parts.update(states[state])
    for name, shape in parts.items():
        for pg in polys(shape, state + "_" + name):
            ax.add_patch(MP(np.array(pg.exterior.coords), closed=True, fc=COL[name], ec="black", lw=0.4))
            for h in pg.interiors:
                ax.add_patch(MP(np.array(h.coords), closed=True, fc="white", ec="black", lw=0.4))
    ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.set_aspect("equal"); ax.set_xticks([]); ax.set_yticks([])

big = (-12, 75), (-2, 45)
R = 35.0                                             # mid tooth radius
def profile(shape, name, z_from, direction, n=900):
    """Tooth surface height along the circle r = R (unrolled), by ray casting the CAD mesh."""
    f = os.path.join(TMP, name + "_ray.stl"); export_stl(shape, f, tolerance=0.005, angular_tolerance=0.02)
    m = trimesh.load(f)
    a0 = 0.5 * np.pi - 1.5 * 2 * np.pi / p.teeth
    th = np.linspace(a0, a0 + 3 * 2 * np.pi / p.teeth, n)
    org = np.stack([R * np.cos(th), R * np.sin(th), np.full(n, z_from)], 1)
    loc, idx, _ = m.ray.intersects_location(org, np.tile([0, 0, direction], (n, 1)), multiple_hits=False)
    z = np.full(n, np.nan); z[idx] = loc[:, 2]
    return (th - a0) * R, z

fig, axs = plt.subplots(2, 2, figsize=(15, 11), gridspec_kw=dict(width_ratios=[1.5, 1]))
zp = C.Z_POLE
rot = C.rotor()
for row, st in enumerate(("off", "on")):
    draw(axs[row, 0], st, *big, True)
    axs[row, 0].set_title(("COIL OFF (no power): the springs pull the armature back onto the hub, %.2f mm off the rotor. Teeth apart"
                           % (p.engaged_gap + p.stroke)) if st == "off" else
                          ("COIL ON: the magnet pulls the armature %.1f mm onto the rotor, compressing the springs. Teeth mesh; the pins drive the hub" % p.stroke),
                          fontsize=11, color="#b00" if st == "off" else "#060", loc="left")
    ax = axs[row, 1]
    u, zr = profile(rot, st + "_rot", zp + 6, -1)
    u, za = profile(states[st]["armature"], st + "_arm", zp - 6, 1)
    lo, hi = zp - 1.6, zp + 2.6
    ax.fill_between(u, lo, zr, color=COL["rotor"], ec="k", lw=0.6)
    ax.fill_between(u, za, hi, color=COL["armature"], ec="k", lw=0.6)
    tip_r, tip_a = np.nanmax(zr), np.nanmin(za)
    if st == "off":
        for zz in (tip_r, tip_a):
            ax.axhline(zz, color="k", lw=0.8, ls="--")
        ax.annotate("", xy=(u[-1] * 0.93, tip_a), xytext=(u[-1] * 0.93, tip_r), arrowprops=dict(arrowstyle="<->", color="k"))
        ax.text(u[-1] * 0.5, (tip_a + tip_r) / 2, "tips %.2f mm apart: they pass\nwithout touching" % (tip_a - tip_r),
                va="center", fontsize=9, bbox=dict(fc="white", ec="none", pad=1))
    ax.text(0.2, lo + 0.15, "rotor teeth (turn with the motor)", color="white", fontsize=9)
    ax.text(0.2, hi - 0.35, "armature teeth (turn the output)", color="white", fontsize=9)
    ax.set_xlim(0, u[-1]); ax.set_ylim(lo, hi); ax.set_aspect("equal"); ax.set_yticks([])
    ax.set_xlabel("around the tooth ring at r = %.0f mm, unrolled flat (mm); 3 of the %d teeth" % (R, p.teeth), fontsize=9)
    ax.set_title("teeth apart" if st == "off" else "teeth meshed: flanks pressed together, the motor turns the output", fontsize=10)
from matplotlib.patches import Patch
names = {"motor": "motor (shaft + flange)", "field_housing": "field housing (fixed, steel)", "coil": "coil", "rotor": "rotor (on motor shaft)",
         "armature": "armature (moves 1.1 mm)", "pins": "drive pins (bonded in the armature)", "springs": "return springs", "clips": "washer + E-clip", "bushes": "bronze bushes", "hub": "output hub",
         "shaft": "output shaft -> 20T pulley", "cover": "cover (aluminium)", "bearings": "bearings"}
fig.legend(handles=[Patch(fc=COL[k], ec="k", label=v) for k, v in names.items()], loc="lower center", ncol=6, fontsize=10)
fig.suptitle("Left: clutch cut through its axis (motor left, output right; upper half only). Right: the tooth ring unrolled flat.", fontsize=12)
plt.tight_layout(rect=(0, 0.06, 1, 0.96)); plt.subplots_adjust(hspace=0.12)
plt.savefig(os.path.join(OUT, "clutch_states.png"), dpi=110)
print("wrote", os.path.join(OUT, "clutch_states.png"))
