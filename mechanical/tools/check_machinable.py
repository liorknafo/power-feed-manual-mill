"""Machinability check: find sharp INTERNAL corners that a rotating cutter cannot make.

An end mill leaves its own radius in every internal corner that runs parallel to its axis (a pocket's vertical
corners, a slot's end corners). A lathe leaves its tool-nose radius in internal corners of revolved profiles.
This script flags, for every machined part, the concave straight edges that run along the part's tool axis
(z in the part's own frame): each needs a corner radius >= the cutter radius, or a relief, in the model.

usage: mechanical/.venv/bin/python mechanical/tools/check_machinable.py
"""

import os
import sys

from build123d import GeomType, Vector

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "clutch"))
sys.path.insert(0, os.path.join(HERE, "..", "feed_unit"))
import clutch as C  # noqa: E402
import feed_unit as FU  # noqa: E402


def concave_axial_edges(shape, axis=Vector(0, 0, 1), tol_deg=5.0, min_len=0.3):
    """Concave straight edges parallel to `axis`: internal corners an end mill along `axis` cannot make sharp."""
    faces = shape.faces()
    edge_faces = {}
    for fi, f in enumerate(faces):
        for e in f.edges():
            key = None
            for k in edge_faces:
                if k.is_same(e):
                    key = k
                    break
            edge_faces.setdefault(key or e, []).append(f)
    out = []
    for e, fs in edge_faces.items():
        if e.geom_type != GeomType.LINE or len(fs) != 2 or e.length < min_len:
            continue
        d = (e.position_at(1) - e.position_at(0)).normalized()
        if abs(d.dot(axis.normalized())) < 1 - (tol_deg / 57.3) ** 2 / 2:
            continue
        p = e.position_at(0.5)
        n1, n2 = fs[0].normal_at(p), fs[1].normal_at(p)
        if n1.dot(n2) > 0.985:          # (near-)tangent faces, e.g. facets of an arc: no real corner
            continue
        eps = 0.05
        if shape.is_inside(p + (n1 - n2) * eps) or shape.is_inside(p + (n2 - n1) * eps):
            out.append((p, e.length))
    return out


def main():
    cov, _ = C.cover()
    parts = {
        # clutch (own frame: z = axis)
        "clutch field_housing": C.field_housing(), "clutch rotor": C.rotor(), "clutch armature": C.armature(),
        "clutch output_hub": C.output_hub(), "clutch output_shaft": C.output_shaft(), "clutch cover": cov,
        # feed unit (frame: z = W, the plate's thickness / the stub's axis)
        "plate": FU.plate(), "stub_shaft": FU.stub_shaft(), "oldham_disc": FU.oldham(),
        "encoder_disc": FU.encoder()[3], "tension_block": FU.tension_block(),
    }
    total = 0
    for name, s in parts.items():
        hits = concave_axial_edges(s)
        total += len(hits)
        print("%-22s %3d sharp internal axial corners" % (name, len(hits)))
        for p, ln in hits[:12]:
            print("     at (%.1f, %.1f, %.1f), %.1f long" % (p.X, p.Y, p.Z, ln))
        if len(hits) > 12:
            print("     ... %d more" % (len(hits) - 12))
    print("TOTAL", total)


if __name__ == "__main__":
    main()
