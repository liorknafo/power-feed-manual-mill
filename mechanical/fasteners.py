"""ISO fasteners for the models, from bd_warehouse: screws, nuts, set screws and washers placed by a point and a
direction, circlips, and the internal threads of tapped holes and heat-set inserts.

threads=True gives real ISO threads (for the viewer); threads=False gives plain shanks (fast, for the checks). The
machined parts themselves keep their tap-drill holes: an internal thread is a separate shape, shown in the hole.
"""

from build123d import Align, Compound, Cylinder, Plane, Pos

from bd_warehouse.fastener import (ButtonHeadScrew, CounterSunkScrew, HexNut, PlainWasher, SetScrew,
                                   SocketHeadCapScrew)
from bd_warehouse.thread import IsoThread

PITCH = {3: 0.5, 4: 0.7, 5: 0.8, 6: 1.0}
MIN = (Align.CENTER, Align.CENTER, Align.MIN)
_cache = {}


def _make(kind, d, length, threads):
    key = (kind, d, length, threads)
    if key not in _cache:
        size = "M%d-%g" % (d, PITCH[d])
        simple = not threads
        if kind == "shcs":            # ISO 4762 socket head cap screw
            obj = SocketHeadCapScrew(size, length, "iso4762", simple=simple)
        elif kind == "csk":           # ISO 10642 countersunk socket screw
            obj = CounterSunkScrew(size, length, "iso10642", simple=simple)
        elif kind == "button":        # ISO 7380 button head
            obj = ButtonHeadScrew(size, length, "iso7380_1", simple=simple)
        elif kind == "set":           # ISO 4026 flat-point set screw
            obj = SetScrew(size, length, "iso4026", simple=simple)
        elif kind == "nut":           # ISO 4032 hex nut
            obj = HexNut(size, "iso4032", simple=simple)
        elif kind == "washer":        # ISO 7089 plain washer
            obj = PlainWasher("M%d" % d, "iso7089")
        elif kind == "thread":        # internal thread of a tapped hole, `length` deep
            obj = _thread(d, length) if threads else None
        else:
            raise ValueError(kind)
        _cache[key] = obj
    return _cache[key]


def _thread(d, length):
    """Internal ISO thread; a few lengths trip an edge case in bd_warehouse's faded ends, so fall back gracefully."""
    for finishes, ln in ((("fade", "fade"), length), (("square", "square"), length), (("fade", "fade"), length - 0.1)):
        try:
            return IsoThread(major_diameter=d, pitch=PITCH[d], length=ln, external=False, end_finishes=finishes)
        except Exception:  # noqa: BLE001 - try the next variant
            continue
    raise ValueError("no internal thread M%d x %s" % (d, length))


def at(kind, d, length, point, direction, threads=True):
    """A fastener with its head's seat (a screw), its seat face (a nut, a washer) or its mouth (a tapped hole's thread)
    at `point`; a screw's shank and a hole's thread run along `direction`, a nut or washer stacks along it. Returns a
    Compound of solids, each moved on its own (a Compound moved as a whole confuses the booleans)."""
    obj = _make(kind, d, length, threads)
    if obj is None:
        return None
    dx, dy, dz = direction
    screw = kind in ("shcs", "csk", "button", "set")
    z_dir = (-dx, -dy, -dz) if screw else (dx, dy, dz)       # bd_warehouse screws point their shank along -Z
    loc = Plane(origin=point, z_dir=z_dir).location
    return Compound(children=[so.moved(loc) for so in obj.solids()])


def circlip(r_in, r_out, t, z, x=0.0, y=0.0):
    """A retaining ring (DIN 471 / 472) as a plain flat ring, seated at height z."""
    return Pos(x, y, z) * (Cylinder(r_out, t, align=MIN) - Cylinder(r_in, t, align=MIN))
