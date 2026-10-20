"""Generate KiCad 9 schematics from a Python circuit description.

Every used pin gets a short wire stub ending in a net label, so connectivity
comes only from net names. This keeps the generated sheets correct by
construction and lets ERC plus the exported netlist verify the result.
"""

import copy
import os
import uuid as uuidlib

from sexp import Sym, dump, find, find_all, parse

KICAD_SHARE = "/Applications/KiCad/KiCad.app/Contents/SharedSupport"
SYM_DIR = os.path.join(KICAD_SHARE, "symbols")
FP_DIR = os.path.join(KICAD_SHARE, "footprints")

GRID = 2.54
STUB = 2.54
NS = uuidlib.UUID("6f1c2a52-6a8e-4b9a-9a5e-2b8f4d0c7e11")


def uid(*parts):
    """Deterministic UUID, so regenerating doesn't churn the files."""
    return str(uuidlib.uuid5(NS, "/".join(str(p) for p in parts)))


def snap(v, g=1.27):
    return round(round(v / g) * g, 4)


# ---------------------------------------------------------------- libraries

_lib_cache = {}


def _load_lib(lib):
    if lib not in _lib_cache:
        tree = parse(open(os.path.join(SYM_DIR, lib + ".kicad_sym")).read())
        _lib_cache[lib] = {s[1]: s for s in find_all(tree, "symbol")}
    return _lib_cache[lib]


def _rename_units(sym, old, new):
    for u in find_all(sym, "symbol"):
        if u[1].startswith(old + "_"):
            u[1] = new + u[1][len(old):]


def library_symbol(lib_id):
    """Flattened copy of a library symbol, named `lib:name` for lib_symbols."""
    lib, name = lib_id.split(":")
    syms = _load_lib(lib)
    sym = copy.deepcopy(syms[name])
    ext = find(sym, "extends")
    if ext:
        parent = library_symbol(lib + ":" + ext[1])
        parent_name = ext[1]
        merged = copy.deepcopy(parent)
        merged[1] = name
        _rename_units(merged, parent_name, name)
        # child properties override the parent's
        child_props = {p[1]: p for p in find_all(sym, "property")}
        merged = [x for x in merged if not (isinstance(x, list) and x and x[0] == "property" and x[1] in child_props)]
        insert_at = next(i for i, x in enumerate(merged) if isinstance(x, list) and x and x[0] == "symbol")
        for p in child_props.values():
            merged.insert(insert_at, p)
            insert_at += 1
        sym = merged
    else:
        sym[1] = name
    out = copy.deepcopy(sym)
    out[1] = lib_id
    return out


def custom_symbol(lib_id, pins, ref="U", value=None, footprint="", datasheet="~", description=""):
    """Rectangular symbol. pins: list of (number, name, etype, side) where side is
    'L', 'R', 'T' or 'B'. Pins on the same side are stacked in list order.
    A pin entry of None on a side inserts a gap."""
    name = lib_id.split(":")[1]
    sides = {"L": [], "R": [], "T": [], "B": []}
    for p in pins:
        sides[p[3]].append(p)
    n_lr = max(len(sides["L"]), len(sides["R"]), 1)
    n_tb = max(len(sides["T"]), len(sides["B"]), 1)
    h = (n_lr + 1) * GRID
    w = max(20.32, (n_tb + 1) * GRID)
    w = snap(w, GRID * 2)
    h = snap(h, GRID * 2)
    plen = 2.54
    body = [Sym("rectangle"), [Sym("start"), -w / 2, h / 2], [Sym("end"), w / 2, -h / 2],
            [Sym("stroke"), [Sym("width"), 0.254], [Sym("type"), Sym("default")]],
            [Sym("fill"), [Sym("type"), Sym("background")]]]
    pin_nodes = []

    def add(p, x, y, ang):
        num, pname, etype, _ = p
        if num is None:  # spacer
            return
        pin_nodes.append([Sym("pin"), Sym(etype), Sym("line"), [Sym("at"), x, y, ang], [Sym("length"), plen],
                          [Sym("name"), pname, _font()], [Sym("number"), str(num), _font()]])

    top = snap(h / 2 - GRID, GRID)
    for i, p in enumerate(sides["L"]):
        add(p, -w / 2 - plen, top - i * GRID, 0)
    for i, p in enumerate(sides["R"]):
        add(p, w / 2 + plen, top - i * GRID, 180)
    left = snap(-(len(sides["T"]) - 1) * GRID / 2, GRID)
    for i, p in enumerate(sides["T"]):
        add(p, left + i * GRID, h / 2 + plen, 270)
    left = snap(-(len(sides["B"]) - 1) * GRID / 2, GRID)
    for i, p in enumerate(sides["B"]):
        add(p, left + i * GRID, -h / 2 - plen, 90)

    def prop(k, v, x, y, hide=False):
        eff = _font()
        node = [Sym("property"), k, v, [Sym("at"), x, y, 0], [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]]]
                + ([[Sym("hide"), Sym("yes")]] if hide else [])]
        return node

    return [Sym("symbol"), lib_id,
            [Sym("pin_names"), [Sym("offset"), 1.016]],
            [Sym("exclude_from_sim"), Sym("no")], [Sym("in_bom"), Sym("yes")], [Sym("on_board"), Sym("yes")],
            prop("Reference", ref, 0, h / 2 + 1.27),
            prop("Value", value or name, 0, -h / 2 - 1.27),
            prop("Footprint", footprint, 0, 0, True),
            prop("Datasheet", datasheet, 0, 0, True),
            prop("Description", description, 0, 0, True),
            [Sym("symbol"), name + "_0_1", body],
            [Sym("symbol"), name + "_1_1"] + pin_nodes,
            [Sym("embedded_fonts"), Sym("no")]]


def _font(size=1.27):
    return [Sym("effects"), [Sym("font"), [Sym("size"), size, size]]]


def symbol_pins(sym):
    """{(unit, number): (name, etype, x, y, angle)}; unit 0 = shared by all units."""
    base = sym[1].split(":")[-1]
    pins = {}
    for u in find_all(sym, "symbol"):
        suffix = u[1][len(base) + 1:]
        unit = int(suffix.split("_")[0])
        for p in find_all(u, "pin"):
            at = find(p, "at")
            num = find(p, "number")[1]
            pins[(unit, num)] = (find(p, "name")[1], str(p[1]), float(at[1]), float(at[2]), int(float(at[3])) if len(at) > 3 else 0)
    return pins


def unit_count(sym):
    base = sym[1].split(":")[-1]
    units = set()
    for u in find_all(sym, "symbol"):
        units.add(int(u[1][len(base) + 1:].split("_")[0]))
    return max(units) if units else 1


PROJECT_FP_DIRS = {}  # lib nickname -> directory, for project-local footprint libraries


def footprint_exists(fp):
    if not fp:
        return False
    lib, name = fp.split(":")
    base = PROJECT_FP_DIRS.get(lib, os.path.join(FP_DIR, lib + ".pretty"))
    return os.path.exists(os.path.join(base, name + ".kicad_mod"))


# ---------------------------------------------------------------- circuit model


class Part:
    def __init__(self, ref_prefix, lib_id, value, footprint, conns, unit=1, props=None, symbol=None, ref=None, dnp=False):
        self.ref_prefix = ref_prefix
        self.lib_id = lib_id
        self.value = value
        self.footprint = footprint
        self.conns = conns  # {pin_number: net_name or None (no-connect)}
        self.unit = unit
        self.props = props or {}
        self.symbol = symbol  # custom symbol definition, if not from a library
        self.ref = ref
        self.dnp = dnp


class Sheet:
    def __init__(self, name, filename, title):
        self.name = name
        self.filename = filename
        self.title = title
        self.rows = []  # list of lists of Part (each row laid out left to right)
        self.notes = []

    def row(self, *parts):
        self.rows.append(list(parts))
        return parts

    def note(self, text):
        self.notes.append(text)


POWER_NETS = {"GND": "power:GND", "+3V3": "power:+3V3", "+5V": "power:+5V", "+36V": "power:+36V"}


class Project:
    def __init__(self, name, title):
        self.name = name
        self.title = title
        self.sheets = []
        self.root_uuid = uid(name, "root")
        self.symbols = {}  # lib_id -> flattened symbol
        self.pwr_flags = []  # (sheet, net)
        self.lib_name = "power_feed"

    def sheet(self, name, filename, title):
        s = Sheet(name, filename, title)
        self.sheets.append(s)
        return s

    # -- references
    def _assign_refs(self):
        counters = {}
        used = set()
        for s in self.sheets:
            for row in s.rows:
                for p in row:
                    if p.ref:
                        used.add(p.ref)
        for s in self.sheets:
            for row in s.rows:
                for p in row:
                    if p.ref:
                        continue
                    n = counters.get(p.ref_prefix, 0)
                    while True:
                        n += 1
                        cand = "%s%d" % (p.ref_prefix, n)
                        if cand not in used:
                            break
                    counters[p.ref_prefix] = n
                    used.add(cand)
                    p.ref = cand

    def _sym(self, part):
        if part.symbol is not None:
            self.symbols[part.lib_id] = part.symbol
        elif part.lib_id not in self.symbols:
            self.symbols[part.lib_id] = library_symbol(part.lib_id)
        return self.symbols[part.lib_id]

    def net_sheets(self):
        where = {}
        for s in self.sheets:
            for row in s.rows:
                for p in row:
                    for net in p.conns.values():
                        if net:
                            where.setdefault(net, set()).add(s.name)
        return where

    def intended_netlist(self):
        """{net: {(ref, pin)}} for verification against KiCad's export."""
        nets = {}
        for s in self.sheets:
            for row in s.rows:
                for p in row:
                    for pin, net in p.conns.items():
                        if net:
                            nets.setdefault(net, set()).add((p.ref, str(pin)))
        return nets

    # -- writing
    def write(self, outdir):
        os.makedirs(outdir, exist_ok=True)
        self._assign_refs()
        for s in self.sheets:
            for row in s.rows:
                for p in row:
                    self._sym(p)
        where = self.net_sheets()
        sheet_uuids = {s.name: uid(self.name, "sheet", s.name) for s in self.sheets}
        for i, s in enumerate(self.sheets):
            self._write_sheet(outdir, s, sheet_uuids[s.name], where, page=i + 2)
        self._write_root(outdir, sheet_uuids)
        self._write_project(outdir)
        self._write_lib_tables(outdir)

    def _write_lib_tables(self, outdir):
        """Project symbol library for custom parts + lib tables pointing at KiCad's stock libraries."""
        custom = {k: v for k, v in self.symbols.items() if k.startswith(self.lib_name + ":")}
        lib = [Sym("kicad_symbol_lib"), [Sym("version"), Sym("20241209")], [Sym("generator"), "build_schematic.py"],
               [Sym("generator_version"), "9.0"]]
        for lib_id, sym in sorted(custom.items()):
            s2 = copy.deepcopy(sym)
            s2[1] = lib_id.split(":")[1]
            lib.append(s2)
        with open(os.path.join(outdir, self.lib_name + ".kicad_sym"), "w") as f:
            f.write(dump(lib) + "\n")
        used_libs = sorted({k.split(":")[0] for k in self.symbols} - {self.lib_name})
        rows = ['  (lib (name "%s")(type "KiCad")(uri "${KICAD9_SYMBOL_DIR}/%s.kicad_sym")(options "")(descr ""))' % (l, l)
                for l in used_libs]
        rows.append('  (lib (name "%s")(type "KiCad")(uri "${KIPRJMOD}/%s.kicad_sym")(options "")(descr "project parts"))'
                    % (self.lib_name, self.lib_name))
        with open(os.path.join(outdir, "sym-lib-table"), "w") as f:
            f.write("(sym_lib_table\n  (version 7)\n" + "\n".join(rows) + "\n)\n")
        fp_libs = sorted({p.footprint.split(":")[0] for s in self.sheets for row in s.rows for p in row if p.footprint})
        rows = []
        for l in fp_libs:
            if l in PROJECT_FP_DIRS:
                rows.append('  (lib (name "%s")(type "KiCad")(uri "${KIPRJMOD}/%s.pretty")(options "")(descr "project footprints"))' % (l, l))
            else:
                rows.append('  (lib (name "%s")(type "KiCad")(uri "${KICAD9_FOOTPRINT_DIR}/%s.pretty")(options "")(descr ""))' % (l, l))
        with open(os.path.join(outdir, "fp-lib-table"), "w") as f:
            f.write("(fp_lib_table\n  (version 7)\n" + "\n".join(rows) + "\n)\n")

    def _write_root(self, outdir, sheet_uuids):
        items = []
        x, y = 25.4, 40.64
        for i, s in enumerate(self.sheets):
            su = sheet_uuids[s.name]
            w, h = 50.8, 20.32
            items.append([Sym("sheet"), [Sym("at"), x, y], [Sym("size"), w, h],
                          [Sym("exclude_from_sim"), Sym("no")], [Sym("in_bom"), Sym("yes")], [Sym("on_board"), Sym("yes")],
                          [Sym("dnp"), Sym("no")], [Sym("fields_autoplaced"), Sym("yes")],
                          [Sym("stroke"), [Sym("width"), 0.1524], [Sym("type"), Sym("solid")]],
                          [Sym("fill"), [Sym("color"), 0, 0, 0, 0.0]],
                          [Sym("uuid"), su],
                          [Sym("property"), "Sheetname", s.title, [Sym("at"), x, y - 0.7116, 0],
                           [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]], [Sym("justify"), Sym("left"), Sym("bottom")]]],
                          [Sym("property"), "Sheetfile", s.filename, [Sym("at"), x, y + h + 0.5846, 0],
                           [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]], [Sym("justify"), Sym("left"), Sym("top")]]],
                          [Sym("instances"), [Sym("project"), self.name,
                                              [Sym("path"), "/" + self.root_uuid, [Sym("page"), str(i + 2)]]]]])
            x += 63.5
            if x > 330:
                x = 25.4
                y += 38.1
        items.append(_text(self.title + " — generated by hardware/tools/build_schematic.py; edit the script, not this file.", 25.4, 25.4, 2.0))
        root = self._sch_header(self.root_uuid, "A3", self.title, "1") + [[Sym("lib_symbols")]] + items + [
            [Sym("sheet_instances"), [Sym("path"), "/", [Sym("page"), "1"]]], [Sym("embedded_fonts"), Sym("no")]]
        with open(os.path.join(outdir, self.name + ".kicad_sch"), "w") as f:
            f.write(dump(root) + "\n")

    def _sch_header(self, u, paper, title, page):
        return [Sym("kicad_sch"), [Sym("version"), Sym("20250114")], [Sym("generator"), "eeschema"],
                [Sym("generator_version"), "9.0"], [Sym("uuid"), u], [Sym("paper"), paper],
                [Sym("title_block"), [Sym("title"), title], [Sym("company"), self.title]]]

    def _write_sheet(self, outdir, s, su, where, page):
        path = "/" + self.root_uuid + "/" + su
        items = []
        used_ids = []
        y = 30.48
        x0 = 20.32
        maxw = 360.0
        pwr_count = [0]

        def place_label(net, x, y, outward):
            ang = {"L": 180, "R": 0, "U": 90, "D": 270}[outward]
            just = {"L": "right", "R": "left", "U": "left", "D": "right"}[outward]
            if net in POWER_NETS:
                pwr_count[0] += 1
                pid = POWER_NETS[net]
                psym = self.symbols.setdefault(pid, library_symbol(pid))
                # power symbol pin is at its origin; GND points down, supplies point up
                rot = 0
                is_gnd = net == "GND"
                rot = {"L": 270 if is_gnd else 90, "R": 90 if is_gnd else 270, "U": 180 if is_gnd else 0, "D": 0 if is_gnd else 180}[outward]
                ref = "#PWR%s%03d" % (s.name[:3].upper(), pwr_count[0])
                items.append(_symbol_instance(pid, ref, net, "", x, y, rot, self.name, path,
                                              uid(self.name, s.name, "pwr", pwr_count[0]), {"1": uid(self.name, s.name, "pwrpin", pwr_count[0])},
                                              in_bom=False, on_board=False))
                return
            u = uid(self.name, s.name, "label", net, x, y)
            if len(where.get(net, ())) > 1:
                items.append([Sym("global_label"), net, [Sym("shape"), Sym("passive")], [Sym("at"), x, y, ang],
                              [Sym("fields_autoplaced"), Sym("yes")],
                              [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]], [Sym("justify"), Sym(just)]],
                              [Sym("uuid"), u]])
            else:
                items.append([Sym("label"), net, [Sym("at"), x, y, ang],
                              [Sym("fields_autoplaced"), Sym("yes")],
                              [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]], [Sym("justify"), Sym(just), Sym("bottom")]],
                              [Sym("uuid"), u]])

        for note in s.notes:
            items.append(_text(note, x0, y, 1.5))
            y += 2.54 * (note.count("\n") + 2)
        y += 7.62

        for row in s.rows:
            # measure parts in this row
            metrics = []
            for p in row:
                sym = self.symbols[p.lib_id]
                pins = {k[1]: v for k, v in symbol_pins(sym).items() if k[0] in (0, p.unit)}
                xs = [v[2] for v in pins.values()] or [0]
                ys = [v[3] for v in pins.values()] or [0]
                lab = max([len(n or "") for n in p.conns.values()] + [4]) * 1.0 + 6
                left = -min(xs + [-5.08]) + (lab if any(v[4] == 0 for v in pins.values()) else 6)
                right = max(xs + [5.08]) + (lab if any(v[4] == 180 for v in pins.values()) else 6)
                up = max(ys + [5.08]) + 10
                down = -min(ys + [-5.08]) + 10
                metrics.append((p, pins, left, right, up, down))
            row_up = max(m[4] for m in metrics)
            row_down = max(m[5] for m in metrics)
            x = x0
            cy = snap(y + row_up, GRID)
            for p, pins, left, right, up, down in metrics:
                if x + left + right > maxw and x > x0:
                    # wrap
                    y = cy + row_down
                    cy = snap(y + row_up, GRID)
                    x = x0
                cx = snap(x + left, GRID)
                pin_uuids = {num: uid(self.name, p.ref, p.unit, "pin", num) for num in pins}
                props = dict(p.props)
                fpos = {}
                for prop in find_all(sym, "property"):
                    if prop[1] in ("Reference", "Value"):
                        at = find(prop, "at")
                        fpos[prop[1]] = (float(at[1]), -float(at[2]), int(float(at[3])) if len(at) > 3 else 0)
                items.append(_symbol_instance(p.lib_id, p.ref, p.value, p.footprint, cx, cy, 0, self.name, path,
                                              uid(self.name, p.ref, p.unit), pin_uuids, unit=p.unit, extra=props, dnp=p.dnp,
                                              field_pos=fpos))
                # wires + labels; stacked pins at the same point share one stub
                done = set()
                last_end = {}  # (outward, net) -> list of stub ends already labelled
                order = sorted(pins.items(), key=lambda kv: (kv[1][4], kv[1][2], -kv[1][3]))
                for num, (pname, etype, px, py, ang) in order:
                    if num not in p.conns:
                        raise ValueError("%s (%s) pin %s (%s) is not assigned" % (p.ref, p.lib_id, num, pname))
                    net = p.conns[num]
                    sx, sy = cx + px, cy - py
                    key = (round(sx, 3), round(sy, 3))
                    if key in done:
                        continue
                    done.add(key)
                    if net is None:
                        items.append([Sym("no_connect"), [Sym("at"), sx, sy], [Sym("uuid"), uid(self.name, p.ref, p.unit, "nc", num)]])
                        continue
                    outward = {0: "L", 180: "R", 90: "D", 270: "U"}[ang]
                    dx, dy = {"L": (-STUB, 0), "R": (STUB, 0), "U": (0, -STUB), "D": (0, STUB)}[outward]
                    ex, ey = sx + dx, sy + dy
                    items.append([Sym("wire"), [Sym("pts"), [Sym("xy"), sx, sy], [Sym("xy"), ex, ey]],
                                  [Sym("stroke"), [Sym("width"), 0], [Sym("type"), Sym("default")]],
                                  [Sym("uuid"), uid(self.name, p.ref, p.unit, "wire", num)]])
                    prev = last_end.get((outward, net))
                    # only bridge directly adjacent stubs, so the wire can never cross another pin's stub
                    if prev and abs(prev[0] - ex) + abs(prev[1] - ey) <= GRID + 0.01:
                        # bridge to the neighbouring stub of the same net instead of a second label
                        items.append([Sym("wire"), [Sym("pts"), [Sym("xy"), prev[0], prev[1]], [Sym("xy"), ex, ey]],
                                      [Sym("stroke"), [Sym("width"), 0], [Sym("type"), Sym("default")]],
                                      [Sym("uuid"), uid(self.name, p.ref, p.unit, "bridge", num)]])
                    else:
                        place_label(net, ex, ey, outward)
                    last_end[(outward, net)] = (ex, ey)
                for num in p.conns:
                    if num not in pins:
                        raise ValueError("%s (%s) has no pin %s in unit %d" % (p.ref, p.lib_id, num, p.unit))
                x = cx + right
            y = cy + row_down

        for net in [n for (sh, n) in self.pwr_flags if sh == s.name]:
            pwr_count[0] += 1
            fx, fy = snap(x0 + (20.32 * pwr_count[0]) % 300, GRID), 20.32
            items.append(_symbol_instance("power:PWR_FLAG", "#FLG%s%02d" % (s.name[:3].upper(), pwr_count[0]), "PWR_FLAG", "",
                                          fx, fy, 0, self.name, path, uid(self.name, s.name, "flag", net),
                                          {"1": uid(self.name, s.name, "flagpin", net)}, in_bom=False, on_board=False))
            self.symbols.setdefault("power:PWR_FLAG", library_symbol("power:PWR_FLAG"))
            items.append([Sym("wire"), [Sym("pts"), [Sym("xy"), fx, fy], [Sym("xy"), fx, fy + STUB]],
                          [Sym("stroke"), [Sym("width"), 0], [Sym("type"), Sym("default")]], [Sym("uuid"), uid(self.name, s.name, "flagw", net)]])
            place_label(net, fx, fy + STUB, "D")

        paper = "A2" if y > 270 else "A3"
        used = sorted({find(it, "lib_id")[1] for it in items if it[0] == "symbol"})
        lib_symbols = [Sym("lib_symbols")] + [self.symbols[l] for l in used]
        doc = self._sch_header(uid(self.name, "sheetfile", s.name), paper, s.title, str(page)) + [lib_symbols] + items + [
            [Sym("embedded_fonts"), Sym("no")]]
        with open(os.path.join(outdir, s.filename), "w") as f:
            f.write(dump(doc) + "\n")

    def _write_project(self, outdir):
        import json
        pro = {
            "meta": {"filename": self.name + ".kicad_pro", "version": 3},
            "sheets": [[self.root_uuid, "Root"]] + [[uid(self.name, "sheet", s.name), s.title] for s in self.sheets],
            "boards": [],
            "text_variables": {},
        }
        p = os.path.join(outdir, self.name + ".kicad_pro")
        if not os.path.exists(p):
            with open(p, "w") as f:
                json.dump(pro, f, indent=2)


def _text(s, x, y, size):
    return [Sym("text"), s, [Sym("exclude_from_sim"), Sym("no")], [Sym("at"), x, y, 0],
            [Sym("effects"), [Sym("font"), [Sym("size"), size, size]], [Sym("justify"), Sym("left"), Sym("top")]],
            [Sym("uuid"), uid("text", s, x, y)]]


def _symbol_instance(lib_id, ref, value, footprint, x, y, rot, project, path, u, pin_uuids, unit=1,
                     extra=None, in_bom=True, on_board=True, dnp=False, field_pos=None):
    field_pos = field_pos or {}

    def prop(k, v, dx, dy, hide=False):
        eff = [Sym("effects"), [Sym("font"), [Sym("size"), 1.27, 1.27]]]
        if hide:
            eff.append([Sym("hide"), Sym("yes")])
        ang = 0
        if k in field_pos:
            dx, dy, ang = field_pos[k]
        return [Sym("property"), k, v, [Sym("at"), x + dx, y + dy, ang], eff]

    is_power = ref.startswith("#")
    node = [Sym("symbol"), [Sym("lib_id"), lib_id], [Sym("at"), x, y, rot], [Sym("unit"), unit],
            [Sym("exclude_from_sim"), Sym("no")], [Sym("in_bom"), Sym("yes" if in_bom else "no")],
            [Sym("on_board"), Sym("yes" if on_board else "no")], [Sym("dnp"), Sym("yes" if dnp else "no")],
            [Sym("uuid"), u],
            prop("Reference", ref, 2.54, -1.27, hide=is_power),
            prop("Value", value, 2.54, 1.27),
            prop("Footprint", footprint, 0, 0, hide=True),
            prop("Datasheet", "~", 0, 0, hide=True)]
    for k, v in (extra or {}).items():
        node.append(prop(k, v, 0, 0, hide=True))
    for num, pu in pin_uuids.items():
        node.append([Sym("pin"), str(num), [Sym("uuid"), pu]])
    node.append([Sym("instances"), [Sym("project"), project, [Sym("path"), path, [Sym("reference"), ref], [Sym("unit"), unit]]]])
    return node
