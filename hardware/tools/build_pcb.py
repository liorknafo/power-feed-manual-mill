"""Build the controller PCB (placement, planes, rules) from the schematic netlist.

Run with KiCad's bundled Python (it provides `pcbnew`):
  /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3 \
      hardware/tools/build_pcb.py [--route]

Steps: export netlist -> create 4-layer 100x100 mm board -> place fixed parts -> auto-place passives
next to the pins they serve -> GND / +36V planes -> (optional) Freerouting -> fill zones -> save.
"""

import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
KDIR = os.path.normpath(os.path.join(HERE, "..", "kicad"))
SCH = os.path.join(KDIR, "power-feed.kicad_sch")
PCB = os.path.join(KDIR, "power-feed.kicad_pcb")
NET = os.path.join(KDIR, "power-feed.net.xml")
KCLI = "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"
FP_DIR = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints"
JAVA = os.environ.get("JAVA", "/opt/homebrew/opt/openjdk@25/bin/java")
FREEROUTING = os.path.expanduser(os.environ.get(
    "FREEROUTING_JAR", "~/Documents/Codex/2026-10-03/i-x20/outputs/home-robot-r1/pcb/tools/freerouting.jar"))

W, H = 100.0, 100.0
EDGE_MARGIN = 0.6


def mm(v):
    return pcbnew.FromMM(v)


def find_net(board, short):
    """Nets local to one sheet are named '/<sheet title>/<NET>'; accept the short name."""
    n = board.FindNet(short)
    if n is not None and n.GetNetCode() > 0:
        return n
    for name, ni in board.GetNetsByName().items():
        if str(name).endswith("/" + short):
            return ni
    raise KeyError(short)


def V(x, y):
    return pcbnew.VECTOR2I(mm(x), mm(y))


# ------------------------------------------------------------------ fixed placement
# ref: (x, y, rotation_deg). Positions are footprint origins (pin 1 for most connectors).
FIXED = {
    # bottom-edge connectors, left to right: power, motor, clutch | feed-unit header, panel ribbon
    "J1": (16.1, 88.8, 0),
    "J2": (30.6, 88.8, 0),
    "J3": (55.0, 88.8, 0),
    "J5": (68.5, 95.0, 90),
    "J4": (77.7, 72.0, 0),
    # input protection column on the left
    "F1": (5.5, 84.0, 90),
    "Q1": (10.0, 50.0, 90),
    "D2": (19.5, 54.0, 90),
    "C1": (13.0, 37.0, 0),
    # 5 V buck, top left
    "U1": (14.0, 15.0, 0),
    "L1": (25.0, 15.0, 0),
    # motor power stage above the motor terminal
    "C27": (16.5, 74.0, 0),
    # FETs rotated so each half-bridge reads: top row G_L, SNS, G_H, MOT (toward the TMC5160),
    # bottom row MOT, MOT, VM, VM (toward the motor terminal) -> short, straight phase outputs
    "U2": (39.25, 56.0, 180),
    "Q10": (29.5, 78.5, 270),
    "Q11": (36.0, 78.5, 270),
    "Q12": (42.5, 78.5, 270),
    "Q13": (49.0, 78.5, 270),
    "R18": (32.75, 70.0, 0),
    "R21": (45.75, 70.0, 0),
    # clutch driver, right-middle
    "F2": (70.0, 34.0, 0),
    "Q2": (74.0, 42.0, 0),
    "D6": (85.0, 41.0, 90),
    "Q4": (79.0, 51.0, 0),
    "R42": (85.0, 51.0, 0),
    "D7": (78.0, 57.5, 0),
    # Pico across the top, USB toward the right edge
    "A1": (64.0, 15.0, 270),
    "SW1": (31.0, 31.0, 0),
    # mounting holes
    "H1": (4.0, 4.0, 0),
    "H2": (96.0, 4.0, 0),
    "H3": (4.0, 96.0, 0),
    "H4": (96.0, 96.0, 0),
}

# Parts placed next to a specific pad: ref -> (anchor ref, anchor pad). Anchors may themselves be
# NEAR-placed parts; they are resolved in dependency order.
NEAR = {
    # TMC5160 support
    "C11": ("U2", "33"), "C12": ("U2", "4"), "C13": ("U2", "32"), "C14": ("U2", "34"),
    "C15": ("U2", "5"), "R9": ("U2", "29"), "C16": ("U2", "29"), "C17": ("U2", "3"), "C18": ("U2", "20"),
    "C19": ("U2", "42"), "C21": ("U2", "35"), "C23": ("U2", "2"), "C25": ("U2", "43"),
    "R10": ("U2", "41"), "R11": ("U2", "39"), "R12": ("U2", "36"), "R13": ("U2", "38"),
    "R14": ("U2", "1"), "R15": ("U2", "47"), "R16": ("U2", "44"), "R17": ("U2", "46"),
    "R19": ("U2", "8"), "R20": ("U2", "7"), "R22": ("U2", "9"), "R23": ("U2", "10"),
    "C20": ("Q10", "7"), "C22": ("Q11", "7"), "C24": ("Q12", "7"), "C26": ("Q13", "7"),
    # buck + input
    "C2": ("U1", "2"), "C3": ("U1", "2"), "C4": ("Q1", "3"), "R2": ("U1", "4"), "C5": ("U1", "7"),
    "R3": ("U1", "5"), "R4": ("U1", "5"), "R5": ("U1", "8"), "C6": ("U1", "8"), "C7": ("U1", "5"),
    "C8": ("L1", "2"), "C9": ("L1", "2"), "R6": ("L1", "2"), "D3": ("R6", "2"),
    "R1": ("Q1", "1"), "D1": ("Q1", "1"), "R7": ("C1", "1"), "R8": ("R7", "2"), "C10": ("R7", "2"),
    # Pico, SPI, interlock
    "D4": ("A1", "39"), "C28": ("A1", "36"), "R37": ("A1", "5"),
    "R24": ("U2", "14"), "R25": ("U2", "15"), "R26": ("U2", "13"), "R27": ("U2", "16"), "R28": ("U2", "13"),
    "R29": ("U2", "26"), "R30": ("U2", "27"),
    "U3": ("U2", "28"), "C29": ("U4", "5"), "C30": ("U5", "5"), "R31": ("U3", "1"),
    "U4": ("Q4", "1"), "U5": ("Q3", "1"), "R32": ("U4", "1"), "R33": ("U5", "1"),
    "R36": ("U3", "2"), "C31": ("U3", "2"),
    # clutch
    "C32": ("F2", "2"), "Q3": ("Q2", "1"), "R38": ("Q2", "1"), "D5": ("Q2", "1"), "R39": ("Q3", "3"),
    "R40": ("Q4", "1"), "R41": ("Q4", "1"), "R43": ("D7", "1"), "D8": ("R43", "2"),
    "U6": ("R42", "1"), "R44": ("U6", "3"), "C33": ("U6", "3"), "C34": ("U6", "5"),
    "R45": ("U6", "4"), "R46": ("U6", "4"), "R47": ("U6", "1"), "C35": ("R47", "2"),
    # panel / feed-unit inputs: ESD + pull-up + series R at the connector, filter cap on the clean side
    "D9": ("J4", "9"), "D10": ("J4", "12"), "D11": ("J5", "4"), "D12": ("J4", "3"),
    "R48": ("J4", "8"), "R50": ("J4", "9"), "R52": ("J4", "10"),
    "R58": ("J4", "11"), "R60": ("J4", "12"), "R62": ("J4", "13"),
    "R54": ("J5", "6"), "R56": ("J5", "7"), "R64": ("J5", "3"), "R66": ("J5", "4"), "R68": ("J5", "5"),
    "R70": ("J4", "3"), "R71": ("J4", "4"), "R72": ("J4", "3"), "R73": ("J4", "4"),
    "R34": ("J4", "6"), "R35": ("J4", "7"),
    "R49": ("R48", "2"), "R51": ("R50", "2"), "R53": ("R52", "2"), "R55": ("R54", "2"), "R57": ("R56", "2"),
    "R59": ("R58", "2"), "R61": ("R60", "2"), "R63": ("R62", "2"), "R65": ("R64", "2"), "R67": ("R66", "2"),
    "R69": ("R68", "2"),
    "C36": ("R49", "2"), "C37": ("R51", "2"), "C38": ("R53", "2"), "C39": ("R55", "2"), "C40": ("R57", "2"),
    "C41": ("R59", "2"), "C42": ("R61", "2"), "C43": ("R63", "2"), "C44": ("R65", "2"), "C45": ("R67", "2"),
    "C46": ("R69", "2"),
}

# Router widths are limited by the smallest pad a net touches (TQFP-48 pins are 0.3 mm), so the
# high-current paths get extra copper from post-route pours (add_power_pours) instead.
POWER_NETS = ["VIN_RAW", "VIN_F", "SNS_A", "SNS_B", "CLUTCH_VIN", "COIL_P", "COIL_N", "CLUTCH_SHUNT"]
HV_NETS = ["+36V", "MOT_A1", "MOT_A2", "MOT_B1", "MOT_B2"]
SUPPLY_NETS = ["+5V", "+3V3", "VSYS", "BUCK_SW", "TMC_5V", "TMC_12V", "TMC_VCC"]
GATE_NETS_PREFIX = ("G_H", "G_L", "TMC_H", "TMC_L", "TMC_C")
GLOBAL_NETS = {"GND", "+3V3", "+5V", "+36V"}


# ------------------------------------------------------------------ netlist
def export_netlist():
    subprocess.run([KCLI, "sch", "export", "netlist", "--format", "kicadxml", "-o", NET, SCH],
                   check=True, capture_output=True)
    root = ET.parse(NET).getroot()
    comps = {}
    for c in root.find("components"):
        ref = c.get("ref")
        comps[ref] = {
            "value": c.findtext("value"),
            "footprint": c.findtext("footprint"),
            "path": c.find("sheetpath").get("tstamps") + c.findtext("tstamps"),
        }
    pin_net = {}
    for net in root.iter("net"):
        name = net.get("name")
        for n in net.iter("node"):
            pin_net[(n.get("ref"), n.get("pin"))] = name
    return comps, pin_net


def load_fp(fpid):
    lib, name = fpid.split(":")
    path = os.path.join(KDIR, lib + ".pretty") if lib == "power_feed" else os.path.join(FP_DIR, lib + ".pretty")
    fp = pcbnew.FootprintLoad(path, name)
    if fp is None:
        raise RuntimeError("footprint not found: " + fpid)
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    return fp


# ------------------------------------------------------------------ geometry helpers
# extra keep-clear ring (mm) around parts whose pins need escape room
KEEP_CLEAR = {"U2": 1.6, "A1": 0.6, "U1": 0.5}


def courtyard_box(fp, pad=0.25):
    pad += KEEP_CLEAR.get(fp.GetReference(), 0.0)
    cy = fp.GetCourtyard(pcbnew.F_CrtYd)
    if cy.OutlineCount():
        bb = cy.BBox()
        x0, y0, x1, y1 = bb.GetLeft(), bb.GetTop(), bb.GetRight(), bb.GetBottom()
    else:
        bb = fp.GetBoundingBox(False)
        x0, y0, x1, y1 = bb.GetLeft(), bb.GetTop(), bb.GetRight(), bb.GetBottom()
    p = mm(pad)
    return (x0 - p, y0 - p, x1 + p, y1 + p)


def overlaps(a, b):
    return not (a[2] <= b[0] or b[2] <= a[0] or a[3] <= b[1] or b[3] <= a[1])


def inside_board(box):
    m = mm(EDGE_MARGIN)
    return box[0] >= m and box[1] >= m and box[2] <= mm(W) - m and box[3] <= mm(H) - m


def pad_pos(fp, num):
    for p in fp.Pads():
        if p.GetNumber() == num:
            return p.GetPosition()
    raise KeyError("%s has no pad %s" % (fp.GetReference(), num))


# ------------------------------------------------------------------ board
def setup_rules(board):
    ds = board.GetDesignSettings()
    ds.SetCopperLayerCount(4)
    board.SetCopperLayerCount(4)
    ds.m_TrackMinWidth = mm(0.15)
    ds.m_MinClearance = mm(0.15)
    ds.m_ViasMinSize = mm(0.5)
    ds.m_MinThroughDrill = mm(0.2)  # TQFP-48 thermal vias; JLCPCB 4-layer minimum is 0.15 mm
    ds.m_HoleClearance = mm(0.25)
    ds.m_CopperEdgeClearance = mm(0.4)
    ns = ds.m_NetSettings
    default = ns.GetDefaultNetclass()
    default.SetTrackWidth(mm(0.2))
    default.SetClearance(mm(0.15))
    default.SetViaDiameter(mm(0.6))
    default.SetViaDrill(mm(0.3))

    def klass(name, width, clearance, via_d, via_drill, patterns):
        nc = pcbnew.NETCLASS(name)
        nc.SetTrackWidth(mm(width))
        nc.SetClearance(mm(clearance))
        nc.SetViaDiameter(mm(via_d))
        nc.SetViaDrill(mm(via_drill))
        ns.SetNetclass(name, nc)
        for pat in patterns:
            ns.SetNetclassPatternAssignment(pat, name)
            if not pat.startswith("*"):
                ns.SetNetclassPatternAssignment("*/" + pat, name)

    klass("Power", 0.8, 0.2, 0.9, 0.45, POWER_NETS)
    # 0.15 mm clearance: TQFP-48 pads are 0.3 mm wide on a 0.5 mm pitch (0.2 mm gaps)
    klass("HV", 0.3, 0.15, 0.8, 0.4, HV_NETS)
    klass("Supply", 0.3, 0.15, 0.7, 0.35, SUPPLY_NETS)
    klass("Gate", 0.3, 0.15, 0.6, 0.3, [p + "*" for p in GATE_NETS_PREFIX])
    ns.RecomputeEffectiveNetclasses()


def add_outline(board):
    s = pcbnew.PCB_SHAPE(board)
    s.SetShape(pcbnew.SHAPE_T_RECT)
    s.SetStart(V(0, 0))
    s.SetEnd(V(W, H))
    s.SetLayer(pcbnew.Edge_Cuts)
    s.SetWidth(mm(0.1))
    board.Add(s)


def add_zone(board, net, layer, poly, priority=0, clearance=0.3):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(find_net(board, net))
    z.SetAssignedPriority(priority)
    z.SetLocalClearance(mm(clearance))
    z.SetMinThickness(mm(0.25))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
    z.SetThermalReliefGap(mm(0.3))
    z.SetThermalReliefSpokeWidth(mm(0.5))
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    o = z.Outline()
    o.NewOutline()
    for x, y in poly:
        o.Append(mm(x), mm(y))
    board.Add(z)
    return z


def build(route=False):
    comps, pin_net = export_netlist()
    if os.path.exists(PCB):
        os.remove(PCB)
    board = pcbnew.NewBoard(PCB)
    setup_rules(board)

    nets = {}
    for name in sorted({n.replace("/", "{slash}") if n.startswith("unconnected-") else n for n in pin_net.values()}):
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        nets[name] = ni

    fps = {}
    for ref, c in comps.items():
        fp = load_fp(c["footprint"])
        fp.SetReference(ref)
        fp.SetValue(c["value"])
        if ref in ("U1", "U2"):  # exposed pads + thermal-via arrays: solid to every GND pour
            fp.SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
        fp.SetPath(pcbnew.KIID_PATH(c["path"]))
        for pad in fp.Pads():
            n = pin_net.get((ref, pad.GetNumber()))
            if n:
                if n.startswith("unconnected-"):
                    n = n.replace("/", "{slash}")
                pad.SetNet(nets[n])
        board.Add(fp)
        fps[ref] = fp
    for i, ref in enumerate(["H1", "H2", "H3", "H4"]):
        fp = load_fp("MountingHole:MountingHole_3.2mm_M3")
        fp.SetReference(ref)
        fp.SetValue("M3")
        fp.SetBoardOnly(True)
        fp.SetExcludedFromBOM(True)
        board.Add(fp)
        fps[ref] = fp

    placed = []
    for ref, (x, y, rot) in FIXED.items():
        fp = fps[ref]
        fp.SetOrientationDegrees(rot)
        fp.SetPosition(V(x, y))
        box = courtyard_box(fp)
        for other_ref, ob in placed:
            if overlaps(box, ob):
                print("WARNING: fixed parts overlap: %s / %s" % (ref, other_ref))
        if not inside_board(box) and not ref.startswith(("H", "J")):
            print("WARNING: fixed part off board: %s" % ref)
        placed.append((ref, box))

    # anchors for parts without a NEAR entry: average of pads on placed parts sharing a local net
    def target_for(ref):
        if ref in NEAR:
            aref, apad = NEAR[ref]
            p = pad_pos(fps[aref], apad)
            c = fps[aref].GetPosition()
            # push a little away from the anchor body, along the pad direction
            dx, dy = p.x - c.x, p.y - c.y
            d = math.hypot(dx, dy) or 1
            return p.x + dx / d * mm(2.5), p.y + dy / d * mm(2.5)
        pts = []
        fp = fps[ref]
        my_nets = {p.GetNetname() for p in fp.Pads()} - GLOBAL_NETS - {""}
        for oref, _ in placed:
            for p in fps[oref].Pads():
                if p.GetNetname() in my_nets:
                    pts.append(p.GetPosition())
        if not pts:
            return None
        return sum(p.x for p in pts) / len(pts), sum(p.y for p in pts) / len(pts)

    todo = [r for r in fps if r not in FIXED]
    # parts with explicit anchors first, then by how connected they are to placed parts
    pending = list(todo)
    for _ in range(12):
        nxt = []
        placed_refs = {r for r, _ in placed}
        for ref in sorted(pending, key=lambda r: (r not in NEAR, r)):
            if ref in NEAR and NEAR[ref][0] not in placed_refs:
                nxt.append(ref)
                continue
            t = target_for(ref)
            if t is None:
                nxt.append(ref)
                continue
            fp = fps[ref]
            best = None
            for radius in [i * 0.5 for i in range(0, 60)]:
                steps = max(1, int(2 * math.pi * radius / 0.5))
                for k in range(steps):
                    a = 2 * math.pi * k / steps
                    x, y = t[0] + mm(radius) * math.cos(a), t[1] + mm(radius) * math.sin(a)
                    for rot in (0, 90):
                        fp.SetOrientationDegrees(rot)
                        fp.SetPosition(pcbnew.VECTOR2I(int(round(x / mm(0.25))) * mm(0.25),
                                                       int(round(y / mm(0.25))) * mm(0.25)))
                        box = courtyard_box(fp)
                        if inside_board(box) and not any(overlaps(box, b) for _, b in placed):
                            best = (rot, fp.GetPosition(), box)
                            break
                    if best:
                        break
                if best:
                    break
            if not best:
                print("WARNING: could not place", ref)
                nxt.append(ref)
                continue
            fp.SetOrientationDegrees(best[0])
            fp.SetPosition(best[1])
            placed.append((ref, best[2]))
            placed_refs.add(ref)
        pending = nxt
        if not pending:
            break
    if pending:
        print("UNPLACED:", pending)

    add_outline(board)
    for h in ("H1", "H2", "H3", "H4"):
        c = fps[h].GetPosition()
        ka = pcbnew.ZONE(board)
        ka.SetIsRuleArea(True)
        ka.SetDoNotAllowCopperPour(True)
        ka.SetDoNotAllowTracks(True)
        ka.SetDoNotAllowVias(True)
        ka.SetDoNotAllowPads(False)
        ka.SetDoNotAllowFootprints(False)
        ka.SetLayerSet(pcbnew.LSET.AllCuMask())
        o = ka.Outline()
        o.NewOutline()
        for k in range(24):
            a = 2 * math.pi * k / 24
            o.Append(c.x + int(mm(3.6) * math.cos(a)), c.y + int(mm(3.6) * math.sin(a)))
        board.Add(ka)
    # planes: In1 = GND everywhere; In2 = +36V under the power section, +3V3 elsewhere
    full = [(0.5, 0.5), (W - 0.5, 0.5), (W - 0.5, H - 0.5), (0.5, H - 0.5)]
    add_zone(board, "GND", pcbnew.In1_Cu, full)
    # +36V on In2, minus a corridor above the TMC5160 (x 33..46, y 28..48.5) that carries +3V3 to VCC_IO
    add_zone(board, "+36V", pcbnew.In2_Cu,
             [(0.5, 6.5), (21.0, 6.5), (21.0, 28.5), (32.5, 28.5), (32.5, 49.0), (46.5, 49.0), (46.5, 28.5),
              (92.5, 28.5), (92.5, 61), (64.5, 61), (64.5, 86), (0.5, 86)], priority=1)
    # +3V3 takes the rest of In2 as ONE connected region (top strip, right-edge strip, bottom-right);
    # outlines must not overlap +36V or the router may via into the wrong plane
    add_zone(board, "+3V3", pcbnew.In2_Cu,
             [(0.5, 0.5), (99.5, 0.5), (99.5, 99.5), (0.5, 99.5), (0.5, 86.5), (65.0, 86.5), (65.0, 61.5),
              (93.0, 61.5), (93.0, 28.0), (47.0, 28.0), (47.0, 48.5), (32.0, 48.5), (32.0, 28.0), (21.5, 28.0),
              (21.5, 6.0), (0.5, 6.0)])
    board.SetLayerType(pcbnew.In1_Cu, pcbnew.LT_POWER)
    board.SetLayerType(pcbnew.In2_Cu, pcbnew.LT_POWER)

    add_bridge_vias(board)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(PCB)
    print("saved", PCB)

    if route:
        autoroute(board)
    return board


def add_via(board, net, x, y, d=0.8, drill=0.4):
    v = pcbnew.PCB_VIA(board)
    v.SetPosition(pcbnew.VECTOR2I(int(x), int(y)))
    v.SetWidth(mm(d))
    v.SetDrill(mm(drill))
    v.SetNet(find_net(board, net))
    board.Add(v)
    return v


def add_track(board, net, a, b, width):
    t = pcbnew.PCB_TRACK(board)
    t.SetStart(a)
    t.SetEnd(b)
    t.SetWidth(mm(width))
    t.SetLayer(pcbnew.F_Cu)
    t.SetNet(find_net(board, net))
    board.Add(t)


HALF_BRIDGES = {"Q10": "MOT_A1", "Q11": "MOT_A2", "Q12": "MOT_B1", "Q13": "MOT_B2"}


def add_bridge_vias(board):
    """Two +36V vias straight below each FET's VM pads (pins 7, 8) into the In2 +36V plane."""
    for q in HALF_BRIDGES:
        fp = board.FindFootprintByReference(q)
        for num in ("7", "8"):
            p = fp.FindPadByNumber(num).GetPosition()
            via_pos = pcbnew.VECTOR2I(p.x, p.y + mm(1.75))
            add_via(board, "+36V", via_pos.x, via_pos.y)
            add_track(board, "+36V", p, via_pos, 0.6)


def poly_zone(board, net, layer, pts_iu, priority, clearance=0.25):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(find_net(board, net))
    z.SetAssignedPriority(priority)
    z.SetLocalClearance(mm(clearance))
    z.SetMinThickness(mm(0.3))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    o = z.Outline()
    o.NewOutline()
    for x, y in pts_iu:
        o.Append(int(x), int(y))
    board.Add(z)
    return z


def add_power_pours(board):
    """Solid copper for the high-current paths, added after routing (zones only keep copper that
    touches their own net, so they can only thicken existing connections)."""
    def bbox(ref):
        b = board.FindFootprintByReference(ref).GetCourtyard(pcbnew.F_CrtYd).BBox()
        return b.GetLeft(), b.GetTop(), b.GetRight(), b.GetBottom()

    def pad(ref, num):
        return board.FindFootprintByReference(ref).FindPadByNumber(num)

    for q, mot in HALF_BRIDGES.items():
        x0, y0, x1, y1 = bbox(q)
        jpad = [p for p in board.FindFootprintByReference("J2").Pads() if p.GetNetname().endswith("/" + mot)][0]
        jp, js = jpad.GetPosition(), jpad.GetSize()
        m5 = pad(q, "5").GetPosition()
        m6 = pad(q, "6").GetPosition()
        # FET body + drop from the two bottom MOT pads to the motor terminal pad
        poly_zone(board, mot, pcbnew.F_Cu, [(x0, y0), (x1, y0), (x1, y1), (m6.x + mm(0.6), y1),
                                            (jp.x + js.x // 2, jp.y), (jp.x - js.x // 2, jp.y),
                                            (m5.x - mm(0.9), y1), (x0, y1)], priority=5)
        # VM pads + their vias
        v7, v8 = pad(q, "7").GetPosition(), pad(q, "8").GetPosition()
        poly_zone(board, "+36V", pcbnew.F_Cu, [(v7.x - mm(0.45), v7.y - mm(0.9)), (v8.x + mm(0.45), v8.y - mm(0.9)),
                                               (v8.x + mm(0.6), v8.y + mm(2.35)), (v7.x - mm(0.6), v7.y + mm(2.35))],
                  priority=7)
    # sense nodes: from the FETs' SNS pads up to the sense resistor
    for net, (qa, qb, r) in {"SNS_A": ("Q10", "Q11", "R18"), "SNS_B": ("Q12", "Q13", "R21")}.items():
        ra = bbox(r)
        pa, pb = pad(qa, "3").GetPosition(), pad(qb, "3").GetPosition()
        top = ra[1]
        poly_zone(board, net, pcbnew.F_Cu, [(pa.x - mm(0.4), pa.y), (pa.x - mm(0.4), top),
                                            (pb.x + mm(0.4), top), (pb.x + mm(0.4), pb.y)], priority=6)


def stitch_ground(board, pitch=2.0, via_d=0.5, drill=0.25, keep=0.35):
    """GND vias wherever both outer GND fills (and the In1 plane) have room for one."""
    zones = {z.GetLayer(): z for z in board.Zones()
             if z.GetNetname() == "GND" and not z.GetIsRuleArea() and z.GetLayer() in (pcbnew.F_Cu, pcbnew.B_Cu, pcbnew.In1_Cu)}
    r = via_d / 2 + keep
    ring_pts = [(0, 0)] + [(r * math.cos(a * math.pi / 4), r * math.sin(a * math.pi / 4)) for a in range(8)]
    holes = [(t.GetPosition().x, t.GetPosition().y, t.GetDrillValue() / 2) for t in board.GetTracks()
             if t.Type() == pcbnew.PCB_VIA_T]
    keepout = []
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetDrillSize().x > 0:
                holes.append((p.GetPosition().x, p.GetPosition().y, p.GetDrillSize().x / 2))
        if fp.GetReference().startswith("H"):
            continue
        keepout.append(courtyard_box(fp, pad=0.0))
    n = 0
    y = 2.0
    while y < H - 2.0:
        x = 2.0 + (pitch / 2 if int(y / pitch) % 2 else 0)
        while x < W - 2.0:
            ok = True
            for layer, z in zones.items():
                for dx, dy in ring_pts:
                    if not z.HitTestFilledArea(layer, V(x + dx, y + dy), 0):
                        ok = False
                        break
                if not ok:
                    break
            px, py = mm(x), mm(y)
            if ok and any(b[0] <= px <= b[2] and b[1] <= py <= b[3] for b in keepout):
                ok = False
            if ok and any(math.hypot(px - hx, py - hy) < hr + mm(drill / 2 + 0.4) for hx, hy, hr in holes):
                ok = False
            if ok:
                add_via(board, "GND", px, py, via_d, drill)
                holes.append((px, py, mm(drill / 2)))
                n += 1
            x += pitch
        y += pitch
    return n


def add_outer_ground(board):
    """GND pours on both outer layers, added after routing so the router sees free signal layers."""
    full = [(0.5, 0.5), (W - 0.5, 0.5), (W - 0.5, H - 0.5), (0.5, H - 0.5)]
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        z = add_zone(board, "GND", layer, full, priority=0, clearance=0.3)
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THT_THERMAL)  # solid to SMD pads, relieved THT pads


def autoroute(board, pours=True):
    dsn = os.path.join(KDIR, "power-feed.dsn")
    ses = os.path.join(KDIR, "power-feed.ses")
    if not pcbnew.ExportSpecctraDSN(board, dsn):
        raise RuntimeError("DSN export failed")
    print("routing with Freerouting ...")
    # Freerouting sometimes exits before its save thread has written the session; retry.
    for attempt in range(4):
        if os.path.exists(ses):
            os.remove(ses)
        subprocess.run([JAVA, "-jar", FREEROUTING, "-de", dsn, "-do", ses, "-mp", "60", "-mt", "1"], check=True)
        if os.path.exists(ses) and os.path.getsize(ses) > 0:
            break
        print("empty session file, retrying (%d)" % (attempt + 1))
    else:
        raise RuntimeError("Freerouting produced no session file")
    if not pcbnew.ImportSpecctraSES(board, ses):
        raise RuntimeError("SES import failed")
    if pours:
        add_power_pours(board)
        add_outer_ground(board)
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        print("ground stitching vias:", stitch_ground(board))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(PCB)
    print("routed and saved", PCB)


def reroute():
    """Second Freerouting pass on the saved board: keeps existing copper, routes what is left."""
    board = pcbnew.LoadBoard(PCB)
    autoroute(board, pours=False)


if __name__ == "__main__":
    if "--reroute" in sys.argv:
        reroute()
    else:
        build(route="--route" in sys.argv)
