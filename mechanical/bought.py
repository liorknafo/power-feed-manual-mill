"""Bought parts, modelled from their datasheets (sources in each docstring), each in its own frame. The feed unit places
them; the checks and the viewer use these shapes instead of stand-in blocks.

Frames: a panel part has the panel's OUTER face at z = 0, its front (what you see and press) toward +z and its body
behind the panel toward -z.
"""

import math

from build123d import Align, Box, Compound, Cylinder, Polyline, Pos, Rot, chamfer, extrude, make_face, Axis

MIN = (Align.CENTER, Align.CENTER, Align.MIN)


def cyl(r, z0, z1, x=0.0, y=0.0):
    return Pos(x, y, z0) * Cylinder(r, z1 - z0, align=MIN)


def box(x0, x1, y0, y1, z0, z1):
    return Pos(x0, y0, z0) * Box(x1 - x0, y1 - y0, z1 - z0, align=(Align.MIN, Align.MIN, Align.MIN))


# ------------------------------------------------------------------ motor
def nema23_23hs45():
    """STEPPERONLINE 23HS45-4204S (drawing: oyostepper.com/images/upload/File/23HS45-4204S.pdf). Flange face at z = 0,
    shaft toward +z: flange 57 square x 4.8 with 4x Ø5.1 through on a 47.0 square, pilot Ø38.1 x 1.6, shaft Ø10 x 24
    from the flange face with a 0.5 deep flat 20 long from the tip (cut to 21 for the clutch: see MACHINING.md), body
    to 113 behind the face, rear cap, leads leaving sideways from the rear cap. The body's corners behind the flange are relieved (the drawing
    shows the step, not its size: the 17 mm leg here assumes room for the M5 nuts; check yours)."""
    flange = box(-28.5, 28.5, -28.5, 28.5, -4.8, 0.0)
    for k in range(4):
        a = math.radians(45 + 90 * k)
        r = 47.0 / math.sqrt(2)
        flange = flange - cyl(2.55, -5.0, 0.1, r * math.cos(a), r * math.sin(a))
    body = chamfer(box(-28.5, 28.5, -28.5, 28.5, -113.0, -4.8).edges().filter_by(Axis.Z), 17.0)
    pilot = cyl(19.05, 0.0, 1.6)
    lead = Pos(28.5, 0, -109.0) * Rot(0, 90, 0) * Cylinder(3.5, 5.0, align=MIN)     # leads out sideways at the rear cap
    # (on the +x flat: in the unit that faces sideways, clear of the PCB above and the mains tray below; bolt the
    # motor on with its leads that way)
    return flange + body + pilot + lead


def nema23_shaft(length=21.0):
    """The motor shaft, Ø10, cut to `length` from the flange face, with its 0.5 deep flat."""
    s = cyl(5.0, 0.0, length)
    return s - Pos(5.0 - 0.5 + 5.0, 0, max(length - 20.0, 2.0)) * Box(10, 20, 30, align=MIN)


# ------------------------------------------------------------------ pulleys, belt
def htd5m_pulley(teeth, bore, set_d):
    """HTD 5M pulley for a 15 mm belt (HPC 20HTD5M-15F / 40HTD5M-15F: shop.hpceurope.com/pdf/frPDFauto/HTD5M_15.pdf),
    HUB TURNED OFF to fit the 24 mm belt chamber: flange, 20.5 mm of teeth, flange (23.5 overall), centred on z = 0.
    OD = PD - 1.14; flanges Ø36 (20T) / Ø71 (40T) x 1.5. Teeth modelled as grooves, 2 set screws (90 deg apart)
    drilled and tapped radially through a tooth root at mid-width."""
    pd = teeth * 5.0 / math.pi
    od = pd - 1.14
    fl = 36.0 if teeth <= 20 else 71.0
    w = 20.5
    body = cyl(od / 2, -w / 2, w / 2)
    groove = Pos(od / 2, 0, -w / 2 - 1) * Box(4.12, 3.0, w + 2, align=MIN)          # HTD groove 2.06 deep, ~3 wide
    for k in range(teeth):
        body = body - Rot(0, 0, 360.0 / teeth * (k + 0.5)) * groove
    body = body + cyl(fl / 2, -w / 2 - 1.5, -w / 2) + cyl(fl / 2, w / 2, w / 2 + 1.5)
    body = body - cyl(bore / 2, -20, 20)
    for a in pulley_set_screw_angles(teeth):               # set-screw holes through a tooth root, at mid-width
        body = body - Rot(0, 0, a) * Rot(0, 90, 0) * Cylinder(set_d * 0.4, od, align=MIN)
    return body


def pulley_set_screw_angles(teeth):
    """Two set screws 90 degrees apart, each in a tooth root (groove centre)."""
    return (180.0 / teeth, 180.0 / teeth + 90.0)


def htd5m_belt_teeth(c1, n1, c2, n2, w, z0):
    """The teeth of an HTD-5M belt (Gates HTD 5M profile: 5 pitch, 3.8 overall, 2.06 tooth height, pitch line 0.57
    outside the land) running over an n1-tooth pulley at c1 = (x, y) and an n2-tooth one at c2, both unrotated
    htd5m_pulley()s (grooves centred at 360/n * (k + 1/2)). The backing band (land to back, 1.74) is the caller's.
    On the wraps a tooth (2.0 deep x 2.8 wide, a little under the groove) sits in every groove; on the straight
    spans they follow at 5 mm pitch, in step with the last tooth of the big pulley's wrap, and are left out where the
    span is still within a tooth height of a pulley, so the model shows the teeth without claiming the meshing in the
    entry zone."""
    r1, r2 = n1 * 5.0 / math.pi / 2, n2 * 5.0 / math.pi / 2
    dx, dy = c2[0] - c1[0], c2[1] - c1[1]
    c = math.hypot(dx, dy)
    a12 = math.atan2(dy, dx)
    beta = math.asin((r1 - r2) / c)
    teeth = []

    def tooth(x, y, ang):                  # ang: outward normal of the belt's inner face at (x, y) on the pitch line
        r_land = -0.57
        return Pos(x, y, z0) * Rot(0, 0, math.degrees(ang)) * Pos(r_land - 2.0, -1.4, 0) * Box(
            2.1, 2.8, w, align=(Align.MIN, Align.MIN, Align.MIN))
    # wraps: the big pulley wraps pi + 2 beta on the far side from c2, the small pi - 2 beta on the far side from c1
    for (cx, cy), r, n, mid, half in ((c1, r1, n1, a12 + math.pi, math.pi / 2 + beta),
                                      (c2, r2, n2, a12, math.pi / 2 - beta)):
        for k in range(n):
            g = math.radians(360.0 / n * (k + 0.5))
            if abs((g - mid + math.pi) % (2 * math.pi) - math.pi) < half - 0.5 * 2 * math.pi / n:
                teeth.append(tooth(cx + r * math.cos(g), cy + r * math.sin(g), g))
    # spans: the outer tangents touch both pulleys where the normal is at a12 +/- (pi/2 - beta)
    for sgn in (1, -1):
        nrm = a12 + sgn * (math.pi / 2 - beta)
        p1 = (c1[0] + r1 * math.cos(nrm), c1[1] + r1 * math.sin(nrm))
        p2 = (c2[0] + r2 * math.cos(nrm), c2[1] + r2 * math.sin(nrm))
        L = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
        skip1 = math.sqrt((r1 + 2.06) ** 2 - r1 ** 2) + 1.0
        skip2 = math.sqrt((r2 + 2.06) ** 2 - r2 ** 2) + 1.0
        # keep pitch with the wrap: the last groove inside pulley 1's wrap is s0 of pitch line back from the tangent
        # point (the wrap runs from the tangent point toward a12 + pi, i.e. in the +sgn direction of angle)
        step = 2 * math.pi / n1
        delta = min(((math.radians(360.0 / n1 * (j + 0.5)) - nrm) * sgn) % (2 * math.pi) for j in range(n1))
        s0 = r1 * (delta % step)
        k = 1
        while True:
            d = 5.0 * k - s0
            if d > L - skip2:
                break
            if d < skip1:
                k += 1
                continue
            t = d / L
            teeth.append(tooth(p1[0] + t * (p2[0] - p1[0]), p1[1] + t * (p2[1] - p1[1]), nrm))
            k += 1
    return teeth


# ------------------------------------------------------------------ encoder
def lpd3806():
    """LPD3806-600BM-G5-24C (rcscomponents.kiev.ua/datasheets/lpd-ab-english2020.pdf): Ø38 x 35 body, Ø20 x 5 face boss,
    Ø6 shaft 13 beyond the boss with a 0.6 flat 10 long, 3x M3 on Ø30 and 3x M3 on Ø28 in the face, radial cable exit.
    Face (with the boss and shaft) toward -z; the body's front face at z = 0."""
    body = cyl(19.0, 0.0, 35.0)
    for k in range(3):
        for rr, off in ((15.0, 0.0), (14.0, 30.0)):
            a = math.radians(90 + 120 * k + 60 + off)
            body = body - cyl(1.25, -0.1, 6.0, rr * math.cos(a), rr * math.sin(a))
    boss = cyl(10.0, -5.0, 0.0)
    shaft = cyl(3.0, -18.0, -5.0) - Pos(3.0 - 0.6 + 5, 0, -18.0) * Box(10, 10, 10, align=MIN)
    cable = Pos(0, 19.0, 28.5) * Rot(-90, 0, 0) * Cylinder(3.0, 8.0, align=MIN)
    return body + boss + shaft + cable


# ------------------------------------------------------------------ power supply
def lrs75():
    """Mean Well LRS-75-36 (meanwell.com/Upload/PDF/LRS-75/LRS-75-SPEC.PDF): 99 x 97 x 30, case 240A. Frame: base at
    z = 0, the 99 length along +y from the terminal end at y = 0, the 97 width along x from x = 0. The 5-pole terminal
    block (1 L, 2 N, 3 FG, 4 -V, 5 +V, pitch 9.5, M3.5 screws facing up) sits in a recess at the terminal end, first
    pole 10 from the x = 0 edge, screw heads ~3.5 below the top (INFERRED from the drawing). Side vent slots."""
    case = box(0, 97.0, 0, 99.0, 0, 30.0)
    case = case - box(10.0 - 4.75 - 1.0, 10.0 + 4 * 9.5 + 4.75 + 1.0, -0.1, 17.0, 26.0, 30.1)   # terminal recess
    for k, head in enumerate(lrs75_terminal_heads()):        # terminal screw heads (Ø6, 2.5 high) on their clamps
        x = 10.0 + 9.5 * k
        case = case + head
        case = case - cyl(1.2, 25.6, 26.6, x, 8.5) - box(x - 0.4, x + 0.4, 6.5, 10.5, 26.0, 26.6)   # cross slot
    for yy in (54.0, 59.0, 65.0):                            # side vent slots on the 99 x 30 face
        case = case - box(-0.1, 1.0, yy - 1.65, yy + 1.65, 8.3, 24.4)
    return case


def lrs75_terminal_heads():
    """The 5 terminal screw heads of the LRS-75 (its frame), for clearance checks."""
    return [cyl(3.0, 24.0, 26.5, 10.0 + 9.5 * k, 8.5) for k in range(5)]


# ------------------------------------------------------------------ panel controls (panel outer face at z = 0)
def hexagon(af, z0, z1, x=0.0, y=0.0):
    """Hex nut outline, `af` across flats."""
    r = af / math.sqrt(3)
    pts = [(x + r * math.cos(math.radians(60 * k)), y + r * math.sin(math.radians(60 * k))) for k in range(6)]
    return extrude(Pos(0, 0, z0) * make_face(Polyline(*pts, close=True)), amount=z1 - z0)


def estop_idec_xa(panel_t):
    """IDEC XA1E-BV302R E-stop, Ø29 mushroom, twist/pull to reset, 2 NC, locks from behind (IDEC X-series E-stop catalog,
    files.valinonline.com/userfiles/documents/idec-e-stop-catalog.pdf; DigiKey XA1E-BV302-R): Ø16.2 hole, 20.6 above the
    panel, 27.9 behind it. M16 threaded body (Ø15.8) through the panel, ~Ø24 lock ring + ~19 x 22 contact block behind
    it (INFERRED); the bezel sits on the yellow legend plate (estop_legend_plate), 1 above the panel."""
    body = cyl(7.9, -panel_t, 1.0)
    bezel = cyl(11.0, 1.0, 6.0)
    stem = cyl(7.0, 6.0, 12.0)
    head = cyl(14.5, 12.0, 18.6) + cyl(12.0, 18.6, 20.6)
    lock = cyl(12.0, -panel_t - 4.0, -panel_t)
    block = box(-9.5, 9.5, -11.0, 11.0, -27.9, -panel_t - 4.0)
    return body + bezel + stem + head + lock + block


def estop_legend_plate():
    """Yellow Ø45 'EMERGENCY STOP' legend plate under the E-stop's bezel (~1 thick, INFERRED)."""
    return cyl(22.5, 0.0, 1.0) - cyl(8.2, -0.1, 1.1)


def toggle_kn3c(panel_t):
    """KN3(C)-103 type ON-OFF-ON toggle (Micros / NINIGI drawings), M12x0.75 bushing (Ø12.2-12.5 hole, a 1.9 wide key
    notch toward +y for the lock washer's tab, ~1.8 x 1.3 x panel deep, INFERRED), lever Ø6 17.5 above
    the bushing top, body 29 (along the throw, x) x 15.8 x 19 behind the panel, M3 screw terminals 7 below it. Two
    nuts (~14 AF x 3, INFERRED): one behind, one in front on a washer."""
    bushing = cyl(6.0, -panel_t - 3.0, 9.5 - panel_t - 3.0)
    front_nut = hexagon(14.0, 0.0, 3.0)
    back_nut = hexagon(14.0, -panel_t - 3.0, -panel_t)
    lever = Pos(0, 0, 9.5 - panel_t - 3.0) * Rot(0, 12, 0) * Cylinder(3.0, 17.5, align=MIN)   # thrown to one side
    body = box(-14.5, 14.5, -7.9, 7.9, -panel_t - 3.0 - 19.0, -panel_t - 3.0)
    terms = box(-10.0, 10.0, -4.0, 4.0, -panel_t - 3.0 - 26.0, -panel_t - 3.0 - 19.0)
    tab = box(-0.9, 0.9, 5.6, 6.9, -panel_t, 0.0)         # lock-washer tab in the key notch
    return bushing + front_nut + back_nut + lever + body + terms + tab


def pushbutton_gq16(panel_t):
    """Onpow GQ16F-10 16 mm metal pushbutton, momentary NO (Ø16 hole, M16x1, panel <= 6): Ø18 flat head 1.5 proud,
    Ø11.8 button, 23 max behind the panel with the screw-terminal block; nut ~21.8 (INFERRED) behind the panel."""
    head = cyl(9.0, 0.0, 1.5) + cyl(5.9, 1.5, 2.0)
    thread = cyl(8.0, -18.0, 0.0)
    nut = hexagon(19.0, -panel_t - 3.0, -panel_t)
    term = box(-3.1, 3.1, -6.0, 6.0, -23.0, -18.0)
    return head + thread + nut + term


def encoder_pec11r(panel_t, knob=True):
    """Bourns PEC11R-4220F-S0024 (bourns.com/docs/Product-Datasheets/PEC11R.pdf): 12.5 x 13.4 body 6.5 deep behind its
    mounting face, M7x0.75 bushing 7 long (nut and washer on the front, no anti-rotation lug), Ø6 D shaft 20 from the
    mounting face, push switch; pins 3.7 below the body. With a Ø30 skirted knob (Ø17.2 grip, 17 high, Ø6 bore 15
    deep, grub screw; INFERRED), 1 mm above the bushing's top (which stands 1.5 above the nut): the shaft
    goes 12 into the bore and ends 3 short of its bottom. The knob is
    returned as a separate solid so its fit on the shaft is checked."""
    body = box(-6.25, 6.25, -6.7, 6.7, -panel_t - 6.5, -panel_t)
    pins = box(-5.0, 5.0, -0.5, 0.5, -panel_t - 10.2, -panel_t - 6.5)
    bushing = cyl(3.5, -panel_t, 7.0 - panel_t)
    nut = hexagon(11.0, 0.5, 2.5) + (cyl(6.0, 0.0, 0.5) - cyl(3.6, -0.1, 0.6))
    shaft = cyl(3.0, 7.0 - panel_t, 20.0 - panel_t)
    s = body + pins + bushing + nut + shaft
    if knob:
        z0 = 7.0 - panel_t + 1.0                               # 1 above the bushing's top
        k = (cyl(15.0, z0, z0 + 2.0) + cyl(8.6, z0 + 2.0, z0 + 17.0)) - cyl(3.05, z0 - 0.1, z0 + 15.0)
        return Compound(children=[s, k])
    return s


def oled_sh1106():
    """1.3" SH1106 128x64 OLED module (Pololu 3760 drawing as the proxy for the generic 4-pin board): PCB 35.4 x 33.5 x
    1.2 with 4x Ø3.05 holes on 30.4 x 28.5, glass ~34.5 x 23 x 1.5 on its front (+z), active area 29.42 x 14.7 whose
    centre is the frame's origin; 4-pin header along the top edge. PCB front face at z = 0."""
    yc = -(33.5 / 2 - 7.35 - 14.7 / 2)        # PCB centre relative to the active area's centre (2.05 below it)
    pcb = box(-17.7, 17.7, yc - 16.75, yc + 16.75, -1.2, 0.0)
    for dx in (-15.2, 15.2):
        for dy in (-14.25, 14.25):
            pcb = pcb - cyl(1.525, -1.3, 0.1, dx, yc + dy)
    glass_top = yc + 16.75 - 5.25
    glass = box(-17.25, 17.25, glass_top - 23.0, glass_top, 0.0, 1.5)
    header = box(-5.1, 5.1, yc + 16.75 - 3.3, yc + 16.75 - 0.7, -1.2 - 8.5, -1.2)
    return pcb + glass + header


def pushbutton_pbs110(panel_t):
    """PBS-110 7 mm momentary NO (Ø7 hole, M7; mounts from BEHIND with the nut in front): Ø6 cap whose top sits
    ~11.2 - t above the panel front, nut 10 AF x 1.5, rear flange Ø9.8 x 1.6, Ø7.2 body 14.5 below the flange to the
    terminal tips; thread above the flange ~4.9 (panel <= ~3.4)."""
    flange = cyl(4.9, -panel_t - 1.6, -panel_t)
    thread = cyl(3.5, -panel_t, 4.9 - panel_t)
    nut = hexagon(10.0, 0.0, 1.5)
    cap = cyl(3.0, 4.9 - panel_t, 11.2 - panel_t)
    body = cyl(3.6, -panel_t - 1.6 - 9.0, -panel_t - 1.6)
    terms = box(-2.0, 2.0, -0.25, 0.25, -panel_t - 1.6 - 14.5, -panel_t - 1.6 - 9.0)
    return flange + thread + nut + cap + body + terms


def usb_c_panel_6069():
    """Adafruit #6069 USB-C round panel-mount extension, panel end (adafruit.com/product/6069 and its fab drawing):
    M12x1 thread through a Ø12.5 hole, ~12 proud of the panel with its nut (~17 AF x 4.5, INFERRED from photos) and a
    tethered dust cap (Ø16 x 9, INFERRED) screwed on; Ø18 x 6 flange on the inside (modelled by the caller's USB
    path). Panel outer face at z = 0, front toward +z."""
    return cyl(5.7, 0.0, 12.0) + hexagon(17.0, 0.0, 4.5) + cyl(8.0, 6.0, 15.0)


# ------------------------------------------------------------------ end-wall and box parts
def iec_inlet_se_as_303():
    """Switch Electronics SE-AS-303 type snap-in C14 + 5x20 fuse + rocker (cdn.shopify.com/.../489596.pdf): cut-out
    47.2 x 28.3, flange 50 x 30.8 standing 3.3 proud of the panel, body 47 x 27.5, 20.5 deep from the front with
    4.8 mm tabs 10.5 further. C14 bay 24 x 16 left, fuse drawer under it,
    rocker 19 x 13 in a 21.2 x 15.2 bezel right (the side-by-side 3-in-1 layout; positions within the face INFERRED).
    Width along x, height along y, the panel's front face at z = 0 (front +z)."""
    flange = box(-25.0, 25.0, -15.4, 15.4, 0.0, 3.3)
    body = box(-23.5, 23.5, -13.75, 13.75, 3.3 - 20.5, 0.0)
    tabs = box(-15.0, 15.0, -10.0, 10.0, 3.3 - 31.0, 3.3 - 20.5)
    s = flange + body + tabs
    s = s - box(-22.5, 1.5, 15.4 - 2.7 - 16.0, 15.4 - 2.7, 3.3 - 18.6, 3.4)     # C14 bay, left (1 mm walls; C13 nose 18.5 in)
    s = s + box(2.5, 23.7, -7.6, 7.6, 3.3, 4.3) + box(3.6, 22.6, -6.5, 6.5, 4.3, 7.0)   # rocker bezel + rocker, right
    s = s - box(-21.5, -0.5, -14.5, -5.0, 2.0, 3.4)                               # fuse drawer, under the bay
    return s


def c13_plug():
    """Moulded C13 cord connector (Linetek LS-13, temflexcontrols.com/pdf/power-cord-cat.pdf): nose 23 x ~15 x 18.5
    (into the inlet's C14 bay), body 24.6 x 17 x 19, strain relief Ø13.6 x 17. Mating face at z = 0, cord toward +z."""
    nose = box(-11.5, 11.5, -7.5, 7.5, -18.5, 0.0)
    body = box(-12.3, 12.3, -8.5, 8.5, 0.0, 19.0)
    relief = cyl(6.8, 19.0, 36.0) + cyl(7.2, 30.0, 33.5)
    return nose + body + relief


def gland_m16(panel_t):
    """Lapp SKINTOP ST-M 16x1.5 (Ø16.2 hole): hex 19 AF, dome ~21 above the panel, thread 8; SKINDICHT SM-M locknut
    19 AF x 3 inside."""
    out = hexagon(19.0, 0.0, 6.0) + cyl(8.5, 6.0, 15.0) + cyl(6.0, 15.0, 21.0)
    thread = cyl(8.0, -8.0, 0.0)
    nut = hexagon(19.0, -panel_t - 3.0, -panel_t)
    return out + thread + nut


def fan_40x10():
    """Sunon MF40100V2-1000U-A99 40 x 40 x 10 fan (Sunon MF40100V2 drawing, sunon.com product page / DigiKey
    259-1580-ND: 4x Ø4.3 on 32, Ø38 venturi, hub ~Ø21 INFERRED), blowing toward +z; inlet face at z = 0."""
    frame = box(-20.0, 20.0, -20.0, 20.0, 0.0, 10.0) - cyl(19.0, -0.1, 10.1)
    for dx in (-16.0, 16.0):
        for dy in (-16.0, 16.0):
            frame = frame - cyl(2.15, -0.1, 10.1, dx, dy)
    hub = cyl(10.5, 0.0, 10.0)
    blades = Compound(children=[Rot(0, 0, 360 / 7 * k) * Pos(14.0, 0, 5.0) * Rot(25, 0, 0) * Box(9.0, 1.0, 8.0) for k in range(7)])
    return frame + hub + blades
