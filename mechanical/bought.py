"""Bought parts, modelled from their datasheets (sources in each docstring), each in its own frame. The feed unit places
them; the checks and the viewer use these shapes instead of stand-in blocks.

Every part here is one that Amazon.com delivers to Israel (checked with an Israeli address, 2026-10-22); the docstring names
the listing (ASIN) and the drawing the dimensions come from. Where the listing gives no number, it says INFERRED.

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
PULLEYS = {   # teeth: (flange Ø, flange thickness, tooth width), both hubless (flange, teeth, flange)
    20: (35.0, 2.0, 16.0),   # Saelno 20-5M-15-8mm, B0FNQTJZMX: tooth OD 30.69, flanges Ø35, 16 between them, 20 overall
    40: (67.0, 3.0, 16.0),   # uxcell 5M-40T-1/2B, B0G5YDQS1N: flanges Ø67, 22 overall (16 + 2 x 3 INFERRED)
}


def htd5m_pulley(teeth, bore, set_d):
    """HTD 5M pulley for a 15 mm belt, both bought hubless (PULLEYS: flange, teeth, flange), centred on z = 0, re-bored
    to the shaft (the 20T comes Ø8, the 40T Ø12.7 with a keyway: Ø15 leaves a 0.25 trace of it). OD = PD - 1.14
    (30.69 for the 20T, as its drawing). Teeth modelled as grooves, 2 set screws (90 deg apart) radially through a
    tooth root at mid-width (the 20T comes with 2x M5 there)."""
    pd = teeth * 5.0 / math.pi
    od = pd - 1.14
    fl, ft, w = PULLEYS[teeth]
    body = cyl(od / 2, -w / 2, w / 2)
    groove = Pos(od / 2, 0, -w / 2 - 1) * Box(4.12, 3.0, w + 2, align=MIN)          # HTD groove 2.06 deep, ~3 wide
    for k in range(teeth):
        body = body - Rot(0, 0, 360.0 / teeth * (k + 0.5)) * groove
    body = body + cyl(fl / 2, -w / 2 - ft, -w / 2) + cyl(fl / 2, w / 2, w / 2 + ft)
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
    """Aideepen LPD3806-600BM-G5-24C, 600 P/R, NPN open collector, 5-24 V (Amazon B01MZ4V1XP drawing): Ø38 x 35 body,
    Ø20 x 5 face boss, Ø6 shaft 13 beyond the boss with a flat (5.5 across) 10 long, 3x M3 x 8 deep on Ø30 and 3x on
    Ø28 (30 deg apart) in the face, radial cable exit, 1.5 m cable (green A, white B, red +V, black 0 V).
    Face (with the boss and shaft) toward -z; the body's front face at z = 0."""
    body = cyl(19.0, 0.0, 35.0)
    for k in range(3):
        for rr, off in ((15.0, 0.0), (14.0, 30.0)):
            a = math.radians(90 + 120 * k + 60 + off)
            body = body - cyl(1.25, -0.1, 6.0, rr * math.cos(a), rr * math.sin(a))
    boss = cyl(10.0, -5.0, 0.0)
    shaft = cyl(3.0, -18.0, -5.0) - Pos(3.0 - 0.5 + 5, 0, -18.0) * Box(10, 10, 10, align=MIN)
    cable = Pos(0, 19.0, 28.5) * Rot(-90, 0, 0) * Cylinder(3.0, 8.0, align=MIN)
    return body + boss + shaft + cable


# ------------------------------------------------------------------ power supply
LRS150_POLE1 = 15.8      # x of terminal 1 (L) from the case's x = 0 edge; 7 poles at 9.5
LRS150_TERM_Y = 6.6      # terminal screw centres from the terminal end


def lrs150():
    """Mean Well LRS-150-36 (Amazon B019GYOQDW, PowerNex, ships to Israel; meanwell.com/Upload/PDF/LRS-150/LRS-150-SPEC.PDF,
    case 241A, ±1): 159 x 97 x 30. Frame: base at z = 0, the 159 length along +y from the terminal end at y = 0, the 97
    width along x from x = 0. Base holes 2x M3 L=3 at 24 and 102 from the terminal end, 32 from the x = 97 edge. The
    7-pole terminal block (1 L, 2 N, 3 FG, 4-5 -V, 6-7 +V, pitch 9.5, screws facing up) runs from 10.2 to 78.5 along
    the terminal end and 16 into the case; terminal 1 at 15.8 from the x = 0 edge, screw centres 6.6 from the end
    (measured on the drawing). Block and screw heads sit in a recess under the top, heads ~3.5 below it (INFERRED, as on
    the LRS-75's drawing)."""
    case = box(0, 97.0, 0, 159.0, 0, 30.0)
    x0, x1 = LRS150_POLE1 - 4.75 - 1.0, LRS150_POLE1 + 6 * 9.5 + 4.75 + 1.0
    case = case - box(x0, x1, -0.1, 17.0, 26.0, 30.1)                  # terminal recess
    for k, head in enumerate(lrs150_terminal_heads()):       # terminal screw heads (Ø6.5, 2.5 high) on their clamps
        x = LRS150_POLE1 + 9.5 * k
        case = case + head
        case = case - cyl(1.2, 25.6, 26.6, x, LRS150_TERM_Y) - box(x - 0.4, x + 0.4, LRS150_TERM_Y - 2, LRS150_TERM_Y + 2, 26.0, 26.6)
    return case


def lrs150_terminal_heads():
    """The 7 terminal screw heads of the LRS-150 (its frame), for clearance checks."""
    return [cyl(3.25, 24.0, 26.5, LRS150_POLE1 + 9.5 * k, LRS150_TERM_Y) for k in range(7)]


# ------------------------------------------------------------------ panel controls (panel outer face at z = 0)
def hexagon(af, z0, z1, x=0.0, y=0.0):
    """Hex nut outline, `af` across flats."""
    r = af / math.sqrt(3)
    pts = [(x + r * math.cos(math.radians(60 * k)), y + r * math.sin(math.radians(60 * k))) for k in range(6)]
    return extrude(Pos(0, 0, z0) * make_face(Polyline(*pts, close=True)), amount=z1 - z0)


ESTOP_KEY = (3.5, 24.1)      # the 22 mm E-stop's notch: width (3.3 +0.2 on its drawing: cut at the top), extent over Ø22.35


def estop_zgszjhf22(panel_t):
    """ZGSZJHF 22 mm ultra-short E-stop, 1 NO + 1 NC, latching, twist to release (Amazon B0FZCN2HCP drawing): Ø22.35
    hole with a 3.3 wide anti-rotation notch (24.1 over the notch), panel 1-4 mm; Ø32.3 mushroom (16.8 tall) whose top
    is 25.7 above the panel face; the body ends 10.2 behind the panel face and its 4 pins (14.2 x 8.6 apart) 18.3
    behind it. Its ring nut and the key in the notch are drawn but not dimensioned: nut Ø30 x 4, key 3 wide (INFERRED);
    collar Ø28 x 2.5 and neck Ø24 between the head and the panel (INFERRED from the drawing's proportions). Wires
    soldered to the pins and bent sideways: an 18 x 12 x 8 envelope behind them. Panel face at z = 0 (here the top of
    the legend plate the collar sits on); the notch toward +y."""
    h0 = 25.65 - 16.76
    collar = cyl(14.0, 0.0, 2.5)
    neck = cyl(12.0, 2.5, h0)
    head = cyl(16.13, h0, 25.65)
    body = cyl(11.0, -10.15, 0.0)
    key = box(-1.5, 1.5, 10.5, 11.9, -panel_t, 0.0)
    nut = cyl(15.0, -panel_t - 4.0, -panel_t)
    pins = None
    for px in (-7.1, 7.1):
        for py in (-4.3, 4.3):
            p = box(px - 0.6, px + 0.6, py - 0.3, py + 0.3, -18.3, -10.15)
            pins = p if pins is None else pins + p
    wires = box(-9.0, 9.0, -6.0, 6.0, -26.3, -18.3)
    return collar + neck + head + body + key + nut + pins + wires


def estop_legend_plate():
    """Yellow Ø45 'EMERGENCY STOP' legend plate under the E-stop's collar, 1 thick: PRINT it (yellow PETG), with the
    E-stop's Ø22.35 hole and key notch."""
    w, ext = ESTOP_KEY
    return cyl(22.5, 0.0, 1.0) - cyl(11.2, -0.1, 1.1) - box(-w / 2, w / 2, 0.0, ext - 11.2, -0.1, 1.1)


def toggle_eten1322(panel_t):
    """DMWD ON-OFF-ON latching DPDT toggle, E-TEN1322 type, 6 screw terminals (Amazon B0BTSPYM34 drawing): Ø12 hole;
    bushing Ø11.7 with a 1.0 deep x 2.0 wide key slot (thread pitch and length not given: M12, ~9.5 long, INFERRED),
    lever 17.3 above the bushing; body with its terminals 29.7 along the throw (x) x 19.8, 28.7 from the body's top to
    the ends of the terminal screws (body 22 + screws, INFERRED from the photo). Two nuts ~14 AF x 3 (INFERRED), one
    behind, one in front on a washer; a keyed washer's tab in a notch at +y (INFERRED)."""
    bushing = cyl(5.85, -panel_t - 3.0, 9.5 - panel_t - 3.0)
    front_nut = hexagon(14.0, 0.0, 3.0)
    back_nut = hexagon(14.0, -panel_t - 3.0, -panel_t)
    lever = Pos(0, 0, 9.5 - panel_t - 3.0) * Rot(0, 12, 0) * Cylinder(3.0, 17.3, align=MIN)   # thrown to one side
    z_top = -panel_t - 3.0
    body = box(-14.85, 14.85, -9.9, 9.9, z_top - 22.0, z_top)
    terms = None
    for tx in (-10.2, 0.0, 10.2):                          # 3 x 2 screw terminals (tabs 7.1 wide)
        for ty in (-5.5, 5.5):
            t_ = box(tx - 3.55, tx + 3.55, ty - 3.5, ty + 3.5, z_top - 28.7, z_top - 22.0)
            terms = t_ if terms is None else terms + t_
    tab = box(-0.9, 0.9, 5.6, 6.9, -panel_t, 0.0)         # lock-washer tab in the key notch
    return bushing + front_nut + back_nut + lever + body + terms + tab


def pushbutton_16mm(panel_t):
    """Gebildet 16 mm stainless momentary NO pushbutton, screw terminals (Amazon B081JJBN4G drawing): Ø16 hole, M16x1;
    Ø18 head 1.9 proud, Ø11.6 button; threaded body 11.7 and the terminal block to 22 behind the head (screws 5.9
    apart). Nut ~20 AF x 3 (INFERRED: not dimensioned) behind the panel."""
    head = cyl(9.0, 0.0, 1.9) + cyl(5.8, 1.9, 2.3)
    thread = cyl(8.0, -11.7, 0.0)
    block = box(-6.5, 6.5, -8.0, 8.0, -20.0, -11.7)
    term = box(-3.0, 3.0, -5.2, 5.2, -22.0, -20.0)
    nut = hexagon(20.0, -panel_t - 3.0, -panel_t)
    return head + thread + block + term + nut


def encoder_pec11r(panel_t, knob=True):
    """Bourns PEC11R-4220F-S0024 (bourns.com/docs/Product-Datasheets/PEC11R.pdf): 12.5 x 13.4 body 6.5 deep behind its
    mounting face, M7x0.75 bushing 7 long (nut and washer on the front, no anti-rotation lug), Ø6 D shaft 20 from the
    mounting face, push switch; pins 3.7 below the body. With the uxcell Ø30 knurled aluminium handwheel (Amazon
    B0DGFB1K9F drawing: Ø30 x 10 knurled rim, Ø18 x 10 hub with 2 set screws, Ø6 bore through, 20 overall), hub toward
    the panel, 1 mm above the bushing's top (which stands 1.5 above the nut): the shaft goes 12 into the hub, past its
    set screws. The knob is returned as a separate solid so its fit on the shaft is checked."""
    body = box(-6.25, 6.25, -6.7, 6.7, -panel_t - 6.5, -panel_t)
    pins = box(-5.0, 5.0, -0.5, 0.5, -panel_t - 10.2, -panel_t - 6.5)
    bushing = cyl(3.5, -panel_t, 7.0 - panel_t)
    nut = hexagon(11.0, 0.5, 2.5) + (cyl(6.0, 0.0, 0.5) - cyl(3.6, -0.1, 0.6))
    shaft = cyl(3.0, 7.0 - panel_t, 20.0 - panel_t)
    s = body + pins + bushing + nut + shaft
    if knob:
        z0 = 7.0 - panel_t + 1.0                               # 1 above the bushing's top
        k = (cyl(9.0, z0, z0 + 10.0) + cyl(15.0, z0 + 10.0, z0 + 20.0)) - cyl(3.05, z0 - 0.1, z0 + 20.1)
        return Compound(children=[s, k])
    return s


OLED_HOLES = (30.3, 28.0)    # hole centres of the Hosyond board (its photo callouts)
OLED_YC = -(33.7 / 2 - 7.35 - 14.7 / 2)   # its PCB centre relative to the active area's centre (2.15 below it)


def oled_sh1106():
    """Hosyond 1.3" SH1106 128x64 I2C OLED, white (Amazon B0C3L7N917: PCB 35.5 x 33.7, holes on 30.3 x 28, active area
    29.42 x 14.7, 4-pin header GND VCC SCL SDA along the top edge). The listing gives no glass, hole or thickness
    numbers: glass ~34.5 x 23 x 1.5, holes Ø3, PCB 1.2 and the glass position are from the Pololu 3760 drawing of the
    same generic board (INFERRED). The active area's centre is the frame's origin; PCB front face at z = 0."""
    yc = OLED_YC
    pcb = box(-17.75, 17.75, yc - 16.85, yc + 16.85, -1.2, 0.0)
    hx, hy = OLED_HOLES[0] / 2, OLED_HOLES[1] / 2
    for dx in (-hx, hx):
        for dy in (-hy, hy):
            pcb = pcb - cyl(1.5, -1.3, 0.1, dx, yc + dy)
    glass_top = yc + 16.85 - 5.25
    glass = box(-17.25, 17.25, glass_top - 23.0, glass_top, 0.0, 1.5)
    header = box(-5.1, 5.1, yc + 16.85 - 3.3, yc + 16.85 - 0.7, -1.2 - 8.5, -1.2)
    return pcb + glass + header


def pushbutton_pbs110(panel_t):
    """Twidec PBS-110 7 mm momentary NO, pre-wired (Amazon B07RPS2ZY3 drawing: Ø7 thread, Ø6 cap, nut 10 AF / 11 AC,
    8.6 body below the flange, 28 from the cap's top to the heat-shrunk wire joints, 120 mm wires). Mounts from BEHIND
    with the nut in front: cap top ~11.2 - t above the panel front, nut 10 AF x 1.5, rear flange Ø9.8 x 1.6, body and
    wire joints 14.5 below the flange; thread above the flange ~4.9 (panel <= ~3.4). Cap height, flange and nut
    thickness are not on the listing (INFERRED from the PBS-110 standard part)."""
    flange = cyl(4.9, -panel_t - 1.6, -panel_t)
    thread = cyl(3.5, -panel_t, 4.9 - panel_t)
    nut = hexagon(10.0, 0.0, 1.5)
    cap = cyl(3.0, 4.9 - panel_t, 11.2 - panel_t)
    body = cyl(3.6, -panel_t - 1.6 - 9.0, -panel_t - 1.6)
    terms = box(-2.0, 2.0, -0.25, 0.25, -panel_t - 1.6 - 14.5, -panel_t - 1.6 - 9.0)
    return flange + thread + nut + cap + body + terms


def usb_c_panel(panel_t):
    """QIANRENON USB-C 5 Gbps threaded panel-mount extension, female in the panel, 30 cm cable to a male plug (Amazon
    B0FX2MZDTS drawing): Ø21.5 hole, M21 thread, panel <= 8, Ø27.5 flange outside, 37 from the flange's face to the
    end of the strain relief. Not dimensioned (INFERRED from the photo): flange 2.5 thick, hex nut 24 AF x 3 inside,
    body Ø18 to 27 behind the panel face, strain relief Ø8 to 34.5. The male plug's length isn't given either: the
    caller's USB path allows 38. Panel outer face at z = 0, front toward +z."""
    flange = cyl(13.75, 0.0, 2.5)
    thread = cyl(10.5, -11.0, 0.0)
    nut = hexagon(24.0, -panel_t - 3.0, -panel_t)
    body = cyl(9.0, -27.0, -11.0)
    relief = cyl(4.0, -34.5, -27.0)
    return flange + thread + nut + body + relief


# ------------------------------------------------------------------ end-wall and box parts
def iec_inlet_snapin():
    """JOYNANO 3-in-1 snap-in C14 inlet + 5x20 fuse + lit rocker, for 0.8-1.5 mm panels (Amazon B071VLCTPB: cut-out
    47 x 28, 50 x 30 x 32 overall, 5 A fuse supplied: fit T 2 A). Its listing has no drawing, so the shape is the same
    generic module's drawing on Amazon B0H5TNSZ7L (SINORS): flange 50 x 30.8 x 3, cut-out 47 x 27.5 R2, body 20.5 from
    the flange's face, tabs 10 further; tabs taken to 29 behind the panel (32 overall). C14 bay 24 x 16 left, fuse
    drawer under it, rocker 19 x 13 in a 21.2 x 15.2 bezel right (positions within the face INFERRED).
    Width along x, height along y, the panel's front face at z = 0 (front +z)."""
    flange = box(-25.0, 25.0, -15.4, 15.4, 0.0, 3.0)
    body = box(-23.5, 23.5, -13.75, 13.75, 3.0 - 20.5, 0.0)
    tabs = box(-15.0, 15.0, -10.0, 10.0, -29.0, 3.0 - 20.5)
    s = flange + body + tabs
    s = s - box(-22.5, 1.5, 15.4 - 2.7 - 16.0, 15.4 - 2.7, 3.0 - 18.6, 3.1)     # C14 bay, left (1 mm walls; C13 nose 18.5 in)
    s = s + box(2.5, 23.7, -7.6, 7.6, 3.0, 4.0) + box(3.6, 22.6, -6.5, 6.5, 4.0, 6.7)   # rocker bezel + rocker, right
    s = s - box(-21.5, -0.5, -14.5, -5.0, 1.7, 3.1)                               # fuse drawer, under the bay
    return s


def c13_plug():
    """The C13 end of the mains cord (an ordinary C13 cord with an Israeli plug, bought locally; the shape is the
    Linetek LS-13 catalogue's, temflexcontrols.com/pdf/power-cord-cat.pdf, INFERRED):
    nose 23 x ~15 x 18.5 (into the inlet's C14 bay), body 24.6 x 17 x 19, strain relief Ø13.6 x 17. Mating face at
    z = 0, cord toward +z."""
    nose = box(-11.5, 11.5, -7.5, 7.5, -18.5, 0.0)
    body = box(-12.3, 12.3, -8.5, 8.5, 0.0, 19.0)
    relief = cyl(6.8, 19.0, 36.0) + cyl(7.2, 30.0, 33.5)
    return nose + body + relief


def gland_m16(panel_t):
    """uxcell M16x1.5 nylon cable gland, cable 4-8 (Amazon B08K8HPHGM: thread 9 long; Ø16.2 hole). Its hex, dome and
    locknut aren't on the listing: hex 19 AF, dome ~21 above the panel, locknut 19 AF x 3 inside, as the Lapp SKINTOP
    ST-M 16x1.5 of the same size (INFERRED)."""
    out = hexagon(19.0, 0.0, 6.0) + cyl(8.5, 6.0, 15.0) + cyl(6.0, 15.0, 21.0)
    thread = cyl(8.0, -9.0, 0.0)
    nut = hexagon(19.0, -panel_t - 3.0, -panel_t)
    return out + thread + nut


def fan_40x10():
    """Noctua NF-A4x10 5V, 40 x 40 x 10, 3-pin (Amazon B00NEMGCIA; Noctua's NF-A4x10 drawing: 4 holes on 32 for the
    supplied self-tapping screws; hole Ø, venturi and hub INFERRED as the usual 40 mm frame), blowing toward +z;
    inlet face at z = 0."""
    frame = box(-20.0, 20.0, -20.0, 20.0, 0.0, 10.0) - cyl(19.0, -0.1, 10.1)
    for dx in (-16.0, 16.0):
        for dy in (-16.0, 16.0):
            frame = frame - cyl(2.15, -0.1, 10.1, dx, dy)
    hub = cyl(10.5, 0.0, 10.0)
    blades = Compound(children=[Rot(0, 0, 360 / 7 * k) * Pos(14.0, 0, 5.0) * Rot(25, 0, 0) * Box(9.0, 1.0, 8.0) for k in range(7)])
    return frame + hub + blades
