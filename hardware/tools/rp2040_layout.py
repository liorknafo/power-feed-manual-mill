"""Replace the Pico module (A1) with the bare RP2040 circuit (edits power-feed.kicad_pcb in place; rerunnable).

Run with KiCad's Python after manual_fixes.py, silk_cleanup.py and layout_fixes_v2.py:
  /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3 \
      hardware/tools/rp2040_layout.py

Steps:
1. Renumber. Adding the RP2040 parts renumbered the schematic (refmap.OLD2NEW); the footprints are renamed to match.
2. Sync from the schematic netlist (kicad-cli sch export netlist): every footprint gets the schematic's value, fields,
   symbol path and pad nets; the new parts are loaded from the KiCad libraries; A1 is deleted.
3. Place the RP2040 group where the Pico was, the USB-C on the bottom edge between J6 and H4, and move the panel
   input filters that sat there (RAPID, MODE_R, DIAL_A/B/SW) to the free strip along the right edge. Connectors
   J1, J2, J5 (clutch), J6 (panel), J7 (encoder) and the mounting holes stay where they are.
4. Copper. rp2040_routes.json owns the copper of every net of the moved / new parts (OWNED_NETS) plus the
   GND / +3V3 / +5V copper inside the two rework boxes (BOXES): it is deleted and redrawn from the file.
   The file was produced by --autoroute: PREROUTE (hand routing of the chip's power pins, the crystal, the QSPI
   lines, the USB pair and the +5V trunk) is drawn and locked, every GND / +3V3 pad gets a fan-out via, and
   Freerouting routes the rest with 0.5 mm clearance to the 36 V nets. Later hand edits go back into the file
   with --dump. GND stitching vias under the redrawn copper are dropped.
5. Zones are refilled. Run silk_cleanup.py again afterwards.

Dev options: --place-only (steps 1-3, owned copper removed, no routes), --autoroute (Freerouting on the placed
board, writes rp2040_routes.json), --dump (current board -> rp2040_routes.json).
"""

import json
import os
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_pcb import FREEROUTING, JAVA, KDIR, PCB, mm  # noqa: E402
from refmap import OLD2NEW  # noqa: E402

KCLI = "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"
FPLIB = "/Applications/KiCad/KiCad.app/Contents/SharedSupport/footprints"
SCH = os.path.join(KDIR, "power-feed.kicad_sch")
ROUTES = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rp2040_routes.json")
F, B = pcbnew.F_Cu, pcbnew.B_Cu
MCU = "/RP2040, SPI, E-stop interlock/"

# ref: (x, y, orientation deg). All on the top side.
PLACE = {
    # RP2040, rotated 180: crystal / SWD / RUN north, QSPI + USB + core supply south, SPI + ADC west, I/O east
    "U3": (60.0, 13.0, 180),
    # decoupling: IOVDD 1 + 10 (east) and 33 + 42 (west) are tied pairwise by +3V3 traces inside the pad ring, with
    # C29 at pin 1, C33 at pin 42 and C30 / C32 just beyond them (out of the signal escapes); IOVDD 22 (north) has a
    # via inside the pad ring into the +3V3 plane (C31 nearby); USB_VDD 48 + IOVDD 49 -> C45 + C35; ADC_AVDD 43 +
    # VREG_VIN 44 -> C40 + C36 (1 uF); VDD_CORE: C37 1 uF at VREG_VOUT 45, C38 + C39 at DVDD 50, and DVDD 23 / 50
    # are tied by a B.Cu spine under the chip
    "C29": (65.9, 16.3, 0), "C30": (70.6, 16.3, 0), "C32": (54.6, 18.6, 180), "C33": (54.3, 15.6, 180),
    "C31": (52.6, 7.0, 90), "C39": (62.7, 21.3, 270),
    "C40": (51.4, 21.3, 270), "C36": (52.9, 21.3, 270), "C37": (54.4, 21.3, 270),
    "C45": (59.4, 21.3, 270), "C35": (59.4, 24.4, 90), "C38": (61.2, 21.3, 270),
    # crystal: XIN straight up to pad 1, XOUT through R26 (1k) to pad 3, 15 pF caps at the pads, no vias
    "Y1": (62.0, 5.4, 90), "R26": (59.4, 5.0, 90), "C41": (65.3, 6.5, 0), "C42": (61.6, 2.2, 0),
    # QSPI flash south-east of the chip (rotated so all six lines run without vias), 100 nF at VCC, SS pull-up,
    # BOOTSEL (1k + button), RESET button + RUN pull-up
    "U4": (69.0, 22.0, 180), "C51": (63.6, 24.6, 270), "R24": (77.0, 24.0, 0), "R25": (76.8, 20.4, 0),
    "SW1": (82.0, 8.0, 0), "SW2": (37.5, 4.5, 0), "R27": (44.4, 6.6, 0),
    # SWD header (1x3 THT) and RESET button top left, next to the chip's SWD / RUN pins; status LED
    "J3": (46.0, 3.2, 90), "R28": (43.8, 22.0, 180), "D4": (40.8, 22.0, 180),
    # USB: 27R right below the chip, then the pair runs to the USB-C on the bottom edge (mouth on the edge, y = 100)
    "R32": (56.2, 21.3, 90), "R31": (57.7, 21.3, 90),
    "J4": (87.6, 96.35, 0), "R29": (89.6, 89.6, 0), "R30": (89.6, 88.0, 0),
    # supply: +5V -> D5 -> VSYS and VBUS -> D6 -> VSYS, AP2112K-3.3 -> +3V3 (the only +3V3 source, into In2)
    "D5": (69.0, 28.7, 180), "D6": (90.5, 19.5, 90), "U5": (86.0, 24.6, 180), "C43": (88.9, 24.6, 90),
    "C44": (82.6, 25.2, 90),
    # panel input filters moved off the USB-C to the right-edge strip (pull-up 10k, series 1k, cap)
    "R61": (96.8, 58.0, 90), "R62": (94.8, 58.0, 90), "C55": (96.8, 61.4, 90),
    "R69": (96.8, 65.0, 90), "R70": (94.8, 65.0, 90), "C59": (94.8, 68.4, 90),
    "R67": (96.8, 72.0, 90), "R68": (94.8, 72.0, 90), "C58": (96.8, 75.4, 90),
    "R71": (96.8, 79.0, 90), "R72": (94.8, 79.0, 90), "C60": (96.8, 82.4, 90),
    "R59": (96.8, 86.0, 90), "R60": (94.8, 86.0, 90), "C54": (94.8, 89.4, 90),
}
REDRAW = {"LIM_L", "TMC_DIAG0", "TMC_DIAG1", "CLUTCH_HS_PWM", "CLUTCH_LS_EN"}
FIXED = {"J1", "J2", "J5", "J6", "J7", "H1", "H2", "H3", "H4"}
SHARED = {"GND", "+3V3", "+5V"}
# rework boxes (x0, y0, x1, y1) mm: shared-net copper wholly inside is owned by rp2040_routes.json (the old Pico
# area, the old filter / new USB-C area, the right-edge strip the filters moved to, the old Pico 3V3 feed to C34)
BOXES = [(37.0, 0.0, 100.0, 31.0), (31.5, 0.0, 37.0, 9.0), (92.0, 74.0, 92.8, 74.8), (74.0, 85.5, 100.0, 100.0),
         (93.5, 55.0, 100.0, 92.0), (85.0, 27.5, 85.8, 46.2)]


# Hand pre-routes (locked before Freerouting runs, so it routes the rest around them). (net, layer, width, points);
# a layer "V" entry is a via list (net, "V", diameter, points). Nets are given by their last path element.
PREROUTE = [
    # south side of U3: AVDD 43 + VREG_VIN 44 -> C40 / C36, VREG_VOUT 45 -> C37, USB 46/47 -> 27R, USB_VDD 48 +
    # IOVDD 49 -> C45, DVDD 50 -> C38 (and a via inside the pad ring to the VDD_CORE spine)
    ("+3V3", "F", 0.2, [(57.4, 16.88), (57.4, 17.1), (57.8, 17.1), (57.8, 16.88)]),
    ("+3V3", "F", 0.3, [(57.4, 17.1), (53.4, 17.1), (52.9, 17.6), (52.9, 20.4)]),
    ("+3V3", "F", 0.3, [(52.9, 17.6), (51.4, 17.6), (51.4, 20.4)]),
    ("VDD_CORE", "F", 0.25, [(58.2, 16.88), (58.2, 17.3), (55.0, 20.5), (54.4, 20.5)]),
    ("USB_DM", "F", 0.2, [(58.6, 16.88), (58.6, 17.5), (56.3, 19.8), (56.2, 20.3)]),
    ("USB_DP", "F", 0.2, [(59.0, 16.88), (59.0, 17.9), (57.7, 19.2), (57.7, 20.3)]),
    ("+3V3", "F", 0.2, [(59.4, 16.88), (59.4, 20.3)]),
    ("+3V3", "F", 0.2, [(59.8, 16.88), (59.8, 19.6), (59.4, 20.0)]),
    ("VDD_CORE", "F", 0.2, [(60.2, 16.88), (60.2, 20.0), (60.725, 20.525)]),
    ("VDD_CORE", "F", 0.2, [(60.2, 16.1), (60.2, 15.3)]),
    ("VDD_CORE", "V", 0.5, [(60.2, 15.3), (59.3, 10.7)]),
    ("VDD_CORE", "B", 0.25, [(60.2, 15.3), (57.9, 15.3), (57.9, 10.7), (59.3, 10.7)]),  # round the B.Cu EP pad
    ("VDD_CORE", "F", 0.2, [(59.4, 9.9), (59.3, 10.7)]),
    # north: IOVDD 22 -> via inside the pad ring into the In2 +3V3 plane; crystal without vias
    ("+3V3", "F", 0.2, [(59.8, 9.9), (60.0, 10.7)]),
    ("+3V3", "V", 0.5, [(60.0, 10.7)]),
    ("XOUT", "F", 0.2, [(60.2, 9.3), (60.2, 8.0), (59.4, 7.2), (59.4, 6.0)]),
    ("XIN", "F", 0.2, [(60.6, 9.3), (60.6, 7.45), (62.6, 7.45), (62.85, 7.2), (62.85, 6.5), (64.525, 6.5)]),
    ("XTAL_OUT", "F", 0.2, [(59.4, 4.225), (61.15, 4.225)]),
    ("XTAL_OUT", "F", 0.2, [(60.825, 2.2), (60.825, 3.7)]),
    # IOVDD 1 + 10 and 33 + 42 tied inside the pad ring; pin 1 -> C29, pin 42 -> C33; C32 / C39 on the rails
    ("+3V3", "F", 0.2, [(63.1, 12.0), (62.45, 12.0), (62.45, 15.6), (63.1, 15.6)]),
    ("+3V3", "F", 0.2, [(56.9, 12.0), (57.55, 12.0), (57.55, 15.6), (56.9, 15.6)]),
    ("+3V3", "F", 0.25, [(63.88, 15.6), (64.5, 15.6), (65.125, 16.225)]),
    ("+3V3", "F", 0.25, [(56.12, 15.6), (55.075, 15.6)]),
    ("+3V3", "F", 0.25, [(55.375, 18.6), (55.375, 17.1)]),
    ("VDD_CORE", "F", 0.25, [(61.2, 20.525), (62.7, 20.525)]),
    # signal escapes: every I/O pin gets a short stub out of the pad ring into open copper (Freerouting goes on
    # from the stub ends); SWD and RUN are routed all the way to J3 / SW2
    ("SPI_CS", "F", 0.2, [(57.4, 9.56), (53.5, 9.56)]),
    ("SPI_MISO", "F", 0.2, [(57.8, 9.3), (57.8, 8.9), (57.6, 8.7), (53.5, 8.7)]),
    ("RUN", "F", 0.2, [(58.2, 9.3), (58.2, 8.5), (55.1, 5.4), (42.6, 5.4), (42.6, 2.25), (42.0, 2.25)]),
    ("RUN", "F", 0.2, [(43.575, 6.6), (43.575, 5.4)]),
    ("SWDIO", "F", 0.2, [(58.6, 9.3), (58.6, 8.3), (54.75, 4.45), (51.08, 4.45), (51.08, 4.0)]),
    ("SWCLK", "F", 0.2, [(59.0, 9.3), (59.0, 8.1), (52.3, 1.4), (46.0, 1.4), (46.0, 2.4)]),
    ("RAPID", "F", 0.2, [(61.4, 9.3), (61.4, 7.85), (67.0, 7.85)]),
    ("MODE_R", "F", 0.2, [(61.8, 9.3), (61.8, 8.25), (67.0, 8.25)]),
    ("MODE_L", "F", 0.2, [(62.2, 9.3), (62.2, 8.65), (67.0, 8.65)]),
    ("DIAL_SW", "F", 0.2, [(62.6, 9.3), (62.6, 9.05), (67.0, 9.05)]),
    ("SPI_SCK", "F", 0.2, [(56.3, 10.4), (48.0, 10.4), (47.2, 10.4), (46.8, 10.0)]),
    ("SPI_MOSI", "F", 0.2, [(56.3, 10.8), (48.0, 10.8)]),
    ("DRV_EN", "F", 0.2, [(56.3, 11.2), (48.0, 11.2)]),
    ("TMC_DIAG0", "F", 0.2, [(56.3, 11.6), (48.0, 11.6)]),
    ("TMC_DIAG1", "F", 0.2, [(56.3, 12.4), (48.0, 12.4)]),
    ("LED_STAT", "F", 0.2, [(56.3, 13.6), (48.0, 13.6)]),
    ("VM_SENSE", "F", 0.2, [(56.3, 14.0), (48.0, 14.0)]),
    ("CLUTCH_ISENSE", "F", 0.2, [(56.3, 14.4), (50.9, 14.4), (50.5, 14.8), (50.5, 15.1)]),
    ("DIAL_B", "F", 0.2, [(63.7, 10.4), (68.0, 10.4)]),
    ("DIAL_A", "F", 0.2, [(63.7, 10.8), (68.0, 10.8)]),
    ("CLUTCH_LS_EN", "F", 0.2, [(63.7, 11.6), (68.0, 11.6)]),
    ("LIM_R", "F", 0.2, [(63.7, 12.4), (68.0, 12.4), (68.6, 12.4)]),
    ("LIM_L", "F", 0.2, [(63.7, 12.8), (68.0, 12.8)]),
    ("I2C_SCL", "F", 0.2, [(63.7, 13.2), (68.0, 13.2)]),
    ("I2C_SDA", "F", 0.2, [(63.7, 13.6), (68.0, 13.6)]),
    ("E_OK_SENSE", "F", 0.2, [(63.7, 14.0), (68.0, 14.0)]),
    ("CLUTCH_HS_PWM", "F", 0.2, [(63.7, 14.4), (68.0, 14.4)]),
    ("ENC_OUT_B", "F", 0.2, [(63.7, 14.8), (68.0, 14.8)]),
    ("ENC_OUT_A", "F", 0.2, [(63.7, 15.2), (68.0, 15.2), (68.6, 15.2)]),
    # the four links Freerouting never finished, by hand: ISENSE west -> B.Cu south -> east under the +5V branch
    # to the clutch shunt amp; SCK west -> B.Cu down the open strip x 40.4 to its 33R; LIM_R / ENC_OUT_A drop to
    # B.Cu under the flash, cross the USB pair on F.Cu and join their old routes at U2
    ("CLUTCH_ISENSE", "V", 0.6, [(50.5, 15.1), (50.5, 28.6), (64.0, 28.6), (73.9, 28.6)]),
    ("CLUTCH_ISENSE", "B", 0.2, [(50.5, 15.1), (50.5, 28.6)]),
    ("CLUTCH_ISENSE", "F", 0.2, [(50.5, 28.6), (64.0, 28.6)]),
    ("CLUTCH_ISENSE", "B", 0.2, [(64.0, 28.6), (73.9, 28.6)]),
    ("CLUTCH_ISENSE", "F", 0.2, [(73.9, 28.6), (75.0, 29.7), (75.0, 34.0)]),
    ("SPI_SCK", "V", 0.6, [(46.8, 10.0)]),
    ("SPI_SCK", "B", 0.2, [(46.8, 10.0), (46.4, 9.6), (40.4, 9.6), (40.4, 47.1), (40.8, 47.5), (41.2, 47.5)]),
    ("LIM_R", "V", 0.6, [(68.6, 12.4), (62.6, 29.3), (51.0, 40.9)]),
    ("LIM_R", "B", 0.2, [(68.6, 12.4), (62.6, 18.4), (62.6, 29.3)]),
    ("LIM_R", "F", 0.2, [(62.6, 29.3), (51.0, 40.9)]),
    ("LIM_R", "B", 0.2, [(51.0, 40.9), (45.84, 46.06), (45.84, 47.02)]),
    ("ENC_OUT_A", "V", 0.6, [(68.6, 15.2), (63.4, 29.9), (53.3, 40.0)]),
    ("ENC_OUT_A", "B", 0.2, [(68.6, 15.2), (63.4, 20.4), (63.4, 29.9)]),
    ("ENC_OUT_A", "F", 0.2, [(63.4, 29.9), (53.3, 40.0)]),
    ("ENC_OUT_A", "B", 0.2, [(53.3, 40.0), (53.3, 51.0), (51.17, 53.13)]),
    # QSPI: SD0 / SCLK / SD3 drop straight into the flash's west pins, SS / SD1 / SD2 run over the top and down its
    # east side; flash VCC -> C51
    ("QSPI_SS", "F", 0.2, [(62.6, 16.88), (62.6, 17.3), (75.4, 17.3), (75.4, 23.905), (73.2, 23.905)]),
    ("QSPI_SD1", "F", 0.2, [(62.2, 16.88), (62.2, 17.7), (75.0, 17.7), (75.0, 22.635), (73.2, 22.635)]),
    ("QSPI_SD2", "F", 0.2, [(61.8, 16.88), (61.8, 18.1), (74.6, 18.1), (74.6, 21.365), (73.2, 21.365)]),
    ("QSPI_SD0", "F", 0.2, [(61.4, 16.88), (61.4, 18.5), (64.4, 18.5), (64.4, 20.095), (64.8, 20.095)]),
    ("QSPI_SCLK", "F", 0.2, [(61.0, 16.88), (61.0, 18.9), (64.0, 18.9), (64.0, 21.365), (64.8, 21.365)]),
    ("QSPI_SD3", "F", 0.2, [(60.6, 16.88), (60.6, 19.3), (63.6, 19.3), (63.6, 22.635), (64.8, 22.635)]),
    ("+3V3", "F", 0.3, [(64.8, 23.905), (63.6, 23.905)]),
    ("QSPI_SS", "F", 0.2, [(75.4, 20.4), (75.975, 20.4)]),
    ("GND", "F", 0.3, [(66.675, 16.3), (68.5, 16.6)]),  # C29 GND, boxed in by the escapes
    ("GND", "V", 0.6, [(68.5, 16.6)]),
    # a GND via for the F.Cu pour island between the moved filter column and J6
    ("GND", "V", 0.6, [(92.4, 74.4)]),
    # USB pair, 0.2 / 0.2 mm, D- west / D+ east: down from the 27R, B.Cu diagonal to the corridor between the clutch
    # driver and J7, down to y 88-91, F.Cu east under J6 to the USB-C (D- pads joined in front of the pad row,
    # D+ pads behind it)
    ("USB_CONN_DM", "F", 0.2, [(56.2, 22.075), (56.2, 23.3)]),
    ("USB_CONN_DM", "V", 0.6, [(56.2, 23.3), (71.402, 91.546)]),
    ("USB_CONN_DM", "B", 0.2, [(56.2, 23.3), (56.2, 28.5), (69.76, 42.06), (69.76, 45.593), (70.72, 46.553),
                               (70.72, 72.539), (71.802, 73.621), (71.802, 88.0), (71.402, 88.4), (71.402, 91.546)]),
    ("USB_CONN_DM", "F", 0.2, [(71.402, 91.546), (79.979, 91.546), (83.041, 94.608), (85.422, 94.608),
                               (86.85, 93.18), (86.85, 92.305)]),
    ("USB_CONN_DM", "F", 0.2, [(86.85, 93.18), (86.95, 93.28)]),
    ("USB_CONN_DM", "F", 0.15, [(86.95, 93.28), (87.85, 93.28)]),
    ("USB_CONN_DM", "F", 0.2, [(87.85, 93.28), (87.85, 92.305)]),
    ("USB_CONN_DP", "F", 0.2, [(57.7, 22.075), (57.7, 23.3)]),
    ("USB_CONN_DP", "V", 0.6, [(57.7, 23.3), (72.202, 88.75), (86.284, 90.614), (87.909, 91.049)]),
    ("USB_CONN_DP", "B", 0.2, [(57.7, 23.3), (57.7, 29.434), (70.16, 41.894), (70.16, 45.427), (71.12, 46.387),
                               (71.12, 72.373), (72.202, 73.455), (72.202, 88.75)]),
    ("USB_CONN_DP", "F", 0.2, [(72.202, 88.75), (83.888, 88.75), (84.808, 89.67), (85.763, 89.67),
                               (86.284, 90.191), (86.284, 90.614)]),
    ("USB_CONN_DP", "B", 0.2, [(86.284, 90.614), (86.718, 91.049), (87.909, 91.049)]),
    ("USB_CONN_DP", "F", 0.2, [(87.909, 91.049), (87.909, 91.328)]),
    ("USB_CONN_DP", "F", 0.2, [(87.35, 92.305), (87.35, 91.328), (88.35, 91.328), (88.35, 92.305)]),
    # +5V: the buck trunk (F.Cu along y 11.3) used to run through the Pico area; it now stops at x 45 (trunk_end),
    # drops to B.Cu, comes back up south of the chip and feeds the U7 / U8 branch and D5 (+5V -> VSYS)
    ("+5V", "V", 0.8, [(45.0, 11.304), (45.0, 27.6)]),
    ("+5V", "B", 0.5, [(45.0, 11.304), (45.0, 27.6)]),
    ("+5V", "F", 0.5, [(45.0, 27.6), (65.3, 27.6), (66.387, 28.681), (67.35, 28.7)]),
]


def short_net(board, name):
    for full in (name, MCU + name):
        if board.FindNet(full) is not None:
            return board.FindNet(full)
    hits = [n for n in board.GetNetsByName().values() if str(n.GetNetname()).endswith("/" + name)
            and not str(n.GetNetname()).startswith("/Pico")]
    if len(hits) != 1:
        raise KeyError(name)
    return hits[0]


def preroute(board):
    """Dev: draw PREROUTE (locked) and drop the GND stitching vias it lands on."""
    n = 0
    new = []
    for net, layer, w, pts in PREROUTE:
        ni = short_net(board, net)
        if layer == "V":
            for x, y in pts:
                v = pcbnew.PCB_VIA(board)
                v.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
                v.SetWidth(mm(w))
                v.SetDrill(mm(0.4 if w >= 0.8 else 0.3 if w >= 0.6 else 0.25))
                v.SetNet(ni)
                v.SetLocked(True)
                board.Add(v)
                new.append(v)
                n += 1
            continue
        for a, b in zip(pts, pts[1:]):
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(pcbnew.VECTOR2I(mm(a[0]), mm(a[1])))
            t.SetEnd(pcbnew.VECTOR2I(mm(b[0]), mm(b[1])))
            t.SetWidth(mm(w))
            t.SetLayer(F if layer == "F" else B)
            t.SetNet(ni)
            t.SetLocked(True)
            board.Add(t)
            new.append(t)
            n += 1
    ends = {(p.x, p.y) for t in board.GetTracks() if t.Type() != pcbnew.PCB_VIA_T and t.GetNetname() == "GND"
            for p in points(t)}
    mine = {t.m_Uuid.AsString() for t in new}
    for v in [t for t in board.GetTracks() if t.Type() == pcbnew.PCB_VIA_T and t.GetNetname() == "GND"
              and (t.GetPosition().x, t.GetPosition().y) not in ends and t.m_Uuid.AsString() not in mine]:
        vs = v.GetEffectiveShape(F)
        for t in new:
            if t.GetNetname() == "GND":
                continue
            if t.Type() == pcbnew.PCB_VIA_T:
                hit = (t.GetPosition() - v.GetPosition()).EuclideanNorm() < (t.GetWidth(F) + v.GetWidth(F)) // 2 + mm(0.25)
            else:
                hit = t.GetEffectiveShape().Collide(vs, mm(0.25 if "+5V" not in t.GetNetname() else 0.3))
            if hit:
                board.Delete(v)
                break
    return n


# ----------------------------------------------------------------------------------------------- netlist

def load_netlist():
    """{ref: {value, footprint, fields, path, sheetname, sheetfile, pins: {pad: net}}} from the schematic."""
    out = os.path.join(tempfile.mkdtemp(), "net.xml")
    subprocess.run([KCLI, "sch", "export", "netlist", "--format", "kicadxml", "-o", out, SCH], check=True,
                   capture_output=True)
    root = ET.parse(out).getroot()
    comps = {}
    for c in root.find("components"):
        props = {p.get("name"): p.get("value") for p in c.findall("property")}
        fields = {f.get("name"): (f.text or "") for f in c.find("fields")} if c.find("fields") is not None else {}
        comps[c.get("ref")] = {
            "value": c.findtext("value"), "footprint": c.findtext("footprint"), "fields": fields,
            "path": c.find("sheetpath").get("tstamps") + c.findtext("tstamps"),
            "sheetname": props.get("Sheetname", ""), "sheetfile": props.get("Sheetfile", ""),
            "dnp": "dnp" in props, "pins": {}}
    for n in root.find("nets"):
        for nd in n.findall("node"):
            if nd.get("ref") in comps:
                name = n.get("name")
                if name.startswith("unconnected-"):  # KiCad stores the "/" of a no-connect net name as {slash}
                    name = name.replace("/", "{slash}")
                comps[nd.get("ref")]["pins"][nd.get("pin")] = name
    return comps


def get_net(board, name):
    n = board.FindNet(name)
    if n is None:
        n = pcbnew.NETINFO_ITEM(board, name)
        board.Add(n)
    return n


def renumber(board):
    """Old refs -> schematic refs, in two passes so that no two footprints ever share a reference."""
    if board.FindFootprintByReference("A1") is None:
        return 0
    fps = {f.GetReference(): f for f in board.GetFootprints()}
    moving = [(fps[o], n) for o, n in OLD2NEW.items() if o in fps]
    for fp, _ in moving:
        fp.SetReference("_tmp_" + fp.GetReference())
    for fp, n in moving:
        fp.SetReference(n)
    return len(moving)


def rename_nets(board):
    """The MCU sheet was renamed ("Pico, ..." -> "RP2040, ..."): move tracks, vias and zones to the new net names."""
    old = "/Pico, SPI, E-stop interlock/"
    n = 0
    for item in list(board.GetTracks()) + list(board.Zones()):
        name = item.GetNetname()
        if name.startswith(old):
            item.SetNet(get_net(board, MCU + name[len(old):]))
            n += 1
    return n


def sync(board, comps):
    """Values, fields, paths and pad nets from the schematic; new footprints from the libraries; A1 out."""
    added = 0
    a1 = board.FindFootprintByReference("A1")
    if a1 is not None:
        board.Delete(a1)
    for ref, c in sorted(comps.items()):
        fp = board.FindFootprintByReference(ref)
        if fp is None:
            lib, name = c["footprint"].split(":")
            fp = pcbnew.FootprintLoad(os.path.join(FPLIB, lib + ".pretty"), name)
            fp.SetFPID(pcbnew.LIB_ID(lib, name))
            fp.SetReference(ref)
            board.Add(fp)
            x, y, rot = PLACE.get(ref, (50.0, 50.0, 0))
            fp.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
            fp.SetOrientationDegrees(rot)
            added += 1
        fp.SetValue(c["value"])
        for k, v in c["fields"].items():
            if k in ("Footprint", "Datasheet", "Description", "Value", "Reference"):
                continue
            if fp.HasFieldByName(k):
                fp.GetFieldByName(k).SetText(v)
            else:
                f = pcbnew.PCB_FIELD(fp, fp.GetNextFieldId(), k)
                f.SetText(v)
                f.SetVisible(False)
                f.SetLayer(pcbnew.F_Fab)
                fp.AddField(f)
        fp.SetPath(pcbnew.KIID_PATH(c["path"]))
        fp.SetSheetname(c["sheetname"])
        fp.SetSheetfile(c["sheetfile"])
        for p in fp.Pads():
            name = c["pins"].get(p.GetNumber())
            if name is None:
                if p.GetNumber():
                    p.SetNetCode(0)
                continue
            if p.GetNetname() != name:
                p.SetNet(get_net(board, name))
    return added


# ----------------------------------------------------------------------------------------------- placement

def place(board):
    n = 0
    # exposed pad + its thermal vias: solid to the GND pours (no relief spokes needed or possible there)
    board.FindFootprintByReference("U3").SetLocalZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
    for ref, (x, y, rot) in PLACE.items():
        fp = board.FindFootprintByReference(ref)
        if fp.IsFlipped():
            fp.Flip(fp.GetPosition(), False)
        if fp.GetPosition() != pcbnew.VECTOR2I(mm(x), mm(y)) or abs(fp.GetOrientationDegrees() - rot % 360) > 0.01 \
                and abs(fp.GetOrientationDegrees() - (rot % 360 - 360)) > 0.01:
            fp.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
            fp.SetOrientationDegrees(rot)
            n += 1
    return n


# ----------------------------------------------------------------------------------------------- copper ownership

def owned_nets(comps):
    """(full, partial): nets redrawn entirely, and RP2040 nets that also reach parts outside the rework."""
    placed = set(PLACE)
    users = {}
    for ref, c in comps.items():
        for net in c["pins"].values():
            users.setdefault(net, set()).add(ref)
    partial = {n for n in comps["U3"]["pins"].values() if n not in SHARED and users[n] - placed}
    # RP2040 nets Freerouting could not finish from the cut ends of their old routes: redrawn end to end
    partial -= {n for n in partial if n.split("/")[-1] in REDRAW}
    full = {n for r in placed for n in comps[r]["pins"].values()} - SHARED - partial
    return full, partial


def inside(p, boxes=BOXES):
    x, y = p.x / 1e6, p.y / 1e6
    return any(x0 <= x <= x1 and y0 <= y <= y1 for x0, y0, x1, y1 in boxes)


def points(t):
    return [t.GetPosition()] if t.Type() == pcbnew.PCB_VIA_T else [t.GetStart(), t.GetEnd()]


def is_owned(t, full, partial):
    n = t.GetNetname()
    if n in full or n in partial:
        return True
    return n in SHARED and all(inside(p) for p in points(t))


def clear_owned(board, full, partial):
    dead = [t for t in board.GetTracks() if is_owned(t, full, partial)]
    for t in dead:
        board.Delete(t)
    return len(dead)


def rip_up(board, full, partial):
    """Dev: what Freerouting gets to redo. Full nets and the in-box part of the partial nets go, as do shared
    tracks inside the boxes and GND stitching vias under the new footprints; stitching vias elsewhere stay."""
    top = BOXES[:1]
    keep_out = []
    for fp in board.GetFootprints():
        if fp.GetReference() in PLACE:
            cb = fp.GetCourtyard(F).BBox()
            bb = pcbnew.BOX2I(cb.GetPosition(), cb.GetSize())
            bb.Inflate(mm(0.6))
            keep_out.append(bb)
    ends = {(p.x, p.y) for t in board.GetTracks() if t.Type() != pcbnew.PCB_VIA_T and t.GetNetname() == "GND"
            for p in points(t)}
    dead = []
    for t in board.GetTracks():
        n = t.GetNetname()
        if n in full or (n in partial and any(inside(p, top) for p in points(t))):
            dead.append(t)
        elif n in SHARED and t.Type() != pcbnew.PCB_VIA_T and all(inside(p) for p in points(t)):
            dead.append(t)
        elif n in SHARED and t.Type() == pcbnew.PCB_VIA_T and inside(t.GetPosition()):
            p = t.GetPosition()
            if any(bb.Contains(p) for bb in keep_out) or (n != "GND" or (p.x, p.y) in ends):
                dead.append(t)
    for t in dead:
        board.Delete(t)
    return len(dead)


def dump(board, full, partial, path=ROUTES):
    items = []
    for t in board.GetTracks():
        if not is_owned(t, full, partial):
            continue
        if t.Type() == pcbnew.PCB_VIA_T:
            items.append(["via", t.GetNetname(), t.GetPosition().x, t.GetPosition().y, t.GetWidth(F), t.GetDrillValue()])
        else:
            items.append(["track", t.GetNetname(), t.GetLayerName(), t.GetStart().x, t.GetStart().y, t.GetEnd().x,
                          t.GetEnd().y, t.GetWidth()])
    items.sort(key=lambda i: (i[0], i[1], i[2:]))
    with open(path, "w") as f:
        f.write("[\n" + ",\n".join(json.dumps(i) for i in items) + "\n]\n")
    return len(items)


def apply_routes(board, path=ROUTES):
    if not os.path.exists(path):
        return []
    items = json.load(open(path))
    added = []
    for it in items:
        if it[0] == "via":
            _, net, x, y, d, drill = it
            t = pcbnew.PCB_VIA(board)
            t.SetPosition(pcbnew.VECTOR2I(x, y))
            t.SetWidth(d)
            t.SetDrill(drill)
        else:
            _, net, layer, x0, y0, x1, y1, w = it
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(pcbnew.VECTOR2I(x0, y0))
            t.SetEnd(pcbnew.VECTOR2I(x1, y1))
            t.SetWidth(w)
            t.SetLayer(board.GetLayerID(layer))
        t.SetNet(get_net(board, net))
        board.Add(t)
        added.append(t)
    return added


def drop_stitch_vias(board, items, clearance=0.2):
    """GND stitching vias (no track attached) that the redrawn copper runs over: delete them (the routes were
    made with these vias already removed, some of them outside the boxes)."""
    ends = {(p.x, p.y) for t in board.GetTracks() if t.Type() != pcbnew.PCB_VIA_T and t.GetNetname() == "GND"
            for p in points(t)}
    mine = {t.m_Uuid.AsString() for t in items}
    n = 0
    for v in [t for t in board.GetTracks() if t.Type() == pcbnew.PCB_VIA_T and t.GetNetname() == "GND"
              and t.m_Uuid.AsString() not in mine and (t.GetPosition().x, t.GetPosition().y) not in ends]:
        vs = v.GetEffectiveShape(F)
        for t in items:
            if t.GetNetname() == "GND":
                continue
            if t.Type() == pcbnew.PCB_VIA_T:
                hit = (t.GetPosition() - v.GetPosition()).EuclideanNorm() < \
                    (t.GetWidth(F) + v.GetWidth(F)) // 2 + mm(clearance)
            else:
                hit = t.GetEffectiveShape().Collide(vs, mm(clearance))
            if hit:
                board.Delete(v)
                n += 1
                break
    return n


# ----------------------------------------------------------------------------------------------- dev: fan-out, autoroute

def fanout(board, clearance=0.2):
    """Dev: a 0.6/0.3 via (GND -> In1, +3V3 -> In2) beside every GND / +3V3 SMD pad of the placed parts (and of
    the other parts in the boxes whose pad lost its track), on the
    first free spot outward from the part, joined by a 0.3 mm stub. (The outer GND pours count as connected for
    Freerouting, so it would not drop these vias itself.)"""
    import math
    fps = [fp for fp in board.GetFootprints()
           if fp.GetReference() in PLACE or any(inside(p.GetPosition()) for p in fp.Pads())]
    tracked = {}
    for t in board.GetTracks():
        if t.Type() != pcbnew.PCB_VIA_T:
            for q in points(t):
                tracked.setdefault(t.GetNetname(), []).append(q)
    obst_f, obst_b = [], []
    for fp in board.GetFootprints():
        for p in fp.Pads():
            for lay, lst in ((F, obst_f), (B, obst_b)):
                if p.IsOnLayer(lay):
                    lst.append((p.GetNetname(), p.GetEffectiveShape(lay)))
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_VIA_T:
            obst_f.append((t.GetNetname(), t.GetEffectiveShape(F)))
            obst_b.append((t.GetNetname(), t.GetEffectiveShape(B)))
        else:
            (obst_f if t.GetLayer() == F else obst_b).append((t.GetNetname(), t.GetEffectiveShape()))
    yards = [(fp.GetReference(), fp.GetCourtyard(F)) for fp in board.GetFootprints()]
    edge = board.GetBoardEdgesBoundingBox()
    n = 0
    for fp in fps:
        c = fp.GetPosition()
        for p in fp.Pads():
            net = p.GetNetname()
            if net not in ("GND", "+3V3") or p.GetAttribute() != pcbnew.PAD_ATTRIB_SMD or \
                    (fp.GetReference() == "U3"):
                continue
            pc = p.GetPosition()
            if fp.GetReference() not in PLACE and any(p.HitTest(q) for q in tracked.get(net, [])):
                continue
            if any(nn == net and t.Type() == pcbnew.PCB_VIA_T and (t.GetPosition() - pc).EuclideanNorm() < mm(1.3)
                   for nn, t in ((t.GetNetname(), t) for t in board.GetTracks())):
                continue
            base = math.atan2(pc.y - c.y, pc.x - c.x) if pc != c else 0.0
            half = max(p.GetSize().x, p.GetSize().y) / 2e6
            done = False
            for dist in (half + 0.45, half + 0.65, half + 0.9, half + 1.2):
                for da in (0, 30, -30, 60, -60, 90, -90, 120, -120, 180):
                    a = base + math.radians(da)
                    v = pcbnew.VECTOR2I(int(pc.x + mm(dist) * math.cos(a)), int(pc.y + mm(dist) * math.sin(a)))
                    if not edge.Contains(v) or min(v.x - edge.GetLeft(), edge.GetRight() - v.x, v.y - edge.GetTop(),
                                                   edge.GetBottom() - v.y) < mm(0.8):
                        continue
                    circ = pcbnew.SHAPE_CIRCLE(v, mm(0.3))
                    seg = pcbnew.SHAPE_SEGMENT(pc, v, mm(0.3))
                    if any(nn != net and (s.Collide(circ, mm(clearance)) or s.Collide(seg, mm(clearance)))
                           for nn, s in obst_f):
                        continue
                    if any(nn != net and s.Collide(circ, mm(clearance)) for nn, s in obst_b):
                        continue
                    if any(r != fp.GetReference() and y.Contains(v) for r, y in yards):
                        continue
                    via = pcbnew.PCB_VIA(board)
                    via.SetPosition(v)
                    via.SetWidth(mm(0.6))
                    via.SetDrill(mm(0.3))
                    via.SetNet(p.GetNet())
                    board.Add(via)
                    tr = pcbnew.PCB_TRACK(board)
                    tr.SetStart(pc)
                    tr.SetEnd(v)
                    tr.SetWidth(mm(0.3))
                    tr.SetLayer(F)
                    tr.SetNet(p.GetNet())
                    board.Add(tr)
                    obst_f += [(net, via.GetEffectiveShape(F)), (net, tr.GetEffectiveShape())]
                    obst_b.append((net, via.GetEffectiveShape(B)))
                    n += 1
                    done = True
                    break
                if done:
                    break
            if not done:
                print("no fan-out spot for %s pad %s" % (fp.GetReference(), p.GetNumber()))
    return n


def autoroute(board, full, partial):
    """Dev: Freerouting on a copy (existing copper locked, 0.5 mm clearance class for the 36 V nets); the owned
    copper of the result is written to rp2040_routes.json."""
    tmp = tempfile.mkdtemp()
    src = os.path.join(tmp, "in.kicad_pcb")
    board.Save(src)
    b = pcbnew.LoadBoard(src)
    for t in b.GetTracks():
        t.SetLocked(True)
    dsn, ses = os.path.join(tmp, "r.dsn"), os.path.join(tmp, "r.ses")
    if not pcbnew.ExportSpecctraDSN(b, dsn):
        raise RuntimeError("DSN export failed")
    # 0.5 mm for the 36 V / power classes, in the DSN only (editing the netclasses would change the project file)
    import re
    parts = open(dsn).read().split("(class ")
    for i in range(1, len(parts)):
        if parts[i].startswith(("HV", "Power")):
            parts[i] = re.sub(r"\(clearance [\d.]+\)", "(clearance 500)", parts[i], count=1)
    open(dsn, "w").write("(class ".join(parts))
    if os.environ.get("RP_SES"):          # dev: reuse a session file from an earlier run of the same DSN
        ses = os.environ["RP_SES"]
    else:
        # headless: with the GUI enabled Freerouting 2.4 exits before its save thread has written the session
        subprocess.run([JAVA, "-Djava.awt.headless=true", "-jar", FREEROUTING, "--gui.enabled=false", "-de", dsn,
                        "-do", ses, "-mp", "12", "-mt", "1"], check=True, stdout=subprocess.DEVNULL)
    if os.environ.get("RP_DSN_COPY"):
        import shutil
        shutil.copy(dsn, os.environ["RP_DSN_COPY"])
    if not pcbnew.ImportSpecctraSES(b, ses):
        raise RuntimeError("SES import failed")
    return dump(b, full, partial)


# ----------------------------------------------------------------------------------------------- main

def trunk_end(board):
    """The +5V trunk from the buck ran on to x 52.45 (old Pico VSYS); it now ends at x 45, where PREROUTE drops it
    to B.Cu. The segment is not owned by rp2040_routes.json, so both flows shorten it here."""
    for t in board.GetTracks():
        if t.Type() == pcbnew.PCB_TRACE_T and t.GetNetname() == "+5V" and t.GetLayer() == F:
            for get, put in ((t.GetStart, t.SetStart), (t.GetEnd, t.SetEnd)):
                p = get()
                if abs(p.x - mm(52.45)) < mm(0.05) and abs(p.y - mm(11.304)) < mm(0.05):
                    put(pcbnew.VECTOR2I(mm(45.0), p.y))
                    return 1
    return 0


def tie_testen(board):
    """TESTEN (pin 19) to the exposed pad, inward on F.Cu."""
    p = [q for q in board.FindFootprintByReference("U3").Pads() if q.GetNumber() == "19"][0].GetPosition()
    t = pcbnew.PCB_TRACK(board)
    t.SetStart(p)
    t.SetEnd(pcbnew.VECTOR2I(p.x, p.y + mm(1.9)))
    t.SetWidth(mm(0.2))
    t.SetLayer(F)
    t.SetNet(board.FindNet("GND"))
    board.Add(t)


def main():
    board = pcbnew.LoadBoard(PCB)
    comps = load_netlist()
    print("renumbered", renumber(board))
    print("nets renamed", rename_nets(board))
    print("new footprints", sync(board, comps))
    print("placed", place(board), "+5V trunk shortened", trunk_end(board))
    full, partial = owned_nets(comps)
    if "--autoroute" in sys.argv or "--place-only" in sys.argv:
        print("ripped up", rip_up(board, full, partial))
        tie_testen(board)
        print("pre-routed", preroute(board))
        print("fan-out vias", fanout(board))
        if "--autoroute" in sys.argv:
            print("routed items", autoroute(board, full, partial))
            clear_owned(board, full, partial)
            added = apply_routes(board)
            print("applied", len(added), "stitching vias dropped", drop_stitch_vias(board, added))
    elif "--dump" in sys.argv:
        print("dumped", dump(board, full, partial))
        return
    else:
        print("cleared", clear_owned(board, full, partial))
        added = apply_routes(board)
        print("applied", len(added), "stitching vias dropped", drop_stitch_vias(board, added))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(PCB)
    print("saved", PCB)


if __name__ == "__main__":
    main()
