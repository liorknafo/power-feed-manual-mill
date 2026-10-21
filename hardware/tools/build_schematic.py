"""Power-feed controller board: circuit description -> KiCad schematic.

Run:  python3 hardware/tools/build_schematic.py
Then: python3 hardware/tools/check_schematic.py
Edit this file, not the generated .kicad_sch files.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
import kicad_gen  # noqa: E402
from kicad_gen import Part, Project, custom_symbol  # noqa: E402

OUT = os.path.join(os.path.dirname(__file__), "..", "kicad")
kicad_gen.PROJECT_FP_DIRS["power_feed"] = os.path.join(OUT, "power_feed.pretty")

# ------------------------------------------------------------------ footprints
R0402 = "Resistor_SMD:R_0402_1005Metric"
R0603 = "Resistor_SMD:R_0603_1608Metric"
R1206 = "Resistor_SMD:R_1206_3216Metric"
R2512 = "Resistor_SMD:R_2512_6332Metric"
C0603 = "Capacitor_SMD:C_0603_1608Metric"
C0805 = "Capacitor_SMD:C_0805_2012Metric"
C1206 = "Capacitor_SMD:C_1206_3216Metric"
C1210 = "Capacitor_SMD:C_1210_3225Metric"
CP_BULK = "Capacitor_THT:CP_Radial_D12.5mm_P5.00mm"
SOD123 = "Diode_SMD:D_SOD-123"
SMA = "Diode_SMD:D_SMA"
SMB = "Diode_SMD:D_SMB"
SOT23 = "Package_TO_SOT_SMD:SOT-23"
SOT23_5 = "Package_TO_SOT_SMD:SOT-23-5"
SOT23_6 = "Package_TO_SOT_SMD:SOT-23-6"
DPAK = "Package_TO_SOT_SMD:TO-252-2"
LED0603 = "LED_SMD:LED_0603_1608Metric"
TQFP48 = "Package_QFP:TQFP-48-1EP_7x7mm_P0.5mm_EP5x5mm_ThermalVias"
QFN56 = "Package_DFN_QFN:QFN-56-1EP_7x7mm_P0.4mm_EP3.2x3.2mm_ThermalVias"
SOIC8_208 = "Package_SO:SOIC-8_5.3x5.3mm_P1.27mm"
XTAL3225 = "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm"
USB_C = "Connector_USB:USB_C_Receptacle_HRO_TYPE-C-31-M-12"
HDR3 = "Connector_PinHeader_2.54mm:PinHeader_1x03_P2.54mm_Vertical"
TB2 = "Connector_Phoenix_MSTB:PhoenixContact_MSTBA_2,5_2-G-5,08_1x02_P5.08mm_Horizontal"
TB4 = "Connector_Phoenix_MSTB:PhoenixContact_MSTBA_2,5_4-G-5,08_1x04_P5.08mm_Horizontal"
XH = "Connector_JST:JST_XH_B%dB-XH-A_1x0%d_P2.50mm_Vertical"
IDC_2X7 = "Connector_IDC:IDC-Header_2x07_P2.54mm_Vertical"
FUSE_5X20 = "power_feed:Fuseholder_XC_BLX-A_5x20mm_P22.00mm"
PTC = "Fuse:Fuse_2920_7451Metric"
SOT223 = "Package_TO_SOT_SMD:SOT-223-3_TabPin2"
SOIC8 = "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm"
BUTTON = "Button_Switch_SMD:SW_SPST_B3S-1000"
L_BUCK = "Inductor_SMD:L_TDK_VLS6045EX_VLS6045AF"


def P(mpn, lcsc=""):
    """Ordering properties. An empty LCSC number means 'not yet verified at LCSC'."""
    return {"MPN": mpn, "LCSC": lcsc}


# Generic passives: (value, footprint) -> (MPN, LCSC). R() / C() look the part up here unless the call passes its
# own props=. Every entry was checked against the JLCPCB parts library (spec, stock) on 2026-10-20;
# "B" = JLCPCB Basic part (no extra feeder fee), "E" = Extended. All resistors are 1 % thick film; all capacitors
# are X7R/X5R (C0G for 100 pF) with a voltage rating at or above the value's marking.
RES_PARTS = {
    ("10k", R0603): ("UNI-ROYAL 0603WAF1002T5E", "C25804"),      # B
    ("1k", R0603): ("UNI-ROYAL 0603WAF1001T5E", "C21190"),       # B
    ("100k", R0603): ("UNI-ROYAL 0603WAF1003T5E", "C25803"),     # B
    ("2.2k", R0603): ("UNI-ROYAL 0603WAF2201T5E", "C4190"),      # B
    ("4.7k", R0603): ("UNI-ROYAL 0603WAF4701T5E", "C23162"),     # B
    ("22k", R0603): ("UNI-ROYAL 0603WAF2202T5E", "C31850"),      # B
    ("33", R0603): ("UNI-ROYAL 0603WAF330JT5E", "C23140"),       # B
    ("47", R0603): ("UNI-ROYAL 0603WAF470JT5E", "C23182"),       # B
    ("100", R0603): ("UNI-ROYAL 0603WAF1000T5E", "C22775"),      # B
    ("330", R0603): ("UNI-ROYAL 0603WAF3300T5E", "C23138"),      # B
    ("2.2", R0603): ("UNI-ROYAL 0603WAF220KT5E", "C22939"),      # B
    ("7.5k", R0603): ("UNI-ROYAL 0603WAF7501T5E", "C23234"),     # B
    ("200k", R0603): ("UNI-ROYAL 0603WAF2003T5E", "C25811"),     # B
    ("41.2k", R0603): ("FOJAN FRC0603F4122TS", "C2930101"),      # E (no Basic E96 value)
    ("31.6k", R0603): ("UNI-ROYAL 0603WAF3162T5E", "C25967"),    # E (no Basic E96 value)
    ("4.7", R0402): ("UNI-ROYAL 0402WGF470KTCE", "C25121"),      # E (no Basic 0402 value)
    ("4.7k", R1206): ("UNI-ROYAL 1206W4F4701T5E", "C17936"),     # B, 250 mW
    ("27", R0603): ("UNI-ROYAL 0603WAF270JT5E", "C25190"),       # B   (RP2040 USB series)
    ("5.1k", R0603): ("UNI-ROYAL 0603WAF5101T5E", "C23186"),     # B   (USB-C CC pull-downs)
}
CAP_PARTS = {
    ("2.2nF 50V", C0603): ("FH 0603B222K500NT", "C1604"),                    # B, 50 V X7R
    ("3.3nF", C0603): ("Samsung CL10B332KB8NNNC", "C1613"),                  # B, 50 V X7R
    ("22nF 100V", C0805): ("Yageo CC0805KRX7R0BB223", "C107137"),            # E, 100 V X7R
    ("4.7uF 10V", C0805): ("Samsung CL21A475KAQNNNE", "C1779"),              # B, 25 V X5R
    ("470nF", C0603): ("Samsung CL10B474KA8NNNC", "C1623"),                  # B, 25 V X7R
    ("10uF 25V", C1206): ("Samsung CL31A106KBHNNNE", "C13585"),              # B, 50 V X5R
    ("1nF", C0603): ("Samsung CL10B102KB8NNNC", "C1588"),                    # B, 50 V X7R
    ("10nF", C0603): ("FH 0603B103K500NT", "C57112"),                        # B, 50 V X7R
    ("100nF 100V", C0805): ("Samsung CL21B104KCFNNNE", "C28233"),            # B, 100 V X7R
    ("100nF", C0603): ("Yageo CC0603KRX7R9BB104", "C14663"),                 # B, 50 V X7R
    ("220nF 50V", C0603): ("Samsung CL10B224KB8NNNC", "C64705"),             # E, 50 V X7R
    ("2.2uF 100V", C1210): ("Murata GRM32ER72A225KA35L", "C86054"),          # E, 100 V X7R
    ("100pF", C0603): ("Samsung CL10C101JB8NNNC", "C14858"),                 # B, 50 V C0G
    ("22uF 10V", C1206): ("Samsung CL31A226KAHNNNE", "C12891"),              # B, 25 V X5R
    ("1uF", C0603): ("Samsung CL10A105KB8NNNC", "C15849"),                   # B, 50 V X5R (RP2040 VREG, LDO)
    ("15pF", C0603): ("Samsung CL10C150JB8NNNC", "C1644"),                   # B, 50 V C0G (12 MHz crystal)
}

GND, V3, V5, VM = "GND", "+3V3", "+5V", "+36V"


def _stock_props(table, kind, val, fp):
    """Ordering properties for a generic passive; fails loudly so no part silently ends up without an LCSC number."""
    try:
        return P(*table[(val, fp)])
    except KeyError:
        raise KeyError("no LCSC part for %s %r in %s: add it to the parts table or pass props=" % (kind, val, fp)) from None


def R(val, a, b, fp=R0603, **kw):
    if "props" not in kw:
        kw["props"] = _stock_props(RES_PARTS, "resistor", val, fp)
    return Part("R", "Device:R", val, fp, {"1": a, "2": b}, **kw)


def C(val, a, b, fp=C0603, **kw):
    if "props" not in kw:
        kw["props"] = _stock_props(CAP_PARTS, "capacitor", val, fp)
    return Part("C", "Device:C", val, fp, {"1": a, "2": b}, **kw)


def CP(val, plus, minus, fp=CP_BULK, **kw):
    return Part("C", "Device:C_Polarized", val, fp, {"1": plus, "2": minus}, **kw)


def D(lib_id, val, k, a, fp, **kw):
    return Part("D", lib_id, val, fp, {"1": k, "2": a}, **kw)


XH_LCSC = {3: "C144394", 4: "C144395", 5: "C157991", 7: "C144398"}


def XH_CONN(n, nets, props=None, **kw):
    pr = {"MPN": "JST B%dB-XH-A(LF)(SN)" % n, "LCSC": XH_LCSC.get(n, "")}
    pr.update(props or {})
    return Part("J", "Connector_Generic:Conn_01x%02d" % n, "JST-XH %dp" % n, XH % (n, n),
                {str(i + 1): net for i, net in enumerate(nets)}, props=pr, **kw)


# ------------------------------------------------------------------ TMC5160 symbol
TMC_PINS = [
    # left: supplies, charge pump, SPI, mode/control, encoder
    (33, "VS", "power_in", "L"), (4, "VSA", "power_in", "L"), (29, "VCC", "passive", "L"),
    (20, "VCC_IO", "power_in", "L"), (5, "5VOUT", "power_out", "L"), (3, "12VOUT", "power_out", "L"),
    (34, "VCP", "passive", "L"), (32, "CPI", "passive", "L"), (31, "CPO", "passive", "L"),
    (None, "", "", "L"),
    (13, "CSN", "input", "L"), (14, "SCK", "input", "L"), (15, "SDI", "input", "L"), (16, "SDO", "tri_state", "L"),
    (None, "", "", "L"),
    (21, "SD_MODE", "input", "L"), (22, "SPI_MODE", "input", "L"), (12, "CLK", "input", "L"),
    (11, "TST_MODE", "input", "L"), (28, "DRV_ENN", "input", "L"),
    (26, "DIAG0", "open_collector", "L"), (27, "DIAG1", "open_collector", "L"),
    (17, "REFL", "input", "L"), (18, "REFR", "input", "L"),
    (24, "ENCA", "input", "L"), (23, "ENCB", "input", "L"), (25, "ENCN", "input", "L"),
    # right: bridge A, bridge B
    (41, "HA1", "output", "R"), (42, "CA1", "passive", "R"), (40, "BMA1", "passive", "R"), (39, "LA1", "output", "R"),
    (36, "HA2", "output", "R"), (35, "CA2", "passive", "R"), (37, "BMA2", "passive", "R"), (38, "LA2", "output", "R"),
    (8, "SRAH", "input", "R"), (7, "SRAL", "input", "R"),
    (None, "", "", "R"),
    (1, "HB1", "output", "R"), (2, "CB1", "passive", "R"), (48, "BMB1", "passive", "R"), (47, "LB1", "output", "R"),
    (44, "HB2", "output", "R"), (43, "CB2", "passive", "R"), (45, "BMB2", "passive", "R"), (46, "LB2", "output", "R"),
    (9, "SRBH", "input", "R"), (10, "SRBL", "input", "R"),
    # bottom: grounds
    (6, "GNDA", "power_in", "B"), (19, "GNDD", "power_in", "B"), (30, "GNDD", "power_in", "B"), (49, "EP", "passive", "B"),
]
TMC_SYM = custom_symbol("power_feed:TMC5160A-TA", TMC_PINS, ref="U", value="TMC5160A-TA", footprint=TQFP48,
                        datasheet="https://www.analog.com/media/en/technical-documentation/data-sheets/TMC5160A_datasheet_rev1.18.pdf",
                        description="Stepper controller/driver, SPI, external MOSFETs, TQFP-48-EP")


def build():
    pr = Project("power-feed", "Mill power feed controller")

    # ================================================================ power
    s = pr.sheet("power", "power.kicad_sch", "Power input, protection, 5 V buck")
    s.note("36 V from the S-350-36 PSU. Mains AC never comes onto this board.\n"
           "Q1 = reverse-polarity protection (P-FET, gate clamped by D1). D2 = bus TVS.\n"
           "U1 LM5164: 36 V -> 5 V, ~300 kHz (R_RON 41.2k), 47 uH, type-3 ripple injection (RA/CA/CB).")
    s.row(
        Part("J", "Connector:Screw_Terminal_01x02", "36V IN", TB2, {"1": "VIN_RAW", "2": GND},
             props=P("KF2EDGR-5.08-2P (+ plug KF2EDGK-5.08-2P C440880)", "C441204")),
        Part("F", "Device:Fuse", "5A T 5x20", FUSE_5X20, {"1": "VIN_RAW", "2": "VIN_F"},
             props=P("BLX-A holder + Littelfuse 0215005.MXP (C142718)", "C3131")),
        Part("Q", "Transistor_FET:Q_PMOS_GDS", "SQD50P06-15L", DPAK, {"1": "Q1_G", "2": "VIN_F", "3": VM},
             props=P("SQD50P06-15L_GE3", "C3281500")),
        R("22k", "Q1_G", GND),
        D("Device:D_Zener", "15V", VM, "Q1_G", SOD123, props=P("onsemi MMSZ5245BT1G", "C129730")),
        D("Device:D_Zener", "SMBJ36A (TVS)", VM, GND, SMB, props=P("SMBJ36A", "C114001")),
        CP("470uF 63V", VM, GND, props=P("Panasonic EEUFR1J471", "C407954")),
        C("2.2uF 100V", VM, GND, C1210),
    )
    s.row(
        Part("U", "Regulator_Switching:LM5164DDA", "LM5164DDAR", props=P("LM5164DDAR", "C477928"), footprint= "Package_SO:HSOP-8-1EP_3.9x4.9mm_P1.27mm_EP2.41x3.1mm_ThermalVias",
             conns={"1": GND, "2": VM, "3": VM, "4": "BUCK_RON", "5": "BUCK_FB", "6": None, "7": "BUCK_BST", "8": "BUCK_SW", "9": GND}),
        C("2.2uF 100V", VM, GND, C1210),
        C("2.2uF 100V", VM, GND, C1210),
        R("41.2k", "BUCK_RON", GND),
        C("2.2nF 50V", "BUCK_BST", "BUCK_SW"),
        Part("L", "Device:L", "47uH", L_BUCK, {"1": "BUCK_SW", "2": V5}, props=P("TDK VLS6045EX-470M", "C404505")),
        R("100k", V5, "BUCK_FB"),
        R("31.6k", "BUCK_FB", GND),
        R("200k", "BUCK_SW", "BUCK_RAMP"),
        C("3.3nF", "BUCK_RAMP", V5),
        C("100pF", "BUCK_RAMP", "BUCK_FB"),
        C("22uF 10V", V5, GND, C1206),
        C("22uF 10V", V5, GND, C1206),
    )
    s.row(
        R("4.7k", V5, "LED_PWR"),
        D("Device:LED", "green", GND, "LED_PWR", LED0603, props=P("KENTO KT-0603G", "C12624")),
        R("100k", VM, "VM_SENSE"),
        R("7.5k", "VM_SENSE", GND),
        C("10nF", "VM_SENSE", GND),
    )
    pr.pwr_flags += [("power", VM), ("power", V5)]

    # ================================================================ motor driver
    s = pr.sheet("driver", "driver.kicad_sch", "TMC5160 stepper driver")
    s.note("TMC5160 in SPI motion-controller mode (SD_MODE=0, SPI_MODE=1), internal clock.\n"
           "Sense resistors 50 mOhm -> 4.7 A RMS full scale; trim with GLOBALSCALER/IRUN.\n"
           "SRxL Kelvin lines: route R(SRAL)/R(SRBL) from the GND pad of each sense resistor.\n"
           "Each dual FET = one half-bridge: unit A high side (D=VM, S=BM), unit B low side (D=BM, S=sense).")
    tmc = {"33": VM, "4": VM, "29": "TMC_VCC", "20": V3, "5": "TMC_5V", "3": "TMC_12V",
           "34": "TMC_VCP", "32": "TMC_CPI", "31": "TMC_CPO",
           "13": "TMC_CSN", "14": "TMC_SCK", "15": "TMC_SDI", "16": "TMC_SDO",
           "21": GND, "22": V3, "12": GND, "11": GND, "28": "TMC_DRV_ENN",
           "26": "TMC_DIAG0", "27": "TMC_DIAG1", "17": "LIM_L", "18": "LIM_R",
           "24": "ENC_OUT_A", "23": "ENC_OUT_B", "25": "ENC_OUT_N",
           "6": GND, "19": GND, "30": GND, "49": GND,
           "8": "SRAH", "7": "SRAL", "9": "SRBH", "10": "SRBL"}
    for half in ("A1", "A2", "B1", "B2"):
        ph = half[0]
        num = {"A1": ("41", "42", "40", "39"), "A2": ("36", "35", "37", "38"),
               "B1": ("1", "2", "48", "47"), "B2": ("44", "43", "45", "46")}[half]
        tmc[num[0]] = "TMC_H" + half
        tmc[num[1]] = "TMC_C" + half
        tmc[num[2]] = "MOT_" + half
        tmc[num[3]] = "TMC_L" + half
    s.row(Part("U", "power_feed:TMC5160A-TA", "TMC5160A-TA", TQFP48, tmc, symbol=TMC_SYM,
               props={"LCSC": "C516354"}))
    s.row(
        C("100nF 100V", VM, GND, C0805),  # VS
        C("100nF 100V", VM, GND, C0805),  # VSA
        C("22nF 100V", "TMC_CPI", "TMC_CPO", C0805),
        C("100nF 100V", "TMC_VCP", VM, C0805),
        C("4.7uF 10V", "TMC_5V", GND, C0805),
        R("2.2", "TMC_5V", "TMC_VCC"),
        C("470nF", "TMC_VCC", GND),
        C("10uF 25V", "TMC_12V", GND, C1206),  # >=10x bootstrap after DC bias
        C("100nF", V3, GND),
    )
    for i, half in enumerate(("A1", "A2", "B1", "B2")):
        sns = "SNS_" + half[0]
        q = "Q%d" % (10 + i)
        s.row(
            C("220nF 50V", "TMC_C" + half, "MOT_" + half, C0603),
            R("4.7", "TMC_H" + half, "G_H" + half, R0402),
            R("4.7", "TMC_L" + half, "G_L" + half, R0402),
            Part("Q", "Transistor_FET:Q_Dual_NMOS_S1G1S2G2D2D2D1D1", "CSD88537ND", SOIC8,
                 {"1": "MOT_" + half, "2": "G_H" + half, "7": VM, "8": VM}, unit=1, ref=q,
                 props=P("CSD88537ND", "C117439")),
            Part("Q", "Transistor_FET:Q_Dual_NMOS_S1G1S2G2D2D2D1D1", "CSD88537ND", SOIC8,
                 {"3": sns, "4": "G_L" + half, "5": "MOT_" + half, "6": "MOT_" + half}, unit=2, ref=q,
                 props=P("CSD88537ND", "C117439")),
            C("2.2uF 100V", VM, GND, C1210),
        )
    s.row(
        R("0.05 1% 3W", "SNS_A", GND, R2512, props=P("CSM2512FT50L0", "C346860")),
        R("47", "SNS_A", "SRAH"),
        R("47", GND, "SRAL"),
        R("0.05 1% 3W", "SNS_B", GND, R2512, props=P("CSM2512FT50L0", "C346860")),
        R("47", "SNS_B", "SRBH"),
        R("47", GND, "SRBL"),
        CP("470uF 63V", VM, GND, props=P("Panasonic EEUFR1J471", "C407954")),
        Part("J", "Connector:Screw_Terminal_01x04", "MOTOR A+ A- B+ B-", TB4,
             {"1": "MOT_A1", "2": "MOT_A2", "3": "MOT_B1", "4": "MOT_B2"},
             props=P("KF2EDGR-5.08-4P (+ plug KF2EDGK-5.08-4P C440882)", "C441206")),
    )

    # ================================================================ MCU + interlock
    s = pr.sheet("mcu", "mcu.kicad_sch", "RP2040, SPI, E-stop interlock")
    s.note("Interlock (hardware, independent of firmware):\n"
           "  TMC_DRV_ENN = NAND(DRV_EN, E_OK)      -> driver off unless the MCU enables it AND E-stop loop closed\n"
           "  CLUTCH_*    = AND(MCU request, E_OK)  -> clutch off on E-stop\n"
           "MCU outputs have pull-downs so everything is off while the RP2040 is in reset.\n"
           "E-stop: NC contact between ESTOP_SRC (+3V3 via 330R) and ESTOP_RET. Open or broken wire -> E_OK low.\n"
           "The MCU only reads E_OK through 100k (E_OK_SENSE), so firmware can never hold the interlock enabled.\n"
           "The E-stop removes driver enable and clutch drive; it does not disconnect the 36 V bus.\n"
           "RP2040 per Raspberry Pi's minimal design: 12 MHz crystal (1k series), W25Q128JV QSPI flash (firmware uses 2 MB), internal\n"
           "1.1 V core regulator. GPIO numbers are the ones the old Pico module used, so the firmware is unchanged.\n"
           "+3V3: AP2112K from VSYS = +5V or USB VBUS through Schottkys (USB alone powers the logic for flashing).\n"
           "Flash a blank board: hold BOOTSEL, tap RESET, copy the UF2. SWD on a 1x3 0.1in header: SWCLK, GND, SWDIO (Debug Probe pinout).")
    mcu = {str(i): None for i in range(1, 58)}
    mcu.update({
        "1": V3, "10": V3, "22": V3, "33": V3, "42": V3, "49": V3,              # IOVDD
        "43": V3, "44": V3, "48": V3,                                          # ADC_AVDD, VREG_VIN, USB_VDD
        "45": "VDD_CORE", "23": "VDD_CORE", "50": "VDD_CORE",                  # VREG_VOUT -> DVDD
        "19": GND, "57": GND,                                                  # TESTEN, exposed pad
        "20": "XIN", "21": "XOUT", "24": "SWCLK", "25": "SWDIO", "26": "RUN",
        "46": "USB_DM", "47": "USB_DP",
        "51": "QSPI_SD3", "52": "QSPI_SCLK", "53": "QSPI_SD0", "54": "QSPI_SD2", "55": "QSPI_SD1", "56": "QSPI_SS",
        "2": "ENC_OUT_A", "3": "ENC_OUT_B", "4": "CLUTCH_HS_PWM", "5": "E_OK_SENSE",          # GPIO0..3
        "6": "I2C_SDA", "7": "I2C_SCL", "8": "LIM_L", "9": "LIM_R", "11": "CLUTCH_LS_EN",    # GPIO4..8
        "13": "DIAL_A", "14": "DIAL_B", "15": "DIAL_SW", "16": "MODE_L", "17": "MODE_R",    # GPIO10..14
        "18": "RAPID", "27": "SPI_MISO", "28": "SPI_CS", "29": "SPI_SCK", "30": "SPI_MOSI",  # GPIO15..19
        "31": "DRV_EN", "32": "TMC_DIAG0", "34": "TMC_DIAG1", "37": "LED_STAT",              # GPIO20..22, 25
        "38": "VM_SENSE", "39": "CLUTCH_ISENSE",                                             # GPIO26/27 = ADC0/1
    })
    s.row(Part("U", "MCU_RaspberryPi:RP2040", "RP2040", QFN56, mcu, props=P("Raspberry Pi RP2040", "C2040")))
    s.row(*[C("100nF", V3, GND) for _ in range(6)], C("100nF", V3, GND), C("100nF", V3, GND),   # IOVDD x6, AVDD, USB_VDD
          C("1uF", V3, GND), C("1uF", "VDD_CORE", GND), C("100nF", "VDD_CORE", GND), C("100nF", "VDD_CORE", GND),
          C("100nF", "VDD_CORE", GND))                                       # 3rd: right at DVDD pin 23 (layout review)
    s.row(
        Part("U", "Memory_Flash:W25Q128JVS", "W25Q128JVS", SOIC8_208,
             {"1": "QSPI_SS", "2": "QSPI_SD1", "3": "QSPI_SD2", "4": GND, "5": "QSPI_SD0", "6": "QSPI_SCLK",
              "7": "QSPI_SD3", "8": V3}, props=P("Winbond W25Q128JVSIQ (W25Q16 is out of stock; firmware uses the first 2 MB)", "C113767")),
        C("100nF", V3, GND),
        R("10k", "QSPI_SS", V3),
        R("1k", "QSPI_SS", "BOOTSEL_SW"),
        Part("SW", "Switch:SW_Push", "BOOTSEL", BUTTON, {"1": "BOOTSEL_SW", "2": GND}, props=P("Omron B3S-1000", "C2733655")),
        Part("Y", "Device:Crystal_GND24", "12MHz", XTAL3225, {"1": "XIN", "2": GND, "3": "XTAL_OUT", "4": GND},
             props=P("Abracon ABM8-272-T3 (12 MHz, CL 10 pF, ESR 50R; RPi-approved)", "C20625731")),
        R("1k", "XOUT", "XTAL_OUT"),
        C("15pF", "XIN", GND), C("15pF", "XTAL_OUT", GND),
    )
    s.row(
        Part("SW", "Switch:SW_Push", "RESET", BUTTON, {"1": "RUN", "2": GND}, props=P("Omron B3S-1000", "C2733655")),
        R("10k", "RUN", V3),
        Part("J", "Connector_Generic:Conn_01x03", "SWD", HDR3, {"1": "SWCLK", "2": GND, "3": "SWDIO"},
             props=P("2.54 mm 1x3 male header, hand-solder (Debug Probe's JST-to-0.1in cable)", "")),
        R("1k", "LED_STAT", "LED_STAT_A"),
        D("Device:LED", "green", GND, "LED_STAT_A", LED0603, props=P("KENTO KT-0603G", "C12624")),
    )
    pr.pwr_flags += [("mcu", "VSYS"), ("mcu", "VBUS"), ("mcu", GND)]   # GND: no symbol drives it now the Pico is gone
    s.row(
        Part("J", "Connector:USB_C_Receptacle_USB2.0_16P", "USB-C", USB_C,
             {"A4": "VBUS", "A9": "VBUS", "B4": "VBUS", "B9": "VBUS", "A1": GND, "A12": GND, "B1": GND, "B12": GND,
              "S1": GND, "A5": "USB_CC1", "B5": "USB_CC2", "A6": "USB_CONN_DP", "B6": "USB_CONN_DP",
              "A7": "USB_CONN_DM", "B7": "USB_CONN_DM", "A8": None, "B8": None},
             props=P("HRO TYPE-C-31-M-12", "C165948")),
        R("5.1k", "USB_CC1", GND), R("5.1k", "USB_CC2", GND),
        R("27", "USB_CONN_DP", "USB_DP"), R("27", "USB_CONN_DM", "USB_DM"),
        Part("U", "Power_Protection:USBLC6-2SC6", "USBLC6-2SC6", SOT23_6,                 # ESD at the connector
             {"1": "USB_CONN_DP", "6": "USB_CONN_DP", "3": "USB_CONN_DM", "4": "USB_CONN_DM", "5": "VBUS", "2": GND},
             props=P("STMicroelectronics USBLC6-2SC6", "C7519")),
        D("Device:D_Schottky", "B5819W", "VSYS", V5, SOD123, props=P("Changjing B5819W SL", "C8598")),
        D("Device:D_Schottky", "B5819W", "VSYS", "VBUS", SOD123, props=P("Changjing B5819W SL", "C8598")),
        Part("U", "Regulator_Linear:AP2112K-3.3", "AP2112K-3.3", SOT23_5,
             {"1": "VSYS", "2": GND, "3": "VSYS", "4": None, "5": V3}, props=P("Diodes AP2112K-3.3TRG1", "C51118")),
        C("1uF", "VSYS", GND), C("4.7uF 10V", V3, GND, fp=C0805),
    )
    s.row(
        R("33", "SPI_SCK", "TMC_SCK"),
        R("33", "SPI_MOSI", "TMC_SDI"),
        R("33", "SPI_CS", "TMC_CSN"),
        R("33", "TMC_SDO", "SPI_MISO"),
        R("10k", "TMC_CSN", V3),
        R("10k", "TMC_DIAG0", V3),
        R("10k", "TMC_DIAG1", V3),
    )
    s.row(
        Part("U", "74xGxx:74LVC1G00", "74LVC1G00", SOT23_5, props=P("SN74LVC1G00DBVR", "C7826"), conns=
             {"1": "DRV_EN", "2": "E_OK", "3": GND, "4": "TMC_DRV_ENN", "5": V3}),
        C("100nF", V3, GND),
        R("100k", "DRV_EN", GND),
        Part("U", "74xGxx:74AHCT1G08", "74AHCT1G08", SOT23_5, props=P("SN74AHCT1G08DBVR", "C113521"), conns=
             {"1": "CLUTCH_LS_EN", "2": "E_OK", "3": GND, "4": "CLUTCH_LS_GATE", "5": V5}),
        C("100nF", V5, GND),
        R("100k", "CLUTCH_LS_EN", GND),
        Part("U", "74xGxx:74AHCT1G08", "74AHCT1G08", SOT23_5, props=P("SN74AHCT1G08DBVR", "C113521"), conns=
             {"1": "CLUTCH_HS_PWM", "2": "E_OK", "3": GND, "4": "CLUTCH_HS_DRV", "5": V5}),
        C("100nF", V5, GND),
        R("100k", "CLUTCH_HS_PWM", GND),
    )
    s.row(
        R("330", V3, "ESTOP_SRC"),
        R("1k", "ESTOP_RET", "E_OK"),
        R("10k", "E_OK", GND),
        C("100nF", "E_OK", GND),
        # read-only tap: even if GP3 is driven high by mistake, E_OK stays at ~0.3 V (100k vs 10k)
        R("100k", "E_OK", "E_OK_SENSE"),
    )

    # ================================================================ clutch
    s = pr.sheet("clutch", "clutch.kicad_sch", "Clutch coil driver")
    s.note("Asymmetric half-bridge for the tooth-clutch coil (~24 Ohm, 1.5 A pull-in, 0.2-0.4 A hold).\n"
           "  Engage/hold: Q_LS on, PWM Q_HS. Off-time current recirculates Q_LS -> R_SHUNT -> D_FW2 (slow decay),\n"
           "  so the shunt sees the coil current continuously and ADC1 reads the average.\n"
           "  Release: both off -> coil sees -36 V through D_FW2/D_FW1, energy returns to the bus (fast decay).\n"
           "PTC limits the damage if firmware ever leaves the coil fully on.")
    s.row(
        Part("F", "Device:Polyfuse", "PTC 0.5A/60V", PTC, {"1": VM, "2": "CLUTCH_VIN"},
             props=P("Littelfuse 2920L050DR (0.5 A hold, 1 A trip, 60 V)", "C207082")),
        C("2.2uF 100V", "CLUTCH_VIN", GND, C1210),
        Part("Q", "Transistor_FET:Q_PMOS_GDS", "AGM60P20R", SOT223, {"1": "HS_GATE", "2": "COIL_P", "3": "CLUTCH_VIN"},
             props=P("AGM60P20R", "C6719402")),
        R("2.2k", "HS_GATE", "CLUTCH_VIN"),
        D("Device:D_Zener", "12V", "CLUTCH_VIN", "HS_GATE", SOD123, props=P("onsemi MMSZ5242BT1G", "C82050")),
        R("4.7k", "HS_GATE", "HS_PULL", R1206),  # ~0.13 W while on
        Part("Q", "Transistor_FET:Q_NMOS_GSD", "2N7002", SOT23, {"1": "CLUTCH_HS_DRV", "2": GND, "3": "HS_PULL"},
             props=P("2N7002", "C8545")),
    )
    s.row(
        Part("J", "Connector:Screw_Terminal_01x02", "CLUTCH COIL", TB2, {"1": "COIL_P", "2": "COIL_N"},
             props=P("KF2EDGR-5.08-2P (+ plug KF2EDGK-5.08-2P C440880)", "C441204")),
        D("Device:D_Schottky", "SS210 (fast decay)", "CLUTCH_VIN", "COIL_N", SMA, props=P("SS210", "C14996")),
        D("Device:D_Schottky", "SS210 (freewheel)", "COIL_P", GND, SMA, props=P("SS210", "C14996")),
        R("100", "CLUTCH_LS_GATE", "LS_G"),
        R("100k", "LS_G", GND),
        Part("Q", "Transistor_FET:Q_NMOS_GSD", "SI2308A", SOT23, {"1": "LS_G", "2": "CLUTCH_SHUNT", "3": "COIL_N"},
             props=P("UMW SI2308A", "C347491")),
        R("0.1 1% 1W", "CLUTCH_SHUNT", GND, R1206, props=P("FOJAN FRM121WFR100TM", "C7467250")),
        R("22k", "COIL_P", "LED_CLU"),
        D("Device:LED", "yellow", GND, "LED_CLU", LED0603, props=P("XINGLIGHT XL-1608UYC-06", "C965802")),
    )
    s.row(
        R("1k", "CLUTCH_SHUNT", "OA_IN"),
        C("1nF", "OA_IN", GND),
        Part("U", "Amplifier_Operational:TLV9061xDBV", "TLV9061", SOT23_5, props=P("TLV9061IDBVR", "C398358"), conns=
             {"1": "OA_OUT", "2": GND, "3": "OA_IN", "4": "OA_FB", "5": V3}),
        C("100nF", V3, GND),
        R("200k", "OA_OUT", "OA_FB"),
        R("10k", "OA_FB", GND),
        R("1k", "OA_OUT", "CLUTCH_ISENSE"),
        C("10nF", "CLUTCH_ISENSE", GND),
    )

    # ================================================================ panel I/O
    s = pr.sheet("io", "io.kicad_sch", "Panel and sensor I/O")
    s.note("All panel inputs: pull-up at the connector, ESD array, series R + RC filter into the logic.\n"
           "Switch contacts go to GND (active low). Limit switches are NC to GND, so a broken wire reads as 'hit'.\n"
           "Output encoder (LPD3806, NPN open collector) powered from 5 V, pulled up to 3.3 V (safe for the RP2040 + TMC5160).")
    s.row(
        # one ribbon to the front panel: OLED, E-stop, mode switch, rapid button, speed dial
        Part("J", "Connector_Generic:Conn_02x07_Odd_Even", "PANEL 2x7 IDC", IDC_2X7,
             {"1": GND, "2": V3, "3": "OLED_SCL", "4": "OLED_SDA", "5": GND, "6": "ESTOP_SRC", "7": "ESTOP_RET",
              "8": "MODE_L_RAW", "9": "MODE_R_RAW", "10": "RAPID_RAW", "11": "DIAL_A_RAW", "12": "DIAL_B_RAW",
              "13": "DIAL_SW_RAW", "14": GND},
             props={"MPN": "ZHOURI DC3-2.54-14PAS (2x7 shrouded box header)", "LCSC": "C2977595",
                    "Description": "front panel ribbon; E-stop NC loop on pins 6-7, so an unplugged ribbon = E-stop"}),
        # one cable from the feed unit: output encoder + both limit switches
        XH_CONN(7, [V5, GND, "ENC_A_RAW", "ENC_B_RAW", "ENC_N_RAW", "LIM_L_RAW", "LIM_R_RAW"],
                props={"Description": "feed unit: encoder (5 V, A, B, N) + NC limit switches L/R"}),
    )
    inputs = [  # raw net, clean net, pull-up, series R, filter C
        ("MODE_L_RAW", "MODE_L", "10k", "1k", "100nF"),
        ("MODE_R_RAW", "MODE_R", "10k", "1k", "100nF"),
        ("RAPID_RAW", "RAPID", "10k", "1k", "100nF"),
        ("LIM_L_RAW", "LIM_L", "10k", "1k", "10nF"),
        ("LIM_R_RAW", "LIM_R", "10k", "1k", "10nF"),
        ("DIAL_A_RAW", "DIAL_A", "10k", "1k", "10nF"),
        ("DIAL_B_RAW", "DIAL_B", "10k", "1k", "10nF"),
        ("DIAL_SW_RAW", "DIAL_SW", "10k", "1k", "100nF"),
        ("ENC_A_RAW", "ENC_OUT_A", "2.2k", "100", "100pF"),
        ("ENC_B_RAW", "ENC_OUT_B", "2.2k", "100", "100pF"),
        ("ENC_N_RAW", "ENC_OUT_N", "2.2k", "100", "100pF"),
    ]
    for raw, clean, pu, rs, cf in inputs:
        s.row(R(pu, V3, raw), R(rs, raw, clean), C(cf, clean, GND))
    s.row(R("10k", V3, "OLED_SCL"), R("10k", V3, "OLED_SDA"), R("100", "OLED_SCL", "I2C_SCL"), R("100", "OLED_SDA", "I2C_SDA"))
    esd = [["MODE_L_RAW", "MODE_R_RAW", "RAPID_RAW", "LIM_L_RAW"],
           ["LIM_R_RAW", "DIAL_A_RAW", "DIAL_B_RAW", "DIAL_SW_RAW"],
           ["ENC_A_RAW", "ENC_B_RAW", "ENC_N_RAW", "ESTOP_RET"],
           ["OLED_SCL", "OLED_SDA", None, None]]
    s.row(*[Part("D", "Power_Protection:USBLC6-4SC6", "USBLC6-4SC6", SOT23_6,
                 {"1": e[0], "3": e[1], "4": e[2], "6": e[3], "2": GND, "5": V3},
                 props=P("USBLC6-4SC6", "C111212")) for e in esd])
    return pr


if __name__ == "__main__":
    pr = build()
    pr.write(OUT)
    print("wrote", os.path.abspath(OUT))
