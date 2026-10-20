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
PICO = "Module:RaspberryPi_Pico_SMD"
TB2 = "Connector_Phoenix_MSTB:PhoenixContact_MSTBA_2,5_2-G-5,08_1x02_P5.08mm_Horizontal"
TB4 = "Connector_Phoenix_MSTB:PhoenixContact_MSTBA_2,5_4-G-5,08_1x04_P5.08mm_Horizontal"
XH = "Connector_JST:JST_XH_B%dB-XH-A_1x0%d_P2.50mm_Vertical"
FUSE_5X20 = "power_feed:Fuseholder_XC_BLX-A_5x20mm_P22.00mm"
PTC = "Fuse:Fuse_2920_7451Metric"
SOT223 = "Package_TO_SOT_SMD:SOT-223-3_TabPin2"
SOIC8 = "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm"
BUTTON = "Button_Switch_SMD:SW_SPST_B3S-1000"
L_BUCK = "Inductor_SMD:L_TDK_VLS6045EX_VLS6045AF"


def P(mpn, lcsc=""):
    """Ordering properties. An empty LCSC number means 'not yet verified at LCSC'."""
    return {"MPN": mpn, "LCSC": lcsc}

GND, V3, V5, VM = "GND", "+3V3", "+5V", "+36V"


def R(val, a, b, fp=R0603, **kw):
    return Part("R", "Device:R", val, fp, {"1": a, "2": b}, **kw)


def C(val, a, b, fp=C0603, **kw):
    return Part("C", "Device:C", val, fp, {"1": a, "2": b}, **kw)


def CP(val, plus, minus, fp=CP_BULK, **kw):
    return Part("C", "Device:C_Polarized", val, fp, {"1": plus, "2": minus}, **kw)


def D(lib_id, val, k, a, fp, **kw):
    return Part("D", lib_id, val, fp, {"1": k, "2": a}, **kw)


XH_LCSC = {3: "C144394", 4: "C144395", 5: "C157991"}


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
        D("Device:D_Zener", "15V", VM, "Q1_G", SOD123, props=P("MMSZ5245B")),
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
        D("Device:LED", "green", GND, "LED_PWR", LED0603),
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
    s = pr.sheet("mcu", "mcu.kicad_sch", "Pico, SPI, E-stop interlock")
    s.note("Interlock (hardware, independent of firmware):\n"
           "  TMC_DRV_ENN = NAND(DRV_EN, E_OK)      -> driver off unless Pico enables it AND E-stop loop closed\n"
           "  CLUTCH_*    = AND(Pico request, E_OK) -> clutch off on E-stop\n"
           "Pico outputs have pull-downs so everything is off while the Pico is in reset.\n"
           "E-stop: NC contact between ESTOP_SRC (+3V3 via 330R) and ESTOP_RET. Open or broken wire -> E_OK low.\n"
           "The Pico only reads E_OK through 100k (E_OK_SENSE), so firmware can never hold the interlock enabled.\n"
           "The E-stop removes driver enable and clutch drive; it does not disconnect the 36 V bus.\n"
           "SWD: solder a 3-pin header to the Pico's own SWCLK/GND/SWDIO pads (module edge).")
    pico = {str(i): None for i in range(1, 41)}
    pico.update({"3": GND, "8": GND, "13": GND, "18": GND, "23": GND, "28": GND, "38": GND, "33": GND,
                 "1": "ENC_OUT_A", "2": "ENC_OUT_B", "4": "CLUTCH_HS_PWM", "5": "E_OK_SENSE",
                 "6": "I2C_SDA", "7": "I2C_SCL", "9": "LIM_L", "10": "LIM_R", "11": "CLUTCH_LS_EN",
                 "14": "DIAL_A", "15": "DIAL_B", "16": "DIAL_SW", "17": "MODE_L", "19": "MODE_R", "20": "RAPID",
                 "21": "SPI_MISO", "22": "SPI_CS", "24": "SPI_SCK", "25": "SPI_MOSI", "26": "DRV_EN",
                 "27": "TMC_DIAG0", "29": "TMC_DIAG1", "30": "RUN", "31": "VM_SENSE", "32": "CLUTCH_ISENSE",
                 "36": V3, "39": "VSYS"})
    s.row(Part("A", "MCU_Module:RaspberryPi_Pico", "Raspberry Pi Pico", PICO, pico))
    pr.pwr_flags += [("mcu", "VSYS")]
    s.row(
        D("Device:D_Schottky", "B5819W", "VSYS", V5, SOD123, props=P("B5819W")),
        Part("SW", "Switch:SW_Push", "RESET", BUTTON, {"1": "RUN", "2": GND}),
        R("33", "SPI_SCK", "TMC_SCK"),
        R("33", "SPI_MOSI", "TMC_SDI"),
        R("33", "SPI_CS", "TMC_CSN"),
        R("33", "TMC_SDO", "SPI_MISO"),
        R("10k", "TMC_CSN", V3),
        R("10k", "TMC_DIAG0", V3),
        R("10k", "TMC_DIAG1", V3),
    )
    s.row(
        Part("U", "74xGxx:74LVC1G00", "74LVC1G00", SOT23_5, props=P("SN74LVC1G00DBVR"), conns=
             {"1": "DRV_EN", "2": "E_OK", "3": GND, "4": "TMC_DRV_ENN", "5": V3}),
        C("100nF", V3, GND),
        R("100k", "DRV_EN", GND),
        Part("U", "74xGxx:74AHCT1G08", "74AHCT1G08", SOT23_5, props=P("SN74AHCT1G08DBVR"), conns=
             {"1": "CLUTCH_LS_EN", "2": "E_OK", "3": GND, "4": "CLUTCH_LS_GATE", "5": V5}),
        C("100nF", V5, GND),
        R("100k", "CLUTCH_LS_EN", GND),
        Part("U", "74xGxx:74AHCT1G08", "74AHCT1G08", SOT23_5, props=P("SN74AHCT1G08DBVR"), conns=
             {"1": "CLUTCH_HS_PWM", "2": "E_OK", "3": GND, "4": "CLUTCH_HS_DRV", "5": V5}),
        C("100nF", V5, GND),
        R("100k", "CLUTCH_HS_PWM", GND),
    )
    s.row(
        XH_CONN(2, ["ESTOP_SRC", "ESTOP_RET"], props={"Description": "E-stop, normally-closed contact"}),
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
             props=P("Littelfuse 2920L050/60MR (verify 60 V rating + LCSC)")),
        C("2.2uF 100V", "CLUTCH_VIN", GND, C1210),
        Part("Q", "Transistor_FET:Q_PMOS_GDS", "AGM60P20R", SOT223, {"1": "HS_GATE", "2": "COIL_P", "3": "CLUTCH_VIN"},
             props=P("AGM60P20R", "C6719402")),
        R("2.2k", "HS_GATE", "CLUTCH_VIN"),
        D("Device:D_Zener", "12V", "CLUTCH_VIN", "HS_GATE", SOD123, props=P("MMSZ5242B")),
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
        D("Device:LED", "yellow", GND, "LED_CLU", LED0603),
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
           "Output encoder (LPD3806, NPN open collector) powered from 5 V, pulled up to 3.3 V (safe for Pico + TMC5160).")
    s.row(
        XH_CONN(4, [GND, "MODE_L_RAW", "MODE_R_RAW", "RAPID_RAW"], props={"Description": "ON-OFF-ON mode switch + rapid button"}),
        XH_CONN(4, [GND, "DIAL_A_RAW", "DIAL_B_RAW", "DIAL_SW_RAW"], props={"Description": "speed dial encoder"}),
        XH_CONN(3, [GND, "LIM_L_RAW", "LIM_R_RAW"], props={"Description": "limit switches (NC)"}),
        XH_CONN(4, [GND, V3, "OLED_SCL", "OLED_SDA"], props={"Description": "OLED (3.3 V only)"}),
        XH_CONN(5, [V5, GND, "ENC_A_RAW", "ENC_B_RAW", "ENC_N_RAW"], props={"Description": "output-side encoder"}),
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
