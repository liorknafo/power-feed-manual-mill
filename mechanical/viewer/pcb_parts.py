"""PCB component list for the 3D viewer: where each part sits on the board and what it is.

Run with KiCad's Python (needs pcbnew):
  /Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/3.9/bin/python3 \
      mechanical/viewer/pcb_parts.py
Writes mechanical/viewer/out/pcb_parts.json: one entry per footprint with its bounding box in PCB mm (x right, y down),
value, MPN / LCSC and schematic sheet (both read from the generated .kicad_sch files), and a one-line role.
"""

import glob
import json
import os
import re

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
KICAD = os.path.join(ROOT, "hardware", "kicad")

# what the important parts do, by value (prefix match); generic passives fall back to their kind
ROLES = {
    "TMC5160": "Stepper motor controller/driver. Runs the ramp generator, reads the output encoder and drives the four external MOSFET half-bridges over SPI.",
    "CSD88537": "Dual N-MOSFET: one half-bridge of the motor's H-bridges (4 of these drive the two motor coils).",
    "LM5164": "36 V -> 5 V buck regulator for the logic, gate drives and encoder.",
    "AP2112K": "3.3 V LDO for the RP2040, flash, OLED and logic (from VSYS = 5 V or USB).",
    "RP2040": "The microcontroller: runs the Rust firmware (state machine, UI, clutch current control).",
    "W25Q": "QSPI flash: holds the firmware and the saved settings.",
    "12MHz": "12 MHz crystal: the RP2040's clock.",
    "Raspberry Pi Pico": "Pico module (older board revision): the microcontroller.",
    "74LVC1G00": "NAND gate of the hardware E-stop interlock: the driver can only be enabled if the MCU asks AND the E-stop loop is closed.",
    "74AHCT1G08": "AND gate of the E-stop interlock for the clutch: the clutch drive drops when the E-stop opens, whatever the firmware does.",
    "TLV9061": "Op-amp: amplifies the clutch shunt voltage for the MCU's ADC (clutch current control).",
    "AGM60P": "P-MOSFET: high side of the clutch coil driver (PWM).",
    "SI2308": "N-MOSFET: low side of the clutch coil driver.",
    "2N7002": "Small N-MOSFET: level shifter for the clutch high-side gate.",
    "B5819W": "Schottky diode (power OR-ing / protection).",
    "SS210": "Schottky diode: clutch coil freewheel path.",
    "USB-C": "USB-C: flashing and power for the logic on the bench.",
    "BOOTSEL": "BOOTSEL button: hold while tapping RESET to get the RP2040's USB drive for flashing.",
    "RESET": "Reset button for the MCU.",
    "SWD": "SWD debug header (SWCLK, GND, SWDIO) for the Raspberry Pi Debug Probe.",
    "Fuse": "Input fuse for the 36 V supply.",
}
KIND = {"R": "Resistor", "C": "Capacitor", "D": "Diode / LED", "L": "Inductor", "Q": "Transistor", "U": "IC",
        "J": "Connector", "F": "Fuse / PTC", "SW": "Button", "Y": "Crystal", "A": "Module", "TP": "Test point"}
CONNECTORS = {"J1": "36 V input from the power supply.", "J2": "Stepper motor (A+ A- B+ B-).",
              "J3": "SWD debug header (SWCLK, GND, SWDIO) for the Raspberry Pi Debug Probe. Hand-soldered.",
              "J4": "USB-C: flashing (hold BOOTSEL, tap RESET) and logic power on the bench.",
              "J5": "Clutch coil.", "J6": "Ribbon to the control panel (E-stop, switch, rapid, dial, OLED).",
              "J7": "Output encoder and the two limit switches."}


def schematic_info():
    """ref -> {value, mpn, lcsc, sheet} from the generated schematic sheets."""
    info = {}
    for path in glob.glob(os.path.join(KICAD, "*.kicad_sch")):
        txt = open(path).read()
        title = re.search(r'\(title "([^"]*)"', txt)
        sheet = title.group(1) if title else os.path.basename(path)
        for blk in txt.split("\n\t(symbol\n")[1:]:                     # top-level symbol instances
            ref = re.search(r'\(property "Reference" "([^"#][^"]*)"', blk)
            if not ref:
                continue
            get = lambda k: (re.search(r'\(property "%s" "([^"]*)"' % k, blk) or [None, ""])[1]  # noqa: E731
            info[ref.group(1)] = {"value": get("Value"), "mpn": get("MPN"), "lcsc": get("LCSC"), "sheet": sheet}
    return info


def role(ref, value):
    if ref in CONNECTORS:
        return CONNECTORS[ref]
    for k, v in ROLES.items():
        if value.startswith(k) or k in value:
            return v
    prefix = re.match(r"[A-Z]+", ref).group(0)
    return KIND.get(prefix, "Component")


def main():
    board = pcbnew.LoadBoard(os.path.join(KICAD, "power-feed.kicad_pcb"))
    sch = schematic_info()
    out = []
    for fp in board.GetFootprints():
        ref, val = fp.GetReference(), fp.GetValue()
        bb = fp.GetBoundingBox(False, False)
        s = sch.get(ref, {})
        out.append({"ref": ref, "value": val, "mpn": s.get("mpn", ""), "lcsc": s.get("lcsc", ""), "sheet": s.get("sheet", ""),
                    "role": role(ref, val), "footprint": fp.GetFPIDAsString().split(":")[-1],
                    "x": pcbnew.ToMM(bb.GetX()), "y": pcbnew.ToMM(bb.GetY()),
                    "w": pcbnew.ToMM(bb.GetWidth()), "h": pcbnew.ToMM(bb.GetHeight()), "bottom": fp.IsFlipped()})
    os.makedirs(os.path.join(HERE, "out"), exist_ok=True)
    with open(os.path.join(HERE, "out", "pcb_parts.json"), "w") as fh:
        json.dump(out, fh, indent=0)
    print("%d parts, %d with schematic info" % (len(out), sum(1 for p in out if p["sheet"])))


if __name__ == "__main__":
    main()
