"""Reference map: board refs before the RP2040 change -> schematic refs after it.

Adding the RP2040 parts to the MCU sheet (commit 2d7a7f7) renumbered every part annotated after it. The schematic is
the source of truth, so rp2040_layout.py renames the board footprints to match. The earlier hand-fix scripts were
written against the old numbering; they look parts up through cur() so they stay rerunnable on either board.

Round 2 (commit 62b23bf, PCB review round 2): a VDD_CORE 100 nF (new C40) and the USBLC6-2SC6 ESD array (new U5)
were added, so C40..C63 -> C41..C64 and U5..U9 -> U6..U10 (OLD2NEW2, applied on top of OLD2NEW).
"""

OLD2NEW = {
    "C29": "C46", "C30": "C47", "C31": "C48", "C32": "C49", "C33": "C50", "C35": "C52", "C36": "C53", "C37": "C54",
    "C38": "C55", "C39": "C56", "C40": "C57", "C41": "C58", "C42": "C59", "C43": "C60", "C44": "C61", "C45": "C62",
    "C46": "C63", "D4": "D5", "D5": "D7", "D6": "D8", "D7": "D9", "D8": "D10", "D9": "D11", "D10": "D12",
    "D11": "D13", "D12": "D14", "J3": "J5", "J4": "J6", "J5": "J7", "R24": "R33", "R25": "R34", "R26": "R35",
    "R27": "R36", "R28": "R37", "R29": "R38", "R30": "R39", "R31": "R40", "R32": "R41", "R33": "R42", "R34": "R43",
    "R35": "R44", "R36": "R45", "R37": "R46", "R38": "R47", "R39": "R48", "R40": "R49", "R41": "R50", "R42": "R51",
    "R43": "R52", "R44": "R53", "R45": "R54", "R46": "R55", "R47": "R56", "R48": "R57", "R49": "R58", "R50": "R59",
    "R51": "R60", "R52": "R61", "R53": "R62", "R54": "R63", "R55": "R64", "R56": "R65", "R57": "R66", "R58": "R67",
    "R59": "R68", "R60": "R69", "R61": "R70", "R62": "R71", "R63": "R72", "R64": "R73", "R65": "R74", "R66": "R75",
    "R67": "R76", "R68": "R77", "R69": "R78", "R70": "R79", "R71": "R80", "R72": "R81", "R73": "R82", "SW1": "SW2",
    "U3": "U6", "U4": "U7", "U5": "U8", "U6": "U9",
}


OLD2NEW2 = dict([("C%d" % n, "C%d" % (n + 1)) for n in range(40, 64)] +
                [("U%d" % n, "U%d" % (n + 1)) for n in range(5, 10)])


def renumbered(board):
    """True once rp2040_layout.py has renamed the footprints (the Pico module A1 is gone, the RP2040 U3 is there)."""
    return board.FindFootprintByReference("A1") is None


def renumbered2(board):
    """True once the round-2 renumbering is applied (U10, the TLV9061, exists)."""
    return board.FindFootprintByReference("U10") is not None


def cur(board, old_ref):
    """The current reference of a part known by its pre-RP2040 reference."""
    ref = OLD2NEW.get(old_ref, old_ref) if renumbered(board) else old_ref
    return OLD2NEW2.get(ref, ref) if renumbered2(board) else ref
