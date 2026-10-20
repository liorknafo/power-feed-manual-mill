"""Run KiCad DRC (with schematic parity) on the board and summarise the result."""

import collections
import json
import os
import subprocess
import sys

KCLI = "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"
KDIR = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "kicad"))
PCB = os.path.join(KDIR, "power-feed.kicad_pcb")
REPORT = os.path.join(KDIR, "drc.json")


def main():
    subprocess.run([KCLI, "pcb", "drc", "--format", "json", "--schematic-parity", "--severity-all",
                    "-o", REPORT, PCB], capture_output=True, text=True)
    d = json.load(open(REPORT))
    groups = collections.Counter()
    examples = {}
    for key in ("violations", "unconnected_items", "schematic_parity"):
        for v in d.get(key, []):
            k = (key, v.get("severity"), v.get("type"))
            groups[k] += 1
            examples.setdefault(k, [])
            if len(examples[k]) < 4:
                examples[k].append(v.get("description") + " :: " + "; ".join(i.get("description", "") for i in v.get("items", [])))
    errors = sum(n for (key, sev, _), n in groups.items() if sev == "error")
    for k, n in sorted(groups.items(), key=lambda kv: (kv[0][1] != "error", -kv[1])):
        print("%-18s %-8s %-32s %4d" % k + (n,) if False else "%-18s %-8s %-32s %4d" % (k[0], k[1], k[2], n))
        for e in examples[k]:
            print("      " + e[:200])
    print("RESULT:", "PASS" if errors == 0 else "FAIL (%d errors)" % errors)
    return 0 if errors == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
