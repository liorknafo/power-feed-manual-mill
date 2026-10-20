"""Verify the generated schematic: KiCad ERC, netlist == intended netlist, footprints exist."""

import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(__file__))
import build_schematic  # noqa: E402
from kicad_gen import footprint_exists  # noqa: E402

KCLI = "/Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli"
OUT = os.path.abspath(build_schematic.OUT)
TOP = os.path.join(OUT, "power-feed.kicad_sch")

# Known, reviewed ERC findings that are not design errors.
ALLOWED = []


def run(args):
    return subprocess.run(args, capture_output=True, text=True)


def main():
    pr = build_schematic.build()
    pr.write(OUT)
    ok = True

    # 1. ERC
    erc = os.path.join(OUT, "erc.json")
    run([KCLI, "sch", "erc", "--format", "json", "--severity-all", "-o", erc, TOP])
    data = json.load(open(erc))
    problems = []
    for sh in data["sheets"]:
        for v in sh["violations"]:
            items = [i["description"] for i in v["items"]]
            if any(v["type"] == t and any(m in d for d in items) for t, m in ALLOWED):
                continue
            problems.append("%s %s [%s] %s :: %s" % (sh["path"], v["severity"], v["type"], v["description"], "; ".join(items)))
    print("ERC: %d unexpected findings" % len(problems))
    for p in problems:
        print("  " + p)
    ok &= not any(" error " in p for p in problems)

    # 2. netlist comparison
    netf = os.path.join(OUT, "power-feed.net.xml")
    run([KCLI, "sch", "export", "netlist", "--format", "kicadxml", "-o", netf, TOP])
    root = ET.parse(netf).getroot()
    actual = {}
    for net in root.iter("net"):
        raw = net.get("name")
        if raw.startswith("unconnected-") or raw.startswith("Net-("):
            continue
        name = raw.split("/")[-1]
        nodes = {(n.get("ref"), n.get("pin")) for n in net.iter("node")}
        actual[name] = nodes
    intended = pr.intended_netlist()
    mism = 0
    for name in sorted(set(intended) | set(actual)):
        a, i = actual.get(name, set()), intended.get(name, set())
        if a != i:
            mism += 1
            print("  NET %s: missing %s extra %s" % (name, sorted(i - a), sorted(a - i)))
    print("Netlist: %d nets intended, %d mismatches" % (len(intended), mism))
    ok &= mism == 0
    single = [n for n, nodes in intended.items() if len(nodes) < 2]
    if single:
        print("Nets with a single connection (check!): %s" % single)
        ok = False

    # 3. footprints
    missing = []
    for s in pr.sheets:
        for row in s.rows:
            for p in row:
                if not footprint_exists(p.footprint):
                    missing.append("%s %s '%s'" % (p.ref, p.value, p.footprint))
    print("Footprints: %d missing/unassigned" % len(missing))
    for m in sorted(set(missing)):
        print("  " + m)

    n_parts = len({p.ref for s in pr.sheets for row in s.rows for p in row})
    print("Parts: %d" % n_parts)
    print("RESULT:", "PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
