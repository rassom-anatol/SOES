#!/usr/bin/env python3
"""Survey EtherCAT Slave Information (ESI) files and tabulate their PDO layouts.

Reads one or more ESI XML files (or directories of them) and writes CSV tables
describing each device's process-data layout: which PDOs exist, whether they are
fixed or master-assignable, what each one maps, and how often each CiA402 object
appears across the surveyed devices.

The last of those is the point of the exercise -- it shows which optional objects
real drives consistently expose, which is what a new object dictionary should
include.

Usage:
    esi_survey.py OUTDIR INPUT [INPUT ...]

INPUT may be an .xml file or a directory containing them. Parsing is streaming,
so multi-gigabyte vendor files are handled without loading them whole.
"""

import csv
import os
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict

# CiA402 / CoE indices worth naming in the output. Anything not listed is still
# counted, just without a friendly name.
KNOWN = {
    0x1C12: "RxPDO assign", 0x1C13: "TxPDO assign",
    0x1C32: "SM2 sync", 0x1C33: "SM3 sync", 0x10F1: "Error settings",
    0x603F: "Error code", 0x6040: "Controlword", 0x6041: "Statusword",
    0x605A: "Quick stop option", 0x605B: "Shutdown option",
    0x605C: "Disable op option", 0x605D: "Halt option",
    0x605E: "Fault reaction option", 0x6060: "Modes of operation",
    0x6061: "Modes of operation display", 0x6062: "Position demand",
    0x6063: "Position actual internal", 0x6064: "Position actual",
    0x6065: "Following error window", 0x6067: "Position window",
    0x606B: "Velocity demand", 0x606C: "Velocity actual",
    0x606D: "Velocity window", 0x6071: "Target torque",
    0x6072: "Max torque", 0x6074: "Torque demand",
    0x6077: "Torque actual", 0x6078: "Current actual",
    0x607A: "Target position", 0x607C: "Home offset",
    0x607D: "Software position limit", 0x607F: "Max profile velocity",
    0x6080: "Max motor speed", 0x6081: "Profile velocity",
    0x6083: "Profile acceleration", 0x6084: "Profile deceleration",
    0x6085: "Quick stop deceleration", 0x6086: "Motion profile type",
    0x6091: "Gear ratio", 0x6098: "Homing method",
    0x6099: "Homing speeds", 0x609A: "Homing acceleration",
    0x60B0: "Position offset", 0x60B1: "Velocity offset",
    0x60B2: "Torque offset", 0x60B8: "Touch probe function",
    0x60B9: "Touch probe status", 0x60BA: "Touch probe pos1 pos",
    0x60BB: "Touch probe pos1 neg", 0x60BC: "Touch probe pos2 pos",
    0x60BD: "Touch probe pos2 neg", 0x60C1: "Interpolation data record",
    0x60C2: "Interpolation time period", 0x60F4: "Following error actual",
    0x60FD: "Digital inputs", 0x60FE: "Digital outputs",
    0x60FF: "Target velocity", 0x6502: "Supported drive modes",
}


def hexval(text):
    """Parse an ESI numeric field. ESI writes hex as '#x1600'; decimal appears bare."""
    if text is None:
        return None
    t = text.strip()
    if not t:
        return None
    try:
        if t.lower().startswith("#x"):
            return int(t[2:], 16)
        if t.lower().startswith("0x"):
            return int(t[2:], 16)
        return int(t)
    except ValueError:
        return None


def text_of(el, tag):
    child = el.find(tag)
    return child.text.strip() if child is not None and child.text else ""


def device_name(dev):
    """Prefer the English <Name>, fall back to any <Name>, then <Type>."""
    for name in dev.findall("Name"):
        if name.get("LcId") in ("1033", None) and name.text:
            return name.text.strip()
    for name in dev.findall("Name"):
        if name.text:
            return name.text.strip()
    return text_of(dev, "Type")


def parse_pdos(dev, kind):
    """Yield one dict per RxPdo/TxPdo element, with its mapped entries."""
    for pdo in dev.findall(kind):
        idx = hexval(text_of(pdo, "Index"))
        entries = []
        bits = 0
        for e in pdo.findall("Entry"):
            ei = hexval(text_of(e, "Index"))
            bl = hexval(text_of(e, "BitLen")) or 0
            bits += bl
            entries.append({
                "index": ei,
                "sub": hexval(text_of(e, "SubIndex")) or 0,
                "bitlen": bl,
                "name": text_of(e, "Name"),
                "datatype": text_of(e, "DataType"),
                # index 0 / sub 0 is padding, not a real mapping
                "padding": (ei in (0, None)),
            })
        yield {
            "index": idx,
            "name": text_of(pdo, "Name"),
            "sm": pdo.get("Sm"),
            "fixed": pdo.get("Fixed") == "1",
            "mandatory": pdo.get("Mandatory") == "1",
            "entries": entries,
            "bits": bits,
        }


def iter_devices(path):
    """Stream <Device> elements out of an ESI file, freeing each after use."""
    vendor = ""
    try:
        ctx = ET.iterparse(path, events=("end",))
        for _, el in ctx:
            tag = el.tag.rsplit("}", 1)[-1]
            if tag == "Vendor" and not vendor:
                vendor = text_of(el, "Name") or ""
            elif tag == "Device":
                yield vendor, el
                el.clear()
    except ET.ParseError as exc:
        print(f"  ! parse error in {os.path.basename(path)}: {exc}", file=sys.stderr)


def collect_inputs(args):
    files = []
    for a in args:
        if os.path.isdir(a):
            files += [os.path.join(a, f) for f in sorted(os.listdir(a))
                      if f.lower().endswith(".xml")]
        elif os.path.isfile(a):
            files.append(a)
        else:
            print(f"  ! not found: {a}", file=sys.stderr)
    return files


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return 2

    outdir, inputs = sys.argv[1], sys.argv[2:]
    os.makedirs(outdir, exist_ok=True)
    files = collect_inputs(inputs)
    if not files:
        print("no input files", file=sys.stderr)
        return 1

    dev_rows, pdo_rows, entry_rows = [], [], []
    # object index -> set of devices mapping it, split by direction
    obj_devices = defaultdict(set)
    obj_dir = defaultdict(Counter)
    # same, but only over devices that actually implement CiA402
    obj_devices_402 = defaultdict(set)
    cia402_devices = set()
    total_devices = 0

    for path in files:
        base = os.path.basename(path)
        print(f"  parsing {base} ({os.path.getsize(path) // 1024} KB)")
        for vendor, dev in iter_devices(path):
            dtype = dev.find("Type")
            name = device_name(dev)
            # Skip devices with no process data at all -- couplers, gateways.
            rx = list(parse_pdos(dev, "RxPdo"))
            tx = list(parse_pdos(dev, "TxPdo"))
            if not rx and not tx:
                continue
            total_devices += 1

            product = hexval(dtype.get("ProductCode")) if dtype is not None else None
            revision = hexval(dtype.get("RevisionNo")) if dtype is not None else None
            type_str = dtype.text.strip() if dtype is not None and dtype.text else ""
            has_coe = dev.find(".//CoE") is not None
            key = f"{type_str or name}"

            mapped = set()
            for kind, pdos in (("Rx", rx), ("Tx", tx)):
                for p in pdos:
                    pdo_rows.append({
                        "file": base, "device": key, "direction": kind,
                        "pdo_index": f"0x{p['index']:04X}" if p["index"] is not None else "",
                        "pdo_name": p["name"], "sm": p["sm"] or "",
                        "assignable": "no" if p["sm"] is not None else "yes",
                        "fixed_mapping": "yes" if p["fixed"] else "no",
                        "mandatory": "yes" if p["mandatory"] else "no",
                        "entries": len(p["entries"]),
                        "bits": p["bits"], "bytes": round(p["bits"] / 8, 2),
                    })
                    for e in p["entries"]:
                        if e["padding"]:
                            continue
                        mapped.add(e["index"])
                        obj_dir[e["index"]][kind] += 1
                        entry_rows.append({
                            "file": base, "device": key, "direction": kind,
                            "pdo_index": f"0x{p['index']:04X}" if p["index"] is not None else "",
                            "obj_index": f"0x{e['index']:04X}",
                            "obj_sub": e["sub"], "bitlen": e["bitlen"],
                            "obj_name": e["name"], "datatype": e["datatype"],
                            "known_as": KNOWN.get(e["index"], ""),
                        })
            is402 = {0x6040, 0x6041} <= mapped
            if is402:
                cia402_devices.add(key)
            for i in mapped:
                obj_devices[i].add(key)
                if is402:
                    obj_devices_402[i].add(key)

            dev_rows.append({
                "file": base, "vendor": vendor, "device": key, "name": name,
                "product_code": f"0x{product:X}" if product is not None else "",
                "revision": f"0x{revision:X}" if revision is not None else "",
                "coe": "yes" if has_coe else "no",
                "rxpdos": len(rx), "txpdos": len(tx),
                "cia402": "yes" if {0x6040, 0x6041} <= mapped else "no",
            })

    def write(fname, rows, fields):
        p = os.path.join(outdir, fname)
        with open(p, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=fields)
            w.writeheader()
            w.writerows(rows)
        print(f"  wrote {fname}: {len(rows)} rows")

    write("devices.csv", dev_rows,
          ["file", "vendor", "device", "name", "product_code", "revision",
           "coe", "rxpdos", "txpdos", "cia402"])
    write("pdos.csv", pdo_rows,
          ["file", "device", "direction", "pdo_index", "pdo_name", "sm",
           "assignable", "fixed_mapping", "mandatory", "entries", "bits", "bytes"])
    write("entries.csv", entry_rows,
          ["file", "device", "direction", "pdo_index", "obj_index", "obj_sub",
           "bitlen", "obj_name", "datatype", "known_as"])

    # The headline table: how many distinct devices map each object, so a high
    # percentage means "real drives consistently expose this".
    freq = []
    for idx, devs in sorted(obj_devices.items(), key=lambda kv: -len(kv[1])):
        freq.append({
            "obj_index": f"0x{idx:04X}",
            "known_as": KNOWN.get(idx, ""),
            "devices": len(devs),
            "pct_of_devices": round(100.0 * len(devs) / total_devices, 1) if total_devices else 0,
            "rx_mappings": obj_dir[idx]["Rx"],
            "tx_mappings": obj_dir[idx]["Tx"],
        })
    write("object_frequency.csv", freq,
          ["obj_index", "known_as", "devices", "pct_of_devices",
           "rx_mappings", "tx_mappings"])

    # The same table restricted to devices that actually implement CiA402.
    # This is the one to read: a high percentage here means real drives
    # consistently expose the object, undiluted by I/O terminals sharing the file.
    n402 = len(cia402_devices)
    freq402 = []
    for idx, devs in sorted(obj_devices_402.items(), key=lambda kv: -len(kv[1])):
        freq402.append({
            "obj_index": f"0x{idx:04X}",
            "known_as": KNOWN.get(idx, ""),
            "devices": len(devs),
            "pct_of_cia402_devices": round(100.0 * len(devs) / n402, 1) if n402 else 0,
            "rx_mappings": obj_dir[idx]["Rx"],
            "tx_mappings": obj_dir[idx]["Tx"],
        })
    write("cia402_object_frequency.csv", freq402,
          ["obj_index", "known_as", "devices", "pct_of_cia402_devices",
           "rx_mappings", "tx_mappings"])

    print(f"\n  {total_devices} devices with process data across {len(files)} file(s)")
    print(f"  {n402} of them implement CiA402 (controlword + statusword mapped)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
