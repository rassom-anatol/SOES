#!/bin/sh
# Check the generated object dictionary is consistent and up to date.
#
# The generator is the single point of failure for four artifacts, so the
# checks come in two kinds:
#
#   1. Checks that the committed files still match the YAML. Catches someone
#      editing the generated C directly, whose change regeneration would then
#      silently revert.
#   2. Checks on what the generator *emits*. Idempotency proves nothing about
#      correctness -- a generator that emits the wrong ESI data type emits it
#      reproducibly. Everything under "inspecting the generated artifacts"
#      below exists because it was once wrong in a way that reached OP and
#      then produced bad numbers, or that a master trips over during a scan.
#
# Intended for CI. Exits non-zero on any disagreement.

set -e

root=$(cd "$(dirname "$0")/.." && pwd)
yaml="$root/applications/cia402_drive/od.yaml"
gen="$root/applications/cia402_drive/generated"
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

echo "== regenerating to a scratch directory =="
python3 "$root/tools/gen_od.py" "$yaml" "$tmp" >/dev/null

# The ESI is named after the device rather than generically, so ask the
# generator what it called it rather than restating the rule here.
esi=$(python3 -c "
import sys, yaml
sys.path.insert(0, '$root/tools')
from gen_od import esi_name
print(esi_name(yaml.safe_load(open('$yaml'))))
")

fail=0
for f in slave_objectlist.c utypes.h ecat_options.h "$esi"; do
    if ! diff -q "$gen/$f" "$tmp/$f" >/dev/null 2>&1; then
        echo "FAIL: $f differs from what the YAML generates"
        diff -u "$gen/$f" "$tmp/$f" | head -20 || true
        fail=1
    fi
done
if [ $fail -ne 0 ]; then
    echo "regenerate with: python3 tools/gen_od.py $yaml $gen"
    exit 1
fi
echo "  generated files are up to date"

echo "== compiling the generated object list =="
gcc -c -o /dev/null -Wall -Wextra -Wno-unused-parameter \
    -I "$root/soes" -I "$root/soes/include/sys/gcc" -I "$gen" \
    "$gen/slave_objectlist.c"
echo "  compiles clean"

echo "== inspecting the generated artifacts =="
python3 - "$gen" "$esi" <<'PY'
import os, re, sys

gen, esi_file = sys.argv[1], sys.argv[2]


def rd(name):
    with open(os.path.join(gen, name), encoding="utf-8", errors="replace") as fh:
        return fh.read()


esi, opt, c = rd(esi_file), rd("ecat_options.h"), rd("slave_objectlist.c")

fails = []


def check(name, problems):
    if problems:
        print(f"  FAIL  {name}")
        for p in problems:
            print(f"          {p}")
        fails.append(name)
    else:
        print(f"  ok    {name}")


def define(name):
    m = re.search(r"#define\s+%s\s+(?:0x([0-9A-Fa-f]+)|(\d+))" % name, opt)
    if not m:
        return None
    return int(m.group(1), 16) if m.group(1) else int(m.group(2))


# --- parse the generated C -------------------------------------------------

# Sub-entry tables, keyed by index then sub-index.
blocks = {}
for m in re.finditer(r"const _objd SDO([0-9A-Fa-f]{4})\[\]\s*=\s*\{(.*?)\n\};", c, re.S):
    subs = {}
    for e in re.finditer(r"\{0x([0-9A-Fa-f]{2}),\s*(DTYPE_\w+),\s*(\d+),\s*"
                         r"(ATYPE_\w+),\s*(\w+),\s*(.*?),\s*(.*?)\}\s*,",
                         m.group(2)):
        subs[int(e.group(1), 16)] = {
            "dtype": e.group(2), "bits": int(e.group(3)), "access": e.group(4),
            "value": e.group(6).strip(), "data": e.group(7).strip(),
        }
    blocks[int(m.group(1), 16)] = subs

# The master object list: index -> object code.
rows = []
for m in re.finditer(r"\{0x([0-9A-Fa-f]{4}),\s*(OTYPE_\w+),\s*(\d+),\s*\d+,\s*\w+,\s*\w+\}", c):
    rows.append((int(m.group(1), 16), m.group(2)))
codes = dict(rows)

# --- parse the ESI PDO entries ---------------------------------------------

esi_entries = []
for m in re.finditer(r"<Entry>(.*?)</Entry>", esi, re.S):
    body = m.group(1)

    def tag(t, body=body):
        g = re.search(r"<%s>(.*?)</%s>" % (t, t), body, re.S)
        return g.group(1).strip() if g else None

    idx = tag("Index")
    esi_entries.append({
        "index": int(idx.replace("#x", ""), 16) if idx else 0,
        "sub": int(tag("SubIndex") or 0),
        "bits": int(tag("BitLen") or 0),
        "datatype": tag("DataType"),
    })

# --- 1. ESI data types match the C object list -----------------------------
#
# A master takes its PDO interpretation from the ESI, so a signed object
# described as unsigned reads back as ~4.29e9 instead of a small negative
# number, with the state machine reaching OP and nothing complaining.

ESI_FOR_DTYPE = {
    "DTYPE_BOOLEAN": "BOOL",
    "DTYPE_INTEGER8": "SINT",
    "DTYPE_UNSIGNED8": "USINT",
    "DTYPE_INTEGER16": "INT",
    "DTYPE_UNSIGNED16": "UINT",
    "DTYPE_INTEGER32": "DINT",
    "DTYPE_UNSIGNED32": "UDINT",
    "DTYPE_INTEGER64": "LINT",
    "DTYPE_UNSIGNED64": "ULINT",
    "DTYPE_REAL32": "REAL",
}

bad = []
for e in esi_entries:
    if e["index"] == 0:          # padding carries no name or type
        continue
    sub = blocks.get(e["index"], {}).get(e["sub"])
    if sub is None:
        bad.append(f"ESI maps 0x{e['index']:04X}:{e['sub']:02X}, "
                   f"which the object list does not define")
        continue
    want = ESI_FOR_DTYPE.get(sub["dtype"])
    if want is None:
        bad.append(f"0x{e['index']:04X}:{e['sub']:02X} has {sub['dtype']}, "
                   f"which has no ESI equivalent in this check")
    elif e["datatype"] != want:
        bad.append(f"0x{e['index']:04X}:{e['sub']:02X} is {sub['dtype']} "
                   f"but the ESI says {e['datatype']}, expected {want}")
    if e["bits"] != sub["bits"]:
        bad.append(f"0x{e['index']:04X}:{e['sub']:02X} is {sub['bits']} bits "
                   f"but the ESI says {e['bits']}")
check("ESI data types match the object list", bad)

# --- 2. FMMU declarations --------------------------------------------------
#
# Optional in EtherCATInfo.xsd, so schema validation does not cover this.

bad = [f"ESI has no <Fmmu>{d}</Fmmu>" for d in ("Outputs", "Inputs")
       if f"<Fmmu>{d}</Fmmu>" not in esi]
check("ESI declares the output and input FMMUs", bad)

# --- 3. objects a scanning master expects ----------------------------------

REQUIRED = {0x1000: "Device Type", 0x1018: "Identity",
            0x1C00: "SM Communication Type",
            0x1C12: "SM2 PDO assignment", 0x1C13: "SM3 PDO assignment"}
bad = [f"0x{i:04X} {n} is not in the object list"
       for i, n in REQUIRED.items() if i not in codes]
check("mandatory communication objects present", bad)

# --- 4. ascending index order ----------------------------------------------
#
# SDO_findobject stops scanning at the first index greater than the one it
# wants, so an out-of-order entry is simply invisible.

indices = [i for i, _ in rows]
bad = [f"0x{b:04X} follows 0x{a:04X}"
       for a, b in zip(indices, indices[1:]) if b <= a]
check("object list is in ascending index order", bad)

# --- 5. the boot mailbox description matches the boot constants ------------
#
# ESC_checkmbx compares the two and refuses BOOT with
# ALERR_INVALIDBOOTMBXCONFIG. Only meaningful if a BootStrap block is emitted
# at all; with FoE disabled there is nothing to enter BOOT for.

m = re.search(r"<BootStrap>([0-9A-Fa-f]{16})</BootStrap>", esi)
bad = []
if m:
    raw = bytes.fromhex(m.group(1))
    got = [int.from_bytes(raw[i:i + 2], "little") for i in range(0, 8, 2)]
    want = [define("MBX0_sma"), define("MBXSIZEBOOT"),
            define("MBX1_sma"), define("MBXSIZEBOOT")]
    labels = ["MBX0 address", "MBX0 length", "MBX1 address", "MBX1 length"]
    bad = [f"<BootStrap> {lab} is 0x{g:04X}, ecat_options.h says 0x{w:04X}"
           for lab, g, w in zip(labels, got, want) if g != w]
check("boot mailbox description matches the boot constants", bad)

# --- 6. string objects have somewhere to read from -------------------------
#
# esc_coe.c takes the normal-response path for anything over four bytes and
# copy2mbx's from the data pointer, so a NULL one is a memcpy from NULL during
# an ordinary master scan of 0x1008.

bad = [f"0x{idx:04X}:{sub:02X} is VISIBLE_STRING with a NULL data pointer"
       for idx, subs in blocks.items() for sub, s in subs.items()
       if s["dtype"] == "DTYPE_VISIBLE_STRING" and s["data"] == "NULL"]
check("string objects carry a data pointer", bad)

# --- 7. 0x6502 only claims modes whose objects exist -----------------------
#
# Bit n of Supported Drive Modes means mode n+1 is supported. Advertising a
# mode the dictionary cannot serve invites the master to select it.

MODES = {
    0: ("pp  profile position", [0x607A, 0x6081]),
    1: ("vl  velocity", [0x6042]),
    2: ("pv  profile velocity", [0x60FF, 0x6083]),
    3: ("tq  profile torque", [0x6071, 0x6087]),
    5: ("hm  homing", [0x6098, 0x6099]),
    6: ("ip  interpolated position", [0x60C1]),
    7: ("csp cyclic sync position", [0x607A]),
    8: ("csv cyclic sync velocity", [0x60FF]),
    9: ("cst cyclic sync torque", [0x6071]),
}

bad = []
for idx, subs in blocks.items():
    if idx < 0x6000 or (idx - 0x6502) % 0x800:
        continue
    base = idx - 0x6502
    supported = int(subs[0]["value"], 0)
    for bit in range(32):
        if not supported & (1 << bit):
            continue
        if bit not in MODES:
            bad.append(f"0x{idx:04X} sets bit {bit}, which this check does "
                       f"not know -- add it to MODES")
            continue
        name, needed = MODES[bit]
        missing = [f"0x{n + base:04X}" for n in needed if n + base not in blocks]
        if missing:
            bad.append(f"0x{idx:04X} claims {name} but "
                       f"{', '.join(missing)} are absent")
check("0x6502 matches the objects present", bad)

# --- 8. FoE is described consistently --------------------------------------
#
# The ESI is what invites the request; foe_cfg is NULL unless FOE_config() is
# called, and the handler dereferences it without checking.

use_foe = define("USE_FOE")
declared = "<FoE" in esi
bad = []
if use_foe and not declared:
    bad.append("USE_FOE is 1 but the ESI does not declare FoE")
if declared and not use_foe:
    bad.append("the ESI declares FoE but USE_FOE is 0")
if declared and "FOE_config" not in c:
    bad.append("the ESI declares FoE, so something must call FOE_config()")
check("FoE support is described consistently", bad)

# --- 9. SyncManager addresses agree ----------------------------------------
#
# ESC_checkSM23 refuses PREOP->SAFEOP on any mismatch, since the master
# configures SMs from the ESI and the slave checks them against its own build.

sm_esi = {}
for m in re.finditer(r"<Sm\b[^>]*>(\w+)</Sm>", esi):
    attrs = m.group(0)
    addr = re.search(r'StartAddress="#x([0-9A-Fa-f]+)"', attrs)
    ctl = re.search(r'ControlByte="#x([0-9A-Fa-f]+)"', attrs)
    sm_esi[m.group(1)] = (int(addr.group(1), 16) if addr else None,
                          int(ctl.group(1), 16) if ctl else None)

bad = []
for esi_name, a_name, c_name in (("MBoxOut", "MBX0_sma", "MBX0_smc"),
                                 ("MBoxIn", "MBX1_sma", "MBX1_smc"),
                                 ("Outputs", "SM2_sma", "SM2_smc"),
                                 ("Inputs", "SM3_sma", "SM3_smc")):
    esi_addr, esi_ctl = sm_esi.get(esi_name, (None, None))
    for got, want, what in ((esi_addr, define(a_name), a_name),
                            (esi_ctl, define(c_name), c_name)):
        if got != want:
            bad.append(f"{esi_name}: the ESI says 0x{got:04X}, "
                       f"{what} says 0x{want:04X}")
check("SyncManager addresses and control bytes agree with the build", bad)

# --- 10. the output SyncManager feeds the watchdog --------------------------
#
# Bit 6 of SM2's control byte is Watchdog Trigger Enable. Without it the
# SyncManager does not feed the ESC process data watchdog, 0x0440 reads expired
# however much process data arrives, and any watchdog reaction built on it can
# never fire. Measured on hardware before this check existed.

_, smc_out = sm_esi.get("Outputs", (None, None))
bad = []
if smc_out is not None and not (smc_out & 0x40):
    bad.append(f"SM2 control byte is 0x{smc_out:02X}: bit 6 clear, so the "
               f"process data watchdog is never fed")
check("the output SyncManager triggers the watchdog", bad)

if fails:
    print(f"\n{len(fails)} check(s) failed")
    sys.exit(1)
PY

echo "== all checks passed =="
