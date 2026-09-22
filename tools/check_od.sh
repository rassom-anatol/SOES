#!/bin/sh
# Check the generated object dictionary is consistent and up to date.
#
# Two failures this catches, both of which the stack otherwise punishes at
# runtime with an AL status code rather than anything legible:
#
#   1. The committed generated files no longer match the YAML, because someone
#      edited the C directly. Regeneration would silently revert their change.
#   2. The SyncManager addresses in the ESI disagree with the constants
#      compiled into ecat_options.h. ESC_checkSM23 refuses PREOP->SAFEOP on any
#      mismatch, since the master configures SMs from the ESI and the slave
#      checks them against its own build.
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

fail=0
for f in slave_objectlist.c utypes.h ecat_options.h slave.xml; do
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

echo "== cross-checking ESI against ecat_options.h =="
python3 - "$gen" <<'PY'
import re, sys, os
gen = sys.argv[1]
esi = open(os.path.join(gen, "slave.xml"), encoding="utf-8", errors="replace").read()
opt = open(os.path.join(gen, "ecat_options.h"), encoding="utf-8").read()

# SyncManager start addresses as the master will read them from the ESI
sm_esi = {}
for m in re.finditer(r'<Sm[^>]*StartAddress="#x([0-9A-Fa-f]+)"[^>]*>(\w+)</Sm>', esi):
    sm_esi[m.group(2)] = int(m.group(1), 16)

def define(name):
    m = re.search(r"#define\s+%s\s+0x([0-9A-Fa-f]+)" % name, opt)
    return int(m.group(1), 16) if m else None

pairs = [("MBoxOut", "MBX0_sma"), ("MBoxIn", "MBX1_sma"),
         ("Outputs", "SM2_sma"), ("Inputs", "SM3_sma")]

bad = 0
for esi_name, c_name in pairs:
    a, b = sm_esi.get(esi_name), define(c_name)
    status = "ok" if a == b else "MISMATCH"
    if a != b:
        bad = 1
    print(f"  {esi_name:8} ESI 0x{a:04X}  {c_name} 0x{b:04X}   {status}")

sys.exit(bad)
PY

echo "== all checks passed =="
