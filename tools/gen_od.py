#!/usr/bin/env python3
"""Generate the SOES object dictionary from a declarative description.

Reads od.yaml and writes slave_objectlist.c, utypes.h and ecat_options.h.

Why this exists rather than hand-editing the C: those three files, the ESI and
the SII image all restate the same facts, and the stack punishes disagreement
silently. ESC_checkSM23 refuses PREOP->SAFEOP on any SyncManager mismatch with
an AL status code that has to be reverse-engineered; SDO_findobject assumes the
object list is sorted ascending and misbehaves if it is not; a missing sentinel
runs off the end of the array. All of it is mechanical, and all of it is
exactly what drifted in the demo this fork inherited, where the vendor tool's
project file disagreed with both the C and the XML about the PDO layout.

The generated files are committed. Neither the standalone build nor the
consuming application should need Python, and CI can check that regenerating
produces no diff.

Usage:  gen_od.py <od.yaml> <output-dir>
"""

import os
import sys
import textwrap

import yaml

# --------------------------------------------------------------------------
# type table: YAML name -> (SOES DTYPE, bit length, C type, signed)
# --------------------------------------------------------------------------
TYPES = {
    "bool":   ("DTYPE_BOOLEAN",        1,  "uint8_t"),
    "i8":     ("DTYPE_INTEGER8",       8,  "int8_t"),
    "u8":     ("DTYPE_UNSIGNED8",      8,  "uint8_t"),
    "i16":    ("DTYPE_INTEGER16",     16,  "int16_t"),
    "u16":    ("DTYPE_UNSIGNED16",    16,  "uint16_t"),
    "i32":    ("DTYPE_INTEGER32",     32,  "int32_t"),
    "u32":    ("DTYPE_UNSIGNED32",    32,  "uint32_t"),
    "i64":    ("DTYPE_INTEGER64",     64,  "int64_t"),
    "u64":    ("DTYPE_UNSIGNED64",    64,  "uint64_t"),
    "real32": ("DTYPE_REAL32",        32,  "float"),
    "string": ("DTYPE_VISIBLE_STRING", 0,  "char"),
}

# IEC 61131 type names as the ESI uses them. A master takes its PDO
# interpretation from here, so this table has to follow the declared type and
# not the bit width: describing a signed object as unsigned reaches OP without
# complaint and then reports -1 as 4294967295.
ESI_TYPES = {
    "bool":   "BOOL",
    "i8":     "SINT",
    "u8":     "USINT",
    "i16":    "INT",
    "u16":    "UINT",
    "i32":    "DINT",
    "u32":    "UDINT",
    "i64":    "LINT",
    "u64":    "ULINT",
    "real32": "REAL",
}

ACCESS = {
    "ro":    "ATYPE_RO",
    "rw":    "ATYPE_RW",
    "rwpre": "ATYPE_RWpre",
    "wo":    "ATYPE_WO",
}

# Object code, as the profile defines it for the object rather than as
# something to infer from the shape of its sub-entries. Getting it wrong is
# not merely a mislabelled code: for an ARRAY, esc_coe.c reports sub 0's data
# type as the element type in the Get Object Description response, so a RECORD
# described as an ARRAY is wrong twice over.
OBJECT_CODES = {
    "var":    "OTYPE_VAR",
    "array":  "OTYPE_ARRAY",
    "record": "OTYPE_RECORD",
}


def die(msg):
    print(f"gen_od: {msg}", file=sys.stderr)
    sys.exit(1)


def cident(s):
    """A C identifier fragment from an arbitrary name."""
    out = "".join(ch if ch.isalnum() else "_" for ch in s)
    return out.strip("_")


class Object:
    """One dictionary entry, already resolved for a specific axis."""

    def __init__(self, index, spec, axis, cfg):
        self.index = index
        self.axis = axis
        self.name = spec["name"]
        self.is_record = "subs" in spec
        self.subs = []

        code = spec.get("object_code", "record" if self.is_record else "var")
        if code not in OBJECT_CODES:
            die(f"0x{index:04X}: unknown object_code {code!r}")
        if (code == "var") != (not self.is_record):
            die(f"0x{index:04X}: object_code {code!r} does not match whether "
                f"the object has sub-entries")
        self.otype = OBJECT_CODES[code]

        if self.is_record:
            for sub in spec["subs"]:
                self.subs.append(self._sub(sub, cfg))
        else:
            self.subs.append(self._sub(dict(spec, sub=0), cfg))

    def _sub(self, spec, cfg):
        t = spec.get("type", "u32")
        if t == "record":
            die(f"0x{self.index:04X}: nested records are not supported")
        if t not in TYPES:
            die(f"0x{self.index:04X}: unknown type {t!r}")
        dtype, bits, ctype = TYPES[t]

        value = spec.get("value", 0)
        # "@name" pulls a value from the device block, so identity lives in one
        # place rather than being restated per object.
        if isinstance(value, str) and value.startswith("@"):
            value = cfg["device"][value[1:]]

        if t == "string":
            bits = 8 * (len(str(value)) if value else 1)

        var = spec.get("var")
        return {
            "sub": spec.get("sub", 0),
            "name": spec["name"],
            "type": t,
            "dtype": dtype,
            "bits": bits,
            "ctype": ctype,
            "access": ACCESS[spec.get("access", "ro")],
            "value": value,
            "var": var,
        }

    @property
    def maxsub(self):
        return max(s["sub"] for s in self.subs)

    def cname(self):
        return f"SDO{self.index:04X}"


def build_objects(cfg):
    """Expand the YAML object list into concrete per-axis objects."""
    naxes = cfg["axes"]["count"]
    stride = cfg["axes"]["stride"]
    objs = []

    for spec in cfg["objects"]:
        base = spec["index"]
        if spec.get("axis"):
            for n in range(naxes):
                objs.append(Object(base + n * stride, spec, n, cfg))
        else:
            objs.append(Object(base, spec, None, cfg))

    # SDO_findobject scans assuming ascending order; a sort here is the one
    # place that invariant needs to hold.
    objs.sort(key=lambda o: o.index)

    seen = set()
    for o in objs:
        if o.index in seen:
            die(f"duplicate index 0x{o.index:04X}")
        seen.add(o.index)
    return objs


def resolve_pdo(cfg, pdo, objs, axis):
    """Turn a PDO entry list into mapping words plus a running bit offset."""
    by_index = {o.index: o for o in objs}
    stride = cfg["axes"]["stride"]
    entries = []
    bits = 0

    for e in pdo["entries"]:
        if "pad" in e:
            entries.append({"index": 0, "sub": 0, "bits": e["pad"],
                            "name": "padding", "type": None, "var": None})
            bits += e["pad"]
            continue

        idx = e["object"] + axis * stride
        sub = e.get("sub", 0)
        obj = by_index.get(idx)
        if obj is None:
            die(f"PDO maps 0x{idx:04X} which is not in the dictionary")
        match = [s for s in obj.subs if s["sub"] == sub]
        if not match:
            die(f"PDO maps 0x{idx:04X}:{sub:02X} which does not exist")
        s = match[0]

        entries.append({"index": idx, "sub": sub, "bits": s["bits"],
                        "name": e.get("name", s["name"]), "type": s["type"],
                        "var": s["var"]})
        bits += s["bits"]

    if bits % 8:
        die(f"PDO 0x{pdo['index']:04X} is {bits} bits, not a whole number of "
            f"bytes -- add a pad entry")
    return entries, bits // 8


# SyncManager control byte, per ETG.1000.4 register 0x0804+n*8:
#   bits 1:0  operation mode   00 buffered, 10 mailbox
#   bits 3:2  direction        00 read by master, 01 written by master
#   bit  4    interrupt in ECAT
#   bit  5    interrupt in PDI
#   bit  6    watchdog trigger enable
SMC_OUTPUTS = 0x24              # buffered, master writes, PDI interrupt
SMC_INPUTS = 0x20               # buffered, master reads, PDI interrupt
SMC_WATCHDOG = 0x40


def sm_control(cfg):
    """Control bytes for the two process data SyncManagers.

    The watchdog trigger bit is the whole reason this is computed rather than
    written down. Without it the SyncManager does not feed the ESC process data
    watchdog, so register 0x0440 reads expired no matter how much process data
    arrives and any watchdog check built on it is inert -- measured on hardware,
    where SM2 read back 0x24 and 0x0440 stayed 0 while frames flowed every 2 ms
    against a 100 ms timeout.

    It has to be right in two places at once: ESC_checkSM23 compares the byte
    the master wrote, which comes from the ESI, against the compiled constant,
    and refuses PREOP->SAFEOP if they differ. Emitting both from here is what
    makes that impossible to get wrong.
    """
    out = SMC_OUTPUTS
    if cfg["sync_managers"].get("watchdog", True):
        out |= SMC_WATCHDOG
    return out, SMC_INPUTS


def compute_sm(cfg, rx_bytes, tx_bytes):
    """Check the SyncManager layout against the constraint esc.c enforces.

    A buffered SyncManager holds three copies, so SM2 needs 3*length of space
    before SM3 starts. esc.c:752 refuses the SAFEOP transition otherwise, and
    the failure surfaces as an AL status code rather than anything legible.
    """
    sm = cfg["sync_managers"]
    reserve = cfg["axes"]["reserve"]
    rx_res = rx_bytes * reserve
    tx_res = tx_bytes * reserve

    if sm["outputs"] + 3 * rx_res > sm["inputs"]:
        die(f"SM2 at 0x{sm['outputs']:04X} reserving {rx_res} bytes needs "
            f"0x{sm['outputs'] + 3*rx_res:04X}, past SM3 at 0x{sm['inputs']:04X}")
    top = sm["inputs"] + 3 * tx_res
    if top > 0x2000:
        die(f"SM3 reservation ends at 0x{top:04X}, past the 0x2000 top of "
            f"LAN9252 process RAM")

    return {"rx_reserved": rx_res, "tx_reserved": tx_res, "top": top}


# --------------------------------------------------------------------------
# emitters
# --------------------------------------------------------------------------

BANNER = """/*
 * GENERATED by tools/gen_od.py from {src}
 *
 * Do not edit. Edit the YAML and regenerate; the whole point of generating
 * these files is that the object list, the process image and the SyncManager
 * arithmetic cannot drift apart.
 */
"""


def emit_objectlist(cfg, objs, rx, tx, src):
    out = [BANNER.format(src=src)]
    out.append('#include "esc_coe.h"\n#include "utypes.h"\n#include <stddef.h>\n')

    # Names go in their own table so they land in flash rather than being
    # duplicated inline for every entry.
    names = {}
    for o in objs:
        for s in o.subs:
            key = f"acName{o.index:04X}_{s['sub']:02X}"
            names[key] = s["name"]
        names[f"acName{o.index:04X}"] = o.name

    out.append("\n/* object and entry names */")
    for k in sorted(names):
        out.append(f'static const char {k}[] = "{names[k]}";')

    def mapping_word(e):
        return (e["index"] << 16) | (e["sub"] << 8) | e["bits"]

    # PDO mapping objects. ATYPE_RO throughout: the process image is fixed,
    # so the master reads the mapping and does not reconfigure it.
    for pdo, entries in ((cfg["rxpdo"], rx), (cfg["txpdo"], tx)):
        idx = pdo["index"]
        out.append(f"\n/* {pdo['name']} */")
        out.append(f"static const char acNamePDO{idx:04X}[] = \"{pdo['name']}\";")
        out.append(f"const _objd SDO{idx:04X}[] =\n{{")
        out.append(f"   {{0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNamePDO{idx:04X}, "
                   f"{len(entries)}, NULL}},")
        for i, e in enumerate(entries, 1):
            w = mapping_word(e)
            out.append(f"   {{0x{i:02X}, DTYPE_UNSIGNED32, 32, ATYPE_RO, "
                       f"acNamePDO{idx:04X}, 0x{w:08X}, NULL}},   "
                       f"/* {e['name']} */")
        out.append("};")

    # SyncManager Communication Type. A master enumerating a CoE device reads
    # this to learn what each SM carries before it configures anything, so it
    # is not optional even though nothing in the slave consults it. The four
    # values are the fixed SOES layout: two mailbox SMs then outputs, inputs.
    out.append('\nstatic const char acName1C00[] = "SM Communication Type";')
    out.append("const _objd SDO1C00[] =\n{")
    out.append("   {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00, 4, NULL},")
    for sub, (kind, what) in enumerate(((1, "mailbox receive"),
                                        (2, "mailbox send"),
                                        (3, "process data output"),
                                        (4, "process data input")), 1):
        out.append(f"   {{0x{sub:02X}, DTYPE_UNSIGNED8, 8, ATYPE_RO, "
                   f"acName1C00, {kind}, NULL}},   /* {what} */")
    out.append("};")

    # SyncManager assignment, likewise fixed.
    for idx, assigned in ((0x1C12, cfg["rxpdo"]["index"]),
                          (0x1C13, cfg["txpdo"]["index"])):
        out.append(f"\nstatic const char acNameSM{idx:04X}[] = \"SM assignment\";")
        out.append(f"const _objd SDO{idx:04X}[] =\n{{")
        out.append(f"   {{0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acNameSM{idx:04X}, 1, NULL}},")
        out.append(f"   {{0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acNameSM{idx:04X}, "
                   f"0x{assigned:04X}, NULL}},")
        out.append("};")

    # Ordinary objects.
    for o in objs:
        out.append(f"\n/* 0x{o.index:04X} {o.name} */")
        out.append(f"const _objd {o.cname()}[] =\n{{")
        if o.is_record:
            out.append(f"   {{0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, "
                       f"acName{o.index:04X}, {o.maxsub}, NULL}},")
        for s in o.subs:
            if o.is_record and s["sub"] == 0:
                continue
            data = f"&Obj.{s['var']}" if s["var"] else "NULL"
            if s["var"] and o.axis is not None:
                data = f"&Obj.axis[{o.axis}].{s['var']}"
            val = s["value"]
            if s["type"] == "string":
                # A string lives in the data pointer, never the value field.
                # Anything longer than four bytes takes the normal-response
                # path in esc_coe.c, which copy2mbx's from data -- so a NULL
                # here is a memcpy from NULL during an ordinary master scan.
                if not s["var"]:
                    data = f'(void *)"{val}"'
                val = 0
            elif isinstance(val, str):
                val = 0
            out.append(f"   {{0x{s['sub']:02X}, {s['dtype']}, {s['bits']}, "
                       f"{s['access']}, acName{o.index:04X}_{s['sub']:02X}, "
                       f"{val}, {data}}},")
        out.append("};")

    # The master list. Ascending order and the sentinel are both load-bearing.
    out.append("\n/* Object list. Ascending index order is required by")
    out.append("   SDO_findobject; the 0xffff entry terminates the scan. */")
    out.append("const _objectlist SDOobjects[] =\n{")
    rows = [(o.index, o.otype, o.maxsub, f"acName{o.index:04X}", o.cname())
            for o in objs]
    rows.append((cfg["rxpdo"]["index"], "OTYPE_RECORD", len(rx),
                 f"acNamePDO{cfg['rxpdo']['index']:04X}",
                 f"SDO{cfg['rxpdo']['index']:04X}"))
    rows.append((cfg["txpdo"]["index"], "OTYPE_RECORD", len(tx),
                 f"acNamePDO{cfg['txpdo']['index']:04X}",
                 f"SDO{cfg['txpdo']['index']:04X}"))
    rows.append((0x1C00, "OTYPE_ARRAY", 4, "acName1C00", "SDO1C00"))
    for idx in (0x1C12, 0x1C13):
        rows.append((idx, "OTYPE_ARRAY", 1, f"acNameSM{idx:04X}", f"SDO{idx:04X}"))
    rows.sort(key=lambda r: r[0])
    for idx, ot, ms, nm, arr in rows:
        out.append(f"   {{0x{idx:04X}, {ot}, {ms}, 0, {nm}, {arr}}},")
    out.append("   {0xffff, 0xff, 0xff, 0xff, NULL, NULL}")
    out.append("};")
    return "\n".join(out) + "\n"


def emit_utypes(cfg, objs, src):
    """The Obj struct the dictionary points into.

    This is not an internal detail: it is the data contract between this stack
    and the application that consumes it, so its shape is a cross-repo
    interface.
    """
    out = [BANNER.format(src=src)]
    out.append("#ifndef __UTYPES_H__\n#define __UTYPES_H__\n")
    out.append('#include "cc.h"\n')

    per_axis, global_vars = {}, {}
    for o in objs:
        for s in o.subs:
            if not s["var"]:
                continue
            (per_axis if o.axis is not None else global_vars)[s["var"]] = s["ctype"]

    out.append("/* Per-axis process and parameter data. */")
    out.append("typedef struct\n{")
    for var in sorted(per_axis):
        member = var.split(".")[-1] if "." not in var else None
        if member is None:
            continue
        out.append(f"   {per_axis[var]:<10} {var};")
    out.append("} _Axis;\n")

    out.append("typedef struct\n{")
    # Grouped (dotted) globals become nested structs so the generated C can
    # write &Obj.Group.Member without further bookkeeping.
    groups = {}
    for var in sorted(global_vars):
        if "." in var:
            g, m = var.split(".", 1)
            groups.setdefault(g, {})[m] = global_vars[var]
        else:
            out.append(f"   {global_vars[var]:<10} {var};")
    for g in sorted(groups):
        out.append("   struct\n   {")
        for m in sorted(groups[g]):
            out.append(f"      {groups[g][m]:<10} {m};")
        out.append(f"   }} {g};")
    out.append(f"   _Axis      axis[{cfg['axes']['count']}];")
    out.append("} _Objects;\n")
    out.append("extern _Objects Obj;\n")
    out.append("#endif /* __UTYPES_H__ */")
    return "\n".join(out) + "\n"


def emit_options(cfg, rx_bytes, tx_bytes, rx_entries, tx_entries, sm, src):
    d = cfg["mailbox"]
    s = cfg["sync_managers"]
    reserve = cfg["axes"]["reserve"]
    out = [BANNER.format(src=src)]
    out.append("#ifndef ECAT_OPTIONS_H\n#define ECAT_OPTIONS_H\n")
    # FoE is off unless something calls FOE_config(): foe_cfg stays NULL and
    # esc_foe.c dereferences it on the first request. The ESI is what invites
    # that request, so the option and the description are emitted together.
    out.append(f"#define USE_FOE           {1 if cfg['mailbox'].get('foe') else 0}")
    out.append("#define USE_EOE           0\n")
    out.append(f"#define MBXSIZE           {d['size']}")
    out.append(f"#define MBXSIZEBOOT       {d['size_boot']}")
    out.append(f"#define MBXBUFFERS        {d['buffers']}\n")
    out.append(f"#define MBX0_sma          0x{s['mbx_out']:04X}")
    out.append(f"#define MBX0_sml          MBXSIZE")
    out.append(f"#define MBX0_sme          MBX0_sma+MBX0_sml-1")
    out.append(f"#define MBX0_smc          0x26")
    out.append(f"#define MBX1_sma          0x{s['mbx_in']:04X}")
    out.append(f"#define MBX1_sml          MBXSIZE")
    out.append(f"#define MBX1_sme          MBX1_sma+MBX1_sml-1")
    out.append(f"#define MBX1_smc          0x22\n")
    out.append(f"#define MBX0_sma_b        0x{s['mbx_out']:04X}")
    out.append(f"#define MBX0_sml_b        MBXSIZEBOOT")
    out.append(f"#define MBX0_sme_b        MBX0_sma_b+MBX0_sml_b-1")
    out.append(f"#define MBX0_smc_b        0x26")
    out.append(f"#define MBX1_sma_b        0x{s['mbx_in']:04X}")
    out.append(f"#define MBX1_sml_b        MBXSIZEBOOT")
    out.append(f"#define MBX1_sme_b        MBX1_sma_b+MBX1_sml_b-1")
    out.append(f"#define MBX1_smc_b        0x22\n")
    out.append(f"/* Reserved for {reserve} axes at {rx_bytes}/{tx_bytes} bytes each,")
    out.append(f"   so adding axes needs no ESI change. SM3 must start at or after")
    out.append(f"   SM2 + 3*{sm['rx_reserved']} = 0x{s['outputs'] + 3*sm['rx_reserved']:04X};")
    out.append(f"   the reservation ends at 0x{sm['top']:04X}, inside the 0x2000 top. */")
    smc_out, smc_in = sm_control(cfg)
    if smc_out & SMC_WATCHDOG:
        out.append("/* SM2 control bit 6 enables the watchdog trigger, without which")
        out.append("   the ESC process data watchdog at 0x0440 is never fed. */")
    out.append(f"#define SM2_sma           0x{s['outputs']:04X}")
    out.append(f"#define SM2_smc           0x{smc_out:02X}")
    out.append(f"#define SM2_act           1")
    out.append(f"#define SM3_sma           0x{s['inputs']:04X}")
    out.append(f"#define SM3_smc           0x{smc_in:02X}")
    out.append(f"#define SM3_act           1\n")
    out.append(f"#define MAX_MAPPINGS_SM2  {max(8, len(rx_entries) * reserve)}")
    out.append(f"#define MAX_MAPPINGS_SM3  {max(8, len(tx_entries) * reserve)}\n")
    out.append(f"#define MAX_RXPDO_SIZE    {sm['rx_reserved']}")
    out.append(f"#define MAX_TXPDO_SIZE    {sm['tx_reserved']}\n")
    out.append("#endif /* ECAT_OPTIONS_H */")
    return "\n".join(out) + "\n"


def emit_esi(cfg, objs, rx, tx, rx_bytes, tx_bytes, sm, src):
    """Emit the EtherCAT Slave Information file.

    Deliberately omits the <Dictionary> block. The mailbox declares
    SdoInfo="true", so a master enumerates the object dictionary over CoE at
    scan time rather than reading it from the XML; leaving it out removes a
    large block that would have to be kept in step with the C for no benefit.
    What the ESI must carry is the part a master cannot discover before it has
    configured the device: identity, SyncManager addresses, the PDO layout,
    distributed clock operation modes, and the EEPROM image data.

    The SyncManager addresses here and the constants in ecat_options.h come
    from the same source, which is the point: ESC_checkSM23 refuses
    PREOP->SAFEOP on any disagreement between them.
    """
    d = cfg["device"]
    s_ = cfg["sync_managers"]
    mbx = cfg["mailbox"]["size"]
    ee = cfg["eeprom"]
    foe = cfg["mailbox"].get("foe")
    smc_out, smc_in = sm_control(cfg)

    def pdo_block(tag, pdo, entries, sm_index):
        out = [f'      <{tag} Fixed="1" Sm="{sm_index}" Mandatory="1">']
        out.append(f'        <Index>#x{pdo["index"]:04X}</Index>')
        out.append(f'        <Name>{pdo["name"]}</Name>')
        for i, e in enumerate(entries, 1):
            out.append("        <Entry>")
            out.append(f'          <Index>#x{e["index"]:04X}</Index>')
            out.append(f"          <SubIndex>{e['sub']}</SubIndex>")
            out.append(f"          <BitLen>{e['bits']}</BitLen>")
            if e["index"] != 0:
                out.append(f"          <Name>{e['name']}</Name>")
                out.append(f"          <DataType>{esi_type(e)}</DataType>")
            out.append("        </Entry>")
        out.append(f"      </{tag}>")
        return out

    x = ['<?xml version="1.0" encoding="UTF-8"?>',
         "<!-- GENERATED by tools/gen_od.py. Do not edit; edit the YAML. -->",
         "<EtherCATInfo>",
         "  <Vendor>",
         f'    <Id>#x{d["vendor_id"]:X}</Id>',
         f'    <Name LcId="1033">{d["vendor_name"]}</Name>',
         "  </Vendor>",
         "  <Descriptions>",
         "    <Groups>",
         "      <Group>",
         f'        <Type>{d["group"]}</Type>',
         f'        <Name LcId="1033">{d["group_name"]}</Name>',
         "      </Group>",
         "    </Groups>",
         "    <Devices>",
         '      <Device Physics="YY">',
         f'      <Type ProductCode="#x{d["product_code"]:X}" '
         f'RevisionNo="#x{d["revision"]:X}">{d["name"]}</Type>',
         f'      <Name LcId="1033">{d["name"]}</Name>',
         f'      <GroupType>{d["group"]}</GroupType>',
         "      <Profile>",
         "        <ProfileNo>402</ProfileNo>",
         "      </Profile>",
         f'      <Sm ControlByte="#x26" DefaultSize="{mbx}" Enable="1" '
         f'StartAddress="#x{s_["mbx_out"]:04X}">MBoxOut</Sm>',
         f'      <Sm ControlByte="#x22" DefaultSize="{mbx}" Enable="1" '
         f'StartAddress="#x{s_["mbx_in"]:04X}">MBoxIn</Sm>',
         f'      <Sm ControlByte="#x{smc_out:02X}" Enable="1" '
         f'StartAddress="#x{s_["outputs"]:04X}">Outputs</Sm>',
         f'      <Sm ControlByte="#x{smc_in:02X}" Enable="1" '
         f'StartAddress="#x{s_["inputs"]:04X}">Inputs</Sm>']
    # FMMUs. Optional in EtherCATInfo.xsd, so nothing validates their
    # absence, but a master that finds none configures no process data.
    x.insert(x.index(f'      <Sm ControlByte="#x26" DefaultSize="{mbx}" Enable="1" '
                     f'StartAddress="#x{s_["mbx_out"]:04X}">MBoxOut</Sm>'),
             "      <Fmmu>Outputs</Fmmu>")
    x.insert(x.index(f'      <Sm ControlByte="#x26" DefaultSize="{mbx}" Enable="1" '
                     f'StartAddress="#x{s_["mbx_out"]:04X}">MBoxOut</Sm>'),
             "      <Fmmu>Inputs</Fmmu>")
    x += pdo_block("RxPdo", cfg["rxpdo"], rx, 2)
    x += pdo_block("TxPdo", cfg["txpdo"], tx, 3)
    x += ['      <Mailbox DataLinkLayer="true">',
          '        <CoE CompleteAccess="false" PdoUpload="false" SdoInfo="true"/>']
    if foe:
        x.append("        <FoE/>")
    x += ["      </Mailbox>",
          "      <Dc>"]
    # DC-Synchron first so it is the default operation mode; a master that
    # takes the first entry then gets DC rather than free-run.
    for name, desc, aa in (("DcSync0", "DC-Synchron", ee["assign_activate"]),
                           ("DcOff", "SM-Synchron", "#x0")):
        x += ["        <OpMode>",
              f"          <Name>{name}</Name>",
              f"          <Desc>{desc}</Desc>",
              f"          <AssignActivate>{aa}</AssignActivate>",
              '          <CycleTimeSync0 Factor="1">0</CycleTimeSync0>',
              '          <ShiftTimeSync0 Input="0">0</ShiftTimeSync0>',
              "        </OpMode>"]
    x += ["      </Dc>",
          "      <Eeprom>",
          f'        <ByteSize>{ee["byte_size"]}</ByteSize>',
          f'        <ConfigData>{ee["config_data"]}</ConfigData>']
    if foe:
        x.append(f"        <BootStrap>{bootstrap(cfg)}</BootStrap>")
    x += ["      </Eeprom>",
          "      </Device>",
          "    </Devices>",
          "  </Descriptions>",
          "</EtherCATInfo>"]
    return "\n".join(x) + "\n"


def bootstrap(cfg):
    """The boot mailbox description, as four little-endian 16-bit words.

    Derived rather than written down. ESC_checkmbx compares this against the
    MBX*_b constants and refuses BOOT with ALERR_INVALIDBOOTMBXCONFIG on a
    mismatch, and a hand-carried hex string is exactly the kind of restated
    SyncManager arithmetic this generator exists to eliminate.
    """
    s = cfg["sync_managers"]
    size = cfg["mailbox"]["size_boot"]
    words = (s["mbx_out"], size, s["mbx_in"], size)
    return "".join(w.to_bytes(2, "little").hex().upper() for w in words)


def esi_type(entry):
    t = ESI_TYPES.get(entry["type"])
    if t is None:
        die(f"0x{entry['index']:04X}:{entry['sub']:02X} has type "
            f"{entry['type']!r}, which has no ESI equivalent and so cannot be "
            f"mapped into a PDO")
    return t


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    src, outdir = sys.argv[1], sys.argv[2]

    with open(src) as fh:
        cfg = yaml.safe_load(fh)

    # The banner names the source. Use a repo-relative path so regenerating
    # from a different working directory produces identical bytes -- otherwise
    # the idempotency check in tools/check_od.sh can never pass.
    src = os.path.relpath(os.path.abspath(src),
                          os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

    objs = build_objects(cfg)
    rx, rx_bytes = resolve_pdo(cfg, cfg["rxpdo"], objs, 0)
    tx, tx_bytes = resolve_pdo(cfg, cfg["txpdo"], objs, 0)
    sm = compute_sm(cfg, rx_bytes, tx_bytes)

    os.makedirs(outdir, exist_ok=True)
    files = {
        "slave_objectlist.c": emit_objectlist(cfg, objs, rx, tx, src),
        "utypes.h": emit_utypes(cfg, objs, src),
        "ecat_options.h": emit_options(cfg, rx_bytes, tx_bytes, rx, tx, sm, src),
        "slave.xml": emit_esi(cfg, objs, rx, tx, rx_bytes, tx_bytes, sm, src),
    }
    for name, text in files.items():
        with open(os.path.join(outdir, name), "w") as fh:
            fh.write(text)

    print(f"gen_od: {len(objs)} objects, {cfg['axes']['count']} axis/axes")
    print(f"  RxPDO 0x{cfg['rxpdo']['index']:04X}: {len(rx)} entries, {rx_bytes} bytes"
          f"  (reserved {sm['rx_reserved']} for {cfg['axes']['reserve']} axes)")
    print(f"  TxPDO 0x{cfg['txpdo']['index']:04X}: {len(tx)} entries, {tx_bytes} bytes"
          f"  (reserved {sm['tx_reserved']})")
    print(f"  SM2 0x{cfg['sync_managers']['outputs']:04X}  "
          f"SM3 0x{cfg['sync_managers']['inputs']:04X}  "
          f"top 0x{sm['top']:04X}")
    for name in files:
        print(f"  wrote {os.path.join(outdir, name)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
