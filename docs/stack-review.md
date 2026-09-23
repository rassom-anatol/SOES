# Stack review — findings

A read of the tree as it stands, against the plan in [`cia402-roadmap.md`](cia402-roadmap.md).
Ordered by what it changes, not by where it lives. Line references are to the
current working tree.

Findings carry a **Resolved** note where the work has since been done. Section 2
and section 4 are closed; sections 1, 3 and 5 are not, and the reason in each
case is that verifying the change needs a master and a LAN9252 on the wire.

---

## 1. Two findings that change the roadmap

### 1.1 The two-thread split in §3.2 is not safe with this HAL

[`esc_hw.c:294-341`](../soes/hal/linux-lan9252-spidev/esc_hw.c#L294-L341) implements
a CSR access as a **stateful three-frame sequence** — write the command register,
read it back, read the data register — over a single global `spi_fd` with no lock,
against a single command register inside the chip. A PRAM transfer is the same shape
with more steps. Interleave an RT-thread PRAM access with a worker-thread SDO and
both transactions corrupt: the second write of `ESC_CSR_CMD_REG` lands while the
first is still outstanding, and each thread reads the other's result.

The rt-kernel XMC4 HAL that §3.2 models the design on does not have this problem,
because there the ESC is memory-mapped: one access is one bus cycle and is atomic
against an ISR by construction. Over SPI that property is gone, and nothing replaces it.

The same exposure exists one level up. `ESCvar`, the `MBX[]` buffers and the
application's `Obj` storage are all touched from both paths — `COE_pdoUnpack` writes
the same object storage that the SDO download path writes. The stack offers
`CC_ATOMIC_*` in [`cc.h`](../soes/include/sys/gcc/cc.h) for individual scalars and
**no critical-section abstraction at all**; there is no hook a port can fill in to
make a two-context design safe.

Two ways out, and the choice belongs in §3.2 before any of it is written:

- Serialise bus access behind a lock the RT thread can acquire without unbounded
  inversion (priority inheritance, or a lock the worker only ever holds for one
  complete transaction), and extend the same discipline to `ESCvar` and `Obj`.
- Keep mailbox handling on the cyclic thread and bound its work per cycle instead —
  e.g. one mailbox step per cycle — accepting slower SDO-Info in exchange for a
  single-threaded stack.

The second is smaller and removes the problem rather than managing it. The reason
§3.2 rejected it — that mailbox work is unbounded — is about *total* latency, not
per-cycle cost, and a step-per-cycle bound addresses the part that matters.

### 1.2 `watchdog_cnt = INT32_MAX` leaves the drive with no watchdog

§3.3 sets `.watchdog_cnt = INT32_MAX` on the reasoning that the ESC hardware SM
watchdog should be used instead of the software counter. **Nothing in this tree reads
the hardware watchdog.** [`ESC_WDstatus()`](../soes/esc.c#L223) is defined and called
from nowhere; register 0x0440 is never sampled.

The ESC does not change AL state by itself when the SM watchdog expires — it sets the
status bit and expects the application to react by dropping to SAFEOP+ERROR with
`ALERR_WATCHDOG` (0x001B). With the software counter disabled at
[`ecat_slv.c:186-204`](../soes/ecat_slv.c#L186-L204) and no hardware check, a master
that stops sending frames produces no reaction at all: the drive holds its last
setpoint indefinitely.

The sync error counter designed in §3.3.1 does not cover this. It tracks the *balance*
between SM2 events and SYNC0 edges; if both stop together — cable pulled, master
killed — the balance stays where it was and never trips.

What is needed, cyclically, before OP is entered on hardware:

- Read 0x0440 when the process data watchdog is enabled and outputs are present, and
  call `ESC_ALstatusgotoerror(ESCsafeop | ESCerror, ALERR_WATCHDOG)` on expiry.
- A DC liveness check distinct from the sync counter: if DC was activated and SYNC0
  has stopped arriving, that is `ALERR_FATALSYNCERROR` (0x002C), not a counter drift.
- Validate the watchdog configuration the master wrote (0x0400 divider, 0x0420 time).
  A master that disables it leaves the slave unprotected, and the slave currently
  cannot tell.

This is the most safety-relevant gap in the tree and it is not in the roadmap.

---

## 2. The CiA402 dictionary does not work as generated

Everything in this section is a defect in [`tools/gen_od.py`](../tools/gen_od.py), so
per §4.5 the fix goes in the generator, never in the output. None of it is caught by
[`tools/check_od.sh`](../tools/check_od.sh) — see §4 below for why.

`applications/cia402_drive/` has **no `CMakeLists.txt` and no `Obj` definition**, so the
generated object list is never compiled or linked by any build in the tree. That is why
these have gone unnoticed.

### 2.1 Every signed object is declared unsigned in the ESI

[`esi_type()`](../tools/gen_od.py#L507) maps a PDO entry to an ESI data type on **bit
width alone**:

```python
return {1: "BOOL", 8: "USINT", 16: "UINT", 32: "UDINT", 64: "ULINT"}.get(bits, "UDINT")
```

So 0x607A, 0x6064, 0x606C and 0x60F4 emit `UDINT`; 0x6071 and 0x6077 emit `UINT`;
0x6060 and 0x6061 emit `USINT`. Meanwhile
[`slave_objectlist.c`](../applications/cia402_drive/generated/slave_objectlist.c)
correctly carries `DTYPE_INTEGER32` and `DTYPE_INTEGER16` for the same objects.

A master takes PDO interpretation from the ESI, so a negative position reads as roughly
4.29e9. This is the single most damaging item in the list because it fails *silently* —
the state machine reaches OP and the numbers are simply wrong in one direction.

The generator already knows each object's signedness; `esi_type` needs the object's
declared type, not its width.

**Resolved.** `esi_type` takes the entry rather than its bit count and looks the name
up in a new `ESI_TYPES` table keyed by the YAML type. A type with no ESI equivalent is
now an error at generation time rather than a silent `UDINT`.

### 2.2 No `<Fmmu>` elements in the generated ESI

Both retained reference ESIs declare them —
[`lan9252_diag/slave.xml:980-981`](../applications/lan9252_diag/slave.xml#L980-L981)
has `<Fmmu>Outputs</Fmmu><Fmmu>Inputs</Fmmu>`, and the XMC4 one adds
`<Fmmu>MBoxState</Fmmu>`. The generator emits none, and `grep -c Fmmu tools/gen_od.py`
returns 0.

Note that the `xmllint --schema` check §4.5 plans **will not catch this**: `<Fmmu>` is
optional in `EtherCATInfo.xsd`. Schema validity is not the property that matters here.

**Resolved.** Both elements are emitted ahead of the `<Sm>` block, and `check_od.sh`
asserts their presence for exactly the reason above.

### 2.3 0x1C00 is missing from the object dictionary

Sync Manager Communication Type is present in both retained demos
([`lan9252_diag/slave_objectlist.c:119-125`](../applications/lan9252_diag/slave_objectlist.c#L119-L125))
and is expected by masters enumerating a CoE device. The generator has no concept of it.

**Resolved.** Emitted as an ARRAY of four `UNSIGNED8`, describing the fixed SOES
layout: two mailbox SyncManagers, then outputs and inputs.

### 2.4 The BootStrap block contradicts the boot mailbox configuration

[`od.yaml`](../applications/cia402_drive/od.yaml) carries
`bootstrap: "0010800080108000"`, copied from the demo. That decodes to MBX0 at 0x1000
length 0x80, MBX1 at 0x1080 length 0x80 — a 128-byte layout. The generated
[`ecat_options.h`](../applications/cia402_drive/generated/ecat_options.h) has
`MBXSIZEBOOT 256` and `MBX1_sma_b 0x1100`.

Entering BOOT therefore fails `ESC_checkmbx` and yields `ALERR_INVALIDBOOTMBXCONFIG`.

Beyond the wrong value, this is the exact failure mode §4.1 exists to prevent: the
string is hand-carried SyncManager arithmetic living in the source-of-truth file. It
should be **derived** from `mailbox.size_boot` and the boot mailbox addresses, like
every other SM number in the generator.

**Resolved twice over.** The `bootstrap` key is gone from `od.yaml` and a `bootstrap()`
function derives the four words from the mailbox configuration; and since FoE is now
off (§2.7) no `<BootStrap>` element is emitted at all. `check_od.sh` decodes the
element against the `MBX*_b` constants whenever one is present, so re-enabling FoE
cannot reintroduce the mismatch.

### 2.5 An SDO read of 0x1008 dereferences NULL

The generator emits `VISIBLE_STRING` objects with `data = NULL` and `value = 0`
([`gen_od.py:280-286`](../tools/gen_od.py#L280-L286)):

```c
{0x00, DTYPE_VISIBLE_STRING, 72, ATYPE_RO, acName1008_00, 0, NULL},
```

"cmc_drive" is 9 bytes, which exceeds the 4-byte expedited limit, so the upload takes
the normal-response path at [`esc_coe.c:414`](../soes/esc_coe.c#L414) and calls
`copy2mbx(NULL, dst, 9)` — a `memcpy` from NULL. Masters read 0x1008 during a scan.

0x1009 and 0x100A are 3 bytes each, so they go expedited and quietly return
`0x00000000` instead of their version strings — wrong, but not fatal, which is why the
failure mode differs between the three.

The correct shape is in the retained reference,
[`rtl_xmc4_dynpdo/slave_objectlist.c:123`](../applications/rtl_xmc4_dynpdo/slave_objectlist.c#L123):
the string literal goes in the `data` field.

**Resolved.** A string object with no backing variable now emits
`(void *)"cmc_drive"` as its data pointer, and `check_od.sh` fails on any
`DTYPE_VISIBLE_STRING` entry left with a NULL one.

### 2.6 0x6502 advertises a mode that does not exist

`0x000003A0` sets bit 5 — homing mode — alongside csp (7), csv (8) and cst (9). There
are no homing objects anywhere in the dictionary. The v1 set described in §4.2 is
csp/csv/cst, so the value is `0x380`.

**Resolved.** `od.yaml` now declares `0x00000380`. `check_od.sh` decodes the value
bit by bit against a table of the objects each mode needs, and fails both on a mode
whose objects are absent and on a bit the table does not recognise.

### 2.7 FoE is enabled with no handler configured

`USE_FOE 1` in the generated options, `<FoE/>` in the generated ESI, and **no call to
`FOE_config()` anywhere in the tree**. `foe_cfg` stays NULL and the first FoE request
dereferences it at [`esc_foe.c:70`](../soes/esc_foe.c#L70).

The ESI actively invites the request. Until there is a firmware-update story, set
`USE_FOE 0` and drop both `<FoE/>` and `<BootStrap>` from the generated ESI — which
also makes §2.4 moot in the interim.

**Resolved.** One `mailbox.foe` key in `od.yaml`, currently `false`, drives `USE_FOE`,
`<FoE/>` and `<BootStrap>` together so the three cannot disagree. `check_od.sh` also
requires a call to `FOE_config()` to exist before the ESI is allowed to advertise FoE,
which is the condition that actually has to hold before the key can be flipped.

### 2.8 The ARRAY/RECORD heuristic contradicts the profile

[`gen_od.py:115-121`](../tools/gen_od.py#L115-L121) picks `OTYPE_ARRAY` whenever every sub-entry
shares a type, and `OTYPE_RECORD` otherwise. That makes 0x1018 an ARRAY (the Identity
object is a RECORD, data type IDENTITY) and makes 0x608F, 0x6091 and 0x6092 ARRAYs,
where the drive profile defines all three as RECORDs. 0x2000 also lands as an ARRAY
with two differently-named sub-entries.

The consequence is worse than a mislabelled object code. For ARRAY,
[`esc_coe.c:1325-1330`](../soes/esc_coe.c#L1325-L1330) reports **sub 0's** data type —
`UNSIGNED8` — as the array element type in the Get Object Description response. So the
object description is wrong twice: wrong object code and wrong element type.

Object code is a property of the object as the profile defines it, not something to
infer from the shape of the sub-entries. It belongs in `od.yaml` as an explicit field
with a sensible default.

**Resolved.** The heuristic is deleted. `object_code` is an `od.yaml` field defaulting
to `var` for a scalar and `record` for anything with sub-entries — which is what the
profile specifies for every object currently in the dictionary, so nothing needs to
state it explicitly yet. `array` is now opt-in, which is the right way round given what
it costs in the object description.

---

## 3. Efficiency — roughly 20% of the cycle is recoverable now

The measured figures in §3.4 (median 141 µs, p99 220 µs, ~15.7 SPI transfers per cycle)
are the baseline these are measured against.

### 3.1 The per-cycle local-time read is dead weight

**Correction, and not resolved.** The saving described below does not exist as stated.
`ESC_ALeventread()` is itself `ESC_read (ESCREG_ALEVENT, ...)`
([`esc.c:129`](../soes/esc.c#L129)), and this HAL appends an ALEVENT tail read to
*every* `ESC_read` — so the substitution costs 3 frames for the CSR access plus 3 for
the tail, exactly the 6 it replaces, and reads ALEVENT twice in the process. The
opportunity is real but it is one change, not two: the local-time read has to go *and*
the tail has to be suppressed on the replacement, which is the mechanism §3.2
describes. Left for the hardware session, because the whole claim is a measurement.

[`ecat_slv_poll()`](../soes/ecat_slv.c#L306) reads `ESCREG_LOCALTIME` into `ESCvar.Time`
every cycle. **Nothing in this tree reads `ESCvar.Time`** — the only other occurrence is
in `soes/doc/tutorial.txt`.

That read costs one CSR access (3 SPI frames) plus the unconditional ALEVENT tail
(3 more) = **6 of the ~15.7 transfers per cycle**.

It cannot simply be deleted, because the rest of the poll depends on the ALEVENT
refresh it incidentally performs. Replacing it with a direct
`ESCvar.ALevent = ESC_ALeventread()` keeps the refresh and costs 3 transfers instead of
6 — around 25-30 µs off a 141 µs cycle, for a two-line change. That is larger than
anything left on the §3.4 optimisation list.

### 3.2 The ALEVENT tail can go from the PRAM path without the deferred audit

§3.4 defers eliminating the tail read
([`esc_hw.c:626`](../soes/hal/linux-lan9252-spidev/esc_hw.c#L626),
[`:663`](../soes/hal/linux-lan9252-spidev/esc_hw.c#L663)) on the grounds that it
requires auditing every place in `esc.c` that assumes `ESCvar.ALevent` freshness.

That audit is not needed for the **process-data** path specifically:

- `DIG_process` tests `ALevent & ESCREG_ALEVENT_SM2` *before* calling `rxpdo_read_sm2`,
  so the read cannot invalidate its own precondition.
- The TxPDO branch does not test `ALevent` at all.
- §3.2's loop re-reads ALEVENT explicitly at the top of every cycle.

A per-call suppression flag on the two PRAM transfers recovers a further ~6 transfers
per cycle with no reasoning about `esc.c` required. The general case can stay deferred.

### 3.3 Unconditional timing instrumentation on the hot path

`spi_xfer` takes two `clock_gettime` calls per transfer
([`esc_hw.c:184-191`](../soes/hal/linux-lan9252-spidev/esc_hw.c#L184-L191)), so roughly
30 per cycle. Through the vDSO this is under a microsecond in total and not worth
removing — but it is permanent instrumentation with no compile-time switch, in the one
path being tuned to microseconds. It should be behind the same kind of flag as
`ESC_DEBUG`.

**Resolved.** Both `clock_gettime` calls and the two counters are inside
`#if ESC_HW_SPI_STATS`, which defaults to 1 so the diagnostic application keeps
working; `-DESC_HW_SPI_STATS=0` removes them from the cyclic path and makes the two
accessors return zero.

---

## 4. `check_od.sh` validates the two things that cannot drift

The script proves (a) that regenerating from the YAML reproduces the committed files
and (b) that four SyncManager start addresses agree between the ESI and
`ecat_options.h`. Both are structurally guaranteed by the generator: (a) is idempotency
of a pure function, (b) reads the same YAML keys through two code paths in the same run.

It prints `all checks passed` over every defect in §2.

The generator is now the single point of failure for four artifacts and has no test of
what it *emits*. The checks that would have caught §2 are all cheap:

| Check | Catches |
|---|---|
| Each ESI `<DataType>` matches the signedness and width of the object's `DTYPE_*` | §2.1 |
| ESI contains `<Fmmu>Outputs</Fmmu>` and `<Fmmu>Inputs</Fmmu>` | §2.2 |
| Object list contains 0x1000, 0x1018, 0x1C00, 0x1C12, 0x1C13 | §2.3 |
| `<BootStrap>` decodes to the `MBX*_b` constants in `ecat_options.h` | §2.4 |
| Every `DTYPE_VISIBLE_STRING` entry has a non-NULL `data` pointer | §2.5 |
| Every bit set in 0x6502 has its mode's objects present | §2.6 |
| `USE_FOE` and the presence of `<FoE/>` agree | §2.7 |
| `gcc -c` on the generated object list | link-level and type errors generally |

**Resolved.** All eight are implemented, plus a check that the object list is in
ascending index order — `SDO_findobject` stops at the first index past the one it wants,
so an out-of-order entry is simply invisible. Every one of them failed on the tree as
reviewed and passes now.

Two structural gaps alongside it:

- **`applications/cia402_drive` is not a build target.** It needs a `CMakeLists.txt` and
  at minimum a translation unit defining `Obj`, so that the generated dictionary is
  compiled by the ordinary build rather than only by a CI one-liner.

  **Resolved.** `objects.c` defines `Obj` and a `CMakeLists.txt` builds the pair as a
  static library. It is a second build *configuration* rather than a second target,
  because `soes` compiles against whichever application supplies `ecat_options.h`;
  `SOES_DEMO` is now a cache variable and selects between the two.

- **The CI workflow does not run.** [`build.yml`](../.github/workflows/build.yml) targets
  the `ubuntu-20.04` runner image, which has been retired, so the job has not executed
  in some time. It also never invoked `check_od.sh` and never built the drive
  application. The §4.1 promise of "a CI check that regeneration produces no diff" does
  not currently exist.

  **Resolved.** `ubuntu-latest` and `actions/checkout@v4`, running `check_od.sh` first
  and then both build configurations.

---

## 5. Protocol features absent from the stack

These are not bugs — the stack never claimed them — but each is work that has to be
scheduled or consciously dropped, and two of them are visible to a master.

### 5.1 No Emergency messages, at any level

There is no EMCY code in the tree: not a disabled option, absent. A drive fault reaches
the master only as a TxPDO bit plus 0x603F on the next cycle, never as an unsolicited
message.

For a CiA402 drive this is the notable omission. It is also a decision that interacts
with §4.2: an EMCY carries an error register value, which conventionally lives in
0x1001, and that object is not in the dictionary either. Decide before the process image
is frozen, because adding 0x1001 later is cheap but adding EMCY changes what the
mailbox path has to do while the cyclic thread is running.

### 5.2 No validation of the master's DC configuration

[`ESC_checkDC`](../soes/esc.c#L268) reads 0x0981, and if the sync unit is active it
delegates the entire decision to the application's `esc_check_dc_handler`, returning
`ALERR_DCINVALIDSYNCCFG` if none is registered.

Nothing checks the SYNC0 cycle time the master wrote against what the slave can
actually sustain, the sync type, or the shift times. The AL status code set is rich
here — 0x30 invalid sync config, 0x35 invalid cycle time, 0x36 invalid SYNC0 cycle
time, 0x37 invalid SYNC1 cycle time, all defined in
[`esc.h:138-145`](../soes/esc.h#L138-L145) and none of them used — so a
misconfiguration produces one undifferentiated error whatever the cause.

The `dc_checker()` §3.3 copies from the reference application sets `dcsync` and the
counter limit; it validates nothing. Once 0x1C32:05 carries an honest minimum cycle
time, comparing the master's SYNC0 period against it is a few lines and turns a class
of silent misconfiguration into a specific diagnostic.

### 5.3 0x1C32 / 0x1C33 are absent

Known and scheduled in §3.3. Worth recording here only because §2 shows the generator is
where they must be added, and because the +3 / −1 weighting specified in §3.3.1 is the
ETG.1020 semantics — the plan is right, it just has nowhere to live yet.

### 5.4 Mailbox depth

Three buffers with one transfer outstanding, and an incoming frame is **dropped** while
an xoe transfer is in progress ([`esc.c:657-659`](../soes/esc.c#L657-L659)), relying on the
master's repeat mechanism to re-send.

This is adequate for SDO traffic at PREOP and is not worth changing now. It does become
the throughput limit for an SDO-Info walk over a four-axis dictionary — which is the
case `MBXSIZE 256` was raised for in §4.3 — so if that walk turns out slow, the mailbox
depth is the thing to look at, not the mailbox size.

---

## Summary of actions

**Before any hardware run in OP**

1. Decide the threading model (§1.1) — it determines whether a lock is needed
   everywhere or nowhere.
2. Wire a real watchdog: 0x0440 check, DC liveness check, watchdog configuration
   validation (§1.2).

**Before the ESI ships** — all in `gen_od.py`. ~~Done.~~

3. ~~Signed ESI data types (§2.1).~~
4. ~~`<Fmmu>` elements (§2.2).~~
5. ~~0x1C00 (§2.3).~~
6. ~~Derive `<BootStrap>`, or disable FoE and drop it (§2.4, §2.7).~~ Both.
7. ~~String objects carry their data pointer (§2.5).~~
8. ~~0x6502 to 0x380 (§2.6).~~
9. ~~Explicit object code per object (§2.8).~~
10. ~~Extend `check_od.sh` to the table in §4; make `cia402_drive` build; fix the CI
    runner image.~~

**Cheap performance — needs hardware after all**

11. Delete the local-time read *and* suppress the tail on whatever replaces it. Not the
    two-line change §3.1 described; see the correction there. Both halves are one
    change and the benefit is a measurement, so it belongs in a session with the
    LAN9252 attached.
12. Suppress the ALEVENT tail on PRAM transfers (§3.2). Same session.

**Decide, then schedule**

13. Emergency messages and 0x1001 (§5.1).
14. DC configuration validation (§5.2).
