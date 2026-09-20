# CiA402 EtherCAT drive interface — roadmap

## Context

This repository is a hard fork of the unmaintained [OpenEtherCATsociety/SOES](https://github.com/OpenEtherCATsociety/SOES) EtherCAT slave stack. It is being developed as a git submodule of **cmc**, a ROS 2 motion controller (TMC4671 FOC controller + TMC6200 gate driver) running under Ubuntu 26.04 and ROS 2 Lyrical.

**Hardware targets differ by build.** The EtherCAT build requires the custom carrier board being designed for the Compute Module 4, since the LAN9252 lives on it. The ROS 2 build carries no such requirement and runs on either a stock Raspberry Pi 4 or that same CM4 carrier. Both platforms use the BCM2711, so the peripheral analysis in this document — SPI1's fixed ALT4 assignment, AUX block behaviour, `gpiochip0` as the SoC bank — applies to either. This pairs with the single-transport rule in §5.3: a build targets one transport, and the EtherCAT choice implies the carrier.

The end state: **cmc presents itself as a CiA402-compatible servo drive**, commanded by an external EtherCAT master. ROS 2 DDS, EtherCAT (CoE) and later CANopen-over-CAN become three interchangeable transports onto one shared CiA402 core, with the SOES cyclic task running on a `SCHED_FIFO` thread inside cmc's existing `axis` executable.

Two things block that today. The Raspberry Pi HAL depends on **bcm2835**, which is GPLv2 with no linking exception plus a paid commercial licence — incompatible with cmc's Apache-2.0 licence. And the stack's `use_interrupt = 0` configuration makes Distributed Clocks structurally unreachable, which a motion controller cannot live with.

Sequencing is deliberate: fix and re-verify the standalone stack on real hardware first, then integrate.

### Licence position

| Component | Licence | Consequence |
|---|---|---|
| bcm2835 | GPLv2, **no** linking exception, + paid commercial | Remove entirely |
| lgpio (`lg`) | Unlicense / public domain | Safe; remains in cmc for its BSP pins |
| spidev / gpiochip UAPI | Kernel syscall interface | No obligation |
| SOES | GPLv2 **with** linking exception | Links into Apache-2.0 cmc provided this fork's source stays published |
| CiA402 sources | Specification only | Work from **ETG.6010** / IEC 61800-7-201. No GPL reference implementations are read or reused |

The SOES linking exception is load-bearing — it is what permits linking into an Apache-2.0 application at all. Preserve it when modifying this tree.

---

## Phase 0 — Prerequisites

### Master and test topology

The authoritative master is **TwinCAT**, running on the Windows boot of the development workstation (Linux and Windows are on separate NVMe drives, so the two boots are mutually exclusive). The slave runs on separate hardware, so master and slave never contend for one machine. Production hardware is the CM4 carrier. Phase 1 bring-up need not wait for it if that is preferable — a stock Pi 4 with a LAN9252 breakout exercises the same SoC and the same pin assignment.

TwinCAT is the standard to validate against, not a fallback. As the reference implementation it is materially better than SOEM at the three things this project leans on hardest: ESI validation (Phase 4), DC/SYNC0 diagnostics (Phase 3), and driving a CiA402 axis natively through NC rather than by hand-assembled SDO writes (Phases 4–5). It also writes the SII EEPROM directly, which removes any need for `eepromtool`.

The EtherCAT NIC is the workstation's wired adapter (`enp4s0` under Linux), cabled directly to the LAN9252 IN port. Under Windows, TwinCAT binds it with the Beckhoff real-time driver.

**Iteration workflow.** Editing on Linux and testing from Windows costs two reboots per iteration, which is untenable for a stack needing many. The fix is to make the target board the build host — build and deploy there over ssh, driving that ssh session *from the Windows boot* (VS Code Remote-SSH works well). Linux then becomes optional for a test cycle rather than mandatory.

**SOEM is optional.** It is not needed for correctness; TwinCAT covers every verification step below. Its one genuine advantage is scriptability: the Phase 1.4b loopback harness and the Phase 4/5 regression checks automate naturally against a headless Linux master, whereas TwinCAT is GUI-driven (scriptable via ADS, but heavier). Build it on the Linux boot only if unattended regression runs are wanted.

### Steps

1. Capture a **baseline** against the current unmodified build: TwinCAT online scan, SM/PDO configuration, and the AL state reached. Fixing the reset bug changes behaviour — the chip will actually reset for the first time — so Phase 1 should be expected to *surface* problems previously masked by instantly-satisfied wait loops. A "before" is needed for comparison.
2. **Fix the bus assignment.** cmc owns SPI0 CE0 (TMC4671) and CE1 (TMC6200), and BCM GPIO 6, 22, 23, 24, 25, 26, 27.

**SPI1 is reserved for the LAN9252.** Its pins are fixed in silicon at ALT4 and cannot be remapped by the overlay:

| Signal | BCM | Header | Note |
|---|---|---|---|
| MISO | GPIO19 | 35 | |
| MOSI | GPIO20 | 38 | |
| SCLK | GPIO21 | 40 | |
| CS | GPIO16 | 36 | CE2 position, remapped to chip select 0 → `/dev/spidev1.0` |
| **IRQ** | GPIO17 | 11 | CE1 position, plain GPIO — edge-monitored from the RT thread (Phase 3) |
| **SYNC0** | GPIO18 | 12 | CE0 position, plain GPIO — lets the RT thread wait on the DC edge directly |
| **RESET** | GPIO25 | 22 | **shared with the TMC4671 reset** (cmc `CTRL_RST`) |

**Overlay: `dtoverlay=spi1-1cs,cs0_pin=16`.** The `spi-bcm2835aux` driver declares its chip selects as `cs-gpios` rather than using the AUX peripheral's native CS, so any pin can serve — GPIO18 was never a hardware chip select in the Linux path either, making the move to GPIO16 functionally identical rather than a downgrade. The node stays `/dev/spidev1.0` (chip-select *index* 0), not `spidev1.2`. Cost is one GPIO write per transfer inside the driver, negligible against the ioctl cost analysed in Phase 3.4.

All three SPI1 CE positions are consumed — one as the real chip select, two as plain GPIO — which keeps the bus signals in one physical group and leaves **GPIO4/GPIO5 available for UART3**. This forecloses a second SPI1 chip select, acceptable since the LAN9252 is the only device on the bus.

**The reset line is shared with the TMC4671.** Both devices' reset inputs are init and recovery paths, not part of normal operation: a drive fault is cleared through the CiA402 state machine via Fault Reset in controlword bit 7, never by asserting a reset pin. Sharing therefore matches how both are actually used, and it keeps a pin free. The one constraint it imposes is on pulse width — the assert time must satisfy whichever device needs longest. The LAN9252 requires 200 us; `esc_hw_cfg_t.reset_pulse_us` carries the value (default 500 us) so it can be raised without touching code once the TMC4671 minimum is confirmed.

**SYNC0 on GPIO18 is an optimisation, not a dependency.** Waiting on the DC edge directly lets the RT thread skip the ALEVENT read — six SPI transfers, roughly 100-180 us — on the DC path each cycle, which is a meaningful share of a 1 ms budget. Phase 3 works without it by waiting on IRQ and reading ALEVENT to discover the cause.

Before committing the board layout, confirm on the target that the Ubuntu 26.04 overlay set supports the parameter: `dtoverlay -h spi1-1cs` should list `cs0_pin`. If it does not, fall back to stock `dtoverlay=spi1-1cs` with CS on GPIO18 and RESET on GPIO16 — a pin-role swap only, no change elsewhere.

GPIO18 is also PWM0 / PCM_CLK; nothing in cmc or this roadmap uses either, so it is free as a plain output.

**The BCM numbers are authoritative; the header column is a reference.** On a Pi 4 they are literal 40-pin header positions. On the CM4 carrier they become net names in the schematic, while the BCM assignments carry over unchanged.

**AUX block contention — a board-design consideration.** SPI1 is an AUX peripheral, and so is the mini-UART. cmc drives the TMC4671 over `/dev/ttyS0` at 921600 baud (`cmc/include/comm/UARTConstants.cpp:11`) — that *is* the mini-UART. Both share the AUX block's bus and interrupt, and both have shallow FIFOs. Secondary to the syscall cost analysed in Phase 3.4, but it belongs in the jitter budget. Moving the TMC4671 to the PL011 (`/dev/ttyAMA0`) on the new motherboard would remove it entirely.

---

## Phase 1 — Fix and re-test standalone

**Deliverable:** a `lan9252_diag` binary reaching OP against TwinCAT, with no bcm2835 anywhere in the tree.

### 1.1 Build system

- [`CMakeLists.txt:4`](../CMakeLists.txt#L4) — `cmake_minimum_required` 2.8.12 → **3.22**. CMake 4.x hard-errors below 3.5; this currently blocks configure outright.
- Delete the `configure_file(version.h.in → ${SOES_SOURCE_DIR}/soes/version.h)` block and commit a static `soes/version.h`. It writes into the source tree, which breaks out-of-tree builds and dirties the submodule on every build in Phase 5. Delete `version.h.in`. Drop the CPack block.
- [`cmake/Linux.cmake`](../cmake/Linux.cmake) — delete the `RPI_VARIANT` branch; move `-Werror` off directory-global `add_compile_options` onto `target_compile_options(soes PRIVATE ...)`. This is what would otherwise leak into cmc's C++ in Phase 5.
- [`soes/CMakeLists.txt`](../soes/CMakeLists.txt) — `install(TARGETS soes DESTINATION bin)` → `lib`.

### 1.2 Retention policy

**XMC4 and AM335x are kept.** Both are plausible future targets for this stack, so their HALs and demo applications stay in the tree and are maintained through the Phase 2 rename.

Keep:

| Path | Reason |
|---|---|
| `soes/hal/xmc4/`, `soes/hal/rt-kernel-xmc4/` | XMC4 target; the rt-kernel variant is also the only correct in-tree DC/interrupt example (Phase 3.2) |
| `soes/hal/tiesc/` | TI ESC, used by AM335x and K2G |
| `applications/rtl_xmc4_dynpdo/` | Only in-tree example of dynamic PDO mapping and of `dc_checker` / 0x10F1 / 0x1C32 — Phases 3 and 4 both reference it |
| `applications/xmc4300_slavedemo/`, `applications/tiesc_am335x/`, `applications/tiesc_k2gice/` | Target demos |
| `cmake/toolchain/rt-kernel-xmc4.cmake`, `cmake/Platform/rt-kernel.cmake` | Needed to build the above |

Remove only what the new HAL genuinely supersedes:

| Path | Reason |
|---|---|
| `soes/hal/raspberrypi-lan9252/` | Replaced by `linux-lan9252-spidev`; bcm2835 must go for licensing |
| `soes/hal/linux-lan9252/` | Superseded; also polls the wrong register after reset (0x304 instead of 0x1F8) |
| `drivers/linux/lan9252/` | Out-of-tree kernel module serving only the HAL above |
| `applications/raspberry_lan9252demo/`, `applications/linux_lan9252demo/` | Superseded by `lan9252_diag` |

`soes/hal/rt-kernel-twrk60/`, `applications/rtl_slavedemo/` and `applications/rtl_lwip_eoe/` are neither superseded nor named as future targets — leave them alone unless a reason to remove them comes up.

**Verification gap to accept.** The XMC4 and TI toolchains are not available on the development machine, so code in those HALs cannot be compiled here and the Phase 2 rename cannot be verified against them. The rename is mechanical and greppable, which makes this tolerable, but treat those edits as unproven until someone builds for the target. This cost is the price of keeping the targets, and it is worth stating rather than discovering later.

### 1.3 New HAL — `soes/hal/linux-lan9252-spidev/`

Nothing is Pi-specific once bcm2835 is gone; spidev + gpiochip UAPI is any Linux.

**Replace the `user_arg` token parser, don't extend it.** The current `strtok(arg, " ,.-")` at `esc_hw.c:444-468` treats `.` and `-` as delimiters, so a path like `/dev/spidev1.0` would be shredded. `esc_cfg_t.user_arg` is `void *` — point it at a typed struct declared in `esc_hw.h`:

```c
typedef struct {
   const char *spidev;        /* "/dev/spidev1.0" */
   uint32_t    spi_speed_hz;  /* 20000000 */
   uint8_t     spi_mode;      /* 0 */
   const char *gpiochip;      /* NULL = polled, no IRQ */
   int         irq_line;      /* BCM offset; -1 = polled */
   int         reset_line;    /* -1 = none */
   uint32_t    op_timeout_ms; /* per-wait-loop deadline */
} esc_hw_cfg_t;
```

**bcm2835 → spidev mapping** (this is the complete surface):

| bcm2835 | Replacement |
|---|---|
| `bcm2835_init` / `bcm2835_close` | gone (also removes the root / `/dev/mem` requirement) |
| `bcm2835_spi_begin` / `_end` | `open(cfg->spidev, O_RDWR)` / `close()` |
| `setBitOrder(MSBFIRST)` | `SPI_IOC_WR_LSB_FIRST = 0` |
| `setDataMode(MODE0)` | `SPI_IOC_WR_MODE = SPI_MODE_0` |
| `setClockDivider(16\|32)` | `SPI_IOC_WR_MAX_SPEED_HZ = cfg->spi_speed_hz` — arbitrary Hz, no Pi-model dependence |
| `chipSelect` / `setChipSelectPolarity` | the device node *is* the chip select |
| `bcm2835_spi_transfern` ×4 | `ioctl(fd, SPI_IOC_MESSAGE(1), &xfer)` with `tx_buf == rx_buf` |

Also set `SPI_IOC_WR_BITS_PER_WORD = 8`. Keep `usleep()` — it was never bcm2835.

**Bug fixes** (all verified against the current source):

| Site | Fix |
|---|---|
| `esc_hw.c:35` — `ESC_RESET_CTRL_RST = (0x01 & 0x40)` = **0** | `BIT(6)` — ETHERCAT_RST alone, since a full digital reset re-latches SPI mode strapping. Then poll **0x1F8**, not 0x304; the deleted `linux-lan9252` HAL polls the wrong register and is not a safe reference here |
| `esc_hw.c:185, :281` — `realloc` in the cyclic path, unchecked | **Remove the allocation.** `fifo_size` is 5 bits → ≤31, so `size ≤ 3+4*31 = 127`. Use `uint8_t buf[3+4*32]` on the stack, clamp `fifo_size`, and let the outer `while (len > 0)` iterate. No malloc in the cyclic path is a hard prerequisite for `mlockall` in Phase 3 |
| `ESC_init` returns `void` → `ecat_slv.c:362-368` spins forever | `int ESC_init(...)` / `int ecat_slv_init(...)`; bound the DLSTATUS spin with a deadline and return non-zero with a `DPRINT` |
| `esc_hw.c:447, 513-534` — one `counter` shared across three wait loops | `clock_gettime(CLOCK_MONOTONIC)` deadlines, one per loop. `usleep(100)` sleeps *at least* 100 µs, so an iteration count was never a real timeout |
| **`esc_hw.c:107, 126, 143, 239`** — `do {} while (value & BUSY)` with **no timeout at all** | Give each `cfg->op_timeout_ms`. On expiry set a sticky `hw_fault`, `DPRINT`, and make `ESC_read`/`ESC_write` no-ops so the stack falls out of OP. **Worse than the cumulative-timeout bug** — it hangs a `SCHED_FIFO` thread and livelocks a core |

### 1.4 Application — two sub-steps

**1.4a — keep the existing object dictionary, change one variable at a time.** Copy `applications/raspberry_lan9252demo/` → `applications/lan9252_diag/`, leaving `slave_objectlist.c`, `utypes.h`, `ecat_options.h`, `slave.xml` and `slave.bin` byte-identical. If the HAL and the OD change together and SAFEOP fails, there is no way to tell whether the fault is in `ESC_checkSM23` or the SPI layer. Change only:

- **Un-swap the callbacks** — LED writes move to `cb_set_outputs`, button reads to `cb_get_inputs` (`applications/raspberry_lan9252demo/main.c:22-42`). The demo has these inverted. It is load-bearing for Phase 3, not cosmetic: under the DC flag split, `DIG_process(OUTPUTS)` in the SM2 path would write LEDs and `DIG_process(INPUTS)` in the SYNC0 path would read buttons.
- **Fix `Obj.Buttons.Button0` ×6 → `Button0..Button5`** (`main.c:36-41`). All six button reads currently assign to the same field, so only the last survives.
- Reduce to **two pins** (one LED, one button) on non-colliding GPIOs, via gpiochip UAPI. Fewer pins, fewer ways to be wrong.
- `main()` takes argv: `lan9252_diag /dev/spidev1.0 20000000`.

**1.4b — then delete the demo entirely.** Replace LEDs and buttons with a self-measuring loopback slave with no GPIO at all:

```
RxPDO 0x1600:  0x7000:01 u32 pattern        0x7000:02 u32 master_counter
TxPDO 0x1A00:  0x6000:01 u32 pattern_echo   0x6000:02 u32 master_counter_echo
               0x6000:03 u32 slave_counter  0x6000:04 u32 cycle_us_max
               0x6000:05 u32 cycle_us_last  0x6000:06 u32 sm2_missed_count
```

This becomes the regression harness for every later phase — it measures its own jitter and lost frames, needs no hardware beyond the LAN9252, and permanently removes pin-conflict risk from this tree. 24 bytes each way fits the existing 42-byte SM2 budget, so `ecat_options.h` is untouched. Wire `safeoutput_override` at this point so the hook is exercised before it matters.

### 1.5 Ubuntu 26.04 specifics

- `/boot/firmware/config.txt`: add `dtoverlay=spi1-1cs,cs0_pin=16`. Leave `dtparam=spi=on` alone — cmc needs spidev0. Verify `/dev/spidev1.0` appears after reboot and that GPIO17 and GPIO18 are *not* claimed (`gpioinfo`, or `cat /sys/kernel/debug/gpio`); if the driver grabbed them, the overlay ignored `cs0_pin`.
- **Group ownership is `dialout`, not `spi`.** Measured on the target: `/dev/spidev*` and `/dev/gpiochip0` are both `root:dialout` on this Ubuntu 26.04 arm64 image, contrary to the `99-com.rules` convention that assigns them to `spi` and `gpio`. Check with `stat -c '%U %G %a' /dev/spidev1.0 /dev/gpiochip0` rather than assuming either way, and add the user to whichever group actually owns the nodes.
- `cat /sys/module/spidev/parameters/bufsiz` — 4096 by default, ample for the 131-byte PRAM burst.

### 1.6 Verification

1. TwinCAT online scan shows correct Vendor ID / Product Code / SM config, and a PDO map matching `slave_objectlist.c`.
2. INIT → PREOP → SAFEOP → OP, no AL status code. A `SMRESULT_ERRSM2/3` means `ecat_options.h` and the SII/ESI disagree.
3. Loopback: incrementing pattern echoed with zero mismatches over 10 minutes.
4. Unplug the cable in OP → SAFEOP+ERROR with `ALERR_WATCHDOG`; the safe-state hook fires.
5. **Reset — verified in software, no logic analyser needed.** `lan9252_diag reset` dirties a register, writes `BIT(6)`, then samples BYTE_TEST hard: a genuine reset makes the device stop returning `0x87654321` for a few hundred microseconds and then recover. Observed on hardware. Note that IRQ_CFG survives the reset and that is correct — `BIT(6)` is ETHERCAT_RST, which resets the EtherCAT core only, while the host interface block is outside that domain. That scoping is the reason for preferring it over a full digital reset, which would re-latch the SPI mode strapping.
6. Point `.spidev` at a nonexistent node → init returns non-zero with a diagnostic, does not hang.
7. Yank LAN9252 power mid-OP → the process degrades, does not spin a core at 100%.
8. `grep -ri bcm2835 .` at the repo root returns nothing.

---

## Phase 2 — Callback renaming

### Rationale

SOES names its application callbacks from the **master's** point of view, which is the standard EtherCAT convention: the ETG specification, the ESI format (`<Sm>Outputs</Sm>` / `<Sm>Inputs</Sm>`), Beckhoff's SSC (`APPL_InputMapping` / `APPL_OutputMapping`) and the CANopen index ranges (0x6000 = inputs, 0x7000 = outputs) all use it. `cb_get_inputs` therefore means "sample hardware into the TxPDO the master will read", and `cb_set_outputs` means "apply the RxPDO the master sent". The stack states this plainly at [`ecat_slv.c:144`](../soes/ecat_slv.c#L144) ("Master Inputs", SM3) and [`ecat_slv.c:162`](../soes/ecat_slv.c#L162) ("Master Outputs", SM2).

The convention is correct but easy to invert, and the Raspberry Pi demo did invert it. The fix is to rename to **PDO-centric** names.

Note that TxPDO/RxPDO are not perspective-free either — ETG.1000 defines them from the *slave's* viewpoint, since the slave transmits TxPDO and receives RxPDO. The argument for them is **conventional fixity plus local consistency**: unlike inputs/outputs, the PDO referent is nailed down by the standard and never inverted in practice, and it already matches every symbol an author looks at while writing the callback — SM2 ↔ RxPDO ↔ 0x1600 ↔ 0x1C12, SM3 ↔ TxPDO ↔ 0x1A00 ↔ 0x1C13. The object dictionary is already PDO-named; the callbacks are the only thing speaking a different language, and they speak it from the master's side.

Slave-centric naming is rejected separately: it becomes actively misleading in Phase 5, where "the master" is one of three interchangeable transports.

### Renames

| Current | New |
|---|---|
| `cb_set_outputs()` | `cb_apply_rxpdo()` |
| `cb_get_inputs()` | `cb_update_txpdo()` |
| `RXPDO_update()` | `rxpdo_read_sm2()` — and make `static` |
| `TXPDO_update()` | `txpdo_write_sm3()` — and make `static` |
| `DIG_PROCESS_OUTPUTS_FLAG` | `DIG_PROCESS_RXPDO_FLAG` |
| `DIG_PROCESS_INPUTS_FLAG` | `DIG_PROCESS_TXPDO_FLAG` |
| `safeoutput_override` | `safe_state_override` |
| `APP_safeoutput()` | `APP_safe_state()` |

Rename the stack-internal functions too — `TXPDO_update()` and `cb_update_txpdo()` three lines apart in `DIG_process` would be worse than the status quo. `RXPDO_update` and `TXPDO_update` are currently non-`static` with no header declaration; fix that at the same time.

**Call sites:** [`soes/ecat_slv.h`](../soes/ecat_slv.h) (15, 20, 28, 29), [`soes/ecat_slv.c`](../soes/ecat_slv.c) (134-150, 162-178, 218-251, 343-344), [`soes/esc.h`](../soes/esc.h) (320, 445, 741), [`soes/esc.c`](../soes/esc.c) (922, 930, 1354), plus the new HAL and application.

Because the XMC4 and TI HALs are retained (§1.2), the rename must reach them too — roughly 30 call sites across the tree rather than the 20 in the core alone. Those edits cannot be compile-checked locally; see the verification gap in §1.2.

**Verification:** grep for every old name returns nothing; then re-run the entire 1.6 checklist. A pure rename must produce bit-identical loopback numbers.

---

## Phase 3 — Interrupts and Distributed Clocks

### The gate

[`soes/esc.c:833-837`](../soes/esc.c#L833-L837) returns early when `use_interrupt == 0`, before `ESC_checkDC()` and before the `ESCREG_ALEVENT_DC_SYNC0` mask bit is ever set. Setting `use_interrupt = 1` is not an optimisation — it is the precondition for DC existing at all.

### 3.1 IRQ wait — gpiochip UAPI, not lgpio

lgpio runs a single shared alert thread that `ppoll()`s all alert GPIOs on a 0.5 ms timeout, buffers up to 2000 reports, `qsort`s them by timestamp and re-emits via a pipe — and its scheduling policy is not settable through the public API. Placing that in front of a SYNC0 deadline adds a normal-priority scheduling hop, a pipe round trip and a sort.

Use the kernel chardev v2 UAPI directly: `GPIO_V2_GET_LINE_IOCTL` with `GPIO_V2_LINE_FLAG_EDGE_RISING`, then `ppoll()`/`read()` the line-request fd **from the `SCHED_FIFO` thread itself**. Same kernel mechanism, no extra threads.

A useful consequence: the HAL then has **zero lgpio dependency** — spidev plus gpiochip UAPI, pure C, linkable anywhere. lgpio stays in cmc where it belongs, on the slow BSP pins.

Rising edge is correct as the code stands, because `ESC_interrupt_enable` configures the LAN9252 IRQ pin push-pull active-high (`IRQ_CFG ← 0x111`). If the motherboard uses open-drain active-low, flip to `EDGE_FALLING` — keep it to one line.

### 3.2 Two threads, and the flag split

Model on [`soes/hal/rt-kernel-xmc4/esc_hw.c:214-262`](../soes/hal/rt-kernel-xmc4/esc_hw.c#L214-L262), the only correct in-tree example — one reason that HAL is retained. Translate its two ISRs into one RT loop:

```
loop:
  esc_hw_irq_wait(2 * cycle_ns)
  ESC_read(ESCREG_ALEVENT, &ESCvar.ALevent, 2)     /* explicit, see below */
  if ALevent & (SM2|SM3):
      if dcsync == 0:  DIG_process(RXPDO | APP_HOOK | TXPDO)
      else:            synccounter++;  DIG_process(RXPDO)
  if ALevent & DC_SYNC0:
      synccounter--
      if |synccounter| > synccounterlimit: ALstatusgotoerror(SAFEOP|ERROR, ALERR_SYNCERROR)
      DIG_process(APP_HOOK | TXPDO)
      ESC_read(ESCREG_DCSYNCSTAT, ...)             /* ack the SYNC0 latch */
  if ALevent & (CONTROL|SMCHANGE|SM0|SM1|EEP):
      signal the low-priority worker               /* NOT in this thread */
```

**Two threads, not one.** Mailbox/CoE/FoE work is unbounded and must never sit in front of a SYNC0 deadline — hand it to a normal-priority worker via semaphore, as the xmc4 HAL does.

One LAN9252 difference from the xmc4: ALEVENT is memory-mapped and free there, but costs a full CSR cycle over SPI here. **Do not rely on the piggybacked ALEVENT tail read** (`esc_hw.c:373-376`) inside the loop — that value is from the *previous* transaction.

### 3.3 Configuration and objects

Copy `dc_checker()` from the scratch reference (`rtl_xmc4_dynpdo/main.c:48-56`): set `ESCvar.dcsync = 1` and `synccounterlimit` from 0x10F1:02. Set `.use_interrupt = 1`, both interrupt hooks, `.esc_check_dc_handler`, and `.watchdog_cnt = INT32_MAX` (use the ESC hardware SM watchdog rather than the software counter). Also copy `cb_state_change`'s PREOP→SAFEOP branch — **without an initial TxPDO write, SM3 never fires and the master stalls at SAFEOP**.

Objects to add: **0x10F1** (ErrorSettings; `:02` SyncErrorCounterLimit), **0x1C32** and **0x1C33** (SM2/SM3 sync parameters — `:01` sync mode, `:02` cycle time, `:05` minimum cycle time, `:0B` SMEventMissedCnt, `:20` SyncError).

### 3.3.1 Sync error counting — replace SOES's semantics with ETG.1020

**The new HAL implements ETG.1020 semantics. SOES's existing behaviour is not carried over.**

What SOES does today, in [`soes/hal/rt-kernel-xmc4/esc_hw.c`](../soes/hal/rt-kernel-xmc4/esc_hw.c), is maintain a signed balance between two event sources: `synccounter += 1` on each SM2/SM3 AL event ([line 259](../soes/hal/rt-kernel-xmc4/esc_hw.c#L259)), `-= 1` on each SYNC0 edge ([line 219](../soes/hal/rt-kernel-xmc4/esc_hw.c#L219)), both guarded by `APPSTATE_OUTPUT`, tripping to SAFEOP+ERROR when the absolute value exceeds the limit ([lines 222-223](../soes/hal/rt-kernel-xmc4/esc_hw.c#L222-L223)). `ESC_checkDC` zeroes it when DC is activated ([`esc.c:291`](../soes/esc.c#L291)).

That counter's value is exactly `(frames received) − (SYNC0 pulses)` since the last reset. It has **no decay**: a missed frame costs −1 permanently, while a good cycle contributes +1 against the next SYNC0's −1 and nets zero. A drive losing one frame an hour and otherwise perfect therefore accumulates monotonically and trips after `limit` hours. It is a lifetime-total policy where a drive needs a rate policy.

The decisive objection is interoperability rather than the flaw itself: **0x10F1:02 is a standardised object**, and a master writes it expecting ETG semantics. Implementing something else makes the configured value mean something different — and far stricter — than the master intends. This would show up first against TwinCAT.

**Implementation in `linux-lan9252-spidev`:**

| Condition | Action |
|---|---|
| SYNC0 fires and no SM2 event arrived since the previous SYNC0 | `sync_error_counter += 3`; increment 0x1C32:0B `SMEventMissedCnt` |
| SYNC0 fires and the SM2 event did arrive | `sync_error_counter -= 1`, floored at 0 |
| `sync_error_counter > SyncErrorCounterLimit` (0x10F1:02) | Set 0x1C32:20 `SyncError`; `ESC_ALstatusgotoerror(ESCsafeop \| ESCerror, ALERR_SYNCERROR)`; reset the counter |

The 3:1 weighting gives the limit a concrete meaning worth stating: the counter only grows when the **error rate exceeds 25%**, and the limit then controls how long a burst above that rate is tolerated before dropping to SAFEOP. Below 25% the counter drains to zero and the drive runs indefinitely.

Widen `ESCvar.synccounter` from `int8_t` to `int16_t` ([`esc.h:510`](../soes/esc.h#L510)); `synccounterlimit` is already `uint16_t` ([`esc.h:476`](../soes/esc.h#L476)). With `+3` weighting an `int8_t` overflows at a limit of only 42. This is a pure widening and is compatible with how the retained XMC4 and TI HALs use the field, but it does touch a shared struct those HALs cannot be compile-checked against here (§1.2).

The XMC4 HAL keeps its existing balance-counter logic — only the field's type changes.

#### Sizing the limit

The `+3` / `−1` weighting turns the limit into a time budget: **limit ÷ 3 is the number of consecutive fully-missed cycles tolerated before tripping**. At a 1 ms cycle a limit of 24 tolerates eight such cycles (~8 ms); the top of the range, 255, tolerates eighty-five (~85 ms).

Two bounds determine the value:

- **Lower — survive the worst scheduling excursion.** A stall spanning *n* cycles adds `3n`, then needs `3n` good cycles to drain, so the limit must exceed `3n` for the worst *n* the platform actually produces. **This number is to be measured, not assumed.**
- **Upper — fire before the SM watchdog.** If `limit ÷ 3 × cycle_time` exceeds the SM watchdog period, the watchdog always trips first and the sync error counter never fires at all. The master uses TwinCAT's 100 ms default, capping the useful limit near **300** at a 1 ms cycle — so the full 255 range stays meaningful, with little headroom above it.

**The value is deliberately left open.** Derive it from a `cyclictest` run under representative load on the real target with PREEMPT_RT (§3.6): take the p99.99 latency, convert to missed cycles at the chosen SYNC0 period, multiply by three, and add margin. A limit guessed too low drops the drive to SAFEOP on a transient stall; one guessed too high defers to the watchdog and never fires.

### 3.4 Achievable cycle time

The cost is **ioctls, not bits**. One CSR access is three SPI transactions (write CMD / poll / read DATA) plus the unconditional ALEVENT tail read = **six `SPI_IOC_MESSAGE` ioctls per register read**. At an estimated 15-30 µs of syscall and DMA setup each on BCM2711 — a figure to replace with measurement — a 2-byte register read costs about 100-180 µs; the 7 payload bytes at 12.5 MHz (~4.5 µs) are noise.

**1 ms is not reachable with this HAL as written.** Optimisations in order:

1. **Collapse each CSR access into one `SPI_IOC_MESSAGE`** carrying an array of `spi_ioc_transfer` with `cs_change`. Three ioctls → one. Biggest structural win, no semantic risk.
2. **Merge the ALEVENT tail into the same `SPI_IOC_MESSAGE`** rather than deleting it — costs 7 bytes, saves ~60 µs, and avoids auditing every place in `esc.c` that assumes `ESCvar.ALevent` freshness.
3. **Raise the clock to 30 MHz.** Verified on hardware: a byte-test sweep from 8 to 30 MHz all returned `0x87654321`, and 40 MHz failed cleanly on the timeout path. 30 MHz is the datasheet ceiling for the plain serial mode this HAL uses, so there is no headroom above it. Re-validate on the CM4 carrier, since signal integrity on a designed PCB differs from the bring-up rig.
4. Use LAN9252 **fast-read (0x0B)** for PRAM bursts.

After 1–3, expect `DIG_process` in the **200-400 µs** range for a 24-byte PDO pair. That is the SPI cost alone; scheduling latency is additive and must be measured on the target rather than assumed.

**Target SYNC0 = 1 ms.** This is the cycle time a CiA402 drive in cyclic synchronous position, velocity or torque mode is expected to sustain — a conservative figure, given that commercial servo drives commonly run at 250 µs or below. The target comes from the profile, not from any downstream consumer. The period itself is set by the master, so it is configurable by construction; the slave's obligations are to publish an honest floor in 0x1C32:05 and to size `SyncErrorCounterLimit` for the period actually in use.

1 ms is reachable only once optimisations 1–3 land, leaving roughly 600-800 µs of the period for OS scheduling after `DIG_process` costs 200-400 µs. **The target platform runs PREEMPT_RT**, so the remaining budget is a question for measurement rather than estimation — run `cyclictest` under representative load on the real carrier (§3.6) and compare its p99.99 against that 600-800 µs. That measurement, not an assumed figure, is what decides whether 1 ms holds and what `SyncErrorCounterLimit` must be (§3.3.1).

2 ms is an acceptable fallback if the measured tail will not fit inside the budget at 1 ms.

### 3.5 Real-time setup

`mlockall(MCL_CURRENT|MCL_FUTURE)` process-wide at startup before any thread spawns; `mallopt(M_TRIM_THRESHOLD, -1)` and `mallopt(M_MMAP_MAX, 0)`; pre-fault the stack; `SCHED_FIFO` priority **60** with `PTHREAD_EXPLICIT_SCHED`; pin to an isolated core with `isolcpus=3 nohz_full=3 irqaffinity=0-2` in `cmdline.txt`. Core isolation complements PREEMPT_RT rather than substituting for it — on a four-core part it removes the scheduler contention that the RT patch alone does not.

Priority 60 rather than 80 leaves headroom to promote the SPI controller's IRQ thread above it later.

Run non-root via `/etc/security/limits.d/99-cmc-rt.conf` (`rtprio 90`, `memlock unlimited`) rather than file capabilities, because `colcon build --symlink-install` silently strips file caps on reinstall.

### 3.6 Verification

Inter-edge histogram on `esc_hw_irq_wait` alone (p99.9 under 100 µs deviation at 1 ms); OP with DC enabled and `ESCvar.dcsync == 1` confirmed; the sync error counter drains to zero and stays there over an hour; master killed mid-OP → `ALERR_SYNCERROR` or `ALERR_WATCHDOG` and the safe-state hook fires; `cyclictest -p 60 -t1 -m -i 1000 -a 3` under `stress-ng --cpu 4` (this number decides whether 1 ms holds or 2 ms is needed); `/proc/<pid>/status` `VmLck` non-zero and `maps` stable, proving no runtime allocation.

---

## Phase 4 — CiA402 object dictionary

Independent of Phase 3; the two can proceed in parallel given an agreed contract on the 0x10F1 / 0x1C32 definitions.

### 4.1 Solve the ESI/SII sync problem first

`ESC_checkSM23` ([`soes/esc.c:712-795`](../soes/esc.c#L712-L795)) rejects PREOP→SAFEOP on **any** mismatch between the master's SM configuration (derived from the ESI XML) and the compiled constants. ESI drift is a hard SAFEOP failure, not a cosmetic issue.

Five artifacts must agree — `slave_objectlist.c`, `utypes.h`, `ecat_options.h`, `slave.xml`, `slave.bin` — and the rt-labs Eclipse generator that kept them in sync is out of tree. **`slave.esx` is already stale** and disagrees with the `.c` and `.xml` on PDO layout, so regenerating from it would silently revert things.

**Approach: one declarative YAML is the source of truth, and a Python script generates all five artifacts.**

```
applications/cia402_drive/od.yaml          <- hand-edited source of truth
applications/cia402_drive/generated/{slave_objectlist.c,utypes.h,ecat_options.h,slave.xml,slave.bin}
tools/gen_od.py
```

Making the `.c` the source of truth and hand-maintaining the ESI is exactly the state that produced the stale `.esx`; for a 67-object dictionary times *n* axes it would drift within a month. With a generator, the SM arithmetic is *computed* rather than hand-carried into three files, multi-axis is a loop rather than copy-paste, and the drift `ESC_checkSM23` punishes becomes structurally impossible. **Commit the generated files** so neither the standalone build nor cmc needs Python at build time, with a CI check that regeneration produces no diff. Note that `utypes.h` is not an internal detail: it is the data contract between this stack and cmc, so the generator's output shape is a cross-repo interface and changing it breaks a consumer in another repository. Delete `slave.esx`.

Author `od.yaml` from **ETG.6010** (*Implementation Directive for CiA402 Drive Profile*), which specifies how the CiA402 objects map onto EtherCAT CoE — the correct primary source here, and more directly applicable than the generic CiA 402 / IEC 61800-7-201 profile text. Cross-check the generated SII against the existing known-good 256-byte `slave.bin` as a structural reference, and against TwinCAT's EEPROM view.

No third-party EDS or reference implementation is used as input. GPL-licensed CiA402 implementations are deliberately excluded — reading them in order to reimplement is the pattern that creates derivative-work exposure, which defeats the point of keeping this tree free of licence encumbrances.

### 4.2 Fixed process image and the v1 object set

**The process image is fixed, not master-configurable.** A fixed-function drive gains nothing from dynamic mapping, and avoiding it removes the PREOP remapping protocol, the sub-index-0 zeroing dance and a class of configuration failure. 0x1600, 0x1A00, 0x1C12 and 0x1C13 are therefore all `ATYPE_RO` with constant values.

This departs from what the vendor survey shows Beckhoff doing, and the departure is deliberate. A survey of the ELM72xx and EL72xx ESI files (23 CiA402 devices, via [`tools/esi_survey.py`](../tools/esi_survey.py)) found **93% of their 3,633 PDOs carry exactly one object** — 0x1610 Controlword, 0x1611 Target position, 0x1612 Target velocity and so on — with only 575 pre-bound to a SyncManager and the rest assembled by the master through 0x1C12/0x1C13. That granularity exists so one firmware can serve any process image a customer asks for. A single-purpose drive has no such requirement.

**Zero-copy was considered and rejected.** Setting `MAX_MAPPINGS_SM2`/`_SM3` to 0 makes `rxpdo`/`txpdo` extern symbols the application supplies ([`soes/ecat_slv.c:25-35`](../soes/ecat_slv.c#L25-L35)), so the stack skips `COE_pdoPack`/`Unpack` and reads process data straight into the application struct — the `xmc4300_slavedemo` model. It saves a few microseconds of bit-slicing per cycle, which is noise beside the 200-400 µs of SPI in §3.4, and it buys that with silent fragility: the C struct layout must match the wire layout exactly, and CiA402's mixed widths (u16, i8, i32, i16) produce natural padding unless packed, while packing invites unaligned accesses on ARM. A layout error corrupts data rather than failing to compile. **Keep `MAX_MAPPINGS` non-zero**, sized to the actual entry count rather than the default 16.

#### v1 object set (CSP / CSV / CST), single axis

**Mapped into the cyclic image.** Fields are ordered widest-first so every 32-bit value lands naturally aligned — the alternative, listing them in index order, puts each `int32` on an odd offset because `0x6060` is a single byte. Nothing in the protocol requires this; the master reads the order from 0x1600/0x1A00. It costs nothing and removes a class of unaligned-access problem on ARM. Note the consequence: **controlword and statusword are not at offset 0**, which may surprise someone reading the ESI who expects the conventional layout.

**RxPDO 0x1600 — master to drive**

| Offset | Bytes | Object | Type | Meaning |
|---|---|---|---|---|
| 0 | 4 | 0x607A | i32 | Target position |
| 4 | 4 | 0x60FF | i32 | Target velocity |
| 8 | 2 | 0x6040 | u16 | Controlword |
| 10 | 2 | 0x6071 | i16 | Target torque |
| 12 | 2 | 0x6072 | u16 | Max torque |
| 14 | 1 | 0x6060 | i8 | Modes of operation |
| 15 | 1 | — | — | padding |

**TxPDO 0x1A00 — drive to master**

| Offset | Bytes | Object | Type | Meaning |
|---|---|---|---|---|
| 0 | 4 | 0x6064 | i32 | Position actual |
| 4 | 4 | 0x606C | i32 | Velocity actual |
| 8 | 4 | 0x60F4 | i32 | Following error actual |
| 12 | 4 | 0x2000:01 | u32 | Vendor status and fault flags |
| 16 | 2 | 0x6041 | u16 | Statusword |
| 18 | 2 | 0x6077 | i16 | Torque actual |
| 20 | 2 | 0x603F | u16 | Error code |
| 22 | 1 | 0x6061 | i8 | Modes of operation display |
| 23 | 1 | — | — | padding |

**16 bytes out, 24 bytes in**, per axis.

0x6072 and 0x60F4 come from the ESI survey: both are mapped by **100%** of the CiA402 devices examined, at 2 and 4 bytes. Universal adoption suggests masters expect them. Also universal but omitted as Beckhoff-specific: 0x603E and 0x60EA.

**SDO-readable, not mapped.** 0x605A Quick stop option code, 0x6502 Supported drive modes, the mapping and assignment objects 0x1600/0x1A00 and 0x1C12/0x1C13, and Phase 3's 0x10F1/0x1C32/0x1C33.

**Scaling objects — SDO-only, required for master-side unit conversion.** Without these a master cannot turn encoder increments into user units, which TwinCAT NC needs:

| Object | Type | Purpose |
|---|---|---|
| 0x608F | record | Position encoder resolution — `:01` increments, `:02` motor revolutions |
| 0x6091 | record | Gear ratio — `:01` motor revolutions, `:02` shaft revolutions |
| 0x6092 | record | Feed constant — `:01` feed, `:02` shaft revolutions |
| 0x6076 | u32 | Motor rated torque, mNm — also the reference for the torque scaling in §5.3 |

Deferred to v2, with survey support if wanted later: the touch-probe group 0x60B8-0x60BD (87% of devices) and the offsets 0x60B1/0x60B2 (65%).

#### Vendor objects (0x2000-0x5FFF)

An EtherCAT build has no ROS topics, so everything an operator needs must come through the object dictionary. The manufacturer-specific range carries it, mirroring what cmc's six telemetry message types publish today (board, analog, control, digital, feedback, states):

- **0x2000:01 Drive status and fault flags** (u32) is **PDO-mapped**, since a fault must reach the master in the cycle it occurs rather than on the next SDO poll. It is the only vendor object in the cyclic image.
- Everything else — DC bus voltage, board and MOSFET temperatures, phase currents, gate driver fault detail, hall and encoder raw values — is **SDO-readable only**. These cost dictionary space but no frame bytes, and an operator polling them at human rates loses nothing.

Populate the bit assignments of 0x2000:01 from cmc's existing fault and status sources, principally `Bsp`'s `DRV_FAULT` and `CTRL_STA` inputs and the TMC6200 fault register.

#### Identity

The demo ships rt-labs' Vendor ID 0x1337 with Product Code 1234, which must not go out on a product. Use an identity from the **ETG-designated evaluation range** for bench work — confirm the exact value from ETG documentation rather than copying one from another device — and keep Vendor ID, Product Code and Revision as three loudly-marked fields in `od.yaml` so replacing them is a one-line change in one file. Obtaining an assigned ETG Vendor ID is a prerequisite before any unit ships, because the identity is baked into both the ESI and the SII image and masters match on it.

Generator invariants that make hand-error impossible: ascending index order; the `{0xffff,0xff,0xff,0xff,NULL,NULL}` sentinel; sub 0x00 first as `DTYPE_UNSIGNED8/8/ATYPE_RO/maxsub`; mapping words as `(index<<16)|(sub<<8)|bitlength` with index 0 / sub 0 meaning padding; and `ATYPE_RO` throughout the mapping and assignment objects, since the image is fixed.

**The cost of a fixed image is that it must be right before the ESI ships**, because changing it later means regenerating firmware, ESI and SII together and re-scanning in TwinCAT.

### 4.3 SM layout

The binding constraint is `SM2_sma + 3*SM_length <= SM3_sma` ([`soes/esc.c:752-757`](../soes/esc.c#L752-L757)) — note it uses the **runtime** length the master configured, not `MAX_RXPDO_SIZE`.

**Size the layout for the eventual axis count, not for v1.** SM addresses live in the ESI and the SII image, so enlarging them later breaks the ESI and forces a re-scan in TwinCAT. Reserving the space now costs only DPRAM, of which the LAN9252 has 4 KB and this design uses a fraction.

At 16 bytes out and 24 bytes in per axis (§4.2), reserving for **four axes** gives 64 and 96:

```
MBXSIZE          256     /* up from 128: SDO-Info over a large dictionary needs room */
MBX0_sma         0x1000
MBX1_sma         0x1100
SM2_sma          0x1200
MAX_RXPDO_SIZE   64      /* 4 axes x 16 bytes */
SM3_sma          0x1300  /* = 0x1200 + 3*64 */
MAX_TXPDO_SIZE   96      /* 4 axes x 24 bytes; top = 0x1300 + 3*96 = 0x1420 < 0x2000 */
MAX_MAPPINGS_SM2 32      /* 6 entries per axis at 4 axes, plus headroom */
MAX_MAPPINGS_SM3 32      /* 8 entries per axis at 4 axes, plus headroom */
```

**Four is a placeholder and needs confirming.** It is the one number in this layout not derived from a decision already taken. Pick the real maximum before the ESI ships; after that it is expensive to change, and until then it is a single key in `od.yaml`. 0x1420 leaves roughly 2.9 KB of DPRAM spare, so a larger count is affordable if the answer is more than four.

### 4.4 Multi-axis

**v1 ships one axis, but the generator emits the indexed form from the start.** This is the reason §4.1 pays for itself: adding axes later must not require re-deriving the SM arithmetic, regenerating the ESI or re-scanning in TwinCAT, and with the layout reserved above it does not.

Axis *n* sits at `index + n*0x800` — axis 0 at 0x6040, axis 1 at 0x6840, axis 2 at 0x7040. In `od.yaml` this is one `axes:` key; in the generator a loop; in `utypes.h` it becomes `Obj.axis[n]`. Write the generator to take the count as a parameter from the outset rather than special-casing a single axis, since retrofitting the indexed form is the change this structure exists to avoid.

The populated PDO length still reflects the axes actually present, so a single-axis v1 puts 16 and 24 bytes on the wire regardless of the reservation.

### 4.5 Verification

Generator idempotent under a CI diff check; the master's PDO map byte-for-byte matches `od.yaml` including padding; SDO read/write of every RW object and a full SDO-Info OD list (which validates `MBXSIZE 256`); a master write to 0x1C12 or 0x1600 is rejected, confirming the mapping really is read-only; `xmllint --schema EtherCATInfo.xsd` on the ESI. **If a `SMRESULT_ERRSM2/3` appears, fix the generator, never the output.**

---

## Phase 5 — cmc integration

### 5.1 Pre-integration fixes in cmc (separate, standalone commits)

- `cmc/src/main.cpp:68-72` — `while (!g_shutdownFlag) executor.spin_some();` busy-spins a core at 100% with no sleep, and `g_shutdownFlag` is never set because the signal handler calls `exit()` directly. Replace with `executor.spin()` and a handler that sets the flag and calls `rclcpp::shutdown()`. A 100%-CPU normal-priority spin next to a `SCHED_FIFO` thread will show up in the jitter histogram.
- `cmc/include/comm/SPI.hpp:1-3` — the include guard `#endif`s on line 3, leaving the entire header body unguarded.
- `cmc/setup_cmc.sh:553` — the "never clone --recursive" warning is stale; the orphaned gitlink it refers to is gone. Replace with an explicit `git submodule update --init --depth 1 external/soes`.

### 5.2 Submodule and build

`git submodule add <fork-url> external/soes` — **`external/`, not `include/`**, since cmc's `include/` holds .cpp files and is swept by globs and include paths.

**Do not `add_subdirectory`.** Even with Phase 1.1's fixes, list the sources explicitly so nothing is inherited:

```cmake
set(SOES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/external/soes)
add_library(soes_ecat STATIC
  ${SOES_DIR}/soes/esc.c ${SOES_DIR}/soes/esc_coe.c ${SOES_DIR}/soes/esc_foe.c
  ${SOES_DIR}/soes/esc_eoe.c ${SOES_DIR}/soes/esc_eep.c ${SOES_DIR}/soes/ecat_slv.c
  ${SOES_DIR}/soes/hal/linux-lan9252-spidev/esc_hw.c
  ${SOES_DIR}/applications/cia402_drive/generated/slave_objectlist.c)
target_include_directories(soes_ecat PUBLIC
  ${SOES_DIR} ${SOES_DIR}/soes ${SOES_DIR}/soes/include/sys/gcc
  ${SOES_DIR}/applications/cia402_drive/generated)
target_compile_options(soes_ecat PRIVATE -Wall -Wextra -Wno-unused-parameter)
set_target_properties(soes_ecat PROPERTIES POSITION_INDEPENDENT_CODE ON C_STANDARD 11)
target_link_libraries(axis soes_ecat)
```

No new `package.xml` dependencies. Add a cmc `NOTICE` entry naming SOES's GPLv2-with-linking-exception and this fork's URL; keep the sources behind the submodule boundary rather than copying them in, so the licence boundary stays legible.

### 5.3 Transport-neutral CiA402 core

```
include/cia402/Cia402Sm.{hpp,cpp}      pure state machine — no I/O, no ROS, no clock
include/cia402/Cia402Core.{hpp,cpp}    OD values + Step(); owns the SM
include/cia402/DriveInterface.hpp      abstract: ApplySetpoint / ReadFeedback
include/cia402/Transport.hpp           abstract: Poll(), Id(), IsCommanding()
include/transport/{EtherCatTransport,DdsTransport,CanOpenTransport}.{hpp,cpp}
```

`Cia402Sm` is a free function — `transition(controlword, current_state, fault) → {next_state, statusword}`. The nine states and the transition table come from the **ETG.6010 / CiA402 state diagram**, written from the specification. **This is the highest-value unit test in the project:** an exhaustive controlword × state table under `ament_add_gtest`, running in CI with no hardware.

`DriveInterface` keeps `Cia402Core` free of TMC specifics; `Axis` implements it against `Controller` and `GateDriver`.

**The torque scaling contract belongs here**, because it is easy to get wrong and expensive to rediscover. CiA402 expresses torque as per-thousandths of rated torque (`0x6071`, `0x6077`, `0x6072`), with `0x6076` Motor rated torque in mNm as the scaling reference. The TMC4671's torque registers are signed 16-bit — `PID_TORQUE_FLUX_TARGET` (0x64), `PID_TORQUE_FLUX_ACTUAL` (0x69), `PID_TORQUE_FLUX_LIMITS` (0x5E) — so the widths match exactly and no range is lost. Three rules:

- **Compute in `int32`.** The scaling multiply overflows `int16` well before the operands do; saturate on the way back down.
- **The TMC4671 regulates current, not torque.** Its FOC loop controls Iq, so the conversion runs through the motor's torque constant Kt, a commissioning parameter, with `0x6076` as the bridge to physical units.
- **Torque and flux share one register.** `PID_TORQUE_FLUX_TARGET` packs torque in the high half and flux in the low half, so write both fields together rather than read-modify-write — on the cyclic path the latter costs an extra SPI round trip per cycle.

**Exactly one transport, fixed at configuration.** WebSocket, ROS 2 DDS, EtherCAT or CANopen is selected when the device is configured and started, and does not change for the life of the run. This is a decided constraint on the cmc architecture and it removes a great deal: no priority ladder, no arbitration between concurrent commanders, no observer mode, and no handover path that would otherwise have to pass through a safe state to avoid a step discontinuity in the setpoint.

**The transport is selected at compile time.** `Cia402Core` binds to one transport implementation in the build; the others are not compiled in at all. This is stronger than a runtime switch and simplifies several things at once: an EtherCAT build genuinely contains no ROS (no `rclcpp` link dependency, not merely an unused one), dead transports cannot be reached by accident, and the binary shrinks to what the device actually does.

In CMake terms this means separate executable targets over a shared core rather than one binary with runtime branches — `axis_ethercat`, `axis_ros2` and so on, each linking the common `Cia402Core` plus its own transport. The ROS-specific sources and the `ament` dependencies belong only to the ROS target.

**An EtherCAT build contains no ROS.** No `rclcpp::init`, no `Axis` ROS node, no publishers, no 20 ms wall timer. This is the structurally significant consequence, and it dictates the shape of the refactor: `Axis` today (`cmc/include/axis/Axis.cpp`, ~916 lines) is simultaneously the ROS node *and* the drive logic, so the drive logic has to come out of it first. `Cia402Core`, `DriveInterface`, `Tmc`, `Controller` and `GateDriver` must all be ROS-free — most already are; `Axis` is the exception.

Practical consequences to plan for:

- **Telemetry.** ROS topics do not exist in an EtherCAT build, so anything an operator needs must be reachable as TxPDO entries or SDO-readable objects. Decide what moves into the object dictionary before Phase 4 fixes the PDO layout, since retrofitting entries means re-deriving the SM arithmetic (§4.3).
- **Extraction order.** Splitting the drive logic out of the ROS node is its own commit, landing before `EtherCatTransport` is wired up — alongside the SPI ownership change in §5.4.

`safe_state_override` **must not be NULL here** — wire it to `Cia402Core::ForceSafeState()` → Switch On Disabled plus `ctrl_.Disable()`. This is the most safety-relevant line in the integration.

### 5.4 Threading

**No mutex on the hot path.** A `SCHED_FIFO`-60 thread blocking on a mutex held by a `SCHED_OTHER` ROS thread is unbounded priority inversion. The payload is under 64 bytes, so use a **seqlock** (`std::atomic<uint32_t> seq_` plus a double buffer; the writer bumps to odd then even, the reader retries on odd-or-changed) — one per direction. Lock-free, no syscall, no inversion, and no page fault under lock thanks to the Phase 1 `realloc` removal plus `mlockall`.

**SPI ownership.** `Tmc` has a `spi_mutex_`, and `Axis::TimerCB` (`cmc/include/axis/Axis.cpp:361-378`) currently does TMC4671 SPI reads (`GetPhaseCurrents`, `GetRPM`) from the 20 ms ROS timer on `/dev/spidev0.0`. If the EtherCAT thread also drives the TMC4671 each cycle while that timer is running, the two contend on that mutex — priority inversion on the hot path, defeating the seqlock.

**This concern disappears entirely in an EtherCAT build.** With no ROS, there is no 20 ms timer and no second SPI user: the `SCHED_FIFO` thread is the sole owner of TMC SPI by construction, and `spi_mutex_` is uncontended on the hot path. The seqlock is then only needed if a non-RT thread reads drive state — which, in a ROS-free build, may be nothing at all.

What remains is the extraction work, not an arbitration problem: `GetPhaseCurrents` and `GetRPM` must be callable from the cyclic thread rather than from `Axis::TimerCB`, which is part of lifting the drive logic out of the ROS node (§5.3).

**The hardware watchdog is a hard requirement, not an open question.** `bsp_.PetWatchdog()` exists *only* on the 20 ms ROS timer (`cmc/include/axis/Axis.cpp:363`). Remove ROS and nothing pets it, so the board resets after the 625 ms window — an EtherCAT build would reboot in under a second with no other symptom. Give it an explicit owner before `EtherCatTransport` runs on hardware.

The natural owner is the cyclic thread itself, gated on elapsed time exactly as `Bsp::PetWatchdog` already does internally: at 1 ms SYNC0 the thread wakes far more often than the 625 ms window needs, the check is a timestamp comparison plus an occasional GPIO toggle, and it ties the watchdog to the liveness of the thread that actually matters. The alternative — a separate low-priority timer thread — keeps the RT path pure but will happily go on petting the watchdog while the cyclic thread is wedged, which is the failure the watchdog exists to catch.

### 5.5 Verification

`colcon build` clean and `nm -C libsoes_ecat.a | grep -c ' T '` non-zero (proves the library is not silently empty); cmc's own C++ still builds without `-Werror`; `git -C external/soes status --porcelain` clean after a full build (proves the `configure_file` removal landed); `Cia402Sm` gtest exhaustive; hardware enable sequence Shutdown → Switch On → Enable Operation with matching statuswords and the motor spinning in CSV; arbitration test (EtherCAT in OP rejects a ROS command, falls back to DDS within one cycle on SAFEOP); `chrt -p` confirms exactly one `SCHED_FIFO 60` thread; p99.9 jitter does not regress from the Phase 3 standalone number under full ROS telemetry load.

---

## Dependency graph

```
Phase 0 ──► Phase 1 ──► Phase 2 ──► Phase 3 ──► Phase 5
                            │                      ▲
                            └──────► Phase 4 ──────┘
```

Phase 1 ships and is testable alone. Phase 2 must follow 1.2 so that files about to be deleted are not renamed first. Phase 4 is independent of Phase 3 and can run in parallel. Phase 5 needs both.

---

## Settled decisions

- **Object dictionary tooling** — YAML source plus a Python generator (§4.1).
- **Sync error semantics** — ETG.1020, replacing SOES's balance counter (§3.3.1).
- **Cycle time** — 1 ms SYNC0 target, 2 ms acceptable fallback (§3.4).
- **Platform** — EtherCAT builds require the CM4 carrier; ROS 2 builds run on a stock Pi 4 or the carrier. PREEMPT_RT is present on the target.
- **SM watchdog** — TwinCAT default, 100 ms.
- **Retention** — XMC4 and AM335x/TI HALs and demos are kept (§1.2).
- **Process image** — fixed, not master-configurable; `ATYPE_RO` mapping objects, dynamic machinery retained (§4.2).
- **PDO field order** — widest-first for natural alignment; 16 bytes out, 24 in, per axis (§4.2).
- **Telemetry** — vendor objects in 0x2000-0x5FFF, SDO-readable, with only the status/fault word PDO-mapped (§4.2).
- **Scaling objects** — 0x608F, 0x6091, 0x6092 and 0x6076 present, SDO-only (§4.2).
- **Identity** — ETG evaluation range for now, replaced by an assigned Vendor ID before shipping (§4.2).
- **Axis count** — v1 ships one axis; the generator and SM layout are built for several from the start (§4.3, §4.4).
- **Transport** — exactly one, selected at **compile time** as separate executable targets; no ROS in an EtherCAT build (§5.3).

## Open questions

- **Maximum axis count** — §4.3 reserves SM space for four as a placeholder. This is the one number in the layout not derived from a decision already taken, and it must be fixed before the ESI ships.
- **`SyncErrorCounterLimit` value** — method settled (§3.3.1); the number waits on a `cyclictest` measurement under PREEMPT_RT on the real target.
- **0x2000:01 bit assignments** — which cmc fault and status sources map to which bits of the one PDO-mapped vendor word (§4.2).
