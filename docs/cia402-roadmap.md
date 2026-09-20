# CiA402 EtherCAT drive interface — roadmap

## Context

This repository is a hard fork of the unmaintained [OpenEtherCATsociety/SOES](https://github.com/OpenEtherCATsociety/SOES) EtherCAT slave stack. It is being developed as a git submodule of **cmc**, a ROS 2 motion controller (TMC4671 FOC controller + TMC6200 gate driver) running on a Raspberry Pi 4 under Ubuntu 26.04 and ROS 2 Lyrical.

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

The authoritative master is **TwinCAT**, running on the Windows boot of the development workstation (Linux and Windows are on separate NVMe drives, so the two boots are mutually exclusive). The slave runs on separate hardware — the Raspberry Pi 4 — so master and slave never contend for one machine.

TwinCAT is the standard to validate against, not a fallback. As the reference implementation it is materially better than SOEM at the three things this project leans on hardest: ESI validation (Phase 4), DC/SYNC0 diagnostics (Phase 3), and driving a CiA402 axis natively through NC rather than by hand-assembled SDO writes (Phases 4–5). It also writes the SII EEPROM directly, which removes any need for `eepromtool`.

The EtherCAT NIC is the workstation's wired adapter (`enp4s0` under Linux), cabled directly to the LAN9252 IN port. Under Windows, TwinCAT binds it with the Beckhoff real-time driver.

**Iteration workflow.** Editing on Linux and testing from Windows costs two reboots per iteration, which is untenable for a stack needing many. The fix is to make the Pi the build host — build and deploy there over ssh, driving that ssh session *from the Windows boot* (VS Code Remote-SSH works well). Linux then becomes optional for a test cycle rather than mandatory.

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
| **RESET** | GPIO18 | 12 | CE0 position, plain GPIO — optional ESC hard reset |

**Overlay: `dtoverlay=spi1-1cs,cs0_pin=16`.** The `spi-bcm2835aux` driver declares its chip selects as `cs-gpios` rather than using the AUX peripheral's native CS, so any pin can serve — GPIO18 was never a hardware chip select in the Linux path either, making the move to GPIO16 functionally identical rather than a downgrade. The node stays `/dev/spidev1.0` (chip-select *index* 0), not `spidev1.2`. Cost is one GPIO write per transfer inside the driver, negligible against the ioctl cost analysed in Phase 3.4.

The only free GPIO on this board are the three SPI1 CE positions (GPIO16/17/18) plus GPIO4/TXD3 and GPIO5/RXD3. All three CE pins are consumed here — one as the real chip select, two as plain GPIO — which keeps every LAN9252 signal in one physical group and leaves **GPIO4/GPIO5 available for UART3**. This forecloses a second SPI1 chip select, acceptable since the LAN9252 is the only device on the bus.

Before committing the board layout, confirm on the target that the Ubuntu 26.04 overlay set supports the parameter: `dtoverlay -h spi1-1cs` should list `cs0_pin`. If it does not, fall back to stock `dtoverlay=spi1-1cs` with CS on GPIO18 and RESET on GPIO16 — a pin-role swap only, no change elsewhere.

GPIO18 is also PWM0 / PCM_CLK; nothing in cmc or this roadmap uses either, so it is free as a plain output.

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
- **Do not assume `dialout`.** That group governs tty/serial nodes. Ubuntu Pi images assign `/dev/spidev*` to `spi` and `/dev/gpiochip*` to `gpio` via `99-com.rules`. Run `stat -c '%U %G %a' /dev/spidev1.0 /dev/gpiochip0` and `getent group spi gpio` on the target, then add the user to whatever actually owns the nodes; add a udev rule if the image ships none for spidev1.
- `cat /sys/module/spidev/parameters/bufsiz` — 4096 by default, ample for the 131-byte PRAM burst.

### 1.6 Verification

1. TwinCAT online scan shows correct Vendor ID / Product Code / SM config, and a PDO map matching `slave_objectlist.c`.
2. INIT → PREOP → SAFEOP → OP, no AL status code. A `SMRESULT_ERRSM2/3` means `ecat_options.h` and the SII/ESI disagree.
3. Loopback: incrementing pattern echoed with zero mismatches over 10 minutes.
4. Unplug the cable in OP → SAFEOP+ERROR with `ALERR_WATCHDOG`; the safe-state hook fires.
5. Logic-analyse the first SPI transaction after boot: `0x40` written to 0x1F8 and the readback bit clears — proves the reset fix landed.
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

### 3.4 Achievable cycle time

The cost is **ioctls, not bits**. One CSR access is three SPI transactions (write CMD / poll / read DATA) plus the unconditional ALEVENT tail read = **six `SPI_IOC_MESSAGE` ioctls per register read**. At roughly 15-30 µs of syscall and DMA setup each on a Pi 4, a 2-byte register read costs about 100-180 µs; the 7 payload bytes at 12.5 MHz (~4.5 µs) are noise.

**1 ms is not reachable with this HAL as written.** Optimisations in order:

1. **Collapse each CSR access into one `SPI_IOC_MESSAGE`** carrying an array of `spi_ioc_transfer` with `cs_change`. Three ioctls → one. Biggest structural win, no semantic risk.
2. **Merge the ALEVENT tail into the same `SPI_IOC_MESSAGE`** rather than deleting it — costs 7 bytes, saves ~60 µs, and avoids auditing every place in `esc.c` that assumes `ESCvar.ALevent` freshness.
3. **Raise the clock** to 20 MHz (the LAN9252 tops out at 30 MHz in the plain serial mode this HAL uses); validate with the byte-test register reading `0x87654321` at 20 and 25 MHz first.
4. Use LAN9252 **fast-read (0x0B)** for PRAM bursts.

After 1–3, expect `DIG_process` in the **200-400 µs** range for a 24-byte PDO pair. Then add OS latency: without PREEMPT_RT (currently blocked by the Ubuntu 26.04 tryboot A/B bug documented in cmc), `SCHED_FIFO` on a mainline kernel gives roughly 50-150 µs wakeup latency with 1-3 ms tail excursions under load.

**Target SYNC0 = 1 ms.** This is the cycle time a CiA402 drive in cyclic synchronous position, velocity or torque mode is expected to sustain — a conservative figure, given that commercial servo drives commonly run at 250 µs or below. The target comes from the profile, not from any downstream consumer. The period itself is set by the master, so it is configurable by construction; the slave's obligations are to publish an honest floor in 0x1C32:05 and to size `SyncErrorCounterLimit` for the period actually in use.

1 ms is reachable only once optimisations 1–3 land, leaving roughly 600-800 µs of the period for OS scheduling after `DIG_process` costs 200-400 µs. The risk is concentrated in the tail: without PREEMPT_RT, `SCHED_FIFO` excursions of 1-3 ms exceed a whole period, so at 1 ms the sync error counter stops being a formality and becomes the mechanism deciding whether those excursions drop the drive to SAFEOP. Size the limit against a measured `cyclictest` histogram (§3.6), not an assumption.

2 ms is an acceptable fallback if the measured tail cannot be brought inside budget at 1 ms. Revisit both when PREEMPT_RT unblocks.

### 3.5 Real-time setup

`mlockall(MCL_CURRENT|MCL_FUTURE)` process-wide at startup before any thread spawns; `mallopt(M_TRIM_THRESHOLD, -1)` and `mallopt(M_MMAP_MAX, 0)`; pre-fault the stack; `SCHED_FIFO` priority **60** with `PTHREAD_EXPLICIT_SCHED`; pin to an isolated core with `isolcpus=3 nohz_full=3 irqaffinity=0-2` in `cmdline.txt` (worth more than PREEMPT_RT on a 4-core Pi).

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

Making the `.c` the source of truth and hand-maintaining the ESI is exactly the state that produced the stale `.esx`; for a 67-object dictionary times *n* axes it would drift within a month. With a generator, the SM arithmetic is *computed* rather than hand-carried into three files, multi-axis is a loop rather than copy-paste, and the drift `ESC_checkSM23` punishes becomes structurally impossible. **Commit the generated files** so neither the standalone build nor cmc needs Python at build time, with a CI check that regeneration produces no diff. Delete `slave.esx`.

Author `od.yaml` from **ETG.6010** (*Implementation Directive for CiA402 Drive Profile*), which specifies how the CiA402 objects map onto EtherCAT CoE — the correct primary source here, and more directly applicable than the generic CiA 402 / IEC 61800-7-201 profile text. Cross-check the generated SII against the existing known-good 256-byte `slave.bin` as a structural reference, and against TwinCAT's EEPROM view.

No third-party EDS or reference implementation is used as input. GPL-licensed CiA402 implementations are deliberately excluded — reading them in order to reimplement is the pattern that creates derivative-work exposure, which defeats the point of keeping this tree free of licence encumbrances.

### 4.2 Minimum v1 object set (CSP / CSV / CST)

0x6040 Controlword, 0x6041 Statusword, 0x6060/0x6061 Modes of operation and display, 0x6064 Position actual, 0x606C Velocity actual, 0x6077 Torque actual, 0x607A Target position, 0x60FF Target velocity, 0x6071 Target torque, 0x603F Error code, 0x605A Quick stop option code, 0x6502 Supported drive modes, plus 0x1600/0x1A00, 0x1C12/0x1C13 and Phase 3's 0x10F1/0x1C32/0x1C33.

RxPDO = 13 bytes (pad to 14); TxPDO = 15 bytes (pad to 16).

Generator invariants that make hand-error impossible: ascending index order; the `{0xffff,0xff,0xff,0xff,NULL,NULL}` sentinel; sub 0x00 first as `DTYPE_UNSIGNED8/8/ATYPE_RO/maxsub`; mapping words as `(index<<16)|(sub<<8)|bitlength` with index 0 / sub 0 meaning padding; and `ATYPE_RWpre` on 0x1C12/0x1C13/0x160x/0x1A0x so the master can reconfigure PDOs in PREOP, which TwinCAT and SOEM both do.

### 4.3 SM layout

The binding constraint is `SM2_sma + 3*SM_length <= SM3_sma` ([`soes/esc.c:752-757`](../soes/esc.c#L752-L757)) — note it uses the **runtime** length the master configured, not `MAX_RXPDO_SIZE`.

```
MBXSIZE          256     /* up from 128: SDO-Info over a 67-object dict needs room */
MBX0_sma         0x1000
MBX1_sma         0x1100
SM2_sma          0x1200
MAX_RXPDO_SIZE   64
SM3_sma          0x1300  /* = 0x1200 + 3*64 */
MAX_TXPDO_SIZE   64      /* top = 0x1440 < 0x2000 */
MAX_MAPPINGS_SM2 16      /* stack default; 5 used at one axis */
MAX_MAPPINGS_SM3 16
```

64 bytes rather than 14/16 leaves headroom for 0x60B0/0x60B2 offsets, 0x6078, 0x60FD or a second axis without re-deriving the map. 2.9 KB of the LAN9252's 4 KB DPRAM stays spare.

### 4.4 Multi-axis

Ship single-axis and prove OP first. Then axis *n* is `index + n*0x800` (axis 0 = 0x6040, axis 1 = 0x6840, and so on) — one `axes: 3` key in `od.yaml` and a loop in the generator; `utypes.h` becomes `Obj.axis[n]`. Three axes is roughly 45 Rx / 48 Tx bytes, still inside 64; at four, the generator recomputes §4.3 automatically. That is the payoff for §4.1.

### 4.5 Verification

Generator idempotent under a CI diff check; the master's PDO map byte-for-byte matches `od.yaml` including padding; SDO read/write of every RW object and a full SDO-Info OD list (which validates `MBXSIZE 256`); master reconfigures 0x1C12/0x1600 in PREOP with a reduced map and SAFEOP still succeeds; `xmllint --schema EtherCATInfo.xsd` on the ESI. **If a `SMRESULT_ERRSM2/3` appears, fix the generator, never the output.**

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

**Exactly one transport, fixed at configuration.** WebSocket, ROS 2 DDS, EtherCAT or CANopen is selected when the device is configured and started, and does not change for the life of the run. This is a decided constraint on the cmc architecture and it removes a great deal: no priority ladder, no arbitration between concurrent commanders, no observer mode, and no handover path that would otherwise have to pass through a safe state to avoid a step discontinuity in the setpoint.

`Cia402Core` therefore holds a single `Transport*`, resolved at startup from configuration. The unselected transports need not be constructed at all, which is simpler than building them and leaving them idle.

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
- **Retention** — XMC4 and AM335x/TI HALs and demos are kept (§1.2).
- **Transport** — exactly one, fixed at configuration; no ROS in an EtherCAT build (§5.3).

## Open questions

- **`SyncErrorCounterLimit` value** — to be derived from a measured `cyclictest` histogram at the chosen cycle time, against the ETG.1020 semantics in §3.3.1.
- **Telemetry in a ROS-free build** — which quantities must become TxPDO entries or SDO-readable objects. This has to be answered before Phase 4 fixes the PDO layout, since adding entries later means re-deriving the SM arithmetic (§4.3).
- **Transport selection mechanism** — compile-time build variants or a runtime configuration switch. "No ROS in an EtherCAT build" is achievable either way, but the choice affects how `main()` and the CMake targets are structured.
