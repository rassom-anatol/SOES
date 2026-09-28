# Raspberry Pi platform notes

Facts about the BCM2711 and its Linux support that bear on this project, kept
here so a board design does not have to rediscover them. Everything below was
read off a running Pi 4 (Ubuntu, kernel 7.0.0-1017-raspi) rather than recalled,
and the source is named in each case.

**The CM4 caveat applies throughout.** These are the controllers and pin groups
the Raspberry Pi firmware's overlay set exposes on a Pi 4. The CM4 brings out
more GPIO than the 40-pin header does, so there may be routes available there
that are not listed here. Treat this as a starting point for a schematic and
confirm against the CM4 datasheet and the BCM2711 peripherals document before
committing copper.

---

## 1. SPI controllers

Source: `/boot/firmware/current/overlays/README` and the overlay file list on
the target.

| Controller | Type | MISO / MOSI / SCLK | CS0 | CS1 | Driver |
|---|---|---|---|---|---|
| SPI0 | full | 9, 10, 11 | 8 | 7 | `spi-bcm2835` |
| SPI3 | full | 1, 2, 3 | 0 | 24 | `spi-bcm2835` |
| SPI4 | full | 5, 6, 7 | 4 | 25 | `spi-bcm2835` |
| SPI5 | full | 13, 14, 15 | 12 | 26 | `spi-bcm2835` |
| SPI6 | full | 19, 20, 21 | 18 | 27 | `spi-bcm2835` |
| SPI1 | AUX | 19, 20, 21 | 18 | 17, 16 (CS2) | `spi-bcm2835aux` |
| SPI2 | AUX | — | — | — | `spi-bcm2835aux` |

**Five full-featured controllers can run at once** — SPI0, SPI3, SPI4, SPI5 and
SPI6 — which is four axes plus the EtherCAT slave controller, each on its own
bus. They share the `spi-bcm2835` driver with SPI0 rather than the AUX driver,
so they should behave like SPI0.

Using CS0 only, those five consume GPIO **0-15 and 18-21**, leaving **16, 17,
22, 23, 24, 25, 26, 27** for interrupts, SYNC0, resets and enables. Eight pins
of side-band for five devices is workable but not generous.

### The AUX controllers are not equivalent

SPI1 and SPI2 are part of the AUX peripheral block, which also contains the
mini-UART. Two consequences measured on this project:

- The AUX chip select is a GPIO the driver toggles, not a hardware strobe. That
  is why batching several frames into one `SPI_IOC_MESSAGE` measured *slower*
  in proportion to how many were packed, where on SPI0 a split transfer costs
  about 3 us.
- The block is shared with the mini-UART, so SPI1 traffic and `/dev/ttyS0`
  traffic contend for one bus and one interrupt.

The LAN9252 currently sits on SPI1 because that was what the evaluation wiring
allowed. On a custom board it should go on one of the full controllers.

### Two routing traps

**SPI0's CS1 is GPIO7, which is also SPI4's SCLK.** If SPI4 is used then SPI0
is single-chip-select only, so "two axes sharing SPI0 on CE0 and CE1" is not
available alongside SPI4.

**SPI1 and SPI6 occupy the same pins** (19, 20, 21 with CS0 on 18). They are
alternatives, not additions.

---

## 2. SPI and UART contend for the same pins

SPI3-6 and UART2-5 are alternate functions of the same GPIO groups. Each group
provides one or the other, never both:

```
GPIO  0- 3    SPI3   or   UART2
GPIO  4- 7    SPI4   or   UART3
GPIO  8-11    SPI0   or   UART4
GPIO 12-15    SPI5   or   UART5      (UART0 and the mini-UART also default here)
GPIO 16-21    SPI1 / SPI2 (AUX)   or   SPI6
```

**A five-SPI-bus design therefore leaves no UART at all**, including the primary
one: UART0 and the mini-UART both default to GPIO 14/15, which SPI5 would be
using. Since the motion controller currently drives TMC4671 telemetry over
`/dev/ttyS0` at 921600 baud, this is a real decision rather than a detail —
per-axis SPI or a telemetry UART, not both.

---

## 3. CPU frequency is a single domain

```
affected_cpus: 0 1 2 3     related_cpus: 0 1 2 3     policies: policy0
```

All four A72 cores share one clock and one governor. Per-core frequency policy
is not possible; the governor is all four cores or none.

Available steps are 600 MHz to 1500 MHz in 100 MHz increments, driver
`cpufreq-dt`.

**This matters more than it looks.** With no SPI or DMA interrupts (see below),
SPI transfer time on this platform *is* CPU time, so it scales with core clock.
The `ondemand` governor treats a core that sleeps most of each cycle as idle and
parks it near 600 MHz, and every wake then pays a ramp-up inside the transfer.
See `measurements.md` for what that cost.

What fixed it was the frequency being *constant*, not it being high — so if
thermal headroom becomes a constraint, pinning a lower fixed frequency keeps the
benefit. At 48 C with one axis running there is currently about 32 C of margin
before the Pi 4 soft-throttles at 80 C.

---

## 4. There are no SPI or DMA interrupts

```
15,16,21,22   DMA IRQ        0 0 0 0
28            fe215080.spi   0 0 0 0     (AUX, SPI1)
38            fe204000.spi   0 0 0 0     (SPI0)
```

Zero on every core after hours of cyclic traffic. For transfers of our size the
`spi-bcm2835` family polls in-kernel rather than taking an interrupt or setting
up DMA.

Three consequences worth carrying into any tuning or board work:

- **Pinning and prioritising an SPI interrupt is not available to us.** The
  usual real-time recipe of giving the device interrupt a dedicated core and an
  RT priority has no equivalent here.
- **SPI time is CPU time**, which is why core frequency dominates (section 3).
- A cycle's cost is therefore CPU-bound, and adding a bus does not add an
  interrupt load, but it does add serial CPU work unless the buses are driven
  from different cores.

---

## 5. Boot configuration layout

This image uses the tryboot A/B scheme (`autoboot.txt` contains
`tryboot_a_b=1`), so there is **no `/boot/firmware/cmdline.txt`**. The kernel
command line lives in two places, normally identical:

```
/boot/firmware/current/cmdline.txt    active; matches /proc/cmdline
/boot/firmware/new/cmdline.txt        candidate for the next A/B switch
```

Edit `current/` to change the running configuration and mirror it into `new/`
so a kernel update does not silently revert it. The file must remain a single
line; a newline in the middle truncates the arguments and the Pi will not boot.
`/boot/firmware` is the FAT partition `mmcblk0p1`, so recovery from a bad edit
means reading the SD card on another machine — which requires physical access.

Kernel support relevant to isolation, from `/boot/config-$(uname -r)`:
`CONFIG_NO_HZ_FULL=y`, `CONFIG_CPU_ISOLATION=y`, `CONFIG_HZ_1000=y`,
**`CONFIG_PREEMPT_RT` absent** — the running kernel is `PREEMPT_DYNAMIC`.
