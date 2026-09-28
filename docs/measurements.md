# Measurement log

Every timing figure quoted anywhere in this repository should appear here first,
with the commit it was taken at and the configuration it was taken under.

**Why this file exists.** Numbers from this project have twice been compared
across runs that differed in loop structure, governor and axis count, and the
difference read as a regression that had not happened. A figure without its
configuration is not a measurement, it is an anecdote. In particular:

- **Quote medians and percentiles, never means.** The cycle-time distribution
  has a tail running to ten milliseconds under some configurations, and a mean
  over that describes nothing real. One earlier comparison of two means — one
  from a tight distribution, one from a long-tailed one — produced a claimed
  10x regression that did not exist.
- **Never divide a figure from one run by a figure from another.** A
  "per-transfer cost" obtained by dividing one run's CPU time by another run's
  transfer count was wrong by a factor of two and a half.
- **Record the governor.** It turned out to matter more than everything else
  put together.

---

## Configuration axes that change the numbers

| Axis | Values seen so far |
|---|---|
| Loop structure | free-running `ecat_slv()` / SYNC0-driven `ecat_slv_run_dc()` |
| CPU governor | `ondemand` (600-1500 MHz) / `performance` (1500 MHz fixed) |
| Axis count | 1 / 2 / 4 (distinct dictionary variants, distinct product codes) |
| SPI clock | 25 MHz (the measured knee; see 2026-09 entry) |
| Master cycle | SYNC0 period, and separately the master's process data rate |
| Scheduling | `SCHED_OTHER`, shared core, no isolation (as of this writing) |

---

## 2026-09-27 — governor is the dominant factor

Commit `bec6dbc`. Target: Pi 4, Ubuntu kernel 7.0.0-1017-raspi
(`PREEMPT_DYNAMIC`, not RT), no core isolation, `SCHED_OTHER`, LAN9252 on SPI1
at 25 MHz. Master: TwinCAT, DC enabled, SYNC0 1 ms. Dictionary variant
`cmc_drive_1ax`. Sync error limit raised to 60000 for the duration so the slave
would stay in OP long enough to sample.

All figures are medians and percentiles over 5 s windows, from the application's
own reporting.

### Loop structure, both under `ondemand`

| | free-running | SYNC0-driven |
|---|---|---|
| SPI per cycle, median | 89.9 us | 301.4 us |
| SPI per cycle, p99 | 140.3 us | 6100.6 us |
| Transfers per cycle, median / p99 / max | 9 / 15 / 15 | 21 / 21 / 21 |
| Per transfer, median | 10.0 us | 14.8 us |
| Per transfer, p99 | 12.5 us | 292.3 us |
| CPU per cycle | 99.3 us | 308.3 us |
| Run-queue wait per cycle | 0.21 us | 7.82 us |
| Cycles per second | ~10,000 | ~570 |

The free-running loop does roughly half the work: its TxPDO path never executes,
which is why its transfer count is 9 rather than 21. The two are **not**
comparable as a before-and-after.

### Governor, both SYNC0-driven

| | `ondemand` | `performance` |
|---|---|---|
| SPI per cycle, median | 293.8 us | **156.4 us** |
| SPI per cycle, p99 | 7340.6 us | **205.4 us** |
| Per transfer, median | 14.5 us | **9.3 us** |
| Per transfer, p99 | 349.6 us | **10.2 us** |
| Wall median / p99 / max | 959 / 8064 / 10131 us | **999 / 1069 / 1091 us** |
| Run-queue wait per cycle | 7.92 us | **0.01 us** |
| Cycles per second | ~559 | **~997** |

**Per-transfer p99 improved 34x.** With `performance` the loop catches every
SYNC0 edge and the worst cycle in five seconds is 1091 us against a 1000 us
period.

The mechanism is section 3 and 4 of `rpi-platform.md`: there are no SPI or DMA
interrupts, so SPI time is CPU time, and `ondemand` parked a core that sleeps
most of each millisecond near 600 MHz. Every wake paid a ramp-up *inside* the
transfer, which is why it presented as slow SPI rather than as scheduling delay.

**Not the cause, despite looking like it:** scheduling (run-queue wait was 8 us
per cycle before the change, 0.01 us after), the HAL's busy-poll on the CSR
BUSY bit (transfer count was a flat 21 with zero variance, and that poll costs
transfers rather than microseconds), and the AUX SPI controller (suspected on
circumstantial grounds, exonerated by the governor result).

### Still outstanding at this commit

Process data arrives on **50%** of SYNC0 periods — the master is delivering
frames at 500 Hz against a 1000 Hz SYNC0. The slave keeps up; the master's
cyclic task rate does not match the commanded cycle. The sync error counter
detects this correctly and would trip at the real limit of 24.

### Temperature

48.2 C at 1.5 GHz with one axis running. Pi 4 soft-throttles at 80 C.

---

## Earlier figures, and their status

Recorded for traceability. Configuration was not captured at the time, which is
why this file now exists.

| Figure | Where quoted | Status |
|---|---|---|
| 141 us median, 220 us p99, 15.7 transfers | roadmap 3.4 | Superseded. Free-running loop before AL-event tail suppression. |
| 93 us median, 148 us p99, 9.5 transfers | roadmap 3.4 | Consistent with the 89.9 us median measured above for the same loop structure. |
| "roughly 850 us of a 1 ms period remains" | roadmap 3.4 | **Withdrawn.** Arithmetic on the free-running figure, extrapolated to a blocking loop without flagging the assumption. The loop structure changes the answer. |
| 7.7 us per transfer | conversation | **Withdrawn.** Mean divided by mean. The correct figure for that configuration is 10.0 us median. |
| 38 us per transfer | conversation | **Withdrawn.** One run's CPU time divided by another run's transfer count, while the system was in a fault loop. The correct figure is 14.8 us median. |
| TMC4671 on SPI0: 67.1 us at 1 MHz, 26.8 us at 2 MHz, 13.3 us at 8 MHz split read | roadmap 5.4 | Medians only; no percentiles taken, and measured with no device attached. Valid for transaction cost, says nothing about a real TMC4671's tail. |

---

## What to record for each future entry

Commit SHA, kernel, governor, loop structure, axis count / dictionary variant,
SPI clock, master cycle and process data rate, scheduling policy and isolation.
Then medians and p99 for: SPI per cycle, transfers per cycle, per transfer, wall
per cycle, CPU per cycle, run-queue wait. Temperature if the governor or clock
changed.
