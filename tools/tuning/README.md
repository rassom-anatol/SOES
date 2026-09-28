# Real-time tuning for the Linux slave

Changes to the host that affect the cyclic loop, kept here so they are
reviewable, version controlled and reversible rather than typed into a shell
once and forgotten.

**Measure after each one, separately.** Every figure quoted below is in
[`../../docs/measurements.md`](../../docs/measurements.md) with the commit and
configuration it was taken under. Applying several changes at once and
measuring the total is how an earlier round of this work attributed a 34x
improvement to the wrong cause for most of a session.

## Applied

### `cpu-governor.service` — pin the governor to performance

The one that mattered. Per-transfer p99 350 us to 10.2 us; worst cycle in five
seconds 10131 us to 1091 us against a 1000 us period. Install and revert
instructions are in the unit file's header.

This is **not** persistent without the unit: a reboot restores `ondemand` and
the cycle silently regresses to missing roughly half its deadlines. That is the
whole reason this file exists rather than a note saying "run this command".

## Considered and deliberately not applied

### Core isolation — `isolcpus`, `nohz_full`, `irqaffinity`, `skew_tick`

Aimed at scheduling jitter. With the performance governor applied, run-queue
wait measured **0.01 us per cycle** and the worst cycle was 91 us over the
period, so there is no scheduling jitter left to remove. Deferred until
something demands it; it needs a boot-parameter edit with physical access to
recover from, which is not worth spending on insurance against a measured
non-problem.

The parameters, if it ever is needed, plus the tryboot A/B file layout and the
recovery path, are documented in
[`../../docs/rpi-platform.md`](../../docs/rpi-platform.md) section 5.

### `SCHED_FIFO` for the cyclic thread

Same reasoning: run-queue wait is 0.01 us per cycle. Worth adding when
isolation is, not before.

### Everything in a master-side tuning script that touches the NIC

NAPI threading, interrupt coalescing, flow control, ring sizes, offloads,
queueing disciplines, XPS and RSS are all inert here. The LAN9252 is an
EtherCAT Slave Controller: it handles Ethernet in hardware and Linux never sees
a frame. There is no NIC in the real-time path.

Likewise `processor.max_cstate`, `intel_idle.max_cstate` and `mce=ignore_ce`
are x86-only, and this kernel exposes no `cpuidle` states at all.

## Still to do

- **`swapoff -a`** and removing the swapfile from `/etc/fstab`. A 2 GB swapfile
  is currently active. This removes an unbounded rare failure rather than a
  measured one, so it needs no measurement to justify and none to verify.
- **`mlockall`** in the application, for the same reason.
