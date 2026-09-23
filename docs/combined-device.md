# Combined device: CiA402 axis plus digital and analog I/O

What the EtherCAT standard asks for if this device is to present general-purpose
digital and analog I/O alongside the drive axis. This is an EtherCAT-level question
rather than a drive-profile one, which is why it lives here and not in
[`cia402-requirements.md`](cia402-requirements.md).

Nothing here is decided. The point of the document is that **one part of it has a
deadline** and the rest does not.

Items marked **[verify]** should be read out of the relevant ETG specification before
being relied on.

---

## 1. The deadline

Roadmap §4.3 reserves **64 bytes out and 96 in**, sized as 4 axes x 16/24, and states
plainly that SyncManager addresses live in the ESI and the SII image, so enlarging them
later breaks the ESI and forces a re-scan in the master.

**Digital and analog I/O consume PDO bytes from that same reservation.** If this device
may ever carry I/O, the byte budget has to account for it *before the ESI is generated*
— either by lowering the axis reservation, by raising `SM3_sma`, or by extending both.
There is roughly 2.9 KB of DPRAM spare, so the room exists; what does not exist is a
second chance to choose.

This is the same question as the existing "maximum axis count" open item, and it should
sit beside it: **what is the largest process image this device will ever present?**

Everything else in this document can be decided later. This cannot.

---

## 2. The index ranges collide

CiA402 reserves **0x6000-0x67FF for axis 0**, and 0x800 per axis thereafter — the stride
roadmap §4.4 already uses. The generic I/O profile, CiA401, places its objects at:

| Index | Object |
|---|---|
| 0x6000 | Read digital input 8-bit |
| 0x6100 | Read digital input 16-bit |
| 0x6200 | Write digital output 8-bit |
| 0x6401 | Read analog input 16-bit |
| 0x6411 | Write analog output 16-bit |

Every one of them falls inside axis 0's range. **Adding the CiA401 object set to this
dictionary is not available.**

A second consequence: **0x1000 Device Type names one profile.** It currently reads
`0x00020192` — servo drive, profile 402. A device that is genuinely both cannot say so
in 0x1000 without moving to the modular device approach below.

---

## 3. Three ways out

| Option | What it costs |
|---|---|
| **A — I/O in the manufacturer range 0x2000-0x5FFF** | No profile-level interoperability for the I/O: a master sees named PDO entries, not a recognised I/O device. Zero collision risk, no new machinery, 0x1000 stays 402. |
| **B — Modular Device Profile (ETG.5001)** | The drive and the I/O become modules with per-slot index ranges and a module ident list (0xF000 / 0xF010 / 0xF030 / 0xF050). Real conformance, but it restructures the entire dictionary and the master then performs module configuration matching against the ident list. **[verify]** the exact object set, the 0x1000 value for a modular device, and whether ETG.5001 defines a drive module profile that can carry CiA402. |
| **C — I/O at an unused axis offset** | Looks tidy and is subtly wrong. A master that expects a CiA402 axis at index + 0x800 finds digital inputs instead. Rejected. |

**Recommendation: option A.** The vendor range already holds 0x2000-0x2002, the master
is our own configuration rather than an arbitrary third party, and the I/O half gains
nothing from profile conformance that a correct ESI does not already provide. Option B
is the right answer for a product sold as a modular I/O system, which this is not.

The decision is worth recording explicitly rather than arrived at by default, because
it is the one that cannot be reversed cheaply once the ESI ships.

---

## 4. What follows, assuming option A

- [ ] **A second PDO pair, not a widened one.** One PDO per functional block is the
      convention: 0x1601 / 0x1A01 for the I/O alongside 0x1600 / 0x1A00 for the axis,
      both listed in 0x1C12 / 0x1C13 with sub-index 0 becoming 2. The mapping and
      assignment objects stay `ATYPE_RO` per roadmap §4.2. The reason to split rather
      than extend is that it keeps the axis image byte-stable if the I/O block changes
      later.

- [ ] **Outputs must have a defined value when the bus is not driving them.** This is the
      profile requirement that survives the move into the manufacturer range. CiA401
      carries error mode and error value objects for digital (0x6206 / 0x6207) and
      analog (0x6443 / 0x6444) outputs precisely because *what does the output do when
      the master goes away* has to be answerable per channel.
      SOES provides the mechanism — the `safe_state_override` hook, roadmap §5.3 — but
      the mechanism is only reached if something notices the master left. See
      [`stack-review.md`](stack-review.md) §1.2: with no watchdog wired up, outputs hold
      their last commanded value indefinitely. For a drive that is bad; for a drive
      with digital outputs it is bad with a wider blast radius.
      The fault value should be **configurable per channel**, not compiled in.

- [ ] **The analog scaling contract has to be written.** CiA401 would have supplied the
      raw-to-engineering-unit conversion, the range and the limit objects. In the
      manufacturer range the units, the scaling, the over-range indication and the
      resolution are all ours to define and document — the same treatment the torque
      scaling gets in roadmap §5.3, and for the same reason: easy to get wrong, and
      expensive to rediscover from the master side.

- [ ] **Analog sampling lands in the cyclic budget.** An ADC on the carrier means more
      SPI on a thread already measured at ~141 µs median for the EtherCAT half, plus the
      TMC4671 half. Whether analog channels are sampled every cycle or at a divided rate
      is a roadmap §3.4 question and should be answered there, not in the dictionary.
      Digital I/O through the gpiochip UAPI is comparatively cheap but is not free
      either.

- [ ] **ESI.** Additional `<RxPdo>` and `<TxPdo>` elements with correct signedness. This
      was a live bug for the axis PDOs — [`stack-review.md`](stack-review.md) §2.1 — and
      is now fixed at the source: the generator emits the ESI data type from the
      declared YAML type, so new PDOs inherit the fix rather than the bug. `<Fmmu>` is unchanged: still one Outputs
      and one Inputs entry regardless of how many PDOs feed them. With a fixed process
      image the I/O PDOs are not optional, so no `Exclude` attributes are needed.

- [ ] **The generator has to learn about blocks.** `od.yaml` currently has one `rxpdo`
      and one `txpdo` key. Supporting a second pair is a structural change to
      [`gen_od.py`](../tools/gen_od.py) and to the SyncManager arithmetic it computes,
      and it should land at the same time as the fixes in
      [`stack-review.md`](stack-review.md) §2 rather than after them.

---

## 5. Open questions

- **Will this device carry I/O at all?** Everything above is contingent on it, and the
  reservation deadline in §1 means the answer is needed before the ESI ships even if the
  I/O itself is years away.
- **How many channels, of what width?** The reservation needs a byte count, not a
  design. Digital in/out as one 16-bit word each and *n* analog channels at 16 bits is
  enough to size it.
- **Do any outputs have a safety-relevant fault value?** If yes, the per-channel fault
  value stops being a configuration convenience and becomes part of the same argument as
  the drive's safe state.
