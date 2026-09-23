# CiA402 requirements checklist

What the drive profile obliges, expressed as things to verify rather than things to
build. Companion to [`cia402-roadmap.md`](cia402-roadmap.md) (what we are building) and
[`stack-review.md`](stack-review.md) (what is currently wrong). If the device is also
to carry general-purpose digital and analog I/O, see
[`combined-device.md`](combined-device.md) — the object ranges collide with the CiA402
axis range and the SyncManager reservation has to account for it before the ESI ships.

Items marked **[verify]** are ones to read out of **ETG.6010** / IEC 61800-7-201 before
relying on them — either the profile text qualifies them, or they depend on choices
ETG.6010 makes for the EtherCAT mapping specifically. Everything unmarked is
unambiguous in the profile.

Scope is the v1 mode set from roadmap §4.2: **csp, csv, cst, single axis**.

---

## 1. Commitments already made that carry obligations

These are the highest-priority items: the dictionary as generated today already claims
things, and each claim brings required behaviour with it.

- [ ] **0x605A Quick stop option code is `2`, which commits the drive to a ramp.**
      Option code 2 means *decelerate on the quick stop ramp, then transition to Switch
      on disabled*. That requires a quick stop deceleration (**0x6085**) and an actual
      ramp implementation. Option code 1 requires 0x6084 instead. Only option code 0 —
      immediate disable, coast to stop — is free of further obligation.
      **Until a ramp exists, 0x605A should read 0.** A drive that advertises a
      controlled stop and coasts instead is worse than one that admits it coasts.

- [ ] **0x6502 = `0x000003A0` claims homing mode (bit 5).** Homing brings its own
      mandatory set — **0x6098** homing method, **0x6099** homing speeds, **0x609A**
      homing acceleration, **0x607C** home offset — plus homing-specific statusword
      semantics on bits 10 and 12. Reduce to `0x380` (csp, csv, cst) unless homing is
      actually in scope. See [`stack-review.md`](stack-review.md) §2.6.

- [ ] **Controlword bit 8 (Halt) applies in all cyclic modes and has no implementation.**
      It is not mode-specific and not optional. Its behaviour is governed by **0x605D**
      halt option code; absent that object, the profile's default for the halt action
      still applies.

- [ ] **The option codes define required behaviour whether or not the objects exist.**
      0x605B shutdown, 0x605C disable operation, 0x605D halt, 0x605E fault reaction.
      Omitting an object does not omit the obligation — it fixes the behaviour at the
      profile default. 0x605E is the significant one for a servo drive: it governs what
      the drive does between detecting a fault and reaching the Fault state.

---

## 2. State machine

The exhaustively specified part of the profile, and therefore the part that admits an
exhaustive test (roadmap §5.3).

- [ ] **Eight states, not nine.** Roadmap §5.3 says "nine states"; the profile defines
      **eight** — Not ready to switch on, Switch on disabled, Ready to switch on,
      Switched on, Operation enabled, Quick stop active, Fault reaction active, Fault —
      plus a *Start* pseudo-state that makes nine boxes in the published diagram. Fix
      the cardinality before writing the test table.
      **Done** — roadmap §5.3 now says eight and says why the ninth box is not a state.

### 2.1 Command encoding — controlword bits 7, 3, 2, 1, 0

| Command | b7 | b3 | b2 | b1 | b0 | Transitions |
|---|---|---|---|---|---|---|
| Shutdown | 0 | x | 1 | 1 | 0 | 2, 6, 8 |
| Switch on | 0 | 0 | 1 | 1 | 1 | 3 |
| Switch on + enable operation | 0 | 1 | 1 | 1 | 1 | 3 + 4 |
| Disable voltage | 0 | x | x | 0 | x | 7, 9, 10, 12 |
| Quick stop | 0 | x | 0 | 1 | x | 7, 10, 11 |
| Disable operation | 0 | 0 | 1 | 1 | 1 | 5 |
| Enable operation | 0 | 1 | 1 | 1 | 1 | 4, 16 |
| Fault reset | rising edge | x | x | x | x | 15 |

- [ ] **Bit 2 (Quick stop) is active low.** A controlword of 0x0000 is therefore a quick
      stop request, not an idle value.
- [ ] **Fault reset is edge-triggered, not level.** A controlword arriving every cycle
      with bit 7 set must produce exactly one reset attempt, not one per cycle.
- [ ] Bits 4, 5, 6 and 9 are mode-specific; bit 10 reserved; bits 11-15
      manufacturer-specific.

### 2.2 State encoding — statusword bits 6, 5, 3, 2, 1, 0

| State | b6 | b5 | b3 | b2 | b1 | b0 |
|---|---|---|---|---|---|---|
| Not ready to switch on | 0 | x | 0 | 0 | 0 | 0 |
| Switch on disabled | 1 | x | 0 | 0 | 0 | 0 |
| Ready to switch on | 0 | 1 | 0 | 0 | 0 | 1 |
| Switched on | 0 | 1 | 0 | 0 | 1 | 1 |
| Operation enabled | 0 | 1 | 0 | 1 | 1 | 1 |
| Quick stop active | 0 | 0 | 0 | 1 | 1 | 1 |
| Fault reaction active | 0 | x | 1 | 1 | 1 | 1 |
| Fault | 0 | x | 1 | 0 | 0 | 0 |

Remaining bits: 4 voltage enabled, 7 warning, 9 remote, 10 target reached (unused in the
cyclic modes), 11 internal limit active, 12 and 13 mode-specific, 8 and 14-15
manufacturer-specific.

### 2.3 Transitions

- [ ] **0 and 1 are automatic** — power-on/reset, then initialisation complete.
- [ ] **2 through 12 are command-driven**, per the table in §2.1.
- [ ] **13 is reachable from every state** and is the only transition not driven by the
      master: any state → Fault reaction active on fault detection.
- [ ] **14 is automatic** once the fault reaction completes: Fault reaction active → Fault.
- [ ] **15 requires the fault to have cleared.** Fault → Switch on disabled on a rising
      edge of controlword bit 7, *and only if the fault condition is gone*. A reset
      request with the fault still present must be refused, leaving the drive in Fault.
- [ ] **16 exists only when 0x605A is 5-8** (Quick stop active → Operation enabled).
      With 0x605A = 2 this transition must not be reachable.

Two properties worth asserting in the unit test beyond reproducing the table:

- [ ] No controlword value moves the drive out of Fault to anywhere except Switch on
      disabled.
- [ ] Every state can be driven to Fault reaction active by the fault input.

---

## 3. Mandatory objects

### 3.1 Unconditional

- [ ] 0x6040 Controlword — present
- [ ] 0x6041 Statusword — present
- [ ] 0x6060 Modes of operation — present
- [ ] 0x6061 Modes of operation display — present
- [ ] 0x6502 Supported drive modes — present, value wrong (§1)

### 3.2 Per mode

| Mode | Required | Status |
|---|---|---|
| csp | 0x607A target position, 0x6064 position actual value | present |
| csv | 0x60FF target velocity, 0x606C velocity actual value | present |
| cst | 0x6071 target torque, 0x6077 torque actual value, 0x6072 max torque, 0x6076 motor rated torque | present |

- [ ] **[verify] 0x60C2 Interpolation time period.** Mandatory in interpolated position
      mode. For csp its necessity depends on where the setpoint interval comes from: with
      DC-Synchron the period is the master-configured SYNC0 period and is readable via
      0x1C32:02, but without DC the drive has no other way to learn it. Check what
      ETG.6010 requires for csp specifically before omitting it — this is the single
      object in the v1 set whose status I would not assume.

- [ ] **0x6060 must reject modes 0x6502 does not claim.** Writing an unsupported mode
      must not take effect, and 0x6061 must not follow it. The generated dictionary
      currently accepts any `int8`.

---

## 4. Mode-specific statusword semantics

- [ ] **Bit 12 = "drive follows the command value"**, in all three cyclic modes. This is
      the drive's per-cycle acknowledgement that the setpoint was consumed, and it is
      what a master checks first.
- [ ] **Bit 13 = "following error"**, in csp. Meaningful only with a following error
      window (**0x6065**) and timeout (**0x6066**), neither of which is in the
      dictionary. 0x60F4 (following error actual value) is mapped but is a measurement,
      not a trip condition. **Until 0x6065/0x6066 exist, bit 13 must read 0** rather
      than being driven from an ad-hoc threshold.
- [ ] Bit 13 is reserved in csv and cst.
- [ ] Bit 10 (target reached) is not used in the cyclic modes.
- [ ] Bit 11 (internal limit active) should reflect torque, current or position limiting
      — it is the standard channel for "the drive is clamping your setpoint", and
      0x6072 Max torque makes it reachable.

---

## 5. Units and scaling

- [ ] **Torque is per-thousandth of rated torque.** 0x6071, 0x6072 and 0x6077 are all in
      units of 1/1000 of **0x6076** motor rated torque (mNm). Already correct in roadmap
      §5.3.
- [ ] **Position scaling** comes from 0x608F position encoder resolution, 0x6091 gear
      ratio and 0x6092 feed constant — all present.
- [ ] **Velocity and acceleration scaling are absent.** The factor group also defines
      **0x6094** velocity encoder factor, **0x6096** velocity factor and **0x6097**
      acceleration factor. Without them the units of 0x606C and 0x60FF default to
      increments per second, and a master has to be told out of band. This needs to be a
      decision rather than an omission, because a master doing unit conversion — which
      is the reason §4.2 added the position scaling objects at all — needs the velocity
      chain for the same reason.
- [ ] **[verify]** Whether ETG.6010 requires the whole factor group once any of it is
      present, or permits the position-only subset.

---

## 6. Error handling

- [ ] **0x603F Error code takes values from the standard emergency error code table**,
      not from an internal enumeration. The relevant ranges are the CANopen ones —
      0x2310 continuous over-current, 0x3210 DC link over-voltage, 0x4210 excess
      temperature, 0x7305 incremental sensor fault, 0x8611 following error, and so on.
      Mapping TMC6200 GSTAT bits onto arbitrary values here would be non-conformant;
      the vendor-specific detail belongs in 0x2000, where roadmap §4.2 already puts it.
      **The translation from GSTAT bits to standard error codes is a piece of design
      work that does not currently exist anywhere in the plan.**
- [ ] 0x603F holds the code for the most recent fault and must be valid whenever
      statusword bit 3 is set.
- [ ] **[verify]** Whether ETG.6010 treats 0x603F as mandatory. It is the only standard
      channel for *what* failed, so it should be treated as required in practice
      regardless.
- [ ] Statusword bit 7 (warning) is the non-latching counterpart — a condition that does
      not force a state change. Decide whether anything drives it.

---

## 7. Behavioural requirements that shape the architecture

- [ ] **The state machine is evaluated every cycle from that cycle's controlword.** It is
      not an event handler hung off SDO writes. This fixes where `Cia402Sm` sits
      relative to the RxPDO unpack in roadmap §5.3.
- [ ] **One setpoint consumed per cycle, acknowledged in statusword bit 12.** A cycle in
      which the setpoint was not consumed — missed frame, late thread — must clear bit
      12 rather than silently reusing the previous value. This is a protocol-level
      representation of the missed-cycle condition, distinct from 0x1C32:0B and from the
      sync error counter in roadmap §3.3.1, and it is the one a master reacts to first.
- [ ] **A fault must reach the master in the cycle it occurs.** Roadmap §4.2 already puts
      the vendor status word in the TxPDO for this reason; the same argument applies to
      statusword bit 3 and 0x603F, both of which are mapped.
- [ ] **Mode changes are acknowledged, not assumed.** 0x6061 reflects the mode actually
      in effect, which may lag 0x6060 by a cycle or be refused outright.

---

## 8. Not in scope for v1, listed so the omission is deliberate

Each of these is optional in the profile, and each is the kind of thing a master may
expect from a "normal" servo drive:

- 0x607E Polarity — direction inversion. Commonly needed at commissioning.
- 0x607D Software position limit.
- 0x6065 / 0x6066 Following error window and timeout — see §4.
- 0x6083 / 0x6084 Profile acceleration and deceleration — only required by the profiled
  modes, which v1 does not claim, but 0x6084 becomes required if 0x605A is set to 1.
- 0x6085 Quick stop deceleration — **required** if 0x605A is 2, per §1.
- 0x60B0 / 0x60B1 / 0x60B2 Position, velocity and torque offsets.
- 0x60B8-0x60BD Touch probe group.
- 0x6007 Abort connection option code — what the drive does when the fieldbus
  connection drops. Interacts directly with the watchdog work in
  [`stack-review.md`](stack-review.md) §1.2 and is worth revisiting when that is
  settled, since it is the profile's own answer to the same question.
