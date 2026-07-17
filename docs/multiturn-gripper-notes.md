# Multi-turn Feetech gripper: feasibility notes

Context: custom parallel gripper = STS3215 (same as stock SO-101) driving a worm
gear → 4-bar compliant linkage. Total worm travel ≈ 35 rad ≈ **5.6 turns ≈
22,800 encoder ticks** (4096/turn). Optional external knob lets a human spin the
worm shaft directly (the output is non-backdrivable, the worm shaft is not).

All paths below are under `lerobot/src/lerobot/`.

## Verdict: yes, multi-turn is feasible with small, local changes

### 1. The STS3215 already supports multi-turn feedback — LeRobot disables it

`motors/feetech/feetech.py:219-226` (`configure_motors`): LeRobot clears **bit 4
of the Phase register (addr 18)** on STS3215 specifically to "force position
readings to be in the range [0, resolution-1] and prevent overflow or negative
values". I.e. with bit 4 *set*, Present_Position accumulates across turns
(goes negative / past 4095). For the gripper motor we simply skip that clearing
(or set the bit).

### 2. The wire format has the headroom

`motors/feetech/tables.py` (`STS_SMS_SERIES_ENCODINGS_TABLE`):
Present_Position / Goal_Position are 2-byte **sign-magnitude, sign bit 15** →
±32,767 ticks = **±8 turns = ±50 rad**. Our 5.6-turn stroke fits even if homed
at one end; comfortably if homed mid-stroke.

### 3. Calibration software model is range-based and doesn't care about turns

- `motors_bus.py` `MotorCalibration` stores plain-int `range_min`/`range_max`;
  `_normalize`/`_unnormalize` linearly map that to the gripper's
  `RANGE_0_100` norm mode (`robots/so_follower/so_follower.py:59`). Nothing in
  software caps the range at 4095 → a 22,800-tick range normalizes fine, and
  everything upstream (teleop, datasets, policies) sees the same 0–100 gripper.
- `record_ranges_of_motion` (`motors_bus.py:802`) just min/max-tracks live
  positions with torque off → **the "turn the knob by hand" calibration flow
  works unchanged** in multi-turn feedback mode.

## The three real problems

### A. Homing_Offset can't represent multi-turn offsets

`Homing_Offset` is sign-magnitude with **sign bit 11 → ±2047 ticks (±half
turn)** (`tables.py:209`), which is why `set_half_turn_homings` exists. It
cannot recenter a 5.6-turn range. Fix: for the gripper, leave Homing_Offset at
0 and carry the full offset in `range_min`/`range_max` (software-side), or
handle offset entirely in a gripper-aware driver subclass.

### B. Power-cycle position ambiguity (the actual hard problem)

The encoder is absolute only within one turn; the turn count is volatile. After
a power cycle the motor reads somewhere in [0, 4095] and we don't know which of
the ~6 turns we're on. Mitigations, best first:

1. **Auto-home at connect (turnkey answer):** drive toward the closed hard stop
   with a low torque/current limit until Present_Position stops changing, call
   that `range_min`, re-seed the turn count. The compliant 4-bar makes the
   stall contact gentle. A few seconds at startup, zero user interaction.
2. Persist last-known position on disconnect and re-seed on connect — works
   *because* the worm is non-backdrivable, but breaks if someone turns the
   manual knob while unpowered. Good as a fast path with (1) as fallback/verify.
3. Physical index/limit switch — extra BOM, probably unnecessary given (1).

### C. `is_calibrated` round-trips through motor registers

`feetech.py` `read_calibration`/`write_calibration`/`is_calibrated` write
`range_min/max` into the motor's Min/Max_Position_Limit registers and compare
on connect. Multi-turn values (>4095 or negative) likely clamp in firmware →
`is_calibrated` would return False every boot and re-prompt calibration. The
gripper's range must live in the software calibration file only, with the
register round-trip bypassed for that motor. (Note: STS firmware convention is
min=max=0 limits for multi-turn operation — to be verified on hardware.)

## Other things to watch

- **Stroke speed:** 5.6 turns at STS3215 no-load speed (~1 rev/s at 12 V) ≈
  4–6 s full stroke. Fine for grasping, but teleop gripper tracking will lag
  the leader's trigger; consider worm lead / gearing choice with this in mind.
- **Grip force sensing:** through a worm stage, `Present_Load` no longer
  reflects grip force cleanly (worm friction dominates, and the linkage adds
  compliance). Current-based force estimates need characterization; the
  compliant linkage itself may be the better force regulator.
- **Leader side unchanged:** the leader gripper stays a stock direct-drive
  STS3215 producing 0–100; our follower consumes 0–100. All multi-turn detail
  stays inside the follower's motor handling.

## Sketch of the integration surface

1. Fork/variant of `SOFollower` (registered as e.g. `so101_wormgripper_follower`).
2. Gripper motor: keep Phase bit 4 set (multi-turn feedback), Operating_Mode
   position, Goal_Position writes as today (sign bit 15 covers the range).
3. `calibrate()`: normal flow for joints 1–5; gripper branch = knob-based
   `record_ranges_of_motion` **or** automated stall-homing (preferred).
4. `connect()`: gripper homing/verify step before torque-on.
5. Skip register round-trip for gripper calibration (problem C).

## VALIDATED ON HARDWARE (2026-07-14, follower_2 gripper servo on free-spin test board)

The working multi-turn recipe on a real STS3215 (differs from prior hypotheses):

- `Operating_Mode = 3` ("step servo") — despite docs suggesting relative steps,
  this firmware treats `Goal_Position` as an **absolute multi-turn target**
  (sign-magnitude, bit 15). Confirmed: absolute goals of 8192 (2 turns, 3.1 s)
  and 16384 (4 turns, 14.2 s) reached, `Present_Position` accumulating
  monotonically, zero wraps.
- `Phase` bit 4 set (multi-turn feedback), as hypothesized.
- `Max_Position_Limit` **widened, not zeroed**: the register accepts >4095
  (28672 verified). Goals clamp to the [Min,Max] window in every mode —
  and `0/0` limits mean "clamp to zero", NOT "limits off" (this drove the
  motor hard to position 0 in one test). Mode 0 also rejects out-of-window
  goals with an error status.
- `Goal_Velocity` must be set nonzero in mode 3 or the motor creeps (~75 t/s).
  2400 gave ~1.4 s/turn under no load.

Pitfalls discovered:

- **Latched Overload error (status bit 5)**: triggered by (a) enabling torque
  while outside the limit window, and (b) ~20 s of continuous high-speed
  travel (stock `Protection_Current`=250 is tuned for intermittent jaw moves).
  While latched, LeRobot raises on every write (error bit rides in every
  status packet). Clears on power cycle; sometimes self-clears when the
  violating condition ends. Driver must tolerate error bits and expose a
  clear-overload path; protection thresholds need retuning for multi-second
  worm strokes.
- Status bit 4 (sensor/angle error) appears transiently when goals fight the
  limit window.
- In mode 3 a new goal sent mid-move is NOT dropped (absolute target just
  updates) — earlier "dropped step" reads were the absolute-goal semantics.
- Broadcast ping doesn't work through these Waveshare adapters on macOS;
  use per-ID pings.
- Consequence for calibration convention: operate multi-turn motors in the
  POSITIVE domain [0, stroke] (home = 0 near a hard stop), since
  Min_Position_Limit=0 and negative range would clamp at zero.

## Bench session 2 (2026-07-14 PM): homing + protection fully validated

- **Overload mechanism decoded** (one-variable isolation): the servo latches
  Overload when `Present_Load` exceeds `Overload_Torque` (as % of max, e.g.
  25 -> ~250 load units) continuously for `Protection_Time` (x10 ms).
  `Protection_Current` is irrelevant to this trip. LeRobot's stock comment
  ("25% torque when overloaded") mis-describes the register: it is the trip
  THRESHOLD. Overload_Torque=80 completed 2-turn strokes that 25 always
  tripped at exactly ~2.0-2.7 s (scaling with Protection_Time).
- **Software unlatch**: Torque_Enable 0 -> 1 -> 0 clears the latch. No power
  cycle needed. Verified repeatedly, including mid-cruise auto-recovery.
- **Stall-homing rehearsal, 3/3**: cruise at Goal_Velocity=500,
  Max_Torque_Limit=400, detector = position frozen (<25 ticks) AND load >= 250
  over a 0.4 s window -> back off 300 ticks. Load pegs at exactly the torque
  limit (450/450/450) at the stop. Detection beats firmware protection when
  Overload_Torque is parked at 80 during homing.
- **Counter re-seed**: writing 128 to Torque_Enable recalibrates the
  multi-turn counter so current position reads exactly 2048 (verified
  12340 -> 2048). This is the missing homing primitive (Homing_Offset is
  limited to +/-2047 and useless here). Side effect: can latch a transient
  overload flag — clear it after.
- Cruising load on the free-spin rig is ~110-210 (11-21% duty) at 500 t/s —
  meaningful rig friction; a real worm stage will sit higher. Set
  Overload_Torque above cruise, below stall.
- All of the above is implemented in the plugin: `home_gripper()` (with
  mid-cruise re-seed so the positive-domain goal window can't exhaust),
  `clear_overload()`, `reseed_position()`. Homing at connect is controlled by
  `gripper_home_mode`: "stall" (default), "assume_closed" (human cranks the
  worm knob to closed while unpowered -> zero-motion re-seed; trust-based, no
  verification), or "off".

## Calibration/zeroing option space (2026-07-17, after real-gripper sessions)

Established: motor-side load is blind to finger contact through the worm's
mechanical advantage -> load-based stall homing unreliable here. Shipped UX:
park-closed-on-disconnect + verified zero-motion restore (`auto` mode), with
`assume_closed` as the one-time/manual fallback. Remaining options with
current hardware, in priority order:

1. Position-stagnation homing: slow drive, low torque cap, stop = <50 ticks
   net progress over 2-3 s (outlasts compliance creep; needs no load signal).
   Would restore fully-automatic homing for unknown states. Untested.
2. Home to the OPEN stop if the CAD gives it a rigid frame stop (crisper jam
   signature than the compliant fingertip end); zero = open - measured span.
3. Two-stop self-test: find both ends, compare span to measured 19,794 ->
   assembly/diagnostic validation. Ideal for a setup wizard, not daily use.
4. Knob witness marks (CAD): "align marks = closed" turns assume_closed into
   a visual operation; doubles as the recovery flow.
5. Ship-closed convention: worm self-locks unpowered, so factory-closed
   grippers ARRIVE closed -> first-boot setup is just assume_closed + state
   file. Suggested product OOBE: mount -> pip install -> one setup command ->
   never again ("auto" verifies every connect).

## Distribution: no fork needed — ship a pip package

LeRobot has an official third-party plugin system
(`utils/import_utils.py:register_third_party_plugins`, documented in
`docs/source/integrate_hardware.mdx`). Every relevant CLI (`lerobot-teleoperate`,
`lerobot-calibrate`, `lerobot-record`, `lerobot-replay`, `lerobot-train`,
`lerobot-eval`, async inference) scans installed distributions whose name starts
with `lerobot_robot_` (also `lerobot_teleoperator_`, `lerobot_camera_`, …) and
imports them, letting them self-register via `@RobotConfig.register_subclass`.

So the product is a PyPI package, e.g. `lerobot_robot_fastgripper`:

- Config: `@RobotConfig.register_subclass("fastgripper_follower")` dataclass
  (subclassing `SOFollowerConfig` + `RobotConfig`).
- Robot class: subclass of `SOFollower` overriding `calibrate()`/`connect()`
  for the gripper (homing, multi-turn range handling).
- Bus: a small `FeetechMotorsBus` subclass that keeps Phase bit 4 set on the
  gripper motor and bypasses the position-limit-register round-trip for it —
  the bus is instantiated by our robot class, so no upstream edits.

User experience:

```sh
pip install lerobot lerobot_robot_fastgripper
lerobot-teleoperate --robot.type=fastgripper_follower --robot.port=... \
  --teleop.type=so101_leader --teleop.port=...
```

Everything else (datasets, training, policies) works untouched since the
gripper still presents as 0–100. Risk to track: the plugin API + Feetech driver
internals we subclass aren't a stability-guaranteed API; pin a lerobot version
range in the package metadata.

Next experiments (need powered hardware): set Phase bit 4 on a bench STS3215,
spin >1 turn, confirm accumulating reads; test Goal_Position across turns;
characterize stall-homing current threshold through the worm.
