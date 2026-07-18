# How it works

The interesting engineering problem: driving a ~4.8-revolution gripper with
a servo whose encoder only knows where it is within one revolution — and
doing it so users never notice.

## The mechanism

A worm on the servo shaft drives a worm wheel coupled to a compliant 4-bar
linkage that keeps the fingers parallel. Two consequences:

- **Force multiplication.** The worm stage's reduction turns a hobby
  servo's torque into serious grip force, applied slowly and controllably.
- **Self-locking.** Friction geometry means the wheel cannot drive the
  worm. Grip force costs zero holding current, and an unpowered gripper is
  a rigid gripper. The software leans on this hard (see below).

The full stroke takes about 4.8 servo revolutions — 19,200 encoder ticks at
4096 ticks/rev.

## The servo problem

The STS3215's magnetic encoder is absolute within one turn: it reads
0–4095 and wraps. Feetech's firmware has a multi-turn mode that stacks a
turn counter on top — but the counter lives in RAM. Power-cycle the servo
and it forgets which of the ~5 turns it was on. Position 2048 could be
fully closed, fully open, or anywhere between.

Stock LeRobot has no answer for this because stock SO-101 joints never
leave one revolution. The plugin's job is (1) putting the servo into a
multi-turn mode that actually behaves, and (2) making the turn counter
survive power cycles *logically*, since it can't physically.

## Multi-turn mode, the part the datasheet doesn't tell you

Bench-validated configuration, applied automatically on every connect:

- **Operating mode 3** — absolute multi-turn positioning. Goal positions
  are sign-magnitude encoded and span multiple revolutions.
- **Phase register bit 4 set** — makes position feedback accumulate across
  turns instead of wrapping. LeRobot's standard configure pass clears this
  bit; the plugin re-asserts it.
- **Widened position limits** — goals beyond `Max_Position_Limit` are
  silently rejected in mode 3, and zeroed limits mean "clamp everything to
  zero," not "no limits." The plugin sets an explicit 0–28672 window.
- **Nonzero goal velocity** — mode 3 with velocity 0 doesn't stop; it
  creeps at ~75 ticks/s.
- **Tuned gains and protection** — a stiffer position P-gain (the stock
  value crawls over long multi-turn moves) and torque/overload settings
  matched to worm-stage load characteristics, where load spikes at speed
  are normal and the classic stall signature never appears (the worm's
  mechanical advantage hides finger contact from the motor).

Details, register values, and the experiments behind them:
[multiturn-gripper-notes.md](https://github.com/Transcend-Mechanics/fastgripper-lerobot/blob/main/docs/multiturn-gripper-notes.md).

## Surviving power cycles: park and restore

The volatile turn counter meets the self-locking worm:

1. **On disconnect, the gripper parks closed** and the plugin saves the
   servo's raw position to a state file.
2. **While off, the mechanism cannot move** — the worm can't be
   back-driven. The saved position stays true no matter how the arm is
   handled or transported.
3. **On connect, the plugin verifies** the servo reads within a small
   tolerance of the parked value, re-seeds the turn counter, and restores
   full multi-turn calibration with **zero motion**. No homing sweep, no
   limit-switch hunt, no per-session ritual.

If verification fails — meaning someone turned the drive by hand while
powered off, which takes deliberate effort — the plugin refuses to move and
prints the two commands that re-establish zero. Failing loudly beats
guessing with a gripper that can push hard.

This is also why initial setup requires the gripper closed: closed is the
one position that ships from the factory, survives transit (self-locking
again), and is mechanically unambiguous.

## What the rest of the stack sees

Above the plugin, none of this exists. The gripper is one joint,
`gripper.pos`, normalized 0–100 like every LeRobot gripper. Teleop maps the
leader trigger onto it, datasets record it, policies command it. A model
trained on a stock SO-101 dataset schema runs unmodified — the multi-turn
bookkeeping, parking, and restore live entirely below the robot API.
