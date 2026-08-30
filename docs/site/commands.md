# Command reference

The `fastgripper` CLI wraps the LeRobot tooling with your saved ports so
daily use needs no arguments. Configuration lives in
`~/.config/fastgripper/config.json`; calibration and gripper state live in
`~/.cache/huggingface/lerobot/calibration/robots/fastgripper_follower/`.

## fastgripper setup

```sh
fastgripper setup --follower-port PORT [--follower-id ID] \
                  [--leader-port PORT] [--leader-id ID]
```

One-time. Saves ports/ids to the config file, then establishes the gripper
zero: it asks whether the gripper is fully closed — answer `y`, or `g` for
a guided close (hold `a`, it stops at the hard stop, press `c` to confirm)
— then connects, records the closed position as 0, and parks. IDs default to `follower_1` /
`leader_1` — the ID names the calibration files, so keep it stable per arm.

Re-run it any time a port changes; already-saved values are kept unless you
pass a replacement, so updating one port is just
`fastgripper setup --follower-port /dev/newport`.

## fastgripper calibrate

```sh
fastgripper calibrate            # follower arm
fastgripper calibrate --leader   # leader arm
```

Runs LeRobot's standard SO-101 calibration (middle pose, then full range of
motion per joint) against the saved ports. Once per arm, redo only after
rebuilding an arm.

## fastgripper jog

```sh
fastgripper jog [--status]
```

Keyboard control of the gripper alone, with a live load readout. Used to
close the gripper before `setup` and for diagnosing mechanical issues.
`--status` prints a one-line health probe and exits.

| Key | Action |
| --- | --- |
| `a` (hold) | drive toward closed |
| `d` (hold) | drive toward open |
| `f` | toggle fast/slow speed |
| space | stop |
| `r` | re-seed the turn counter at the current position |
| `o` / `c` | mark current position as open / closed (saved to `gripper_jog_cal.json`) |
| `q` | quit |

A load guard stops motion automatically when the mechanism binds — hitting
it at the closed end is the normal way to find the hard stop.

## fastgripper teleop

```sh
fastgripper teleop [extra lerobot-teleoperate args...]
```

Runs `lerobot-teleoperate` with the saved follower + leader. Any extra
arguments pass straight through, e.g. `fastgripper teleop --fps 30`.
`Ctrl-C` stops; the gripper parks closed on exit.

## fastgripper status

```sh
fastgripper status
```

Health check, in order: prints the saved config, pings servos 1–6 on the
follower bus, dumps the gripper's key registers (position, temperature,
Phase, Operating_Mode), and reports whether arm calibration and the parked
state file exist — with the command to run if either is missing.

## fastgripper preflight

```sh
fastgripper preflight            # go/no-go, ~5 s, no motion
fastgripper preflight --trigger  # also prompts you to squeeze/release the trigger
```

Checks the saved (or `--follower-*/--leader-*`) target: ports present, all
six servos answer on each arm, servo bus voltage (12 V supply on?), stale
turn counters (a single-turn joint reading past 4095 — power-cycle the
arm's supply), calibration files, parked gripper state, and the leader's
trigger calibration. Prints one line per check and `GO` / `NO-GO`.
`fastgripper teleop` runs it automatically (`--skip-preflight` to bypass).

## fastgripper usb

```sh
fastgripper usb soak --seconds 30   # read both arms at 60 Hz concurrently, no motion
fastgripper usb watch               # log every serial-device drop/return; run DURING teleop
```

USB link diagnostics for the "teleop dies with *Device not configured*"
failure — see [USB serial drops](../troubleshooting/usb-serial-drops.md).
`soak` needs the ports free; `watch` opens nothing and can run alongside a
session. Both accept `--follower-port/--leader-port` overrides.

## Using plain LeRobot commands

Everything LeRobot ships works with the robot type `fastgripper_follower`:

```sh
lerobot-record    --robot.type=fastgripper_follower --robot.port=... --robot.id=... \
                  --teleop.type=so101_leader --teleop.port=... --teleop.id=... ...
lerobot-calibrate --robot.type=fastgripper_follower --robot.port=... --robot.id=...
lerobot-teleoperate ...   # what `fastgripper teleop` runs for you
```

The gripper appears as one more joint, `gripper.pos`, normalized 0 (closed)
to 100 (open) — datasets and policies never see the multi-turn mechanics.

## Files on disk

| Path | What | Written by |
| --- | --- | --- |
| `~/.config/fastgripper/config.json` | ports and ids | `setup` |
| `~/.cache/huggingface/lerobot/calibration/robots/fastgripper_follower/<id>.json` | arm + gripper calibration | `calibrate`, `setup` |
| `.../<id>_gripper_state.json` | last parked position | every disconnect |

All plain JSON; delete the state file to force a fresh `setup`, delete the
calibration file to force recalibration.
