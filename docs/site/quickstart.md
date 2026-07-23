# FastGripper quickstart — from sealed box to teleoperation

This guide assumes your gripper is already **mechanically assembled** on an
SO-101 follower arm and that you have **never operated an SO-101 before**.
Everything below is software and cables. Budget ~30 minutes.

## What you need

- SO-101 **follower arm** with the FastGripper installed (the gripper servo
  replaces the stock gripper servo and keeps its bus ID, **6**).
- SO-101 **leader arm** (the smaller one with the trigger) if you want to
  teleoperate. Recording/policies need it too; a follower alone can still be
  jogged and scripted.
- The two USB serial driver boards that came with the arms, and USB cables
  that carry **data** (some charge-only cables look identical — if a port
  never shows up, suspect the cable first).
- The power supplies from your kit (typically 12 V for the follower, 5–7.4 V
  for the leader — use the ones your kit shipped with, don't swap them).
- A computer with **Python 3.12+**. macOS and Linux are tested; the `jog`
  command uses a POSIX terminal and won't run on native Windows (WSL works).

**Do not open the gripper by hand while it's unpowered.** It ships fully
closed, and the software uses that fact to establish its zero. (The worm
gear can't be back-driven anyway — if it feels stuck, that's by design.)

## 1. Install the software

```sh
python3 -m venv fastgripper-env
source fastgripper-env/bin/activate
pip install "lerobot[feetech]" lerobot_robot_fastgripper
```

That's the entire install: LeRobot auto-discovers the plugin. Verify with
`fastgripper --help`.

> Linux only: if serial ports later fail with "permission denied", add
> yourself to the serial group and re-login:
> `sudo usermod -aG dialout $USER`

## 2. Plug in and find your ports

Connect each arm's driver board to USB and power on both arms (servo LEDs
flash briefly — normal). Then find which port is which:

```sh
lerobot-find-port
```

It asks you to unplug one cable so it can tell you that cable's port (e.g.
`/dev/tty.usbmodem5C4C1267251` on macOS, `/dev/ttyACM0` on Linux). Run it
twice — once per arm — and write both ports down.

## 3. One-time setup (establishes the gripper zero)

With the gripper still in its shipped, fully-closed position:

```sh
fastgripper setup \
  --follower-port /dev/tty.usbmodemXXXX --follower-id follower_1 \
  --leader-port   /dev/tty.usbmodemYYYY --leader-id   leader_1
```

It saves the ports to `~/.config/fastgripper/config.json` (so you never type
them again), asks **"Is the gripper fully closed?"** — answer `y` — and
briefly connects to record the gripper's zero and parked state.

Why this matters: the gripper's stroke spans ~4.8 servo turns, but the
servo's encoder only knows its position *within* one turn. The closed
position is the reference that makes the other ~4.8 turns meaningful, and
because the worm gear holds position with power off, that reference stays
trustworthy across every power cycle from now on.

If the gripper is *not* closed (or you're not sure), answer `g` instead —
setup walks you through closing it: hold `a` to drive toward closed, it
stops itself at the hard stop, press `c` to confirm, and setup continues
from there.

## 4. Calibrate the arms (once)

```sh
fastgripper calibrate            # follower
fastgripper calibrate --leader   # leader
```

This is LeRobot's standard SO-101 calibration and it walks you through it
on-screen. For each arm you will: move the arm to the middle of its range
and press Enter, then slowly move **every joint through its full range of
motion** and press Enter again. Take the joints to their actual limits —
this is how the software learns what "0–100%" means for each joint. On the
leader, also squeeze the trigger fully open and fully closed.

Calibration files land in `~/.cache/huggingface/lerobot/calibration/` and
persist forever; you only redo this if you rebuild an arm.

## 5. Teleoperate

```sh
fastgripper teleop
```

Move the leader; the follower mirrors it. Squeeze the leader's trigger to
close the gripper. `Ctrl-C` to stop — on exit the follower **parks the
gripper closed** and remembers that, so the next `fastgripper teleop` starts
instantly with a verified zero-motion restore. That park-on-exit /
verify-on-start cycle is the daily rhythm; there is no per-session homing.

## Daily use, in full

```sh
source fastgripper-env/bin/activate
fastgripper teleop
```

That's it.

## Beyond teleop

The robot type `fastgripper_follower` works in every LeRobot command, e.g.:

```sh
lerobot-record --robot.type=fastgripper_follower \
  --robot.port=/dev/tty.usbmodemXXXX --robot.id=follower_1 \
  --teleop.type=so101_leader --teleop.port=/dev/tty.usbmodemYYYY \
  --teleop.id=leader_1 ...
```

(`fastgripper teleop` also passes any extra `lerobot-teleoperate` arguments
straight through.)

Next steps: [record a dataset](tutorial-record-dataset.md) toward training a
policy, or [drive the gripper from Python](tutorial-python-api.md).

## If something's off

- **First stop:** `fastgripper status` — pings all six servos and reports
  the gripper's registers, calibration, and parked state, with a hint for
  whatever is missing.
- **"…likely moved by hand while off"** error on connect: the boot
  position didn't match where it was parked (someone turned the input shaft
  by hand, or a servo was swapped). Close the gripper with
  `fastgripper jog`, then re-run `fastgripper setup` to re-establish zero.
- **A port stopped appearing:** cable (data vs charge-only), then USB hub,
  then re-run `lerobot-find-port` — port names can change between reboots,
  especially on macOS; re-run `fastgripper setup` with just the changed
  `--follower-port`/`--leader-port` to update the saved config.
- **Arm is limp / won't hold position:** check the arm's power supply —
  USB powers only the driver board, not the servos.
- More failure modes and fixes: [troubleshooting](troubleshooting.md).
  The full servo-level story: [how it works](how-it-works.md).
