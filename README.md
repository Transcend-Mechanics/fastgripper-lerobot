# fastgripper-lerobot

**FastGripper** — a worm-gear-driven, compliant parallel gripper for the
SO-101 arm, shipped as a turnkey [LeRobot](https://github.com/huggingface/lerobot)
plugin. High grip force, zero-power holding (the worm self-locks), and a
multi-turn drive that the plugin makes invisible: teleop, datasets, and
policies see a standard 0–100 gripper.

## Install

```sh
pip install lerobot lerobot_robot_fastgripper
```

LeRobot auto-discovers any installed `lerobot_robot_*` package — no fork, no
config. The robot type is `fastgripper_follower`.

## Quick start

New to the SO-101 entirely? Follow the step-by-step
**[quickstart guide](docs/site/quickstart.md)** (box → teleop in ~30 min).
Full docs — tutorials, command reference, internals — live in
**[docs/site/](docs/site/index.md)**. The short version:

```sh
# one-time (gripper ships closed; the worm holds it there in transit)
fastgripper setup --follower-port /dev/tty.usbmodemXXXX --follower-id follower_1 \
                  --leader-port /dev/tty.usbmodemYYYY --leader-id leader_1
fastgripper calibrate            # standard SO-101 arm calibration (once)

# daily
fastgripper teleop
```

Every session ends with the gripper **parking itself closed** and starts with
a **verified zero-motion restore** — the worm cannot backdrive while
unpowered, so the parked position is trustworthy across power cycles. If the
mechanism is ever moved by hand while off, the next start detects the
mismatch and tells you exactly what to do.

## Commands

| Command | What it does |
| --- | --- |
| `fastgripper setup` | save ports/ids, establish the gripper zero (one-time) |
| `fastgripper calibrate [--leader]` | standard LeRobot arm calibration |
| `fastgripper jog` | keyboard jog + live load/torque survey |
| `fastgripper teleop` | teleoperate with the saved setup (extra lerobot args pass through) |
| `fastgripper status` | servo, calibration, and parked-state health check |

Plain LeRobot commands work too (`lerobot-record`, `lerobot-train`, …) with
`--robot.type=fastgripper_follower`.

## Why a plugin is needed at all

The gripper's stroke spans ~4.8 servo revolutions, but the STS3215's encoder
is absolute only within one revolution. This package runs the servo in its
(undocumented) absolute multi-turn mode and manages the volatile turn counter
across power cycles — plus tuned motion/protection profiles for driving a
worm stage. Design details and all bench findings:
[docs/multiturn-gripper-notes.md](docs/multiturn-gripper-notes.md).

## Development

```sh
uv venv --python 3.12 .venv
uv pip install -p .venv/bin/python -e ".[dev]"
.venv/bin/pytest tests            # simulated-hardware suite, no robot needed
```

License: Apache-2.0.
