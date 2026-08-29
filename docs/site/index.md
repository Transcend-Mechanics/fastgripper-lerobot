# FastGripper documentation

FastGripper is a worm-gear-driven parallel gripper for the SO-101 arm. The
worm gives it two properties stock grippers don't have: high grip force from
a standard STS3215 servo, and zero-power holding — it cannot be back-driven,
so whatever it's gripping stays gripped when power drops.

The software side is a LeRobot plugin. Install one pip package and the
gripper behaves like any other LeRobot robot: teleop, dataset recording, and
trained policies all see a standard gripper that goes from 0 (closed) to 100
(open). The multi-turn drive, turn counting, and power-cycle bookkeeping are
handled underneath.

```sh
pip install "lerobot[feetech]" lerobot_robot_fastgripper
```

## Where to go

| Page | For |
| --- | --- |
| [Quickstart](quickstart.md) | First-time setup, box → teleop in ~30 min. No SO-101 experience assumed. |
| [Command reference](commands.md) | Every `fastgripper` command, flags, and the files they touch. |
| [Tutorial: record a dataset](tutorial-record-dataset.md) | Teleop → recorded episodes → replay, the path to training policies. |
| [Tutorial: Python control](tutorial-python-api.md) | Drive the gripper from your own code, no teleop. |
| [How it works](how-it-works.md) | The multi-turn servo problem and how the plugin solves it. For the robotics-minded. |
| [Troubleshooting](troubleshooting.md) | When something's off. Start with `fastgripper preflight`. |
| [USB serial drops](../troubleshooting/usb-serial-drops.md) | Teleop dies with "Device not configured" / "no status packet". Hub, cable, or USB power — not the gripper. |

## Source and support

- Code, issues, PRs: [github.com/Transcend-Mechanics/fastgripper-lerobot](https://github.com/Transcend-Mechanics/fastgripper-lerobot)
- Package: [pypi.org/project/lerobot_robot_fastgripper](https://pypi.org/project/lerobot_robot_fastgripper/)
- License: Apache-2.0
