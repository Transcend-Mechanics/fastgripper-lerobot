# Tutorial: control the gripper from Python

No teleop, no CLI — just the LeRobot robot API. Useful for scripted cells,
grasp testing, and integrating the gripper into your own stack.

## Connect

```python
from lerobot.utils.import_utils import register_third_party_plugins
from lerobot_robot_fastgripper import FastGripperFollowerConfig, FastGripperFollower

register_third_party_plugins()

cfg = FastGripperFollowerConfig(
    port="/dev/tty.usbmodemXXXX",   # from `lerobot-find-port`
    id="follower_1",                # must match the id you calibrated with
)
robot = FastGripperFollower(cfg)
robot.connect()
```

`connect()` runs the same verified restore as every other entry point: it
checks the gripper is where it was parked and refuses (with instructions)
if the mechanism was moved while off. If you've completed
`fastgripper setup` once, this just works.

## Read and command

The robot exposes the standard LeRobot interface. All positions are floats,
normalized per joint; the gripper is `gripper.pos`, 0 (closed) to 100
(open):

```python
obs = robot.get_observation()
print(obs["gripper.pos"])            # e.g. 3.1 — parked closed

robot.send_action({"gripper.pos": 80.0})   # open
robot.send_action({"gripper.pos": 0.0})    # close / grip
```

`send_action` is a goal command, not a blocking move — it returns
immediately and the servo travels at its configured speed. Poll
`get_observation()` if you need to know when it arrives:

```python
import time

def move_gripper_and_wait(robot, target, tol=2.0, timeout=6.0):
    robot.send_action({"gripper.pos": target})
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if abs(robot.get_observation()["gripper.pos"] - target) < tol:
            return True
        time.sleep(0.05)
    return False   # blocked — probably gripping something
```

Timing out short of the target is not an error when closing on an object:
the commanded position sets the squeeze, the worm supplies the force, and
the servo simply holds there. Commanding 0 on a 30 mm object is the normal
way to grip it.

## Grip, hold, release

```python
robot.send_action({"gripper.pos": 100.0})  # open wide
# ... position the object ...
robot.send_action({"gripper.pos": 0.0})    # close onto it
time.sleep(2.0)
# The grip survives power loss from here on — the worm gear self-locks.
robot.send_action({"gripper.pos": 100.0})  # release
```

## Disconnect

```python
robot.disconnect()
```

Always disconnect. This is what parks the gripper closed and records the
parked position — the thing that makes the *next* `connect()` instant and
safe. If your script can crash mid-grip, wrap the work in `try/finally`
with `disconnect()` in the `finally`.

The other five joints work identically (`shoulder_pan.pos`,
`shoulder_lift.pos`, `elbow_flex.pos`, `wrist_flex.pos`, `wrist_roll.pos`)
— see `robot.action_features` for the full dict.
