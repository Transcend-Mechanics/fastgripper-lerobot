# Tutorial: record a dataset

Recording teleoperated episodes is the first step toward training a policy.
This assumes you've completed the [quickstart](quickstart.md) — setup done,
both arms calibrated, teleop working.

One extra dependency: dataset tooling isn't part of the base install.

```sh
pip install "lerobot[feetech,dataset]"
```

## 1. Add a camera

Policies need to see. Plug in a webcam and find it:

```sh
lerobot-find-cameras opencv
```

Note the camera index (usually `0`).

## 2. Record

`lerobot-record` takes the same robot/teleop arguments `fastgripper teleop`
fills in for you, plus a dataset name and camera config. A minimal run of 5
episodes:

```sh
lerobot-record \
  --robot.type=fastgripper_follower \
  --robot.port=/dev/tty.usbmodemXXXX --robot.id=follower_1 \
  --robot.cameras="{front: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}}" \
  --teleop.type=so101_leader \
  --teleop.port=/dev/tty.usbmodemYYYY --teleop.id=leader_1 \
  --dataset.repo_id=YOURNAME/fastgripper-pick-place \
  --dataset.num_episodes=5 \
  --dataset.single_task="Pick up the block and place it in the bin"
```

You teleoperate each episode; keyboard controls during recording:
right arrow ends the current episode, left arrow re-records it, `Esc` stops
the session. Between episodes the arm holds position while the dataset
writes out.

Notes for this gripper specifically:

- The gripper records as `gripper.pos` (0 closed – 100 open), like any
  SO-101 dataset. Policies trained on it transfer with no special handling.
- Grip force is position-based: squeezing the leader trigger fully closed
  commands 0, and the worm gear supplies force without stalling the servo.
  Consistent trigger habits make cleaner data.
- Every session still ends with the gripper parking closed — recording
  changes nothing about the daily rhythm.

`--dataset.repo_id` doubles as a Hugging Face Hub upload target. Add
`--dataset.push_to_hub=false` to keep data local
(`~/.cache/huggingface/lerobot/`).

## 3. Replay an episode

Sanity-check the data by making the follower re-execute episode 0 without
the leader:

```sh
lerobot-replay \
  --robot.type=fastgripper_follower \
  --robot.port=/dev/tty.usbmodemXXXX --robot.id=follower_1 \
  --dataset.repo_id=YOURNAME/fastgripper-pick-place \
  --dataset.episode=0
```

If the replayed motion matches what you teleoperated — including the grip —
the dataset is good.

## 4. Train

From here it's stock LeRobot: `lerobot-train` with your dataset, then
`lerobot-record` with `--policy.path` to evaluate. Follow
[LeRobot's imitation-learning guide](https://huggingface.co/docs/lerobot/il_robots);
nothing about it is FastGripper-specific.
