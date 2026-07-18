# Troubleshooting

First stop, always:

```sh
fastgripper status
```

It prints the saved config, pings every servo on the follower bus, dumps
the gripper's key registers, and tells you which of calibration / parked
state is missing and what command fixes it.

## Connection problems

**Port not found / nothing responds.** In order of likelihood:

1. Charge-only USB cable — swap for one known to carry data.
2. Arm power is off — USB powers the driver board only; servos need the
   power supply. Servo LEDs blink briefly at power-on.
3. Port name changed — this happens across reboots and hub changes,
   especially on macOS. Re-run `lerobot-find-port`, then update the saved
   config: `fastgripper setup --follower-port /dev/newport` (other saved
   values are kept).
4. Linux permission denied — `sudo usermod -aG dialout $USER`, log out
   and back in.

**Some servos respond, others don't.** The bus is a daisy-chain: a servo
with a loose or damaged cable takes everything downstream of it off the
bus. `fastgripper status` shows exactly which IDs answer — the first
missing ID points at the cable to check. A shorted data line in any link
can also hold the whole bus down (symptom: driver-board RX LED on solid);
unplug the chain from the board and re-add segments until it reappears.

## Gripper problems

**"…likely moved by hand while off" on connect.** The plugin verified the
boot position against where it parked and they disagree — someone turned
the drive while unpowered, or the servo was swapped. Recovery is two
commands: `fastgripper jog` (hold `a` until the load guard stops it at
fully closed, `q`), then `fastgripper setup` to re-establish zero.

**Gripper won't move by hand.** Correct — the worm gear self-locks and
cannot be back-driven. Never force it; use `fastgripper jog`.

**Follower gripper stays slightly open at full trigger squeeze.** Stale
leader calibration — trigger range drifts with wear and re-seating. Re-run
`fastgripper calibrate --leader` and squeeze the trigger through its full
range, firmly, at both ends.

**Motion stops mid-travel during jog.** The load guard tripped. That's it
working — it fires on mechanical binding or on reaching the closed hard
stop. If it trips repeatedly mid-stroke with nothing in the jaws, inspect
the mechanism for debris or misalignment before turning anything up.

**Servo protection latch.** Sustained overload trips the STS3215's
internal protection, which then poisons every read on the bus until
cleared. The plugin clears and recovers from this automatically during
normal operation; if you hit persistent overload errors in your own
scripts, power-cycle the arm, then reconnect — and check what the gripper
was being asked to push through.

## Calibration problems

**LeRobot asks to recalibrate / values look shifted.** Calibration files
are keyed by the `id` you pass; a different `--robot.id` means a different
(likely missing) file. Keep one stable id per physical arm. Files live in
`~/.cache/huggingface/lerobot/calibration/robots/fastgripper_follower/`.

**Start over completely.** Delete that directory plus
`~/.config/fastgripper/config.json`, close the gripper with
`fastgripper jog`, and walk the [quickstart](quickstart.md) again from
`setup`. Total time: about 10 minutes.

## Still stuck

Open an issue with the output of `fastgripper status`:
[github.com/Transcend-Mechanics/fastgripper-lerobot/issues](https://github.com/Transcend-Mechanics/fastgripper-lerobot/issues)
