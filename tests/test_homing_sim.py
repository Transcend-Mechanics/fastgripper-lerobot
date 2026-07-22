"""Simulated-hardware tests: drive the REAL home_gripper() state machine
against a software model of the STS3215 (position dynamics, load response,
hard stop). No serial hardware involved."""

import json
import time
from unittest.mock import patch

import pytest
from lerobot.motors import MotorCalibration

from lerobot_robot_fastgripper import (
    FastGripperFollower,
    FastGripperFollowerConfig,
)


class ServoSim:
    """Minimal STS3215 model: chases Goal_Position at a fixed velocity,
    clamps at a hard stop, load rises to the torque limit when pushing."""

    CRUISE_LOAD = 180  # matches bench rig: ~11-21% duty while moving

    def __init__(self, pos=3000, stop_min=None, stop_max=None, velocity=3000):
        self.pos = pos
        self.goal = pos
        self.stop_min = stop_min
        self.stop_max = stop_max
        self.velocity = velocity  # ticks per simulated second
        self.torque_on = False
        self.torque_limit = 600
        self.registers = {}
        self.last_t = time.monotonic()
        self.reseed_count = 0

    def _advance(self):
        now = time.monotonic()
        dt = now - self.last_t
        self.last_t = now
        if not self.torque_on:
            return
        delta = self.goal - self.pos
        step = min(abs(delta), self.velocity * dt)
        self.pos += step if delta > 0 else -step
        if self.stop_min is not None and self.pos < self.stop_min:
            self.pos = self.stop_min
        if self.stop_max is not None and self.pos > self.stop_max:
            self.pos = self.stop_max

    @property
    def stalled(self):
        at_stop = (self.stop_min is not None and self.pos <= self.stop_min and self.goal < self.pos) or (
            self.stop_max is not None and self.pos >= self.stop_max and self.goal > self.pos
        )
        return self.torque_on and at_stop

    # --- bus-facing API ---
    def read(self, reg, motor, normalize=False):
        self._advance()
        if reg == "Present_Position":
            return int(self.pos)
        if reg == "Present_Load":
            return self.torque_limit if self.stalled else (self.CRUISE_LOAD if self.torque_on else 0)
        return self.registers.get(reg, 0)

    def write(self, reg, motor, value, normalize=False):
        self._advance()
        if reg == "Goal_Position":
            self.goal = value
        elif reg == "Max_Torque_Limit":
            self.torque_limit = value
        self.registers[reg] = value

    def enable_torque(self, motor=None):
        self._advance()
        self.torque_on = True

    def disable_torque(self, motor=None):
        self._advance()
        self.torque_on = False

    def reseed(self, motor):
        self._advance()
        shift = 2048 - self.pos
        self.pos = 2048
        self.goal += shift
        if self.stop_min is not None:
            self.stop_min += shift
        if self.stop_max is not None:
            self.stop_max += shift
        self.reseed_count += 1
        return 2048


@pytest.fixture
def rig(tmp_path):
    """Robot wired to a ServoSim instead of serial hardware."""

    def build(sim, **cfg_kwargs):
        cfg = FastGripperFollowerConfig(
            port="/dev/null", id="sim", calibration_dir=tmp_path, **cfg_kwargs
        )
        robot = FastGripperFollower(cfg)
        patches = [
            patch.object(robot.bus, "read", side_effect=sim.read),
            patch.object(robot.bus, "write", side_effect=sim.write),
            patch.object(robot.bus, "enable_torque", side_effect=sim.enable_torque),
            patch.object(robot.bus, "disable_torque", side_effect=sim.disable_torque),
            patch.object(robot.bus, "reseed_position", side_effect=sim.reseed),
            patch.object(
                robot.bus, "clear_overload", side_effect=lambda m: (sim.disable_torque(), True)[1]
            ),
            patch.object(robot.bus, "configure_multiturn_motor"),
            patch.object(robot.bus, "write_calibration"),
            patch.object(robot, "_save_calibration"),
        ]
        for p in patches:
            p.start()
        return robot

    yield build
    patch.stopall()


def test_homing_finds_stop_and_seeds_calibration(rig):
    sim = ServoSim(pos=3000, stop_min=1400)
    robot = rig(sim)  # close direction -1 (default)

    robot.home_gripper()

    cal = robot.calibration["gripper"]
    assert cal.range_min == robot.HOME_SEED == 2048
    assert cal.range_max == 2048 + robot.config.gripper_stroke_ticks
    assert cal.homing_offset == 0
    # Back-off left the jaw off the stop and torque was released at the end.
    assert sim.goal > sim.stop_min
    assert not sim.torque_on


def test_homing_positive_close_direction(rig):
    sim = ServoSim(pos=3000, stop_max=5200)
    robot = rig(sim, gripper_close_direction=+1)

    robot.home_gripper()

    assert robot.calibration["gripper"].range_min == 2048
    assert sim.goal < sim.stop_max  # backed off below the +stop


def test_homing_reseeds_when_goal_window_exhausts(rig):
    # Power-on-like state: counter near zero, stop several turns away in the
    # negative direction. Without mid-cruise re-seed the 0-clamp would stop it.
    sim = ServoSim(pos=700, stop_min=-6000, velocity=20000)
    robot = rig(sim)

    robot.home_gripper()

    assert sim.reseed_count >= 2  # at least start-area reseed + final seed
    assert robot.calibration["gripper"].range_min == 2048


def test_homing_times_out_without_stop(rig):
    sim = ServoSim(pos=3000, stop_min=None, stop_max=None, velocity=20000)
    robot = rig(sim)
    with patch.object(FastGripperFollower, "HOMING_TIMEOUT_S", 2.0):
        with pytest.raises(RuntimeError, match="no hard stop"):
            robot.home_gripper()
    assert not sim.torque_on  # left safe


def test_assume_closed_homes_without_motion(rig):
    """Manual homing: no cruise, no stall — just re-seed + calibration."""
    sim = ServoSim(pos=3777)  # wherever the human left it (at the stop, they say)
    robot = rig(sim, gripper_home_mode="assume_closed")

    robot.assume_gripper_closed()

    assert sim.reseed_count == 1
    assert sim.pos == 2048          # counter re-seeded, shaft never moved
    assert not sim.torque_on
    cal = robot.calibration["gripper"]
    assert cal.range_min == 2048
    assert cal.range_max == 2048 + robot.config.gripper_stroke_ticks


def test_park_then_auto_restore_round_trip(rig, tmp_path):
    """Park drives to closed + saves state; 'auto' verifies and restores."""
    sim = ServoSim(pos=9000)
    robot = rig(sim)
    robot.assume_gripper_closed()  # establishes calibration + state at 9000->2048

    # Simulate a session that moved the gripper, then parked on disconnect.
    sim.torque_on = True
    sim.pos = sim.goal = 12000
    robot.park_gripper()
    assert abs(sim.pos - 2048) < 100        # ended re-seeded at closed
    assert not sim.torque_on
    state = json.loads(robot._gripper_state_fpath.read_text())
    assert abs(state["parked_raw"] - 2048) < 60

    # Next session: boot reading matches parked -> zero-motion restore.
    robot.restore_gripper_from_parked()
    assert robot.calibration["gripper"].range_min == 2048


def test_auto_rejects_moved_gripper(rig, tmp_path):
    sim = ServoSim(pos=5000)
    robot = rig(sim)
    robot._gripper_state_fpath.parent.mkdir(parents=True, exist_ok=True)
    robot._gripper_state_fpath.write_text(json.dumps({"parked_raw": 2048}))
    with pytest.raises(RuntimeError, match="moved by hand"):
        robot.restore_gripper_from_parked()


def test_auto_tolerates_torque_off_settle_drift(rig, tmp_path):
    """Cutting torque at disconnect lets the reading settle up to ~200 ticks
    (seen live: boot 1846 vs parked 2048). That must NOT read as 'moved'."""
    sim = ServoSim(pos=1846)
    robot = rig(sim)
    robot._gripper_state_fpath.parent.mkdir(parents=True, exist_ok=True)
    robot._gripper_state_fpath.write_text(json.dumps({"parked_raw": 2048}))
    robot.restore_gripper_from_parked()
    assert robot.calibration["gripper"].range_min == 2048


def test_disconnect_after_failed_restore_skips_park(rig, tmp_path):
    """Live failure 2026-07-21: a failed 'auto' restore was followed by a
    blind park that moved the gripper ~1000 ticks and overwrote the state
    file with a false claim. Without an established zero, disconnect must
    neither move the motor nor touch the state file."""
    sim = ServoSim(pos=5000)
    robot = rig(sim)
    robot._gripper_state_fpath.parent.mkdir(parents=True, exist_ok=True)
    robot._gripper_state_fpath.write_text(json.dumps({"parked_raw": 2048}))
    robot.calibration["gripper"] = MotorCalibration(
        id=6, drive_mode=0, homing_offset=0, range_min=2048, range_max=21248
    )
    with pytest.raises(RuntimeError, match="moved by hand"):
        robot.restore_gripper_from_parked()

    with patch("lerobot_robot_fastgripper.fastgripper_follower.SOFollower.disconnect"):
        robot.disconnect()

    assert sim.pos == 5000  # no blind motion
    assert json.loads(robot._gripper_state_fpath.read_text()) == {"parked_raw": 2048}


def test_disconnect_park_failure_clears_state(rig, tmp_path):
    """If parking dies mid-move (e.g. USB drop), the state file no longer
    describes reality — it must be removed, not left as a stale claim."""
    sim = ServoSim(pos=9000)
    robot = rig(sim)
    robot.assume_gripper_closed()
    assert robot._gripper_state_fpath.exists()

    patch.object(robot.bus, "read", side_effect=ConnectionError("no status packet")).start()
    with patch("lerobot_robot_fastgripper.fastgripper_follower.SOFollower.disconnect"):
        robot.disconnect()

    assert not robot._gripper_state_fpath.exists()


def test_auto_without_state_gives_setup_guidance(rig, tmp_path):
    sim = ServoSim(pos=2048)
    robot = rig(sim)
    with pytest.raises(RuntimeError, match="assume_closed"):
        robot.restore_gripper_from_parked()


def test_normalization_maps_0_100_to_homed_range(rig):
    """After homing, the 0-100 gripper action must span [2048, 2048+stroke]."""
    sim = ServoSim(pos=3000, stop_min=1400)
    robot = rig(sim)
    robot.home_gripper()

    bus = robot.bus
    bus.calibration = dict(robot.calibration)
    # fill arm motors with dummy single-turn calibrations so _unnormalize works
    for name, m in bus.motors.items():
        if name not in bus.calibration:
            bus.calibration[name] = MotorCalibration(
                id=m.id, drive_mode=0, homing_offset=0, range_min=0, range_max=4095
            )

    ids = {bus.motors["gripper"].id: 0.0}
    closed = bus._unnormalize(dict(ids))[bus.motors["gripper"].id]
    ids = {bus.motors["gripper"].id: 100.0}
    opened = bus._unnormalize(dict(ids))[bus.motors["gripper"].id]
    assert closed == 2048
    assert opened == 2048 + robot.config.gripper_stroke_ticks
    # round trip
    back = bus._normalize({bus.motors["gripper"].id: opened})[bus.motors["gripper"].id]
    assert abs(back - 100.0) < 1e-6
