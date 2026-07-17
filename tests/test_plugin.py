"""Hardware-free tests: plugin discovery, config registration, factory
resolution, and the multi-turn calibration register bypass."""

from unittest.mock import patch

import pytest
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.robots.config import RobotConfig
from lerobot.robots.utils import make_robot_from_config
from lerobot.utils.import_utils import register_third_party_plugins

from lerobot_robot_fastgripper import (
    MultiTurnFeetechMotorsBus,
    FastGripperFollower,
    FastGripperFollowerConfig,
)


def test_plugin_is_discoverable():
    """The lerobot CLIs find this package by its lerobot_robot_ dist name."""
    register_third_party_plugins()
    assert RobotConfig.get_choice_class("fastgripper_follower") is FastGripperFollowerConfig


def test_factory_resolves_robot_class(tmp_path):
    cfg = FastGripperFollowerConfig(port="/dev/null", id="test", calibration_dir=tmp_path)
    robot = make_robot_from_config(cfg)
    assert isinstance(robot, FastGripperFollower)
    assert isinstance(robot.bus, MultiTurnFeetechMotorsBus)
    assert robot.bus.multiturn_motors == {"gripper"}
    # Same motor table and gripper norm mode as the stock SO-101
    assert list(robot.bus.motors) == [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_roll",
        "gripper",
    ]
    assert robot.bus.motors["gripper"].norm_mode is MotorNormMode.RANGE_0_100


def _make_bus():
    return MultiTurnFeetechMotorsBus(
        port="/dev/null",
        motors={
            "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
            "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        },
        multiturn_motors={"gripper"},
    )


def test_unknown_multiturn_motor_rejected():
    with pytest.raises(ValueError, match="not present"):
        MultiTurnFeetechMotorsBus(
            port="/dev/null",
            motors={"gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100)},
            multiturn_motors={"typo"},
        )


def test_write_calibration_uses_wide_window_for_multiturn():
    from lerobot_robot_fastgripper.multiturn_feetech_bus import MULTITURN_MAX_LIMIT

    bus = _make_bus()
    calibration = {
        "wrist_roll": MotorCalibration(id=5, drive_mode=0, homing_offset=100, range_min=0, range_max=4095),
        "gripper": MotorCalibration(id=6, drive_mode=0, homing_offset=0, range_min=0, range_max=22800),
    }

    with patch.object(MultiTurnFeetechMotorsBus, "write") as mock_write:
        bus.write_calibration(calibration)

    # Gripper registers get the validated multi-turn convention: offset 0 and
    # a wide positive limit window (NOT 0/0, which the firmware treats as
    # "clamp goals to zero")...
    gripper_writes = {c.args[0]: c.args[2] for c in mock_write.call_args_list if c.args[1] == "gripper"}
    assert gripper_writes == {
        "Homing_Offset": 0,
        "Min_Position_Limit": 0,
        "Max_Position_Limit": MULTITURN_MAX_LIMIT,
    }
    # ...while the wrist_roll round-trip is unchanged...
    roll_writes = {c.args[0]: c.args[2] for c in mock_write.call_args_list if c.args[1] == "wrist_roll"}
    assert roll_writes == {"Homing_Offset": 100, "Min_Position_Limit": 0, "Max_Position_Limit": 4095}
    # ...and the true multi-turn range survives in the software cache.
    assert bus.calibration["gripper"].range_max == 22800


def test_write_calibration_rejects_negative_multiturn_range():
    bus = _make_bus()
    calibration = {
        "gripper": MotorCalibration(id=6, drive_mode=0, homing_offset=0, range_min=-100, range_max=22800),
    }
    with patch.object(MultiTurnFeetechMotorsBus, "write"), pytest.raises(ValueError, match="positive domain"):
        bus.write_calibration(calibration)


def test_is_calibrated_ignores_multiturn_registers():
    bus = _make_bus()
    bus.calibration = {
        "wrist_roll": MotorCalibration(id=5, drive_mode=0, homing_offset=100, range_min=0, range_max=4095),
        "gripper": MotorCalibration(id=6, drive_mode=0, homing_offset=0, range_min=0, range_max=22800),
    }
    # What the motor registers would report after write_calibration: gripper
    # limits clamped/zeroed, wrist_roll faithful.
    register_state = {
        "wrist_roll": MotorCalibration(id=5, drive_mode=0, homing_offset=100, range_min=0, range_max=4095),
        "gripper": MotorCalibration(id=6, drive_mode=0, homing_offset=0, range_min=0, range_max=28672),
    }
    with patch.object(MultiTurnFeetechMotorsBus, "read_calibration", return_value=register_state):
        assert bus.is_calibrated

    # A genuine mismatch on a single-turn motor is still caught.
    register_state["wrist_roll"] = MotorCalibration(
        id=5, drive_mode=0, homing_offset=0, range_min=0, range_max=4095
    )
    with patch.object(MultiTurnFeetechMotorsBus, "read_calibration", return_value=register_state):
        assert not bus.is_calibrated
