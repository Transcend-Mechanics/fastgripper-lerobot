"""Hardware-free tests for the preflight verdict logic, the leader read retry,
and the auto->stall fallback."""

from unittest.mock import MagicMock, patch

import pytest
from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.utils.import_utils import register_third_party_plugins

from lerobot_robot_fastgripper import (
    FastGripperFollower,
    FastGripperFollowerConfig,
    FastGripperLeader,
    FastGripperLeaderConfig,
)
from lerobot_robot_fastgripper.preflight import (
    ArmReadings,
    Readings,
    evaluate,
    trigger_pct,
    verdict,
)

LEADER_CAL = {"gripper": {"id": 6, "drive_mode": 0, "homing_offset": 0, "range_min": 2058, "range_max": 3230}}


def healthy_arm(port="/dev/cu.x", gripper=2100):
    return ArmReadings(
        port=port, port_exists=True, alive=[1, 2, 3, 4, 5, 6],
        positions={1: 2000, 2: 2100, 3: 2200, 4: 800, 5: 300, 6: gripper},
        voltages={i: 12.2 for i in range(1, 7)},
    )


def healthy():
    return Readings(leader=healthy_arm(), follower=healthy_arm(gripper=2060),
                    leader_cal=LEADER_CAL, follower_cal={"gripper": {}}, parked_raw=2048)


def levels(findings):
    return {f.level for f in findings}


def test_healthy_is_go():
    f = evaluate(healthy())
    assert verdict(f)
    assert "FAIL" not in levels(f) and "WARN" not in levels(f)


def test_missing_port_is_no_go():
    r = healthy()
    r.leader.port_exists = False
    f = evaluate(r)
    assert not verdict(f)
    assert any("not present" in x.text for x in f)


def test_dead_servo_is_no_go():
    r = healthy()
    r.follower.alive = [1, 2, 3, 4, 5]
    assert not verdict(evaluate(r))


def test_usb_only_voltage_is_no_go():
    r = healthy()
    r.follower.voltages = {i: 4.9 for i in range(1, 7)}
    f = evaluate(r)
    assert not verdict(f)
    assert any("PSU" in x.text for x in f)


def test_stale_turn_counter_on_single_turn_joint_is_no_go():
    r = healthy()
    r.follower.positions[4] = 4925   # the wrist_flex case from 2026-08-27 (offset 0)
    f = evaluate(r)
    assert not verdict(f)
    assert any("power-cycle" in x.text for x in f)


def test_homing_offset_overflow_is_only_a_warning():
    r = healthy()
    r.follower.positions[5] = 5042        # the wrist_roll case from 2026-08-28
    r.follower.homing_offsets[5] = -1938  # encoder = 3104, inside one turn
    f = evaluate(r)
    assert verdict(f)
    assert any(x.level == "WARN" and "overflow" in x.text for x in f)
    assert not any("power-cycle" in x.text for x in f)


def test_decode_sign_magnitude():
    from lerobot_robot_fastgripper.preflight import decode_sign_magnitude
    assert decode_sign_magnitude(1938) == 1938
    assert decode_sign_magnitude(1938 | (1 << 11)) == -1938


def test_multiturn_gripper_above_4095_is_fine():
    r = healthy()
    r.follower.positions[6] = 9417   # mid-stroke on the worm gripper
    r.parked_raw = 9400
    assert verdict(evaluate(r))


def test_missing_park_is_warn_not_fail():
    r = healthy()
    r.parked_raw = None
    f = evaluate(r)
    assert verdict(f)
    assert any(x.level == "WARN" and "stall-home" in x.text for x in f)


def test_park_mismatch_is_warn():
    r = healthy()
    r.follower.positions[6] = 9417   # parked 2048
    f = evaluate(r)
    assert verdict(f)
    assert any("moved while off" in x.text for x in f)


def test_trigger_pct_matches_lerobot_normalisation():
    g = LEADER_CAL["gripper"]
    assert trigger_pct(2058, g) == 0.0
    assert trigger_pct(3230, g) == 100.0
    assert trigger_pct(1000, g) == 0.0            # clamped below range_min
    assert trigger_pct(2058, {**g, "drive_mode": 1}) == 100.0


def test_interactive_trigger_not_reaching_closed_is_no_go():
    r = healthy()
    r.trigger_squeezed_pct = 38.6   # the "closed is half open" case
    r.trigger_released_pct = 100.0
    f = evaluate(r)
    assert not verdict(f)
    assert any("never closes" in x.text for x in f)


def test_leader_plugin_registered_and_shares_cal_dir():
    register_third_party_plugins()
    assert TeleoperatorConfig.get_choice_class("fastgripper_leader") is FastGripperLeaderConfig
    assert FastGripperLeader.name == "so_leader"


def test_leader_get_action_retries_transient_failures(tmp_path):
    cfg = FastGripperLeaderConfig(port="/dev/null", id="t", calibration_dir=tmp_path, read_retry_delay_s=0)
    leader = FastGripperLeader(cfg)
    calls = {"n": 0}

    def flaky_sync_read(_reg):
        calls["n"] += 1
        if calls["n"] < 3:
            raise ConnectionError("There is no status packet!")
        return {"gripper": 12.0}

    leader.bus = MagicMock()
    leader.bus.sync_read.side_effect = flaky_sync_read
    leader.bus.motors = {"gripper": None}
    with patch.object(type(leader), "is_connected", new=property(lambda self: True)):
        assert leader.get_action() == {"gripper.pos": 12.0}
    assert calls["n"] == 3


def test_leader_get_action_gives_up_after_retries(tmp_path):
    cfg = FastGripperLeaderConfig(port="/dev/null", id="t", calibration_dir=tmp_path,
                                  read_retries=2, read_retry_delay_s=0)
    leader = FastGripperLeader(cfg)
    leader.bus = MagicMock()
    leader.bus.sync_read.side_effect = ConnectionError("no status packet")
    with patch.object(type(leader), "is_connected", new=property(lambda self: True)):
        with pytest.raises(ConnectionError):
            leader.get_action()
    assert leader.bus.sync_read.call_count == 2


@pytest.mark.parametrize("fallback,expect_home", [("stall", True), ("error", False)])
def test_auto_mode_falls_back_to_stall_homing(tmp_path, fallback, expect_home):
    cfg = FastGripperFollowerConfig(port="/dev/null", id="t", calibration_dir=tmp_path,
                                    gripper_home_mode="auto", gripper_auto_fallback=fallback)
    robot = FastGripperFollower(cfg)
    with patch("lerobot.robots.so_follower.SOFollower.connect"), \
         patch.object(robot, "restore_gripper_from_parked", side_effect=RuntimeError("No parked gripper state found. x")), \
         patch.object(robot, "home_gripper") as home:
        if expect_home:
            robot.connect()
            home.assert_called_once()
        else:
            with pytest.raises(RuntimeError):
                robot.connect()
            home.assert_not_called()


def test_leader_get_action_reopens_port_after_device_vanishes(tmp_path):
    cfg = FastGripperLeaderConfig(port="/dev/null", id="t", calibration_dir=tmp_path,
                                  read_retry_delay_s=0, reconnect_timeout_s=2.0)
    leader = FastGripperLeader(cfg)
    calls = {"n": 0}

    def vanish_then_ok(_reg):
        calls["n"] += 1
        if calls["n"] == 1:
            raise OSError(6, "Device not configured")
        return {"gripper": 3.0}

    leader.bus = MagicMock()
    leader.bus.sync_read.side_effect = vanish_then_ok
    leader.bus.motors = {"gripper": None}
    leader.bus.port_handler.openPort.return_value = True
    leader.bus.port_handler.setBaudRate.return_value = True
    with patch.object(type(leader), "is_connected", new=property(lambda self: True)), \
         patch("lerobot_robot_fastgripper.fastgripper_leader.time.sleep"):
        assert leader.get_action() == {"gripper.pos": 3.0}
    leader.bus.port_handler.openPort.assert_called()


def test_leader_get_action_raises_if_device_never_returns(tmp_path):
    cfg = FastGripperLeaderConfig(port="/dev/definitely-not-here", id="t", calibration_dir=tmp_path,
                                  read_retry_delay_s=0, reconnect_timeout_s=0.3)
    leader = FastGripperLeader(cfg)
    leader.bus = MagicMock()
    leader.bus.sync_read.side_effect = OSError(6, "Device not configured")
    with patch.object(type(leader), "is_connected", new=property(lambda self: True)), \
         patch("lerobot_robot_fastgripper.fastgripper_leader.time.sleep"):
        with pytest.raises(OSError):
            leader.get_action()


def test_leader_disconnect_swallows_dead_device_errors(tmp_path):
    cfg = FastGripperLeaderConfig(port="/dev/null", id="t", calibration_dir=tmp_path)
    leader = FastGripperLeader(cfg)
    with patch("lerobot.teleoperators.so_leader.SOLeader.disconnect",
               side_effect=OSError(6, "Device not configured")):
        leader.disconnect()   # must not raise


def test_handshake_retries_a_single_missed_ping(tmp_path):
    from lerobot_robot_fastgripper import MultiTurnFeetechMotorsBus
    from lerobot.motors import Motor, MotorNormMode
    bus = MultiTurnFeetechMotorsBus("/dev/null", {"m1": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100)},
                                    multiturn_motors=())
    calls = {"n": 0}

    def flaky(self):
        calls["n"] += 1
        if calls["n"] < 3:
            raise RuntimeError("motor check failed\n\nMissing motor IDs:\n  - 5 (expected model: 777)")

    with patch("lerobot.motors.feetech.FeetechMotorsBus._handshake", new=flaky), \
         patch("time.sleep"):
        bus._handshake()
    assert calls["n"] == 3


def test_handshake_does_not_retry_other_errors(tmp_path):
    from lerobot_robot_fastgripper import MultiTurnFeetechMotorsBus
    from lerobot.motors import Motor, MotorNormMode
    bus = MultiTurnFeetechMotorsBus("/dev/null", {"m1": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100)},
                                    multiturn_motors=())
    with patch("lerobot.motors.feetech.FeetechMotorsBus._handshake", side_effect=RuntimeError("firmware mismatch")):
        with pytest.raises(RuntimeError, match="firmware"):
            bus._handshake()


def test_leader_get_action_reconnects_on_termios_error(tmp_path):
    import termios
    cfg = FastGripperLeaderConfig(port="/dev/null", id="t", calibration_dir=tmp_path,
                                  read_retry_delay_s=0, reconnect_timeout_s=2.0)
    leader = FastGripperLeader(cfg)
    calls = {"n": 0}

    def flush_fails_then_ok(_reg):
        calls["n"] += 1
        if calls["n"] == 1:
            raise termios.error(6, "Device not configured")   # from tcdrain() on a dead fd
        return {"gripper": 5.0}

    leader.bus = MagicMock()
    leader.bus.sync_read.side_effect = flush_fails_then_ok
    leader.bus.motors = {"gripper": None}
    leader.bus.port_handler.openPort.return_value = True
    leader.bus.port_handler.setBaudRate.return_value = True
    with patch.object(type(leader), "is_connected", new=property(lambda self: True)), \
         patch("lerobot_robot_fastgripper.fastgripper_leader.time.sleep"):
        assert leader.get_action() == {"gripper.pos": 5.0}
