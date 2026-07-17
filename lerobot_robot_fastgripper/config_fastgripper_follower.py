from dataclasses import dataclass

from lerobot.robots.config import RobotConfig
from lerobot.robots.so_follower.config_so_follower import SOFollowerConfig


@RobotConfig.register_subclass("fastgripper_follower")
@dataclass
class FastGripperFollowerConfig(RobotConfig, SOFollowerConfig):
    """SO-101 follower with a multi-turn worm-gear parallel gripper on motor 6.

    The gripper motor is a stock Feetech STS3215 kept in multi-turn feedback
    mode (Phase register bit 4 set): its position accumulates across the
    ~5.6 turns (~35 rad) of worm travel instead of wrapping at 4095.
    """

    # Full-stroke travel in encoder ticks (4096 per turn). MEASURED on the
    # real gripper 2026-07-15: 19,794 ticks (4.83 turns) between hard stops.
    # Default is measured span minus ~600 (homing back-off + open-side
    # margin) so the commanded range never drives into either stop.
    gripper_stroke_ticks: int = 19200

    # How to establish the gripper's multi-turn zero at connect:
    #   "auto"          - (default) verify the gripper is where the last
    #                     session parked it (closed) and restore instantly
    #                     with zero motion. Works because the worm cannot
    #                     backdrive while unpowered. Falls back with a clear
    #                     message if the state file is missing or the
    #                     mechanism was moved by hand.
    #   "stall"         - drive to the closed hard stop with stall detection
    #                     (works on rigid mechanisms; on this compliant worm
    #                     the motor-side load barely sees contact, so prefer
    #                     "auto")
    #   "assume_closed" - trust that a human placed the mechanism at the
    #                     closed stop while unpowered; zero motion, instant,
    #                     NO verification. The one-time setup for "auto".
    #   "off"           - skip homing entirely (turn counter must already be
    #                     valid from a previous session without power loss)
    gripper_home_mode: str = "auto"

    # On disconnect, drive the gripper to its closed position, re-seed the
    # counter, and save a parked-state file — this is what makes "auto"
    # instant and verification-backed on the next connect.
    park_gripper_closed_on_disconnect: bool = True

    # Encoder direction that drives the jaws toward the CLOSED hard stop
    # (depends on worm handedness): +1 or -1.
    gripper_close_direction: int = -1
