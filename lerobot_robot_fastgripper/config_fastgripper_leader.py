from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig
from lerobot.teleoperators.so_leader.config_so_leader import SOLeaderConfig


@TeleoperatorConfig.register_subclass("fastgripper_leader")
@dataclass
class FastGripperLeaderConfig(TeleoperatorConfig, SOLeaderConfig):
    """Stock SO-101 leader with a read-retry tolerant get_action().

    Identical hardware and calibration to `so101_leader` (calibration files
    are shared: same `so_leader` directory and id). The only difference is
    that a transient serial glitch on the leader no longer kills teleop.
    """

    read_retries: int = 4
    read_retry_delay_s: float = 0.002
