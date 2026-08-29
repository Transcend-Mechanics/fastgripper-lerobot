"""SO-101 leader with retry-tolerant reads.

LeRobot's SOLeader.get_action() does one sync_read and raises on the first
'no status packet'. On this bench a single garbled transaction is common
enough (half-duplex bus, shared USB tree) that it ended sessions several
times in one afternoon while every servo was perfectly healthy. The read is
idempotent, so retry a few times before giving up — the same treatment the
follower already gets in FastGripperFollower.get_observation().

Note: a vanished serial device ([Errno 6] Device not configured) is NOT
recoverable here and still propagates — that is a cable/USB problem.
"""

import logging
import time

from lerobot.teleoperators.so_leader import SOLeader

from .config_fastgripper_leader import FastGripperLeaderConfig

logger = logging.getLogger(__name__)


class FastGripperLeader(SOLeader):
    config_class = FastGripperLeaderConfig
    name = "so_leader"  # share calibration files with so101_leader

    def __init__(self, config: FastGripperLeaderConfig):
        super().__init__(config)
        self.config = config

    def get_action(self) -> dict[str, float]:
        last_exc: Exception | None = None
        retries = max(1, int(self.config.read_retries))
        for attempt in range(retries):
            try:
                action = super().get_action()
                if attempt:
                    logger.warning(
                        "%s: recovered leader read after %d retr%s",
                        self, attempt, "y" if attempt == 1 else "ies",
                    )
                return action
            except ConnectionError as e:
                last_exc = e
                time.sleep(self.config.read_retry_delay_s)
        assert last_exc is not None
        raise last_exc
