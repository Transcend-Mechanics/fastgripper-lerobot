"""SO-101 leader with retry-tolerant reads.

LeRobot's SOLeader.get_action() does one sync_read and raises on the first
'no status packet'. On this bench a single garbled transaction is common
enough (half-duplex bus, shared USB tree) that it ended sessions several
times in one afternoon while every servo was perfectly healthy. The read is
idempotent, so retry a few times before giving up — the same treatment the
follower already gets in FastGripperFollower.get_observation().

A vanished serial device ([Errno 6] Device not configured) -- the CH343
re-enumerating when its bus-powered hub sags under the follower's load
(seen live 2026-08-27/29 on two different leaders) -- is handled too: the
port is reopened by path for up to `reconnect_timeout_s` and the read
resumes, instead of ending the session. The follower holds its last goal
in the meantime.
"""

import logging
import os
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
            except OSError as e:          # pyserial SerialException is an OSError
                last_exc = e
                if self._reconnect():
                    continue
                break
        assert last_exc is not None
        raise last_exc

    def disconnect(self) -> None:
        """A leader whose USB device is gone must not take the follower's
        orderly shutdown with it: lerobot-teleoperate calls teleop.disconnect()
        BEFORE robot.disconnect(), so an exception here skips the follower's
        park-and-save (seen live: park ran from __del__ at interpreter exit,
        failed, and cleared the parked state)."""
        try:
            super().disconnect()
        except Exception as e:  # OSError / termios.error / ConnectionError
            logger.warning("%s: disconnect skipped cleanup (device gone: %s)", self, e)
            try:
                self.bus.port_handler.closePort()
            except Exception:
                pass

    def _reconnect(self) -> bool:
        """Reopen the serial port by path after the device re-enumerated."""
        port = self.config.port
        deadline = time.time() + self.config.reconnect_timeout_s
        logger.warning("%s: serial device lost -- reopening %s", self, port)
        ph = self.bus.port_handler
        while time.time() < deadline:
            time.sleep(0.25)
            if not os.path.exists(port):
                continue
            try:
                try:
                    ph.closePort()
                except Exception:
                    pass
                if ph.openPort() and ph.setBaudRate(self.bus.port_handler.baudrate):
                    logger.warning("%s: reconnected %s", self, port)
                    return True
            except Exception:
                continue
        logger.error("%s: could not reopen %s within %.0fs", self, port, self.config.reconnect_timeout_s)
        return False
