import logging
from typing import Iterable

from lerobot.motors import MotorCalibration
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

logger = logging.getLogger(__name__)

# Phase register (addr 18) bit 4 selects the STS3215 angle feedback mode.
# LeRobot's FeetechMotorsBus.configure_motors clears it to force single-turn
# readings in [0, 4095]; with it set, Present_Position accumulates across
# turns (sign-magnitude int16 on the wire -> usable range of +/- 8 turns).
PHASE_MULTITURN_BIT = 0x10

# Hardware-validated (2026-07-14, STS3215 on free-spin rig): goals clamp to the
# [Min,Max]_Position_Limit window in every operating mode, the registers accept
# values > 4095, and 0/0 means "clamp to zero" NOT "limits off". So multi-turn
# motors get a wide positive window and must operate in [0, stroke].
MULTITURN_MAX_LIMIT = 28672  # 7 turns

# In mode 3 ("step servo") this firmware treats Goal_Position as an ABSOLUTE
# multi-turn target. Goal_Velocity must be nonzero or the motor creeps.
# Wide open: full duty available and the overload-protection threshold moved
# out of reach (Overload_Torque=100 -> trips only at a sustained 1000, which
# demand at full speed ~850-950 doesn't pin). The earlier fast-then-crawl was
# duty pegging AT the limit (800) which equalled the trip threshold for
# Protection_Time (~2 s) -> torque cut to 20%. Thermal note: full-duty
# strokes are fine intermittently; watch servo temp on heavy continuous use.
MULTITURN_GOAL_VELOCITY = 3072
MULTITURN_TORQUE_LIMIT = 1000
MULTITURN_OVERLOAD_TORQUE = 100


class MultiTurnFeetechMotorsBus(FeetechMotorsBus):
    """FeetechMotorsBus where selected motors run in multi-turn mode.

    For multi-turn motors:
    - Phase bit 4 is kept set (upstream configure_motors clears it).
    - Operating_Mode is 3 (absolute multi-turn positioning on this firmware).
    - Position limits are widened to [0, MULTITURN_MAX_LIMIT]; the true
      calibrated range lives ONLY in the software calibration (Homing_Offset
      is unusable: sign-magnitude with sign bit 11 -> only +/- 2047 ticks).
    - is_calibrated skips the register round-trip for these motors.
    """

    HANDSHAKE_RETRIES = 4
    HANDSHAKE_RETRY_DELAY_S = 0.2

    def _handshake(self) -> None:
        """LeRobot pings every motor once at connect and aborts on a single
        miss. On this bus a dropped ping is routine (half-duplex, shared USB
        tree; preflight saw all 6 answer one second earlier while connect
        found motor 5 'missing', live 2026-08-29). Retry before giving up."""
        import time as _time

        for attempt in range(self.HANDSHAKE_RETRIES):
            try:
                super()._handshake()
                if attempt:
                    logger.warning("handshake succeeded after %d retr%s", attempt, "y" if attempt == 1 else "ies")
                return
            except RuntimeError as e:
                if "Missing motor IDs" not in str(e) or attempt == self.HANDSHAKE_RETRIES - 1:
                    raise
                _time.sleep(self.HANDSHAKE_RETRY_DELAY_S)

    def __init__(self, *args, multiturn_motors: Iterable[str] = (), **kwargs):
        super().__init__(*args, **kwargs)
        self.multiturn_motors = set(multiturn_motors)
        unknown = self.multiturn_motors - set(self.motors)
        if unknown:
            raise ValueError(f"multiturn_motors not present on the bus: {unknown}")

    def ensure_multiturn_feedback(self) -> None:
        """Set Phase bit 4 on all multi-turn motors. Requires an open port."""
        for motor in self.multiturn_motors:
            phase = self.read("Phase", motor, normalize=False)
            if not (phase & PHASE_MULTITURN_BIT):
                self.write("Phase", motor, phase | PHASE_MULTITURN_BIT)
                logger.info("Enabled multi-turn feedback on '%s' (Phase=0x%02x)", motor, phase | 0x10)

    def configure_multiturn_motor(self, motor: str) -> None:
        """Full validated multi-turn recipe. Call with torque disabled, BEFORE
        enabling torque (torque-on outside the limit window latches an
        Overload error that then poisons every status packet)."""
        phase = self.read("Phase", motor, normalize=False)
        if not (phase & PHASE_MULTITURN_BIT):
            self.write("Phase", motor, phase | PHASE_MULTITURN_BIT)
        self.write("Min_Position_Limit", motor, 0)
        self.write("Max_Position_Limit", motor, MULTITURN_MAX_LIMIT)
        self.write("Operating_Mode", motor, OperatingMode.STEP.value)
        self.write("Goal_Velocity", motor, MULTITURN_GOAL_VELOCITY, normalize=False)
        self.write("Max_Torque_Limit", motor, MULTITURN_TORQUE_LIMIT)
        self.write("Overload_Torque", motor, MULTITURN_OVERLOAD_TORQUE)
        # The robot-level configure halves P (16) on all joints for arm
        # smoothness; on a multi-turn axis that stretches the end-of-move
        # taper into a long crawl. Full P + max accel keeps the profile crisp.
        self.write("P_Coefficient", motor, 32)
        self.write("Acceleration", motor, 254)

    def configure_motors(self, **kwargs) -> None:
        super().configure_motors(**kwargs)
        for motor in self.multiturn_motors:
            self.configure_multiturn_motor(motor)

    def write_calibration(self, calibration_dict: dict[str, MotorCalibration], cache: bool = True) -> None:
        single_turn = {m: c for m, c in calibration_dict.items() if m not in self.multiturn_motors}
        super().write_calibration(single_turn, cache=False)

        for motor in self.multiturn_motors & set(calibration_dict):
            cal = calibration_dict[motor]
            if cal.range_min < 0:
                raise ValueError(
                    f"Multi-turn motor '{motor}' calibration must be in the positive domain "
                    f"[0, stroke] (got range_min={cal.range_min}): goals clamp at "
                    "Min_Position_Limit=0. Re-home so the closed stop reads near 0."
                )
            if self.protocol_version == 0:
                self.write("Homing_Offset", motor, 0)
            self.write("Min_Position_Limit", motor, 0)
            self.write("Max_Position_Limit", motor, MULTITURN_MAX_LIMIT)

        if cache:
            self.calibration = calibration_dict

    def clear_overload(self, motor: str) -> bool:
        """Clear a latched Overload error (status bit 5) in software.

        Hardware-validated: cycling Torque_Enable 0 -> 1 -> 0 unlatches it (no
        power cycle needed). While latched, the error bit rides in every status
        packet, so use raw error-tolerant IO here."""
        import time as _time

        id_ = self.motors[motor].id
        addr, _ = self.model_ctrl_table[self.motors[motor].model]["Torque_Enable"]
        for val in (0, 1, 0):
            self.packet_handler.write1ByteTxRx(self.port_handler, id_, addr, val)
            _time.sleep(0.25)
        _, _, err = self.packet_handler.ping(self.port_handler, id_)
        return err == 0

    def reseed_position(self, motor: str) -> int:
        """Re-seed the multi-turn counter: current position becomes 2048.

        Uses Feetech's position-recalibration command (write 128 to
        Torque_Enable). May transiently latch an overload flag, which is
        cleared before returning. Returns the new raw position (~2048)."""
        id_ = self.motors[motor].id
        addr, _ = self.model_ctrl_table[self.motors[motor].model]["Torque_Enable"]
        self.packet_handler.write1ByteTxRx(self.port_handler, id_, addr, 128)
        self.clear_overload(motor)
        return self.read("Present_Position", motor, normalize=False)

    @property
    def is_calibrated(self) -> bool:
        motors_calibration = self.read_calibration()
        if set(motors_calibration) != set(self.calibration):
            return False

        for motor, cal in motors_calibration.items():
            if motor in self.multiturn_motors:
                # Registers hold the wide window by convention; the real range
                # is software-only.
                continue
            ref = self.calibration[motor]
            if (ref.range_min, ref.range_max) != (cal.range_min, cal.range_max):
                return False
            if self.protocol_version == 0 and ref.homing_offset != cal.homing_offset:
                return False

        return True
