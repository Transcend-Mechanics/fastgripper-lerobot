import json
import logging
import time

from lerobot.motors import MotorCalibration
from lerobot.motors.feetech import OperatingMode
from lerobot.robots.so_follower import SOFollower

from .config_fastgripper_follower import FastGripperFollowerConfig
from .multiturn_feetech_bus import (
    MULTITURN_GOAL_VELOCITY,
    MULTITURN_OVERLOAD_TORQUE,
    MULTITURN_TORQUE_LIMIT,
    MULTITURN_MAX_LIMIT,
    MultiTurnFeetechMotorsBus,
)

logger = logging.getLogger(__name__)


class FastGripperFollower(SOFollower):
    """SO-101 follower whose gripper is a worm-gear parallel gripper.

    Motor layout is identical to the stock SO-101 (six STS3215 on one serial
    bus); only the gripper's motor handling differs: multi-turn feedback,
    software-only range calibration, and (eventually) stall-homing on connect.
    The gripper still presents as a 0-100 action, so teleop, datasets and
    policies are unaffected.
    """

    config_class = FastGripperFollowerConfig
    name = "fastgripper_follower"
    multiturn_motors = frozenset({"gripper"})

    def __init__(self, config: FastGripperFollowerConfig):
        super().__init__(config)
        # Rebuild the bus as the multi-turn-aware subclass, reusing the motor
        # table the parent constructor defined.
        self.bus = MultiTurnFeetechMotorsBus(
            port=config.port,
            motors=self.bus.motors,
            calibration=self.calibration,
            multiturn_motors=self.multiturn_motors,
        )
        # True only after the multi-turn zero is established in THIS session
        # (restore/assume/home). Parking without it would drive to a raw
        # target that means nothing and save a false parked state.
        self._gripper_zero_ok = False

    def connect(self, calibrate: bool = True) -> None:
        super().connect(calibrate)
        mode = self.config.gripper_home_mode
        if mode == "auto":
            self.restore_gripper_from_parked()
        elif mode == "stall":
            self.home_gripper()
        elif mode == "assume_closed":
            self.assume_gripper_closed()
        elif mode != "off":
            raise ValueError(
                f"Unknown gripper_home_mode '{mode}' "
                "(expected 'auto', 'stall', 'assume_closed', or 'off')"
            )

    @property
    def _gripper_state_fpath(self):
        return self.calibration_dir / f"{self.id}_gripper_state.json"

    # The worm can't backdrive, but cutting servo torque at disconnect lets
    # gear backlash + the compliant linkage settle the READING by up to ~200
    # ticks (observed live: 47/48/102/202 across sessions). 450 clears that
    # with margin while still catching any real hand-move, which shows up as
    # hundreds-to-thousands of ticks (~450 ticks is only ~2% of the stroke).
    PARK_TOLERANCE_TICKS = 450
    PARK_TIMEOUT_S = 15.0        # full stroke at speed is ~7 s

    def restore_gripper_from_parked(self) -> None:
        """Zero-motion start: verify the gripper is where the last session
        parked it (closed), then restore the counter and calibration.

        The worm cannot backdrive while unpowered, so if the boot reading
        matches the parked reading the position is trustworthy — across
        power cycles included."""
        motor = "gripper"
        path = self._gripper_state_fpath
        if not path.exists():
            raise RuntimeError(
                "No parked gripper state found (first run, or state file removed). "
                "One-time setup: close the gripper fully by hand (jog tool, slow, "
                "until the guard stops), then run once with "
                "--robot.gripper_home_mode=assume_closed. It will park itself on "
                "exit and 'auto' will work from then on."
            )
        parked = json.loads(path.read_text())["parked_raw"]
        boot = self.bus.read("Present_Position", motor, normalize=False)
        if abs(boot - parked) > self.PARK_TOLERANCE_TICKS:
            raise RuntimeError(
                f"Gripper is not where the last session parked it (read {boot}, "
                f"expected ~{parked}) — it was likely moved by hand while off. "
                "Close it fully and run once with "
                "--robot.gripper_home_mode=assume_closed to re-establish zero."
            )
        self.assume_gripper_closed()
        logger.info("Gripper restored from parked state (boot=%d, parked=%d)", boot, parked)

    def park_gripper(self) -> None:
        """Drive the gripper to closed, re-seed, and save the parked state.
        Called on disconnect so the next session's 'auto' start is instant."""
        motor = "gripper"
        bus = self.bus
        cal = self.calibration.get(motor)
        if cal is None:
            return
        target = cal.range_min
        try:
            bus.write("Goal_Position", motor, target, normalize=False)
            t0 = time.time()
            while time.time() - t0 < self.PARK_TIMEOUT_S:
                pos = bus.read("Present_Position", motor, normalize=False)
                if abs(pos - target) < 60:
                    break
                time.sleep(0.1)
        except RuntimeError:
            logger.warning("Parking: error flag mid-move; clearing and saving best-effort state")
        bus.clear_overload(motor)  # error-tolerant torque-off
        seeded = bus.reseed_position(motor)
        self._gripper_state_fpath.parent.mkdir(parents=True, exist_ok=True)
        self._gripper_state_fpath.write_text(json.dumps({"parked_raw": int(seeded)}))
        logger.info("Gripper parked closed and state saved (raw=%d)", seeded)

    def disconnect(self) -> None:
        if self.config.park_gripper_closed_on_disconnect:
            if not self._gripper_zero_ok:
                # Never park on an unestablished zero: the raw target would be
                # meaningless (blind motion) and the saved state a false claim.
                # Seen live: a failed 'auto' restore followed by park drove the
                # gripper ~1000 ticks and overwrote a stale-but-honest state.
                logger.warning(
                    "Skipping gripper park: zero was never established this "
                    "session. Close the gripper (fastgripper jog) and re-run "
                    "`fastgripper setup` before the next session."
                )
            else:
                try:
                    self.park_gripper()
                except Exception:
                    # The state file now describes a position we failed to
                    # reach — an honest "unknown" beats a wrong claim.
                    self._gripper_state_fpath.unlink(missing_ok=True)
                    logger.warning(
                        "Gripper parking failed; cleared parked state. Close the "
                        "gripper and re-run `fastgripper setup` before the next "
                        "session.",
                        exc_info=True,
                    )
        try:
            super().disconnect()
        except RuntimeError:
            # A latched error bit on ANY motor (seen live: shoulder_lift
            # overload after minutes of holding a pose) makes the standard
            # disable-torque-on-disconnect raise. Clear all latches with the
            # error-tolerant torque cycle, then disconnect normally.
            logger.warning("Disconnect hit a latched motor error; clearing latches and retrying")
            for motor in self.bus.motors:
                self.bus.clear_overload(motor)
            super().disconnect()

    def configure(self) -> None:
        super().configure()
        # The parent just wrote Operating_Mode=POSITION to every motor, which
        # undoes the multi-turn setup — re-assert it for the gripper last.
        # (Torque must be off: enabling torque outside the limit window, or
        # writing EEPROM under torque, latches an Overload error.)
        with self.bus.torque_disabled():
            for motor in self.multiturn_motors:
                self.bus.configure_multiturn_motor(motor)

    def calibrate(self) -> None:
        if self.calibration:
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, "
                "or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
                self.bus.write_calibration(self.calibration)
                return

        logger.info(f"\nRunning calibration of {self}")
        self.bus.disable_torque()
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)

        # The gripper must already be counting turns while we record its range.
        self.bus.ensure_multiturn_feedback()

        input(
            f"Move {self} to the middle of its range of motion (the gripper knob "
            "position does not matter) and press ENTER...."
        )
        # Half-turn homing is meaningless for a multi-turn axis (Homing_Offset
        # is limited to +/- half a turn), so the gripper keeps offset 0 and its
        # absolute recorded ticks ARE the calibration.
        arm_motors = [m for m in self.bus.motors if m not in self.multiturn_motors]
        homing_offsets = self.bus.set_half_turn_homings(arm_motors)
        homing_offsets.update(dict.fromkeys(self.multiturn_motors, 0))

        # The gripper is NOT range-recorded: the worm can't be back-driven by
        # hand, so its min==max would fail LeRobot's validation (seen live).
        # Its calibrated range comes from the established zero instead —
        # identical to what assume_gripper_closed()/setup writes.
        full_turn_motor = "wrist_roll"
        unknown_range_motors = [
            m for m in self.bus.motors if m != full_turn_motor and m not in self.multiturn_motors
        ]
        print(
            f"Move all arm joints except '{full_turn_motor}' sequentially through their entire "
            "ranges of motion.\n(Leave the gripper alone — it is calibrated by "
            "`fastgripper setup`, not here.)\nRecording positions. Press ENTER to stop..."
        )
        range_mins, range_maxes = self.bus.record_ranges_of_motion(unknown_range_motors)
        range_mins[full_turn_motor] = 0
        range_maxes[full_turn_motor] = 4095
        for motor in self.multiturn_motors:
            range_mins[motor] = self.HOME_SEED
            range_maxes[motor] = self.HOME_SEED + self.config.gripper_stroke_ticks

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=0,
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    # Stall-homing profile (hardware-validated 2026-07-14 on a free-spin rig
    # with hand-applied stops: 3/3 detections, no unrecoverable latch).
    # Constants re-tuned against the REAL worm gripper survey (2026-07-15):
    # cruise load 110-247, compliance-windup zone 330-440, hard stop 640+.
    HOMING_TORQUE_LIMIT = 400     # gentle push (load pegs here at the stop)
    HOMING_VELOCITY = 500         # ticks/s cruise toward the stop
    HOMING_LOAD_THRESHOLD = 350   # stall = load >= this SUSTAINED for the
    #   window. Real-gripper data: slow cruise never exceeds ~250; contact
    #   saturates toward the 400 torque limit. NOTE: no position-freeze
    #   requirement — the compliant 4-bar creeps (winds up) for 1000+ ticks
    #   under load, so waiting for stillness grinds deep into the spring
    #   (found live: homing from closed pressed far past the natural stop).
    HOMING_WINDOW_S = 0.8         # sustained-load duration to call it a stop
    HOMING_BACKOFF_TICKS = 600    # enough to fully escape the windup preload
    HOMING_CRUISE_STEP = 600      # goal lead while cruising. Was 1500: the
    #   re-seed guard is step+200 and re-seeds land at 2048, so a close-
    #   direction cruise had only ~350 ticks of runway per re-seed (churn
    #   found live in the jog tool); 600 gives ~1250
    HOMING_TIMEOUT_S = 45.0       # > full stroke at homing speed
    HOME_SEED = 2048              # reseed_position() lands here (half a turn)

    def assume_gripper_closed(self) -> None:
        """Manual homing: trust that the mechanism is at the closed stop.

        For use when a human cranked the worm knob to fully closed while the
        robot was unpowered (the worm can't backdrive, so the position holds).
        Re-seeds the counter so the current position reads HOME_SEED and
        writes the same calibration stall-homing would — with zero motion and
        no verification. A wrong assumption gives a silently wrong zero;
        prefer "stall" mode when the mechanism can home itself.
        """
        motor = "gripper"
        self.bus.clear_overload(motor)  # error-tolerant torque-off
        seeded = self.bus.reseed_position(motor)
        logger.info("Gripper manually homed (assume_closed): re-seeded to %d", seeded)
        # Record the parked-state immediately so "auto" works next session
        # even if this one ends without a clean disconnect.
        self._gripper_state_fpath.parent.mkdir(parents=True, exist_ok=True)
        self._gripper_state_fpath.write_text(json.dumps({"parked_raw": int(seeded)}))
        self.calibration[motor] = MotorCalibration(
            id=self.bus.motors[motor].id,
            drive_mode=0,
            homing_offset=0,
            range_min=self.HOME_SEED,
            range_max=self.HOME_SEED + self.config.gripper_stroke_ticks,
        )
        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        self._gripper_zero_ok = True

    def home_gripper(self) -> None:
        """Find the closed hard stop by stall detection and re-seed the
        volatile multi-turn counter so the stop reads HOME_SEED (2048).

        After homing, the gripper's calibrated range is
        [HOME_SEED, HOME_SEED + gripper_stroke_ticks] and the 0-100 action maps
        onto it. The compliant 4-bar softens the stall contact; firmware
        overload protection is parked out of the way during the cruise and
        restored after.
        """
        motor = "gripper"
        bus = self.bus
        bus.disable_torque(motor)
        bus.configure_multiturn_motor(motor)
        # Park firmware protection so our detector (sub-second) always wins;
        # restored below.
        bus.write("Max_Torque_Limit", motor, self.HOMING_TORQUE_LIMIT)
        bus.write("Overload_Torque", motor, 80)
        bus.write("Protection_Time", motor, 254)
        bus.write("Acceleration", motor, 80)
        bus.write("Goal_Velocity", motor, self.HOMING_VELOCITY, normalize=False)
        bus.enable_torque(motor)

        window: list[tuple[float, int, int]] = []  # (t, pos, load)
        stall_pos: int | None = None
        # Guard margin must exceed the cruise step: the goal is pos +/- STEP
        # and must stay inside [0, MULTITURN_MAX_LIMIT] or the servo rejects
        # it with an error status (validated live: goal -29 -> write error).
        step = self.HOMING_CRUISE_STEP
        guard_low, guard_high = step + 200, MULTITURN_MAX_LIMIT - (step + 200)
        t0 = time.time()
        try:
            while time.time() - t0 < self.HOMING_TIMEOUT_S:
                try:
                    pos = bus.read("Present_Position", motor, normalize=False)
                    # Goals clamp to [0, MULTITURN_MAX_LIMIT]; a long cruise
                    # can exhaust the window (esp. after power-on, when the
                    # counter starts within one turn). Re-seed mid-cruise —
                    # the shaft doesn't move, only the reading jumps to 2048.
                    if pos < guard_low or pos > guard_high:
                        bus.reseed_position(motor)
                        bus.enable_torque(motor)  # reseed path leaves torque off
                        window.clear()
                        continue
                    load = bus.read("Present_Load", motor, normalize=False)
                    now = time.time()
                    window.append((now, pos, abs(load)))
                    window = [s for s in window if now - s[0] <= self.HOMING_WINDOW_S]
                    # cruise toward the closed stop (goal kept inside window)
                    goal = pos + self.config.gripper_close_direction * step
                    bus.write(
                        "Goal_Position",
                        motor,
                        max(0, min(MULTITURN_MAX_LIMIT, goal)),
                        normalize=False,
                    )
                except RuntimeError:
                    # A latched error bit (e.g. Overload) poisons every status
                    # packet and makes any read/write raise. Validated
                    # recovery: torque-cycle unlatch, then resume the cruise.
                    logger.warning("Homing: error flag latched mid-cruise; clearing and resuming")
                    bus.clear_overload(motor)
                    bus.enable_torque(motor)
                    window.clear()
                    continue
                if len(window) >= 4 and (window[-1][0] - window[0][0]) >= self.HOMING_WINDOW_S * 0.9:
                    min_load = min(s[2] for s in window)
                    if min_load >= self.HOMING_LOAD_THRESHOLD:
                        stall_pos = pos
                        break
                time.sleep(0.06)
        finally:
            if stall_pos is None:
                # clear_overload is the error-tolerant "torque off": a plain
                # disable_torque raises if an error bit is riding.
                bus.clear_overload(motor)

        if stall_pos is None:
            raise RuntimeError(
                f"Gripper homing found no hard stop within {self.HOMING_TIMEOUT_S}s. "
                "Check the mechanism, then re-run."
            )

        # Relieve pressure, settle, then re-seed the counter at the backed-off
        # position: closed stop reads ~HOME_SEED.
        backoff_target = (
            stall_pos - self.config.gripper_close_direction * self.HOMING_BACKOFF_TICKS
        )
        bus.write("Goal_Position", motor, backoff_target, normalize=False)
        # WAIT for the back-off to finish before re-seeding. A fixed short sleep
        # cuts the move off after a blip: at HOMING_VELOCITY a full backoff takes
        # longer than 0.6 s (e.g. 600 ticks @ 500 = 1.2 s), and reseed_position
        # freezes the reading wherever it landed. Poll to completion instead.
        deadline = time.time() + self.HOMING_BACKOFF_TICKS / max(self.HOMING_VELOCITY, 1) + 2.0
        while time.time() < deadline:
            if abs(bus.read("Present_Position", motor, normalize=False) - backoff_target) <= 40:
                break
            time.sleep(0.05)
        bus.clear_overload(motor)  # error-tolerant torque-off
        seeded = bus.reseed_position(motor)
        logger.info("Gripper homed: stall at %d, re-seeded to %d", stall_pos, seeded)

        # Operating protection profile. NOTE: deliberately NOT the stock
        # Overload_Torque=25 — bench-validated that Overload_Torque is the
        # trip *threshold* (% load sustained for Protection_Time), and 25
        # latches on any multi-second stroke. 80 completes 2-turn strokes.
        bus.write("Max_Torque_Limit", motor, MULTITURN_TORQUE_LIMIT)
        bus.write("Overload_Torque", motor, MULTITURN_OVERLOAD_TORQUE)
        bus.write("Protection_Time", motor, 200)
        # Restore operating speed — homing dropped Goal_Velocity to the slow
        # cruise value; without this the whole session runs at homing speed.
        bus.write("Goal_Velocity", motor, MULTITURN_GOAL_VELOCITY, normalize=False)
        bus.write("Acceleration", motor, 150)

        self.calibration[motor] = MotorCalibration(
            id=bus.motors[motor].id,
            drive_mode=0,
            homing_offset=0,
            range_min=self.HOME_SEED,
            range_max=self.HOME_SEED + self.config.gripper_stroke_ticks,
        )
        bus.write_calibration(self.calibration)
        self._save_calibration()
        self._gripper_zero_ok = True
