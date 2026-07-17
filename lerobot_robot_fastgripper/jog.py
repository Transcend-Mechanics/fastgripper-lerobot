"""Keyboard jog + torque survey for the STS3215 multi-turn worm gripper.

Port of the YAM Test repo's calibrate.py (DM-J4310/CAN) to the SO-101 Feetech
setup, using the hardware-validated multi-turn recipe (Operating_Mode 3 +
Phase bit 4 + widened limits). Jog the worm through the mechanism and watch
the live load readout to measure torque requirements across the stroke.

Torque caveat: the STS3215 reports Present_Load as 0-1000 PWM duty, not N*m.
The N*m column is APPROXIMATE (duty x 2.94 N*m stall @ 12 V) — use it for
"where is it hardest / how close to the limit", not for lab numbers.

Keys:
    a / d : HOLD to jog open / closed (dead-man: release stops)
    f     : toggle slow/FAST jog speed
    r     : reset the peak-load readout
    space : stop jogging immediately
    o     : mark current position as OPEN
    c     : mark current position as CLOSED
    q     : save marks + exit               Ctrl-C: abort without saving

Positions are "logical ticks": continuous across mid-jog counter re-seeds
(the servo's goal window is [0, 28672]; the tool re-seeds transparently and
keeps a running offset, so marks/span stay valid across many turns).

Usage (needs a real terminal — run via `!` in Claude Code or a Terminal tab):
    .venv/bin/python fastgripper jog                  # follower_2 gripper
    .venv/bin/python fastgripper jog --port /dev/cu.usbmodemXXXX --id 6
    .venv/bin/python fastgripper jog --status         # one-line smoke test
"""

import argparse
import json
import select
import sys
import termios
import time
import tty

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

CAL_FILE = "gripper_jog_cal.json"
DEFAULT_PORT = "/dev/cu.usbmodem5C4C1267161"  
DEFAULT_ID = 6

VEL_SLOW = 500     # ticks/s (homing speed — validated gentle)
VEL_FAST = 2000
JOG_LEAD = 600     # goal lead distance per loop while a jog key is held
HOLD_TIMEOUT_S = 0.45
# Measured on the real worm gripper (2026-07-15): at SLOW, cruise 110-247,
# windup 330-440, stop 640+. At FAST (2000 t/s) worm drag alone runs 550-650
# and the stop pegs at the torque limit (800) — so the guard must scale with
# speed or it fires on normal cruising (found live).
LOAD_LIMIT_SLOW = 500
LOAD_LIMIT_FAST = 720
LOAD_TRIP_S = 0.3
TORQUE_LIMIT_REG = 800  # Max_Torque_Limit during survey (80%)
LOOP_HZ = 30
STALL_TORQUE_NM = 2.94  # STS3215 @ 12 V, for the approximate N*m column
TICKS_PER_TURN = 4096
# Low guard must leave real runway below the 2048 re-seed point (re-seed churn
# otherwise pauses closing jogs every ~350 ticks — found live).
GUARD_LOW, GUARD_HIGH = JOG_LEAD + 200, 26900
MAX_LIMIT = 28672


class Servo:
    """Raw, error-tolerant IO wrapper (error bits ride in status packets and
    must never crash the survey; overload latches are cleared inline)."""

    def __init__(self, port: str, motor_id: int):
        self.id = motor_id
        self.bus = FeetechMotorsBus(
            port=port, motors={"m": Motor(motor_id, "sts3215", MotorNormMode.RANGE_0_100)}
        )
        self.bus._connect(handshake=False)
        self.bus.set_baudrate(1_000_000)
        self.ph, self.po = self.bus.packet_handler, self.bus.port_handler
        self.table = self.bus.model_ctrl_table["sts3215"]
        self.offset = 0  # logical = raw + offset (survives re-seeds)

    def _rw(self, reg):
        addr, ln = self.table[reg]
        fn = self.ph.read1ByteTxRx if ln == 1 else self.ph.read2ByteTxRx
        val, _, err = fn(self.po, self.id, addr)
        return val, err

    def read(self, reg):
        return self._rw(reg)[0]

    def write(self, reg, value):
        addr, ln = self.table[reg]
        fn = self.ph.write1ByteTxRx if ln == 1 else self.ph.write2ByteTxRx
        fn(self.po, self.id, addr, value)

    def pos_raw(self):
        v, err = self._rw("Present_Position")
        return (-(v & 0x7FFF) if v & 0x8000 else v), err

    def pos(self):
        raw, err = self.pos_raw()
        return raw + self.offset, err

    def load(self):
        v = self.read("Present_Load")
        return (v & 0x3FF)

    def goal(self, logical):
        raw = max(0, min(MAX_LIMIT, logical - self.offset))
        self.write("Goal_Position", raw if raw >= 0 else (0x8000 | -raw))

    def unlatch(self):
        for v in (0, 1, 0):
            self.write("Torque_Enable", v)
            time.sleep(0.15)

    def reseed_if_needed(self):
        raw, _ = self.pos_raw()
        if GUARD_LOW <= raw <= GUARD_HIGH:
            return False
        self.write("Torque_Enable", 128)  # recalibrate: current position -> 2048
        time.sleep(0.05)
        new_raw, err = self.pos_raw()
        self.offset += raw - new_raw
        if err:
            self.unlatch()  # only pay the slow torque-cycle if a flag latched
        self.write("Torque_Enable", 1)
        return True

    def setup_multiturn(self):
        self.unlatch()
        self.write("Lock", 0)
        phase = self.read("Phase")
        if not (phase & 0x10):
            self.write("Phase", phase | 0x10)
        self.write("Min_Position_Limit", 0)
        self.write("Max_Position_Limit", MAX_LIMIT)
        self.write("Operating_Mode", 3)
        self.write("Max_Torque_Limit", TORQUE_LIMIT_REG)
        self.write("Overload_Torque", 80)   # park firmware protection above us
        self.write("Protection_Time", 254)
        self.write("Acceleration", 100)

    def close(self, restore_protection=True):
        self.unlatch()  # leaves torque off, flags clear
        if restore_protection:
            self.write("Overload_Torque", 80)
            self.write("Protection_Time", 200)
            self.write("Max_Torque_Limit", 500)
        self.po.closePort()


def read_key():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--id", type=int, default=DEFAULT_ID)
    parser.add_argument("--cal", default=CAL_FILE)
    parser.add_argument("--gripper", default="default")
    parser.add_argument("--status", action="store_true", help="print one telemetry line and exit")
    args = parser.parse_args()

    servo = Servo(args.port, args.id)

    # Refuse to run against a silent servo (otherwise every read returns 0 and
    # the display shows plausible-looking garbage — found live).
    import scservo_sdk as _scs

    _, comm, _ = servo.ph.ping(servo.po, servo.id)
    if comm != _scs.COMM_SUCCESS:
        raise SystemExit(
            f"No response from servo id {servo.id} on {args.port} — check power, "
            "cabling, and port (try --status after fixing)."
        )

    if args.status:
        _, _, err = servo.ph.ping(servo.po, servo.id)
        pos, _ = servo.pos()
        print(
            f"id={args.id} err=0b{err:08b} pos={pos} load={servo.load()} "
            f"temp={servo.read('Present_Temperature')}C "
            f"mode={servo.read('Operating_Mode')} phase=0x{servo.read('Phase'):02x}"
        )
        servo.po.closePort()
        return

    if not sys.stdin.isatty():
        raise SystemExit(
            "gripper_jog.py needs a real terminal for keyboard jogging — run it via "
            "`!` in Claude Code or a Terminal window, not a piped shell"
        )

    servo.setup_multiturn()
    servo.write("Goal_Velocity", VEL_SLOW)
    servo.write("Torque_Enable", 1)

    marks: dict[str, int] = {}
    direction = 0
    fast = False
    peak_load = 0
    over_since = None
    last_jog_key = 0.0

    pos, _ = servo.pos()
    print(__doc__.split("Usage:")[0])
    print(f"start position: {pos} ticks — jog away!\n")

    old_term = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            t0 = time.monotonic()
            key = read_key()
            if key in ("a", "A", "d", "D"):
                direction = -1 if key.lower() == "a" else 1
                last_jog_key = t0
            elif key in ("f", "F"):
                fast = not fast
                servo.write("Goal_Velocity", VEL_FAST if fast else VEL_SLOW)
            elif key == " ":
                direction = 0
            elif key == "r":
                peak_load = 0
            elif key == "o":
                marks["open"] = pos
                print(f"\nmarked OPEN   = {marks['open']} ticks")
            elif key == "c":
                marks["closed"] = pos
                print(f"\nmarked CLOSED = {marks['closed']} ticks")
            elif key == "q":
                break

            if direction != 0 and t0 - last_jog_key > HOLD_TIMEOUT_S:
                direction = 0

            if servo.reseed_if_needed():
                print("\n(counter re-seeded mid-jog; logical position continuous)")

            pos, err = servo.pos()
            load = servo.load()
            peak_load = max(peak_load, load)

            if direction != 0:
                # torque guard: stop driving into a jam (threshold tracks speed
                # because worm drag duty rises steeply with velocity)
                limit = LOAD_LIMIT_FAST if fast else LOAD_LIMIT_SLOW
                if load > limit:
                    if over_since is None:
                        over_since = time.monotonic()
                    elif time.monotonic() - over_since > LOAD_TRIP_S:
                        direction = 0
                        over_since = None
                        print(f"\n!! load {load} > {limit} — jog stopped")
                else:
                    over_since = None
                servo.goal(pos + direction * JOG_LEAD)
            else:
                over_since = None
                servo.goal(pos)  # hold

            if err:
                servo.unlatch()
                servo.write("Torque_Enable", 1)
                print(f"\n(error flag 0b{err:08b} cleared)")

            turns = pos / TICKS_PER_TURN
            nm = load / 1000 * STALL_TORQUE_NM
            print(
                f"\r{pos:+8d}t {turns:+6.2f}rev load{load:4d} (~{nm:4.2f}Nm) "
                f"pk{peak_load:4d} jog{direction:+2d} [{'FAST' if fast else 'slow'}] ",
                end="",
                flush=True,
            )
            time.sleep(max(0.0, 1.0 / LOOP_HZ - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        print("\naborted (Ctrl-C) — nothing saved")
        marks.clear()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_term)
        servo.close()
        print("\nmotor torque off, protection restored")

    if "open" in marks and "closed" in marks:
        try:
            with open(args.cal) as f:
                store = json.load(f)
        except FileNotFoundError:
            store = {"grippers": {}}
        span = abs(marks["closed"] - marks["open"])
        store["grippers"][args.gripper] = {
            "port": args.port,
            "motor_id": args.id,
            "open_ticks": marks["open"],
            "closed_ticks": marks["closed"],
            "span_ticks": span,
            "span_turns": round(span / TICKS_PER_TURN, 2),
            "peak_load": peak_load,
            "peak_torque_nm_approx": round(peak_load / 1000 * STALL_TORQUE_NM, 2),
            "method": "keyboard",
            "calibrated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        with open(args.cal, "w") as f:
            json.dump(store, f, indent=2)
        print(
            f"saved '{args.gripper}' in {args.cal}: span={span} ticks "
            f"({span / TICKS_PER_TURN:.2f} turns), peak load {peak_load} "
            f"(~{peak_load / 1000 * STALL_TORQUE_NM:.2f} Nm)"
        )
    else:
        missing = {"open", "closed"} - marks.keys()
        print(f"NOT saved — missing marks: {', '.join(missing) if missing else ''}")


if __name__ == "__main__":
    main()
