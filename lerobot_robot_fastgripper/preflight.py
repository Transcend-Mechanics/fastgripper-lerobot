"""Go/no-go preflight for an SO-101 leader + FastGripper follower pair.

Every check here corresponds to a failure that cost real bench time:
  - port missing / vanished           -> "Device not configured" mid-teleop
  - servo not answering               -> board power / daisy chain unplugged
  - servo bus voltage low             -> PSU off, arm running on USB 5 V
  - single-turn joint reading > 4095  -> stale turn counter; LeRobot calibration
                                         crashes with "Magnitude exceeds 2047"
                                         until the servo PSU is power-cycled
  - calibration file missing          -> lerobot-calibrate never run
  - parked state missing / mismatch   -> auto restore refuses (now: stall-homes)
  - trigger cal never reaches 0/100%  -> "closed is half open"

The hardware access is isolated in `collect_readings()` so the verdict logic
(`evaluate()`) is pure and unit-testable.
"""

from __future__ import annotations

import json
import os
import pathlib
from dataclasses import dataclass, field

SINGLE_TURN_MAX = 4095
PARK_TOLERANCE_TICKS = 450          # mirrors FastGripperFollower.PARK_TOLERANCE_TICKS
MIN_BUS_VOLTAGE = 6.0               # below this the servos are on USB 5 V, not the PSU
MAX_BUS_VOLTAGE = 13.0
MIN_TRIGGER_SPAN_TICKS = 300
TRIGGER_CLOSED_MAX_PCT = 3.0
TRIGGER_OPEN_MIN_PCT = 97.0

ARM_IDS = (1, 2, 3, 4, 5, 6)
ADDR_TORQUE, ADDR_GOAL, ADDR_POS, ADDR_VOLTAGE = 40, 42, 56, 62


@dataclass
class ArmReadings:
    port: str
    port_exists: bool = False
    alive: list[int] = field(default_factory=list)
    positions: dict[int, int] = field(default_factory=dict)
    voltages: dict[int, float] = field(default_factory=dict)
    open_error: str | None = None


@dataclass
class Readings:
    leader: ArmReadings
    follower: ArmReadings
    leader_cal: dict | None = None        # full calibration json (per motor)
    follower_cal: dict | None = None
    parked_raw: int | None = None         # from <follower_id>_gripper_state.json
    trigger_squeezed_pct: float | None = None   # only when interactive
    trigger_released_pct: float | None = None


@dataclass
class Finding:
    level: str      # "FAIL" | "WARN" | "OK"
    text: str


def trigger_pct(raw: int, cal: dict) -> float:
    """LeRobot's RANGE_0_100 normalisation for the leader gripper (clamped)."""
    lo, hi = cal["range_min"], cal["range_max"]
    if hi == lo:
        return float("nan")
    v = min(hi, max(lo, raw))
    pct = (v - lo) / (hi - lo) * 100.0
    return 100.0 - pct if cal.get("drive_mode") else pct


def evaluate(r: Readings) -> list[Finding]:
    out: list[Finding] = []

    for name, arm, single_turn in (("leader", r.leader, ARM_IDS), ("follower", r.follower, ARM_IDS[:5])):
        if not arm.port_exists:
            out.append(Finding("FAIL", f"{name}: port {arm.port} not present (cable/hub/USB — replug)"))
            continue
        if arm.open_error:
            out.append(Finding("FAIL", f"{name}: cannot open {arm.port}: {arm.open_error}"))
            continue
        missing = [i for i in ARM_IDS if i not in arm.alive]
        if missing:
            out.append(Finding("FAIL", f"{name}: servos not answering: {missing} "
                               "(servo PSU off? daisy chain unplugged?)"))
        else:
            out.append(Finding("OK", f"{name}: all 6 servos answer"))
        if arm.voltages:
            vmin = min(arm.voltages.values())
            vmax = max(arm.voltages.values())
            if vmin < MIN_BUS_VOLTAGE:
                out.append(Finding("FAIL", f"{name}: servo bus {vmin:.1f} V — PSU not powering the arm"))
            elif vmax > MAX_BUS_VOLTAGE:
                out.append(Finding("WARN", f"{name}: servo bus {vmax:.1f} V — above STS3215 12 V rating"))
            else:
                out.append(Finding("OK", f"{name}: servo bus {vmin:.1f}–{vmax:.1f} V"))
        stale = {i: p for i, p in arm.positions.items() if i in single_turn and p > SINGLE_TURN_MAX}
        if stale:
            out.append(Finding("FAIL", f"{name}: joint(s) {stale} read past one turn — stale turn "
                               "counter; power-cycle this arm's servo PSU (calibration would crash)"))

    if r.leader.port_exists and not r.leader.open_error:
        if r.leader_cal is None:
            out.append(Finding("FAIL", "leader: no calibration file (run `fastgripper calibrate --leader`)"))
        else:
            g = r.leader_cal.get("gripper")
            raw = r.leader.positions.get(6)
            if not g:
                out.append(Finding("FAIL", "leader: calibration has no gripper entry"))
            elif g["range_max"] - g["range_min"] < MIN_TRIGGER_SPAN_TICKS:
                out.append(Finding("FAIL", f"leader: trigger range {g['range_min']}..{g['range_max']} "
                                   "too narrow — recalibrate"))
            elif raw is not None:
                pct = trigger_pct(raw, g)
                where = "" if g["range_min"] <= raw <= g["range_max"] else " (OUTSIDE calibrated range)"
                out.append(Finding("OK" if not where else "WARN",
                                   f"leader: trigger now raw {raw} = {pct:.0f}%{where}"))
            if r.trigger_squeezed_pct is not None:
                lvl = "OK" if r.trigger_squeezed_pct <= TRIGGER_CLOSED_MAX_PCT else "FAIL"
                out.append(Finding(lvl, f"leader: trigger squeezed = {r.trigger_squeezed_pct:.1f}% "
                                   f"(need ≤{TRIGGER_CLOSED_MAX_PCT:.0f}% or the gripper never closes)"))
            if r.trigger_released_pct is not None:
                lvl = "OK" if r.trigger_released_pct >= TRIGGER_OPEN_MIN_PCT else "FAIL"
                out.append(Finding(lvl, f"leader: trigger released = {r.trigger_released_pct:.1f}% "
                                   f"(need ≥{TRIGGER_OPEN_MIN_PCT:.0f}% or the gripper never opens)"))

    if r.follower.port_exists and not r.follower.open_error:
        if r.follower_cal is None:
            out.append(Finding("FAIL", "follower: no calibration file (run `fastgripper calibrate`)"))
        else:
            out.append(Finding("OK", "follower: arm calibration present"))
        boot = r.follower.positions.get(6)
        if r.parked_raw is None:
            out.append(Finding("WARN", "follower: no parked gripper state — teleop will stall-home (~20 s)"))
        elif boot is not None and abs(boot - r.parked_raw) > PARK_TOLERANCE_TICKS:
            out.append(Finding("WARN", f"follower: gripper at {boot}, parked at {r.parked_raw} — "
                               "moved while off; teleop will stall-home (~20 s)"))
        elif boot is not None:
            out.append(Finding("OK", f"follower: gripper parked state matches (raw {boot})"))
    return out


def verdict(findings: list[Finding]) -> bool:
    return not any(f.level == "FAIL" for f in findings)


# ---------------------------------------------------------------- hardware

def _cu(port: str) -> str:
    return port.replace("/dev/tty.", "/dev/cu.")


def read_arm(port: str) -> ArmReadings:
    """Raw scservo probe: pings 1-6, reads Present_Position and voltage.
    Torque is never touched."""
    import scservo_sdk as scs

    arm = ArmReadings(port=port, port_exists=os.path.exists(_cu(port)))
    if not arm.port_exists:
        return arm
    ph = scs.PortHandler(_cu(port))
    try:
        if not ph.openPort() or not ph.setBaudRate(1_000_000):
            arm.open_error = "openPort/setBaudRate failed"
            return arm
        pk = scs.PacketHandler(0)
        for i in ARM_IDS:
            if pk.ping(ph, i)[1] != scs.COMM_SUCCESS:
                continue
            arm.alive.append(i)
            pos, r, _ = pk.read2ByteTxRx(ph, i, ADDR_POS)
            if r == scs.COMM_SUCCESS:
                arm.positions[i] = pos
            v, r, _ = pk.read1ByteTxRx(ph, i, ADDR_VOLTAGE)
            if r == scs.COMM_SUCCESS:
                arm.voltages[i] = v / 10.0
    except Exception as e:  # serial vanished mid-probe etc.
        arm.open_error = str(e).splitlines()[0][:100]
    finally:
        try:
            ph.closePort()
        except Exception:
            pass
    return arm


def _read_gripper_raw(port: str) -> int | None:
    import scservo_sdk as scs

    ph = scs.PortHandler(_cu(port))
    try:
        ph.openPort(); ph.setBaudRate(1_000_000)
        pos, r, _ = scs.PacketHandler(0).read2ByteTxRx(ph, 6, ADDR_POS)
        return pos if r == scs.COMM_SUCCESS else None
    finally:
        ph.closePort()


def collect_readings(cfg: dict, interactive: bool = False) -> Readings:
    from lerobot.utils.constants import HF_LEROBOT_CALIBRATION

    leader = read_arm(cfg["leader_port"])
    follower = read_arm(cfg["follower_port"])
    r = Readings(leader=leader, follower=follower)

    lcal = pathlib.Path(HF_LEROBOT_CALIBRATION) / "teleoperators" / "so_leader" / f"{cfg['leader_id']}.json"
    fcal = pathlib.Path(HF_LEROBOT_CALIBRATION) / "robots" / "fastgripper_follower" / f"{cfg['follower_id']}.json"
    state = fcal.with_name(f"{cfg['follower_id']}_gripper_state.json")
    if lcal.exists():
        r.leader_cal = json.loads(lcal.read_text())
    if fcal.exists():
        r.follower_cal = json.loads(fcal.read_text())
    if state.exists():
        r.parked_raw = json.loads(state.read_text()).get("parked_raw")

    if interactive and r.leader_cal and leader.port_exists and not leader.open_error:
        g = r.leader_cal["gripper"]
        input("  SQUEEZE the trigger fully and hold, then press ENTER... ")
        raw = _read_gripper_raw(cfg["leader_port"])
        r.trigger_squeezed_pct = trigger_pct(raw, g) if raw is not None else None
        input("  RELEASE the trigger fully, then press ENTER... ")
        raw = _read_gripper_raw(cfg["leader_port"])
        r.trigger_released_pct = trigger_pct(raw, g) if raw is not None else None
    return r


def run_preflight(cfg: dict, interactive: bool = False) -> bool:
    print(f"preflight: leader {cfg.get('leader_id')}@{cfg.get('leader_port')}  "
          f"follower {cfg.get('follower_id')}@{cfg.get('follower_port')}")
    findings = evaluate(collect_readings(cfg, interactive=interactive))
    for f in findings:
        print(f"  [{f.level:4}] {f.text}")
    ok = verdict(findings)
    print("preflight:", "GO" if ok else "NO-GO")
    return ok
