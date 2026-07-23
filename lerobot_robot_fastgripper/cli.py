"""fastgripper — command-line front door for the FastGripper LeRobot plugin.

Subcommands:
    setup      one-time: save ports/ids and establish the gripper zero
    calibrate  arm calibration (wraps lerobot-calibrate for follower/leader)
    jog        keyboard jog + torque survey of the gripper
    teleop     teleoperate using the saved setup
    status     health check: servo, calibration, parked state

Ports/ids are stored once (by `setup`) in ~/.config/fastgripper/config.json,
so daily use is just `fastgripper teleop`.
"""

import argparse
import json
import os
import pathlib
import signal
import subprocess
import sys

CONFIG_PATH = pathlib.Path.home() / ".config" / "fastgripper" / "config.json"


def _run_child(cmd: list[str]) -> int:
    """Run a lerobot CLI as a child, letting IT own Ctrl-C.

    Ctrl-C hits the whole terminal group. If this wrapper dies on the same
    SIGINT, it disrupts the child's shutdown — which includes the ~2 s
    park-gripper-closed drive (found live: auto-park never ran under
    `fastgripper teleop`, only under bare lerobot-teleoperate). Ignore
    SIGINT here and wait for the child to finish its cleanup."""
    prev = signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        return subprocess.call(cmd)
    finally:
        signal.signal(signal.SIGINT, prev)


def _load_config(require: bool = True) -> dict:
    if not CONFIG_PATH.exists():
        if require:
            raise SystemExit("No fastgripper config found — run `fastgripper setup` first.")
        return {}
    return json.loads(CONFIG_PATH.read_text())


def _save_config(cfg: dict) -> None:
    CONFIG_PATH.parent.mkdir(parents=True, exist_ok=True)
    CONFIG_PATH.write_text(json.dumps(cfg, indent=2))


def cmd_setup(args) -> None:
    cfg = _load_config(require=False)
    for key in ("follower_port", "follower_id", "leader_port", "leader_id"):
        val = getattr(args, key)
        if val:
            cfg[key] = val
    missing = [k for k in ("follower_port", "follower_id") if not cfg.get(k)]
    if missing:
        raise SystemExit(f"Missing {missing} — pass --follower-port/--follower-id (and "
                         "--leader-port/--leader-id for teleop).")
    _save_config(cfg)
    print(f"Saved config to {CONFIG_PATH}: {cfg}")

    print(
        "\nEstablishing the gripper zero. The gripper must be FULLY CLOSED\n"
        "(factory grippers ship closed — the worm holds them there)."
    )
    resp = input("Is the gripper fully closed? [y = yes / g = guide me there / N = abort] ").strip().lower()
    if resp == "g":
        from . import jog

        if not jog.guided_close(cfg["follower_port"]):
            print("Setup not completed — re-run `fastgripper setup` when ready.")
            return
    elif resp != "y":
        print("Re-run `fastgripper setup` and choose 'g' to be walked to the closed stop.")
        return

    from .config_fastgripper_follower import FastGripperFollowerConfig
    from .fastgripper_follower import FastGripperFollower

    robot_cfg = FastGripperFollowerConfig(
        port=cfg["follower_port"], id=cfg["follower_id"], gripper_home_mode="assume_closed"
    )
    robot = FastGripperFollower(robot_cfg)
    robot.connect()
    robot.disconnect()  # parks + saves state: `auto` mode works from now on
    print("\nDone. Gripper zero established and parked-state saved.")
    print("Daily use is now just: fastgripper teleop")


def cmd_calibrate(args) -> None:
    cfg = _load_config()
    if args.leader:
        cmd = ["lerobot-calibrate",
               "--teleop.type=so101_leader",
               f"--teleop.port={cfg['leader_port']}",
               f"--teleop.id={cfg['leader_id']}"]
    else:
        cmd = ["lerobot-calibrate",
               "--robot.type=fastgripper_follower",
               f"--robot.port={cfg['follower_port']}",
               f"--robot.id={cfg['follower_id']}"]
    raise SystemExit(_run_child(cmd))


def cmd_jog(args) -> None:
    cfg = _load_config()
    from . import jog

    sys.argv = ["fastgripper-jog", "--port", cfg["follower_port"]]
    if args.status:
        sys.argv.append("--status")
    jog.main()


def cmd_teleop(args, extra: list[str]) -> None:
    cfg = _load_config()
    for key in ("leader_port", "leader_id"):
        if not cfg.get(key):
            raise SystemExit(f"Config missing {key} — re-run `fastgripper setup` with "
                             "--leader-port/--leader-id.")
    cmd = ["lerobot-teleoperate",
           "--robot.type=fastgripper_follower",
           f"--robot.port={cfg['follower_port']}",
           f"--robot.id={cfg['follower_id']}",
           "--teleop.type=so101_leader",
           f"--teleop.port={cfg['leader_port']}",
           f"--teleop.id={cfg['leader_id']}",
           *extra]
    raise SystemExit(_run_child(cmd))


def cmd_status(args) -> None:
    cfg = _load_config()
    print(f"config: {cfg}")
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    bus = FeetechMotorsBus(port=cfg["follower_port"],
                           motors={"gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100)})
    try:
        bus._connect(handshake=False)
    except Exception as e:
        raise SystemExit(f"Cannot open {cfg['follower_port']}: {e}")
    bus.set_baudrate(1_000_000)
    ph, po = bus.packet_handler, bus.port_handler
    import scservo_sdk as scs

    ok = [mid for mid in range(1, 7) if ph.ping(po, mid)[1] == scs.COMM_SUCCESS]
    print(f"motors responding: {ok or 'NONE'}")
    if 6 in ok:
        T = bus.model_ctrl_table["sts3215"]
        for reg in ("Present_Position", "Present_Temperature", "Phase", "Operating_Mode"):
            a, ln = T[reg]
            v = (ph.read1ByteTxRx if ln == 1 else ph.read2ByteTxRx)(po, 6, a)[0]
            print(f"  gripper {reg} = {v}")
    po.closePort()

    from lerobot.utils.constants import HF_LEROBOT_CALIBRATION

    cal = HF_LEROBOT_CALIBRATION / "robots" / "fastgripper_follower" / f"{cfg['follower_id']}.json"
    state = cal.with_name(f"{cfg['follower_id']}_gripper_state.json")
    print(f"arm calibration: {'OK' if cal.exists() else 'MISSING (run fastgripper calibrate)'}")
    print(f"parked state:    {'OK — auto mode ready' if state.exists() else 'MISSING (run fastgripper setup)'}")


def main() -> None:
    parser = argparse.ArgumentParser(prog="fastgripper", description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("setup", help="one-time setup: save ports and establish gripper zero")
    p.add_argument("--follower-port")
    p.add_argument("--follower-id", default="follower_1")
    p.add_argument("--leader-port")
    p.add_argument("--leader-id", default="leader_1")

    p = sub.add_parser("calibrate", help="arm calibration (follower by default)")
    p.add_argument("--leader", action="store_true", help="calibrate the leader instead")

    p = sub.add_parser("jog", help="keyboard jog + torque survey")
    p.add_argument("--status", action="store_true", help="one-line health probe, no jogging")

    sub.add_parser("teleop", help="teleoperate (extra lerobot-teleoperate args pass through)")

    sub.add_parser("status", help="servo/calibration/state health check")

    args, extra = parser.parse_known_args()
    if args.cmd == "setup":
        cmd_setup(args)
    elif args.cmd == "calibrate":
        cmd_calibrate(args)
    elif args.cmd == "jog":
        cmd_jog(args)
    elif args.cmd == "teleop":
        cmd_teleop(args, extra)
    elif args.cmd == "status":
        cmd_status(args)


if __name__ == "__main__":
    main()
