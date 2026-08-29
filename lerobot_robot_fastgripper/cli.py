"""fastgripper — command-line front door for the FastGripper LeRobot plugin.

Subcommands:
    setup      one-time: save ports/ids and establish the gripper zero
    calibrate  arm calibration (wraps lerobot-calibrate for follower/leader)
    jog        keyboard jog + torque survey of the gripper
    teleop     teleoperate using the saved setup (runs preflight first)
    status     health check: servo, calibration, parked state
    preflight  go/no-go: ports, servos, voltages, stale turn counters,
               calibration files, parked state, trigger calibration
    usb        link diagnostics: `usb soak` (60 Hz read soak, no motion)
               and `usb watch` (log device drops during teleop)

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
        # SIG_IGN is inherited across exec, and Python only installs its
        # KeyboardInterrupt handler when SIGINT is NOT already ignored -- so a
        # child spawned after the line above would be immune to Ctrl-C forever
        # (seen live: teleop could not be stopped, park never ran). Restore the
        # default disposition in the child before it execs.
        proc = subprocess.Popen(
            cmd, preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL)
        )
        return proc.wait()
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


TARGET_KEYS = ("follower_port", "follower_id", "leader_port", "leader_id")


def _add_target_args(p, leader: bool = True) -> None:
    p.add_argument("--follower-port", help="target this follower port (overrides saved config)")
    p.add_argument("--follower-id", help="target this follower id (overrides saved config)")
    if leader:
        p.add_argument("--leader-port", help="target this leader port (overrides saved config)")
        p.add_argument("--leader-id", help="target this leader id (overrides saved config)")


def _resolve_target(args) -> dict:
    """Saved config overlaid with any --follower-*/--leader-* flags for THIS
    invocation only (nothing is written back — only `setup` persists)."""
    cfg = _load_config(require=False)
    for key in TARGET_KEYS:
        val = getattr(args, key, None)
        if val:
            cfg[key] = val
    missing = [k for k in ("follower_port", "follower_id") if not cfg.get(k)]
    if missing:
        raise SystemExit(f"Missing {missing} — run `fastgripper setup` or pass "
                         "--follower-port/--follower-id.")
    return cfg


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
    cfg = _resolve_target(args)
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
    cfg = _resolve_target(args)
    from . import jog

    sys.argv = ["fastgripper-jog", "--port", cfg["follower_port"]]
    if args.status:
        sys.argv.append("--status")
    jog.main()


def cmd_teleop(args, extra: list[str]) -> None:
    cfg = _resolve_target(args)
    for key in ("leader_port", "leader_id"):
        if not cfg.get(key):
            raise SystemExit(f"Config missing {key} — re-run `fastgripper setup` with "
                             "--leader-port/--leader-id (or pass it to teleop directly).")
    if not args.skip_preflight:
        from .preflight import run_preflight

        if not run_preflight(cfg, interactive=False):
            raise SystemExit("preflight FAILED — fix the items above, or pass --skip-preflight.")
    cmd = ["lerobot-teleoperate",
           "--robot.type=fastgripper_follower",
           f"--robot.port={cfg['follower_port']}",
           f"--robot.id={cfg['follower_id']}",
           "--teleop.type=fastgripper_leader",   # so101_leader + read retries
           f"--teleop.port={cfg['leader_port']}",
           f"--teleop.id={cfg['leader_id']}",
           *extra]
    raise SystemExit(_run_child(cmd))


def cmd_usb(args) -> None:
    cfg = _resolve_target(args)
    from . import usb

    ports = {"follower": cfg["follower_port"]}
    if cfg.get("leader_port"):
        ports["leader"] = cfg["leader_port"]
    if args.usb_cmd == "soak":
        raise SystemExit(usb.cmd_soak(ports, args.seconds))
    usb.cmd_watch(ports)


def cmd_preflight(args) -> None:
    cfg = _resolve_target(args)
    from .preflight import run_preflight

    ok = run_preflight(cfg, interactive=args.trigger)
    raise SystemExit(0 if ok else 1)


def cmd_status(args) -> None:
    cfg = _resolve_target(args)
    print(f"target: {cfg}")
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
    p.add_argument("--follower-id")   # no defaults: a bare `setup` must NOT silently
    p.add_argument("--leader-port")   # rename the saved arms (it did, once)
    p.add_argument("--leader-id")

    p = sub.add_parser("calibrate", help="arm calibration (follower by default)")
    p.add_argument("--leader", action="store_true", help="calibrate the leader instead")
    _add_target_args(p)

    p = sub.add_parser("jog", help="keyboard jog + torque survey")
    p.add_argument("--status", action="store_true", help="one-line health probe, no jogging")
    _add_target_args(p, leader=False)

    p = sub.add_parser("teleop", help="teleoperate (extra lerobot-teleoperate args pass through)")
    p.add_argument("--skip-preflight", action="store_true", help="launch without the preflight checks")
    _add_target_args(p)

    p = sub.add_parser("status", help="servo/calibration/state health check")
    _add_target_args(p)

    p = sub.add_parser("usb", help="USB link diagnostics: soak (read both arms at 60 Hz) / watch (log device drops)")
    usub = p.add_subparsers(dest="usb_cmd", required=True)
    ps = usub.add_parser("soak", help="60 Hz sync-read soak on leader+follower, no motion (ports must be free)")
    ps.add_argument("--seconds", type=float, default=30)
    _add_target_args(ps)
    pw = usub.add_parser("watch", help="log every serial-device drop/return; run DURING teleop")
    _add_target_args(pw)

    p = sub.add_parser("preflight", help="go/no-go check of ports, servos, calibration, park, trigger")
    p.add_argument("--trigger", action="store_true",
                   help="also verify the trigger reaches 0%% squeezed / 100%% released (prompts you)")
    _add_target_args(p)

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
    elif args.cmd == "preflight":
        cmd_preflight(args)
    elif args.cmd == "usb":
        cmd_usb(args)


if __name__ == "__main__":
    main()
