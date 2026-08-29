"""USB link diagnostics: `fastgripper usb soak` and `fastgripper usb watch`.

Both exist because of one failure class that is easy to blame on the
gripper and is never the gripper: the arm's USB serial device dropping off
the bus (macOS: "[Errno 6] Device not configured"; Linux: "Input/output
error"). See docs/troubleshooting/usb-serial-drops.md.

  soak   open the leader and follower ports at once and sync-read all six
         servos on each at 60 Hz for --seconds, no motion. Reports per-link
         success/failure and the exact moment a device vanished. Ports must
         be free (no teleop running).
  watch  poll /dev for the leader and follower nodes every 0.2 s and print a
         timestamped line each time one disappears or returns. Opens
         nothing, so run it in a second terminal DURING teleop.
"""

from __future__ import annotations

import os
import sys
import threading
import time


def _cu(port: str) -> str:
    return port.replace("/dev/tty.", "/dev/cu.")


def soak_links(ports: dict[str, str], seconds: float) -> dict[str, tuple[int, int, str]]:
    """Concurrent 60 Hz sync-read soak. Returns {name: (ok, fail, note)}."""
    import scservo_sdk as scs

    res: dict[str, tuple[int, int, str]] = {}

    def one(name: str, port: str) -> None:
        port = _cu(port)
        if not os.path.exists(port):
            res[name] = (0, 0, "ABSENT at start")
            return
        ph = scs.PortHandler(port)
        ok = fail = 0
        note = ""
        t0 = time.time()
        try:
            if not ph.openPort() or not ph.setBaudRate(1_000_000):
                res[name] = (0, 0, "cannot open (in use by another program?)")
                return
            pk = scs.PacketHandler(0)
            rd = scs.GroupSyncRead(ph, pk, 56, 2)
            for i in range(1, 7):
                rd.addParam(i)
            while time.time() - t0 < seconds:
                if rd.txRxPacket() == scs.COMM_SUCCESS:
                    ok += 1
                else:
                    fail += 1
                    if not os.path.exists(port):
                        note = f"VANISHED at {time.time() - t0:.1f}s"
                        break
                time.sleep(1 / 60)
        except Exception as e:
            note = f"EXC at {time.time() - t0:.1f}s: {str(e).splitlines()[0][:70]}"
        finally:
            try:
                ph.closePort()
            except Exception:
                pass
        res[name] = (ok, fail, note)

    threads = [threading.Thread(target=one, args=(n, p)) for n, p in ports.items()]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    return res


def classify(ok: int, fail: int, note: str) -> str:
    if note or ok == 0:
        return "FAIL"
    if fail > ok * 0.01:
        return "WARN"
    return "OK"


def cmd_soak(ports: dict[str, str], seconds: float) -> int:
    print(f"soaking {len(ports)} link(s) for {seconds:.0f}s at 60 Hz (no motion)...", flush=True)
    res = soak_links(ports, seconds)
    bad = 0
    for name, (ok, fail, note) in res.items():
        level = classify(ok, fail, note)
        bad += level == "FAIL"
        print(f"  [{level:4}] {name:>9}: ok={ok:5d} fail={fail:3d} {note}", flush=True)
    print("soak:", "PASS" if not bad else "FAIL — see docs/troubleshooting/usb-serial-drops.md", flush=True)
    return 0 if not bad else 1


def cmd_watch(ports: dict[str, str]) -> None:
    ports = {n: _cu(p) for n, p in ports.items()}
    state = {n: os.path.exists(p) for n, p in ports.items()}
    t0 = time.time()
    print("watching " + ", ".join(f"{n}={'up' if v else 'DOWN'}" for n, v in state.items())
          + "  (Ctrl-C to stop)", flush=True)
    events = 0
    try:
        while True:
            time.sleep(0.2)
            for n, p in ports.items():
                v = os.path.exists(p)
                if v != state[n]:
                    events += 1
                    print(f"{time.strftime('%H:%M:%S')} +{time.time() - t0:7.1f}s  {n}: "
                          f"{'RETURNED' if v else 'VANISHED'}", flush=True)
                    state[n] = v
    except KeyboardInterrupt:
        pass
    print(f"\nwatched {time.time() - t0:.0f}s, {events} drop/return events "
          f"({'clean' if not events else 'see docs/troubleshooting/usb-serial-drops.md'})", flush=True)
    sys.stdout.flush()
