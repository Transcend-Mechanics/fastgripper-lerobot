# Teleop dies with "Device not configured" / "no status packet" — it's the USB link, not the gripper

**Read this if:** teleop runs for seconds-to-minutes and then crashes, the
crash names a *leader* or *follower* serial port, and the servos all answer
fine a moment later. This is a USB power/cabling problem between the
computer and the arm's driver board. It is **not** the gripper, not the
servos, and not calibration — but it looks like all three, which is why this
page exists.

Two days of bench time went into pinning this down (2026‑08‑27/29). Every
sign, test, and fix below was confirmed live.

## The signs

Any of these, alone or together:

| What you see | What it means |
| --- | --- |
| `serial.serialutil.SerialException: read failed: [Errno 6] Device not configured` (macOS) or `[Errno 5] Input/output error` (Linux) | The OS **removed the USB device** mid-session. The file descriptor is dead; nothing in software can read it again until it re-enumerates. |
| `termios.error: (6, 'Device not configured')` on exit | Same thing, caught during shutdown. |
| `ConnectionError: Failed to sync read 'Present_Position' … There is no status packet!` | A read timed out. A single one is a normal glitch; a burst of them right before a `Device not configured` is the device going away. |
| `[TxRxResult] Port is in use!` after a reconnect | The serial SDK's busy flag was left set by an interrupted transaction (the plugin now clears it). |
| `motor check failed … Missing motor IDs: - 5` at connect, when `fastgripper preflight` showed all six a second earlier | One dropped ping on a marginal link (the plugin now retries the handshake). |
| `recovered Present_Position read after N retries` several times a minute | The link is **degrading**. Expect a drop soon. Zero or one of these per session is normal. |
| A serial port that **appears and disappears every 1–3 s** with nothing using it (`fastgripper usb watch`) | A USB hub port tripping its over-current protection and retrying. Classic. |
| `system_profiler SPUSBDataType` (macOS) listing the **same serial number two or three times** | Ghost entries from repeated failed re-enumerations. The hub's state is scrambled. |
| Both rigs / both arms dying **in the same second** | They share a hub, and the hub dropped. |
| Port name changed after a "crash" (`/dev/tty.usbmodem…` suffix different) | Re-enumeration. Same root cause. |

The tell that separates this from a real servo problem: after the crash,
`fastgripper status` or `fastgripper preflight` shows **every servo
answering at ~12 V**. Real servo faults don't heal themselves in five
seconds; USB drops do.

## The cause

The SO‑101's driver board (a CH343 USB‑serial chip) is **powered from USB**.
Its 5 V rail also feeds the board's logic and, if the arm's 12 V supply is
off, the servos try to draw through it too. When the USB port can't hold 5 V
under that load, the chip browns out and re-enumerates. Sources of that, in
order of how often they were the culprit here:

1. **A bus-powered hub** (slim USB‑C "dongle" hubs especially). They share
   ~500 mA across every port. Two driver boards plus a CAN adapter is
   ~430 mA nominal; a follower's servos moving adds spikes on top. The hub
   sags, and the *leader* on the same hub is what drops (it draws the least
   and has the least margin).
2. **A damaged hub.** A shorted driver board plugged into a hub can cook the
   hub's port switches. Afterwards it trips on a single idle board (the 2 Hz
   flap) and shows ghost entries. It will never recover — retire it.
3. **A marginal USB cable.** Works plugged straight into the computer (stiff
   5 V), fails on a hub. Only shows up when you move things around.
4. **Arm 12 V supply off or barrel plug out.** Servos pull through USB;
   even a good port can't hold it.
5. **Charge-only USB‑C cable.** Nothing enumerates at all.

Nothing on this list is the gripper. The worm gripper's higher holding
torque does make the *follower* draw more current, which is why it is the
device most likely to sag a hub — but it does that to any arm on the hub,
and the fix is the same.

## Verify it in five minutes

All commands are no-motion and safe with the arms powered.

**1. Preflight (ports, servos, voltages, calibration, park state):**

```sh
fastgripper preflight
```

Every servo answering at ~12 V after a "crash" = the crash was the link.
A bus voltage under 6 V = the arm's 12 V supply is off (fix that first).

**2. Watch the ports with nothing running** (open a second terminal):

```sh
fastgripper usb watch
```

Leave it 30 s. **Any** `VANISHED`/`RETURNED` line with no program using the
port is a hub or cable fault — software cannot cause a device to leave the
USB bus. A steady 1–3 s rhythm is a hub port over-current trip.

**3. Soak the links at teleop rate, no motion:**

```sh
fastgripper usb soak --seconds 30
```

Reads both arms at 60 Hz concurrently. Pass = every link `OK`, zero
failures. A link that `VANISHED at 0.3s` under pure reads is a power
problem, not a bus-traffic problem.

**4. Isolate.** Move ONE thing at a time, re-run step 2 for 20 s each:

- Plug the flapping arm **straight into the computer** (no hub). Quiet →
  the hub (or the hub's power) is the problem. Still flapping → its cable or
  its 12 V supply.
- Swap the flapping arm's USB cable for one from an arm that is stable.
  Quiet → bin that cable.
- Put *only* the flapping arm on the hub. Still flapping → the hub is
  damaged or too weak for a driver board; try another hub.

**5. Confirm during real use.** Run `fastgripper usb watch` in a spare
terminal for a whole teleop session. Pass = `0 drop/return events`. (The
YAM/i2rt CAN adapter shows one 0.2 s blip at the moment its session opens —
that is a deliberate reset, not a drop.)

## The fixes

In order of effectiveness:

1. **Powered hub, or a real USB 3 hub with its own power input.** Not a slim
   bus-powered dongle. This removes the cause.
2. **One driver board per hub port budget.** If you must use a bus-powered
   hub: never put a *follower* (moving servos) on the same hub as a
   *leader*. Followers direct into the computer; leaders and low-draw
   devices (CAN adapters) on the hub.
3. **Retire any hub that shows ghost entries or a 2 Hz flap on a single
   idle board.** It is damaged; it will keep costing sessions.
4. **Replace any cable that fails on a hub but works direct.**
5. **Arm 12 V on before opening the port**, always.

What the plugin does for you (from the `fix/ctrl-c-and-preflight` change set,
2026‑08‑29, onward) so a glitch is a warning instead of a dead session:

- `fastgripper teleop` runs `preflight` first and refuses to start on a
  missing port, a silent servo, a low bus voltage, or a stale turn counter.
- Leader reads retry transient timeouts; if the leader's device vanishes,
  the plugin **reopens the port by path for up to 6 s** and continues
  (`serial device lost -- reopening … reconnected`). The follower holds
  its last goal meanwhile.
- The follower's reads retry the same way, and the connect handshake
  retries a missed ping before declaring a motor missing.
- A missing or mismatched gripper park state **stall-homes automatically**
  instead of refusing to start, so an unclean exit no longer costs a
  manual `jog` + `setup` loop.
- Ctrl‑C works and always ends with the gripper parked and its state saved.

These make a marginal link survivable. They do not make a bad hub good — if
you see `reopening` more than once a session, go back to *Verify*.

## What it is not

- **Not calibration.** Calibration cannot make a device leave the USB bus.
  Re-calibrating after a drop changes nothing except, sometimes, making
  things worse under time pressure (a trigger swept past its stop, say).
- **Not the gripper's park state.** "Not where it was parked" after one of
  these crashes is a *consequence* (the session died before parking), not
  the cause. The plugin now self-heals it.
- **Not the servos.** A latched servo error after a drop (`Disconnect hit a
  latched motor error`) is the same consequence; a power-cycle of the arm
  clears it, and the plugin clears it itself on the next connect.

## Still stuck

Open an issue with the output of `fastgripper preflight`, the closing
line of `fastgripper usb watch` from a session, and
`system_profiler SPUSBDataType` (macOS) / `lsusb -t` (Linux):
[github.com/Transcend-Mechanics/fastgripper-lerobot/issues](https://github.com/Transcend-Mechanics/fastgripper-lerobot/issues)
