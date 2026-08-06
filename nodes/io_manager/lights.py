"""Drive the lights ESP32 over UART, so the hull colour matches the vessel status.

The Njord rules require the boat to say what it is doing in a colour a marshal can
read from another vessel:

    RED             kill switch pulled, propulsion disabled
    YELLOW/ORANGE   remote operation - a human is steering
    GREEN           autonomous operation

That is three colours for the five states `status.py` can be in, so two more are
chosen here rather than left to look like a fault:

    STANDBY         breathing white - the firmware's own idle profile. Reads as
                    "powered, nobody driving", which is what it means.
    OUT_OF_CONTROL  4 Hz red strobe. Deliberately NOT solid red: solid red is the
                    rules' promise that propulsion is *disabled*, and a boat
                    nobody is steering with live thrusters is the opposite of
                    that. Anything that made those two look alike would be a
                    safety-relevant lie.

The link
--------
`ligmax-subsystems/esp32s/lights_esp/lights_esp.ino` listens on its Serial2 at
115200 8N1 for one-line commands. This uses three of them:

    M<n>   set mode 0..8   -> "OK M<n>"
    P      ping            -> "PONG"
    S      status          -> "STATUS M.. BASE.. B.. COVERS.. L.. R.. UP.."

Wiring, from the sketch's own pin map (`:49-50`, `:74-75`): ESP RX <- Pi TX on
BCM 14 (header pin 8), ESP TX -> Pi RX on BCM 15 (header pin 10). That is the Pi's
primary UART, so it needs `enable_uart=1` and the console off - see the module
docstring in `emergency_stop.py` for why nothing here fails hard if it is not
there.

Design rules, because this is imported by the node that drives actuators:

  * **Never blocks the caller.** `set_status()` writes at most a few bytes to a
    serial port opened with a write timeout, off a worker thread. A wedged ESP32
    or an unplugged cable must not cost the MAVLink loop its 1 Hz heartbeat.
  * **Never raises.** A missing port, a missing `pyserial`, a cable pulled
    mid-run: every one degrades to a logged no-op and `available = False`, and
    the telemetry says so. The dashboard then shows the hull colour as unknown
    instead of asserting a green light that is not lit.
  * **Idempotent and re-asserting.** The mode is re-sent every RESEND_PERIOD
    even when nothing has changed. The ESP32 reverts to its default profile
    after 15 s of silence (`ENABLE_LINK_FALLBACK`, `:526-531`), so a hull that
    has quietly gone back to breathing white while the boat is under way is
    exactly what the re-send prevents.

The firmware is autonomous by design and animates whether or not the Pi ever says
anything, so losing this link never leaves the hull dark - it leaves it showing
the idle profile, which is wrong but not invisible.
"""

import logging
import os
import threading
import time

log = logging.getLogger("io_manager.lights")

# Serial2 on the ESP32 side, reached over the Pi's GPIO 14/15 UART.
#
# `/dev/serial0` is NOT the thing to point at on a Pi 5. There, serial0 is an
# alias the firmware resolves at boot, and if `uart0` is disabled in
# `config.txt` it resolves to `/dev/ttyAMA10` - the 3-pin *debug* connector,
# which is a different set of physical pins and normally carries the kernel
# console. Writing `M0\n` there is silently accepted and never reaches the
# ESP32, which is exactly the failure this constant used to cause: the port
# opened, every write "succeeded", `available` was True, and the hull stayed on
# the firmware's idle profile.
#
# So name the device explicitly, and treat the debug UART as unusable
# (`_DEBUG_UART`) rather than as a port that happens not to answer.
PORT = os.environ.get("LIGMAX_LIGHTS_PORT", "/dev/ttyAMA0")

# GPIO 14/15 needs `enable_uart=1` (and `dtparam=uart0=on`) in
# `/boot/firmware/config.txt` plus a reboot. Without it /dev/ttyAMA0 does not
# exist at all, so a missing port here means the boot config, not the cable.
_DEBUG_UART = "/dev/ttyAMA10"
BAUD = int(os.environ.get("LIGMAX_LIGHTS_BAUD", "115200"))

# Well inside the firmware's 15 s fallback (`lights_esp.ino:131`), so the hull
# never reverts to the idle profile while this node is alive and healthy.
RESEND_PERIOD = float(os.environ.get("LIGMAX_LIGHTS_RESEND_S", "4.0"))
# A dropped write is worth retrying sooner than the ordinary re-send.
RETRY_PERIOD = 1.0
OPEN_RETRY_PERIOD = 5.0
WRITE_TIMEOUT = 0.25

# --- The mapping. This is the authoritative copy. ---------------------------
#
# Mirrored in `ligmax-server/tools/sim_boat.py` (LIGHT_MODES/LIGHT_COLOURS) so the
# simulator can drive the dashboard's cross-check, and in
# `ligmax-server/web/js/status.js` as the colour the dashboard *expects*. The
# dashboard compares the two and shouts if they differ; if they ever do, this file
# is right and the others are stale.
#
# Mode numbers are the `Mode` enum in lights_esp.ino:137-148.
MODE_GREEN = 0
MODE_YELLOW = 1
MODE_RED = 2
MODE_F1_FOG = 5  # 4 Hz red strobe
MODE_BREATHING = 7

STATUS_MODES = {
    "AUTONOMOUS": MODE_GREEN,
    "REMOTE": MODE_YELLOW,
    "KILLED": MODE_RED,
    "OUT_OF_CONTROL": MODE_F1_FOG,
    "STANDBY": MODE_BREATHING,
}

# What the operator's dashboard is told is showing. Names, not mode numbers,
# because "red-strobe" and "red" being different is the whole point and two
# integers side by side would not make that obvious.
STATUS_COLOURS = {
    "AUTONOMOUS": "green",
    "REMOTE": "yellow",
    "KILLED": "red",
    "OUT_OF_CONTROL": "red-strobe",
    "STANDBY": "white",
}

# Where the hull goes if this node cannot work out what the boat is doing. Not
# green, and not the firmware's cheerful idle profile: an unknown state is closer
# to out-of-control than to anything else.
FALLBACK_STATUS = "OUT_OF_CONTROL"

try:
    import serial  # pyserial
except ImportError:  # not on the Pi, or the library is not installed yet
    serial = None


class _WrongPort(Exception):
    """The configured port is not the GPIO 14/15 UART. A config fault, not a cable
    fault - kept as its own type so `_open()` can log it once instead of every
    OPEN_RETRY_PERIOD."""


class Lights:
    """The hull's signal lights. `set_status()` is the whole interface.

    Owns one worker thread. Writes are queued as "the latest wanted mode", never
    accumulated: if the status changes three times between two writes, the light
    goes to the third colour and the first two never existed as far as the hull is
    concerned. That is right for a state indicator and wrong for a command queue,
    which is why this is not one.
    """

    def __init__(self, port=PORT, baud=BAUD):
        self.port_name = port
        self.baud = baud

        self._serial = None
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._closed = False

        self._wanted = None  # status name, or None until someone says
        self._sent = None  # the mode number the ESP32 has acked
        self._sent_status = None
        self._last_write = 0.0
        self._last_open_attempt = 0.0
        self._acks = 0
        self._write_errors = 0
        self._last_error = None
        self._last_ack_at = 0.0

        if serial is None:
            log.warning(
                "pyserial is not installed - the hull lights are a no-op "
                "(pip install pyserial). The ESP32 keeps its idle profile."
            )

        self._thread = threading.Thread(target=self._run, daemon=True, name="lights")
        self._thread.start()

    # -- public API ---------------------------------------------------------

    @property
    def available(self):
        """True when there is a real serial port behind this object."""
        return self._serial is not None

    def set_status(self, status):
        """Ask the hull to show the colour for `status`. Never blocks or raises.

        Unknown status names fall back to FALLBACK_STATUS rather than being
        ignored, because "we do not know what the boat is doing" is itself a thing
        the lights should say.
        """
        if self._closed:
            return
        if status not in STATUS_MODES:
            if status is not None:
                log.warning("unknown status %r for the lights, showing %s",
                            status, FALLBACK_STATUS)
            status = FALLBACK_STATUS
        with self._lock:
            changed = status != self._wanted
            self._wanted = status
        if changed:
            log.info("hull lights -> %s (%s)", STATUS_COLOURS[status], status)
            self._wake.set()

    def telemetry(self):
        """The `telemetry.lights` block, which the dashboard cross-checks.

        `colour` is what the ESP32 has *acked*, not what we wanted - the point of
        reporting it at all is to catch the case where the two differ.
        """
        with self._lock:
            wanted, sent_status, mode = self._wanted, self._sent_status, self._sent
        now = time.monotonic()
        block = {
            "link": self.available and bool(self._last_ack_at)
            and (now - self._last_ack_at) < (RESEND_PERIOD * 3),
            "acks": self._acks,
        }
        if sent_status is not None:
            block["colour"] = STATUS_COLOURS[sent_status]
            block["for_status"] = sent_status
        if mode is not None:
            block["mode"] = mode
        # Only worth reporting while it is actually true, so it does not sit in
        # the panel as a permanent field nobody reads.
        if wanted is not None and wanted != sent_status:
            block["pending"] = STATUS_COLOURS[wanted]
        if self._write_errors:
            block["errors"] = self._write_errors
        if self._last_error:
            block["last_error"] = str(self._last_error)[:120]
        return block

    def close(self):
        """Stop the worker and release the port.

        The lights are deliberately left as they are. The firmware reverts to its
        idle profile after 15 s on its own, and driving the hull to some final
        colour on the way out would be asserting a state nothing is maintaining.
        """
        if self._closed:
            return
        self._closed = True
        self._wake.set()
        self._thread.join(1.5)
        with self._lock:
            port, self._serial = self._serial, None
        if port is not None:
            try:
                port.close()
            except Exception:  # noqa: BLE001 - closing a dead port is not news
                pass

    # -- worker -------------------------------------------------------------

    def _run(self):
        while not self._closed:
            self._wake.clear()
            try:
                self._tick()
            except Exception as exc:  # noqa: BLE001 - the loop must not die
                self._last_error = exc
                log.error("lights worker: %s", exc)
            # Wake early on a status change, otherwise re-assert on the timer.
            self._wake.wait(0.5)

    def _tick(self):
        with self._lock:
            wanted = self._wanted
        if wanted is None:
            return

        if self._serial is None:
            self._open()
            if self._serial is None:
                return

        mode = STATUS_MODES[wanted]
        now = time.monotonic()
        due = (
            mode != self._sent
            or (now - self._last_write) >= RESEND_PERIOD
        )
        if not due:
            self._drain()
            return

        if not self._write(f"M{mode}\n"):
            # Drop the port so the next tick reopens it: on a USB-serial adapter a
            # failed write usually means the device went away.
            self._reset_port()
            self._sent = None
            self._last_write = now - RESEND_PERIOD + RETRY_PERIOD
            return

        self._last_write = now
        # Optimistic: the mode is recorded as sent, and `_drain()` confirms it from
        # the "OK M<n>" the firmware echoes. Without that, `link` would stay false
        # on a working one-way cable and hide the real fault, which is that the
        # colour cannot be verified.
        self._sent = mode
        self._sent_status = wanted
        self._drain()

    def _open(self):
        if serial is None:
            return
        now = time.monotonic()
        if now - self._last_open_attempt < OPEN_RETRY_PERIOD:
            return
        self._last_open_attempt = now

        # The debug connector opens and accepts writes perfectly happily, so
        # without this the failure looks like a working link driving a hull that
        # never changes colour. Refuse it by name instead.
        if os.path.realpath(self.port_name) == _DEBUG_UART:
            if not isinstance(self._last_error, _WrongPort):
                self._last_error = _WrongPort(
                    f"{self.port_name} resolves to {_DEBUG_UART}, the Pi 5 debug "
                    f"connector, not GPIO 14/15"
                )
                log.error(
                    "refusing to drive the lights over %s: it resolves to %s, "
                    "the 3-pin debug connector, not the GPIO 14/15 UART. Set "
                    "enable_uart=1 in /boot/firmware/config.txt, reboot, and "
                    "point LIGMAX_LIGHTS_PORT at /dev/ttyAMA0.",
                    self.port_name,
                    _DEBUG_UART,
                )
            return
        try:
            port = serial.Serial(
                self.port_name,
                self.baud,
                timeout=0,  # non-blocking reads; _drain never waits
                write_timeout=WRITE_TIMEOUT,
            )
        except Exception as exc:  # noqa: BLE001 - no port, busy, no permission
            if str(exc) != str(self._last_error):
                log.error(
                    "cannot open the lights port %s: %s - the hull keeps its "
                    "idle profile",
                    self.port_name,
                    exc,
                )
            self._last_error = exc
            return
        with self._lock:
            self._serial = port
        self._sent = None  # re-assert the mode on a fresh port
        self._last_error = None
        log.info("lights ESP32 on %s @ %s baud", self.port_name, self.baud)

    def _write(self, line):
        port = self._serial
        if port is None:
            return False
        try:
            port.write(line.encode("ascii"))
            return True
        except Exception as exc:  # noqa: BLE001 - a dead cable is not fatal
            self._write_errors += 1
            self._last_error = exc
            log.warning("lights write failed: %s", exc)
            return False

    def _drain(self):
        """Read whatever the ESP32 has said. Never blocks - the port has timeout=0.

        Only an `OK M<n>` that matches what we sent counts as an ack. The firmware
        is explicit that noise on a floating RX line must not be taken for the
        peer being present (`lights_esp.ino:390-395`), and the same applies here in
        the other direction.
        """
        port = self._serial
        if port is None:
            return
        try:
            waiting = port.in_waiting
            if not waiting:
                return
            data = port.read(min(waiting, 512)).decode("ascii", "replace")
        except Exception as exc:  # noqa: BLE001
            self._last_error = exc
            self._reset_port()
            return

        for line in data.replace("\r", "\n").split("\n"):
            line = line.strip()
            if not line:
                continue
            if line.startswith("OK M"):
                try:
                    acked = int(line[4:])
                except ValueError:
                    continue
                if acked == self._sent:
                    self._acks += 1
                    self._last_ack_at = time.monotonic()
            elif line == "PONG":
                self._last_ack_at = time.monotonic()
            elif line.startswith("ERR"):
                log.warning("lights ESP32 refused a command: %s", line)

    def _reset_port(self):
        with self._lock:
            port, self._serial = self._serial, None
        if port is not None:
            try:
                port.close()
            except Exception:  # noqa: BLE001
                pass


if __name__ == "__main__":
    # Bench check on the Pi:  python -m nodes.io_manager.lights
    # Walks the hull through all five colours, three seconds each. Watch the
    # strips; every state the boat can be in should be visibly distinct.
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s %(message)s")
    lights = Lights()
    try:
        for name in ("STANDBY", "AUTONOMOUS", "REMOTE", "OUT_OF_CONTROL", "KILLED"):
            lights.set_status(name)
            print(f"{name:<15} -> {STATUS_COLOURS[name]} (M{STATUS_MODES[name]})")
            time.sleep(3.0)
            print(f"                  {lights.telemetry()}")
    finally:
        lights.close()
