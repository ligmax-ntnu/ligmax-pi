"""Drive the lights ESP32 over UART, so the hull colour matches the vessel status.

The Njord rules require the boat to say what it is doing in a colour a marshal can
read from another vessel:

    RED             kill switch pulled, propulsion disabled
    YELLOW/ORANGE   remote operation - a human is steering
    GREEN           autonomous operation

That is three colours for the five states `status.py` can be in, so two more are
chosen here rather than left to look like a fault:

    STANDBY         breathing white. Reads as "powered, nobody driving", which is
                    what it means.
    OUT_OF_CONTROL  4 Hz red strobe. Deliberately NOT solid red: solid red is the
                    rules' promise that propulsion is *disabled*, and a boat
                    nobody is steering with live thrusters is the opposite of
                    that. Anything that made those two look alike would be a
                    safety-relevant lie.

The link
--------
The ESP32 is now a dumb pixel driver. It listens at 115200 8N1 for exactly one
command and answers nothing at all:

    DATA <NUM_LEDS*6 hex chars>\\n     set the whole strip, one RRGGBB per LED

The firmware used to own the animations and take `M<n>` mode commands, acking
each with `OK M<n>`. It no longer does - `M0\\n` is accepted by the port, reaches
the ESP32, and does nothing. That is why every frame is now rendered here: the
strobe and the breathe are Python, not firmware, and STANDBY is our own white
ramp rather than the firmware's idle profile.

Two consequences worth stating, because they are not recoverable in software:

  * **Nothing acks.** There is no return path to tell us the hull is really lit,
    so `telemetry.lights.link` can only mean "the port is open and frames are
    going out", never "the colour is confirmed". It is reported as such.
  * **Silence is now darkness.** The old firmware animated on its own and fell
    back to an idle profile after 15 s, so a dead Pi left the hull *wrong but
    visible*. A dumb driver holds the last frame it got instead, so if this
    thread dies the hull freezes on whatever colour it was last told. Holding a
    stale colour is worse than reverting, so `close()` blanks the strip.

Wiring, from the sketch's pin map: ESP RX <- Pi TX on BCM 14 (header pin 8),
ESP TX -> Pi RX on BCM 15 (header pin 10). That is the Pi's primary UART, so it
needs `enable_uart=1` and the console off - see the module docstring in
`emergency_stop.py` for why nothing here fails hard if it is not there.

Design rules, because this is imported by the node that drives actuators:

  * **Never blocks the caller.** `set_status()` sets one field and pokes an
    Event. All serial I/O is on a worker thread against a port with a write
    timeout. A wedged ESP32 or an unplugged cable must not cost the MAVLink loop
    its 1 Hz heartbeat.
  * **Never raises.** A missing port, a missing `pyserial`, a cable pulled
    mid-run: every one degrades to a logged no-op and `available = False`, and
    the telemetry says so. The dashboard then shows the hull colour as unknown
    instead of asserting a green light that is not lit.
  * **Re-asserting.** A frame is re-sent every KEEPALIVE_PERIOD even when the
    picture has not changed, so a strip that missed a byte or was powered up
    late catches the next one instead of staying dark until the status happens
    to change.
"""

import logging
import math
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
# console. Writing a frame there is silently accepted and never reaches the
# ESP32: the port opens, every write "succeeds", `available` is True, and the
# hull stays dark.
#
# So name the device explicitly, and treat the debug UART as unusable
# (`_DEBUG_UART`) rather than as a port that happens not to answer.
PORT = os.environ.get("LIGMAX_LIGHTS_PORT", "/dev/ttyAMA0")

# GPIO 14/15 needs `enable_uart=1` (and `dtparam=uart0=on`) in
# `/boot/firmware/config.txt` plus a reboot. Without it /dev/ttyAMA0 does not
# exist at all, so a missing port here means the boot config, not the cable.
#
# This must never be equal to PORT. Setting them the same makes `_open()` refuse
# the one port that works, which looks exactly like a dead cable.
_DEBUG_UART = "/dev/ttyAMA10"
BAUD = int(os.environ.get("LIGMAX_LIGHTS_BAUD", "115200"))

# The strip the firmware expects. A frame is `DATA ` + NUM_LEDS*6 hex + `\n`;
# get the count wrong and the ESP32 drops the whole line, so this has to match
# the sketch's own LED count exactly.
NUM_LEDS = int(os.environ.get("LIGMAX_LIGHTS_NUM_LEDS", "101"))

# 101 LEDs is 612 bytes a frame, and 115200 8N1 carries 11.5 kB/s - so one frame
# occupies the wire for ~53 ms and the ceiling is about 18 fps. 15 fps leaves
# headroom for the UART to keep up while still being smooth enough for the
# breathe. Frames identical to the last one sent are skipped, so a solid colour
# costs one frame per KEEPALIVE_PERIOD rather than 15 a second.
FRAME_PERIOD = 1.0 / float(os.environ.get("LIGMAX_LIGHTS_FPS", "15"))
KEEPALIVE_PERIOD = float(os.environ.get("LIGMAX_LIGHTS_RESEND_S", "1.0"))
OPEN_RETRY_PERIOD = 5.0
WRITE_TIMEOUT = 0.25

# Scales every channel on the way out. The strip at full white is 101 pixels of
# maximum draw, which is more than the hull's supply is sized for on some builds;
# turn this down rather than dimming the individual patterns, so the colours stay
# in the same ratios to each other.
BRIGHTNESS = float(os.environ.get("LIGMAX_LIGHTS_BRIGHTNESS", "1.0"))

STROBE_HZ = 4.0  # OUT_OF_CONTROL, per the docstring
BREATHE_PERIOD = 4.0  # STANDBY, seconds for a full dim->bright->dim cycle
BREATHE_FLOOR = 0.04  # never fully off, so "powered" stays readable

# --- The mapping. This is the authoritative copy. ---------------------------
#
# Mirrored in `ligmax-server/tools/sim_boat.py` (LIGHT_COLOURS) so the simulator
# can drive the dashboard's cross-check, and in `ligmax-server/web/js/status.js`
# as the colour the dashboard *expects*. The dashboard compares the two and
# shouts if they differ; if they ever do, this file is right and the others are
# stale.
#
# The mode numbers this used to carry are gone with the firmware that understood
# them. The pattern is the mapping now.
PATTERN_SOLID = "solid"
PATTERN_STROBE = "strobe"
PATTERN_BREATHE = "breathe"

# status -> (pattern, base RGB)
STATUS_PATTERNS = {
    "AUTONOMOUS": (PATTERN_SOLID, (0, 255, 0)),
    "REMOTE": (PATTERN_SOLID, (255, 100, 0)),
    "KILLED": (PATTERN_SOLID, (255, 0, 0)),
    "OUT_OF_CONTROL": (PATTERN_STROBE, (255, 0, 0)),
    "STANDBY": (PATTERN_BREATHE, (255, 255, 255)),
}

# What the operator's dashboard is told is showing. Names, not raw RGB, because
# "red-strobe" and "red" being different is the whole point and two triples side
# by side would not make that obvious.
STATUS_COLOURS = {
    "AUTONOMOUS": "green",
    "REMOTE": "yellow",
    "KILLED": "red",
    "OUT_OF_CONTROL": "red-strobe",
    "STANDBY": "white",
}

# Where the hull goes if this node cannot work out what the boat is doing. Not
# green, and not a calm idle white: an unknown state is closer to out-of-control
# than to anything else.
FALLBACK_STATUS = "OUT_OF_CONTROL"

try:
    import serial  # pyserial
except ImportError:  # not on the Pi, or the library is not installed yet
    serial = None


class _WrongPort(Exception):
    """The configured port is not the GPIO 14/15 UART. A config fault, not a cable
    fault - kept as its own type so `_open()` can log it once instead of every
    OPEN_RETRY_PERIOD."""


def _scale(rgb, level):
    """One pixel, scaled by `level` (0..1) and the global BRIGHTNESS."""
    k = max(0.0, min(1.0, level)) * BRIGHTNESS
    return tuple(min(255, max(0, int(round(c * k)))) for c in rgb)


def render(status, t):
    """The pixel for `status` at time `t` seconds.

    Every pattern here is a whole-strip colour, so a frame is one pixel repeated.
    Kept as its own function - with no acks coming back, a bench check that prints
    what would go out is the only way to see the animation without the hardware.
    """
    pattern, rgb = STATUS_PATTERNS[status]
    if pattern == PATTERN_STROBE:
        # Square wave: lit for the first half of each period. Fully off between
        # flashes is what makes this unmistakably not solid red.
        return _scale(rgb, 1.0 if (t * STROBE_HZ) % 1.0 < 0.5 else 0.0)
    if pattern == PATTERN_BREATHE:
        # Raised cosine, so the turns at each end are gentle and it reads as
        # breathing rather than as a slow blink.
        phase = (1.0 - math.cos(2.0 * math.pi * t / BREATHE_PERIOD)) / 2.0
        return _scale(rgb, BREATHE_FLOOR + (1.0 - BREATHE_FLOOR) * phase)
    return _scale(rgb, 1.0)


def frame(pixel):
    """The wire format: `DATA ` + NUM_LEDS * RRGGBB + newline."""
    return "DATA " + ("%02X%02X%02X" % pixel) * NUM_LEDS + "\n"


BLANK = frame((0, 0, 0))


class Lights:
    """The hull's signal lights. `set_status()` is the whole interface.

    Owns one worker thread. The status is held as "the latest wanted", never
    accumulated: if it changes three times between two frames, the hull shows the
    third colour and the first two never existed. That is right for a state
    indicator and wrong for a command queue, which is why this is not one.
    """

    def __init__(self, port=PORT, baud=BAUD):
        self.port_name = port
        self.baud = baud

        self._serial = None
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._closed = False

        self._wanted = None  # status name, or None until someone says
        self._shown = None  # the status the strip was last sent a frame for
        self._last_frame = None  # the exact line last written, to skip repeats
        self._last_write = 0.0
        self._last_open_attempt = 0.0
        self._frames = 0
        self._write_errors = 0
        self._last_error = None

        if serial is None:
            log.warning(
                "pyserial is not installed - the hull lights are a no-op "
                "(pip install pyserial). The strip stays dark."
            )

        self._thread = threading.Thread(target=self._run, daemon=True, name="lights")
        self._thread.start()

    # -- public API ---------------------------------------------------------

    @property
    def available(self):
        """True when there is a real serial port behind this object.

        Not a claim that the hull is lit - nothing on this link acks, so that
        cannot be known from here.
        """
        return self._serial is not None

    def set_status(self, status):
        """Ask the hull to show the colour for `status`. Never blocks or raises.

        Unknown status names fall back to FALLBACK_STATUS rather than being
        ignored, because "we do not know what the boat is doing" is itself a thing
        the lights should say.
        """
        if self._closed:
            return
        if status not in STATUS_PATTERNS:
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

        `link` is weaker than it used to be, and deliberately so. The old firmware
        acked every mode, so `link` could mean "the ESP32 confirmed the colour".
        This one says nothing back, so the strongest true statement is "the port is
        open and frames are leaving" - which is what `link` means now. `verified`
        records that the difference is real rather than an outage.
        """
        with self._lock:
            wanted, shown = self._wanted, self._shown
        block = {
            "link": self.available and self._frames > 0,
            "verified": False,  # no return path on this firmware
            "frames": self._frames,
        }
        if shown is not None:
            block["colour"] = STATUS_COLOURS[shown]
            block["for_status"] = shown
        # Only worth reporting while it is actually true, so it does not sit in
        # the panel as a permanent field nobody reads.
        if wanted is not None and wanted != shown:
            block["pending"] = STATUS_COLOURS[wanted]
        if self._write_errors:
            block["errors"] = self._write_errors
        if self._last_error:
            block["last_error"] = str(self._last_error)[:120]
        return block

    def close(self):
        """Stop the worker, blank the strip, release the port.

        Blanking is the change from the old firmware's behaviour, and it is the
        point: a dumb driver holds its last frame forever, so leaving the hull as
        it is would leave a colour asserting a state nothing is maintaining -
        green on a boat with no autonomy running. Dark is honest; stale is not.
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
                port.write(BLANK.encode("ascii"))
                port.flush()
            except Exception:  # noqa: BLE001 - a dead cable is not news on the way out
                pass
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
            # Wake early on a status change, otherwise animate on the frame timer.
            self._wake.wait(FRAME_PERIOD)

    def _tick(self):
        with self._lock:
            wanted = self._wanted
        if wanted is None:
            return

        if self._serial is None:
            self._open()
            if self._serial is None:
                return

        now = time.monotonic()
        line = frame(render(wanted, now))

        # Skip frames that would repaint the strip with what it already has, but
        # never go longer than KEEPALIVE_PERIOD: a strip that missed a byte or
        # came up after us has to get another chance without waiting for the
        # status to change.
        if line == self._last_frame and (now - self._last_write) < KEEPALIVE_PERIOD:
            self._drain()
            return

        if not self._write(line):
            # Drop the port so the next tick reopens it: a failed write usually
            # means the device went away.
            self._reset_port()
            self._last_frame = None
            return

        self._last_write = now
        self._last_frame = line
        self._frames += 1
        with self._lock:
            self._shown = wanted
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
        # never lights. Refuse it by name instead.
        if os.path.realpath(self.port_name) == os.path.realpath(_DEBUG_UART):
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
                    "cannot open the lights port %s: %s - the strip stays dark",
                    self.port_name,
                    exc,
                )
            self._last_error = exc
            return
        with self._lock:
            self._serial = port
        self._last_frame = None  # repaint on a fresh port
        self._last_error = None
        log.info("lights ESP32 on %s @ %s baud, %d LEDs",
                 self.port_name, self.baud, NUM_LEDS)

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
        """Throw away anything the ESP32 said. Never blocks - the port has timeout=0.

        This firmware answers nothing, so there is no ack to read and no state to
        take from what arrives. It is drained anyway because boot chatter and line
        noise still land in the kernel buffer, and a buffer nobody empties is a
        buffer that eventually blocks a read somewhere else.
        """
        port = self._serial
        if port is None:
            return
        try:
            waiting = port.in_waiting
            if waiting:
                port.read(min(waiting, 1024))
        except Exception as exc:  # noqa: BLE001
            self._last_error = exc
            self._reset_port()

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
    # Walks the hull through all five colours, four seconds each - long enough to
    # see one full breathe cycle and a dozen strobe flashes. Watch the strips;
    # every state the boat can be in should be visibly distinct.
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s %(message)s")
    lights = Lights()
    try:
        for name in ("STANDBY", "AUTONOMOUS", "REMOTE", "OUT_OF_CONTROL", "KILLED"):
            lights.set_status(name)
            print(f"{name:<15} -> {STATUS_COLOURS[name]} ({STATUS_PATTERNS[name][0]})")
            time.sleep(4.0)
            print(f"                  {lights.telemetry()}")
    finally:
        lights.close()
