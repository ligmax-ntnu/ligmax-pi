"""Drive the headlights ESP32 over USB, so the operator owns the bow.

The board
---------
A second classic ESP32 (die marked ESP32-D0WDQ6 rev 3, same part as the hull
board's module), on a small carrier with a micro-USB socket and a LiPo input,
plugged into one of the Pi's USB sockets. It owns two things that were on the
hull lights board until 2026-08-13:

  * the two **forward LED strips** - working lights, not signals. Nothing had
    ever driven them: the old firmware kept them black whenever the Pi was
    talking and breathing-white when it was not, and no command existed to say
    anything else.
  * the two **headlight-cover servos**, which had the same problem from the
    other end - the firmware understood open and close, and nothing ever sent
    either, so both covers sat closed for as long as the Pi had been in charge.

`lights.py` keeps the aft hull strips and the five safety colours. The split is
not cosmetic: that link is a UART on the GPIO header that **answers nothing**,
and this one is a serial line that answers everything. Two links, two failures,
two telemetry blocks.

The link
--------
There is no native USB on a D0WDQ6. The socket belongs an onboard USB-UART
bridge (CP2102/CP2104/CH340), so on the Pi this is a `/dev/ttyUSB*` device at
115200 8N1, and on the ESP32 it is plain UART0.

Every command is answered, which is the whole reason the protocol looks like
this:

    ID                      -> "LIGMAX-HEADLIGHTS 1 front=8 servos=2"
    FRONT <L|R|B> <RRGGBB>  -> "OK FRONT L FF8800"
    SRV <L|R> <deg>         -> "OK SRV L 110"
    B <0-255>               -> "OK B200"

**The handshake is not politeness, it is the safety interlock.** This Pi has
more than one USB serial device on it and one of them is the flight controller
(`/dev/ttyACM0`, pymavlink's, held open by the MAVLink pump in `main.py`). So
this module:

  * never opens a port that resolves to the autopilot's device, by name or by
    realpath - opening a tty a second time on Linux succeeds and *steals bytes*
    from whoever else is reading it, which would cost the autopilot its
    heartbeat stream;
  * only considers a port whose `/dev/serial/by-id/` name looks like a USB-UART
    bridge, or one an operator named outright in `LIGMAX_HEADLIGHTS_PORT`;
  * writes `ID` and nothing else until the expected prefix comes back. No
    colour, no angle, not one byte of hex reaches a device that has not
    identified itself.

Opening the port very probably **resets the board** - these carriers wire the
bridge's DTR/RTS to EN/IO0 - so a reconnect means boot chatter, both covers
driven home to closed, and both strips dark, with nothing on the wire to say it
happened. That is why every wanted value here is re-asserted on a keepalive
rather than sent once: after a reset, the next tick puts the bow back the way
the operator left it.

Design rules, copied from `lights.py` because the reasons are the same:

  * **Never blocks the caller.** `set_front()`/`set_servos()` store a value and
    poke an Event; all serial I/O is on a worker thread against a port with a
    write timeout.
  * **Never raises.** No port, no `pyserial`, a cable pulled mid-run: every one
    degrades to a logged no-op with `available = False`, and the telemetry says
    so rather than asserting a bow that is not lit.
  * **Re-asserting**, per the reset problem above.

And one rule that is this module's own: **the covers are a mechanism.** They are
not blanked by `close()`, not overridden by `KILLED`, and not moved by anything
except an operator asking. The strips *are* blanked on the way out, because a
light nobody is maintaining is worth turning off and a cover nobody is
maintaining is worth leaving exactly where it is.
"""

import glob
import logging
import os
import threading
import time

log = logging.getLogger("io_manager.headlights")

# Empty means "search", which is the normal case - see `_candidates()`. Set this
# to a `/dev/serial/by-id/...` path (stable across reboots; `/dev/ttyUSB0` is
# not) when there is more than one bridge on the bus.
PORT = os.environ.get("LIGMAX_HEADLIGHTS_PORT", "")
BAUD = int(os.environ.get("LIGMAX_HEADLIGHTS_BAUD", "115200"))

# The autopilot's port, which this module must never open. Same default as
# `main.py`'s MAVLink device, overridable together with it.
AUTOPILOT_PORT = os.environ.get("LIGMAX_MAVLINK_PORT", "/dev/ttyACM0")

# Where to look, in preference order. by-id first because those names are stable
# and carry the bridge chip's identity; the bare device is the fallback for a
# system with no udev by-id links.
_SEARCH = ("/dev/serial/by-id/*", "/dev/ttyUSB*")

# A by-id name has to look like a USB-UART bridge before it is even opened. This
# is an allow-list on purpose: the failure of a deny-list here is opening the
# flight controller, and "I did not recognise it so I tried it" is not an
# acceptable way to treat a device that might be steering the boat.
_BRIDGE_MARKERS = ("cp210", "ch340", "ch910", "ft232", "ftdi", "silicon_labs",
                   "wch.cn", "usb2.0-ser", "qinheng")

# What the board must say to `ID` before anything else is written to it.
ID_COMMAND = "ID\n"
ID_PREFIX = "LIGMAX-HEADLIGHTS"
ID_TIMEOUT = 2.5  # generous: a board that just reset spends ~1 s in the ROM
                  # bootloader before the sketch's own setup() runs

WRITE_TIMEOUT = 0.25
OPEN_RETRY_PERIOD = 5.0
KEEPALIVE_PERIOD = float(os.environ.get("LIGMAX_HEADLIGHTS_RESEND_S", "1.0"))
TICK_PERIOD = 0.2  # nothing animates here; this only bounds how fast a change goes

# -- The two forward strips --------------------------------------------------
#
# Solid colour per side, because eight pixels behind a cover have nothing to
# animate. `None` means "never asked", which is not the same as black: the board
# boots dark, and asserting black would be a command we were not given.
FRONT_SIDES = ("left", "right")

# -- The cover servos --------------------------------------------------------
#
# The authoritative copy is `headlights_esp.ino`'s SERVO_L_/SERVO_R_ CLOSED and
# OPEN; this mirrors it, and so does ligmax-server's LIGHTS_SERVO_ENDPOINTS.
# Three copies of one bench calibration in three repos, and a mismatch is
# silent - the firmware would clamp an angle the other two believed was
# honoured. Change all three or none.
#
# The travel between the two endpoints is the whole of what anyone has measured.
# Past an endpoint the horn drives the cover into its own mechanical stop and the
# servo holds stall current there, so an angle outside the span is refused here
# rather than passed on. Widening it means measuring the mechanism.
#
# The sides are MIRRORED: left opens by increasing the angle, right by
# decreasing it. 90 deg is not the same place on the two covers, which is why
# nothing here takes an angle without a side.
SERVO_ENDPOINTS = {
    "left": {"closed": 20, "open": 110},
    "right": {"closed": 160, "open": 70},
}
SERVO_SIDES = ("left", "right")

try:
    import serial  # pyserial
except ImportError:  # not on the Pi, or the library is not installed yet
    serial = None


def servo_range(side):
    """`(min_deg, max_deg)` for one cover - the span between its endpoints.

    Derived rather than written out again, because the two sides are mirrored and
    it is the mirroring that makes a hand-written pair of bounds easy to get
    backwards.
    """
    ends = SERVO_ENDPOINTS[side]
    return (min(ends["closed"], ends["open"]), max(ends["closed"], ends["open"]))


def servo_line(side, deg):
    """The wire format: `SRV L|R <deg>` + newline."""
    return "SRV %s %d\n" % ("L" if side == "left" else "R", deg)


def front_line(side, rgb):
    """The wire format: `FRONT L|R RRGGBB` + newline."""
    return "FRONT %s %02X%02X%02X\n" % (("L" if side == "left" else "R"), *rgb)


def parse_colour(value):
    """`"RRGGBB"` (a leading `#` tolerated) or an `(r, g, b)` triple -> a triple.

    Raises ValueError, which every caller here turns into a refusal message
    rather than an exception.
    """
    if isinstance(value, (tuple, list)):
        if len(value) != 3:
            raise ValueError(f"bad colour {value!r}, want three channels")
        out = []
        for channel in value:
            number = int(channel)
            if not 0 <= number <= 255:
                raise ValueError(f"bad colour {value!r}, channels are 0-255")
            out.append(number)
        return tuple(out)
    text = str(value).strip().lstrip("#")
    if len(text) != 6:
        raise ValueError(f"bad colour {value!r}, want 6 hex chars")
    try:
        return (int(text[0:2], 16), int(text[2:4], 16), int(text[4:6], 16))
    except ValueError:
        raise ValueError(f"bad colour {value!r}, want hex digits") from None


def _looks_like_bridge(path):
    name = os.path.basename(path).lower()
    return any(marker in name for marker in _BRIDGE_MARKERS)


class Headlights:
    """The bow: two forward strips and the two cover servos, on their own board.

    `set_front()` and `set_servos()` are the whole API. Both store a wanted value
    and return `(ok, message)`; the worker thread finds the board, proves it is
    the board, and keeps it told.
    """

    def __init__(self, port=PORT, baud=BAUD):
        self.port_name = port  # "" means search
        self.baud = baud

        self._serial = None
        self._opened_path = None  # the device actually in use, for telemetry
        self._identity = None  # what it answered to ID
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._closed = False

        self._front = dict.fromkeys(FRONT_SIDES)  # wanted (r, g, b) or None
        self._front_sent = dict.fromkeys(FRONT_SIDES)
        self._servos = dict.fromkeys(SERVO_SIDES)  # wanted degrees or None
        self._servos_sent = dict.fromkeys(SERVO_SIDES)
        self._last_assert = 0.0

        self._last_open_attempt = 0.0
        self._writes = 0
        self._acks = 0
        self._nacks = 0
        self._write_errors = 0
        self._last_error = None
        self._last_reply = None
        # Ports that were looked at and turned out not to be this board. Kept so
        # the search does not re-open the same wrong device every 5 s.
        self._rejected = set()

        if serial is None:
            log.warning(
                "pyserial is not installed - the headlights and the covers are a "
                "no-op (pip install pyserial)"
            )

        self._thread = threading.Thread(target=self._run, daemon=True, name="headlights")
        self._thread.start()

    # -- public API ---------------------------------------------------------

    @property
    def available(self):
        """True when a port is open **and** the board on it said it was ours.

        Stronger than `lights.available`, and it can be: this link answers, so
        "there is a device" and "it is the right device" are distinguishable
        here in a way they are not on the hull UART.
        """
        return self._serial is not None and self._identity is not None

    def set_front(self, left=None, right=None):
        """Set the forward strips' colour. `None` for a side leaves it alone.

        Colours are `"RRGGBB"` or `(r, g, b)`. Returns `(ok, message)`; never
        blocks, never raises.
        """
        if self._closed:
            return False, "the headlights driver is shut down"
        wanted = {}
        for side, value in (("left", left), ("right", right)):
            if value is None:
                continue
            try:
                wanted[side] = parse_colour(value)
            except ValueError as exc:
                return False, f"{side}: {exc}"
        if not wanted:
            return False, "no colour given: send a left, a right, or both"
        with self._lock:
            changed = {s: c for s, c in wanted.items() if self._front[s] != c}
            self._front.update(wanted)
        if changed:
            log.info("headlights -> %s", ", ".join(
                f"{s} #{r:02X}{g:02X}{b:02X}" for s, (r, g, b) in sorted(changed.items())))
            self._wake.set()
        return True, ", ".join(
            f"{s} #{r:02X}{g:02X}{b:02X}" for s, (r, g, b) in sorted(wanted.items()))

    def set_servos(self, left=None, right=None):
        """Point the headlight covers at explicit angles, in degrees.

        `None` for a side leaves that side alone. Returns `(ok, message)` rather
        than swallowing a bad value: a slider asking for an angle outside the
        cover's travel has to come back as those words, because the alternative -
        nothing moves and nothing is said - is indistinguishable from a dead
        link.
        """
        if self._closed:
            return False, "the headlights driver is shut down"
        wanted = {}
        for side, value in (("left", left), ("right", right)):
            if value is None:
                continue
            lo, hi = servo_range(side)
            try:
                deg = int(round(float(value)))
            except (TypeError, ValueError, OverflowError):
                return False, f"{side} angle {value!r} is not a number of degrees"
            if not lo <= deg <= hi:
                ends = SERVO_ENDPOINTS[side]
                return False, (
                    f"{side} cover travels {lo}-{hi} deg ({ends['closed']} closed, "
                    f"{ends['open']} open); {deg} is outside it"
                )
            wanted[side] = deg
        if not wanted:
            return False, "no angle given: send a left, a right, or both"
        with self._lock:
            changed = {s: d for s, d in wanted.items() if self._servos[s] != d}
            self._servos.update(wanted)
        if changed:
            # WARNING, not INFO: this is the one thing on this board that moves a
            # mechanism, and the log is where somebody looks after finding a cover
            # somewhere they did not leave it.
            log.warning("headlight covers -> %s",
                        ", ".join(f"{s} {d} deg" for s, d in sorted(changed.items())))
            self._wake.set()
        return True, ", ".join(f"{s} {d} deg" for s, d in sorted(wanted.items()))

    def telemetry(self):
        """The `telemetry.headlights` block.

        Its own block rather than more fields on `telemetry.lights`, because this
        is a different board on a different link: the hull can be lit while the
        bow is unplugged, and one `link` field covering both would make each
        failure look like the other. Same reasoning as the two lidars.
        """
        with self._lock:
            front = dict(self._front)
            servos = dict(self._servos)
        block = {
            "link": self.available,
            # This link answers, unlike the hull's. `verified` here means the
            # board identified itself AND the last write was acked - which is
            # genuinely more than `lights.verified` can ever be.
            "verified": bool(self._identity) and self._acks > 0 and self._nacks == 0,
            "writes": self._writes,
            "acks": self._acks,
            # Commanded, never measured. The servos have no feedback of any kind,
            # so an angle is "what the board was last told"; the ack proves the
            # command arrived, not that the cover moved. Sent even while None
            # (nobody has asked, so the bow is however the board booted) because
            # the dashboard merges telemetry key by key and a field that vanished
            # on a restart would leave yesterday's value on screen.
            "servo_left_deg": servos["left"],
            "servo_right_deg": servos["right"],
            "front_left": None if front["left"] is None else "%02X%02X%02X" % front["left"],
            "front_right": None if front["right"] is None else "%02X%02X%02X" % front["right"],
        }
        if self._opened_path:
            block["port"] = self._opened_path
        if self._identity:
            block["board"] = self._identity
        if self._last_reply:
            block["last_reply"] = self._last_reply[:80]
        if self._nacks:
            block["refused"] = self._nacks
        if self._write_errors:
            block["errors"] = self._write_errors
        if self._last_error:
            block["last_error"] = str(self._last_error)[:120]
        return block

    def close(self):
        """Stop the worker, blank the strips, leave the covers alone, drop the port.

        The asymmetry is the point. A light nobody is maintaining should go out -
        same argument as `lights.close()` blanking the hull. A cover is a
        mechanism, and driving one on the way out would move hardware nobody
        asked to move, in the one code path that runs while something is already
        going wrong.
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
                port.write(front_line("left", (0, 0, 0)).encode("ascii"))
                port.write(front_line("right", (0, 0, 0)).encode("ascii"))
                port.flush()
            except Exception:  # noqa: BLE001 - a dead cable is not news on the way out
                pass
            try:
                port.close()
            except Exception:  # noqa: BLE001
                pass

    # -- worker -------------------------------------------------------------

    def _run(self):
        while not self._closed:
            self._wake.clear()
            try:
                self._tick()
            except Exception as exc:  # noqa: BLE001 - the loop must not die
                self._last_error = exc
                log.error("headlights worker: %s", exc)
            self._wake.wait(TICK_PERIOD)

    def _tick(self):
        with self._lock:
            front = dict(self._front)
            servos = dict(self._servos)
        # Nothing to say yet: the bow is dark and closed, which is where the board
        # booted, so there is no reason to hold a port open or to reset the board
        # by opening one.
        if not any(v is not None for v in (*front.values(), *servos.values())):
            return

        if not self.available:
            self._open()
            if not self.available:
                return

        now = time.monotonic()
        due = (now - self._last_assert) >= KEEPALIVE_PERIOD
        wrote = False
        for side in SERVO_SIDES:
            deg = servos[side]
            if deg is None or (deg == self._servos_sent[side] and not due):
                continue
            if not self._write(servo_line(side, deg)):
                return
            self._servos_sent[side] = deg
            wrote = True
        for side in FRONT_SIDES:
            rgb = front[side]
            if rgb is None or (rgb == self._front_sent[side] and not due):
                continue
            if not self._write(front_line(side, rgb)):
                return
            self._front_sent[side] = rgb
            wrote = True
        if wrote:
            self._last_assert = now
        self._read_replies()

    # -- the port -----------------------------------------------------------

    def _candidates(self):
        """Devices worth opening, best first.

        An allow-list, and deliberately so: everything this refuses to consider
        is a device that keeps working because nothing here touched it.
        """
        if self.port_name:
            # Named outright. Still refused if it is the autopilot - an operator
            # setting that by mistake is exactly the accident worth catching.
            return [self.port_name]
        found = []
        for pattern in _SEARCH:
            for path in sorted(glob.glob(pattern)):
                if path in found or path in self._rejected:
                    continue
                if pattern.endswith("by-id/*") and not _looks_like_bridge(path):
                    continue
                found.append(path)
        return found

    def _is_autopilot(self, path):
        try:
            return os.path.realpath(path) == os.path.realpath(AUTOPILOT_PORT)
        except OSError:
            return False

    def _open(self):
        if serial is None:
            return
        now = time.monotonic()
        if now - self._last_open_attempt < OPEN_RETRY_PERIOD:
            return
        self._last_open_attempt = now

        candidates = self._candidates()
        if not candidates:
            if not isinstance(self._last_error, FileNotFoundError):
                self._last_error = FileNotFoundError(
                    "no USB-UART bridge found for the headlights board"
                )
                log.error(
                    "no headlights board found: nothing under /dev/serial/by-id/ "
                    "looks like a USB-UART bridge. Plug it in, or name the device "
                    "in LIGMAX_HEADLIGHTS_PORT. The bow stays dark and the covers "
                    "stay where they are."
                )
            return

        for path in candidates:
            if self._is_autopilot(path):
                # Not opened, not probed, not touched. Opening a tty twice on
                # Linux succeeds and steals bytes from the other reader, and the
                # other reader here is the MAVLink pump.
                log.error(
                    "refusing to probe %s for the headlights board: it is the "
                    "autopilot's port (%s)", path, AUTOPILOT_PORT
                )
                self._rejected.add(path)
                continue
            port = self._try(path)
            if port is not None:
                with self._lock:
                    self._serial = port
                self._opened_path = path
                # Everything has to be said again: the open almost certainly
                # reset the board, so whatever it was showing is gone.
                self._front_sent = dict.fromkeys(FRONT_SIDES)
                self._servos_sent = dict.fromkeys(SERVO_SIDES)
                self._last_error = None
                log.info("headlights board on %s: %s", path, self._identity)
                return

    def _try(self, path):
        """Open one candidate and make it prove what it is. Returns a port or None."""
        try:
            port = serial.Serial(path, self.baud, timeout=0, write_timeout=WRITE_TIMEOUT)
        except Exception as exc:  # noqa: BLE001 - busy, missing, no permission
            if str(exc) != str(self._last_error):
                log.warning("cannot open %s for the headlights: %s", path, exc)
            self._last_error = exc
            return None
        # These carriers wire DTR/RTS to EN/IO0, so the open just reset the board
        # and holding either line asserted can keep it in the bootloader. Drop
        # both, then wait out the boot before expecting an answer.
        for line in ("dtr", "rts"):
            try:
                setattr(port, line, False)
            except Exception:  # noqa: BLE001 - not every driver exposes them
                pass
        identity = self._handshake(port)
        if identity is None:
            log.warning(
                "%s did not answer ID with %s - not the headlights board, leaving "
                "it alone", path, ID_PREFIX
            )
            self._rejected.add(path)
            try:
                port.close()
            except Exception:  # noqa: BLE001
                pass
            return None
        self._identity = identity
        return port

    def _handshake(self, port):
        """`ID` until the board answers, or None. Nothing else may be written first."""
        deadline = time.monotonic() + ID_TIMEOUT
        buffer = ""
        asked = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now - asked > 0.5:  # re-ask: the first one may have gone into a
                asked = now        # board that was still in its bootloader
                try:
                    port.write(ID_COMMAND.encode("ascii"))
                except Exception as exc:  # noqa: BLE001
                    self._last_error = exc
                    return None
            try:
                chunk = port.read(256)
            except Exception as exc:  # noqa: BLE001
                self._last_error = exc
                return None
            if chunk:
                buffer += chunk.decode("ascii", "replace")
                for raw in buffer.splitlines():
                    line = raw.strip()
                    if line.startswith(ID_PREFIX):
                        return line
                buffer = buffer[-256:]  # keep the tail, drop the boot chatter
            time.sleep(0.05)
        return None

    def _write(self, line):
        port = self._serial
        if port is None:
            return False
        try:
            port.write(line.encode("ascii"))
            self._writes += 1
            return True
        except Exception as exc:  # noqa: BLE001 - a pulled cable is not fatal
            self._write_errors += 1
            self._last_error = exc
            log.warning("headlights write failed: %s", exc)
            self._reset_port()
            return False

    def _read_replies(self):
        """Read what the board said. Never blocks - the port has timeout=0.

        Unlike the hull link there is something to learn here: `OK` lines are the
        only evidence in the fleet that a light command actually arrived
        somewhere, and `ERR` lines mean this module and the firmware disagree
        about the protocol, which is worth seeing in telemetry rather than in a
        shrug.
        """
        port = self._serial
        if port is None:
            return
        try:
            waiting = port.in_waiting
            if not waiting:
                return
            chunk = port.read(min(waiting, 512)).decode("ascii", "replace")
        except Exception as exc:  # noqa: BLE001
            self._last_error = exc
            self._reset_port()
            return
        for raw in chunk.splitlines():
            line = raw.strip()
            if not line:
                continue
            self._last_reply = line
            if line.startswith("OK"):
                self._acks += 1
            elif line.startswith("ERR"):
                self._nacks += 1
                log.warning("headlights board refused a command: %s", line)

    def _reset_port(self):
        with self._lock:
            port, self._serial = self._serial, None
        self._identity = None  # it has to identify itself again
        self._opened_path = None
        if port is not None:
            try:
                port.close()
            except Exception:  # noqa: BLE001
                pass


if __name__ == "__main__":
    # Bench check on the Pi:  python -m nodes.io_manager.headlights
    #
    # **This moves hardware.** It sweeps both covers closed -> open -> midway and
    # walks the strips through a few colours, so run it with the bow accessible
    # and the covers unobstructed. Watch the strips and the covers; watch the log
    # for which /dev path it settled on and what the board called itself.
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s %(message)s")
    bow = Headlights()
    try:
        for label, colour in (("white", "FFFFFF"), ("amber", "FF8800"),
                              ("dim", "202020"), ("off", "000000")):
            print(f"strips {label:<8} -> {bow.set_front(colour, colour)}")
            time.sleep(2.0)
        for key in ("closed", "open", "closed"):
            left = SERVO_ENDPOINTS["left"][key]
            right = SERVO_ENDPOINTS["right"][key]
            print(f"covers {key:<8} -> {bow.set_servos(left, right)}")
            time.sleep(3.0)
        # And one place that is neither end, which is the point of an angle: a
        # cover that only ever reaches its endpoints is the old open/close pair
        # with extra steps.
        mid = {s: sum(servo_range(s)) // 2 for s in SERVO_SIDES}
        print(f"covers midway   -> {bow.set_servos(mid['left'], mid['right'])}")
        time.sleep(3.0)
        print(f"telemetry: {bow.telemetry()}")
    finally:
        bow.close()
