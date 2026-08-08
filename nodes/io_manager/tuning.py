"""Read and write the stabilisation tuning on the flight controller.

The two stabilisation loops do not run here. `amas.lua` (roll, the amas) and
`battery_slider.lua` (pitch, the sliding pack) run on the Pixhawk, and every
number they use is an **ArduPilot parameter**. So "change the gains from the
dashboard" is not a new control path - it is the MAVLink parameter protocol,
which this module drives:

    load   PARAM_REQUEST_READ (by name)  ->  PARAM_VALUE
    save   PARAM_SET                     ->  PARAM_VALUE (the stored value, echoed)

Both halves of "save and load automatically" fall out of that:

  * **load** happens without being asked. Every whitelisted parameter is read on
    connect, re-read slowly in the background (REFRESH_PERIOD_S) so an edit made
    in Mission Planner cannot leave the dashboard showing a value nobody has any
    more, and re-read from scratch whenever the operator presses Reload.
    Everything read rides up as `telemetry.tuning.values`, so the dashboard's
    fields fill themselves in.
  * **save** is persistent by construction. ArduPilot's PARAM_SET is a
    set-and-save: the value is in the flight controller's own storage before the
    echo comes back, so it survives a reboot of the Pixhawk, of this Pi, and of
    the dashboard, with nothing on shore having to remember it. The echo is not
    cosmetic either - it carries what was *actually* stored, so a value the
    autopilot clamped or rounded is reported as a failure with the real number
    rather than acked as though it had been taken.

Why a whitelist (`TUNABLES`) rather than passing any name through
--------------------------------------------------------------
Because this is a 5 kW boat and PARAM_SET reaches every parameter on the flight
controller, including the ones that decide what a servo output does and whether
arming is allowed. The whitelist is the list of things the operator's console may
touch, and each entry carries the range a write must fall inside. A name that is
not here is refused on the vessel, whatever the ground station thought it was
sending - `ligmax-server/ligmax_gui/tuning.py` validates the same table before
queueing the command, and this is the copy that matters.

Two bounds in it are not arbitrary and should not be widened without reading why:

    SCR_USER6   the amas' ride-height trim. The translator ESP32 reads its pulse
                as a velocity, so a standing offset makes both amas creep for as
                long as it is set - intended, and it is how the hull is walked to
                a ride height from shore. +/-250 us is half the +/-500 us span
                `roll_output` also has to fit in, so a trim left set from shore
                can never be the reason there is no roll authority left
                (docs/findings.md item 10).
    BSLD_SIGN   not writable from here at all. Getting it wrong is a divergent
                loop driving 1.8 kWh of pack into an endstop, and the procedure
                for setting it is a bench test with the hull supported
                (battery_slider.lua, BEFORE FIRST RUN step 3). It is read and
                displayed so an operator can see what it is; changing it is a job
                for someone standing next to the boat.

Everything here is driven from the same MAVLink pump as `Navigation`, `Trim` and
`MissionUploader` - one message in at a time, one parameter out at a time, never
a blocking wait - because this node owes the autopilot a 1 Hz heartbeat and a
parameter sweep must not be what costs it.
"""

from __future__ import annotations

import logging
import math
import time
from collections import deque

from pymavlink import mavutil

log = logging.getLogger("io_manager.tuning")

# RC input channels a trim knob may be assigned to. 1..8 carry the sticks and the
# aux switches on this boat, and 15/16 are already the amas' height command, so a
# knob belongs on 9..14. Both Lua scripts refuse anything below MIN_RC_CHANNEL
# themselves (`MIN_RC_CHAN`); this is the same rule at the other end, so a bad
# channel is refused with an explanation instead of silently doing nothing.
MIN_RC_CHANNEL = 9
MAX_RC_CHANNEL = 16

# Pacing. One parameter message per REQUEST_INTERVAL_S keeps a full sweep of the
# table under a second on the USB serial link while leaving the pump free for the
# traffic that actually flies the boat.
REQUEST_INTERVAL_S = 0.05
REQUEST_RETRY_S = 2.0
MAX_REQUEST_ATTEMPTS = 3
REFRESH_PERIOD_S = 60.0

# A write is a local exchange over a serial cable; 3 s is already generous. The
# retry exists for a dropped byte, not for a slow autopilot.
WRITE_TIMEOUT_S = 3.0
MAX_WRITE_ATTEMPTS = 2
MAX_QUEUED_WRITES = 32


class Tunable:
    """One flight-controller parameter the operator may see, and its bounds.

    `writable=False` means read and display only - see the module docstring on
    BSLD_SIGN. `channel=True` adds the "9..16, or 0 for off" rule, which is not
    expressible as a plain range because 0 is legal and 1..8 are not.
    """

    __slots__ = ("name", "what", "low", "high", "integer", "channel", "writable")

    def __init__(
        self, name, what, low, high, integer=False, channel=False, writable=True
    ):
        self.name = name
        self.what = what
        self.low = float(low)
        self.high = float(high)
        self.integer = integer
        self.channel = channel
        self.writable = writable

    def clean(self, value):
        """`(number, None)` if this may be written, `(None, why)` if it may not."""
        if not self.writable:
            return None, f"{self.name} ({self.what}) is read-only from the dashboard"
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None, f"{self.name} wants a number, not {value!r}"
        if not math.isfinite(number):
            return None, f"{self.name} wants a finite number"
        if self.integer:
            number = float(round(number))
        if self.channel:
            if number != 0.0 and not (MIN_RC_CHANNEL <= number <= MAX_RC_CHANNEL):
                return None, (
                    f"{self.name} is an RC input channel: 0 turns the knob off, "
                    f"otherwise {MIN_RC_CHANNEL}..{MAX_RC_CHANNEL}. Channels 1..8 "
                    "are the sticks and the aux switches and are not available."
                )
            return number, None
        if not (self.low <= number <= self.high):
            return None, (
                f"{self.name} ({self.what}) must be between "
                f"{self.low:g} and {self.high:g}, not {number:g}"
            )
        return number, None


# The whitelist. Mirrored in `ligmax-server/ligmax_gui/tuning.py`, which also
# carries the labels and help text the dashboard renders - keep the names and the
# ranges in step, and treat this copy as the authority: the server can only
# refuse a write early, this is what can refuse it at all.
TUNABLES = (
    # --- amas.lua: roll stabilisation, and the amas' ride height -------------
    # Kp and Kd map degrees (and degrees/second) to microseconds of actuator
    # command. The output is clamped to +/-500 us either side of neutral, so a
    # gain above 500 saturates on a one-degree error and the number stops meaning
    # anything - which is where the upper bound comes from.
    Tunable("SCR_USER1", "roll Kp", 0.0, 500.0),
    Tunable("SCR_USER2", "roll Kd", 0.0, 500.0),
    Tunable("SCR_USER3", "roll trim knob channel", 0, MAX_RC_CHANNEL,
            integer=True, channel=True),
    Tunable("SCR_USER4", "roll trim knob range, deg", 0.0, 20.0),
    Tunable("SCR_USER5", "roll trim from shore, deg", -10.0, 10.0),
    # The one parameter here that makes the boat move on its own. See the module
    # docstring; the amas creep for as long as this is non-zero, on purpose.
    Tunable("SCR_USER6", "ride-height trim from shore, us", -250.0, 250.0),
    # --- battery_slider.lua: pitch trim, the sliding pack --------------------
    # 0 = parked at BSLD_TRIM, 1 = closed loop while armed, 2 = closed loop always
    # (bench tuning: the pack WILL move with the props idle).
    Tunable("BSLD_ENABLE", "pitch loop mode", 0, 2, integer=True),
    # Normalised travel (-1..+1 of the rail) per degree, per degree-second and per
    # degree/second. u saturates at BSLD_LIMIT, which is at most 1, so a gain
    # above 1 is already full travel for a one-degree error.
    Tunable("BSLD_KP", "pitch Kp", 0.0, 5.0),
    Tunable("BSLD_KI", "pitch Ki", 0.0, 5.0),
    Tunable("BSLD_KD", "pitch Kd", 0.0, 5.0),
    Tunable("BSLD_IMAX", "integral cap", 0.0, 1.0),
    Tunable("BSLD_TRIM", "level-float rail position", -1.0, 1.0),
    Tunable("BSLD_LIMIT", "soft travel limit", 0.0, 1.0),
    Tunable("BSLD_SIGN", "pack direction sign", -1, 1, integer=True, writable=False),
    Tunable("BSLD_TRM_CH", "pitch trim knob channel", 0, MAX_RC_CHANNEL,
            integer=True, channel=True),
    Tunable("BSLD_TRM_DEG", "pitch trim knob range, deg", 0.0, 20.0),
    Tunable("BSLD_TRM_OFS", "pitch trim from shore, deg", -10.0, 10.0),
)

BY_NAME = {spec.name: spec for spec in TUNABLES}

# Parameters that exist only while their Lua script is running. ArduPilot adds a
# script's param table when the script first runs, so silence from all of these
# is evidence about `battery_slider.lua`, not about the link - which is worth
# saying out loud, because "the gains will not save" and "the script is not
# loaded" look identical from the dashboard otherwise.
SCRIPT_PARAMS = tuple(spec.name for spec in TUNABLES if spec.name.startswith("BSLD_"))


class _Write:
    __slots__ = ("command_id", "spec", "value", "attempts", "sent_at")

    def __init__(self, command_id, spec, value):
        self.command_id = command_id
        self.spec = spec
        self.value = value
        self.attempts = 0
        self.sent_at = 0.0


class Tuning:
    """Loads the tunable parameters, and writes the ones the operator changes.

    Owns no thread and no socket: `master` is passed in on every call, exactly
    like `MissionUploader`, so this can never race the heartbeat sender for the
    same connection object.
    """

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._values = {}
        self._seen_at = {}
        self._attempts = {}
        self._asked_at = {}
        self._last_sent = 0.0
        self._refresh_at = 0.0
        self._queue = deque()
        self._active = None
        self._outcomes = deque()
        self._writes_ok = 0
        self._writes_failed = 0
        self._last_write = None
        self._last_error = None

    # -- what the operator asked for -----------------------------------------

    def request_all(self, reason=""):
        """Re-read every parameter, starting on the next `pump()`.

        Values already known are kept while the sweep runs - they are what the
        autopilot last said, and blanking the panel for the second it takes would
        be less useful than showing them. Anything the sweep fails to re-read is
        dropped, so a parameter that has genuinely gone (a script unloaded) stops
        being displayed rather than lingering.
        """
        self._refresh_at = 0.0
        self._attempts.clear()
        self._asked_at.clear()
        self._seen_at.clear()
        if reason:
            log.info("re-reading %d tuning parameters: %s", len(TUNABLES), reason)

    def queue_write(self, command_id, name, value):
        """Validate and queue one `set_param`. `(True, None)` if it was queued.

        Returns `(False, why)` for anything refused, and the caller acks that
        immediately - a refusal the operator can read is the whole point of
        validating here rather than letting the autopilot silently ignore it.
        """
        key = str(name or "").strip().upper()
        spec = BY_NAME.get(key)
        if spec is None:
            return False, (
                f"'{key}' is not a tunable parameter on this vessel "
                f"({len(TUNABLES)} are: {', '.join(sorted(BY_NAME))})"
            )
        number, why = spec.clean(value)
        if why is not None:
            return False, f"refused: {why}"
        if len(self._queue) >= MAX_QUEUED_WRITES:
            return False, "refused: too many parameter writes already queued"
        self._queue.append(_Write(command_id, spec, number))
        return True, None

    # -- fed by the MAVLink pump ---------------------------------------------

    def handle(self, message):
        """Absorb one MAVLink message. Returns True if it was one of ours."""
        if message.get_type() != "PARAM_VALUE":
            return False
        raw = getattr(message, "param_id", "")
        if isinstance(raw, (bytes, bytearray)):
            raw = raw.decode("ascii", "replace")
        name = str(raw).replace("\x00", "").strip().upper()
        spec = BY_NAME.get(name)
        if spec is None:
            return True  # somebody else's parameter, but it was still a PARAM_VALUE
        try:
            value = float(getattr(message, "param_value", 0.0))
        except (TypeError, ValueError):
            return True
        if not math.isfinite(value):
            return True
        self._values[name] = value
        self._seen_at[name] = self._clock()
        self._attempts.pop(name, None)
        if self._active is not None and self._active.spec.name == name:
            self._settle_write(value)
        return True

    def pump(self, master):
        """Send at most one parameter message. Call once per loop tick.

        Writes go before reads: an operator watching a Save should not queue
        behind a background refresh of sixteen other parameters.
        """
        now = self._clock()
        if now >= self._refresh_at:
            # Slow background re-read, and the retry path for anything that has
            # never answered - a BSLD_ parameter appears the moment the script is
            # loaded, and this is what notices.
            self._refresh_at = now + REFRESH_PERIOD_S
            self._attempts.clear()
            self._asked_at.clear()

        if self._service_write(master, now):
            return
        if now - self._last_sent < REQUEST_INTERVAL_S:
            return
        spec = self._next_read(now)
        if spec is None:
            return
        self._last_sent = now
        self._asked_at[spec.name] = now
        self._attempts[spec.name] = self._attempts.get(spec.name, 0) + 1
        try:
            master.mav.param_request_read_send(
                master.target_system,
                master.target_component,
                spec.name.encode("ascii"),
                -1,  # by name, not by index
            )
        except Exception as exc:  # noqa: BLE001 - a dead link is main.py's problem
            self._last_error = f"could not ask for {spec.name}: {exc}"

    def link_down(self):
        """The MAVLink link dropped. Forget everything it was the only source of.

        Same rule as `Navigation.link_down()` and `Trim.link_down()`: a gain still
        shown on the dashboard after the autopilot went away looks like a live
        reading, which is worse than an empty field. Anything queued is failed
        rather than left to be written to a link that no longer exists.
        """
        self._values.clear()
        self._seen_at.clear()
        self._attempts.clear()
        self._asked_at.clear()
        if self._active is not None:
            self._finish(self._active, False, "the MAVLink link dropped")
            self._active = None
        while self._queue:
            self._finish(self._queue.popleft(), False, "the MAVLink link dropped")

    def check_timeout(self):
        """Nothing to do - `pump()` owns the write deadline. Here for symmetry
        with `MissionUploader`, so a reader of `main.py` does not have to wonder
        which of the two conventions applies."""

    def take(self):
        """One finished write's outcome as `(command_id, ok, message)`, or None."""
        if not self._outcomes:
            return None
        return self._outcomes.popleft()

    # -- telemetry -----------------------------------------------------------

    def telemetry(self):
        """The `telemetry.tuning` block the dashboard's tuning panel reads."""
        missing = [
            spec.name
            for spec in TUNABLES
            if spec.name not in self._values
            and self._attempts.get(spec.name, 0) >= MAX_REQUEST_ATTEMPTS
        ]
        block = {
            # Only what the autopilot has actually answered with. A field the
            # dashboard cannot fill is left empty rather than shown as zero.
            "values": dict(self._values),
            "known": len(self._values),
            "of": len(TUNABLES),
            "loading": len(self._values) + len(missing) < len(TUNABLES),
            "queued": len(self._queue),
            "writes": self._writes_ok,
            "write_failures": self._writes_failed,
            # True once any BSLD_ parameter has been seen, which is the only
            # evidence from here that `battery_slider.lua` has run at all.
            "slider_script": any(name in self._values for name in SCRIPT_PARAMS),
        }
        if missing:
            block["missing"] = missing
        if self._active is not None:
            block["pending"] = self._active.spec.name
        if self._last_write:
            block["last_write"] = self._last_write
        if self._last_error:
            block["last_error"] = self._last_error
        return block

    # -- internals -----------------------------------------------------------

    def _next_read(self, now):
        for spec in TUNABLES:
            name = spec.name
            seen = self._seen_at.get(name)
            if seen is not None and now - seen < REFRESH_PERIOD_S:
                continue  # answered recently enough
            asked = self._asked_at.get(name)
            if asked is not None and now - asked < REQUEST_RETRY_S:
                continue  # a request is still in flight
            if self._attempts.get(name, 0) >= MAX_REQUEST_ATTEMPTS:
                # Given up until the next refresh tick. If it has never answered
                # it is reported as missing; if it answered before, the old value
                # is dropped so nothing stale is displayed as live.
                if name in self._values:
                    self._values.pop(name, None)
                    log.warning(
                        "%s stopped answering; dropping it from the tuning panel", name
                    )
                continue
            return spec
        return None

    def _service_write(self, master, now):
        """Progress the write at the head of the queue. True if one is in flight."""
        if self._active is None:
            if not self._queue:
                return False
            self._active = self._queue.popleft()
        write = self._active

        # `attempts` rather than a timestamp decides whether this has been sent at
        # all: `sent_at` starts at 0 and the monotonic clock is small on a freshly
        # booted Pi, which would otherwise read as "sent, still waiting".
        if write.attempts and now - write.sent_at < WRITE_TIMEOUT_S:
            return True  # waiting for the echo
        if write.attempts >= MAX_WRITE_ATTEMPTS:
            self._finish(
                write,
                False,
                f"no PARAM_VALUE echo for {write.spec.name} within "
                f"{WRITE_TIMEOUT_S:.0f}s. Does the autopilot have that "
                "parameter? BSLD_* exist only while battery_slider.lua is "
                "running, and SCR_USER5/6 only on firmware that defines six.",
            )
            self._active = None
            return True  # the next queued write starts on the next tick
        if now - self._last_sent < REQUEST_INTERVAL_S:
            return True  # pacing, not waiting - send on the next tick
        self._send_write(write, master, now)
        return True

    def _send_write(self, write, master, now):
        write.attempts += 1
        write.sent_at = now
        self._last_sent = now
        try:
            master.mav.param_set_send(
                master.target_system,
                master.target_component,
                write.spec.name.encode("ascii"),
                write.value,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
            )
        except Exception as exc:  # noqa: BLE001 - report it, do not raise into the loop
            self._finish(write, False, f"could not send PARAM_SET: {exc}")
            self._active = None
            return
        log.info(
            "PARAM_SET %s = %g (attempt %d)",
            write.spec.name,
            write.value,
            write.attempts,
        )

    def _settle_write(self, stored):
        """The echo arrived. Compare what was stored against what was asked for."""
        write = self._active
        self._active = None
        spec = write.spec
        if spec.integer:
            agrees = round(stored) == round(write.value)
        else:
            agrees = abs(stored - write.value) <= max(1e-4, abs(write.value) * 1e-4)
        if agrees:
            self._finish(write, True, f"{spec.name} = {stored:g}, saved on the autopilot")
            return
        # ArduPilot clamps to its own parameter metadata and rounds integer types,
        # so this is a real answer rather than a comms failure: report the number
        # it kept, because that is what the loop is now running.
        self._finish(
            write,
            False,
            f"the autopilot stored {spec.name} = {stored:g}, not {write.value:g}",
        )

    def _finish(self, write, ok, message):
        if ok:
            self._writes_ok += 1
            self._last_write = message
            log.info("tuning: %s", message)
        else:
            self._writes_failed += 1
            self._last_error = message
            log.warning("tuning: %s", message)
        self._outcomes.append((write.command_id, ok, message))
