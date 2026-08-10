"""The operator's own go-to, and the speed cap it travels at.

Two dashboard commands live here - `goto` and `set_speed_limit` - because they
are one behaviour with two halves. `set_speed_limit` is the ground speed a
`goto` then uses, and the same figure goes to the autopilot as
`MAV_CMD_DO_CHANGE_SPEED`, so a mission uploaded by `mission.py` and flown in
AUTO obeys it too. Both were advertised by the dashboard from the start and
neither existed on the vessel until 2026-08-10: they acked
`'goto' is not implemented on the vessel` (docs/findings.md).

**This is the hand-flown route, not the autonomy node's.** `nodes/self_driving`
plans its own course, picks a speed per behaviour and enforces its own ceiling
(`commander.ceiling`, careful mode, `config.SPEED_LIMIT_KNOTS`); nothing here
changes any of that, and nothing here is consulted when it is driving. What this
is for is the operator clicking a point on the chart with the planner out of the
picture - moving the boat off a start line, closing on a mark by hand, or
checking that GUIDED steers at all before trusting a plan to it.

Three things worth knowing before changing this:

* **The cap is a ceiling under a ceiling.** `VESSEL_SPEED_LIMIT_MS` is NJORD's
  5 knots and lives in `config.py`; a value above it is *refused* rather than
  clamped, because an operator who typed 4 m/s and got 2.57 without being told
  would believe the boat was doing 4. Nothing here can raise the vessel limit -
  raising that is a commit, as `config.py` says.

* **A `goto` is not a mission.** It is a single `SET_POSITION_TARGET_GLOBAL_INT`
  in GUIDED, held by the autopilot until it arrives or is given something else.
  It is not stored, it does not survive a mode change, and it does not appear in
  the chart's ideal-route layer the way `set_mission` does. `main.py` refuses it
  outside GUIDED rather than switching mode itself: a command that silently
  changes flight mode is how a boat ends up navigating when somebody meant to
  nudge it.

* **Neither half is acked by the autopilot.** Same honesty caveat as
  `apply_mode()`: what these functions can promise is that the message went out.
  The confirmation is the boat moving on the chart, and `motion.sog` staying
  under the cap.
"""

import logging
import math

from pymavlink import mavutil

from config import KNOT_MS, VESSEL_SPEED_LIMIT_KNOTS, VESSEL_SPEED_LIMIT_MS

log = logging.getLogger(__name__)

# The slowest cap worth having. Below this the boat cannot hold heading against
# any wind at all, so a value under it is a typo rather than a request - and the
# dashboard's own input carries the same floor.
MIN_LIMIT_MS = 0.2

# The only flight mode that acts on a position target. `main.py` refuses a go-to
# in anything else rather than switching mode on the operator's behalf.
MODE = "GUIDED"

# Refused above this. NJORD's 5 knots, from config.py, which is also what
# `autopilot_bridge.py` clamps the autonomy node's commands to.
MAX_LIMIT_MS = VESSEL_SPEED_LIMIT_MS

# How far from the grid origin a go-to may land. The Njord course is a few
# hundred metres across; a click that resolves to kilometres is a mis-scaled
# chart or a stale origin, and sending the boat there is worse than refusing.
MAX_RANGE_M = 2000.0

# SET_POSITION_TARGET_GLOBAL_INT's type_mask: use the position fields, ignore
# velocity, acceleration, yaw and yaw rate. The speed the boat travels at is
# whatever DO_CHANGE_SPEED last set, which is the whole reason the cap and the
# go-to are one module.
POSITION_ONLY = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
    | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

# DO_CHANGE_SPEED's param1. 1 is ground speed, which is the only kind that means
# anything on a boat.
SPEED_TYPE_GROUND = 1

# MAV_RESULT in the operator's words, for the one ack this module reads. Same
# table `preflight.py` keeps, trimmed to the results DO_CHANGE_SPEED can give.
_RESULT_WORDS = {
    mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED: (
        "temporarily rejected - the autopilot is busy, send it again"
    ),
    mavutil.mavlink.MAV_RESULT_DENIED: "denied by the autopilot",
    mavutil.mavlink.MAV_RESULT_UNSUPPORTED: (
        "unsupported - this firmware does not take DO_CHANGE_SPEED"
    ),
    mavutil.mavlink.MAV_RESULT_FAILED: "failed on the autopilot",
}


def knots(speed_ms):
    return speed_ms / KNOT_MS


class Guided:
    """The standing speed cap, and one-shot go-to targets in GUIDED.

    Holds no timers and refreshes nothing: unlike the ride-height override,
    neither of these expires, so there is nothing to keep alive. The cap is
    remembered only so it can be reported and reapplied ahead of the next
    go-to - the autopilot is the one storing it.
    """

    def __init__(self, limit_ms=MAX_LIMIT_MS):
        # Starts at the vessel limit, so until an operator asks for something
        # slower this changes nothing about how the boat behaves.
        self.limit = float(limit_ms)
        self._told_autopilot = False
        self._refused = None  # the autopilot's own words, if it refused the cap
        self._target = None  # last go-to, as words for the telemetry block

    # -- the cap ------------------------------------------------------------

    def set_limit(self, master, value):
        """Set the go-to / AUTO ground speed cap. Returns `(ok, message)`."""
        try:
            number = float(value)
        except (TypeError, ValueError):
            return False, f"'value' is a speed in m/s, not {value!r}"
        if not math.isfinite(number):
            return False, "'value' must be a finite number of m/s"
        if not MIN_LIMIT_MS <= number <= MAX_LIMIT_MS:
            # Refused, not clamped - see the module docstring.
            return False, (
                f"'value' must be {MIN_LIMIT_MS:g}..{MAX_LIMIT_MS:.2f} m/s: "
                f"{VESSEL_SPEED_LIMIT_KNOTS:g} knots is the vessel limit and "
                "nothing from the dashboard can raise it"
            )

        if not self._send_limit(master, number):
            return False, "the speed cap could not be sent - see the vessel log"
        self.limit = number
        log.warning(
            "speed cap now %.2f m/s (%.2f kn) for go-to and AUTO", number, knots(number)
        )
        return True, (
            f"speed cap {number:.2f} m/s ({knots(number):.2f} kn) for go-to and "
            "AUTO - the autonomy node keeps its own ceiling"
        )

    def _send_limit(self, master, speed_ms):
        """DO_CHANGE_SPEED. True if the message went out."""
        try:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0,  # confirmation
                SPEED_TYPE_GROUND,
                float(speed_ms),
                -1,  # throttle: no change
                0, 0, 0, 0,
            )
        except Exception:
            # A dropped link must not take down the loop that owes the autopilot
            # its heartbeat. The operator gets a refusal and can press again.
            log.exception("speed cap: DO_CHANGE_SPEED send failed")
            return False
        self._told_autopilot = True
        return True

    def note_ack(self, message):
        """Read a COMMAND_ACK, in case it is the speed cap's. Cheap and silent.

        The cap is acked to the operator at the point of sending, like
        `apply_mode()`, because it is one message and the answer arrives after
        the reply this node is already building. That is fine for "denied" -
        press it again - and not fine for **unsupported**, which is a firmware
        that will keep running the boat at whatever `WP_SPEED` holds while the
        dashboard shows a cap in force. So the result is not thrown away: a
        refusal is logged at WARN, which reaches the operator's log panel, and
        `speed_limit_refused` rides up beside the figure so the panel can
        contradict itself out loud rather than quietly.

        Not a `Preflight`-style waiter on purpose. That machinery holds one
        command at a time and would make a go-to's cap and a dockside safety
        switch queue behind each other on a link where only one of them is time
        critical.
        """
        if getattr(message, "get_type", None) is None:
            return
        if message.get_type() != "COMMAND_ACK":
            return
        if getattr(message, "command", None) != mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED:
            return
        result = int(getattr(message, "result", mavutil.mavlink.MAV_RESULT_FAILED))
        if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self._refused = None
            return
        if result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
            return
        why = _RESULT_WORDS.get(result, f"MAV_RESULT {result}")
        self._refused = why
        log.warning(
            "speed cap: the autopilot answered %s - the boat is running at "
            "whatever WP_SPEED holds, not %.2f m/s",
            why,
            self.limit,
        )

    # -- the go-to ----------------------------------------------------------

    def goto(self, master, navigation, x, y):
        """Send the vessel to a grid point. Returns `(ok, message)`.

        `x`/`y` are grid metres east and north - the frame the chart is drawn in
        and the one `set_mission` takes - converted here against the origin
        `navigation` captured. The cap is re-sent immediately before the target,
        so a go-to never inherits a speed set by something else.
        """
        try:
            east, north = float(x), float(y)
        except (TypeError, ValueError):
            return False, "'x' and 'y' are grid metres and must be numbers"
        if not (math.isfinite(east) and math.isfinite(north)):
            return False, "'x' and 'y' must be finite numbers of metres"
        if max(abs(east), abs(north)) > MAX_RANGE_M:
            return False, (
                f"refused: {east:.0f}, {north:.0f} m is further than "
                f"{MAX_RANGE_M:g} m from the grid origin"
            )

        target = navigation.to_global(east, north)
        if target is None:
            return False, "no GPS origin yet - the grid is not georeferenced"
        lat, lon = target

        if not self._send_limit(master, self.limit):
            return False, "the speed cap could not be sent, so no target was"
        try:
            master.mav.set_position_target_global_int_send(
                0,  # time_boot_ms, unused
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                POSITION_ONLY,
                int(round(lat * 1e7)),
                int(round(lon * 1e7)),
                0.0,  # altitude: ignored by a boat, and relative to home anyway
                0, 0, 0,  # velocity, ignored by the mask
                0, 0, 0,  # acceleration, ignored by the mask
                0, 0,  # yaw, yaw rate, ignored by the mask
            )
        except Exception:
            log.exception("go-to: position target send failed")
            return False, "the target could not be sent - see the vessel log"

        # Words rather than a pair of numbers: the chart does not draw this, and
        # the dashboard's generic panel would otherwise show it as two unrelated
        # rows called "Goto target 0" and "Goto target 1".
        self._target = f"{east:.1f}, {north:.1f} m"
        log.warning(
            "go-to: %.1f, %.1f m (%.7f, %.7f) at up to %.2f m/s",
            east, north, lat, lon, self.limit,
        )
        return True, (
            f"go-to {east:.1f}, {north:.1f} m sent at up to {self.limit:.2f} m/s "
            "- watch the chart, GUIDED holds this target until it arrives"
        )

    # -- what goes up the link ----------------------------------------------

    def telemetry(self):
        """Merged into the frame's `telemetry.control` block by `main.py`."""
        block = {
            "speed_limit_ms": round(self.limit, 2),
            "speed_limit_kn": round(knots(self.limit), 2),
            # False means the figure beside it is this node's default and the
            # autopilot has never been told anything - so whatever WP_SPEED holds
            # is what an AUTO mission will actually do.
            "speed_limit_sent": self._told_autopilot,
        }
        if self._refused is not None:
            # The cap did not take. Present only when that has happened, so the
            # panel shows nothing at all in the ordinary case (see note_ack()).
            block["speed_limit_refused"] = self._refused
        if self._target is not None:
            block["goto_target"] = self._target
        return block
