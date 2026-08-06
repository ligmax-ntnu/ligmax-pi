"""Who is in charge of this boat right now.

One question, five answers, decided in one place:

    KILLED          the safety loop is open, propulsion power is cut
    REMOTE          a human is steering - RC, or the shore client
    AUTONOMOUS      running on its own navigation
    STANDBY         powered and linked, deliberately not driving
    OUT_OF_CONTROL  nothing is steering it and propulsion has not been cut

This exists as its own module because two things downstream are switched off it
and they must never disagree: the colour of the lights on the hull
(`lights.py`) and the operator's status indicator on the dashboard
(`ligmax-server/web/js/status.js`). One decision, two consumers.

Why OUT_OF_CONTROL is the interesting one
-----------------------------------------
The other four are states somebody chose. This one is a state you *discover*, and
it is the only one that requires the vessel to admit something has gone wrong. The
test is deliberately narrow, because a status that cries wolf gets ignored:

    the boat is out of control when propulsion is *permitted* and no control
    source is answering.

Both halves matter. A boat with the E-stop relay open is KILLED, not out of
control, however lost it is - the thing that makes an uncommanded boat dangerous
is the thrusters, and those are already dead. And a boat that has simply been
disarmed and is sitting there is STANDBY. Nothing here reads "out of control"
unless it could actually move and nobody is telling it where.

What counts as a control source
-------------------------------
    the autopilot        a live MAVLink heartbeat, in a mode that is either
                         steering (AUTO/GUIDED) or accepting a pilot (MANUAL)
    the RC link          RC_CHANNELS still arriving, i.e. FRSky has not dropped.
                         This shares no hardware with the 5G link, which is why
                         it counts independently (docs/architecture.md safety
                         layer 3).
    the shore operator   a command acked recently over the telemetry link

Losing all three at once, while armed, is the case this reports. Losing the 5G
link alone is not: the autopilot is still flying the boat and the RC still works,
which is the entire point of the layering.

A caveat worth stating plainly: this module infers control state from MAVLink and
has **not been tested against the hardware**. The mode-name lists below are
ArduPilot Rover's and want checking against what the vehicle actually reports
before the colour of the hull is trusted in a race.
"""

import logging
import os
import time

log = logging.getLogger("io_manager.status")

# All five values, in the order `ligmax-server/ligmax_gui/protocol.py` lists them.
AUTONOMOUS = "AUTONOMOUS"
REMOTE = "REMOTE"
STANDBY = "STANDBY"
OUT_OF_CONTROL = "OUT_OF_CONTROL"
KILLED = "KILLED"

# ArduPilot Rover mode names, from the flight-controller's HEARTBEAT. Two lists,
# because the distinction they draw is exactly the REMOTE/AUTONOMOUS one:
# whether the vehicle is steering itself or following a stick.
#
# Not verified against this vehicle's parameters - `test.py` is the scratchpad
# where that would get checked. If the boat reports a mode absent from both lists
# it lands in STANDBY (or OUT_OF_CONTROL if armed), which is the conservative
# reading, and the mode name goes into telemetry so the gap is visible.
AUTONOMOUS_MODES = {
    "AUTO",
    "GUIDED",
    "RTL",
    "SMART_RTL",
    "LOITER",
    "FOLLOW",
    "DOCK",
    "CIRCLE",
}
PILOTED_MODES = {"MANUAL", "ACRO", "STEERING", "SIMPLE"}
# Modes that hold station or do nothing on purpose. Holding *is* a decision, so it
# is standby rather than autonomy - nothing is navigating.
IDLE_MODES = {"HOLD", "INITIALISING", "INITIALIZING"}

# How stale a signal may be before it stops counting as a live control source.
# The autopilot heartbeat is nominally 1 Hz; RC_CHANNELS arrives at the requested
# stream rate. Both are generous on purpose: one missed message is not a lost link,
# and this decides the colour of the hull.
HEARTBEAT_TIMEOUT_S = float(os.environ.get("LIGMAX_STATUS_HEARTBEAT_TIMEOUT_S", "4.0"))
RC_TIMEOUT_S = float(os.environ.get("LIGMAX_STATUS_RC_TIMEOUT_S", "3.0"))
OPERATOR_TIMEOUT_S = float(os.environ.get("LIGMAX_STATUS_OPERATOR_TIMEOUT_S", "15.0"))

# An uncommanded boat is not declared out of control the instant a link blinks.
# It has to stay that way for this long, which stops a momentary gap flashing the
# hull to a red strobe and back.
LOST_CONFIRM_S = float(os.environ.get("LIGMAX_STATUS_LOST_CONFIRM_S", "2.0"))


class StatusMachine:
    """Tracks control sources and reports one of the five statuses.

    Fed from the MAVLink pump in `main.py`; `evaluate()` is pure bookkeeping and
    cheap enough to call every loop. Nothing here talks to hardware, which is what
    makes it testable without a boat.
    """

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self.mode = None  # the autopilot's own mode name
        self.armed = None  # from HEARTBEAT base_mode, None until first seen
        self._heartbeat_at = 0.0
        self._rc_at = 0.0
        self._operator_at = 0.0
        self._lost_since = None
        self._status = None
        self._changed_at = 0.0
        self._reason = "no telemetry from the autopilot yet"

    # -- fed by the MAVLink pump -------------------------------------------

    def note_heartbeat(self, mode=None, armed=None):
        """A HEARTBEAT from the autopilot. `mode` is its mode name, `armed` a bool."""
        self._heartbeat_at = self._clock()
        if mode is not None:
            self.mode = str(mode).upper()
        if armed is not None:
            self.armed = bool(armed)

    def note_rc(self):
        """RC_CHANNELS arrived, so the FRSky link is up."""
        self._rc_at = self._clock()

    def note_operator(self):
        """An operator command reached us over the telemetry link."""
        self._operator_at = self._clock()

    def note_link_down(self):
        """The MAVLink link dropped. Clears what only the autopilot could tell us."""
        self._heartbeat_at = 0.0
        self._rc_at = 0.0
        self.mode = None
        self.armed = None

    # -- the decision -------------------------------------------------------

    def _fresh(self, stamp, window):
        return bool(stamp) and (self._clock() - stamp) < window

    @property
    def autopilot_up(self):
        return self._fresh(self._heartbeat_at, HEARTBEAT_TIMEOUT_S)

    @property
    def rc_up(self):
        return self._fresh(self._rc_at, RC_TIMEOUT_S)

    @property
    def operator_up(self):
        return self._fresh(self._operator_at, OPERATOR_TIMEOUT_S)

    def evaluate(self, estop_engaged, propulsion_permitted=None):
        """Return the current status.

        `estop_engaged` is `EstopRelay.engaged`. `propulsion_permitted` defaults to
        its inverse and exists so a caller that knows better - a physical E-stop
        the Pi cannot see, say - can say so.
        """
        if propulsion_permitted is None:
            propulsion_permitted = not estop_engaged

        status, reason = self._classify(estop_engaged, propulsion_permitted)

        # OUT_OF_CONTROL has to persist before it is believed, so a one-second
        # gap in the heartbeat does not strobe the hull red and back again.
        now = self._clock()
        if status == OUT_OF_CONTROL:
            if self._lost_since is None:
                self._lost_since = now
            if (now - self._lost_since) < LOST_CONFIRM_S:
                # Hold the previous status while we wait to be sure, unless there
                # has never been one - in which case admit we do not know.
                held = self._status or OUT_OF_CONTROL
                if held != OUT_OF_CONTROL:
                    waited = now - self._lost_since
                    reason = (
                        f"holding {held} for another "
                        f"{LOST_CONFIRM_S - waited:.1f} s before believing it: {reason}"
                    )
                status = held
        else:
            self._lost_since = None

        if status != self._status:
            log.warning("vessel status: %s -> %s (%s)", self._status, status, reason)
            self._status = status
            self._changed_at = now
        self._reason = reason
        return status

    def _classify(self, estop_engaged, propulsion_permitted):
        """The rules, in priority order. Returns `(status, reason)`."""
        # 1. The safety loop wins. Red means propulsion is cut, and nothing about
        #    how confused the boat is changes that.
        if estop_engaged:
            return KILLED, "emergency stop engaged, propulsion power cut"
        if not propulsion_permitted:
            return KILLED, "propulsion power is not permitted"

        # 2. Disarmed is a choice, and a disarmed boat cannot run away.
        if self.armed is False:
            return STANDBY, "disarmed"

        # 3. Who is steering? The autopilot's mode is the only thing that
        #    distinguishes "navigating itself" from "following a stick".
        if self.autopilot_up:
            if self.mode in PILOTED_MODES:
                # A piloted mode with no RC and no operator is a boat waiting for
                # sticks that are not coming. Armed, that is out of control.
                if self.rc_up or self.operator_up:
                    return REMOTE, f"autopilot in {self.mode}, pilot link up"
                if self.armed:
                    return (
                        OUT_OF_CONTROL,
                        f"autopilot in {self.mode} but no RC and no operator",
                    )
                return STANDBY, f"autopilot in {self.mode}, no pilot link"
            if self.mode in AUTONOMOUS_MODES:
                return AUTONOMOUS, f"autopilot navigating in {self.mode}"
            if self.mode in IDLE_MODES:
                return STANDBY, f"autopilot holding in {self.mode}"
            # A mode neither list knows. Do not guess it is autonomous.
            if self.armed:
                return OUT_OF_CONTROL, f"autopilot in unrecognised mode {self.mode}"
            return STANDBY, f"autopilot in unrecognised mode {self.mode}"

        # 4. No autopilot. The RC link shares no hardware with 5G or with the
        #    companion computer, so if it is up a human still has the boat even
        #    though we cannot see the mode.
        if self.rc_up:
            return REMOTE, "no autopilot heartbeat, but the RC link is up"

        # 5. Nothing is answering. Armed, this is the state the strobe is for.
        #    `armed is False` already returned at step 2, so it is True or None.
        if self.armed:
            return OUT_OF_CONTROL, "no autopilot, no RC, no operator, and armed"
        # Never heard from the autopilot at all - on the bench that is just "not
        # plugged in", and propulsion cannot be commanded without it anyway.
        return STANDBY, "no autopilot link yet, arming state unknown"

    # -- what goes up the link ---------------------------------------------

    def telemetry(self):
        """The `telemetry.control` block: why the status is what it is.

        The status itself is a top-level frame field, not part of this - it drives
        the indicator and belongs beside `mode` and `estop`. What is here is the
        evidence, so an operator looking at a red strobe can see which of the three
        links went away.
        """
        return {
            "status_reason": self._reason,
            "autopilot_link": self.autopilot_up,
            "rc_link": self.rc_up,
            "operator_link": self.operator_up,
            "autopilot_mode": self.mode or "unknown",
            "armed": self.armed,
            "for_s": round(self._clock() - self._changed_at, 1) if self._status else None,
        }
