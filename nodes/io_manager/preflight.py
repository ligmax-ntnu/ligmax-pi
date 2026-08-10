"""The two dockside actions that go straight to the flight controller.

    the safety switch     the physical button on the Pixhawk, pressed from here
    the compass swing     ArduPilot's large-vehicle mag cal, one known heading

Neither is a thing this node does on its own initiative. Both are an operator on
a dock pressing a button on `/control`, and both change something on the flight
controller rather than on this Pi - which is why they live together and why they
are the only commands in this node that wait for the autopilot's own answer
before acking.

Why these wait, when `set_mode` and `arm` do not
-----------------------------------------------
`apply_mode()` and `apply_arm()` in `main.py` ack "sent, not confirmed", and say
so, because the real confirmation arrives a moment later in the HEARTBEAT and the
operator is already watching it. Neither of these has that. Nothing in ArduPilot's
telemetry says where the safety switch is or whether a compass swing took, so an
ack that meant "the message left the Pi" would be the *only* answer the operator
ever got - and for the safety switch that answer would be a dangerous lie in one
specific direction: `force_safety_on()` can refuse (a board with no safety switch
fitted returns false), and an operator who has been told "acked" believes the
motor outputs are inhibited when they are live.

So both go out as a MAV_CMD and are acked from the COMMAND_ACK, the same
deferred shape `mission.py` and `tuning.py` already use: `handle()` feeds the
result in, `take()` hands `main.py` the outcome to ack against the operator's
command id. One at a time, deliberately - two of these in flight at once would
be two COMMAND_ACKs to tell apart on a link a second GCS may also be using.

What `safety_switch` in telemetry is, and is not
-----------------------------------------------
It is **what this node last commanded and the autopilot accepted**. It is not a
read-back: ArduPilot does not report the switch state in SYS_STATUS or anywhere
else, so nothing here can see somebody walking up to the boat and pressing the
button, and nothing here survives a reboot of the Pixhawk. "unknown" is the
honest starting value and the one it returns to when the link drops. The
dashboard says as much next to the buttons; if that ever stops being true because
a future ArduPilot does publish the state, this is the one method to change.
"""

import logging
import math
import time

from pymavlink import mavutil

log = logging.getLogger(__name__)

# What `telemetry()["safety_switch"]` can say. "on" is the safe state - the
# button not pressed, motor outputs inhibited - which is the opposite of what
# "safety on" sounds like to anyone who has not used a Pixhawk, hence the words
# the dashboard puts beside it.
SAFETY_ON = "on"
SAFETY_OFF = "off"
SAFETY_UNKNOWN = "unknown"

# How long the autopilot gets to answer before the operator is told it did not.
# ArduPilot answers both of these in well under a second on a healthy link; this
# is generous enough that a lumpy 115200 serial link does not produce false
# "no answer" acks, and short enough that the dashboard row does not sit at
# "sent" long enough for anyone to press the button again.
ACK_TIMEOUT_S = 3.0

# MAV_RESULT, in the operator's words rather than the enum's. Anything not
# listed is reported by number, which is still better than silence.
_RESULT_WORDS = {
    mavutil.mavlink.MAV_RESULT_ACCEPTED: "accepted",
    mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED: (
        "temporarily rejected - the autopilot is busy, try again"
    ),
    mavutil.mavlink.MAV_RESULT_DENIED: "denied by the autopilot",
    mavutil.mavlink.MAV_RESULT_UNSUPPORTED: (
        "unsupported - this firmware does not have this command"
    ),
    mavutil.mavlink.MAV_RESULT_FAILED: "failed on the autopilot",
}


class _Pending:
    """One command sent, waiting for its COMMAND_ACK."""

    def __init__(self, command_id, command, params, label, deadline, settle):
        self.command_id = command_id
        self.command = command
        self.params = params  # param1..param7, as sent
        self.label = label
        self.deadline = deadline
        self.settle = settle  # called with the accepted result, to record state
        self.retried_as_int = False


class Preflight:
    """The safety switch and the compass swing, each acked by the autopilot.

    Fed from the MAVLink pump in `main.py` (`handle()`), drained by it
    (`take()`), and told when the link goes away (`link_down()`). Nothing here
    talks to hardware directly, so it is testable without a boat.
    """

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._pending = None
        self._outcomes = []
        self._safety = SAFETY_UNKNOWN
        self._safety_at = None  # wall clock, for the dashboard
        self._compass = None  # the last swing's result, whatever it was

    @property
    def busy(self):
        return self._pending is not None

    # -- the two commands ---------------------------------------------------

    def set_safety(self, master, command_id, safe):
        """Press the Pixhawk's safety switch. `safe=True` inhibits the outputs.

        Returns `(started, message)`. `started` True means the command is on the
        wire and the caller must NOT ack it - `take()` will produce the outcome
        once the autopilot answers. False means it never went, and `message` is
        why, ready to be acked as a failure.

        ArduPilot handles this by calling `force_safety_on()`/`force_safety_off()`
        on the board directly, so it bypasses BRD_SAFETYOPTION - the parameter
        that gates what the *physical* button is allowed to do. It needs
        ArduPilot 4.3 or newer; older firmware answers UNSUPPORTED, which comes
        back to the operator in those words.
        """
        state = (
            mavutil.mavlink.SAFETY_SWITCH_STATE_SAFE
            if safe
            else mavutil.mavlink.SAFETY_SWITCH_STATE_DANGEROUS
        )
        word = SAFETY_ON if safe else SAFETY_OFF

        def settle():
            self._safety = word
            self._safety_at = time.time()
            log.warning(
                "safety switch is now %s - motor outputs %s",
                word,
                "inhibited" if safe else "LIVE",
            )

        return self._send(
            master,
            command_id,
            mavutil.mavlink.MAV_CMD_DO_SET_SAFETY_SWITCH_STATE,
            (float(state), 0, 0, 0, 0, 0, 0),
            label=f"safety switch {word}",
            settle=settle,
        )

    def compass_cal(self, master, command_id, heading_deg, position=None):
        """ArduPilot's large-vehicle mag cal: one known heading, no tumbling.

        Same `(started, message)` contract as `set_safety()`.

        `MAV_CMD_DO_START_MAG_CAL` - the calibration every quadcopter pilot has
        done - wants the whole vehicle rotated through all three axes, which for
        a boat in the water is not a thing anybody is going to do. This is the
        other one ArduPilot offers: point the hull along a heading you know from
        something that is not the compass, say so, and the autopilot solves the
        offsets against the world magnetic model for where it is standing.
        Less accurate than a tumble, and the only kind available here.

        Two things it needs and this checks first, because ArduPilot answers
        both with a bare FAILED:

        * a position, for the magnetic model. `position` is `navigation`'s, which
          is read off the autopilot's own GLOBAL_POSITION_INT - so having one is
          evidence the autopilot has one. Nothing is sent on the wire for it:
          lat/lon go out as 0, which tells ArduPilot to use its own fix rather
          than a copy of it that has been through a float32.
        * a heading. It must be TRUE heading, in degrees, and it must come from
          something other than the compass being calibrated - a handheld bearing,
          a known line on the dock, or the GNSS course over ground with the boat
          making way. The dashboard offers the last of those and says why.
        """
        try:
            heading = float(heading_deg)
        except (TypeError, ValueError):
            return False, "'heading' must be a number, in degrees true"
        if not math.isfinite(heading):
            return False, "'heading' must be a real number of degrees"
        heading %= 360.0

        if position is None:
            return False, (
                "refused: no GNSS position, and the magnetic model needs one - "
                "wait for a fix"
            )

        def settle():
            self._compass = {
                "at": round(time.time(), 1),
                "heading_deg": round(heading, 1),
                "lat": round(position[0], 6),
                "lon": round(position[1], 6),
            }
            log.warning(
                "compass offsets rewritten by a large-vehicle calibration at "
                "%.1f deg true - check the heading on the chart before trusting it",
                heading,
            )

        return self._send(
            master,
            command_id,
            mavutil.mavlink.MAV_CMD_FIXED_MAG_CAL_YAW,
            # param1 yaw, param2 compass bitmask (0 = every compass fitted),
            # param3/4 lat/lon (0 = wherever the autopilot thinks it is).
            (heading, 0, 0, 0, 0, 0, 0),
            label=f"compass calibration at {heading:.1f} deg",
            settle=settle,
        )

    # -- the wire -----------------------------------------------------------

    def _send(self, master, command_id, command, params, label, settle):
        if master is None:
            return False, "no autopilot link"
        if self._pending is not None:
            return False, f"refused: waiting on the {self._pending.label} already sent"
        if command_id is None:
            return False, f"{label} needs a command id to ack against"

        try:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                command,
                0,  # confirmation
                *params,
            )
        except Exception as exc:  # noqa: BLE001 - a dropped link is the loop's problem
            log.exception("%s: send failed", label)
            return False, f"could not reach the autopilot: {exc}"

        self._pending = _Pending(
            command_id=str(command_id),
            command=command,
            params=params,
            label=label,
            deadline=self._clock() + ACK_TIMEOUT_S,
            settle=settle,
        )
        log.info("%s sent, waiting for the autopilot to answer", label)
        return True, f"{label} sent"

    def _resend_as_int(self, master):
        """Answer to MAV_RESULT_COMMAND_LONG_ONLY / _COMMAND_INT_ONLY.

        ArduPilot has been moving commands from COMMAND_LONG to COMMAND_INT one
        release at a time, and a command that has made the move answers a LONG
        with COMMAND_INT_ONLY rather than doing it. That would reach the operator
        as a bare refusal on a dock with no way to tell it from a real one, so it
        is retried in the other shape once, silently. Both commands here carry
        their arguments in param1..4 either way; x/y/z are the fields that differ
        and neither uses them.
        """
        pending = self._pending
        if pending is None or pending.retried_as_int or master is None:
            return False
        try:
            master.mav.command_int_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_FRAME_GLOBAL,
                pending.command,
                0,  # current
                0,  # autocontinue
                *pending.params[:4],
                0,  # x
                0,  # y
                0,  # z
            )
        except Exception:  # noqa: BLE001
            log.exception("%s: COMMAND_INT retry failed", pending.label)
            return False
        pending.retried_as_int = True
        pending.deadline = self._clock() + ACK_TIMEOUT_S
        log.info("%s: firmware wants COMMAND_INT, resent", pending.label)
        return True

    def handle(self, master, message):
        """Feed one MAVLink message in. True if it was ours and is now consumed.

        Matched on the command number, which is enough because only one of these
        is ever in flight. It does mean an ack for the *same* command from a
        second GCS on this link - somebody in Mission Planner - would be read as
        the answer to ours. That is a bench scenario, and the alternative is
        tracking a field ArduPilot only fills in on newer firmware.
        """
        if message.get_type() != "COMMAND_ACK":
            return False
        pending = self._pending
        if pending is None or getattr(message, "command", None) != pending.command:
            return False

        result = int(getattr(message, "result", mavutil.mavlink.MAV_RESULT_FAILED))

        if result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
            # "Working on it." Restart the clock rather than time out underneath
            # a calibration that is genuinely running.
            pending.deadline = self._clock() + ACK_TIMEOUT_S
            return True

        if result in (
            mavutil.mavlink.MAV_RESULT_COMMAND_INT_ONLY,
            mavutil.mavlink.MAV_RESULT_COMMAND_LONG_ONLY,
        ) and self._resend_as_int(master):
            return True

        if result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            pending.settle()
            self._outcomes.append((pending.command_id, True, f"{pending.label}: done"))
        else:
            why = _RESULT_WORDS.get(result, f"MAV_RESULT {result}")
            log.warning("%s: %s", pending.label, why)
            self._outcomes.append((pending.command_id, False, f"{pending.label}: {why}"))
        self._pending = None
        return True

    def check_timeout(self):
        """Fail a command the autopilot never answered. Cheap when idle."""
        pending = self._pending
        if pending is None or self._clock() < pending.deadline:
            return
        self._pending = None
        log.warning("%s: no COMMAND_ACK in %.0f s", pending.label, ACK_TIMEOUT_S)
        self._outcomes.append(
            (
                pending.command_id,
                False,
                f"{pending.label}: the autopilot never answered - state unchanged "
                "as far as this node knows",
            )
        )

    def link_down(self):
        """The MAVLink link dropped.

        Two things go, for the same reason: an ack that can no longer arrive must
        not leave the operator's row at "sent", and a safety-switch state we
        believed a moment ago says nothing about a flight controller we can no
        longer see - it may have rebooted, which puts the switch back where
        BRD_SAFETY_DEFLT says.
        """
        self.check_timeout()
        if self._pending is not None:
            pending = self._pending
            self._pending = None
            self._outcomes.append(
                (pending.command_id, False, f"{pending.label}: the autopilot link dropped")
            )
        self._safety = SAFETY_UNKNOWN
        self._safety_at = None

    def take(self):
        """The next settled outcome as `(command_id, ok, message)`, or None."""
        if not self._outcomes:
            return None
        return self._outcomes.pop(0)

    # -- what goes up the link ----------------------------------------------

    def telemetry(self):
        """`telemetry.preflight` - see the module docstring on what this is worth.

        `safety_switch_seen` is the field that keeps this honest: false means the
        word beside it is a default, not an observation, and the dashboard greys
        it out rather than claiming the outputs are inhibited.
        """
        block = {
            "safety_switch": self._safety,
            "safety_switch_seen": self._safety != SAFETY_UNKNOWN,
            "pending": self._pending.label if self._pending else None,
        }
        if self._safety_at is not None:
            block["safety_switch_at"] = round(self._safety_at, 1)
        if self._compass is not None:
            block["compass_cal"] = dict(self._compass)
        return block
