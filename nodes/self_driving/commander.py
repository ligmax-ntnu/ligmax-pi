"""Intent -> MAVLink. The only module that knows how the boat is actually driven.

A behaviour never talks to the autopilot. It returns an **intent** - "go to this
point at this speed", "creep forward at 0.2 m/s while turning 5 deg/s", "stop" -
and this module turns that into the messages `io_manager` puts on the MAVLink
link. Two reasons that separation is worth a file:

  * a behaviour becomes testable without a boat, because its output is a small
    value object rather than a side effect on a serial port;
  * the day the control seam changes - a different GUIDED message, a different
    ArduPilot version, a fallback to RC override - exactly one file changes.

The seam
--------
**GUIDED, with the Pi as navigator and ArduPilot as helmsman.** Two commands
cover everything the Njord tasks need:

    position target   `SET_POSITION_TARGET_GLOBAL_INT`, position bits only.
                      ArduPilot's own L1 controller steers to it, using the
                      steering and throttle tune that is already on the vehicle.
                      Used for every transit leg, because that tune is better
                      than anything written here would be.

    body velocity     `SET_POSITION_TARGET_LOCAL_NED` in `MAV_FRAME_BODY_NED`,
                      velocity and yaw-rate bits. Used for docking, station
                      keeping and avoidance micro-adjustments - anything where
                      the boat has to creep, reverse, pivot or crab, and where a
                      position target's "drive there" behaviour is the wrong
                      shape entirely.

Both are re-sent every `TARGET_REFRESH_S` even when unchanged, because ArduPilot
times a guided command out and falls back to holding. That timeout is a safety
feature and this is how you stay on the right side of it: a planner that dies
mid-leg stops refreshing, and the boat stops.

Sideways thrust
---------------
The boat has two main thrusters (one per ama) and a third, sideways-only unit.
Which route reaches it depends on how the flight controller is set up, so both
exist and `config.LATERAL_MODE` picks:

    mavlink   the `vy` term of the body-velocity command. If ArduPilot is
              configured with a lateral motor output it drives it; if it is not,
              it silently drops the term. **That is why this is the default** -
              it is the only option that is harmless when unverified.
    rc        an RC override on `LATERAL_RC_CHAN`, the way
              `io_manager/pixhalwk.py` drives the ride height on channel 14.
              Refused unless a channel is configured, because guessing a channel
              number means driving something else on the boat.
    none      no lateral thrust; `dock.py` falls back to an angled approach.

Sign conventions, which are the thing to get right
--------------------------------------------------
Body frame here is **+x forward, +y starboard** - the same one every sensor
return uses (`geo.py`). MAVLink's `MAV_FRAME_BODY_NED` is +x forward, +y RIGHT,
+z DOWN, which is the same for the two axes that matter, so nothing is flipped
on the way out. Yaw rate is **radians per second, positive to starboard**.
"""

from __future__ import annotations

import logging
import math
import time

from . import geo
from .config import (
    ALTERNATION_DEFAULT,
    LATERAL_MAX_MS,
    LATERAL_MODE,
    LATERAL_RC_CENTRE,
    LATERAL_RC_CHAN,
    LATERAL_RC_SPAN,
    KNOT_MS,
    SPEED_LIMIT_KNOTS,
    SPEED_LIMIT_MS,
    SPEED_MIN_MS,
    SPEED_MS,
    TARGET_REFRESH_S,
    YAW_DEADBAND_DEG,
    YAW_MAX_RATE,
    YAW_P,
)

log = logging.getLogger("self_driving.commander")

GOTO = "goto"
VELOCITY = "velocity"
STOP = "stop"
IDLE = "idle"


class Intent:
    """What a behaviour wants this tick. A value object; never mutated.

    Build one with the constructors below rather than directly - they are what
    document which fields go with which kind.
    """

    __slots__ = ("kind", "target", "speed", "vx", "vy", "yaw_rate", "reason")

    def __init__(self, kind, reason, target=None, speed=None, vx=0.0, vy=0.0,
                 yaw_rate=0.0):
        self.kind = kind
        self.reason = reason
        self.target = target
        self.speed = speed
        self.vx = vx
        self.vy = vy
        self.yaw_rate = yaw_rate

    def __repr__(self):
        if self.kind == GOTO:
            return f"<goto {self.target} @{self.speed:.2f} m/s: {self.reason}>"
        if self.kind == VELOCITY:
            return (
                f"<vel fwd={self.vx:+.2f} stbd={self.vy:+.2f} "
                f"yaw={math.degrees(self.yaw_rate):+.0f}deg/s: {self.reason}>"
            )
        return f"<{self.kind}: {self.reason}>"

    def telemetry(self):
        block = {"intent": self.kind, "reason": self.reason}
        if self.kind == GOTO:
            block["target"] = [round(self.target[0], 2), round(self.target[1], 2)]
            block["speed_cmd"] = round(self.speed, 2)
        elif self.kind == VELOCITY:
            block["forward_cmd"] = round(self.vx, 3)
            block["lateral_cmd"] = round(self.vy, 3)
            block["yaw_rate_cmd"] = round(math.degrees(self.yaw_rate), 1)
        return block


def goto(target_xy, speed, reason):
    """Drive to a world point. ArduPilot steers; we only say where."""
    return Intent(GOTO, reason, target=target_xy, speed=speed)


def move(forward=0.0, starboard=0.0, yaw_rate=0.0, reason=""):
    """Body-frame velocity. Negative `forward` is astern - that is how the
    docking task's "reverse out" is expressed (NJORD §9.3)."""
    return Intent(VELOCITY, reason, vx=forward, vy=starboard, yaw_rate=yaw_rate)


def stop(reason):
    """Zero velocity. Not the same as `idle` - this actively commands a stop."""
    return Intent(STOP, reason)


def idle(reason):
    """Command nothing at all. The boat is not ours to drive right now."""
    return Intent(IDLE, reason)


# ------------------------------------------------------------------ helpers

def yaw_rate_towards(desired_heading, current_heading):
    """Proportional yaw rate, rad/s, positive to starboard.

    A deadband, because a boat oscillating a degree either side of its target
    heading looks broken to a jury and wears the thrusters for nothing.
    """
    error = geo.angle_diff(desired_heading, current_heading)
    if abs(error) < YAW_DEADBAND_DEG:
        return 0.0
    return max(-YAW_MAX_RATE, min(YAW_MAX_RATE, YAW_P * error))


def station_keep(state, target_xy, desired_heading, config, reason, ceiling=None):
    """A body-velocity intent that holds a point and a heading.

    This is how the two scored "stay stationary" requirements are met - stop at
    GPS point 4 (§9.1) and hold in the berth (§9.3) - and it is a controller
    rather than simply commanding zero because there is tide in Trondheim. A
    boat commanded to zero speed in a current is a boat leaving the berth
    slowly.

    Inside `HOLD_TOLERANCE_M` it commands nothing, so a boat that is where it
    should be sits still instead of hunting. Outside it, the correction is
    proportional and expressed in the *body* frame, which is what lets the
    lateral thruster do the sideways part without the hull having to turn.

    `ceiling` is the operator's speed setting (`ctx.ceiling`), and passing it is
    how a hold obeys "0.1 m/s" instead of pulling back at the 0.30 m/s docking
    creep regardless. Both axes are held to it, because a hold that crabs at
    0.35 m/s while the operator asked for 0.1 is not slower in any sense they
    would recognise. Defaulted to None - the docking figures - so a caller that
    has no context behaves exactly as this did before.
    """
    if state.position is None or state.heading is None:
        return stop(f"{reason} (no position - holding by stopping)")

    east = target_xy[0] - state.position[0]
    north = target_xy[1] - state.position[1]
    error = math.hypot(east, north)
    heading = desired_heading if desired_heading is not None else state.heading
    yaw = yaw_rate_towards(heading, state.heading)

    if error <= config.HOLD_TOLERANCE_M:
        if yaw == 0.0:
            return stop(f"{reason} - holding, {error:.1f} m off")
        return move(yaw_rate=yaw, reason=f"{reason} - squaring up, {error:.1f} m off")

    starboard, forward = geo.world_to_boat(east, north, state.heading)
    gain = config.HOLD_P
    cap = config.DOCK_SPEED_MS
    sideways = LATERAL_MAX_MS
    if ceiling is not None:
        cap = min(cap, float(ceiling))
        sideways = min(sideways, float(ceiling))
    return move(
        forward=_clamp(forward * gain, -cap, cap),
        starboard=_clamp(starboard * gain, -sideways, sideways),
        yaw_rate=yaw,
        reason=f"{reason} - pulling back {error:.1f} m",
    )


def _clamp(value, low, high):
    return max(low, min(high, value))


# ------------------------------------------------------------- the speed limit

#: The hard ceiling every command out of this file is held to when no lower one
#: has been given: NJORD's 5 knots, from the repo-root `config.py`, which is not
#: overridable from the environment. The operator's own setting
#: (`Commander.speed`) is always at or below it and is what actually binds in a
#: run; this is the floor under the whole arrangement.
CEILING_MS = SPEED_LIMIT_MS


def _limit(value, ceiling=CEILING_MS):
    """One speed, held to the ceiling and to sanity. Always returns a number.

    NaN is mapped to zero rather than clamped. A NaN speed on the wire is not a
    fast boat, it is an undefined one - ArduPilot's behaviour on a NaN parameter
    is not something to find out during a scored run - and "stop" is the only
    safe reading of "no number".
    """
    try:
        out = float(value)
    except (TypeError, ValueError):
        return 0.0
    if out != out:  # NaN
        return 0.0
    return _clamp(out, -ceiling, ceiling)


def _limit_pair(forward, starboard, ceiling=CEILING_MS):
    """Forward and lateral, held to the ceiling **as a resultant**.

    Clamping each axis on its own is not enough and the arithmetic says why: at
    the 5 kn ceiling the forward term alone is 2.572 m/s, and adding the lateral
    thruster's 0.35 m/s gives `hypot(2.572, 0.35)` = 2.596 m/s = **5.05 knots**.
    Each axis is legal and the boat is over the limit, because speed through the
    water is the magnitude of the vector and not either component of it.

    So the pair is scaled down together when the resultant is too big, which
    preserves the *direction* the behaviour asked for - a docking approach that
    wanted to crab at 30 degrees still crabs at 30 degrees, just slower. Scaling
    only one axis would silently rotate the commanded motion, which is a far
    nastier failure than being a little slow.
    """
    forward = _limit(forward, ceiling)
    starboard = _limit(starboard, ceiling)
    resultant = math.hypot(forward, starboard)
    if resultant <= ceiling or resultant <= 0.0:
        return forward, starboard
    scale = ceiling / resultant
    return forward * scale, starboard * scale


# ----------------------------------------------------------------- commander

class Commander:
    """Sends intents down the node link, and owns mode and arming.

    Holds no MAVLink connection of its own - `io_manager` owns the serial port
    (see `link.py`). Everything here is a request; the ack that it was *sent*
    comes back on the next state frame's `mode` and `armed`, which is the only
    honest confirmation there is.
    """

    def __init__(self, link, config):
        self._link = link
        self._config = config
        self._last_sent = 0.0
        self._last_signature = None
        self._mode_requested_at = 0.0
        self._arm_requested_at = 0.0
        self.engaged = False
        self.sent = 0
        self.last_intent = None
        # **The one speed setting**, m/s: what the boat runs a leg at and the
        # ceiling nothing may exceed. It lives here rather than on the pilot
        # because this is the file that enforces it, and a number that lives
        # anywhere other than its enforcement point eventually disagrees with it.
        # `pilot.py` passes it into every `Context`, so a behaviour *plans* at
        # the speed it will actually get instead of asking for more and being
        # silently clamped on the way out.
        self.speed = min(SPEED_LIMIT_MS, float(SPEED_MS))
        # The cardinal alternation prior. See `behaviours/alternation.py`; off
        # unless deliberately switched on, because it is an inference rather than
        # a measurement.
        self.alternation = bool(ALTERNATION_DEFAULT)

    # ------------------------------------------------------------- the speed

    @property
    def ceiling(self):
        """The speed in force right now, m/s. Never above the vessel limit."""
        return min(SPEED_LIMIT_MS, self.speed)

    def set_speed(self, value):
        """Set the one speed, m/s. `(ok, message)` for the operator's ack.

        **Refused rather than clamped** out of range, the same rule
        `io_manager/guided.py` follows for the hand-flown cap: an operator who
        typed 4 m/s and quietly got 2.57 would believe the boat was doing 4.

        Takes effect on the next tick and does not interrupt a run - the point is
        to slow a pass down, not to stop it - and it applies to docking as much as
        to a transit, which is the whole reason it exists in this form.
        """
        try:
            number = float(value)
        except (TypeError, ValueError):
            return False, f"speed is a number of m/s, not {value!r}"
        if number != number or number in (float("inf"), float("-inf")):
            return False, "speed must be a finite number of m/s"
        if not SPEED_MIN_MS <= number <= SPEED_LIMIT_MS:
            return False, (
                f"speed must be {SPEED_MIN_MS:g}..{SPEED_LIMIT_MS:.2f} m/s - "
                f"{SPEED_LIMIT_KNOTS:g} knots is the vessel limit and nothing "
                "from the dashboard can raise it"
            )
        was = self.speed
        self.speed = number
        if abs(was - number) < 1e-9:
            return True, f"speed was already {number:.2f} m/s"
        # WARNING rather than INFO: "why was it going that fast" - or that slow -
        # has to be answerable from the journal afterwards rather than from
        # somebody's memory of what they typed.
        log.warning(
            "SPEED %.2f -> %.2f m/s (%.2f kn) - transits and every docking creep "
            "are held to it",
            was,
            number,
            number / KNOT_MS,
        )
        return True, (
            f"speed {number:.2f} m/s ({number / KNOT_MS:.2f} kn) - transits run "
            "at it and docking is held under it"
        )

    def set_alternation(self, on):
        """Switch the cardinal alternation prior. `(ok, message)`."""
        was = self.alternation
        self.alternation = bool(on)
        if was == self.alternation:
            return True, (
                f"the alternation prior was already "
                f"{'on' if self.alternation else 'off'}"
            )
        if self.alternation:
            log.warning(
                "ALTERNATION PRIOR ON - an uncommitted cardinal may now be "
                "passed on the side the previous mark implies"
            )
            return True, (
                "alternation prior ON - a cardinal the camera has not committed "
                "will be passed on the opposite side to the mark before it, and "
                "said so on the panel"
            )
        log.warning("alternation prior OFF")
        return True, (
            "alternation prior OFF - an uncommitted cardinal holds the planned "
            "line and slows down instead of guessing"
        )

    def set_mark_source(self, sources):
        """Which camera sources may create red/green marks. `(ok, message)`.

        The dashboard's two surprise-task modes, and it writes straight onto the
        config module - which every consumer holds a reference to, so the change is
        live on the next tick and lands in the trip header via `config.snapshot`.

        Switching sources deliberately does NOT clear the existing marks. A mark the
        colour test created is a real observation of a real buoy whichever detector
        found it, and throwing the map away mid-leg would leave the boat blind for
        the several ticks `TRACK_CONFIRM_HITS` needs to build it again - in the
        middle of the one leg it is scored on. `forget_world` is the button for
        starting clean, and it is a separate press for that reason.
        """
        allowed = ("colour", "yolo")
        if sources is None:
            requested = ()
        elif isinstance(sources, str):
            requested = tuple(
                part.strip().lower() for part in sources.split(",") if part.strip()
            )
        else:
            try:
                requested = tuple(str(part).strip().lower() for part in sources)
            except TypeError:
                return False, f"{sources!r} is not a source or a list of them"

        unknown = [part for part in requested if part not in allowed]
        if unknown:
            return False, (
                f"unknown mark source{'s' if len(unknown) > 1 else ''} "
                f"{', '.join(unknown)} - the sources are {', '.join(allowed)}"
            )

        requested = tuple(dict.fromkeys(requested))  # de-duplicate, keep the order
        self._config.MARK_SOURCES = requested
        # One switch, not two. `CAMERA_CREATES_MARKS` is the gate `world.py` reads
        # and this list is what it lets through, so an operator who empties the list
        # means "off" and should not have to find a second control to say it.
        self._config.CAMERA_CREATES_MARKS = bool(requested)

        if not requested:
            log.warning(
                "MARK SOURCES OFF - the camera can no longer create marks, so a "
                "'buoys' leg is blind GNSS transit"
            )
            return True, (
                "mark sources OFF - no camera-created marks, and with both lidars "
                "down a 'buoys' waypoint is now blind transit"
            )
        log.warning("MARK SOURCES: %s", ", ".join(requested))
        return True, (
            f"mark sources {', '.join(requested)} - these may now create red and "
            f"green marks, and a 'buoys' leg will shift its corridor for them "
            f"after {self._config.TRACK_CONFIRM_HITS} sightings"
        )

    # ------------------------------------------------------------- engagement

    def engage(self, state):
        """Ask for GUIDED and for the vehicle to be armed. Idempotent.

        Re-requested at most every two seconds while the vehicle has not
        complied, because `set_mode` gets no reliable ack across MAVLink
        dialects (`io_manager/main.py:apply_mode`) - the real confirmation is
        the next heartbeat's mode, and until it says GUIDED the request is worth
        repeating. Spamming it faster would just crowd the link.
        """
        self.engaged = True
        now = time.time()
        if state.mode != "GUIDED" and now - self._mode_requested_at > 2.0:
            self._mode_requested_at = now
            self._link.control(cmd="set_mode", mode="GUIDED")
            log.info("requesting GUIDED (autopilot is in %s)", state.mode or "unknown")
        if not state.armed and now - self._arm_requested_at > 2.0:
            self._arm_requested_at = now
            self._link.control(cmd="arm", arm=True)
            log.info("requesting arm")

    def disengage(self, reason, disarm=False):
        """Hand the boat back. Puts it in HOLD, which stops it where it is.

        HOLD rather than disarming, by default: an armed boat holding station is
        recoverable by the operator in one command, whereas a disarmed one needs
        arming first - and NJORD §8.2 gives the team twenty seconds to take over
        by remote, which is not a lot of time to be rearming in.
        """
        if not self.engaged:
            return
        self.engaged = False
        self._last_signature = None
        log.warning("autonomy disengaging: %s", reason)
        self._link.control(cmd="set_mode", mode="HOLD")
        if disarm:
            self._link.control(cmd="arm", arm=False)

    # ---------------------------------------------------------------- sending

    def send(self, intent, state, now=None):
        """Put one intent on the wire. Returns what was actually sent.

        Re-sends an unchanged intent every `TARGET_REFRESH_S` as a keepalive
        against ArduPilot's guided-command timeout, and sends immediately
        whenever the intent changes, so a stop is never a refresh period late.

        `now` is the tick's clock, and passing it rather than reading
        `time.time()` here is what makes the keepalive testable. `main.py` already
        ticks on `time.time()`, so in production the two are the same value; in a
        simulation or a replay they are emphatically not, and a `send` that reads
        the wall clock silently stops keeping alive - two thousand simulated ticks
        go by inside one real second, `due` is never true, and a boat whose aim
        point happens to sit still coasts in a straight line off the course while
        every assertion about the run passes. That is not a hypothetical: it is
        what this argument was added to fix.
        """
        self.last_intent = intent
        now = time.time() if now is None else float(now)
        signature = self._signature(intent)
        due = now - self._last_sent >= TARGET_REFRESH_S
        if signature == self._last_signature and not due:
            return None
        self._last_signature = signature
        self._last_sent = now

        if intent.kind == IDLE:
            return None
        if intent.kind == STOP:
            self._link.control(cmd="velocity_target", vx=0.0, vy=0.0, yaw_rate=0.0)
            self._lateral(0.0)
            self.sent += 1
            return intent
        if intent.kind == GOTO:
            self._send_goto(intent, state)
            self.sent += 1
            return intent
        if intent.kind == VELOCITY:
            self._send_velocity(intent)
            self.sent += 1
            return intent
        return None

    def _send_goto(self, intent, state):
        """A position target, in lat/lon, because MAVLink's is global.

        Falls back to a stop rather than to a guessed position if the grid has
        no origin: a target computed against a missing origin would be a point
        near Null Island, and the boat would set off for the Gulf of Guinea.
        """
        position = geo.to_global(intent.target[0], intent.target[1], state.origin)
        if position is None:
            log.error("cannot send a position target without a grid origin")
            self._link.control(cmd="velocity_target", vx=0.0, vy=0.0, yaw_rate=0.0)
            return
        # `_limit` before the 0.05 floor, so the floor can only ever raise a
        # speed off zero and never lift one back over the ceiling.
        speed = max(0.05, _limit(intent.speed or 0.0, self.ceiling))
        self._link.control(
            cmd="position_target",
            lat=position[0],
            lon=position[1],
            speed=speed,
            reason=intent.reason[:120],
        )

    def _send_velocity(self, intent):
        starboard = _clamp(intent.vy, -LATERAL_MAX_MS, LATERAL_MAX_MS)
        # Held to the ceiling as a resultant, not per axis - see `_limit_pair`.
        forward, starboard = _limit_pair(intent.vx, starboard, self.ceiling)
        yaw = _clamp(intent.yaw_rate, -YAW_MAX_RATE, YAW_MAX_RATE)
        # vy goes out on the MAVLink command only when ArduPilot is the thing
        # driving the lateral thruster. In "rc" mode it would be a term the
        # autopilot cannot honour, and in "none" mode there is no such thruster.
        self._link.control(
            cmd="velocity_target",
            vx=forward,
            vy=starboard if LATERAL_MODE == "mavlink" else 0.0,
            yaw_rate=yaw,
            reason=intent.reason[:120],
        )
        self._lateral(starboard)

    def _lateral(self, starboard_ms):
        """Drive the sideways thruster, if it is on an RC channel."""
        if LATERAL_MODE != "rc":
            return
        if not LATERAL_RC_CHAN:
            # Deliberately not a guess. `trim.py` takes the same line: publish
            # nothing until told which channel, because the wrong channel drives
            # something else on the boat.
            return
        fraction = _clamp(starboard_ms / max(1e-6, LATERAL_MAX_MS), -1.0, 1.0)
        self._link.control(
            cmd="rc_override",
            channel=LATERAL_RC_CHAN,
            pwm=int(round(LATERAL_RC_CENTRE + fraction * LATERAL_RC_SPAN)),
        )

    def _signature(self, intent):
        """What counts as "the same command", for the re-send test."""
        if intent.kind == GOTO:
            return (
                GOTO,
                round(intent.target[0], 1),
                round(intent.target[1], 1),
                round(intent.speed or 0.0, 2),
            )
        if intent.kind == VELOCITY:
            return (
                VELOCITY,
                round(intent.vx, 2),
                round(intent.vy, 2),
                round(intent.yaw_rate, 2),
            )
        return (intent.kind,)

    # ------------------------------------------------------------- telemetry

    def telemetry(self):
        block = {"engaged": self.engaged, "commands_sent": self.sent}
        if self.last_intent is not None:
            block.update(self.last_intent.telemetry())
        # The limit in force, in the units the rule is written in. On the panel
        # because a jury asking "what stops it going faster than 5 knots" should
        # get an answer off the screen, and in the trip recording because it is
        # the number every commanded speed in that file has to be read against.
        block["speed_limit_kn"] = SPEED_LIMIT_KNOTS
        block["speed_ceiling_ms"] = round(self.ceiling, 3)
        block["speed_ceiling_kn"] = round(self.ceiling / KNOT_MS, 2)
        # The operator's setting under its own name as well as under
        # `speed_ceiling_*`. They are the same number today and the panel reads
        # this one, because "the speed I set" and "the ceiling being enforced"
        # are two different questions and only one of them has an answer the
        # operator typed.
        block["speed_ms"] = round(self.speed, 3)
        block["speed_kn"] = round(self.speed / KNOT_MS, 2)
        block["speed_min_ms"] = round(SPEED_MIN_MS, 3)
        block["alternation"] = self.alternation
        # Which camera sources may create marks, always sent as a list even when it
        # is empty - `[]` on the panel is "off, and the vessel said so", where a
        # missing key is "this build does not have the feature". /surprise_task
        # shows those two differently on purpose, because with both lidars down the
        # first is a decision and the second is a boat that needs a git pull.
        block["mark_sources"] = list(self._config.MARK_SOURCES)
        block["lateral_mode"] = LATERAL_MODE
        if LATERAL_MODE == "rc" and not LATERAL_RC_CHAN:
            block["lateral_warning"] = (
                "LIGMAX_LATERAL_RC_CHAN is not set, so the sideways thruster is "
                "not being driven"
            )
        return block
