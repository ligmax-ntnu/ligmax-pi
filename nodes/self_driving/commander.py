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
              `io_manager/pixhalwk.py` drives the ride height on channel 16.
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
    LATERAL_MAX_MS,
    LATERAL_MODE,
    LATERAL_RC_CENTRE,
    LATERAL_RC_CHAN,
    LATERAL_RC_SPAN,
    CAREFUL_DEFAULT,
    CAREFUL_SPEED_KNOTS,
    CAREFUL_SPEED_MS,
    KNOT_MS,
    MAX_SPEED_MS,
    SPEED_LIMIT_KNOTS,
    SPEED_LIMIT_MS,
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


def station_keep(state, target_xy, desired_heading, config, reason):
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
    return move(
        forward=_clamp(forward * gain, -config.DOCK_SPEED_MS, config.DOCK_SPEED_MS),
        starboard=_clamp(starboard * gain, -LATERAL_MAX_MS, LATERAL_MAX_MS),
        yaw_rate=yaw,
        reason=f"{reason} - pulling back {error:.1f} m",
    )


def _clamp(value, low, high):
    return max(low, min(high, value))


# ------------------------------------------------------------- the speed limit

#: The ordinary ceiling every command out of this file is held to: the boat's own
#: 5 kn limit, and whatever lower figure the tuning asks for. `min` of the two
#: rather than trusting `MAX_SPEED_MS` alone, because that one is read from the
#: environment and this one is not (`config.SPEED_LIMIT_MS`).
CEILING_MS = min(SPEED_LIMIT_MS, MAX_SPEED_MS)

#: The ceiling in careful mode. Never above the ordinary one - careful mode can
#: only ever slow the boat down, so switching it on can never be the thing that
#: makes the boat go faster, whatever the tuning says.
CAREFUL_CEILING_MS = min(CEILING_MS, CAREFUL_SPEED_MS)


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
        # Careful mode lives here rather than on the pilot because this is the
        # file that enforces it, and a flag that lives anywhere other than its
        # enforcement point eventually disagrees with it. `pilot.py` reads it
        # through the commander so the behaviours plan at the slower speed
        # instead of being clamped after the fact.
        self.careful = bool(CAREFUL_DEFAULT)

    # -------------------------------------------------------- careful mode

    @property
    def ceiling(self):
        """The speed ceiling in force right now, m/s."""
        return CAREFUL_CEILING_MS if self.careful else CEILING_MS

    def set_careful(self, on):
        """Switch careful mode. `(ok, message)` for the operator's ack."""
        was = self.careful
        self.careful = bool(on)
        if was == self.careful:
            state = "on" if self.careful else "off"
            return True, f"careful mode was already {state}"
        if self.careful:
            log.warning(
                "CAREFUL MODE ON - speed held to %.1f kn (%.2f m/s)",
                CAREFUL_SPEED_KNOTS,
                self.ceiling,
            )
            return True, (
                f"careful mode ON - {CAREFUL_SPEED_KNOTS:.1f} kn "
                f"({self.ceiling:.2f} m/s) maximum"
            )
        log.warning(
            "careful mode OFF - speed back to the ordinary %.2f m/s ceiling",
            self.ceiling,
        )
        return True, (
            f"careful mode OFF - back to {self.ceiling / KNOT_MS:.1f} kn "
            f"({self.ceiling:.2f} m/s) maximum"
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

    def send(self, intent, state):
        """Put one intent on the wire. Returns what was actually sent.

        Re-sends an unchanged intent every `TARGET_REFRESH_S` as a keepalive
        against ArduPilot's guided-command timeout, and sends immediately
        whenever the intent changes, so a stop is never a refresh period late.
        """
        self.last_intent = intent
        now = time.time()
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
        block["careful"] = self.careful
        block["lateral_mode"] = LATERAL_MODE
        if LATERAL_MODE == "rc" and not LATERAL_RC_CHAN:
            block["lateral_warning"] = (
                "LIGMAX_LATERAL_RC_CHAN is not set, so the sideways thruster is "
                "not being driven"
            )
        return block
