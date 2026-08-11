"""What every behaviour is, plus the obstacle avoidance they all share.

A behaviour is a small object with one method:

    intent = behaviour.update(ctx)

`ctx` is everything it may read - the boat's state, the world model, the plan -
and the intent is everything it may do (`commander.py`). It owns no clock, no
socket and no autopilot connection, which is what makes every one of them
runnable against a recorded trip on a laptop.

`behaviour.done` going True means "this waypoint is finished, advance the plan".
Nothing else advances it: a behaviour that cannot finish must say so through
`status`, not by quietly giving up, because NJORD §8.2 gives the crew twenty
seconds to take over and they can only use them if the boat has said something.

The one thing every behaviour shares
------------------------------------
Not hitting things. `deconflict()` below is the common avoidance, and it is
deliberately the *same* code for a blind GPS leg and for a buoy leg - the
difference between those tasks is which side a mark must be passed, not whether
to hit it.

The method is a steering nudge, not a planner. Given where the behaviour wants
to aim, it pushes that aim point sideways until the straight line to it clears
every confirmed obstacle by that obstacle's own clearance. Two or three
iterations converge, and the result is a heading the boat can hold rather than a
path it has to track.

Why not A*, RRT or a potential field:

  * an A* over a grid needs a grid, and the Njord course is a dozen discrete
    marks in open water - the grid would be almost entirely empty sea;
  * a sampled planner produces a different path every tick, and a boat that
    re-plans from scratch at 10 Hz weaves visibly, which reads to a jury as
    indecision;
  * a potential field has local minima, and the local minimum between two buoys
    5 m apart is exactly the gate the boat is supposed to drive through.

The nudge has none of those failure modes and it fits in a page. What it cannot
do is find its way out of a concave trap - so `pilot.py` watches for no progress
and says so rather than pretending.
"""

from __future__ import annotations

import math

from .. import geo
from ..commander import goto, move, stop
from ..obsticales import ObstacleType, clearance_for

# How many times to push the aim point before accepting it. Each pass moves it
# clear of the worst offender; three is enough for the counts involved here and
# bounds the work per tick.
DECONFLICT_PASSES = 3

# An obstacle further off the bow than this is not in the way, whatever the
# geometry says. Without it the boat swerves for a buoy it is already abeam of.
AHEAD_CONE_DEG = 100.0

# Below this a corner is a kink rather than a turn and the limiter stays out of
# it. Also the guard on the division in `corner_speed_limit`, where the radius
# goes to infinity as the turn goes to zero.
MIN_TURN_DEG = 12.0


def speed_for_radius(ctx, radius_m):
    """The fastest the boat can hold a turn of `radius_m`, m/s.

    Two laws, and the tighter one wins, because they bind at opposite ends of the
    speed range:

        v = sqrt(A * R)     grip. A turn needs `v^2 / R` of lateral acceleration
                            and the hull only supplies so much. This is what binds
                            at the top of the speed range, and it is the law the
                            first version of this file got wrong.
        v = omega * R       yaw authority. At a walking pace the acceleration law
                            would allow a turn on the spot; what actually stops
                            that is how much yaw moment the thrusters make.

    They cross at `R = A / omega^2`. Above that radius grip is the limit, below it
    the thrusters are, and taking the minimum is correct on both sides of the
    crossing without a special case.
    """
    radius_m = max(0.0, float(radius_m))
    grip = math.sqrt(ctx.config.TURN_LATERAL_ACCEL_MS2 * radius_m)
    authority = ctx.config.TURN_YAW_RATE * radius_m
    return max(ctx.config.CORNER_MIN_SPEED_MS, min(grip, authority))


class Context:
    """Everything a behaviour may read. Built fresh each tick by `pilot.py`."""

    __slots__ = (
        "state", "world", "plan", "config", "now", "waypoint", "leg", "task",
        "clusters", "sweeps", "ceiling", "alternation",
    )

    def __init__(self, state, world, plan, config, now, waypoint, leg, task,
                 clusters=(), ceiling=None, alternation=False, sweeps=()):
        self.state = state
        self.world = world
        self.plan = plan
        self.config = config
        self.now = now
        self.waypoint = waypoint
        self.leg = leg  # (start_xy, end_xy) in world metres, or None
        self.task = task  # "transit" / "buoys" / "dock" - drives classification
        # This tick's raw clusters, BOAT frame, before they became tracks.
        # Only docking uses them, and it needs them precisely *because* they are
        # raw: a berth is a gap between two structures measured right now, and
        # routing that measurement through the tracker would smooth away the
        # centimetres the 2 m berth is decided by.
        self.clusters = clusters
        # This tick's raw sweeps, one dict per lidar, each with a `source` and
        # boat-frame `points` - exactly what `nodes/io_manager/scan.py` builds and
        # what `main.py` already holds. Only the parking behaviours use them, and
        # they need the *points* rather than the clusters because a parking space
        # is three straight edges: a wall reads as one enormous cluster whose
        # centroid sits in the middle of the wall, which says nothing about where
        # its line runs (`perception/lines.py`).
        #
        # Kept as the scan dicts rather than one merged point array on purpose:
        # the line fitter relies on each array being in one sensor's angular
        # order, and two sweeps concatenated interleave at every shared bearing.
        self.sweeps = sweeps
        # **The operator's one speed setting, m/s** - `commander.Commander.speed`,
        # which is what the boat runs a leg at and the ceiling nothing may exceed.
        # Passed in rather than read from config so that a new setting takes
        # effect on the next tick, and so that a behaviour *plans* at the speed it
        # will actually get instead of asking for more and being silently clamped
        # on the way out.
        self.ceiling = (
            config.SPEED_MS if ceiling is None else float(ceiling)
        )
        # Whether the cardinal alternation prior is switched on
        # (`behaviours/alternation.py`). Defaulted, so a `Context` built by hand
        # in a test or a replay tool behaves like an ordinary run.
        self.alternation = bool(alternation)

    # -- the handful of derived figures every behaviour wants ---------------

    @property
    def cruise_speed(self):
        """What to ask for on an open leg: the setting, or this waypoint's own.

        The setting *is* the cruise. There is no separate cruise figure any more,
        because two numbers meaning "how fast does it run a leg" is how the
        dashboard and the boat end up disagreeing about which one is in force.
        """
        return self.speed_limit(self.ceiling)

    @property
    def caution_speed(self):
        """What to ask for among marks. **The same number as `cruise_speed`.**

        Deliberately identical, and kept as its own name rather than collapsed
        into one: the two used to differ because a profile carried a cruise figure
        and a caution figure, and with one operator-set speed there is nothing
        left for them to differ about. An operator who wants the boat slower among
        marks sets it slower - that is what the setting is for, and it is honest in
        a way "I asked for 5 knots and got 1.6 near the buoys" never was.

        The name stays because a behaviour saying `ctx.caution_speed` is saying
        something about *why* it wants that speed, and because the situational
        slow-downs that remain (`CAUTION_SPEED_MS` for an uncommitted cardinal in
        `buoys.py`, the corner limiter, the docking creeps) are the mechanisms that
        actually keep the boat slow where it matters.
        """
        return self.cruise_speed

    @property
    def boat(self):
        return self.state.position

    @property
    def heading(self):
        return self.state.heading

    @property
    def target(self):
        """The current waypoint in world metres, or None."""
        if self.waypoint is None:
            return None
        return self.waypoint.world(self.state.origin)

    @property
    def distance_to_target(self):
        target = self.target
        if target is None or self.boat is None:
            return None
        return geo.distance(self.boat, target)

    def speed_limit(self, default):
        """The waypoint's own speed if it set one, else `default`, else the cap.

        `self.ceiling` is the operator's setting, so it is obeyed here - where the
        behaviour is deciding - rather than only at the wire. The difference is
        visible: a behaviour that knows it is limited to 0.1 m/s reports that
        speed in its telemetry and reasons about arrival times with it, instead of
        reporting 1.2 m/s while the boat does 0.1.
        """
        if self.waypoint is not None and self.waypoint.speed is not None:
            return min(self.waypoint.speed, self.ceiling)
        return min(default, self.ceiling)

    def acceptance_radius(self):
        if self.waypoint is not None and self.waypoint.radius is not None:
            return self.waypoint.radius
        if self.waypoint is not None and self.waypoint.settles:
            return self.config.ARRIVAL_RADIUS_M
        return self.config.WAYPOINT_RADIUS_M


class Behaviour:
    """Base class. Subclasses override `update` and usually `start`."""

    name = "behaviour"

    def __init__(self, config):
        self.config = config
        self.done = False
        self.started_at = None
        self.status = {}
        self._reason = ""

    def start(self, ctx):
        """Called once when this behaviour takes over a waypoint."""
        self.done = False
        self.started_at = ctx.now
        self.status = {}

    def update(self, ctx):  # pragma: no cover - abstract
        raise NotImplementedError

    # -- shared plumbing -----------------------------------------------------

    def elapsed(self, ctx):
        return 0.0 if self.started_at is None else ctx.now - self.started_at

    def note(self, **fields):
        """Record why this tick did what it did, for the operator's panel."""
        self.status.update(fields)

    def telemetry(self):
        return {"behaviour": self.name, **self.status}


# --------------------------------------------------------------- arrival

def has_arrived(ctx):
    """Whether the current waypoint counts as reached. `(bool, why)`.

    Two tests, and the second one is the important one.

    **Radius** - within the acceptance radius. Obvious, and on its own it is a
    trap: a waypoint laid slightly off the line the boat can actually hold, in a
    cross-current, is one the boat orbits forever at radius + epsilon. Every
    naive waypoint follower has this bug and it always shows up on the water
    rather than on the bench.

    **Passing plane** - the boat is past the plane through the waypoint
    perpendicular to the leg, i.e. it is no longer possible to make progress
    towards it by continuing. That releases the orbit.

    The plane test is deliberately NOT applied to waypoints whose role is to
    settle - "stop at GPS point 4" (NJORD §9.1) is not satisfied by driving past
    it, and neither is a dock.
    """
    boat = ctx.boat
    target = ctx.target
    if boat is None or target is None:
        return False, "no position"

    distance = geo.distance(boat, target)
    radius = ctx.acceptance_radius()
    if distance <= radius:
        return True, f"within {radius:.1f} m ({distance:.1f} m)"

    if (
        ctx.config.USE_PASSING_PLANE
        and ctx.leg is not None
        and ctx.waypoint is not None
        and not ctx.waypoint.settles
    ):
        t, _along, cross = geo.project_onto_leg(boat, ctx.leg[0], ctx.leg[1])
        # Past the end of the leg, and not so far off to the side that "past" is
        # meaningless - three radii out, the boat has missed the waypoint rather
        # than passed it, and should be told so instead of counting it.
        if t >= 1.0 and abs(cross) <= radius * 3.0:
            return True, f"passed the mark ({distance:.1f} m abeam)"
    return False, f"{distance:.1f} m to run"


# ------------------------------------------------------------- avoidance

def obstacles_ahead(ctx, kinds=None, within=None):
    """Confirmed tracks in front of the boat, nearest first.

    "In front" is a cone, not a half plane: something 95 deg off the bow is
    abeam and is being passed, and treating it as an obstacle makes the boat
    swerve away from marks it has already dealt with.
    """
    boat, heading = ctx.boat, ctx.heading
    if boat is None or heading is None:
        return []
    limit = within if within is not None else ctx.config.MAX_OBSTACLE_RANGE_M
    found = []
    for track in ctx.world.confirmed():
        if kinds is not None and track.kind not in kinds:
            continue
        distance = geo.distance(boat, track.pos)
        if distance > limit:
            continue
        if abs(geo.relative_bearing(track.pos, boat, heading)) > AHEAD_CONE_DEG:
            continue
        found.append((distance, track))
    found.sort(key=lambda item: item[0])
    return [track for _distance, track in found]


def dynamic_clearance(ctx):
    """Extra metres of room for the speed the boat is being driven at.

    A clearance is a time budget wearing metres. `BUOY_CLEARANCE_M` is 2 m, which
    at the 0.8 m/s caution speed is two and a half seconds to notice a mark is
    not where it was believed to be and steer off it - and at 2.5 m/s is eight
    tenths of a second, which is less than one tick plus the thrusters. A static
    clearance therefore means the boat gets progressively less safe the faster it
    goes while the disc on the operator's chart says it has not changed.

    So the metres-per-m/s is a config figure, `CLEARANCE_PER_MS`, and it is
    **zero by default**: Task 2's gates are 5 m across and any speed term at all
    would have the boat refuse a gate it is meant to drive through. See
    `config.CLEARANCE_PER_MS`, which is where the trade is written down.

    Measured against the *larger* of what the boat is doing and what it has been
    told it may do, not against the speedometer alone. A boat accelerating down a
    leg would otherwise widen its berth as it went, which puts the correction late
    - the whole value of a wide berth is that it is decided at the top of the leg
    and steered once, rather than discovered next to the mark.
    """
    gain = getattr(ctx.config, "CLEARANCE_PER_MS", 0.0)
    if gain <= 0.0:
        return 0.0
    speed = 0.0
    if ctx.state is not None:
        try:
            speed = abs(float(ctx.state.speed or 0.0))
        except (TypeError, ValueError):
            speed = 0.0
    speed = max(speed, ctx.caution_speed)
    return min(ctx.config.CLEARANCE_SPEED_MAX_M, gain * speed)


def lookahead_for(ctx, remaining):
    """How far along the leg to aim, metres. Pure pursuit's one parameter.

    A fixed distance is the wrong shape. What governs whether pure pursuit is
    stable is how much *time* the lookahead represents: 6 m is five seconds at
    the 1.2 m/s cruise and two and a third at 2.5 m/s, and a lookahead that short
    relative to the speed makes the boat correct harder than it can turn,
    overshoot, and weave down the leg with the jury watching the trace.

    So it is the larger of the fixed distance and `LOOKAHEAD_TIME_S` of travel -
    which changes nothing below about 1.5 m/s and gives a fast setting the longer
    rein it needs.

    Measured against the speed the boat has been *told* to run rather than the
    speedometer,
    deliberately. The lookahead moves the aim point, so a lookahead that tracked
    the measured speed would jitter the aim with every ripple in the log, and
    would also lengthen exactly when the corner limiter had just shortened the
    speed for a turn - which is when a long lookahead cuts the corner. The
    `remaining * 0.8` cap is what actually pulls the aim in on the run-up to a
    mark, and it does it on geometry rather than on a noisy input.
    """
    config = ctx.config
    distance = max(config.LOOKAHEAD_M, ctx.cruise_speed * config.LOOKAHEAD_TIME_S)
    return max(config.LOOKAHEAD_MIN_M, min(distance, remaining * 0.8))


# ------------------------------------------------------- how fast a corner allows

def next_leg(ctx):
    """`(bearing, length_m)` of the leg after this one, or None.

    None at the last waypoint, without a plan, or when the next waypoint is on
    top of this one - all three mean "there is no corner here", which is the
    answer the limiter wants rather than an error.
    """
    plan, waypoint = ctx.plan, ctx.waypoint
    if plan is None or waypoint is None or ctx.state is None:
        return None
    following = waypoint.index + 1
    if following >= len(plan.waypoints):
        return None
    here = ctx.target
    there = plan.waypoints[following].world(ctx.state.origin)
    if here is None or there is None:
        return None
    length = geo.distance(here, there)
    if length < 0.5:
        return None
    return geo.bearing_to(here, there), length


def corner_speed_limit(ctx, speed):
    """Hold `speed` down to what the turn at the end of this leg allows.

    `(speed, note)`. The note is a sentence for the operator, empty when nothing
    was limited.

    The Monday course is a slalom - three corners over 100 degrees on legs of
    10-17 m (`plans/README.md`) - and a turn radius is `speed / yaw rate`. At
    2.5 m/s and a yaw rate of 0.5 rad/s that is a 5 m radius, which on a 123
    degree corner cuts inside the mark by more than the 3 m acceptance radius:
    the boat misses the waypoint it is being scored on passing, at speed, and
    looks decisive doing it. The fix is not a slower plan, it is a speed that
    follows the geometry - full pace on the straights, whatever the corner allows
    at the corner.

    Two constraints, and the tighter one wins.

    **The turn must fit inside the acceptance radius.** A circular arc of radius
    `R` tangent to both legs passes `R * (sec(turn/2) - 1)` from the corner, so
    the largest radius that still counts as passing the waypoint is
    `radius / (sec(turn/2) - 1)`.

    **The turn-in must fit on the leg.** That arc starts `R * tan(turn/2)` before
    the corner. On this course that is the binding one: a 104 degree turn at the
    end of a 5 m leg has room for a 2 m radius, not the 4.8 m the acceptance
    radius alone would allow. Half the shorter of the two legs is the budget,
    which leaves the other half for the previous corner's exit.

    Then the boat is allowed as much speed as it can still shed in the distance
    left - `v^2 = v_corner^2 + 2*a*d` - so a long leg into a tight corner runs at
    full pace and brakes late rather than crawling the whole way down it.

    **It looks one corner ahead, not two.** Where two tight corners share a short
    leg the exit from the first eats into the entry to the second, and this will
    be a little optimistic about the pair. The 5 m leg between waypoints 1 and
    1.1 is the one place on Monday's course that happens; `CORNER_MIN_SPEED_MS`
    and the operator's eye are what cover it.
    """
    following = next_leg(ctx)
    if following is None or ctx.leg is None or ctx.boat is None:
        return speed, ""
    bearing, length = following
    turn = abs(geo.angle_diff(bearing, geo.bearing_to(ctx.leg[0], ctx.leg[1])))
    if turn < MIN_TURN_DEG:
        return speed, ""

    # Clamped short of 90 degrees so `cos` cannot reach zero: a 180 degree turn
    # is a real entry in a plan (a course that doubles back) and it must limit
    # hard rather than divide by zero.
    half = min(math.radians(turn) * 0.5, math.radians(89.0))
    radius = ctx.acceptance_radius() / (1.0 / math.cos(half) - 1.0)

    leg_length = geo.distance(ctx.leg[0], ctx.leg[1])
    budget = min(leg_length, length) * 0.5
    radius = min(radius, budget / math.tan(half))

    corner = speed_for_radius(ctx, radius)
    remaining = max(0.0, ctx.distance_to_target or 0.0)
    allowed = math.sqrt(
        corner * corner + 2.0 * ctx.config.TURN_DECEL_MS2 * remaining
    )
    if allowed >= speed:
        return speed, ""
    allowed = max(ctx.config.CORNER_MIN_SPEED_MS, allowed)
    return allowed, (
        f"{turn:.0f} deg turn at {ctx.waypoint.name} in {remaining:.0f} m - "
        f"easing to {allowed:.1f} m/s"
    )


def heading_speed_limit(ctx, speed, aim_xy):
    """Hold `speed` down while the boat is pointing the wrong way. `(speed, note)`.

    The reactive half of the pair, and it covers what `corner_speed_limit` cannot.
    That one is anticipatory - it reads the plan and slows *before* a corner so
    the boat can get round it. This one reads the boat and slows while the boat is
    *already* crosswise, which happens for reasons no amount of looking ahead
    predicts:

      * coming out of a corner. A 115 degree turn is not instant, so for a second
        or two after the waypoint the boat is on the new leg pointing across it,
        and full cruise there is what swings it wide of the *next* mark - which is
        exactly how Monday's course loses waypoint 3.4 having made 3.3 perfectly.
      * a deconflict swerve, which moves the aim point without warning.
      * the moment autonomy is engaged, when the boat is pointing wherever the
        remote pilot left it.

    The bound is the same piece of physics as the corner limiter's, run the other
    way round: turning through `error` on a radius `R` takes `R * error` of arc,
    that arc has to fit in the distance to the aim point, so the largest usable
    radius is `reach / error` - and `speed_for_radius` says how fast the boat may
    go on it. A boat 6 m from an aim point 115 degrees off may do about 1.5 m/s;
    one 20 degrees off is not limited at all.

    Applied in `steer_towards`, so every behaviour that steers gets it without
    having to remember to.
    """
    boat, heading = ctx.boat, ctx.heading
    if boat is None or heading is None or aim_xy is None:
        return speed, ""
    error = abs(geo.angle_diff(geo.bearing_to(boat, aim_xy), heading))
    if error < MIN_TURN_DEG:
        return speed, ""
    # Floored at the minimum lookahead: an aim point half a metre away with a
    # large heading error would otherwise demand a crawl, and at that range the
    # boat is turning on the spot anyway.
    reach = max(ctx.config.LOOKAHEAD_MIN_M, geo.distance(boat, aim_xy))
    allowed = speed_for_radius(ctx, reach / math.radians(error))
    if allowed >= speed:
        return speed, ""
    allowed = max(ctx.config.CORNER_MIN_SPEED_MS, allowed)
    return allowed, (
        f"{error:.0f} deg off the aim - holding {allowed:.1f} m/s while it "
        f"turns onto it"
    )


def deconflict(ctx, aim_xy, extra_clearance=0.0, ignore=()):
    """Push `aim_xy` sideways until the line to it is clear. `(aim, notes)`.

    The obstacle is projected onto the boat-to-aim segment. If it sits inside
    the corridor, the aim point is moved *perpendicular to the segment*, away
    from the obstacle, by exactly the shortfall - so the correction is as small
    as it can be while still being enough, and a boat with nothing in the way is
    not steered at all.

    A moving obstacle is projected forward by the time it will take to reach it,
    so the boat avoids where the Otter is *going to be* rather than where it
    was. That is the difference between passing astern of a crossing vessel and
    arriving at the same place it does.
    """
    boat, heading = ctx.boat, ctx.heading
    if boat is None or heading is None or aim_xy is None:
        return aim_xy, []

    notes = []
    aim = aim_xy
    # Once per call rather than per track per pass: it is the same figure for
    # every obstacle, and computing it inside the loop would make the corridor
    # depend on how many things are in it.
    for_speed = dynamic_clearance(ctx)
    for _pass in range(DECONFLICT_PASSES):
        worst = None
        for track in ctx.world.confirmed():
            if track.id in ignore:
                continue
            # Plus however unsure we are of where it actually is. A mark the boat
            # is looking at right now costs ~35 cm of extra room; one it is
            # remembering from two minutes ago costs the full sigma ceiling. That
            # is the whole behavioural difference between seeing and remembering,
            # and it falls out of one addition rather than a special case.
            clearance = (
                clearance_for(track.kind, ctx.config)
                + extra_clearance
                + track.sigma_m
                + for_speed
            )
            position = _where(track, boat, ctx)
            t, along, cross = geo.project_onto_leg(position, boat, aim)
            if t <= 0.0 or along > geo.distance(boat, aim) + clearance:
                continue  # behind us, or beyond the aim point
            shortfall = clearance - abs(cross)
            if shortfall <= 0.0:
                continue
            if worst is None or shortfall > worst[0]:
                worst = (shortfall, cross, track)

        if worst is None:
            break

        shortfall, cross, track = worst
        # Push away from the obstacle. `cross` positive means it is to starboard
        # of the line, so the aim goes to port - and a dead-ahead obstacle
        # (cross ~ 0) is passed to starboard, which is both the COLREG default
        # and the side a jury expects.
        direction = -1.0 if cross >= 0.0 else 1.0
        segment = geo.bearing_to(boat, aim)
        aim = geo.offset_point(aim, segment + 90.0 * direction, shortfall + 0.3)
        notes.append(
            f"{track.kind.name.lower()} #{track.id} at "
            f"{geo.distance(boat, track.pos):.1f} m - going "
            f"{'port' if direction < 0 else 'starboard'} of it"
        )
    return aim, notes


def _where(track, boat, ctx):
    """Where to treat a track as being: now, or where it will be when we meet it."""
    if track.speed <= 0.15:
        return track.pos
    distance = geo.distance(boat, track.pos)
    own_speed = max(0.3, ctx.state.speed)
    return track.predicted(min(ctx.config.COLREG_HORIZON_S, distance / own_speed))


# An emergency stop is only ever triggered by something the boat can actually
# see. A track older than this is a memory, and memories do not get to slam the
# brakes on - see `emergency_stop_needed`.
EMERGENCY_FRESH_S = 1.0


def emergency_stop_needed(ctx):
    """Something too close to steer around. `(bool, why)`.

    The last line of defence, checked by every behaviour before it commits to
    anything. Deliberately based on the *nearest return*, not on a track: a
    track has to be confirmed over several sweeps before it exists, and
    something that appeared 1.5 m off the bow this instant does not get to wait
    three sweeps for confirmation.

    Deliberately based on a **recent** return, too, and that qualifier arrived
    with remembered marks. `world.all()` now includes established tracks that
    have not been measured for minutes and whose position is uncertain by metres.
    One of those, remembered a metre off the bow, would hold the boat stationary
    against an obstacle that is not there and that no sweep can clear - the boat
    would sit in open water insisting something was in front of it. So the test
    is restricted to tracks measured within the last second: everything that was
    ever going to fire this check still does, because anything genuinely 1.5 m
    off the bow is being hit by the lidar continuously.
    """
    boat, heading = ctx.boat, ctx.heading
    if boat is None or heading is None:
        return False, ""
    limit = ctx.config.MIN_OBSTACLE_RANGE_M + 1.0
    for track in ctx.world.all():
        if track.age(ctx.now) > EMERGENCY_FRESH_S:
            continue
        distance = geo.distance(boat, track.pos)
        if distance > limit:
            continue
        if abs(geo.relative_bearing(track.pos, boat, heading)) > 45.0:
            continue
        return True, (
            f"{track.kind.name.lower()} {distance:.1f} m dead ahead - stopping"
        )
    return False, ""


def steer_towards(ctx, aim_xy, speed, reason):
    """The ordinary way a behaviour drives: a deconflicted position target.

    The heading limit goes last, after `deconflict` has had its say, because a
    swerve round an obstacle is one of the things that leaves the boat crosswise
    to where it is going - limiting against the aim point the behaviour asked for
    rather than the one it is actually being sent to would miss exactly the case
    that matters.
    """
    ok, why = emergency_stop_needed(ctx)
    if ok:
        return stop(why)
    aim, notes = deconflict(ctx, aim_xy)
    if notes:
        reason = f"{reason}; avoiding {notes[0]}"
    speed, turning = heading_speed_limit(ctx, speed, aim)
    if turning:
        reason = f"{reason}; {turning}"
    return goto(aim, speed, reason)


def creep(ctx, forward, desired_heading, reason, starboard=0.0):
    """Slow body-frame movement with a heading hold. Docking's workhorse."""
    from ..commander import yaw_rate_towards

    yaw = 0.0
    if desired_heading is not None and ctx.heading is not None:
        yaw = yaw_rate_towards(desired_heading, ctx.heading)
    return move(forward=forward, starboard=starboard, yaw_rate=yaw, reason=reason)
