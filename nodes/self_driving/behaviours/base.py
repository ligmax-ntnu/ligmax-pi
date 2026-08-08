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


class Context:
    """Everything a behaviour may read. Built fresh each tick by `pilot.py`."""

    __slots__ = (
        "state", "world", "plan", "config", "now", "waypoint", "leg", "task",
        "clusters",
    )

    def __init__(self, state, world, plan, config, now, waypoint, leg, task,
                 clusters=()):
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

    # -- the handful of derived figures every behaviour wants ---------------

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
        """The waypoint's own speed if it set one, else `default`, else the cap."""
        if self.waypoint is not None and self.waypoint.speed is not None:
            return min(self.waypoint.speed, self.config.MAX_SPEED_MS)
        return min(default, self.config.MAX_SPEED_MS)

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
    for _pass in range(DECONFLICT_PASSES):
        worst = None
        for track in ctx.world.confirmed():
            if track.id in ignore:
                continue
            clearance = clearance_for(track.kind, ctx.config) + extra_clearance
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


def emergency_stop_needed(ctx):
    """Something too close to steer around. `(bool, why)`.

    The last line of defence, checked by every behaviour before it commits to
    anything. Deliberately based on the *nearest return*, not on a track: a
    track has to be confirmed over several sweeps before it exists, and
    something that appeared 1.5 m off the bow this instant does not get to wait
    three sweeps for confirmation.
    """
    boat, heading = ctx.boat, ctx.heading
    if boat is None or heading is None:
        return False, ""
    limit = ctx.config.MIN_OBSTACLE_RANGE_M + 1.0
    for track in ctx.world.all():
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
    """The ordinary way a behaviour drives: a deconflicted position target."""
    ok, why = emergency_stop_needed(ctx)
    if ok:
        return stop(why)
    aim, notes = deconflict(ctx, aim_xy)
    if notes:
        reason = f"{reason}; avoiding {notes[0]}"
    return goto(aim, speed, reason)


def creep(ctx, forward, desired_heading, reason, starboard=0.0):
    """Slow body-frame movement with a heading hold. Docking's workhorse."""
    from ..commander import yaw_rate_towards

    yaw = 0.0
    if desired_heading is not None and ctx.heading is not None:
        yaw = yaw_rate_towards(desired_heading, ctx.heading)
    return move(forward=forward, starboard=starboard, yaw_rate=yaw, reason=reason)
