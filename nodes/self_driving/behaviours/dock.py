"""Roles `dock` and `dock_parallel`: Njord Task 3, both halves.

    3.1 normal    a 2 m x 2 m berth. Enter, hold 10 s, REVERSE out.
    3.2 parallel  a 2 m x 4 m berth. Come alongside, hold 5 s, continue forward.

Why the berth is found with the lidar and not with the AR tags
--------------------------------------------------------------
NJORD §10.4 makes the three 18 cm AR tags **optional and awards bonus points for
docking without them**, and never publishes the tag family or the IDs. Against
that, a berth is a **2 m gap between two structures**, and the C1 measures a gap
to +-3 cm whether the tag is wet, shaded, or facing the wrong way. Geometry is
both the more reliable signal here and the one worth more points, so it is the
primary and the only one implemented.

The margin is the whole problem. A 2 m berth is not much wider than this boat
with its amas, so the useful tolerance is tens of centimetres - which is why
every phase below is body-frame velocity rather than a position target. A GUIDED
position target inside a berth would have ArduPilot's L1 controller trying to
*drive to a point*, complete with its turn radius, in a space with no room to
turn. Creeping under a heading hold is the only shape that fits.

The phases, and why each one exists
-----------------------------------
    SEARCH   drive to the operator's dock waypoint and look for a gap. The
             waypoint is where the dock is; the lidar decides where the berth is.
    ALIGN    hold station on the berth's centreline, one standoff back, and
             square up. **The boat must be straight before it is committed** -
             a 2 m berth entered 15 deg crooked is a collision, and a collision
             is a scored deduction.
    ENTER    creep in, holding the berth's bearing, until deep enough or until
             something is close ahead.
    HOLD     station-keep for the rule's duration. In a tide "stay stationary"
             is a controller, not an absence of one.
    EXIT     astern for `dock` (the rules say reverse out), ahead for
             `dock_parallel`, until clear.

Parallel docking, and the third thruster
----------------------------------------
The boat has a sideways-only thruster, so parallel docking is a genuine crab:
square up parallel to the dock face, then translate sideways until alongside.
With `LATERAL_MODE=none` that is not available and `_crab` degrades to an angled
approach - slower and less tidy, but it still ends up alongside.
"""

from .. import geo
from ..commander import move, station_keep, stop
from ..perception.cluster import split_by_gap
from .base import Behaviour, creep

SEARCH = "search"
ALIGN = "align"
ENTER = "enter"
HOLD = "hold"
EXIT = "exit"

# How long to look for a berth before admitting the search has failed. NJORD
# §8.2 gives the crew twenty seconds to take over, so saying so at fifteen
# leaves them all twenty.
SEARCH_TIMEOUT_S = 15.0

# Something this close ahead while entering is the back of the berth.
BERTH_END_M = 1.0

# How close alongside to finish a parallel dock.
ALONGSIDE_M = 0.8


class Dock(Behaviour):
    """Bow-in docking. `parallel=True` switches to the alongside variant."""

    name = "dock"
    task = "dock"

    def __init__(self, config, parallel=False):
        super().__init__(config)
        self.parallel = parallel
        self.name = "dock_parallel" if parallel else "dock"
        self.phase = SEARCH
        self.berth = None          # (centre_xy, bearing_deg) in the WORLD frame
        self._phase_at = None
        self._hold_from = None

    def start(self, ctx):
        super().start(ctx)
        self.phase = SEARCH
        self.berth = None
        self._phase_at = ctx.now
        self._hold_from = None

    # ------------------------------------------------------------------ tick

    def update(self, ctx):
        if ctx.boat is None or ctx.heading is None:
            return stop("docking needs a position and a heading")

        # Re-measure the berth every tick while it is still visible. A floating
        # dock moves, and the boat's own position error grows; a berth captured
        # once at 8 m and then trusted at 1 m is how you hit the pontoon.
        measured = self._find_berth(ctx)
        if measured is not None:
            self.berth = measured

        handler = {
            SEARCH: self._search,
            ALIGN: self._align,
            ENTER: self._enter,
            HOLD: self._hold,
            EXIT: self._exit,
        }[self.phase]
        intent = handler(ctx)

        self.note(
            phase=self.phase,
            phase_s=round(ctx.now - (self._phase_at or ctx.now), 1),
            berth=(
                [round(self.berth[0][0], 2), round(self.berth[0][1], 2)]
                if self.berth
                else None
            ),
            berth_bearing=round(self.berth[1], 1) if self.berth else None,
            using_ar_tags=False,
        )
        return intent

    def _to(self, phase, ctx):
        if self.phase != phase:
            self.phase = phase
            self._phase_at = ctx.now

    # ---------------------------------------------------------------- phases

    def _search(self, ctx):
        """Get to the dock waypoint and look for the gap."""
        if self.berth is not None:
            self._to(ALIGN, ctx)
            return self._align(ctx)

        target = ctx.target
        if target is None:
            return stop("no dock waypoint")

        distance = geo.distance(ctx.boat, target)
        elapsed = ctx.now - (self._phase_at or ctx.now)
        if distance <= ctx.config.ARRIVAL_RADIUS_M:
            # At the waypoint with nothing found. Hold and keep looking rather
            # than wandering: a boat that drifts while searching loses the
            # geometry it was searching from.
            if elapsed > SEARCH_TIMEOUT_S:
                self.note(
                    stuck=(
                        f"no {self._berth_width():.0f} m berth found in "
                        f"{elapsed:.0f} s - take over and reposition"
                    )
                )
            return station_keep(
                ctx.state, target, ctx.heading, ctx.config,
                f"at the dock waypoint, looking for a "
                f"{self._berth_width():.0f} m berth ({elapsed:.0f} s)",
                ceiling=ctx.ceiling,
            )

        from .base import steer_towards

        return steer_towards(
            ctx, target, _under(ctx, ctx.config.DOCK_SPEED_MS * 2.0),
            f"approaching the dock, {distance:.0f} m, looking for the berth",
        )

    def _align(self, ctx):
        """Sit on the approach point and square up before committing."""
        if self.berth is None:
            self._to(SEARCH, ctx)
            return self._search(ctx)

        centre, bearing = self.berth
        approach = self._approach_point(centre, bearing)
        desired = self._approach_heading(bearing)

        offset = geo.distance(ctx.boat, approach)
        misalignment = abs(geo.angle_diff(desired, ctx.heading))
        self.note(align_offset_m=round(offset, 2), align_error_deg=round(misalignment, 1))

        if (
            offset <= ctx.config.HOLD_TOLERANCE_M
            and misalignment <= ctx.config.DOCK_ALIGN_TOLERANCE_DEG
            and ctx.state.speed < ctx.config.STATIONARY_SPEED_MS * 2.0
        ):
            self._to(ENTER, ctx)
            return self._enter(ctx)

        return station_keep(
            ctx.state, approach, desired, ctx.config,
            f"squaring up for the berth ({offset:.1f} m off, "
            f"{misalignment:.0f} deg out)",
            ceiling=ctx.ceiling,
        )

    def _enter(self, ctx):
        """Creep in. Bow first, or sideways for the parallel case."""
        if self.berth is None:
            # Lost sight of it mid-entry. Stop rather than continue blind - the
            # remembered berth is exactly as stale as the reason it vanished.
            self._to(ALIGN, ctx)
            return stop("lost the berth mid-entry - stopping to re-acquire")

        centre, bearing = self.berth
        desired = self._approach_heading(bearing)

        if self.parallel:
            return self._crab(ctx, centre, desired)

        # Bow-in: how far into the berth are we?
        _stbd, forward = geo.world_to_boat(
            centre[0] - ctx.boat[0], centre[1] - ctx.boat[1], ctx.heading
        )
        remaining = forward + ctx.config.DOCK_ENTRY_DEPTH_M
        blocked = self._something_close_ahead(ctx)
        self.note(berth_remaining_m=round(remaining, 2), blocked_ahead=blocked)

        if remaining <= 0.15 or blocked:
            self._to(HOLD, ctx)
            self._hold_from = ctx.now
            return self._hold(ctx)

        # Lateral trim while entering, so a 2 m berth is entered down the middle
        # rather than down one wall.
        sideways, _f = geo.world_to_boat(
            centre[0] - ctx.boat[0], centre[1] - ctx.boat[1], ctx.heading
        )
        lateral = _under(ctx, ctx.config.LATERAL_MAX_MS)
        return creep(
            ctx,
            forward=_under(ctx, ctx.config.DOCK_SPEED_MS),
            desired_heading=desired,
            starboard=_clamp(sideways * 0.5, -lateral, lateral),
            reason=f"entering the berth, {remaining:.2f} m to go",
        )

    def _crab(self, ctx, centre, desired):
        """Parallel docking: hold the dock's heading and translate sideways."""
        sideways, forward = geo.world_to_boat(
            centre[0] - ctx.boat[0], centre[1] - ctx.boat[1], ctx.heading
        )
        gap = abs(sideways) - ALONGSIDE_M
        self.note(alongside_gap_m=round(gap, 2), fore_aft_error_m=round(forward, 2))

        if gap <= 0.1 and abs(forward) < 0.5:
            self._to(HOLD, ctx)
            self._hold_from = ctx.now
            return self._hold(ctx)

        from ..config import LATERAL_MODE

        if LATERAL_MODE == "none":
            # No sideways thrust: approach at an angle instead. Aim at a point
            # alongside and let the heading hold straighten the boat as it
            # arrives. Slower and less tidy, and it still ends up alongside.
            target = geo.offset_point(
                centre,
                geo.bearing_to(centre, ctx.boat),
                ALONGSIDE_M,
            )
            return station_keep(
                ctx.state, target, desired, ctx.config,
                f"no lateral thruster configured - angling alongside, "
                f"{gap:.2f} m off",
                ceiling=ctx.ceiling,
            )

        ahead = _under(ctx, ctx.config.DOCK_SPEED_MS)
        lateral = _under(ctx, ctx.config.LATERAL_MAX_MS)
        return creep(
            ctx,
            forward=_clamp(forward * 0.4, -ahead, ahead),
            desired_heading=desired,
            starboard=_clamp(sideways * 0.5, -lateral, lateral),
            reason=f"crabbing alongside, {gap:.2f} m off the dock",
        )

    def _hold(self, ctx):
        """Stay put for the rule's duration. A controller, because of the tide."""
        required = ctx.waypoint.hold_seconds() if ctx.waypoint else self._default_hold()
        held = ctx.now - (self._hold_from or ctx.now)
        self.note(held_s=round(held, 1), hold_required_s=required)

        if held >= required:
            self._to(EXIT, ctx)
            return self._exit(ctx)

        target = self.berth[0] if self.berth else ctx.boat
        desired = self._approach_heading(self.berth[1]) if self.berth else ctx.heading
        if self.parallel and self.berth:
            # Alongside, not on top of: hold a boat's width off the dock face.
            target = geo.offset_point(
                self.berth[0], geo.bearing_to(self.berth[0], ctx.boat), ALONGSIDE_M
            )
        return station_keep(
            ctx.state, target, desired, ctx.config,
            f"docked - holding {held:.0f}/{required:.0f} s"
            f"{'' if ctx.state.stationary else ' (still moving)'}",
            ceiling=ctx.ceiling,
        )

    def _exit(self, ctx):
        """Out. Astern for a bow-in dock, ahead for a parallel one."""
        if self.berth is None:
            self.done = True
            return stop("clear of the berth")

        distance = geo.distance(ctx.boat, self.berth[0])
        target = ctx.config.DOCK_EXIT_M
        self.note(exit_distance_m=round(distance, 2))

        if distance >= target:
            self.done = True
            return stop(f"clear of the berth ({distance:.1f} m)")

        desired = self._approach_heading(self.berth[1])
        if self.parallel:
            # NJORD §9.3: after the parallel hold the boat "moves forward toward
            # the next GPS point" - so this one leaves ahead.
            return creep(
                ctx, forward=ctx.config.DOCK_SPEED_MS, desired_heading=desired,
                reason=f"leaving the berth ahead, {distance:.1f}/{target:.1f} m",
            )
        # NJORD §9.3: normal docking reverses out. Astern with the heading still
        # held, so the boat backs straight out rather than swinging a quarter
        # into the pontoon.
        return creep(
            ctx, forward=-ctx.config.DOCK_REVERSE_SPEED_MS, desired_heading=desired,
            reason=f"reversing out, {distance:.1f}/{target:.1f} m",
        )

    # -------------------------------------------------------- berth geometry

    def _berth_width(self):
        return (
            self.config.DOCK_BERTH_PARALLEL_M
            if self.parallel
            else self.config.DOCK_BERTH_WIDTH_M
        )

    def _find_berth(self, ctx):
        """`(centre_xy, bearing_deg)` in the world frame, or None.

        The berth is the gap of the right width between two structures. Its
        *bearing* is the direction pointing into it - perpendicular to the line
        joining the two gap edges, on the side away from the boat.

        A width the operator set on the waypoint wins over the rulebook default,
        because the handbook's figures are nominal and the thing in the water is
        what the boat has to fit into.
        """
        clusters = ctx.clusters
        if not clusters:
            return None

        width = self._berth_width()
        if ctx.waypoint is not None and ctx.waypoint.berth_width_m is not None:
            width = ctx.waypoint.berth_width_m

        candidates = split_by_gap(clusters, width, ctx.config.DOCK_GAP_TOLERANCE_M)
        for left, right, separation, midpoint in candidates:
            # Both sides have to be a wall, not a passing buoy. Without this a
            # red and a green buoy the right distance apart read as a berth.
            if min(left.width_m, right.width_m) < ctx.config.DOCK_WALL_MIN_M:
                continue
            centre_world = self._to_world(ctx, midpoint)
            bearing = self._berth_bearing(ctx, left, right, midpoint)
            if bearing is None:
                continue
            self.note(berth_gap_m=round(separation, 2))
            return centre_world, bearing
        return None

    def _berth_bearing(self, ctx, left, right, midpoint):
        """Which way the berth opens, as a world compass bearing.

        The two gap edges define a line; the berth runs perpendicular to it. Of
        the two perpendiculars, the one pointing *away* from the boat is the one
        that goes into the berth - the other one goes back out to sea.
        """
        dx = right.nearest[0] - left.nearest[0]
        dy = right.nearest[1] - left.nearest[1]
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return None
        # Perpendicular in the boat frame, then pick the sign that points away
        # from the boat (the boat is at the boat frame's origin).
        for perpendicular in ((-dy, dx), (dy, -dx)):
            if perpendicular[0] * midpoint[0] + perpendicular[1] * midpoint[1] > 0.0:
                east, north = geo.boat_to_world(
                    perpendicular[0], perpendicular[1], ctx.heading
                )
                return geo.bearing_to((0.0, 0.0), (east, north))
        return None

    def _to_world(self, ctx, boat_point):
        east, north = geo.boat_to_world(boat_point[0], boat_point[1], ctx.heading)
        return (ctx.boat[0] + east, ctx.boat[1] + north)

    def _approach_point(self, centre, bearing):
        """Where to sit while squaring up: one standoff back out of the berth."""
        return geo.offset_point(centre, bearing + 180.0, self.config.DOCK_STANDOFF_M)

    def _approach_heading(self, bearing):
        """Which way to point. Into the berth bow-first, or along it alongside."""
        if self.parallel:
            # Parallel to the dock face, which is 90 deg off the berth's axis.
            return geo.wrap360(bearing + 90.0)
        return bearing

    def _default_hold(self):
        return (
            self.config.DOCK_PARALLEL_HOLD_S
            if self.parallel
            else self.config.DOCK_HOLD_S
        )

    def _something_close_ahead(self, ctx):
        """The back of the berth, or anything else the bow is about to touch.

        Only things measured in the last second count. A remembered structure -
        the world model now keeps established static marks indefinitely, with a
        position uncertain by metres - would otherwise stop the approach dead a
        boat-length short of a berth that is perfectly clear, and no amount of
        looking at it would clear the belief. Inside a berth the lidar is a metre
        or two from the walls and hitting them every sweep, so nothing real is
        lost by insisting the evidence be current.
        """
        from .base import EMERGENCY_FRESH_S

        for track in ctx.world.all():
            if track.age(ctx.now) > EMERGENCY_FRESH_S:
                continue
            if abs(geo.relative_bearing(track.pos, ctx.boat, ctx.heading)) > 35.0:
                continue
            if geo.distance(ctx.boat, track.pos) <= BERTH_END_M:
                return True
        return False


def _clamp(value, low, high):
    return max(low, min(high, value))


def _under(ctx, wanted):
    """`wanted`, held under the operator's speed setting. Every speed in here.

    The docking figures are already slow - 0.3 m/s in, 0.35 m/s sideways - so at
    the ordinary setting this changes nothing. It matters at the other end: an
    operator who sets 0.1 m/s for a first berth attempt means the berth attempt,
    and a manoeuvre that ignored the setting and crept in at three times it would
    be the one place on the boat where the number on the panel was a decoration.

    A cap and never a floor, so a high setting cannot make a berth approach brisk.
    """
    return min(float(wanted), ctx.ceiling)
