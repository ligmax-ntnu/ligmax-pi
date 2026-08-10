"""Role `transit`: drive to the waypoint. Njord Task 1 part 1.

    "Follow these GPS points blindly."

Blind means *ignore the marks' meaning*, not ignore the world - the boat still
must not hit anything (NJORD §9.1: "contact with the buoys should be avoided"),
it just does not care which side it passes them.

Pure pursuit, and why the aim point is not the waypoint
-------------------------------------------------------
Steering straight at a waypoint is the obvious thing and it weaves. The further
off the track the boat is, the harder it turns towards the point, and it arrives
with all of that turn still wound in - so it crosses the line, turns back, and
oscillates the whole way down the leg. On a light trimaran in a Trondheim tide
the oscillation is metres, and NJORD §11.4 has the jury watching the actual
track against the ideal route.

Aiming at a point `LOOKAHEAD_M` further along the *leg* fixes it: the cross
track error and the correction go to zero together, so the boat slides onto the
line and stays there. The lookahead shrinks as the waypoint approaches so the
last few metres aim at the mark itself and the boat actually arrives.

The one thing this does that "blind" does not suggest
-----------------------------------------------------
It still deconflicts (`base.deconflict`). A blind leg with a buoy sitting on the
line is a leg with a buoy sitting on the line.
"""

from .. import geo
from ..commander import stop
from .base import (
    Behaviour,
    corner_speed_limit,
    has_arrived,
    lookahead_for,
    steer_towards,
)


class Transit(Behaviour):
    """Follow the leg to the current waypoint, avoiding whatever is in the way."""

    name = "transit"
    task = "transit"

    def update(self, ctx):
        if ctx.target is None:
            return stop("no waypoint to run to")

        arrived, why = has_arrived(ctx)
        if arrived:
            self.done = True
            self.note(arrival=why)
            return stop(f"waypoint {ctx.waypoint.name} reached: {why}")

        aim = self._aim(ctx)
        speed, pacing = self._speed(ctx)
        reason = f"running to {ctx.waypoint.name}, {ctx.distance_to_target:.0f} m"
        if pacing:
            reason = f"{reason}; {pacing}"
        self.note(
            to_run_m=round(ctx.distance_to_target or 0.0, 1),
            cross_track_m=round(self._cross_track(ctx), 2),
            aim=[round(aim[0], 1), round(aim[1], 1)],
            speed_ms=round(speed, 2),
            pacing=pacing or "clear ahead",
        )
        return steer_towards(ctx, aim, speed, reason)

    # ------------------------------------------------------------------ parts

    def _aim(self, ctx):
        """The pure-pursuit point on the leg, or the waypoint if there is no leg."""
        if ctx.leg is None:
            return ctx.target
        remaining = ctx.distance_to_target or 0.0
        return geo.lookahead_point(
            ctx.boat, ctx.leg[0], ctx.leg[1], lookahead_for(ctx, remaining)
        )

    def _speed(self, ctx):
        """`(speed, note)`. Cruise, paced by the corner and by the arrival.

        Three things pull it down and the smallest wins:

        **The corner ahead** (`base.corner_speed_limit`). On a slalom this is
        what makes a fast attempt possible at all - the straights get the knots
        and the turns do not.

        **The arrival.** A boat that reaches a settle waypoint with way on
        overshoots it, and NJORD §9.1 scores the boat being stationary at GPS
        point 4. Not elegance: points.

        **The profile's own cruise**, which is where it starts.
        """
        cruise = ctx.cruise_speed
        speed, pacing = corner_speed_limit(ctx, cruise)
        remaining = ctx.distance_to_target
        if remaining is None:
            return speed, pacing
        slow_from = max(ctx.acceptance_radius() * 2.0, 4.0)
        if remaining >= slow_from:
            return speed, pacing
        floor = ctx.config.DOCK_SPEED_MS if ctx.waypoint.settles else cruise * 0.5
        arriving = max(floor, cruise * (remaining / slow_from))
        if arriving < speed:
            return arriving, f"{remaining:.0f} m to run - easing off for the arrival"
        return speed, pacing

    def _cross_track(self, ctx):
        """Metres off the ideal route - the figure NJORD §11.4 asks to be shown."""
        if ctx.leg is None or ctx.boat is None:
            return 0.0
        _t, _along, cross = geo.project_onto_leg(ctx.boat, ctx.leg[0], ctx.leg[1])
        return cross
