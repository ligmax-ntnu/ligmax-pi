"""Role `hold`: arrive, then stay put. Njord §9.1's "stop at GPS point 4".

Station keeping is a *scored behaviour* and it is not the absence of one. The
task says the ASV "must stop at GPS point 4 and stay stationary there", and
Trondheim has a tide - a boat commanded to zero throttle in a current is a boat
leaving GPS point 4 at the speed of the current while its telemetry cheerfully
reports zero commanded speed.

So this runs the same closed loop the docking hold uses
(`commander.station_keep`): inside the tolerance it commands nothing, outside it
it pulls back proportionally, and the sideways thruster does the sideways part
so the hull does not have to keep turning to hold a spot.

`hold_s`
--------
    0 (the default)   hold for ever, until the operator advances or stops the
                      run. This is what GPS point 4 wants: the attempt is over,
                      and the boat sitting there is the finish line.
    > 0               hold that long and then advance to the next waypoint,
                      which is how a "wait here" is expressed mid-course.
"""

from ..commander import station_keep, stop
from .base import Behaviour, has_arrived, steer_towards


class Hold(Behaviour):
    """Run to the waypoint, then station-keep on it."""

    name = "hold"
    task = "transit"

    def start(self, ctx):
        super().start(ctx)
        self._arrived_at = None

    def update(self, ctx):
        target = ctx.target
        if target is None:
            return stop("no waypoint to hold on")

        if self._arrived_at is None:
            arrived, why = has_arrived(ctx)
            if not arrived:
                # Ease in: arriving at a hold point with way on means overshoot,
                # and an overshoot at the finish is visible to the jury.
                remaining = ctx.distance_to_target or 0.0
                speed = min(
                    ctx.cruise_speed,
                    max(ctx.config.DOCK_SPEED_MS, remaining * 0.35),
                )
                self.note(to_run_m=round(remaining, 1))
                return steer_towards(
                    ctx, target, speed,
                    f"running to {ctx.waypoint.name} to hold, {remaining:.0f} m",
                )
            self._arrived_at = ctx.now
            self.note(arrival=why)

        held = ctx.now - self._arrived_at
        required = ctx.waypoint.hold_seconds()
        self.note(
            held_s=round(held, 1),
            hold_required_s=required or "indefinite",
            stationary=ctx.state.stationary,
        )

        if required > 0.0 and held >= required:
            self.done = True
            return stop(f"held {ctx.waypoint.name} for {held:.0f} s")

        detail = (
            f"{held:.0f}/{required:.0f} s" if required > 0.0 else f"{held:.0f} s"
        )
        return station_keep(
            ctx.state, target, None, ctx.config,
            f"holding station at {ctx.waypoint.name}, {detail}",
        )
