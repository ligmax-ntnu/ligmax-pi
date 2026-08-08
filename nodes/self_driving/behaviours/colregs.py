"""Role `avoid`: transit while giving way to a vessel, per COLREG. Njord Task 2.

    "Go from this point to the next and watch out for a boat while you do it."

NJORD §9.2: from GPS point 5 to 6 through gates, at a set speed of 2 knots,
while the Otter closes at 2.5 knots from anywhere within +-100 degrees. COLREG
compliance is explicitly part of the score, so this is not "avoid the boat" - it
is "avoid the boat *the way a mariner would*", which is a much narrower target
and sometimes means deliberately doing nothing.

The rules that can arise here
-----------------------------
    Rule 14  head-on. Both vessels turn to STARBOARD and pass port to port.
    Rule 15  crossing. The vessel with the other on her own STARBOARD side
             gives way.
    Rule 16  the give-way vessel takes early and substantial action.
    Rule 17  the stand-on vessel keeps her course and speed - until it becomes
             clear the other is not acting, at which point she must.
    Rule 8   any alteration must be large enough to be "readily apparent to
             another vessel observing visually", and a succession of small
             alterations is specifically discouraged.
    Rule 2   nothing above excuses hitting anything.

Rule 8 is why the turn is one committed `COLREG_TURN_DEG` alteration rather than
a smooth continuous correction. A jury on the dock is the observing vessel, and
they have to be able to *see* the boat decide.

Rule 17 is the one that takes nerve to implement: with the Otter crossing from
our port side we are the stand-on vessel and the correct action is to hold
course and speed. A boat that swerves anyway is not safer, it is
non-compliant - and it is also less predictable to the vessel that is
manoeuvring around it. So this holds, watches, and only overrides when the CPA
keeps shrinking past the point where holding is defensible.

When to act, not just how
-------------------------
Everything keys off the closest point of approach (`geo.closest_point_of_approach`),
which answers "if neither of us does anything, how close do we get, and when".
Two numbers, and both matter: a CPA of 3 m in ninety seconds is not yet a
situation, and a CPA of 3 m in four seconds is an emergency. Acting on range
alone would have the boat manoeuvring around a vessel that is merely nearby and
opening.
"""

from .. import geo
from ..commander import move, stop
from .base import Behaviour, has_arrived, steer_towards
from .transit import Transit

# Rule 13's sector: a vessel more than 112.5 deg abaft the beam is overtaking.
OVERTAKING_DEG = 112.5

# How far past a crossing vessel's stern to aim. Rule 15's "avoid crossing
# ahead" made into a number.
ASTERN_MARGIN_M = 8.0

# The stand-on vessel holds - but not forever. Once the projected CPA falls
# below this, Rule 17(b) applies and we take action of our own.
STAND_ON_ABANDON_M = 4.0


class Colregs(Transit):
    """Transit to the waypoint, giving way to vessels as the rules require."""

    name = "avoid"
    task = "avoid"

    def start(self, ctx):
        super().start(ctx)
        self._committed = None       # the situation we have committed to, if any
        self._committed_at = None

    def update(self, ctx):
        if ctx.target is None:
            return stop("no waypoint to run to")

        arrived, why = has_arrived(ctx)
        if arrived:
            self.done = True
            self.note(arrival=why)
            return stop(f"waypoint {ctx.waypoint.name} reached: {why}")

        # NJORD §9.2 sets the task speed and requires the boat to be at it
        # immediately, so this leg does not use the cruise speed.
        speed = ctx.speed_limit(ctx.config.TASK_SPEED_MS)
        aim = self._aim(ctx)

        threat = self._threat(ctx)
        if threat is None:
            self._committed = None
            self.note(colreg="clear", vessels=len(ctx.world.vessels()))
            return steer_towards(
                ctx, aim, speed,
                f"running to {ctx.waypoint.name} at task speed, no vessel in the way",
            )

        track, cpa_m, tcpa_s, situation = threat
        self.note(
            colreg=situation,
            vessel_id=track.id,
            range_m=round(geo.distance(ctx.boat, track.pos), 1),
            cpa_m=round(cpa_m, 1),
            tcpa_s=round(tcpa_s, 1),
            vessel_speed=round(track.speed, 2),
        )

        # Rule 2, and it outranks everything above it. Astern rather than a
        # turn: inside four metres there is not room to turn out of the way, and
        # backing up buys time no alteration of course can.
        if geo.distance(ctx.boat, track.pos) <= ctx.config.COLREG_PANIC_M:
            return move(
                forward=-ctx.config.DOCK_REVERSE_SPEED_MS,
                reason=(
                    f"vessel #{track.id} {geo.distance(ctx.boat, track.pos):.1f} m "
                    "away - backing off (COLREG rule 2)"
                ),
            )

        if situation == "stand-on":
            # Rule 17. Hold course and speed. This is a decision, and it is
            # published as one so nobody watching thinks the boat has not seen
            # the Otter.
            if cpa_m < STAND_ON_ABANDON_M:
                return self._give_way(
                    ctx, track, speed,
                    "stand-on vessel not keeping clear - taking action "
                    "(COLREG rule 17b)",
                )
            return steer_towards(
                ctx, aim, speed,
                f"vessel #{track.id} crossing from port - standing on, holding "
                f"course and speed (COLREG rule 17); CPA {cpa_m:.0f} m in "
                f"{tcpa_s:.0f} s",
            )

        return self._give_way(ctx, track, speed, self._why(situation, track, cpa_m, tcpa_s))

    # ------------------------------------------------------------- the threat

    def _threat(self, ctx):
        """The vessel that needs a decision, or None. `(track, cpa, tcpa, kind)`.

        Only one at a time, and it is the one with the smallest CPA rather than
        the nearest: a vessel 20 m away and closing fast is the problem, not one
        10 m away and opening.
        """
        boat = ctx.boat
        if boat is None:
            return None
        own_velocity = ctx.state.world_velocity
        worst = None
        for track in ctx.world.vessels():
            distance = geo.distance(boat, track.pos)
            if distance > ctx.config.COLREG_DETECT_RANGE_M:
                continue
            tcpa, cpa = geo.closest_point_of_approach(
                boat, own_velocity, track.pos, track.velocity
            )
            # Already past its closest point: the range is opening and nothing
            # needs doing. Without this the boat manoeuvres to avoid a vessel it
            # has just cleared, which is both wrong and alarming to watch.
            if tcpa < 0.0:
                continue
            if tcpa > ctx.config.COLREG_HORIZON_S:
                continue
            if cpa > ctx.config.COLREG_MIN_CPA_M:
                continue
            if worst is None or cpa < worst[1]:
                worst = (track, cpa, tcpa, self._situation(ctx, track))
        return worst

    def _situation(self, ctx, track):
        """Which COLREG case this is, from the geometry alone.

        Sticky once committed: the relative bearing of a crossing vessel drifts
        as the encounter develops, and a boat that reclassifies mid-manoeuvre
        turns one way and then the other, which is exactly the "succession of
        small alterations" Rule 8 warns against.
        """
        if self._committed is not None and ctx.now - self._committed_at < 20.0:
            return self._committed

        bearing = geo.relative_bearing(track.pos, ctx.boat, ctx.heading)
        # The other vessel's own course, from its tracked velocity. Below a
        # walking pace its direction is noise, and a vessel that is not moving
        # is an obstacle rather than a COLREG situation.
        if track.speed < 0.2:
            situation = "obstacle"
        elif abs(bearing) <= ctx.config.COLREG_HEADON_DEG and self._reciprocal(ctx, track):
            situation = "head-on"
        elif abs(bearing) >= OVERTAKING_DEG:
            situation = "overtaking"
        elif bearing > 0.0:
            situation = "give-way"   # on our starboard: rule 15, we give way
        else:
            situation = "stand-on"   # on our port: rule 17, we hold

        self._committed = situation
        self._committed_at = ctx.now
        return situation

    def _reciprocal(self, ctx, track):
        """Whether the other vessel is coming more or less straight at us."""
        course = geo.bearing_to((0.0, 0.0), track.velocity)
        return abs(geo.angle_diff(course, ctx.heading + 180.0)) <= 40.0

    # ------------------------------------------------------------- the action

    def _give_way(self, ctx, track, speed, reason):
        """Rule 16: early and substantial, to starboard, passing astern.

        The aim point is placed astern of the vessel's *predicted* position
        rather than beside its current one, which is what "do not cross ahead"
        means once both boats are moving. If it has no usable velocity the
        fallback is a plain committed turn to starboard, which is the right
        default for every case in the book.
        """
        boat = ctx.boat
        if track.speed >= 0.2:
            # Where it will be when we get there, then a margin further back
            # along its own course - i.e. behind its stern.
            distance = geo.distance(boat, track.pos)
            seconds = distance / max(0.3, ctx.state.speed)
            ahead = track.predicted(min(seconds, ctx.config.COLREG_HORIZON_S))
            course = geo.bearing_to((0.0, 0.0), track.velocity)
            aim = geo.offset_point(ahead, course + 180.0, ASTERN_MARGIN_M)
            self.note(action="passing astern", astern_of=[round(aim[0], 1), round(aim[1], 1)])
        else:
            aim = geo.offset_point(
                boat, ctx.heading + ctx.config.COLREG_TURN_DEG,
                max(8.0, ctx.config.LOOKAHEAD_M),
            )
            self.note(action=f"turning {ctx.config.COLREG_TURN_DEG:.0f} deg to starboard")

        # Extra room on top of the ordinary clearance while a situation is live.
        aim, _notes = self._widen(ctx, aim)
        return steer_towards(ctx, aim, speed, reason)

    def _widen(self, ctx, aim):
        from .base import deconflict

        return deconflict(ctx, aim, extra_clearance=2.0)

    def _why(self, situation, track, cpa_m, tcpa_s):
        detail = f"CPA {cpa_m:.0f} m in {tcpa_s:.0f} s"
        if situation == "head-on":
            return (
                f"vessel #{track.id} head-on - turning to starboard to pass port "
                f"to port (COLREG rule 14); {detail}"
            )
        if situation == "give-way":
            return (
                f"vessel #{track.id} crossing from starboard - giving way, "
                f"passing astern (COLREG rule 15); {detail}"
            )
        if situation == "overtaking":
            return (
                f"overtaking vessel #{track.id} - keeping clear "
                f"(COLREG rule 13); {detail}"
            )
        return f"vessel #{track.id} in the way - keeping clear; {detail}"
