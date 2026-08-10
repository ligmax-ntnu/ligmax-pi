"""Role `buoys`: transit obeying the lateral marks and the cardinals.

    "Follow these GPS points, but follow buoy rules."

Njord Task 1 part 2 and the gates of Task 2. Two separate rules apply, and they
are enforced differently because they *are* different.

Lateral marks (red / green) - a constraint on the corridor
-----------------------------------------------------------
IALA region A, direction of buoyage seaward = north (NJORD §10.2):

    sailing with the buoyage:  red to PORT,  green to STARBOARD
    sailing against it:        the sense inverts

"Against it" is not hypothetical - a course that runs back down itself does it,
and a boat that applies the outbound rule on the return passes every mark on the
wrong side while being confident and consistent about it. So the direction of
buoyage is carried per leg (`plan.channel_bearing`) and compared against the
leg's own bearing, rather than assumed.

Enforcement is a *shift of the corridor*, not a detour. The leg is projected,
each mark's signed cross-track is measured against it, and a mark on the wrong
side pushes the aim point far enough sideways that it ends up on the right one.
That keeps the boat on a single smooth line through a series of marks instead of
dodging each in turn - which matters, because Task 2's gates come 20-80 m apart
and a boat that recentres between each one arrives at the next one crooked.

Cardinal marks - a constraint on the position
----------------------------------------------
A cardinal says which side of *it* is safe water (NJORD §10.3): a north cardinal
is passed to its north, an east cardinal to its east. That is a much stronger
statement than a lateral mark makes, and it is enforced by routing through an
explicit via-point on the safe side, so the boat visibly goes round rather than
grazing past.

The hard part is that **no lidar can tell a north cardinal from a south one.**
The difference is the topmark's two black cones. Only the camera can see it, the
detector on this boat is weak, and `perception/classify.CardinalVote` therefore
refuses to commit until several observations agree.

So there are two cases and they are handled differently on purpose:

    committed    route through the safe side. This is the scoring case.
    uncommitted  the mark exists (the lidar is sure) but its type does not. Do
                 NOT guess - a coin flip is a 50 % chance of the exact failure
                 the task is scoring. Keep the planned route, widen the
                 clearance, slow down, and say so loudly enough that the
                 operator can use §8.2's twenty seconds if the camera never
                 makes up its mind.
"""

from .. import geo
from ..commander import stop
from ..obsticales import (
    BUOY_TYPES,
    CARDINAL_SAFE_BEARING,
    ObstacleType,
)
from . import alternation
from .base import (
    Behaviour,
    corner_speed_limit,
    dynamic_clearance,
    has_arrived,
    steer_towards,
)
from .transit import Transit

# How far past a cardinal's safe side to aim. The mark is 40 cm across; this is
# the room the hull and the position error need on top of that.
CARDINAL_OFFSET_M = 4.0

# A cardinal is routed around once it is inside this range. Further out the
# camera has not had a good look at it yet and the vote is not worth acting on.
CARDINAL_ENGAGE_M = 18.0

# A mark is only enforced while it is still ahead. Once the boat is past it the
# rule has been obeyed or broken, and steering for it now only makes things
# worse.
ENFORCE_AHEAD_M = -1.0


class Buoys(Transit):
    """Transit, plus the lateral rule and the cardinals."""

    name = "buoys"
    task = "buoys"

    def update(self, ctx):
        if ctx.target is None:
            return stop("no waypoint to run to")

        arrived, why = has_arrived(ctx)
        if arrived:
            self.done = True
            self.note(arrival=why)
            return stop(f"waypoint {ctx.waypoint.name} reached: {why}")

        aim = self._aim(ctx)
        speed = ctx.caution_speed
        reason = f"running to {ctx.waypoint.name} under buoy rules"

        # A cardinal is a stronger constraint than a lateral mark, so it is
        # resolved first and, when it applies, it replaces the aim point rather
        # than nudging it.
        via, cardinal_note, cardinal_speed = self._cardinal(ctx)
        if via is not None:
            aim = via
            reason = cardinal_note
            speed = min(speed, cardinal_speed)
        else:
            aim, lateral_notes = self._lateral(ctx, aim)
            if lateral_notes:
                reason = f"{reason}; {lateral_notes[0]}"
            self.note(lateral=lateral_notes)

        # Last, and applied to whatever the marks left: a via-point round a
        # cardinal is still approached into the same corner, and the corner is
        # what decides whether the boat can hold the line it just chose.
        speed, pacing = corner_speed_limit(ctx, speed)
        if pacing:
            reason = f"{reason}; {pacing}"

        self.note(
            to_run_m=round(ctx.distance_to_target or 0.0, 1),
            marks=len(ctx.world.marks()),
            cardinal=cardinal_note or "none in range",
            speed_ms=round(speed, 2),
            room_for_speed_m=round(dynamic_clearance(ctx), 2),
            pacing=pacing or "clear ahead",
        )
        return steer_towards(ctx, aim, speed, reason)

    # ------------------------------------------------------------- lateral

    def _lateral(self, ctx, aim):
        """Shift the aim point so every red/green mark ends up on its legal side.

        Returns `(aim, notes)`. Notes are sentences for the operator, because
        NJORD §11.4 scores the boat explaining what a detection did to its plan.
        """
        if ctx.leg is None or ctx.boat is None:
            return aim, []

        outbound = self._with_the_buoyage(ctx)
        # Sailing with the buoyage, red belongs to port - i.e. at negative
        # cross-track relative to the leg direction. Against it, swap.
        required = {
            ObstacleType.RED: -1.0 if outbound else +1.0,
            ObstacleType.GREEN: +1.0 if outbound else -1.0,
        }

        notes = []
        shifted = aim
        # The room the speed itself needs, once for the leg - see
        # `base.dynamic_clearance`. Zero on every profile but `fast`.
        for_speed = dynamic_clearance(ctx)
        for track in ctx.world.marks():
            if track.kind not in BUOY_TYPES:
                continue
            # Per mark, not once for the leg: a remembered buoy has to be cleared
            # by its own uncertainty as well as the rule's margin, or the boat
            # passes what it believes is the legal side of a mark that is
            # actually several metres the other way - which scores as passing on
            # the wrong side, the exact failure this behaviour exists to avoid.
            clearance = ctx.config.BUOY_CLEARANCE_M + track.sigma_m + for_speed
            _t, along, cross = geo.project_onto_leg(track.pos, ctx.boat, shifted)
            if along < ENFORCE_AHEAD_M:
                continue  # already passed it
            if along > geo.distance(ctx.boat, shifted) + clearance:
                continue  # beyond the aim point; a later tick's problem
            want = required[track.kind]
            # `cross` is where the mark is relative to our line: positive means
            # to starboard. We need `cross * want > 0`, i.e. the mark on the
            # side the rule says, by at least `clearance`.
            if cross * want >= clearance:
                continue
            shortfall = clearance - cross * want
            # Move the line the other way from the mark, which puts the mark on
            # the required side of it.
            bearing = geo.bearing_to(ctx.boat, shifted)
            shifted = geo.offset_point(shifted, bearing + 90.0 * -want, shortfall)
            side = "port" if want < 0 else "starboard"
            notes.append(
                f"{track.kind.name.lower()} #{track.id} at "
                f"{geo.distance(ctx.boat, track.pos):.0f} m must be to {side} - "
                f"shifting {shortfall:.1f} m"
            )
        return shifted, notes

    def _with_the_buoyage(self, ctx):
        """Whether this leg runs with the direction of buoyage.

        Within 90 deg of the channel bearing counts as with it. The boundary
        case - a leg exactly across the channel - has no correct answer from
        the marks alone, which is precisely why `plan.py` lets the operator set
        `channel_bearing` per waypoint and override this.
        """
        channel = ctx.plan.bearing_of_buoyage(ctx.waypoint)
        if ctx.leg is None:
            leg_bearing = ctx.heading if ctx.heading is not None else channel
        else:
            leg_bearing = geo.bearing_to(ctx.leg[0], ctx.leg[1])
        return abs(geo.angle_diff(leg_bearing, channel)) <= 90.0

    # ------------------------------------------------------------ cardinal

    def _cardinal(self, ctx):
        """`(via_point or None, note, speed)` for the nearest cardinal ahead."""
        boat = ctx.boat
        if boat is None:
            return None, "", ctx.caution_speed

        nearest = None
        for track in ctx.world.marks():
            if track.kind not in CARDINAL_SAFE_BEARING and (
                track.kind != ObstacleType.CARDINAL
            ):
                continue
            distance = geo.distance(boat, track.pos)
            if distance > CARDINAL_ENGAGE_M:
                continue
            if ctx.leg is not None:
                _t, along, _cross = geo.project_onto_leg(
                    track.pos, ctx.leg[0], ctx.leg[1]
                )
                if along < ENFORCE_AHEAD_M:
                    continue
            if nearest is None or distance < nearest[0]:
                nearest = (distance, track)

        if nearest is None:
            return None, "", ctx.caution_speed
        distance, track = nearest

        outbound = self._with_the_buoyage(ctx)
        safe_bearing = CARDINAL_SAFE_BEARING.get(track.kind)
        side_from = "the camera"
        speed = ctx.caution_speed

        if safe_bearing is None:
            # Seen, but the camera has not committed. The default is still not to
            # guess - see the module docstring; a coin flip is a 50 % chance of
            # the exact failure the task is scoring.
            #
            # Unless the operator has switched the alternation prior on, in which
            # case there is something better than a coin flip available: the side
            # the mark before this one forced. It is still an inference, so it
            # buys a route rather than a commitment - `alternation.resolve` never
            # writes to the camera's poll, and the speed stays down.
            guess, why = alternation.resolve(ctx, track, outbound)
            self.note(cardinal_unresolved=track.cardinal.describe())
            if guess is None:
                if why:
                    self.note(alternation=why)
                # Keep the planned line, give it more room (which `deconflict`
                # does through the extra clearance) and slow down so there is
                # still time to act if the vote lands late.
                return (
                    None,
                    f"cardinal #{track.id} at {distance:.0f} m - "
                    f"{track.cardinal.describe()}; holding the planned line",
                    ctx.config.DOCK_SPEED_MS * 2.0,
                )
            safe_bearing = CARDINAL_SAFE_BEARING[guess]
            side_from = "the alternating pattern"
            self.note(alternation=why)
            # Still slower than a committed pass. The prior is good enough to
            # pick a side and not good enough to hurry through it.
            speed = min(speed, ctx.config.CAUTION_SPEED_MS)
            track = _Assumed(track, guess, why)
        else:
            clash = alternation.disagreement(ctx, track, outbound)
            if clash:
                self.note(alternation=clash)

        # The via-point is pushed out by the mark's own uncertainty too. A
        # cardinal says which side of *it* is safe water, so being on the wrong
        # side by the width of the position error is the whole failure.
        via = geo.offset_point(
            track.pos, safe_bearing, CARDINAL_OFFSET_M + track.sigma_m
        )
        # Only route through the via-point while the boat is not yet past it;
        # after that the waypoint itself is the target again.
        if geo.distance(boat, via) < 2.0:
            return None, (
                f"{track.kind.name.lower()} cardinal #{track.id} cleared"
            ), speed
        return (
            via,
            f"{track.kind.name.lower()} cardinal #{track.id} at {distance:.0f} m - "
            f"passing on its {track.kind.name.lower()} side, per {side_from}",
            speed,
        )


class _Assumed:
    """A track wearing a cardinal type the prior suggested, for this tick only.

    The alternative would be writing the guess onto the real track, and that is
    the one thing this feature must not do: `Track.kind` is what the survey file
    and the operator's chart are built from, and a guess written there outlives
    the leg it was guessed on, gets saved between attempts, and comes back as
    fact. So the guess lives for the length of one `_cardinal` call and dies with
    it, and the mark on disk stays an uncommitted cardinal until a camera says
    otherwise.
    """

    __slots__ = ("_track", "kind", "assumed_why")

    def __init__(self, track, kind, why):
        self._track = track
        self.kind = kind
        self.assumed_why = why

    def __getattr__(self, name):
        return getattr(self._track, name)
