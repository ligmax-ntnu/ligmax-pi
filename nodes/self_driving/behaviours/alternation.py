"""The alternation prior: what the marks the boat has already read imply.

    side, why = expected_side(ctx, track, outbound)
    kind, margin = best_cardinal(side, leg_bearing, config)

**Off unless an operator switches it on** (`config.ALTERNATION_DEFAULT`, the
`alternation` command). Everything below is a guess. A good one, but a guess, and
it exists for one situation only: the camera never committed, the mark is 15 m
away and closing, and the boat has to pass it on one side or the other.

What it is, and what it deliberately is not
-------------------------------------------
The prior is a general fact about how marks are laid along any run of water:

    consecutive marks alternate the side you pass them on.

A mark that pushes you the same way as the one before it constrains nothing the
previous mark had not already settled, so nobody lays one there. Marks are laid
to make a boat weave; two in a row of the same hand would make it drift.

That is a statement about buoyage, not about a course. **This file names no
course, no waypoint, and no task**, and it cannot: everything it reasons from is
read out of the world model at run time - the lateral rule the boat applied to a
red or green mark it saw, or a cardinal the camera did commit to. If the boat has
established nothing, the prior says nothing.

Three rules it is held to
-------------------------
**Evidence wins, always.** A committed camera vote is never overridden. Where the
prior disagrees with one, the disagreement is reported to the operator and the
camera is obeyed - a disagreement is a fact worth putting on the panel (it means
either the detector or the pattern is wrong, and both are worth knowing at that
moment), not a conflict to resolve in favour of the cleverer code.

**It only speaks when the geometry lets it.** The prior can say "pass this one to
port"; turning that into a compass direction needs the safe bearing to have a
real sideways component relative to the leg. On a leg running north, east and
west are unambiguous and north and south say nothing at all -
`ALTERNATION_MIN_SIN` is where that line is drawn, and a cardinal within 30
degrees of the leg's axis is declined rather than guessed at.

**It says so, loudly.** Every route it influences carries a sentence naming the
mark it reasoned from. NJORD §11.4 scores the boat explaining itself, and a boat
that quietly guesses which side of a mark to pass is exactly the boat whose
telemetry cannot be trusted at the moment it matters.

Sign convention
---------------
`side` is the side of the *boat's own line* a mark must sit on: `+1` for
starboard, `-1` for port. It is the same convention and the same sign as
`buoys.Buoys._lateral`'s `required` table, on purpose - a second convention here
would be a sign error waiting for a competition morning.
"""

from __future__ import annotations

import math

from .. import geo
from ..obsticales import (
    BUOY_TYPES,
    CARDINAL_SAFE_BEARING,
    ObstacleType,
)


def lateral_side(kind, outbound):
    """Which side of the line a red or green mark must sit on, or None.

    IALA region A: sailing with the buoyage, red is kept to port - i.e. the mark
    sits at negative cross-track. Against it, both swap. `outbound` is
    `Buoys._with_the_buoyage(ctx)`, so the two files cannot disagree about which
    way the channel runs.
    """
    if kind == ObstacleType.RED:
        return -1.0 if outbound else +1.0
    if kind == ObstacleType.GREEN:
        return +1.0 if outbound else -1.0
    return None


def cardinal_side(kind, leg_bearing, min_sin=0.0):
    """`(side, margin)` for a resolved cardinal on a leg, or `(None, margin)`.

    A cardinal says which side of *itself* is safe water: a north cardinal is
    passed to its north. So if the safe bearing points to starboard of the leg,
    the boat goes to starboard and the mark is therefore to **port** of the
    boat's line - hence the negation, which is the sign this whole file turns on.

    `margin` is `|sin|` of the angle between the safe bearing and the leg: 1 when
    the safe side is square across the leg and 0 when it points straight up or
    down it. Zero is not "no preference", it is "this mark cannot be described as
    a side of this leg at all", and it is why `min_sin` exists.
    """
    safe = CARDINAL_SAFE_BEARING.get(kind)
    if safe is None:
        return None, 0.0
    across = math.sin(math.radians(safe - leg_bearing))
    if abs(across) < min_sin:
        return None, abs(across)
    return (-1.0 if across > 0.0 else +1.0), abs(across)


def known_side(track, leg_bearing, outbound, min_sin=0.0):
    """The side this track is already known to require, or None.

    Only the two kinds of knowledge the boat came by honestly: a lateral mark's
    colour, which the lidar measured, and a cardinal the camera committed to.
    An unresolved `CARDINAL` has no side - that is the whole problem - and
    anything else is not a mark that constrains a side at all.
    """
    if track.kind in BUOY_TYPES:
        return lateral_side(track.kind, outbound)
    if track.kind in CARDINAL_SAFE_BEARING:
        side, _margin = cardinal_side(track.kind, leg_bearing, min_sin)
        return side
    return None


def best_cardinal(side, leg_bearing, config):
    """`(kind, margin)`: the cardinal that best puts a mark on `side`, or None.

    Two of the four always qualify roughly and one of those two squarely; the
    squarest is taken, and if even that is inside `ALTERNATION_MIN_SIN` the
    answer is None. On a leg running due north that picks east or west and
    refuses north and south, which is the correct refusal rather than a
    limitation - on a north-running leg a north cardinal genuinely does not tell
    you which way to go round it.
    """
    if side is None:
        return None, 0.0
    best = None
    for kind in CARDINAL_SAFE_BEARING:
        candidate, margin = cardinal_side(
            kind, leg_bearing, config.ALTERNATION_MIN_SIN
        )
        if candidate is None or candidate != side:
            continue
        if best is None or margin > best[1]:
            best = (kind, margin)
    return best if best is not None else (None, 0.0)


def expected_side(ctx, track, outbound):
    """`(side, why)` for `track` from the marks laid before it. Side may be None.

    "Before it" is along the leg, not in time and not by distance from the boat:
    the mark that constrains this one is the previous mark *in the run*, which
    the boat may well have already passed. So every mark with a known side is
    projected onto the leg, and the nearest one behind this one wins.

    `why` is always a sentence, including when the answer is None, because "the
    prior is on and it declined to say anything" is a thing the operator has to
    be able to read off the panel - otherwise a switched-on prior that never
    fires looks exactly like a broken one.
    """
    if ctx.leg is None or ctx.boat is None:
        return None, "no leg to reason along"

    leg_bearing = geo.bearing_to(ctx.leg[0], ctx.leg[1])
    _t, target_along, _cross = geo.project_onto_leg(
        track.pos, ctx.leg[0], ctx.leg[1]
    )

    previous = None
    for other in ctx.world.marks():
        if other.id == track.id:
            continue
        side = known_side(
            other, leg_bearing, outbound, ctx.config.ALTERNATION_MIN_SIN
        )
        if side is None:
            continue
        _t2, along, _c2 = geo.project_onto_leg(other.pos, ctx.leg[0], ctx.leg[1])
        gap = target_along - along
        # Behind this mark along the run, and close enough to be the one before
        # it rather than a mark on another part of the course that happens to
        # lie the right way.
        if gap <= 0.0 or gap > ctx.config.ALTERNATION_MAX_GAP_M:
            continue
        if previous is None or along > previous[0]:
            previous = (along, other, side)

    if previous is None:
        return None, (
            "nothing with a settled side lies ahead of it on this leg, so the "
            "alternating pattern says nothing here"
        )

    _along, other, side = previous
    hand = "port" if side < 0 else "starboard"
    wanted = -side
    return wanted, (
        f"{other.kind.name.lower()} #{other.id} {_gap_text(target_along, _along)} "
        f"back had to be to {hand}, so the next mark in the run is most likely "
        f"the other hand"
    )


def _gap_text(target_along, along):
    return f"{max(0.0, target_along - along):.0f} m"


def resolve(ctx, track, outbound):
    """What the prior thinks `track` is. `(kind or None, note)`.

    The one entry point `behaviours/buoys.py` calls. Returns a resolved cardinal
    type - NORTH/SOUTH/EAST/WEST - which the caller turns into a safe bearing the
    same way it would a committed one, and a sentence explaining where it came
    from. It never touches `track.cardinal`: the camera's poll stays the camera's
    poll, so a prior that guessed wrong on one leg cannot leave a committed vote
    behind to be wrong again on the next.
    """
    if not ctx.alternation:
        return None, ""
    side, why = expected_side(ctx, track, outbound)
    if side is None:
        return None, why
    leg_bearing = geo.bearing_to(ctx.leg[0], ctx.leg[1])
    kind, margin = best_cardinal(side, leg_bearing, ctx.config)
    if kind is None:
        return None, (
            f"{why}, but no cardinal's safe side is more than "
            f"{math.degrees(math.asin(ctx.config.ALTERNATION_MIN_SIN)):.0f} deg "
            f"off this leg's axis, so naming one would be a guess"
        )
    return kind, (
        f"assuming {kind.name.lower()} cardinal from the pattern, not the "
        f"camera: {why}"
    )


def disagreement(ctx, track, outbound):
    """A sentence if the prior contradicts a committed vote, else "".

    Not acted on. It is put in front of the operator because at that moment
    either the detector or the pattern is wrong, both are worth knowing, and the
    twenty seconds NJORD §8.2 allows are only usable by somebody who has been
    told there is something to think about.
    """
    if not ctx.alternation or track.kind not in CARDINAL_SAFE_BEARING:
        return ""
    guess, _why = resolve(ctx, track, outbound)
    if guess is None or guess == track.kind:
        return ""
    return (
        f"NOTE: the camera committed #{track.id} as {track.kind.name.lower()} "
        f"but the alternating pattern suggested {guess.name.lower()} - obeying "
        f"the camera"
    )
