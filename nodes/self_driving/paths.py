"""Where the boat is trying to go, as a line for the operator's chart.

The chart has always been able to draw three kinds of line - the amber ideal
route, a grey candidate, and the cyan committed path
(`ligmax-server/web/js/map.js`, `_drawPaths`) - and the layer toggle has said
"Planned path" since the dashboard was written. Nothing ever published the cyan
one. The operator could see the course as laid, the obstacles, and the trail of
where the boat had been, and had to infer the only thing that says what the boat
is about to do.

This module is that missing publisher.

What "the path" actually is on this boat
----------------------------------------
There is no A*, no RRT and no grid, on purpose - `behaviours/base.py` sets out
why at length. Each tick a behaviour picks a pure-pursuit aim point down the
leg and `deconflict()` pushes it sideways until the straight line to it clears
every confirmed obstacle. The output is *an aim point*, not a route: "the result
is a heading the boat can hold rather than a path it has to track".

So the honest committed line is two things joined:

    boat -> aim      what is on the wire to ArduPilot **this tick**, swerve and
                     all. The whole of the boat's near-term intent is in this
                     one segment, and it is the segment that differs from the
                     amber route - the gap between them *is* the avoidance.
    aim -> waypoints where it means to go afterwards, a few marks deep.

Drawing only the first gives a 6 m stub that reads as noise; drawing only the
second duplicates the amber route in a different colour and hides the swerve.
Together they say "I am going around this, then through there", which is the
sentence the operator is trying to read off the chart.

**It is a statement of intent, not a prediction.** The boat will re-decide the
aim point ten times a second, and the legs beyond the aim are straight lines
that the same avoidance will bend when the boat gets there. That is why the
forward waypoints stop after a handful: past that the cyan would be claiming a
precision it does not have, and the amber route already carries the rest.

Why the two layers are published together
------------------------------------------
`paths` is a list, and **frames merge dicts but not lists**
(`ligmax-server/ligmax_gui/state.py`, `_merge`) - a frame carrying `paths`
replaces the whole layer stack rather than adding to it. Two publishers each
sending their own single `path` would therefore take turns wiping each other
out, and the amber route would blink off the chart the moment autonomy started.
So `layers()` returns both at once and there is exactly one sender
(`main.Node._paths_payload`).
"""

from __future__ import annotations

import math

from . import geo
from .commander import GOTO, VELOCITY

# How many waypoints past the aim point the committed line carries.
#
# Not the whole remaining plan. The cyan line means "what I have committed to",
# and the further out it runs the less true that is - every leg beyond the next
# mark or two will be re-aimed and deconflicted before the boat reaches it. Four
# is enough to show the shape of the next manoeuvre (through the gate, round the
# corner, on to the following mark) and short enough that the cyan never becomes
# a second copy of the amber route competing with it for the operator's eye.
FORWARD_WAYPOINTS = 4

# How far ahead a body-velocity intent is drawn, seconds.
#
# Docking, parking, station keeping and the COLREG stand-on all creep on body
# velocities with no position target at all, so there is no aim point to draw -
# but "where it wants to go" is still a perfectly good question during a parking
# run, and the answer is the commanded velocity vector. Four seconds at the
# 0.12 m/s parking trim is about half a metre, which is visible on a chart zoomed
# in far enough to be watching a berth and invisible on one that is not.
VELOCITY_PROJECTION_S = 4.0

# Below this the commanded velocity is a station-keeping twitch rather than a
# direction, and an arrow drawn from it would swing about the compass while the
# boat sat still.
MIN_VELOCITY_MS = 0.05

# Two points closer together than this are one point. Mostly this catches the
# aim point on top of the waypoint it is aiming at, on the run-in to a mark:
# `lookahead_for()` pulls the aim to 80% of the remaining distance, so the last
# few metres of every leg would otherwise draw a degenerate segment and a second
# dot on top of the first.
MIN_SEGMENT_M = 0.5


def layers(state, intent, plan):
    """Both chart lines for this tick, as the `paths` list goes on the wire.

    Returns `[]` when there is nothing to draw, which is a meaningful value and
    not an error: sending an empty list is how a finished run takes its lines
    *off* the chart, since the list replaces rather than merges.
    """
    out = []
    if (reference := reference_layer(state, plan)) is not None:
        out.append(reference)
    if (planned := planned_layer(state, intent, plan)) is not None:
        out.append(planned)
    return out


def reference_layer(state, plan):
    """The plan as laid, roles and all - the chart's amber ideal-route layer.

    A thin wrapper over `Plan.reference_layer()` so that this module is the one
    place that decides what goes in `paths`. NJORD §11.4 wants the boat to show
    the course it was given next to the course it is taking; this is the first
    half of that pair.
    """
    if plan is None or state is None or not state.origin:
        return None
    layer = plan.reference_layer(state.origin)
    if not layer["points"]:
        return None
    return {**layer, "kind": "reference", "label": f"plan: {plan.name}"}


def planned_layer(state, intent, plan):
    """The line the boat has actually committed to, or None if it has not.

    None rather than an empty path when the boat is not being driven - a STOP or
    an IDLE intent means there is no committed target, and drawing a stale cyan
    line through a stopped boat is the chart telling a lie about the one thing
    it is being consulted for. `layers()` still sends the list, so the previous
    tick's line is replaced by the reference alone and the cyan disappears.
    """
    if state is None or intent is None or state.position is None:
        return None

    boat = (float(state.position[0]), float(state.position[1]))

    if intent.kind == GOTO:
        points = _goto_points(boat, intent, plan, state.origin)
        # The aim point, if it survived the de-duplication. The chart puts a
        # white ring on `target_index`, and the aim point is what it should ring:
        # it is the one point on this line that is on the wire right now.
        target_index = 1 if len(points) > 1 else None
    elif intent.kind == VELOCITY:
        points = _velocity_points(boat, intent, state)
        # Deliberately no ring. The far end of this line is four seconds of the
        # commanded velocity, not a place the boat is driving to, and ringing it
        # would present a projection as a target.
        target_index = None
    else:
        return None

    if len(points) < 2:
        return None

    layer = {
        "points": [[round(x, 2), round(y, 2)] for x, y in points],
        "kind": "planned",
        "label": _label(intent, plan),
    }
    if target_index is not None:
        layer["target_index"] = target_index
    return layer


def _goto_points(boat, intent, plan, origin):
    """`[boat, aim, *the next few waypoints]`, with duplicates dropped."""
    points = [boat]
    _extend(points, intent.target)
    if plan is not None and origin:
        for waypoint in plan.waypoints[plan.index : plan.index + FORWARD_WAYPOINTS]:
            _extend(points, waypoint.world(origin))
    return points


def _velocity_points(boat, intent, state):
    """`[boat, boat + the commanded velocity]`, in world metres.

    The velocity is body frame - `+x` forward, `+y` starboard, the same frame
    every sensor return uses - so it needs the heading to become a direction on
    the chart. Without one there is nothing honest to draw: the boat knows how
    hard it is pushing but not which way that points.
    """
    if state.heading is None:
        return []
    if math.hypot(intent.vx, intent.vy) < MIN_VELOCITY_MS:
        return []
    east, north = geo.boat_to_world(intent.vy, intent.vx, state.heading)
    reach = VELOCITY_PROJECTION_S
    return [boat, (boat[0] + east * reach, boat[1] + north * reach)]


def _extend(points, xy):
    """Append `xy` unless it is None or sits on top of the last point."""
    if xy is None:
        return
    point = (float(xy[0]), float(xy[1]))
    if geo.distance(points[-1], point) < MIN_SEGMENT_M:
        return
    points.append(point)


def _label(intent, plan):
    """A short name for the line. Currently only read by the recording.

    `_drawPaths` draws no label for a planned path - the reason a behaviour gives
    belongs in the autopilot panel, in full, rather than abbreviated onto the
    chart next to a line the operator is trying to read the *shape* of. It is set
    anyway because it costs nothing and it is what makes a trip recording
    readable afterwards, which is where these lines get looked at second.
    """
    waypoint = plan.current if plan is not None else None
    if waypoint is None:
        return "steering"
    return f"to {waypoint.name} ({waypoint.role})"
