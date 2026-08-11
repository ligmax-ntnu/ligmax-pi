#!/usr/bin/env python3
"""The parking geometry, against synthetic lidar sweeps. No hardware needed.

    python3 tests/test_parking.py            # run everything
    python3 tests/test_parking.py -v         # ...and print the envelope tables

Why this one is worth running on a laptop, when most things here are not: every
line of `perception/lines.py`, `perception/parking.py` and the phase logic in
`behaviours/parking.py` is arithmetic on a point cloud, and the two ways it can be
wrong are **a mirrored answer** and **a plausible answer in the wrong place**.
Neither of those looks like a failure on the water - they look like a boat parking
confidently somewhere else - and neither needs a boat to catch.

What is real and what is not
----------------------------
    real    the line fitter, the box finder, the offset arithmetic, the phase
            machine, and the countdown the chart draws
    fake    the lidar. `ray_cast` casts one beam per 0.9 deg (the C1's step) at a
            scene of wall segments and keeps the nearest hit, which is what a
            2-D lidar physically produces - one return per bearing, nearest
            surface only. Getting that part right matters: sampling the walls
            directly instead would put returns from hidden walls into the sweep,
            which breaks the angular ordering both modules rely on and makes
            every result meaningless.

Nothing here fakes a socket, a serial port, `pymavlink` or `python-can`. The parts
that need those need the boat - see docs/testing.md 7j.

What it does NOT prove
----------------------
That the boat fits, that the thrusters can hold it in a tide, that the aft lidar's
mounting geometry is right, or that a real dock scatters like a wall segment. The
envelope tables below are geometry, not seamanship.
"""

from __future__ import annotations

import math
import os
import random
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# BEFORE importing config, which reads these at import time - so a test run can
# never overwrite the boat's real plan file. Same reason as test_autopilot.py.
_SANDBOX = tempfile.mkdtemp(prefix="ligmax-park-")
os.environ["LIGMAX_AP_PLAN_FILE"] = os.path.join(_SANDBOX, "plan.json")
os.environ["LIGMAX_AP_RECORD_DIR"] = os.path.join(_SANDBOX, "trips")

from nodes.self_driving import config, geo  # noqa: E402
from nodes.self_driving.behaviours.base import Context  # noqa: E402
from nodes.self_driving.behaviours.parking import Parking  # noqa: E402
from nodes.self_driving.perception import lines, parking  # noqa: E402
from nodes.self_driving.plan import Plan  # noqa: E402
from nodes.self_driving.state import BoatState  # noqa: E402

VERBOSE = "-v" in sys.argv
ORIGIN = {"lat": 63.4390, "lon": 10.3990}
BEAM_STEP_DEG = 0.9      # the C1's beam step
SENSOR_NOISE_M = 0.03    # the C1's quoted accuracy, as a sigma. Pessimistic.

FAILURES = []


def check(condition, message):
    if condition:
        print(f"  ok    {message}")
    else:
        print(f"  FAIL  {message}")
        FAILURES.append(message)


def section(title):
    print(f"\n=== {title}")


# ------------------------------------------------------------------ the scene

def scene(mouth, depth, standoff, lateral=0.0, skew=0.0, bearing=0.0, gap=0.15,
          closed=False, extra=()):
    """Walls of a parking space, as seen from a boat at the origin. BOAT frame.

    Three sides of a `mouth` x `depth` rectangle with `gap` metres missing at each
    corner - which is the shape the task actually puts in the water, and the shape
    that makes each side arrive as its own run out of the line fitter.

        standoff  metres from the boat to the mouth plane, along the mouth normal
        lateral   metres the boat sits off the space's centreline (+ to port)
        skew      degrees the space is rotated about its own centre: the genuine
                  approach-angle error that the ALIGN phase exists to remove
        bearing   degrees the whole scene is rotated about the boat. Changes which
                  bearings the returns land at and nothing about what is visible,
                  so it is the test for sign errors in the frame conversions
        closed    add the fourth side, which must stop it being a parking space
        extra     more wall segments in the space's own frame - buoys, clutter
    """
    half_w, half_d = mouth / 2.0, depth / 2.0
    local = [
        ((-half_w, -half_d), (-half_w, half_d - gap)),        # one side
        ((half_w, -half_d), (half_w, half_d - gap)),          # the other side
        ((-half_w + gap, half_d), (half_w - gap, half_d)),    # the lone line
    ]
    if closed:
        local.append(((-half_w + gap, -half_d), (half_w - gap, -half_d)))

    centre = (-lateral, standoff + half_d)
    skew_rad = math.radians(skew)
    bearing_rad = math.radians(bearing)

    def place(point):
        u = point[0] * math.cos(skew_rad) - point[1] * math.sin(skew_rad)
        v = point[0] * math.sin(skew_rad) + point[1] * math.cos(skew_rad)
        x, y = centre[0] + u, centre[1] + v
        return (
            x * math.cos(bearing_rad) + y * math.sin(bearing_rad),
            -x * math.sin(bearing_rad) + y * math.cos(bearing_rad),
        )

    walls = [(place(a), place(b)) for a, b in local]
    walls.extend((place(a), place(b)) for a, b in extra)
    return walls


def buoy(centre, radius=0.2, facets=20):
    """A buoy as a closed polygon, in the space's own frame. Occludes properly."""
    out = []
    for i in range(facets):
        first, second = 2 * math.pi * i / facets, 2 * math.pi * (i + 1) / facets
        out.append((
            (centre[0] + radius * math.cos(first), centre[1] + radius * math.sin(first)),
            (centre[0] + radius * math.cos(second), centre[1] + radius * math.sin(second)),
        ))
    return out


def ray_cast(walls, step_deg=BEAM_STEP_DEG, noise=SENSOR_NOISE_M, seed=1):
    """One beam per `step_deg`, nearest hit only. The boat is at the origin.

    Returns `[[starboard, forward], ...]` - the frame `nodes/io_manager/scan.py`
    publishes and the frame both modules under test expect.
    """
    rng = random.Random(seed)
    points = []
    for step in range(int(360.0 / step_deg)):
        bearing = math.radians(step * step_deg)
        dx, dy = math.sin(bearing), math.cos(bearing)
        nearest = None
        for (ax, ay), (bx, by) in walls:
            ex, ey = bx - ax, by - ay
            denominator = dx * ey - dy * ex
            if abs(denominator) < 1e-12:
                continue
            distance = (ax * ey - ay * ex) / denominator
            along = (ax * dy - ay * dx) / denominator
            if distance <= 0.05 or not (0.0 <= along <= 1.0):
                continue
            if nearest is None or distance < nearest:
                nearest = distance
        if nearest is None:
            continue
        if noise:
            nearest += rng.gauss(0.0, noise)
        points.append([dx * nearest, dy * nearest])
    return points


def find(points, mouth, depth):
    """The box the vessel would find, with the vessel's own settings."""
    segments = lines.fit_segments(points, config=config, source="front_lidar")
    box = parking.find_box(
        segments,
        mouth_m=mouth,
        depth_m=depth,
        tolerance_m=config.PARK_BOX_TOLERANCE_M,
        angle_deg=config.PARK_BOX_ANGLE_DEG,
        span_fraction=config.PARK_BOX_SPAN_FRACTION,
        min_line_m=config.LINE_MIN_M,
        max_range_m=config.LINE_MAX_RANGE_M,
    )
    return segments, box


def look(mouth, depth, **kwargs):
    walls = kwargs.pop("walls", None)
    if walls is None:
        walls = scene(mouth, depth, **kwargs)
    return find(ray_cast(walls), mouth, depth)


# ---------------------------------------------------------------- line fitting

def test_lines():
    section("the line fitter")

    # One wall, straight ahead, 4 m long at 3 m. Beams are 5 cm apart there.
    wall = [((-2.0, 3.0), (2.0, 3.0))]
    segments = lines.fit_segments(ray_cast(wall), config=config, source="front_lidar")
    check(len(segments) == 1, f"one flat wall fits as one segment (got {len(segments)})")
    if segments:
        found = segments[0]
        check(
            abs(found.length_m - 4.0) < 0.25,
            f"...of about the right length: {found.length_m:.2f} m of 4.00",
        )
        check(
            lines.axis_diff(found.axis_deg, 90.0) < 3.0,
            f"...running athwartships: axis {found.axis_deg:.1f} deg, wanted 90",
        )
        check(found.rms_m < 0.05, f"...and straight: rms {found.rms_m * 100:.1f} cm")

    # A right angle must not fit as one line, and must not vanish either. The
    # corner is off to one side on purpose: a wall running straight away from the
    # sensor is edge-on and produces almost no returns, which is a property of the
    # lidar rather than of the fitter.
    corner = [((-2.0, 4.0), (1.0, 4.0)), ((1.0, 4.0), (1.0, 1.5))]
    segments = lines.fit_segments(ray_cast(corner), config=config)
    check(len(segments) == 2, f"a right angle splits into two (got {len(segments)})")

    # An axis is undirected: 179 and 1 degrees are two degrees apart, not 178.
    check(abs(lines.axis_diff(179.0, 1.0) - 2.0) < 1e-9, "axis_diff folds at 180")
    check(abs(lines.axis_diff(10.0, 100.0) - 90.0) < 1e-9, "...and tops out at 90")

    # The mean of two nearly-opposite axes is the axis they straddle, not the one
    # at right angles to it. This is the doubled-angle average, and getting it
    # wrong puts a berth axis 90 degrees out.
    class _Fake:
        def __init__(self, axis):
            self.axis_deg = axis
            self.length_m = 1.0

    mean = lines.mean_axis([_Fake(179.0), _Fake(1.0)])
    check(
        min(abs(mean - 0.0), abs(mean - 180.0)) < 1.0,
        f"mean_axis(179, 1) is about 0/180, not 90 (got {mean:.1f})",
    )

    # A single bad return must not cut a wall in half - the whole reason
    # `_max_deviation` asks for support from a neighbour.
    points = ray_cast(wall, noise=0.0)
    points[len(points) // 2][1] += 0.14      # one 14 cm outlier
    segments = lines.fit_segments(points, config=config)
    check(
        len(segments) == 1,
        f"one 14 cm outlier does not split a wall (got {len(segments)} segments)",
    )


# ----------------------------------------------------------- finding the space

def test_finding():
    section("finding the space: three lines, corners open")

    for name, mouth, depth in (("normal 2x2", 2.0, 2.0), ("parallel 4x2", 4.0, 2.0)):
        segments, box = look(mouth, depth, standoff=2.0)
        check(box is not None, f"{name}: found from 2 m out ({len(segments)} lines)")
        if box is None:
            continue
        # Truth: the space's centre sits `standoff + depth/2` ahead, and the way
        # in points dead ahead.
        check(
            math.hypot(box.centre[0], box.centre[1] - (2.0 + depth / 2.0)) < 0.25,
            f"{name}: the dot is in the middle of it "
            f"({box.centre[0]:+.2f}, {box.centre[1]:+.2f}), wanted "
            f"(0.00, {2.0 + depth / 2.0:+.2f})",
        )
        check(
            min(box.into_deg, 360.0 - box.into_deg) < 5.0,
            f"{name}: the way in points ahead ({box.into_deg:.0f} deg)",
        )
        check(
            abs(box.mouth_m - mouth) < 0.2,
            f"{name}: measured the mouth as {box.mouth_m:.2f} m of {mouth:.2f}",
        )

    # The frame conversion, which is where a sign error would live. Rotating the
    # whole scene about the boat must rotate the answer with it and change nothing
    # else - so the same space found on the port bow is the same space.
    for bearing in (-135.0, -50.0, 40.0, 175.0):
        _segments, box = look(2.0, 2.0, standoff=2.0, bearing=bearing)
        if box is None:
            check(False, f"a space {bearing:+.0f} deg off the bow is still found")
            continue
        wanted = bearing % 360.0
        error = abs(((box.into_deg - wanted + 180.0) % 360.0) - 180.0)
        check(
            error < 6.0,
            f"a space {bearing:+.0f} deg off the bow reads its way in as "
            f"{box.into_deg:.0f} deg, wanted {wanted:.0f}",
        )
        check(
            abs(math.hypot(*box.centre) - 3.0) < 0.25,
            f"...at the right range ({math.hypot(*box.centre):.2f} m of 3.00)",
        )

    # Inside it, which is where the lidar sees the least and it matters the most.
    _segments, box = look(2.0, 2.0, standoff=-1.0)
    check(box is not None, "found from inside the space")
    if box is not None:
        check(
            math.hypot(*box.centre) < 0.3,
            f"...and says the boat is in the middle of it "
            f"({box.centre[0]:+.2f}, {box.centre[1]:+.2f})",
        )


def test_refusals():
    section("what must not be a parking space")

    _s, box = look(2.0, 2.0, standoff=2.0, walls=scene(4.0, 2.0, 2.0))
    check(box is None, "a 4 m mouth asked for a 2 m one is refused")

    _s, box = look(4.0, 2.0, standoff=2.0, walls=scene(2.0, 2.0, 2.0))
    check(box is None, "a 2 m mouth asked for a 4 m one is refused")

    _s, box = look(2.0, 2.0, standoff=2.0, walls=scene(3.0, 3.0, 2.0))
    check(box is None, "a 3x3 space asked for 2x2 is refused")

    _s, box = look(2.0, 2.0, standoff=2.0, closed=True)
    check(box is None, "a closed box has no way in and is refused")

    two_walls = scene(2.0, 2.0, 2.0)[:2]
    _s, box = look(2.0, 2.0, walls=two_walls)
    check(box is None, "two walls with nothing across them are not a space")

    _s, box = look(2.0, 2.0, walls=[])
    check(box is None, "empty water is not a space")

    # One long wall, of the kind a pontoon is.
    _s, box = look(2.0, 2.0, walls=[((-6.0, 4.0), (6.0, 4.0))])
    check(box is None, "a single long face is not a space")


def test_buoys_are_ignored():
    section("buoys change nothing (the whole point of parking mode)")

    # Two buoys either side of the mouth, where the course would put them.
    clutter = buoy((-1.4, -2.0)) + buoy((1.5, -2.4))
    _s, with_buoys = look(2.0, 2.0, standoff=3.0, extra=clutter)
    _s, without = look(2.0, 2.0, standoff=3.0)
    check(
        (with_buoys is not None) and (without is not None),
        "a space with buoys outside the mouth is found exactly as without them",
    )
    if with_buoys and without:
        moved = math.hypot(
            with_buoys.centre[0] - without.centre[0],
            with_buoys.centre[1] - without.centre[1],
        )
        check(moved < 0.05, f"...and the dot does not move ({moved * 100:.0f} cm)")

    # A buoy *inside* the space. It must not stop the boat parking - it may
    # occlude enough of a wall to delay the find, which is a different thing and
    # is what closing in fixes.
    inside = buoy((0.35, 0.2))
    found_at = [
        standoff
        for standoff in (3.0, 2.0, 1.0, 0.0)
        if look(2.0, 2.0, standoff=standoff, extra=inside)[1] is not None
    ]
    check(
        bool(found_at),
        f"a buoy sitting in the space does not veto it (found at {found_at} m out)",
    )


def test_envelope():
    """How far off the centreline the space can still be seen into.

    Not a pass/fail so much as the number to lay the parking waypoint by, which is
    why it prints. It is a property of a 2 m box and not of this code: from more
    than about a metre off the axis, one side wall hides the whole interior, and no
    amount of fitting recovers a wall that produced no returns.
    """
    section("the acquisition envelope (geometry, not tuning)")

    reach = {}
    for mouth, depth in ((2.0, 2.0), (4.0, 2.0)):
        rows = []
        for standoff in (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0):
            row = []
            for lateral in (0.0, 0.3, 0.6, 0.9, 1.2):
                _s, box = look(mouth, depth, standoff=standoff, lateral=lateral)
                row.append("yes" if box else " . ")
            rows.append((standoff, row))
        reach[(mouth, depth)] = rows
        if VERBOSE:
            print(f"\n  {mouth:.0f} x {depth:.0f} m space, "
                  f"lateral offset 0.0 0.3 0.6 0.9 1.2 m")
            for standoff, row in rows:
                print(f"    {standoff:.1f} m out   " + "  ".join(row))

    # The one thing worth asserting: the standoff ALIGN actually uses works on the
    # centreline for both types. If this fails, `PARK_STANDOFF_M` is beyond what
    # the sensor can see into and the boat will square up on a space it then loses.
    for mouth, depth in ((2.0, 2.0), (4.0, 2.0)):
        _s, box = look(mouth, depth, standoff=config.PARK_STANDOFF_M)
        check(
            box is not None,
            f"{mouth:.0f}x{depth:.0f}: still visible from PARK_STANDOFF_M "
            f"({config.PARK_STANDOFF_M:.1f} m), which is where ALIGN sits",
        )

    for skew in (-10.0, 0.0, 10.0):
        _s, box = look(2.0, 2.0, standoff=2.0, skew=skew)
        check(box is not None, f"2x2 found with the space skewed {skew:+.0f} deg")


def test_offset():
    section("the static depth offset, per parking type")

    _s, box = look(2.0, 2.0, standoff=2.0)
    if box is None:
        check(False, "the offset tests need a space")
        return

    middle = box.point_at_depth(0.0)
    check(
        abs(box.depth_of(middle) - box.depth_m / 2.0) < 0.02,
        f"a zero offset sits half the depth off the lone line "
        f"({box.depth_of(middle):.2f} m of {box.depth_m / 2.0:.2f})",
    )

    deeper = box.point_at_depth(0.5)
    check(
        box.depth_of(deeper) < box.depth_of(middle) - 0.4,
        f"a positive offset moves the dot TOWARDS the lone line "
        f"({box.depth_of(deeper):.2f} m from it, was {box.depth_of(middle):.2f})",
    )
    check(
        math.hypot(deeper[0] - middle[0], deeper[1] - middle[1]) > 0.45,
        "...by about the offset",
    )

    shallower = box.point_at_depth(-0.5)
    check(
        box.depth_of(shallower) > box.depth_of(middle) + 0.4,
        "a negative offset moves it towards the mouth",
    )

    # The two types are configured separately, which is the requirement.
    normal = Parking(config, parallel=False)
    alongside = Parking(config, parallel=True)
    check(
        normal._mouth(_ctx_for(normal, [])) != alongside._mouth(_ctx_for(alongside, [])),
        "the two parking types look for differently shaped spaces",
    )
    check(
        hasattr(config, "PARK_DEPTH_OFFSET_M")
        and hasattr(config, "PARK_PARALLEL_DEPTH_OFFSET_M"),
        "...and carry one depth offset each",
    )


# ------------------------------------------------------------- the behaviour

def _ctx_for(behaviour, sweep_points, position=(0.0, 0.0), heading=0.0, now=0.0,
             waypoint_xy=(0.0, 6.0), hold_s=None, offset_m=None, next_xy=None,
             ceiling=None, probe_deg=None):
    """A `Context` with no world model at all.

    `world=None` is deliberate and is itself the test: the parking behaviours are
    required to ignore buoys, obstacles and everything else the tracker believes,
    and the cheapest proof of that is that they run to completion with nothing to
    ask. If a future edit reaches for `ctx.world`, every test below raises
    AttributeError on None and says so.
    """
    frame = {
        "t": now,
        "origin": ORIGIN,
        "boat": {"position": [position[0], position[1]], "heading_deg": heading},
        "mode": "GUIDED",
        "armed": True,
        "motion": {"sog": 0.0},
    }
    state = BoatState(frame, received_at=now)
    lat, lon = geo.to_global(waypoint_xy[0], waypoint_xy[1], ORIGIN)
    entry = {"name": "park", "lat": lat, "lon": lon, "role": behaviour.name}
    if hold_s is not None:
        entry["hold_s"] = hold_s
    if offset_m is not None:
        entry["park_offset_m"] = offset_m
    if probe_deg is not None:
        entry["park_probe_deg"] = probe_deg
    entries = [entry]
    if next_xy is not None:
        # A waypoint after this one, which is what the alongside park breaks the
        # tie with: it lies down the space pointing the way it is about to leave.
        nlat, nlon = geo.to_global(next_xy[0], next_xy[1], ORIGIN)
        entries.append({"name": "next", "lat": nlat, "lon": nlon, "role": "transit"})
    plan = Plan.parse({"name": "test", "waypoints": entries})
    return Context(
        state=state,
        world=None,
        plan=plan,
        config=config,
        now=now,
        waypoint=plan.current,
        leg=((position[0], position[1]), waypoint_xy),
        task="dock",
        clusters=(),
        sweeps=[{"source": "front_lidar", "points": sweep_points}],
        # The operator's one speed setting. Defaulted to the boot value, so every
        # test that does not care about it behaves like an ordinary run.
        ceiling=config.SPEED_MS if ceiling is None else ceiling,
    )


# One space, fixed in the WORLD, so the boat can be moved around it between ticks
# and every sweep stays consistent with the one before. The mouth plane is at
# north 4 m, the space runs from there to north 6, centred on east 0, and it opens
# to the south - so its way in points due north (000).
MOUTH_NORTH_M = 4.0
SPACE_EAST_M = 0.0


def sweep_from(position, mouth=2.0, depth=2.0, extra=(), heading=0.0):
    """What the front lidar sees of the fixed world space from `position`.

    Works at any heading, which the alongside walk needs: that one rotates 90
    degrees on the dot, and every sweep after the turn is taken from a boat lying
    across the space.
    """
    centre = (SPACE_EAST_M, MOUTH_NORTH_M + depth / 2.0)
    starboard, forward = geo.world_to_boat(
        centre[0] - position[0], centre[1] - position[1], heading
    )
    return ray_cast(
        scene(
            mouth, depth,
            standoff=forward - depth / 2.0,
            lateral=-starboard,
            # The space rotated by the boat's own heading: from the hull's point of
            # view, turning the boat and turning the world are the same picture.
            #
            # `skew=+heading`, and the sign is not obvious: `scene` puts a space
            # skewed by k at a *relative bearing of -k* (see the bearing assertions
            # in `test_finding`, which pin exactly that). Getting it backwards
            # mirrors the whole scene, and a mirrored 4 x 2 space still fits a 4 x 2
            # space - so the tests keep passing while every heading in them is
            # wrong. That is what it did, before this comment existed.
            skew=heading,
            extra=extra,
        )
    )


def test_behaviour():
    section("the behaviour: phases, the countdown, and leaving")

    behaviour = Parking(config, parallel=False)
    middle = (SPACE_EAST_M, MOUTH_NORTH_M + 1.0)   # the world dot, offset zero

    # Nothing in view: it runs to the waypoint and says what it is looking for.
    ctx = _ctx_for(behaviour, [], position=(0.0, 0.0))
    behaviour.start(ctx)
    intent = behaviour.update(ctx)
    check(behaviour.phase == "search", "with nothing in view it searches")
    check(
        intent.kind in ("goto", "velocity", "stop"),
        f"...and commands something sane ({intent.kind})",
    )
    check(
        behaviour.status["parking"]["seen"] is False,
        "...and tells the chart there is no space yet",
    )

    # The space comes into view, 4 m ahead of a boat on its centreline.
    ctx = _ctx_for(behaviour, sweep_from((0.0, 0.0)), position=(0.0, 0.0), now=1.0)
    behaviour.update(ctx)
    check(behaviour.phase == "align", f"a space in view -> {behaviour.phase}")
    check(behaviour.box is not None, "...and the space is held in world metres")
    if behaviour.box is None:
        return

    dot = behaviour.box["target"]
    check(
        abs(dot[0] - middle[0]) < 0.3 and abs(dot[1] - middle[1]) < 0.3,
        f"the dot lands in the middle of the space ({dot[0]:+.2f}, {dot[1]:+.2f}), "
        f"wanted ({middle[0]:+.2f}, {middle[1]:+.2f})",
    )
    check(
        min(behaviour.box["into_deg"], 360.0 - behaviour.box["into_deg"]) < 5.0,
        f"...and the way in is north ({behaviour.box['into_deg']:.0f} deg)",
    )

    # At the approach point, square and stopped: that is what ALIGN is waiting for.
    approach = geo.offset_point(dot, behaviour.box["into_deg"] + 180.0,
                                config.PARK_STANDOFF_M)
    ctx = _ctx_for(behaviour, sweep_from(approach), position=approach, now=2.0)
    behaviour.update(ctx)
    check(
        behaviour.phase == "enter",
        f"squared up one standoff out -> {behaviour.phase} (wanted enter)",
    )
    intent = behaviour.update(
        _ctx_for(behaviour, sweep_from(approach), position=approach, now=2.5)
    )
    check(
        intent.kind == "velocity" and intent.vx > 0.0,
        f"...and it creeps forwards to get in (vx {getattr(intent, 'vx', None):+.2f})",
    )
    check(
        intent.vx <= config.PARK_SPEED_MS + 1e-9,
        f"...no faster than PARK_SPEED_MS ({config.PARK_SPEED_MS} m/s)",
    )

    # On the dot. It should commit and start counting.
    now = 3.0
    behaviour.update(_ctx_for(behaviour, sweep_from(dot), position=dot, now=now))
    check(behaviour.phase == "hold", f"on the dot it holds (phase {behaviour.phase})")
    required = behaviour.status.get("hold_required_s")
    check(
        abs((required or 0.0) - config.PARK_HOLD_S) < 0.01,
        f"...for {required} s, which is PARK_HOLD_S ({config.PARK_HOLD_S})",
    )

    # The countdown the chart draws next to the boat. Ticked at the 2 Hz the
    # vessel actually publishes at.
    seen = []
    for step in range(1, 40):
        now = 3.0 + step * 0.5
        behaviour.update(_ctx_for(behaviour, sweep_from(dot), position=dot, now=now))
        if behaviour.phase == "hold":
            seen.append(behaviour.status.get("hold_remaining_s"))
        else:
            break

    check(
        bool(seen) and all(b <= a for a, b in zip(seen, seen[1:])),
        f"the countdown only ever counts down ({seen[:3]} ... {seen[-3:]})",
    )
    check(
        bool(seen) and seen[0] > config.PARK_HOLD_S - 1.5,
        f"...starting at about {config.PARK_HOLD_S:.0f} s (got {seen[0] if seen else None})",
    )
    check(bool(seen) and seen[-1] <= 0.6, f"...and reaching zero ({seen[-1] if seen else None})")
    check(
        behaviour.phase == "exit",
        f"after {config.PARK_HOLD_S:.0f} s on the dot it leaves "
        f"(phase {behaviour.phase})",
    )
    check(
        behaviour.status.get("hold_remaining_s") is None,
        "...and the countdown is cleared, so the chart stops drawing a timer",
    )

    # Leaving. A bow-in park reverses out, which is a negative forward command.
    intent = behaviour.update(
        _ctx_for(behaviour, sweep_from(dot), position=dot, now=now + 0.5)
    )
    check(
        intent.kind == "velocity" and intent.vx < 0.0,
        f"a bow-in park reverses out (vx {getattr(intent, 'vx', None)})",
    )
    check(not behaviour.done, "...and is not finished until it is clear")

    # Far enough out, the waypoint is done.
    away = (dot[0], dot[1] - config.PARK_EXIT_M - 0.5)
    behaviour.update(_ctx_for(behaviour, [], position=away, now=now + 5.0))
    check(behaviour.done, "clear of the space, the waypoint is finished")


def test_hold_restarts_on_drift():
    section("the countdown does not count time spent out of the middle")

    behaviour = Parking(config, parallel=False)
    ctx = _ctx_for(behaviour, sweep_from((0.0, 0.0)), position=(0.0, 0.0))
    behaviour.start(ctx)
    behaviour.update(ctx)
    if behaviour.box is None:
        check(False, "the drift test needs a space")
        return
    dot = behaviour.box["target"]

    behaviour.phase = "hold"
    behaviour._hold_from = 0.0
    behaviour.update(_ctx_for(behaviour, sweep_from(dot), position=dot, now=5.0))
    check(
        abs(behaviour.status["hold_elapsed_s"] - 5.0) < 0.2,
        f"five seconds on the dot counts as five ({behaviour.status['hold_elapsed_s']})",
    )

    pushed = (dot[0] + config.PARK_HOLD_TOLERANCE_M + 0.3, dot[1])
    behaviour.update(_ctx_for(behaviour, sweep_from(dot), position=pushed, now=6.0))
    check(
        behaviour.status["hold_elapsed_s"] < 0.2,
        f"drifting out of the middle restarts it "
        f"({behaviour.status['hold_elapsed_s']} s)",
    )
    check(
        behaviour.status["hold_restarts"] == 1,
        f"...and says so, once ({behaviour.status['hold_restarts']})",
    )


def test_always_normal():
    section("the approach runs along the normal, and an offset backs out")

    behaviour = Parking(config, parallel=False)
    ctx = _ctx_for(behaviour, sweep_from((0.0, 0.0)), position=(0.0, 0.0))
    behaviour.start(ctx)
    behaviour.update(ctx)
    if behaviour.box is None:
        check(False, "this test needs a space")
        return
    dot = behaviour.box["target"]

    # On the centreline it advances, straight down the normal, and asks for no
    # sideways motion at all. At heading 000 into a space that opens south, the
    # space's along-axis IS the hull's forward axis.
    behaviour.phase = "enter"
    on_centre = (dot[0], dot[1] - config.PARK_STANDOFF_M)
    intent = behaviour.update(
        _ctx_for(behaviour, sweep_from(on_centre), position=on_centre, now=2.0)
    )
    check(
        intent.kind == "velocity" and intent.vx > 0.05,
        f"on the centreline it advances (vx {getattr(intent, 'vx', 0):+.3f})",
    )
    check(
        abs(intent.vy) < 0.02,
        f"...with no sideways component at all (vy {getattr(intent, 'vy', 0):+.3f})",
    )
    check(
        abs(geo.angle_diff(behaviour._desired_heading(ctx), behaviour.box["into_deg"]))
        < 1.0,
        "...holding the approach heading rather than steering at the dot",
    )

    # Half a metre off the centreline: it must NOT crab across (this hull does not
    # travel sideways) and it must NOT steer in (it would arrive crooked). The only
    # answer left is to back out and line up again.
    off_centre = (dot[0] + 0.5, dot[1] - config.PARK_STANDOFF_M)
    intent = behaviour.update(
        _ctx_for(behaviour, sweep_from(off_centre), position=off_centre, now=3.0)
    )
    check(
        behaviour.phase == "align",
        f"half a metre off the centreline aborts the entry (phase {behaviour.phase})",
    )
    check("re-approach" in intent.reason, f"...and says why: {intent.reason}")
    check(
        behaviour._reapproaches == 1, f"...counting it ({behaviour._reapproaches})"
    )

    # It must give up rather than shuttle in and out of the mouth for the rest of
    # the run. One attempt is already spent, so this takes it one past the limit.
    for attempt in range(config.PARK_MAX_REAPPROACHES):
        behaviour.phase = "enter"
        behaviour.update(
            _ctx_for(
                behaviour, sweep_from(off_centre), position=off_centre,
                now=4.0 + attempt,
            )
        )
    check(
        behaviour._reapproaches == config.PARK_MAX_REAPPROACHES + 1,
        f"it stops after {config.PARK_MAX_REAPPROACHES} attempts "
        f"({behaviour._reapproaches})",
    )
    check(
        "take over" in (behaviour.status.get("stuck") or ""),
        f"...and asks for a human: {behaviour.status.get('stuck')}",
    )


def test_sideways_is_a_trim_not_a_drive():
    section("the sideways thruster trims while travelling and holds while parked")

    behaviour = Parking(config, parallel=False)
    ctx = _ctx_for(behaviour, sweep_from((0.0, 0.0)), position=(0.0, 0.0))
    behaviour.start(ctx)
    behaviour.update(ctx)
    if behaviour.box is None:
        check(False, "this test needs a space")
        return
    dot = behaviour.box["target"]

    # A small offset - inside the centreline tolerance, so the entry continues -
    # is trimmed out, and the trim is held to PARK_TRIM_LATERAL_MS.
    behaviour.phase = "enter"
    nudged = (dot[0] + config.PARK_CENTRE_TOLERANCE_M * 0.9,
              dot[1] - config.PARK_STANDOFF_M)
    intent = behaviour.update(
        _ctx_for(behaviour, sweep_from(nudged), position=nudged, now=2.0)
    )
    check(
        behaviour.phase == "enter",
        f"a small offset does not abort the entry (phase {behaviour.phase})",
    )
    check(
        0.0 < abs(intent.vy) <= config.PARK_TRIM_LATERAL_MS + 1e-6,
        f"...it is trimmed out at no more than PARK_TRIM_LATERAL_MS "
        f"({abs(intent.vy):.3f} of {config.PARK_TRIM_LATERAL_MS})",
    )
    check(
        intent.vx > 0.05,
        f"...while the main thrusters keep it going in (vx {intent.vx:+.3f})",
    )

    # Holding is the thruster's actual job, and there it gets full authority. Put
    # the boat off the dot by more than a trim can hold and the command should
    # exceed the travel cap.
    behaviour.phase = "hold"
    behaviour._hold_from = 5.0
    # The sweep has to be the one taken from where the boat actually is, or the
    # measured space moves with the boat and the error under test is zero.
    pushed = (dot[0] + 0.35, dot[1])
    intent = behaviour.update(
        _ctx_for(behaviour, sweep_from(pushed), position=pushed, now=5.5)
    )
    check(
        abs(intent.vy) > config.PARK_TRIM_LATERAL_MS,
        f"holding the dot uses the full lateral authority "
        f"({abs(intent.vy):.3f} > the {config.PARK_TRIM_LATERAL_MS} travel trim)",
    )
    check(
        abs(intent.vy) <= 0.35 + 1e-6,
        f"...up to LATERAL_MAX_MS and no further ({abs(intent.vy):.3f})",
    )


def test_no_lateral_thruster():
    section("with no sideways thruster at all")

    import nodes.self_driving.behaviours.parking as parking_module

    was = parking_module.LATERAL_MODE
    parking_module.LATERAL_MODE = "none"
    try:
        behaviour = Parking(config, parallel=False)
        ctx = _ctx_for(behaviour, sweep_from((0.0, 0.0)), position=(0.0, 0.0))
        behaviour.start(ctx)
        behaviour.update(ctx)
        if behaviour.box is None:
            check(False, "this test needs a space")
            return
        dot = behaviour.box["target"]

        # The manoeuvre is unchanged - nothing about it needed sideways travel -
        # but the hold says the spot can only be held on one axis.
        behaviour.phase = "hold"
        behaviour._hold_from = 1.0
        behaviour.update(_ctx_for(behaviour, sweep_from(dot), position=dot, now=2.0))
        check(
            "one axis" in (behaviour.status.get("hold_warning") or ""),
            f"the hold warns that station keeping is degraded: "
            f"{behaviour.status.get('hold_warning')}",
        )
        check(
            behaviour.status.get("hold_remaining_s") is not None,
            "...and it still parks and still counts down",
        )
    finally:
        parking_module.LATERAL_MODE = was


def test_parallel_behaviour():
    section("the alongside variant: in bow-first, then 90 degrees on the dot")

    behaviour = Parking(config, parallel=True)
    check(behaviour.name == "park_parallel", "it names itself park_parallel")

    # A waypoint after this one, out to the east: the bow should end up pointing
    # that way, so the boat leaves towards it.
    following = (40.0, MOUTH_NORTH_M + 1.0)
    ctx = _ctx_for(behaviour, sweep_from((0.0, 0.0), mouth=4.0), next_xy=following)
    behaviour.start(ctx)
    behaviour.update(ctx)
    check(behaviour.box is not None, "it finds a 4 m x 2 m space")
    if behaviour.box is None:
        return
    into = behaviour.box["into_deg"]
    dot = behaviour.box["target"]

    # The APPROACH heading is the same as a bow-in park's: down the normal.
    check(
        abs(geo.angle_diff(behaviour._desired_heading(ctx), into)) < 5.0,
        f"it approaches bow-first down the normal, not sideways "
        f"({behaviour._desired_heading(ctx):.0f} vs a way in of {into:.0f})",
    )

    # The PARKING heading is 90 deg off it, on the side the next waypoint is.
    park = behaviour._park_heading(ctx)
    off_axis = abs(abs(geo.angle_diff(park, into)) - 90.0)
    check(off_axis < 5.0, f"the parking angle is 90 deg off the way in ({park:.0f})")
    check(
        abs(geo.angle_diff(park, 90.0)) < 5.0,
        f"...on the side of the waypoint it leaves for ({park:.0f}, wanted about 90)",
    )
    latched = behaviour._park_heading(
        _ctx_for(behaviour, [], heading=200.0, now=2.0, next_xy=following)
    )
    check(
        abs(geo.angle_diff(latched, park)) < 1.0,
        "...and latched, so a swinging compass cannot flip it mid-manoeuvre",
    )

    # Down the normal to the dot, still bow-first. Arriving starts the TURN.
    now = 2.0
    behaviour.phase = "enter"
    behaviour.update(
        _ctx_for(behaviour, sweep_from(dot, mouth=4.0), position=dot, now=now,
                 next_xy=following)
    )
    check(
        behaviour.phase == "turn",
        f"on the dot, bow-in, it starts the 90 deg turn (phase {behaviour.phase})",
    )
    check(
        behaviour.status.get("hold_remaining_s") is None,
        "...and the countdown has NOT started - it is not parked yet",
    )

    # Rotate the boat as the yaw command would, and watch it commit.
    heading = 0.0
    for step in range(1, 30):
        now += 0.5
        heading = min(90.0, heading + 15.0)
        behaviour.update(
            _ctx_for(
                behaviour, sweep_from(dot, mouth=4.0, heading=heading),
                position=dot, heading=heading, now=now, next_xy=following,
            )
        )
        if behaviour.phase == "hold":
            break
    check(behaviour.phase == "hold", f"once round, it holds (phase {behaviour.phase})")
    check(
        abs(geo.angle_diff(heading, park)) <= config.PARK_ALIGN_TOLERANCE_DEG,
        f"...having actually reached the parking angle ({heading:.0f} of {park:.0f})",
    )
    check(
        behaviour.status.get("hold_remaining_s") is not None,
        "...and only now does the countdown start",
    )

    # The heading gate: let it count for two seconds at the parking angle, then
    # knock it 30 deg off and watch the countdown go back to the top.
    now += 0.5
    behaviour.update(
        _ctx_for(behaviour, sweep_from(dot, mouth=4.0, heading=park), position=dot,
                 heading=park, now=now, next_xy=following)
    )
    now += 2.0
    behaviour.update(
        _ctx_for(behaviour, sweep_from(dot, mouth=4.0, heading=park), position=dot,
                 heading=park, now=now, next_xy=following)
    )
    before = behaviour.status["hold_elapsed_s"]
    check(
        before > 1.5,
        f"two seconds at the parking angle counts as two ({before} s)",
    )
    now += 1.0
    crooked = geo.wrap360(park - 30.0)
    behaviour.update(
        _ctx_for(behaviour, sweep_from(dot, mouth=4.0, heading=crooked), position=dot,
                 heading=crooked, now=now, next_xy=following)
    )
    check(
        behaviour.status["hold_elapsed_s"] < before + 1.5,
        f"30 deg off the parking angle restarts the countdown "
        f"({behaviour.status['hold_elapsed_s']} s, was {before})",
    )
    check(
        "parking angle" in (behaviour.status.get("hold_restart_why") or ""),
        f"...and says it was the angle, not the position: "
        f"{behaviour.status.get('hold_restart_why')}",
    )

    # Leaving. Lying across the way out, it must turn back to the approach angle
    # first - coming out sideways would be asking the trim thruster to move the
    # boat, and it is not for that.
    behaviour.phase = "exit"
    behaviour._hold_from = now
    now += 1.0
    behaviour.update(
        _ctx_for(behaviour, sweep_from(dot, mouth=4.0, heading=park), position=dot,
                 heading=park, now=now, next_xy=following)
    )
    check(
        behaviour.phase == "turn",
        f"an alongside park turns back before leaving (phase {behaviour.phase})",
    )
    check(
        abs(geo.angle_diff(behaviour._turn_to, into)) < 1.0,
        f"...back onto the approach angle ({behaviour._turn_to:.0f} of {into:.0f})",
    )

    heading = park
    for step in range(30):
        now += 0.5
        heading = geo.wrap360(heading - 15.0) if step < 6 else into
        behaviour.update(
            _ctx_for(
                behaviour, sweep_from(dot, mouth=4.0, heading=heading), position=dot,
                heading=heading, now=now, next_xy=following,
            )
        )
        if behaviour.phase == "exit":
            break
    check(behaviour.phase == "exit", f"once round it leaves (phase {behaviour.phase})")

    intent = behaviour.update(
        _ctx_for(behaviour, sweep_from(dot, mouth=4.0, heading=into), position=dot,
                 heading=into, now=now + 0.5, next_xy=following)
    )
    check(
        intent.kind == "velocity" and intent.vx < -0.05,
        f"...astern along the normal on the main thrusters "
        f"(vx {getattr(intent, 'vx', 0):+.3f})",
    )
    check(
        abs(intent.vy) < 0.05,
        f"...and not sideways (vy {getattr(intent, 'vy', 0):+.3f})",
    )


def test_offset_clamp():
    section("an offset bigger than the space")

    behaviour = Parking(config, parallel=False)
    ctx = _ctx_for(behaviour, sweep_from((0.0, 0.0)), offset_m=2.5)
    behaviour.start(ctx)
    behaviour.update(ctx)
    if behaviour.box is None:
        check(False, "the clamp test needs a space")
        return
    check(
        behaviour.box["offset_clamped"] is True,
        "a 2.5 m offset into a 2 m space is clamped, not obeyed",
    )
    check(
        abs(behaviour.box["offset_m"]) <= behaviour.box["depth_m"] / 2.0,
        f"...to inside the space (offset {behaviour.box['offset_m']:.2f} m, "
        f"depth {behaviour.box['depth_m']:.2f} m)",
    )
    check(
        behaviour.box["dot_depth_m"] >= config.PARK_OFFSET_MARGIN_M - 0.01,
        f"...leaving the margin off the lone line "
        f"({behaviour.box['dot_depth_m']:.2f} m)",
    )


def test_probe():
    """Nothing in view at the waypoint: creep in towards the docks and look.

    The case this exists for is not a fault. The waypoint before a park is laid
    *just outside* the docks and the boat has one forward-looking lidar, so a berth
    a few metres further in produces no returns at all from there. Standing on the
    waypoint declaring itself stuck would spend NJORD §8.2's twenty seconds on
    something a metre of travel fixes.
    """
    section("the probe: nothing in view from the waypoint")

    behaviour = Parking(config, parallel=False)
    waypoint = (0.0, 6.0)
    ctx = _ctx_for(behaviour, [], position=waypoint, waypoint_xy=waypoint)
    behaviour.start(ctx)
    behaviour.update(ctx)
    check(behaviour.phase == "search", "at the waypoint with nothing in view: search")
    check(
        behaviour._probe_from is None,
        "...and it does not set off immediately - it looks from there first",
    )

    # Past the search timeout, still nothing. Now it should move.
    late = config.PARK_SEARCH_TIMEOUT_S + 1.0
    intent = behaviour.update(
        _ctx_for(behaviour, [], position=waypoint, waypoint_xy=waypoint, now=late)
    )
    check(
        behaviour._probe_from is not None and behaviour.phase == "probe",
        f"after {config.PARK_SEARCH_TIMEOUT_S:.0f} s it probes instead of giving up "
        f"(phase {behaviour.phase})",
    )
    check(
        intent.kind == "velocity" and intent.vx > 0.0,
        f"...moving ahead ({intent.kind}, vx {getattr(intent, 'vx', 0):+.2f})",
    )
    check(
        abs(behaviour.status["probe_bearing_deg"] - config.PARK_PROBE_BEARING_DEG)
        < 0.01,
        f"...on {config.PARK_PROBE_BEARING_DEG:.0f} deg - east and a little south, "
        f"in towards land at Havet ({behaviour.status['probe_bearing_deg']})",
    )
    check(
        intent.vx <= config.PARK_PROBE_SPEED_MS + 1e-9,
        f"...at the docking creep and no faster ({intent.vx:.2f} of "
        f"{config.PARK_PROBE_SPEED_MS})",
    )
    check(
        "120" in intent.reason or "probing" in intent.reason,
        f"...and says what it is doing: {intent.reason}",
    )

    from nodes.self_driving import pilot as pilot_module
    import inspect
    check(
        '"probe"' in inspect.getsource(pilot_module.Pilot._watch_progress),
        "the pilot exempts the probe from its no-progress badge - a probe drives "
        "away from the waypoint on purpose",
    )

    # A waypoint's own bearing beats the config's, because it is a fact about a
    # berth rather than about the boat.
    other = Parking(config, parallel=False)
    ctx = _ctx_for(
        other, [], position=waypoint, waypoint_xy=waypoint, probe_deg=240.0
    )
    other.start(ctx)
    other.update(ctx)
    other.update(
        _ctx_for(
            other, [], position=waypoint, waypoint_xy=waypoint, probe_deg=240.0,
            now=late,
        )
    )
    check(
        abs(other.status["probe_bearing_deg"] - 240.0) < 0.01,
        f"a waypoint's park_probe_deg overrides the default "
        f"({other.status['probe_bearing_deg']})",
    )

    # Out of probe: it stops and asks for a human rather than crossing the basin.
    far = geo.offset_point(waypoint, config.PARK_PROBE_BEARING_DEG,
                           config.PARK_PROBE_M + 0.5)
    intent = behaviour.update(
        _ctx_for(behaviour, [], position=far, waypoint_xy=waypoint, now=late + 60.0)
    )
    check(
        intent.kind == "stop",
        f"{config.PARK_PROBE_M:.0f} m along with nothing found, it stops "
        f"({intent.kind})",
    )
    check(
        "take over" in (behaviour.status.get("stuck") or ""),
        f"...and asks for a human: {behaviour.status.get('stuck')}",
    )

    # And the probe ends the moment a space appears, without needing the timeout
    # again: the boat is somewhere new, and somewhere new is the whole point.
    seen = Parking(config, parallel=False)
    here = (0.0, 0.0)
    ctx = _ctx_for(seen, [], position=here, waypoint_xy=here)
    seen.start(ctx)
    seen.update(ctx)
    seen.update(_ctx_for(seen, [], position=here, waypoint_xy=here, now=late))
    check(seen._probe_from is not None, "probing, with nothing in view")
    seen.update(_ctx_for(seen, sweep_from(here), position=here, waypoint_xy=here,
                         now=late + 1.0))
    check(
        seen.phase == "align" and seen.box is not None,
        f"a space appearing mid-probe ends it and squares up (phase {seen.phase})",
    )


def test_blind_alongside_hold():
    """The alongside hold, with the closed end out of view. One lidar, forwards.

    This is the manoeuvre the broken aft lidar actually costs. The boat comes down
    the normal bow-first, reaches the dot, turns 90 degrees - and from then until it
    turns back, the lone line it measured every depth from is abeam of a hull that
    can only see ahead. So it holds the *remembered* middle of a remembered space,
    and the tests here are that it does exactly that: keeps parking, says it is
    working from memory, and refuses to be handed a different space.
    """
    section("the alongside hold is flown from memory (no aft lidar)")

    behaviour = Parking(config, parallel=True)
    following = (40.0, MOUTH_NORTH_M + 1.0)
    ctx = _ctx_for(behaviour, sweep_from((0.0, 0.0), mouth=4.0), next_xy=following)
    behaviour.start(ctx)
    behaviour.update(ctx)
    if behaviour.box is None:
        check(False, "this test needs a space")
        return
    dot = behaviour.box["target"]
    into = behaviour.box["into_deg"]
    park = behaviour._park_heading(ctx)
    remembered = dict(behaviour.box)

    check(
        behaviour.status["parking"]["source"] == "measured",
        "outside the space, the figures are measured",
    )

    # On the dot and turned: from here on it is memory.
    behaviour.phase = "hold"
    behaviour._latched_at = 10.0
    behaviour._hold_from = 10.0
    now = 11.0
    intent = behaviour.update(
        _ctx_for(behaviour, [], position=dot, heading=park, now=now,
                 next_xy=following)
    )
    block = behaviour.status["parking"]
    check(
        behaviour.box is not None and block["seen"] is True,
        "with the space out of view it still knows where the space is",
    )
    check(
        block["source"] == "remembered" and block["blind"] is True,
        f"...and says the figures are remembered rather than measured "
        f"({block['source']}, blind={block['blind']})",
    )
    check(
        abs(block["mouth_m"] - remembered["mouth_m"]) < 0.02,
        f"...holding the mouth width it measured ({block['mouth_m']:.2f} m)",
    )
    check(
        "remembered middle" in (behaviour.status.get("hold_blind") or ""),
        f"...and the panel says so in words: {behaviour.status.get('hold_blind')}",
    )
    check(
        intent.kind in ("velocity", "stop"),
        f"...while still holding the dot ({intent.kind})",
    )

    # The countdown runs on the memory, which is the requirement: ten continuous
    # seconds in the middle, whether or not the middle is visible.
    for step in range(1, 30):
        now = 11.0 + step * 0.5
        behaviour.update(
            _ctx_for(behaviour, [], position=dot, heading=park, now=now,
                     next_xy=following)
        )
        if behaviour.phase != "hold":
            break
    check(
        behaviour.phase in ("exit", "turn") and behaviour._turned_back,
        f"ten blind seconds on the remembered dot finishes the hold and it turns "
        f"back to leave (phase {behaviour.phase})",
    )

    # A *different* space, offered while the boat is lying across this one, must
    # not be believed. This is the failure the aft lidar's death introduced: from
    # that pose a three-line fit can be a plausible box facing anywhere.
    latched = Parking(config, parallel=True)
    ctx = _ctx_for(latched, sweep_from((0.0, 0.0), mouth=4.0), next_xy=following)
    latched.start(ctx)
    latched.update(ctx)
    latched.phase = "hold"
    latched._latched_at = 5.0
    latched._hold_from = 5.0
    before = dict(latched.box)
    elsewhere = ray_cast(scene(4.0, 2.0, standoff=1.0, bearing=90.0))
    latched.update(
        _ctx_for(latched, elsewhere, position=dot, heading=park, now=6.0,
                 next_xy=following)
    )
    check(
        latched.status.get("box_ignored") is not None,
        f"a fit that disagrees with the memory is refused: "
        f"{latched.status.get('box_ignored')}",
    )
    check(
        geo.distance(latched.box["target"], before["target"]) < 1e-9
        and abs(latched.box["into_deg"] - before["into_deg"]) < 1e-9,
        "...and the remembered space is untouched by it",
    )

    # The three tests, directly, so it is clear which one catches what.
    same = dict(before)
    ok, _why = latched._agrees(ctx, same)
    check(ok, "the same box agrees with itself")
    ok, why = latched._agrees(
        ctx,
        dict(before, target=geo.offset_point(before["target"], 0.0,
                                             config.PARK_LATCH_TOLERANCE_M + 0.2)),
    )
    check(not ok and "jumped" in why, f"a dot that jumped is refused: {why}")
    ok, why = latched._agrees(
        ctx, dict(before, mouth_m=before["mouth_m"] + config.PARK_BOX_TOLERANCE_M + 0.2)
    )
    check(not ok and "mouth" in why, f"a mouth that changed width is refused: {why}")
    ok, why = latched._agrees(ctx, dict(before, into_deg=(before["into_deg"] + 90.0)))
    check(
        not ok and "way in" in why,
        f"a way in 90 deg from the remembered one is refused: {why}",
    )

    # Bow-in parking keeps measuring throughout, because it faces the closed end.
    bow = Parking(config, parallel=False)
    ctx = _ctx_for(bow, sweep_from((0.0, 0.0)))
    bow.start(ctx)
    bow.update(ctx)
    bow.phase = "hold"
    bow._latched_at = 1.0
    bow._hold_from = 1.0
    bow.update(_ctx_for(bow, sweep_from(bow.box["target"]),
                        position=bow.box["target"], now=2.0))
    check(
        bow.status["parking"]["blind"] is False,
        "a bow-in park is never blind - it sits looking at the closed end",
    )
    check(
        abs(geo.angle_diff(into, bow.box["into_deg"])) < 5.0,
        "...and keeps measuring the same space it came in on",
    )


def test_speed_setting_is_obeyed():
    """0.1 m/s means 0.1 m/s, including inside the berth.

    Every speed in a parking run used to come from `config` alone, so an operator
    who set the dashboard's speed to 0.1 m/s for a first attempt got a 0.30 m/s
    entry and a 0.35 m/s hold: the number on the panel was a decoration in the one
    manoeuvre where it matters most.
    """
    section("the operator's speed setting is obeyed by the whole manoeuvre")

    slow = 0.1
    behaviour = Parking(config, parallel=False)
    ctx = _ctx_for(behaviour, sweep_from((0.0, 0.0)), ceiling=slow)
    behaviour.start(ctx)
    behaviour.update(ctx)
    if behaviour.box is None:
        check(False, "this test needs a space")
        return
    dot = behaviour.box["target"]
    check(
        behaviour.status["parking"]["speed_cap_ms"] == slow,
        f"the panel carries the cap in force "
        f"({behaviour.status['parking']['speed_cap_ms']} m/s)",
    )

    # Entering.
    behaviour.phase = "enter"
    start = (dot[0], dot[1] - config.PARK_STANDOFF_M)
    intent = behaviour.update(
        _ctx_for(behaviour, sweep_from(start), position=start, now=2.0, ceiling=slow)
    )
    check(
        0.0 < intent.vx <= slow + 1e-9,
        f"the entry creeps at {intent.vx:.3f} m/s, not PARK_SPEED_MS "
        f"({config.PARK_SPEED_MS})",
    )

    # Holding, where the lateral thruster has full authority - which must also be
    # held under the setting, or a "0.1 m/s" hold crabs at 0.35.
    behaviour.phase = "hold"
    behaviour._hold_from = 5.0
    pushed = (dot[0] + 0.35, dot[1])
    intent = behaviour.update(
        _ctx_for(behaviour, sweep_from(pushed), position=pushed, now=5.5,
                 ceiling=slow)
    )
    check(
        abs(intent.vy) <= slow + 1e-9,
        f"the hold's sideways command is {abs(intent.vy):.3f} m/s, under the "
        f"setting rather than at LATERAL_MAX_MS",
    )
    check(
        math.hypot(intent.vx, intent.vy) <= slow + 1e-9,
        f"...and so is the resultant ({math.hypot(intent.vx, intent.vy):.3f} m/s)",
    )

    # Leaving.
    behaviour.phase = "exit"
    intent = behaviour.update(
        _ctx_for(behaviour, sweep_from(dot), position=dot, now=20.0, ceiling=slow)
    )
    check(
        abs(intent.vx) <= slow + 1e-9,
        f"reversing out is {abs(intent.vx):.3f} m/s, not PARK_REVERSE_SPEED_MS "
        f"({config.PARK_REVERSE_SPEED_MS})",
    )

    # The run to the waypoint, which is a position target rather than a velocity.
    away = Parking(config, parallel=False)
    ctx = _ctx_for(away, [], position=(0.0, -20.0), ceiling=slow)
    away.start(ctx)
    intent = away.update(ctx)
    check(
        intent.kind == "goto" and intent.speed <= slow + 1e-9,
        f"even the run out to the parking waypoint obeys it "
        f"({getattr(intent, 'speed', None)} m/s)",
    )

    # ...and a high setting cannot make a berth approach brisk. The setting is a
    # ceiling, never a floor.
    brisk = Parking(config, parallel=False)
    ctx = _ctx_for(brisk, sweep_from((0.0, 0.0)), ceiling=config.SPEED_LIMIT_MS)
    brisk.start(ctx)
    brisk.update(ctx)
    brisk.phase = "enter"
    start = (brisk.box["target"][0], brisk.box["target"][1] - config.PARK_STANDOFF_M)
    intent = brisk.update(
        _ctx_for(brisk, sweep_from(start), position=start, now=2.0,
                 ceiling=config.SPEED_LIMIT_MS)
    )
    check(
        intent.vx <= config.PARK_SPEED_MS + 1e-9,
        f"at the 5 kn setting the entry is still the docking creep "
        f"({intent.vx:.3f} of {config.PARK_SPEED_MS})",
    )


def test_front_lidar_only():
    section("one lidar, and it looks forward")

    behaviour = Parking(config, parallel=False)
    ctx = _ctx_for(behaviour, sweep_from((0.0, 0.0)))
    check(
        behaviour._sources(ctx) == ("front_lidar",),
        f"parking fits lines to the front unit and nothing else "
        f"({behaviour._sources(ctx)})",
    )
    check(
        not hasattr(config, "PARK_USE_AFT_LIDAR"),
        "there is no aft-lidar switch left to turn on by accident",
    )

    # An aft sweep on the bus is ignored rather than fitted. The scene here is a
    # space *astern*, which is what a mirrored aft unit would offer.
    behaviour.start(ctx)
    ctx = _ctx_for(behaviour, [])
    ctx.sweeps = [
        {"source": "aft_lidar", "points": ray_cast(scene(2.0, 2.0, standoff=2.0))}
    ]
    behaviour.update(ctx)
    check(
        behaviour.box is None,
        "a parking space offered by the aft unit is not believed",
    )


# --------------------------------------------------------------- the AR tags
#
# The tag path has one thing in it that no amount of care makes obvious, and it is
# what these cover: **the sign of the way in**. A berth built from three points can
# come out rotated by 180 degrees (drive out to sea instead of in) or by twice the
# corner angle (53 degrees for a 2 m berth, i.e. into a wall), and both produce a
# confident, complete, plausible box. The lidar version has `test_always_normal`
# for the same reason; this is its counterpart.
#
# The tags themselves are synthesised at exact positions rather than rendered
# through a camera. What a real lens does to them is measured in
# `ligmax-edge/artags.py` and is not this file's question.

def tag_at(tag_id, point, cam=0, incidence=5.0, facing=None):
    """One tag as it arrives off the wire: rig frame, `[x, y, z]` metres."""
    return {
        "id": tag_id, "cam": cam,
        "pos_rig": [point[0], 0.0, point[1]],
        "range_m": math.hypot(*point), "edge_px": 60.0,
        "incidence_deg": incidence, "facing_deg": facing,
    }


def berth_tags(into_deg, mouth_mid, span, depth, ids, present=("r", "l", "b")):
    """The three tags of one berth, placed from its pose. Boat frame."""
    into = lines.unit_of(into_deg)
    width = lines.normal_of(into_deg)          # from the left tag toward the right
    corners = {
        "r": lines.add(mouth_mid, lines.scale(width, 0.5 * span)),
        "l": lines.add(mouth_mid, lines.scale(width, -0.5 * span)),
        "b": lines.add(mouth_mid, lines.scale(into, depth)),
    }
    facing = lines.bearing_of(lines.scale(into, -1.0))
    return [tag_at(ids[i], corners[key], facing=facing)
            for i, key in enumerate(("r", "l", "b")) if key in present]


def test_tag_geometry():
    section("a berth built from AR tags, and which way it faces")
    from nodes.self_driving.perception import artags as tag_geometry

    # Every axis source, at poses chosen to catch a mirrored or double-rotated
    # answer: dead ahead, well off to each side, and behind the beam.
    worst_into, worst_centre, cases = 0.0, 0.0, 0
    for into_deg in (0.0, 37.0, -52.0, 118.0, -160.0):
        for present in (("r", "l", "b"), ("r", "l"), ("r", "b"), ("l", "b"), ("b",)):
            mid = (0.7, 3.4)
            tags = berth_tags(into_deg, mid, 2.0, 2.0, (0, 1, 7), present)
            berth, report = tag_geometry.find_berth(
                tags, mouth_m=2.0, depth_m=2.0, parallel=False,
                prior_into_deg=into_deg)
            if berth is None:
                check(False, f"{''.join(present)} at {into_deg:.0f} deg found "
                             f"nothing: {report.get('why')}")
                continue
            cases += 1
            truth = lines.add(mid, lines.scale(lines.unit_of(into_deg), 1.0))
            worst_into = max(worst_into,
                             abs(geo.angle_diff(berth.box.into_deg, into_deg)))
            worst_centre = max(worst_centre, math.dist(berth.box.centre, truth))
    check(cases == 25, f"all 25 pose/visibility combinations produced a berth")
    check(worst_into < 0.5,
          f"the way in is never mirrored or double-rotated "
          f"(worst {worst_into:.3f} deg over {cases} cases)")
    check(worst_centre < 0.02,
          f"the middle of the berth lands where it should "
          f"(worst {worst_centre * 100:.2f} cm)")

    # The sign test in its sharpest form: swap the two side tags' ids and the berth
    # must turn round, because the ids are the ONLY thing saying which side is which
    # when there is no closed-end tag.
    swapped = berth_tags(0.0, (0.0, 3.0), 2.0, 2.0, (1, 0, 7), ("r", "l"))
    turned, _ = tag_geometry.find_berth(swapped, mouth_m=2.0, depth_m=2.0,
                                        parallel=False, prior_into_deg=0.0)
    check(turned is not None
          and abs(abs(geo.angle_diff(turned.box.into_deg, 0.0)) - 180.0) < 0.5,
          "swapping the two side tags turns the berth round - the ids, not the "
          "boat, decide which way is in")


def test_tag_berth_choice():
    section("which berth, and whether anything is in it")
    from nodes.self_driving.perception import artags as tag_geometry

    # Two bow-in berths side by side opening south-to-north (way in = 000), sharing
    # the middle finger. Berth 1 to starboard, berth 2 to port, exactly as the
    # organisers' drawing lays them out.
    T = {0: tag_at(0, (1.0, 3.0)), 1: tag_at(1, (-1.0, 3.0)),
         3: tag_at(3, (-3.0, 3.0)), 7: tag_at(7, (0.0, 5.0)),
         2: tag_at(2, (-2.0, 5.0))}

    def pick(ids, **kw):
        return tag_geometry.find_berth(
            [T[i] for i in ids], mouth_m=2.0, depth_m=2.0, parallel=False,
            prior_into_deg=0.0, **kw)

    berth, _ = pick([0, 1, 3, 7])
    check(berth is not None and berth.name == "berth 1"
          and berth.occupancy == "free",
          "berth 2's end tag hidden -> berth 1 is taken, and called free")
    berth, _ = pick([0, 1, 3, 2])
    check(berth is not None and berth.name == "berth 2",
          "berth 1's end tag hidden -> berth 2 is taken")
    berth, _ = pick([0, 1, 3])
    check(berth is not None and berth.occupancy == "unknown",
          "neither end tag in view -> a berth is still found, but occupancy is "
          "'unknown' rather than assumed free")
    berth, _ = pick([0, 1, 3, 7, 2], prefer="berth 2")
    check(berth is not None and berth.name == "berth 2"
          and "operator" in berth.why,
          "the operator's named berth overrules the tags, and says so")

    # The shared finger identifies nothing on its own, and two tags from DIFFERENT
    # berths must not be paired into a 4 m berth that does not exist.
    berth, report = pick([1])
    check(berth is None and "belongs to berth 1 and berth 2" in report["why"],
          "the shared finger tag alone is refused, and the message says why")
    berth, report = pick([0, 3])
    check(berth is None,
          "one tag from each berth is not a 4 m berth (the span check refuses it)")

    # The alongside berth, at its own measured span.
    P = [tag_at(4, (2.065, 3.0)), tag_at(6, (-2.065, 3.0)), tag_at(5, (0.0, 5.0))]
    berth, _ = tag_geometry.find_berth(P, mouth_m=4.0, depth_m=2.0, parallel=True,
                                       prior_into_deg=0.0)
    check(berth is not None and abs(berth.mouth_m - 4.13) < 0.01,
          f"the alongside berth measures its 4.13 m span "
          f"({berth.mouth_m:.2f} m) rather than the 4.00 m clear opening")

    # A tag too far away, or seen too obliquely, is not used. Both are the same
    # failure in practice: the other task's tags, or the next berth's, seen from
    # outside.
    far = [tag_at(0, (1.0, 30.0)), tag_at(1, (-1.0, 30.0)), tag_at(7, (0.0, 32.0))]
    berth, _ = tag_geometry.find_berth(far, mouth_m=2.0, depth_m=2.0,
                                       parallel=False, prior_into_deg=0.0)
    check(berth is None, "tags beyond MAX_RANGE_M are not a berth")
    oblique = [tag_at(0, (1.0, 3.0), incidence=85.0),
               tag_at(1, (-1.0, 3.0), incidence=85.0)]
    berth, _ = tag_geometry.find_berth(oblique, mouth_m=2.0, depth_m=2.0,
                                       parallel=False, prior_into_deg=0.0)
    check(berth is None, "tags seen edge-on are dropped before they can be a berth")


def test_tag_behaviour():
    section("the manoeuvre, driven by tags instead of a lidar")
    behaviour = Parking(config, parallel=False, source="artag")
    check(behaviour.name == "park_tag", "the role names itself park_tag")

    # The same fixed world space the lidar tests use: mouth plane at north 4, space
    # running to north 6, opening south, way in due north. The boat sits south of it.
    def ctx_at(position, heading=0.0, now=0.0, present=("r", "l", "b")):
        # The berth in the BOAT frame, from where the boat is.
        east, north = position
        mid_world = (SPACE_EAST_M, MOUTH_NORTH_M)
        rel = (mid_world[0] - east, mid_world[1] - north)
        stbd, fwd = geo.world_to_boat(rel[0], rel[1], heading)
        tags = berth_tags(geo.angle_diff(0.0, heading), (stbd, fwd),
                          2.0, 2.0, (0, 1, 7), present)
        ctx = _ctx_for(behaviour, [], position=position, heading=heading, now=now,
                       waypoint_xy=(SPACE_EAST_M, MOUTH_NORTH_M - 2.0))
        ctx.tags = tags
        return ctx

    ctx = ctx_at((0.0, 0.0))
    behaviour.start(ctx)
    behaviour.update(ctx)
    check(behaviour.box is not None,
          "the berth is found from the tags, with no sweep at all")
    if behaviour.box is not None:
        check(abs(geo.angle_diff(behaviour.box["into_deg"], 0.0)) < 1.0,
              f"the way in comes out due north "
              f"({behaviour.box['into_deg']:.1f} deg), not out to sea")
        check(math.dist(behaviour.box["centre"], (SPACE_EAST_M,
                                                 MOUTH_NORTH_M + 1.0)) < 0.05,
              "the middle of the berth lands in the middle of the berth")
    told = behaviour.status.get("parking", {})
    check(told.get("sensor") == "artag",
          "the panel says which sensor found it")

    # No tags at all: the behaviour has to say so rather than steer on a memory it
    # never had. This is the state an operator sees if cv2.aruco is missing on the
    # Jetson, which is the failure most likely to reach the water.
    blind = Parking(config, parallel=False, source="artag")
    ctx = ctx_at((0.0, 0.0))
    ctx.tags = []
    blind.start(ctx)
    blind.update(ctx)
    check(blind.box is None, "no tags in view means no berth")
    told = blind.status.get("parking", {})
    check(told.get("seen") is False and told.get("sensor") == "artag",
          "and the panel says it is looking for tags and has none")
    check("no tags in view" in (told.get("tags") or {}).get("why", ""),
          "with the reason in words the operator can act on")

    # An alongside tag park is NOT blind after the turn - the cameras look abeam,
    # which is exactly where the closed end ends up. This is the one place losing
    # the lidars improved the manoeuvre, so it is worth pinning down.
    alongside = Parking(config, parallel=True, source="artag")
    ctx = ctx_at((0.0, 0.0))
    alongside.start(ctx)
    alongside.phase = "hold"
    alongside.update(ctx)
    check(alongside._blind(ctx) is False,
          "an alongside park on tags never reports itself blind, because the "
          "cameras point where the closed end goes")


def main():
    test_lines()
    test_finding()
    test_refusals()
    test_buoys_are_ignored()
    test_envelope()
    test_offset()
    test_behaviour()
    test_hold_restarts_on_drift()
    test_always_normal()
    test_sideways_is_a_trim_not_a_drive()
    test_no_lateral_thruster()
    test_parallel_behaviour()
    test_offset_clamp()
    test_probe()
    test_blind_alongside_hold()
    test_speed_setting_is_obeyed()
    test_front_lidar_only()
    test_tag_geometry()
    test_tag_berth_choice()
    test_tag_behaviour()

    print()
    if FAILURES:
        print(f"{len(FAILURES)} check(s) FAILED:")
        for failure in FAILURES:
            print(f"  - {failure}")
        return 1
    print("all checks passed")
    print(
        "\nNone of this proves the boat fits, that the thrusters hold it in a "
        "tide,\nor that the aft lidar is mounted the way the code thinks. "
        "See docs/testing.md 7j."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
