#!/usr/bin/env python3
"""Fly the whole autonomy stack against a simulated boat. No hardware needed.

    python3 tests/test_autopilot.py            # run everything
    python3 tests/test_autopilot.py -v         # ...and print each tick

This exists because the alternative is finding the sign errors on the water with
a jury watching and fifteen minutes on the clock. Everything from
`perception.cluster` through `pilot.tick` is a pure function of its inputs by
construction (`behaviours/base.py`), so the only things that need faking are the
boat's motion and the lidar - which is what `FakeBoat` and `fake_sweep` are.

What is real and what is not
----------------------------
    real    geo, clustering, colour classification, the world model, every
            behaviour, the plan, the pilot's safety checks, the recorder
    fake    the hull's response to a command (a first-order kinematic model),
            the lidar returns (ray-cast against discs), and the node bus

The kinematics are deliberately crude - no added mass, no current, no thruster
lag. They are not trying to predict how the boat handles; they are there so that
"drive north" moves the state north, which is all that is needed to catch a
transposed axis or an inverted buoy rule. Anything that depends on the real
dynamics has to be checked on the water regardless.
"""

from __future__ import annotations

import json
import math
import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# BEFORE importing config, which reads these at import time. The pilot saves the
# plan on every waypoint it advances past, so without this a test run overwrites
# the boat's real course - which is a genuinely bad thing to do at 08:55 on a
# competition morning, when the plan file is the morning's handout.
_SANDBOX = tempfile.mkdtemp(prefix="ligmax-test-")
os.environ["LIGMAX_AP_PLAN_FILE"] = os.path.join(_SANDBOX, "plan.json")
os.environ["LIGMAX_AP_RECORD_DIR"] = os.path.join(_SANDBOX, "trips")

import numpy as np  # noqa: E402

from nodes.self_driving import commander as commander_module  # noqa: E402
from nodes.self_driving import config, geo  # noqa: E402
from nodes.self_driving.obsticales import ObstacleType  # noqa: E402
from nodes.self_driving.perception import cluster_sweep, masks  # noqa: E402
from nodes.self_driving.perception.classify import classify, colour_votes  # noqa: E402
from nodes.self_driving.perception.world import WorldModel  # noqa: E402
from nodes.self_driving.pilot import Pilot  # noqa: E402
from nodes.self_driving.plan import Plan, PlanError  # noqa: E402
from nodes.self_driving.state import BoatState  # noqa: E402

VERBOSE = "-v" in sys.argv
ORIGIN = {"lat": 63.4390, "lon": 10.3990}  # Trondheim, near the Njord course

FAILURES = []


def check(condition, message):
    if condition:
        print(f"  ok    {message}")
    else:
        print(f"  FAIL  {message}")
        FAILURES.append(message)


def section(title):
    print(f"\n=== {title}")


# --------------------------------------------------------------- fake world

COLOURS = {
    ObstacleType.RED: (200, 30, 30),
    ObstacleType.GREEN: (40, 220, 70),
    ObstacleType.CARDINAL: (230, 200, 40),
    ObstacleType.NORTH: (230, 200, 40),
    ObstacleType.EAST: (230, 200, 40),
    ObstacleType.WEST: (230, 200, 40),
    ObstacleType.BOAT: (235, 235, 235),
    ObstacleType.LAND: (225, 225, 220),
}


class Obstacle:
    """A disc in the world, with a colour the fake lidar will report."""

    def __init__(self, xy, kind, radius=0.2, velocity=(0.0, 0.0)):
        self.xy = list(xy)
        self.kind = kind
        self.radius = radius
        self.velocity = velocity

    def step(self, dt):
        self.xy[0] += self.velocity[0] * dt
        self.xy[1] += self.velocity[1] * dt


def fake_sweep(boat_xy, heading, obstacles, fov_deg=120.0, step_deg=0.9):
    """Ray-cast the obstacles into a boat-frame sweep. `(points, flat rgb)`.

    Modelled on the real C1: 0.9 deg steps, 12 m range, a return only where a
    ray actually strikes something. Points come out `[starboard, forward]`, the
    frame `scan.py` delivers, so the code under test sees exactly what it sees
    on the boat.
    """
    points, rgb = [], []
    for bearing in np.arange(-fov_deg, fov_deg, step_deg):
        rad = math.radians(bearing)
        # Ray direction in the world frame.
        direction = geo.boat_to_world(math.sin(rad), math.cos(rad), heading)
        best = None
        for obstacle in obstacles:
            hit = _ray_disc(boat_xy, direction, obstacle.xy, obstacle.radius)
            if hit is not None and (best is None or hit < best[0]):
                best = (hit, obstacle)
        if best is None or best[0] > 12.0:
            continue
        distance, obstacle = best
        points.append([distance * math.sin(rad), distance * math.cos(rad)])
        rgb.extend(COLOURS.get(obstacle.kind, (120, 120, 120)))
    return points, rgb


def _ray_disc(origin, direction, centre, radius):
    """Distance along a unit ray to a disc, or None."""
    ox = centre[0] - origin[0]
    oy = centre[1] - origin[1]
    along = ox * direction[0] + oy * direction[1]
    if along <= 0.0:
        return None
    perpendicular_sq = ox * ox + oy * oy - along * along
    if perpendicular_sq > radius * radius:
        return None
    return along - math.sqrt(max(0.0, radius * radius - perpendicular_sq))


class FakeBoat:
    """First-order kinematics. Enough to catch a sign error, not a hull model.

    **A command is held until the next one replaces it**, and that is not a
    detail - it is what ArduPilot does with a GUIDED target, and it is the whole
    reason `TARGET_REFRESH_S` exists. An earlier version of this class turned and
    accelerated only on the ticks a message happened to arrive on, which quietly
    made the boat's turn rate a property of `Commander`'s re-send logic rather
    than of the hull: a run whose aim point sat still for a few ticks turned at
    8 deg/s instead of 40, and the simulated boat swung wide of corners for a
    reason no real boat would. Anything measured about how the boat handles a
    corner is meaningless without this.
    """

    MAX_YAW = 0.7      # rad/s, at low speed
    ACCEL = 1.5        # m/s^2
    # The reason a fast boat cannot turn like a slow one. A turn is lateral
    # acceleration - `v * yaw_rate` - and a hull can only supply so much of it
    # before it slides sideways instead of turning. Modelling the yaw rate as a
    # constant regardless of speed, as this class used to, gives a boat whose turn
    # radius shrinks in proportion to its speed: it rounds a 123 degree corner at
    # 5 knots as tidily as at 1, no speed limit is ever needed, and the whole
    # question this suite exists to answer is answered wrongly. 1.0 m/s^2 is a
    # guess of the right order for a light trimaran and, like `TURN_YAW_RATE`, is
    # a number to replace with a measurement.
    MAX_LATERAL_ACCEL = 1.0  # m/s^2

    def __init__(self, xy=(0.0, 0.0), heading=0.0):
        self.xy = list(xy)
        self.heading = heading
        self.speed = 0.0
        self.lateral = 0.0
        self.mode = "GUIDED"
        self.armed = True
        self.sent = []
        # The standing order: `("position", target_xy, speed)` or
        # `("velocity", yaw_rate, vx)`. None until the first one arrives.
        self.holding = None

    def apply(self, message, dt):
        """Consume one control message from the fake node bus."""
        command = message.get("cmd")
        self.sent.append(message)
        if command == "set_mode":
            self.mode = message["mode"]
            return
        if command == "arm":
            self.armed = bool(message.get("arm"))
            return
        if command == "position_target":
            self.holding = (
                "position",
                geo.to_world(message["lat"], message["lon"], ORIGIN),
                float(message.get("speed") or 0.5),
            )
            return
        if command == "velocity_target":
            self.holding = (
                "velocity",
                float(message.get("yaw_rate") or 0.0),
                float(message.get("vx") or 0.0),
            )
            self.lateral = float(message.get("vy") or 0.0)

    def _obey(self, dt):
        """Keep doing whatever was last commanded, for one tick."""
        if self.holding is None:
            return
        kind = self.holding[0]
        if kind == "position":
            _kind, target, speed = self.holding
            if target is not None and geo.distance(self.xy, target) > 0.05:
                self._turn(geo.bearing_to(self.xy, target), dt)
            self._accelerate(speed, dt)
            return
        _kind, yaw, forward = self.holding
        self.heading = geo.wrap360(self.heading + math.degrees(yaw) * dt)
        self._accelerate(forward, dt, signed=True)

    def _turn(self, wanted, dt):
        error = geo.angle_diff(wanted, self.heading)
        yaw = min(
            self.MAX_YAW, self.MAX_LATERAL_ACCEL / max(0.1, abs(self.speed))
        )
        rate = math.degrees(yaw) * dt
        self.heading = geo.wrap360(self.heading + max(-rate, min(rate, error)))

    def _accelerate(self, target, dt, signed=False):
        limit = self.ACCEL * dt
        self.speed += max(-limit, min(limit, target - self.speed))
        if not signed:
            self.speed = max(0.0, self.speed)

    def step(self, dt):
        self._obey(dt)
        forward = geo.boat_to_world(0.0, self.speed, self.heading)
        side = geo.boat_to_world(self.lateral, 0.0, self.heading)
        self.xy[0] += (forward[0] + side[0]) * dt
        self.xy[1] += (forward[1] + side[1]) * dt

    def state(self, now):
        velocity = geo.boat_to_world(self.lateral, self.speed, self.heading)
        return BoatState(
            {
                "t": now,
                "origin": ORIGIN,
                "boat": {
                    "position": list(self.xy),
                    "heading_deg": self.heading,
                    "velocity": list(velocity),
                },
                "gps": {"fix": "RTK_FIXED", "lat": 0.0, "lon": 0.0},
                "motion": {"sog": abs(self.speed), "cog_deg": self.heading},
                "mode": self.mode,
                "armed": self.armed,
                "estop": False,
                "status": "AUTONOMOUS",
            },
            received_at=now,
        )


class FakeLink:
    """The node bus, replaced by a list. `commander.py` cannot tell."""

    def __init__(self, boat):
        self.boat = boat
        self.messages = []

    def control(self, **fields):
        self.messages.append(fields)
        return True

    def telemetry(self, **_blocks):
        return True

    def ack(self, *_args, **_kwargs):
        return True

    def log(self, *_args, **_kwargs):
        return True


def run(plan, obstacles, start=(0.0, 0.0), heading=0.0, seconds=180.0, dt=0.1,
        moving=(), watch=None, profile=None, alternation=False, world=None):
    """Fly a plan. Returns `(pilot, boat, track, ticks)`.

    `watch(pilot, world, boat, now)` is called every tick, for the assertions
    that are about what happened *during* the run rather than where it ended -
    a track that was committed and then aged out, a docking phase that was
    entered and left, are invisible from the final state.

    `profile` selects a run profile the way the operator's command does, and
    `world` lets a run start from a world model an earlier run left behind -
    which is the whole two-attempt story (`profiles.py`) and cannot be tested
    without flying one run into the next.
    """
    boat = FakeBoat(start, heading)
    link = FakeLink(boat)
    commander = commander_module.Commander(link, config)
    if profile is not None:
        ok, why = commander.set_profile(profile)
        assert ok, why
    commander.run.alternation = bool(alternation)
    pilot = Pilot(config, commander)
    pilot.plan = plan
    pilot.start()
    world = WorldModel(config) if world is None else world

    track = []
    now = 1000.0
    ticks = 0
    while now < 1000.0 + seconds and not (plan.finished or pilot.mode == "FINISHED"):
        for obstacle in moving:
            obstacle.step(dt)
        state = boat.state(now)
        points, rgb = fake_sweep(boat.xy, boat.heading, obstacles)
        clusters = (
            cluster_sweep(points, rgb, source="front_lidar", config=config)
            if points
            else []
        )
        task = "transit"
        if pilot.behaviour is not None:
            task = getattr(pilot.behaviour, "task", "transit")
        world.observe(clusters, boat.xy, boat.heading, now, task)

        intent = pilot.tick(state, world, clusters, now)
        commander.send(intent, state, now)
        for message in link.messages:
            boat.apply(message, dt)
        link.messages.clear()
        boat.step(dt)

        if watch is not None:
            watch(pilot, world, boat, now)
        track.append(tuple(boat.xy))
        if VERBOSE and ticks % 20 == 0:
            print(
                f"    t={now - 1000:6.1f} ({boat.xy[0]:7.2f},{boat.xy[1]:7.2f}) "
                f"hdg={boat.heading:5.1f} v={boat.speed:5.2f} :: {intent.reason[:64]}"
            )
        now += dt
        ticks += 1
    return pilot, boat, track, ticks


def make_plan(points, role="transit", **extra):
    return Plan.parse(
        {
            "name": "test",
            "waypoints": [
                dict({"name": f"w{i}", "x": x, "y": y, "role": role}, **extra)
                for i, (x, y) in enumerate(points)
            ],
        },
        ORIGIN,
    )


def min_distance(track, xy):
    return min(math.dist(p, xy) for p in track) if track else float("inf")


def side_of(track, obstacle_xy, leg_bearing):
    """Which side of an obstacle the track passed. `+1` starboard, `-1` port.

    Measured at the closest point of approach, projected across the leg
    direction - i.e. exactly what "the mark was on our port side" means.
    """
    closest = min(track, key=lambda p: math.dist(p, obstacle_xy))
    east = closest[0] - obstacle_xy[0]
    north = closest[1] - obstacle_xy[1]
    across, _along = geo.world_to_boat(east, north, leg_bearing)
    # The BOAT is `across` to starboard of the mark, so the mark is on the
    # boat's port when `across` is positive.
    return 1 if across > 0 else -1


# ------------------------------------------------------------------- tests

def test_geo():
    section("geometry")
    check(abs(geo.bearing_to((0, 0), (0, 10)) - 0.0) < 1e-6, "north is bearing 0")
    check(abs(geo.bearing_to((0, 0), (10, 0)) - 90.0) < 1e-6, "east is bearing 90")
    check(abs(geo.bearing_to((0, 0), (0, -10)) - 180.0) < 1e-6, "south is bearing 180")

    east, north = geo.boat_to_world(0.0, 1.0, 0.0)
    check(abs(north - 1.0) < 1e-9 and abs(east) < 1e-9, "heading 0: forward is north")
    east, north = geo.boat_to_world(1.0, 0.0, 0.0)
    check(abs(east - 1.0) < 1e-9, "heading 0: starboard is east")
    east, north = geo.boat_to_world(0.0, 1.0, 90.0)
    check(abs(east - 1.0) < 1e-9, "heading 90: forward is east")
    east, north = geo.boat_to_world(1.0, 0.0, 90.0)
    check(abs(north + 1.0) < 1e-9, "heading 90: starboard is south")

    for heading in (0.0, 37.0, 123.0, 271.0):
        s, f = geo.world_to_boat(*geo.boat_to_world(1.3, -2.7, heading), heading)
        check(
            abs(s - 1.3) < 1e-9 and abs(f + 2.7) < 1e-9,
            f"boat<->world round trips at heading {heading:.0f}",
        )

    # A point 5 m to starboard of a leg running north.
    _t, along, cross = geo.project_onto_leg((5.0, 10.0), (0.0, 0.0), (0.0, 20.0))
    check(abs(along - 10.0) < 1e-6, "along-leg distance is right")
    check(abs(cross - 5.0) < 1e-6, "cross-track is POSITIVE to starboard")

    _t, _along, cross = geo.project_onto_leg((-5.0, 10.0), (0.0, 0.0), (0.0, 20.0))
    check(cross < 0, "cross-track is negative to port")

    t, _a, _c = geo.project_onto_leg((0.0, 25.0), (0.0, 0.0), (0.0, 20.0))
    check(t > 1.0, "the passing plane fires past the end of a leg")

    # Head-on closing at 1 + 1 m/s from 20 m apart: 10 s to a zero CPA.
    tcpa, cpa = geo.closest_point_of_approach(
        (0.0, 0.0), (0.0, 1.0), (0.0, 20.0), (0.0, -1.0)
    )
    check(abs(tcpa - 10.0) < 0.01 and cpa < 0.01, "CPA of a head-on closure")
    tcpa, _cpa = geo.closest_point_of_approach(
        (0.0, 0.0), (0.0, 1.0), (0.0, -20.0), (0.0, -1.0)
    )
    check(tcpa < 0, "CPA is in the past for an opening range")

    lat, lon = geo.to_global(100.0, 200.0, ORIGIN)
    east, north = geo.to_world(lat, lon, ORIGIN)
    check(abs(east - 100.0) < 0.01 and abs(north - 200.0) < 0.01, "lat/lon round trips")


def test_colour():
    section("colour classification")
    for name, rgb in (
        ("red", (200, 30, 30)),
        ("green", (40, 220, 70)),
        ("yellow", (230, 200, 40)),
        ("white", (235, 235, 235)),
        ("dark", (18, 22, 30)),
    ):
        tally, n, weighted = colour_votes([rgb] * 8, config)
        check(tally.get(name, 0) == 8, f"{rgb} reads as {name} ({tally})")
        check(
            weighted.get(name, 0) == 8,
            f"{rgb} carries its full weight with no age on the wire",
        )

    # Uncoloured returns must never read as dark - that is the whole reason
    # scan.py writes -1 rather than black.
    tally, n, weighted = colour_votes([(-1, -1, -1)] * 6, config)
    check(n == 0 and not tally, "uncoloured returns are ignored, not called dark")

    # A whole buoy: clustered, then classified.
    points, rgb = fake_sweep(
        (0.0, 0.0), 0.0, [Obstacle((0.0, 6.0), ObstacleType.RED, 0.2)]
    )
    clusters = cluster_sweep(points, rgb, source="front_lidar", config=config)
    check(len(clusters) == 1, f"one buoy makes one cluster (got {len(clusters)})")
    if clusters:
        kind, confidence, why = classify(clusters[0], config)
        check(kind == ObstacleType.RED, f"a red buoy classifies as RED ({why})")
        check(clusters[0].width_m < config.MAX_MARK_WIDTH_M, "a buoy is mark-sized")

    # The Otter: 2 m across, so size alone must call it a vessel even though its
    # colour is only 'white'. NJORD §9.2 warns the colour may vary.
    points, rgb = fake_sweep(
        (0.0, 0.0), 0.0, [Obstacle((0.0, 8.0), ObstacleType.BOAT, 1.0)]
    )
    clusters = cluster_sweep(points, rgb, source="front_lidar", config=config)
    if clusters:
        kind, _c, why = classify(clusters[0], config, context="avoid")
        check(kind == ObstacleType.BOAT, f"a 2 m object is a vessel ({why})")
        kind, _c, _w = classify(clusters[0], config, context="dock")
        check(kind == ObstacleType.LAND, "the same object is structure while docking")

    # Two buoys 5 m apart - a Njord gate (§9.2) - must stay two clusters and
    # not weld into one. At 8 m, which is inside MAX_OBSTACLE_RANGE_M; the C1
    # cannot resolve a gate at its 12 m datasheet limit anyway.
    points, rgb = fake_sweep(
        (0.0, 0.0), 0.0,
        [
            Obstacle((-2.5, 8.0), ObstacleType.RED, 0.2),
            Obstacle((2.5, 8.0), ObstacleType.GREEN, 0.2),
        ],
    )
    clusters = cluster_sweep(points, rgb, source="front_lidar", config=config)
    check(len(clusters) == 2, f"a 5 m gate is two clusters (got {len(clusters)})")
    if len(clusters) == 2:
        kinds = sorted(classify(c, config)[0].name for c in clusters)
        check(kinds == ["GREEN", "RED"], f"and they are told apart by colour {kinds}")


def test_loose_marks():
    """One painted dot is a buoy, and a marks task only reports marks.

    The three complaints this answers, from the water:

      * marks plainly visible in the lidar were not being detected, because a
        sweep that grazes a 40 cm dome leaves one or two painted returns among a
        crowd of dark ones and both the cluster threshold and the colour vote
        threw that away;
      * green was never detected AT ALL, because red and green were sharing a
        saturation threshold that a warm cast moves in opposite directions;
      * a leg that follows GPS points under buoy rules was reporting a harbour
        full of vessels and shoreline, none of which is part of that task.
    """
    section("loose mark detection, and detection that knows its task")

    from nodes.self_driving.perception.classify import policy_for

    dark, white = (18, 22, 30), (230, 230, 230)
    green, red = (20, 200, 60), (210, 30, 30)

    # ---- one dot is enough, at every stage of the chain --------------------
    single = cluster_sweep([[0.0, 6.0]], list(green), config=config)
    check(len(single) == 1, f"a single painted return makes a cluster ({len(single)})")
    check(
        not cluster_sweep([[0.0, 6.0]], list(dark), config=config),
        "...and a single dark one does not - MIN_CLUSTER_POINTS still bites",
    )
    check(
        not cluster_sweep([[0.0, 6.0]], None, config=config),
        "...nor a single uncoloured one, which could be any stray beam",
    )
    if single:
        kind, confidence, why = classify(single[0], config, context="buoys")
        check(kind == ObstacleType.GREEN, f"and one green dot IS a green buoy ({why})")
        check(
            0.0 < confidence < 0.5,
            f"reported as weak evidence, not certainty ({confidence:.2f})",
        )

    # A mark is not outvoted by the water it is standing in front of.
    grazed = _rgb_cluster([green] + [dark] * 6)
    kind, _c, why = classify(grazed, config, context="buoys")
    check(kind == ObstacleType.GREEN, f"1 green against 6 dark is a green buoy ({why})")
    check("background" in why, f"...and says so in words for the log ({why})")

    # ...but IS outvoted by a mark colour that disagrees, which is the only
    # disagreement that means anything: guessing here is the failure being scored.
    kind, _c, why = classify(_rgb_cluster([green, red]), config, context="buoys")
    check(
        kind == ObstacleType.UNKNOWN,
        f"one green against one red stays UNKNOWN rather than guessing ({why})",
    )
    kind, _c, _w = classify(_rgb_cluster([green, green, red]), config, context="buoys")
    check(kind == ObstacleType.GREEN, "two green against one red resolves to green")

    # ---- green, calibrated for the cast the camera actually sends ----------
    # A warm cast lifts the red channel, which on a GREEN object is the minimum
    # channel - so chroma shrinks and saturation falls. These are the returns that
    # were being called grey.
    for name, rgb in (
        ("neon green under the measured warm cast", (100, 200, 72)),
        ("a dome in shade, half mixed with water", (86, 120, 92)),
        ("a yellow-green dragged down by the cast", (150, 190, 60)),
        ("green against water, dragged towards teal", (40, 170, 150)),
    ):
        kind, _c, why = classify(_rgb_cluster([rgb] * 3), config, context="buoys")
        check(kind == ObstacleType.GREEN, f"{name} {rgb} reads GREEN ({why})")

    # And the thing the low green bar must NOT swallow: an actual grey.
    kind, _c, why = classify(_rgb_cluster([(95, 101, 96)] * 6), config, context="buoys")
    check(
        kind != ObstacleType.GREEN,
        f"a grey with a green tint is not a buoy ({why})",
    )
    # Nor the warm-lit grey that the red bar exists for.
    kind, _c, why = classify(_rgb_cluster([(120, 72, 66)] * 6), config, context="buoys")
    check(
        kind != ObstacleType.RED,
        f"the measured warm cast on a grey surface is not a red buoy ({why})",
    )

    # ---- the same returns, named differently per task ----------------------
    otter = _rgb_cluster([white] * 20, width=2.0, centre=(0.0, 8.0))
    named = {ctx: classify(otter, config, context=ctx)[0] for ctx in
             ("transit", "avoid", "dock", "buoys")}
    check(named["avoid"] == ObstacleType.BOAT, "giving way, a 2 m object is a vessel")
    check(named["dock"] == ObstacleType.LAND, "docking, the same object is structure")
    check(named["transit"] == ObstacleType.BOAT, "on a blind leg it is still a vessel")
    check(
        named["buoys"] == ObstacleType.UNKNOWN,
        f"under buoy rules it is named neither ({named['buoys'].name}) - that task "
        f"is about the marks",
    )

    # A painted cluster too wide for the strict threshold is still a mark on a
    # marks task, given more than one painted return.
    wide = _rgb_cluster([green] * 2 + [white] * 30, width=1.7, centre=(0.0, 5.0))
    check(
        classify(wide, config, context="buoys")[0] == ObstacleType.GREEN,
        "a 1.7 m cluster with two green returns is a mark under buoy rules",
    )
    thin_evidence = _rgb_cluster([green] + [white] * 30, width=1.7, centre=(0.0, 5.0))
    check(
        classify(thin_evidence, config, context="buoys")[0] == ObstacleType.UNKNOWN,
        "...but one green return on a 1.7 m cluster is not enough to call it one",
    )

    # ---- water is a positive answer, and it is not an object --------------
    puddle = _rgb_cluster([dark] * 8, centre=(0.0, 9.0))
    kind, _c, why = classify(puddle, config, context="buoys")
    check(
        kind == ObstacleType.WATER,
        f"a dark cluster the camera DID colour is water, not unknown ({why})",
    )
    blind = _fake_cluster((0.0, 9.0), "green")
    blind.rgb = None
    check(
        classify(blind, config, context="buoys")[0] == ObstacleType.UNKNOWN,
        "while a cluster no camera covered stays UNKNOWN - an object of unknown kind",
    )

    # ---- what each task keeps ---------------------------------------------
    marks_task, blind_task = policy_for("buoys", config), policy_for("transit", config)
    far = config.BUOY_TASK_CLUTTER_RANGE_M + 3.0
    check(
        not marks_task.tracks(ObstacleType.UNKNOWN, far, 4.0)[0],
        "under buoy rules, 4 m of shoreline beyond the clutter range is not tracked",
    )
    check(
        marks_task.tracks(ObstacleType.UNKNOWN, config.BUOY_TASK_CLUTTER_RANGE_M - 1, 4.0)[0],
        "...the same shoreline close enough to hit still is",
    )
    check(
        marks_task.tracks(ObstacleType.UNKNOWN, far, 0.4)[0],
        "...and anything mark-SIZED is tracked at any range, so the camera can "
        "still upgrade it",
    )
    check(
        not marks_task.tracks(ObstacleType.WATER, far, 0.3)[0],
        "...but water out at range is not, however small",
    )
    check(
        marks_task.tracks(ObstacleType.GREEN, 40.0, 0.4)[0],
        "...and a mark is kept at any range at all",
    )
    check(
        blind_task.tracks(ObstacleType.LAND, 40.0, 9.0)[0]
        and blind_task.tracks(ObstacleType.WATER, 40.0, 0.3)[0],
        "off a marks task nothing is dropped - unchanged behaviour",
    )

    # ---- through the world model: the chart, and how fast a mark counts ----
    world = WorldModel(config)
    now = 3000.0
    world.observe([_rgb_cluster([green] * 6, centre=(2.0, 8.0))], (0.0, 0.0), 0.0,
                  now, "buoys")
    check(not world.confirmed(), "one sighting of a mark is not yet steerable")
    world.observe([_rgb_cluster([green] * 6, centre=(2.0, 8.0))], (0.0, 0.0), 0.0,
                  now + 0.1, "buoys")
    check(
        len(world.marks()) == 1,
        f"two sightings confirm it - a mark is only useful while there is room to "
        f"choose a side ({len(world.marks())})",
    )

    # Water close in: tracked, avoided, and never drawn.
    wet = WorldModel(config)
    for i in range(4):
        wet.observe([_rgb_cluster([dark] * 8, centre=(0.0, 3.0))], (0.0, 0.0), 0.0,
                    now + i * 0.1, "buoys")
    water_tracks = [t for t in wet.all() if t.kind == ObstacleType.WATER]
    check(bool(water_tracks), "spray 3 m off the bow is tracked, so it is avoided")
    check(
        wet.telemetry() == [],
        f"...and never drawn on the chart ({len(wet.telemetry())} marker(s))",
    )
    check("water" not in wet.summary(), f"nor counted at a glance ({wet.summary()})")
    check(
        any(m.get("kind") == "WATER" for m in wet.last_measurements),
        "but the trip recording keeps it, so a swerve stays explainable",
    )


def _rgb_cluster(rgbs, width=0.4, centre=(0.0, 6.0)):
    """A `Cluster` with exactly the returns given, for the colour rules."""
    from nodes.self_driving.perception.cluster import Cluster

    arr = np.array(rgbs, dtype=np.int64).reshape(-1, 3)
    return Cluster(
        centre=(float(centre[0]), float(centre[1])),
        nearest=(float(centre[0]), float(centre[1])),
        range_m=math.hypot(*centre),
        bearing_deg=math.degrees(math.atan2(centre[0], centre[1])),
        width_m=width,
        n=arr.shape[0],
        rgb=arr,
        source="front_lidar",
    )


def test_colour_age_weighting():
    """A well-timed return outvotes a mistimed one (§5.2.3).

    179 of 269 coloured points in the 2026-08-08 capture were `stale`, so the
    question this answers is not academic: it is what happens when most of a
    cluster's colour came from a frame a quarter-second away from the return.
    """
    section("colour votes weighted by how mistimed the colour is")

    red, green = (200, 30, 30), (40, 220, 70)
    fresh, stale = config.COLOUR_AGE_FRESH_MS, config.COLOUR_AGE_STALE_MS

    # The whole point, in one case: five stale reds against three fresh greens.
    # On a straight count red wins 5-3. On weight, 5 x 0.25 = 1.25 against
    # 3 x 1.0 = 3.0, so green wins - and the raw tally still says red 5.
    rgb = [red] * 5 + [green] * 3
    ages = [stale] * 5 + [fresh] * 3
    tally, n, weighted = colour_votes(rgb, config, age_ms=ages)
    check(tally["red"] == 5 and tally["green"] == 3, f"the raw tally is unchanged {tally}")
    check(n == 8, "and all eight returns still count as coloured")
    check(
        weighted["green"] > weighted["red"],
        f"but fresh green outweighs stale red ({weighted['green']:.2f} "
        f"vs {weighted['red']:.2f})",
    )

    # Same returns, same ages: with no age array the old behaviour must come
    # back exactly, or this change has silently altered every existing sweep.
    tally_flat, _n, weighted_flat = colour_votes(rgb, config)
    check(
        weighted_flat == {"red": 5.0, "green": 3.0} and tally_flat == tally,
        "with no age_ms the weighted tally is the raw one, unchanged",
    )

    # A uniformly stale cluster must not be punished. Only disagreement is
    # measured, and every return here agrees.
    _t, _n, uniform = colour_votes([red] * 6, config, age_ms=[stale] * 6)
    total = sum(uniform.values())
    check(
        abs(uniform["red"] / total - 1.0) < 1e-9,
        "a uniformly stale cluster is still unanimous, not downgraded",
    )

    # A mismatched age array is dropped rather than applied to the wrong points.
    _t, _n, mismatched = colour_votes(rgb, config, age_ms=[fresh] * 3)
    check(
        mismatched == {"red": 5.0, "green": 3.0},
        "an age array of the wrong length is ignored, not misapplied",
    )

    # -1 is "nothing coloured this", not "infinitely fresh". Those returns are
    # dropped by the lit mask, so what matters is that they do not drag a real
    # return's weight with them when the arrays are sliced.
    _t, n_lit, part = colour_votes(
        [red] * 2 + [(-1, -1, -1)] * 4, config, age_ms=[fresh] * 2 + [-1.0] * 4
    )
    check(n_lit == 2 and part.get("red") == 2.0, "uncoloured returns carry no vote")

    # And the whole chain: ages survive cluster_sweep's filter and sort, so a
    # cluster's ages line up with its own returns rather than the sweep's.
    points, rgb_sweep = fake_sweep(
        (0.0, 0.0), 0.0, [Obstacle((0.0, 6.0), ObstacleType.RED, 0.2)]
    )
    n_points = len(points)
    clusters = cluster_sweep(
        points, rgb_sweep, source="front_lidar", config=config,
        age_ms=[fresh] * n_points,
    )
    check(
        bool(clusters) and clusters[0].age_ms is not None
        and len(clusters[0].age_ms) == clusters[0].n,
        "a cluster's ages are the same length as its returns",
    )
    if clusters:
        kind, _c, why = classify(clusters[0], config)
        check(kind == ObstacleType.RED, f"and a fresh red buoy still reads RED ({why})")


def test_masks():
    section("self-view masks")
    points = [
        [0.0, 2.0],    # dead ahead, inside the corridor -> the boat itself
        [0.3, 5.0],    # still inside the 0.5 m corridor
        [2.0, 5.0],    # off to starboard -> a real object
        [0.0, -3.0],   # astern -> what the aft unit is for
    ]
    keep = masks.keep_mask(points, "box", 0.5, -0.6, 60.0)
    check(list(keep) == [False, False, True, True], "the box mask cuts the hull only")

    kept, _rgb = masks.apply(points, None, keep)
    check(len(kept) == 2, "apply() drops the masked returns")

    # The rgb array MUST be filtered with the points or every colour after the
    # first dropped point shifts by one.
    rgb = [1, 1, 1, 2, 2, 2, 3, 3, 3, 4, 4, 4]
    kept, kept_rgb = masks.apply(points, rgb, keep)
    check(
        kept_rgb is not None and list(kept_rgb[0]) == [3, 3, 3],
        "rgb stays aligned with its points through a mask",
    )

    keep = masks.keep_mask(points, "sector", 0.5, -0.6, 60.0)
    check(list(keep) == [False, False, False, True], "the sector mask is blunter")
    keep = masks.keep_mask(points, "none", 0.5, -0.6, 60.0)
    check(all(keep), "'none' keeps everything")


def test_plan():
    section("the plan")
    plan = make_plan([(0, 20), (0, 40)])
    check(len(plan) == 2, "a two-point plan parses")
    check(plan.current.name == "w0", "the cursor starts at the first waypoint")
    plan.advance()
    check(plan.current.name == "w1" and plan.last_passed == 0, "advance moves on")
    plan.rewind()
    check(plan.current.name == "w0", "rewind is NJORD 8.2's re-entry")

    for bad, why in (
        ({"waypoints": []}, "an empty plan"),
        ({"waypoints": [{"x": 1, "y": 2, "role": "nonsense"}]}, "an unknown role"),
        ({"waypoints": [{"role": "transit"}]}, "a waypoint with no position"),
        ({"waypoints": [{"x": 1, "y": 2, "speed": 99}]}, "an absurd speed"),
    ):
        try:
            Plan.parse(bad, ORIGIN)
            check(False, f"{why} is refused")
        except PlanError:
            check(True, f"{why} is refused")

    # Grid metres need an origin; lat/lon does not.
    try:
        Plan.parse({"waypoints": [{"x": 1, "y": 2}]}, None)
        check(False, "grid metres without an origin are refused")
    except PlanError:
        check(True, "grid metres without an origin are refused")

    plan = Plan.parse(
        {"waypoints": [{"lat": ORIGIN["lat"], "lon": ORIGIN["lon"]}]}, None
    )
    check(len(plan) == 1, "lat/lon needs no origin")

    with tempfile.TemporaryDirectory() as directory:
        path = os.path.join(directory, "plan.json")
        plan = make_plan([(0, 20), (0, 40), (10, 40)])
        plan.advance()
        plan.save(path)
        restored = Plan.load(path)
        check(
            restored is not None and restored.index == 1,
            "a plan survives a restart WITH its cursor",
        )


def test_transit():
    section("transit - NJORD 9.1 part 1")
    plan = make_plan([(0, 25), (25, 25), (25, 0)])
    pilot, boat, track, ticks = run(plan, [], seconds=200)
    check(plan.finished, f"a three-leg course completes ({ticks} ticks)")
    check(math.dist(boat.xy, (25.0, 0.0)) < 5.0, "it finishes near the last waypoint")

    # The passing-plane release: a waypoint the boat cannot quite reach must not
    # trap it in an orbit. Squeeze the radius hard.
    plan = Plan.parse(
        {
            "waypoints": [
                {"name": "tight", "x": 0, "y": 30, "role": "transit", "radius": 0.4},
                {"name": "end", "x": 0, "y": 60, "role": "transit"},
            ]
        },
        ORIGIN,
    )
    pilot, boat, track, ticks = run(plan, [], seconds=200)
    check(plan.finished, "a 0.4 m acceptance radius still completes (passing plane)")


def test_avoidance():
    section("obstacle avoidance on a blind leg")
    buoy = Obstacle((0.0, 25.0), ObstacleType.RED, 0.2)
    plan = make_plan([(0, 50)])
    pilot, boat, track, ticks = run(plan, [buoy], seconds=200)
    closest = min_distance(track, buoy.xy)
    check(plan.finished, "the leg completes with a buoy on the line")
    check(
        closest > 0.8,
        f"it does not hit the buoy on the line (closest {closest:.2f} m)",
    )


def test_buoy_rules():
    section("buoy rules - NJORD 9.1 part 2 / 10.2")
    # Sailing north (with the buoyage): red must end up to PORT.
    red = Obstacle((0.0, 25.0), ObstacleType.RED, 0.2)
    plan = Plan.parse(
        {
            "channel_bearing": 0.0,
            "waypoints": [{"name": "n", "x": 0, "y": 50, "role": "buoys"}],
        },
        ORIGIN,
    )
    pilot, boat, track, ticks = run(plan, [red], seconds=240)
    check(plan.finished, "the buoy leg completes")
    check(
        side_of(track, red.xy, 0.0) > 0,
        "sailing north, a RED mark is left to port (we pass to its east)",
    )

    # Same geometry, green: must end up to STARBOARD, so we pass to its west.
    green = Obstacle((0.0, 25.0), ObstacleType.GREEN, 0.2)
    plan = Plan.parse(
        {
            "channel_bearing": 0.0,
            "waypoints": [{"name": "n", "x": 0, "y": 50, "role": "buoys"}],
        },
        ORIGIN,
    )
    pilot, boat, track, ticks = run(plan, [green], seconds=240)
    check(
        side_of(track, green.xy, 0.0) < 0,
        "sailing north, a GREEN mark is left to starboard (we pass to its west)",
    )

    # Against the buoyage the sense inverts - the failure that is invisible
    # until you are already through the gate.
    red = Obstacle((0.0, -25.0), ObstacleType.RED, 0.2)
    plan = Plan.parse(
        {
            "channel_bearing": 0.0,   # buoyage still runs north; we sail south
            "waypoints": [{"name": "s", "x": 0, "y": -50, "role": "buoys"}],
        },
        ORIGIN,
    )
    pilot, boat, track, ticks = run(plan, [red], seconds=240, heading=180.0)
    check(
        side_of(track, red.xy, 180.0) < 0,
        "sailing AGAINST the buoyage, the red rule inverts",
    )


def test_cardinal():
    section("cardinal marks - NJORD 10.3")
    # An east cardinal on the line: the boat must end up east of it.
    mark = Obstacle((0.0, 30.0), ObstacleType.EAST, 0.2)
    plan = Plan.parse(
        {"waypoints": [{"name": "n", "x": 0, "y": 60, "role": "buoys"}]}, ORIGIN
    )

    # The lidar cannot tell which cardinal a mark is, so the camera's vote is
    # injected the way `world.absorb_detections` would.
    boat = FakeBoat((0.0, 0.0), 0.0)
    link = FakeLink(boat)
    commander = commander_module.Commander(link, config)
    pilot = Pilot(config, commander)
    pilot.plan = plan
    pilot.start()
    world = WorldModel(config)

    track = []
    ever_committed = False
    now, dt = 1000.0, 0.1
    while now < 1240.0 and not plan.finished:
        state = boat.state(now)
        points, rgb = fake_sweep(boat.xy, boat.heading, [mark])
        clusters = cluster_sweep(points, rgb, source="front_lidar", config=config)
        world.observe(clusters, boat.xy, boat.heading, now, "buoys")
        for entry in world.all():
            if entry.kind in (ObstacleType.CARDINAL, ObstacleType.EAST):
                entry.note_cardinal("east", 0.9)
            if entry.kind == ObstacleType.EAST:
                ever_committed = True
        intent = pilot.tick(state, world, clusters, now)
        commander.send(intent, state, now)
        for message in link.messages:
            boat.apply(message, dt)
        link.messages.clear()
        boat.step(dt)
        track.append(tuple(boat.xy))
        now += dt

    # Checked over the run, not at the end: a mark passed 30 s ago has long
    # since aged out of the world model, which is correct behaviour and would
    # make an end-state assertion test the wrong thing.
    check(ever_committed, "the camera vote commits the mark to EAST")
    closest = min(track, key=lambda p: math.dist(p, mark.xy))
    check(
        closest[0] > mark.xy[0],
        f"an EAST cardinal is passed on its east side (x={closest[0]:.1f})",
    )
    check(min_distance(track, mark.xy) > 1.0, "and it is not touched")

    # Uncommitted: the boat must NOT guess a side. It keeps the planned line.
    world = WorldModel(config)
    from nodes.self_driving.perception.classify import CardinalVote

    vote = CardinalVote(config)
    for _ in range(3):
        vote.add("east", 0.9)
    check(vote.committed is None, "three votes is not enough to commit")
    vote.add("east", 0.9)
    check(vote.committed == "east", "four consistent votes commits")

    flapping = CardinalVote(config)
    for name in ("east", "west", "east", "west", "east", "west"):
        flapping.add(name, 0.9)
    check(
        flapping.committed is None,
        "a detector flapping east/west never commits (margin rule)",
    )


def test_colregs():
    section("COLREG - NJORD 9.2")
    # Crossing from starboard: rule 15, we give way. The Otter runs west across
    # our northbound track.
    otter = Obstacle((30.0, 30.0), ObstacleType.BOAT, 1.0, velocity=(-1.29, 0.0))
    plan = Plan.parse(
        {"waypoints": [{"name": "n", "x": 0, "y": 60, "role": "avoid"}]}, ORIGIN
    )
    pilot, boat, track, ticks = run(
        plan, [otter], seconds=200, moving=[otter]
    )
    check(plan.finished, "the collision-avoidance leg completes")
    # The Otter has moved, so compare against where it was when closest.
    check(
        pilot.telemetry(boat.state(1000.0), None).get("mode") in ("FINISHED", "RUNNING"),
        "the pilot survives a crossing vessel",
    )

    # Rule 17: crossing from PORT, we are stand-on and must hold course.
    otter = Obstacle((-30.0, 30.0), ObstacleType.BOAT, 1.0, velocity=(1.29, 0.0))
    plan = Plan.parse(
        {"waypoints": [{"name": "n", "x": 0, "y": 60, "role": "avoid"}]}, ORIGIN
    )
    pilot, boat, track, ticks = run(plan, [otter], seconds=200, moving=[otter])
    drift = max(abs(p[0]) for p in track)
    check(
        drift < 12.0,
        f"as stand-on vessel the boat mostly holds its line (max {drift:.1f} m off)",
    )

    # And the speed: NJORD 9.2 sets 2 kn and requires it immediately.
    check(
        abs(config.TASK_SPEED_MS - 1.0289) < 0.01,
        "the task speed is 2 knots exactly",
    )


def test_hold():
    section("station keeping - NJORD 9.1 'stop and stay stationary'")
    plan = Plan.parse(
        {
            "waypoints": [
                {"name": "gps4", "x": 0, "y": 30, "role": "hold", "hold_s": 8}
            ]
        },
        ORIGIN,
    )
    pilot, boat, track, ticks = run(plan, [], seconds=200)
    check(plan.finished, "a timed hold completes")
    check(
        math.dist(boat.xy, (0.0, 30.0)) < config.HOLD_TOLERANCE_M + 1.0,
        f"it holds the point ({math.dist(boat.xy, (0.0, 30.0)):.2f} m off)",
    )
    check(abs(boat.speed) < 0.3, f"and it is stationary ({boat.speed:.2f} m/s)")


def test_dock():
    section("docking - NJORD 9.3")
    # Two walls with a 2 m gap, 30 m north. Modelled as rows of discs, which is
    # what a pontoon looks like to a 2-D lidar.
    walls = []
    for offset in range(0, 7):
        walls.append(Obstacle((-1.0 - offset * 0.4, 30.0), ObstacleType.LAND, 0.25))
        walls.append(Obstacle((1.0 + offset * 0.4, 30.0), ObstacleType.LAND, 0.25))

    plan = Plan.parse(
        {
            "waypoints": [
                {"name": "gps7", "x": 0, "y": 22, "role": "dock", "hold_s": 4}
            ]
        },
        ORIGIN,
    )
    phases = []

    def watch(pilot, _world, _boat, _now):
        if pilot.behaviour is not None:
            phase = pilot.behaviour.status.get("phase")
            if phase and (not phases or phases[-1] != phase):
                phases.append(phase)

    pilot, boat, track, ticks = run(plan, walls, seconds=300, watch=watch)
    check(bool(phases), f"the dock behaviour runs its phase machine ({phases})")
    check("align" in phases, f"it squares up before committing ({phases})")
    check("enter" in phases, f"it enters the berth ({phases})")
    check("hold" in phases, f"it holds in the berth - NJORD 9.3 ({phases})")
    check(
        "exit" in phases,
        f"it reverses out - NJORD 9.3 requires this ({phases})",
    )
    check(
        min_distance(track, (0.0, 30.0)) < 12.0,
        "the boat gets to the dock",
    )
    # The berth geometry itself, tested directly - it is the part a simulation
    # of the hull cannot vouch for.
    from nodes.self_driving.perception.cluster import split_by_gap

    points, rgb = fake_sweep((0.0, 24.0), 0.0, walls)
    clusters = cluster_sweep(points, rgb, source="front_lidar", config=config)
    pairs = split_by_gap(clusters, 2.0, config.DOCK_GAP_TOLERANCE_M)
    check(bool(pairs), f"a 2 m berth is found in the sweep ({len(clusters)} clusters)")
    if pairs:
        _left, _right, separation, midpoint = pairs[0]
        check(
            abs(separation - 2.0) <= config.DOCK_GAP_TOLERANCE_M,
            f"the measured gap is {separation:.2f} m",
        )
        check(abs(midpoint[0]) < 0.5, "the berth centre is on the centreline")


def test_safety():
    section("the pilot's safety checks")
    plan = make_plan([(0, 40)])
    boat = FakeBoat()
    link = FakeLink(boat)
    commander = commander_module.Commander(link, config)
    pilot = Pilot(config, commander)
    pilot.plan = plan
    pilot.start()
    world = WorldModel(config)
    now = 1000.0

    def frame(**overrides):
        base = {
            "t": now,
            "origin": ORIGIN,
            "boat": {"position": [0.0, 0.0], "heading_deg": 0.0},
            "gps": {"fix": "RTK_FIXED"},
            "mode": "GUIDED",
            "armed": True,
            "estop": False,
        }
        base.update(overrides)
        return BoatState(base, received_at=now)

    intent = pilot.tick(frame(), world, [], now)
    check(intent.kind != "idle", "a healthy boat is driven")

    intent = pilot.tick(frame(estop=True), world, [], now)
    check("emergency stop" in intent.reason, f"E-stop blocks: {intent.reason}")

    pilot.start()
    intent = pilot.tick(frame(mode="MANUAL"), world, [], now)
    check("pilot has the boat" in intent.reason, f"MANUAL hands over: {intent.reason}")

    pilot.start()
    intent = pilot.tick(frame(armed=False), world, [], now)
    check("disarmed" in intent.reason, f"disarmed blocks: {intent.reason}")

    pilot.start()
    stale = BoatState({"t": now, "origin": ORIGIN, "boat": {"position": [0, 0],
                       "heading_deg": 0}, "mode": "GUIDED", "armed": True},
                      received_at=now - 30.0)
    intent = pilot.tick(stale, world, [], now)
    check(
        "no contact" in intent.reason,
        f"a lost node bus stops the boat (NJORD 7.3): {intent.reason}",
    )

    pilot.start()
    intent = pilot.tick(frame(origin=None), world, [], now)
    check("origin" in intent.reason, f"no grid origin blocks: {intent.reason}")

    pilot.start()
    intent = pilot.tick(None, world, [], now)
    check("no state" in intent.reason, f"no state at all blocks: {intent.reason}")


def test_commander():
    section("the commander")
    boat = FakeBoat()
    link = FakeLink(boat)
    commander = commander_module.Commander(link, config)
    state = boat.state(1000.0)

    commander.send(commander_module.goto((0.0, 50.0), 1.2, "test"), state)
    sent = [m for m in link.messages if m.get("cmd") == "position_target"]
    check(len(sent) == 1, "a goto sends one position target")
    if sent:
        east, north = geo.to_world(sent[0]["lat"], sent[0]["lon"], ORIGIN)
        check(
            abs(east) < 0.05 and abs(north - 50.0) < 0.05,
            "the target converts back to the point asked for",
        )

    link.messages.clear()
    over = commander_module.goto((0.0, 50.0), 99.0, "too fast")
    commander.send(over, state)
    sent = [m for m in link.messages if m.get("cmd") == "position_target"]
    check(
        sent and sent[0]["speed"] <= config.MAX_SPEED_MS,
        "an absurd speed is clamped to the cap",
    )

    link.messages.clear()
    commander.send(commander_module.move(forward=-0.25, reason="astern"), state)
    sent = [m for m in link.messages if m.get("cmd") == "velocity_target"]
    check(sent and sent[0]["vx"] < 0, "reverse is a negative forward velocity")

    # Station keeping must command nothing when already on the spot.
    intent = commander_module.station_keep(state, (0.0, 0.0), 0.0, config, "hold")
    check(intent.kind == "stop", f"on-station holds still ({intent.kind})")
    intent = commander_module.station_keep(state, (0.0, 8.0), 0.0, config, "hold")
    check(intent.kind == "velocity" and intent.vx > 0, "off-station pulls back")


def test_recorder():
    section("the trip recorder")
    from nodes.self_driving.recorder import TripRecorder

    with tempfile.TemporaryDirectory() as directory:
        class Cfg:
            pass

        cfg = Cfg()
        for name in dir(config):
            if name.isupper():
                setattr(cfg, name, getattr(config, name))
        cfg.RECORD_DIR = directory
        cfg.snapshot = config.snapshot

        recorder = TripRecorder(cfg)
        path = recorder.start("unittest")
        check(path is not None, "a recording opens")

        boat = FakeBoat()
        world = WorldModel(config)
        link = FakeLink(boat)
        commander = commander_module.Commander(link, config)
        pilot = Pilot(config, commander)
        pilot.plan = make_plan([(0, 20)])
        now = 1000.0
        for _ in range(30):
            recorder.sample(
                boat.state(now), world, pilot,
                commander_module.goto((0.0, 20.0), 1.0, "test"),
                [{"source": "front_lidar", "points": [[0, 1]]}], now,
            )
            now += 0.5
        recorder.event("phase", phase="enter")
        recorder.stop("done")

        sys.path.insert(0, os.path.join(os.path.dirname(
            os.path.dirname(os.path.abspath(__file__))), "tools"))
        import review_trip

        header, rows, footer = review_trip.load(path)
        check(header.get("label") == "unittest", "the header round trips")
        check(len(rows) >= 20, f"the samples round trip ({len(rows)} rows)")
        check(footer.get("why") == "done", "the footer round trips")
        check("config" in header and header["config"], "the config snapshot is stored")
        events = [r for r in rows if r.get("kind") == "event"]
        check(len(events) == 1, "events round trip")
        text = review_trip.summarise(header, rows, footer)
        check("duration" in text, "the summary renders")
        html = review_trip.to_html(header, rows, footer)
        check("<svg" in html, "the HTML review renders a plot")


def test_real_jetson_sample():
    section("against the captured Jetson feed")
    sample = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "data", "edge_sample.jsonl"
    )
    if not os.path.exists(sample):
        print(f"  skip  no capture at {sample}")
        return

    import json

    from nodes.io_manager.scan import front_scan

    sweeps, dets = 0, 0
    clustered = 0
    with open(sample, "r", encoding="utf-8") as handle:
        for line in handle:
            row = json.loads(line)
            if row.get("kind") == "lidar":
                scan = front_scan(row["lidar"])
                if not scan:
                    continue
                sweeps += 1
                points, rgb = masks.apply(
                    scan["points"], scan.get("rgb"), masks.mask_front(scan["points"])
                )
                clusters = cluster_sweep(
                    points, rgb, source="front_lidar", config=config
                )
                clustered += len(clusters)
            else:
                dets += len(row.get("dets") or [])
    check(sweeps > 0, f"real sweeps parse through front_scan ({sweeps})")
    check(clustered > 0, f"real sweeps cluster ({clustered} clusters over {sweeps})")
    print(f"  info  {dets} real detections in the capture")


def test_memory():
    """Remembering marks: what earns it, what it costs, and how it is cleared.

    The four properties the feature is supposed to have, each checked against the
    failure it exists to prevent:

      * a mark seen properly is kept indefinitely (attempt two starts with
        attempt one's map);
      * its position uncertainty grows while it is unseen and collapses the
        instant it is measured again (a remembered buoy must not claim to be
        somewhere exact);
      * a one-off stray return is NOT kept (the "one random green point, once"
        that must never become a permanent phantom);
      * a vessel is never kept at all (the Otter has moved).
    """
    section("remembering marks between attempts")

    from nodes.self_driving.perception.world import Track
    from nodes.self_driving.survey import Survey

    class _Store:
        """A survey that lives in memory, so the test touches no disk."""

        path = "<memory>"

        def __init__(self):
            self.rows = []

        def entries(self):
            return list(self.rows)

        def write(self, entries, origin=None):
            self.rows = list(entries)
            return True

        def clear(self):
            self.rows = []
            return True

    store = _Store()
    world = WorldModel(config, survey=store)
    world.set_origin(ORIGIN)

    # A buoy the boat looks at properly: 26 sweeps spanning 2.5 s, comfortably
    # past both TRACK_ESTABLISH_HITS and TRACK_ESTABLISH_SPAN_S.
    t = 1000.0
    for i in range(26):
        t = 1000.0 + i * 0.1
        world.observe(
            [_fake_cluster((3.0, 10.0), "green")], (0.0, 0.0), 0.0, t, "buoys"
        )
    marks = world.marks()
    check(len(marks) == 1, f"the buoy is tracked ({len(marks)} mark(s))")
    buoy = marks[0]
    check(buoy.established, "26 sightings over 2.5 s establishes it")

    # A mark is permanent from its SECOND sighting - the whole promise of
    # `MARK_ESTABLISH_HITS`. Once the boat has seen a buoy twice it knows about
    # that buoy for the rest of the run, whether or not it ever sees it again.
    quick = WorldModel(config)
    quick.observe(
        [_fake_cluster((3.0, 10.0), "green")], (0.0, 0.0), 0.0, 5000.0, "buoys"
    )
    check(
        not any(tr.established for tr in quick.all()),
        "one sighting is a measurement, not a memory",
    )
    quick.observe(
        [_fake_cluster((3.0, 10.0), "green")], (0.0, 0.0), 0.0, 5000.1, "buoys"
    )
    seen_twice = quick.all()
    check(
        len(seen_twice) == 1 and seen_twice[0].established,
        "a mark seen twice is established at once - no 12 hits, no 2 s span",
    )
    # ...and it survives an interval that would drop anything else several times
    # over, with nothing further measured.
    quick.observe([], (0.0, 0.0), 0.0, 5000.1 + config.TRACK_DROP_AFTER_S * 20, "buoys")
    check(
        len(quick.all()) == 1,
        "and it is still there two minutes later, unseen the whole time",
    )
    check(
        quick.all()[0].confidence <= seen_twice[0].confidence + 1e-9,
        "the confidence floor holds it up without inflating it - a mark does not "
        "become more certain by not being looked at",
    )

    # The span rule has NOT been removed, it has been scoped: structure still has
    # to earn permanence the slow way, so the same 26-hits-in-300-ms burst that
    # used to be the headline case for it must still fail to establish a wall.
    burst = WorldModel(config)
    for i in range(26):
        burst.observe(
            [_fake_cluster((3.0, 6.0), "white", width=3.0)],
            (0.0, 0.0), 0.0, 5000.0 + i * 0.012, "dock",
        )
    walls = [tr for tr in burst.all() if tr.kind == ObstacleType.LAND]
    check(bool(walls), f"the wall is tracked as structure ({len(walls)})")
    check(
        not any(tr.established for tr in burst.all()),
        "26 hits of structure crammed into 300 ms does NOT establish - a wave "
        "crest still cannot buy permanent memory",
    )
    check(
        abs(buoy.sigma_m - config.TRACK_SIGMA_M) < 1e-6,
        f"freshly seen, it is certain to {buoy.sigma_m:.2f} m",
    )

    # A single stray return, once, never again. The thing that must NOT persist.
    world.observe(
        [_fake_cluster((-6.0, 8.0), "green")], (0.0, 0.0), 0.0, t + 0.1, "buoys"
    )
    strays = [tr for tr in world.all() if not tr.established]
    check(len(strays) == 1, "the stray is tracked at first, like anything else")

    # Now nothing is seen for a long time. Well past TRACK_DROP_AFTER_S.
    far = t + 300.0
    world.observe([], (0.0, 0.0), 0.0, far, "buoys")
    kept = world.all()
    check(
        len(kept) == 1 and kept[0].established,
        f"after 300 s unseen only the established mark survives ({len(kept)})",
    )
    check(
        abs(kept[0].sigma_m - config.TRACK_SIGMA_MAX_M) < 1e-6,
        f"...and its uncertainty is at the {config.TRACK_SIGMA_MAX_M:.0f} m ceiling",
    )
    check(
        kept[0].confidence >= config.TRACK_ESTABLISH_FLOOR - 1e-9,
        "...held up by the established confidence floor rather than decayed away",
    )

    # Seeing it again collapses the uncertainty, and does NOT make a second track.
    before = len(world.all())
    world.observe(
        [_fake_cluster((3.4, 10.3), "green")], (0.0, 0.0), 0.0, far + 0.1, "buoys"
    )
    check(
        len(world.all()) == before,
        "re-acquiring a remembered mark re-uses its track instead of duplicating it",
    )
    check(
        abs(world.all()[0].sigma_m - config.TRACK_SIGMA_M) < 1e-6,
        "...and its uncertainty collapses back to sensor accuracy",
    )

    # A vessel is never established, however long it is watched: it has moved.
    vessels = WorldModel(config)
    for i in range(40):
        vessels.observe(
            [_fake_cluster((0.0, 12.0), "white", width=2.0)],
            (0.0, 0.0), 0.0, 2000.0 + i * 0.1, "colregs",
        )
    boats = [tr for tr in vessels.all() if tr.kind == ObstacleType.BOAT]
    check(bool(boats), "the Otter is tracked")
    check(
        not any(tr.established for tr in boats),
        "a vessel is never established - it will have moved by attempt two",
    )

    # The survey round trips through lat/lon, which is the whole point of it
    # being in degrees: the grid origin does not survive a reboot.
    world.save_survey()
    check(len(store.rows) == 1, f"the established mark is written ({len(store.rows)})")
    check(
        "lat" in store.rows[0] and "lon" in store.rows[0],
        "...as lat/lon, not grid metres",
    )
    restored = WorldModel(config, survey=store)
    restored.set_origin(ORIGIN)
    back = restored.all()
    check(len(back) == 1, f"it comes back on the next run ({len(back)})")
    if back:
        moved = geo.distance(back[0].pos, world.all()[0].pos)
        check(moved < 0.5, f"...within {moved:.2f} m of where it was left")
        check(back[0].established, "...still established")
        # Less certain than a mark under the lidar right now, and far more
        # certain than one that merely drifted out of view - the two are
        # different kinds of not-knowing and `Track.refresh` keeps them apart.
        # This one was measured deliberately and has been on its mooring since.
        check(
            abs(back[0].sigma_m - config.SURVEY_SIGMA_M) < 1e-6,
            f"...carrying the survey's own uncertainty, {back[0].sigma_m:.2f} m, "
            f"not the {config.TRACK_SIGMA_MAX_M:.0f} m of a mark nobody can "
            f"account for",
        )
        check(
            config.TRACK_SIGMA_M < back[0].sigma_m < config.TRACK_SIGMA_MAX_M,
            "...which sits between a fresh measurement and a lost one",
        )
        # The number that made this worth changing: clearance is the static
        # figure plus this, and the Monday course's legs are 10-17 m long.
        room = config.BUOY_CLEARANCE_M + back[0].sigma_m
        check(
            room * 2.0 < 10.0,
            f"a remembered mark claims {room:.1f} m, so two of them still leave "
            f"a gap on a 10 m leg ({room * 2.0:.1f} m of the 10)",
        )

    # A survey written against one origin must land in the right place when the
    # grid has been re-zeroed somewhere else - the reboot case in survey.py.
    shifted = WorldModel(config, survey=store)
    shifted.set_origin({"lat": ORIGIN["lat"] + 0.0009, "lon": ORIGIN["lon"]})
    if shifted.all() and back:
        offset = shifted.all()[0].pos[1] - back[0].pos[1]
        check(
            abs(offset + 100.0) < 2.0,
            f"a moved origin shifts the restored mark by the right amount "
            f"({offset:.1f} m for a 100 m move)",
        )

    # Deleting one object, which is what the dashboard's per-object button does.
    ok, message = restored.forget_track(back[0].id)
    check(ok, f"one object can be deleted: {message}")
    check(not restored.all(), "...and it is gone from the model")
    check(not store.rows, "...and from the survey, so it stays gone after a restart")

    # ...and it does not come straight back off the next sweep.
    restored.observe(
        [_fake_cluster((3.0, 10.0), "green")], (0.0, 0.0), 0.0, far + 1.0, "buoys"
    )
    check(
        not restored.all(),
        "a deleted object is not re-created by the very next sweep",
    )

    # Clear-all takes the survey with it, or it all returns on the next start.
    world.forget("unit test")
    check(not world.all() and not store.rows, "clear-all empties model and survey")


def test_speed_limit():
    """The 5 knot vessel limit, and careful mode's 1 knot, at every seam.

    Written as an attack rather than as a demonstration: each check is a way
    somebody could plausibly get the boat over the limit, and asserts that they
    cannot. The limit is on the vessel, so "the default happens to be lower" is
    not a defence - every one of these sets the tuning as high as it will go.
    """
    section("the 5 knot speed limit")

    from nodes.self_driving import commander as cm

    limit = config.SPEED_LIMIT_MS
    check(
        abs(config.SPEED_LIMIT_KNOTS - 5.0) < 1e-9,
        f"the limit is {config.SPEED_LIMIT_KNOTS} knots ({limit:.4f} m/s)",
    )

    # Every configured speed is already inside it.
    over = [
        name for name in dir(config)
        if name.endswith(("_SPEED_MS", "_MAX_MS")) and name.isupper()
        and isinstance(getattr(config, name), float)
        and getattr(config, name) > limit + 1e-9
    ]
    check(not over, f"no configured speed exceeds the limit (offenders: {over})")

    class _Link:
        def __init__(self):
            self.sent = []

        def control(self, **fields):
            self.sent.append(fields)

    def commanded(intent, careful=False):
        """The speed actually put on the wire for one intent, m/s."""
        link = _Link()
        commander = cm.Commander(link, config)
        commander.careful = careful
        state = BoatState(
            {"origin": ORIGIN, "boat": {"position": [0.0, 0.0], "heading_deg": 0.0}}
        )
        commander.send(intent, state)
        speeds = []
        for message in link.sent:
            if message.get("cmd") == "position_target":
                speeds.append(float(message.get("speed") or 0.0))
            elif message.get("cmd") == "velocity_target":
                speeds.append(
                    math.hypot(
                        float(message.get("vx") or 0.0),
                        float(message.get("vy") or 0.0),
                    )
                )
        return max(speeds) if speeds else 0.0

    # A behaviour asking for an absurd speed.
    fast = commanded(cm.goto((100.0, 0.0), 99.0, "attack"))
    check(fast <= limit + 1e-6, f"a goto at 99 m/s goes out at {fast:.3f} m/s")

    # ...and in body-velocity form, forwards and astern.
    fwd = commanded(cm.move(forward=99.0, reason="attack"))
    check(fwd <= limit + 1e-6, f"a 99 m/s forward velocity goes out at {fwd:.3f} m/s")
    astern = commanded(cm.move(forward=-99.0, reason="attack"))
    check(
        astern <= limit + 1e-6,
        f"the limit is symmetric - 99 m/s astern goes out at {astern:.3f} m/s",
    )

    # The resultant, which is the one a per-axis clamp misses: forward at the
    # ceiling plus the lateral thruster is over the limit while each axis is
    # legal on its own.
    both = commanded(
        cm.move(forward=limit, starboard=config.LATERAL_MAX_MS, reason="attack")
    )
    naive = math.hypot(limit, config.LATERAL_MAX_MS)
    check(
        naive > limit,
        f"per-axis clamping would allow {naive / config.KNOT_MS:.2f} kn - so the "
        f"resultant test is not hypothetical",
    )
    check(
        both <= limit + 1e-6,
        f"forward + lateral resolves to {both:.3f} m/s "
        f"({both / config.KNOT_MS:.2f} kn), inside the limit",
    )

    # A NaN speed is a stop, not an undefined command to ArduPilot.
    nan = commanded(cm.move(forward=float("nan"), reason="attack"))
    check(nan == 0.0, f"a NaN velocity goes out as {nan:.3f} m/s, not as NaN")

    # A plan cannot ask for more than the limit, and is refused in words.
    try:
        Plan.parse(
            {
                "name": "fast",
                "waypoints": [
                    {"name": "1", "lat": ORIGIN["lat"], "lon": ORIGIN["lon"],
                     "role": "transit", "speed": 3.0}
                ],
            },
            ORIGIN,
        )
        check(False, "a 3.0 m/s waypoint should have been refused")
    except PlanError as exc:
        check(
            "knot" in str(exc).lower(),
            f"a 3.0 m/s waypoint is refused in the operator's words: {exc}",
        )

    ok_plan = Plan.parse(
        {
            "name": "legal",
            "waypoints": [
                {"name": "1", "lat": ORIGIN["lat"], "lon": ORIGIN["lon"],
                 "role": "transit", "speed": 2.5}
            ],
        },
        ORIGIN,
    )
    check(ok_plan.waypoints[0].speed == 2.5, "a 2.5 m/s waypoint is still accepted")

    # ---------------------------------------------------------- careful mode
    section("careful mode (1 knot)")

    careful = config.CAREFUL_SPEED_MS
    check(
        abs(config.CAREFUL_SPEED_KNOTS - 1.0) < 1e-9,
        f"careful mode is {config.CAREFUL_SPEED_KNOTS} knot ({careful:.4f} m/s)",
    )
    check(careful < limit, "careful mode is slower than the vessel limit")

    slow = commanded(cm.goto((100.0, 0.0), 99.0, "attack"), careful=True)
    check(
        slow <= careful + 1e-6,
        f"in careful mode a goto at 99 m/s goes out at {slow:.3f} m/s "
        f"({slow / config.KNOT_MS:.2f} kn)",
    )
    slow_vel = commanded(
        cm.move(forward=99.0, starboard=99.0, reason="attack"), careful=True
    )
    check(
        slow_vel <= careful + 1e-6,
        f"...and a body velocity at {slow_vel:.3f} m/s",
    )

    # Even a waypoint that legally asks for 2.5 m/s is held to 1 kn.
    link = _Link()
    commander = cm.Commander(link, config)
    ok, message = commander.set_careful(True)
    check(ok and commander.careful, f"careful mode switches on: {message}")
    ctx_speed = None
    from nodes.self_driving.behaviours.base import Context

    ctx = Context(
        state=None, world=None, plan=None, config=config, now=0.0,
        waypoint=ok_plan.waypoints[0], leg=None, task="transit",
        ceiling=commander.ceiling,
    )
    ctx_speed = ctx.speed_limit(config.CRUISE_SPEED_MS)
    check(
        ctx_speed <= careful + 1e-9,
        f"a behaviour PLANS at {ctx_speed:.3f} m/s, rather than asking for more "
        f"and being clamped on the way out",
    )

    ok, message = commander.set_careful(False)
    check(
        ok and not commander.careful and commander.ceiling > careful,
        f"careful mode switches off again: {message}",
    )

    # Careful mode can only ever slow the boat, never speed it up.
    check(
        cm.CAREFUL_CEILING_MS <= cm.CEILING_MS,
        "careful mode's ceiling can never exceed the ordinary one",
    )


# ------------------------------------------------ the two attempts, and the course

def real_course():
    """Monday's Task 1 course out of `plans/task1.json`, in local metres.

    Translated so GPS point 1 sits on the test origin, which is a 2 km shift and
    changes no geometry that matters at this latitude - the point is to fly the
    *actual* handout rather than a tidy invention, because the thing that makes
    this course hard is not something anybody would have thought to invent: legs
    of 4 to 17 m with three corners over 100 degrees.

    Returns `(plan_payload, points)`, points being `[(east, north), ...]`.
    """
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    with open(os.path.join(here, "plans", "task1.json"), encoding="utf-8") as handle:
        course = json.load(handle)

    first = course["waypoints"][0]
    metres_per_lat = 111320.0
    metres_per_lon = 111320.0 * math.cos(math.radians(first["lat"]))
    points, waypoints = [], []
    for entry in course["waypoints"]:
        east = (entry["lon"] - first["lon"]) * metres_per_lon
        north = (entry["lat"] - first["lat"]) * metres_per_lat
        points.append((east, north))
        moved = dict(entry)
        moved.pop("lat", None)
        moved.pop("lon", None)
        moved["x"] = east
        moved["y"] = north
        waypoints.append(moved)
    return (
        dict(course, waypoints=waypoints),
        points,
    )


def seconds_to_finish(track, points, radius, dt=0.1):
    """How long the track took to reach the last point. `None` if it never did.

    Not the tick count: the last waypoint on Task 1 is `hold` with `hold_s` 0 -
    "stop at GPS point 4 and stay stationary" (NJORD §9.1) - so the plan is never
    *finished* and a run always uses its whole time budget. The figure the task
    time multiplier is scored on is when the boat got there.
    """
    for index, position in enumerate(track):
        if math.dist(position, points[-1]) <= radius:
            return index * dt
    return None


def worst_miss(track, points):
    """`(index, metres)` of the waypoint the track came closest to missing.

    The measure the jury actually applies: the boat is scored on *passing* the
    waypoints, and `has_arrived`'s passing-plane test will happily retire one the
    boat swept 5 m wide of. So this asks the track, not the plan.
    """
    worst = (None, 0.0)
    for index, point in enumerate(points):
        closest = min_distance(track, point)
        if closest > worst[1]:
            worst = (index, closest)
    return worst


def test_profiles():
    section("run profiles - the slow attempt and the fast one")

    from nodes.self_driving import profiles

    limit = config.SPEED_LIMIT_MS
    for name, profile in sorted(profiles.PROFILES.items()):
        check(
            profile.ceiling_ms <= limit + 1e-9,
            f"the {name} profile's ceiling is inside the 5 kn vessel limit "
            f"({profile.ceiling_kn:.2f} kn)",
        )
        check(
            profile.cruise_ms <= profile.ceiling_ms + 1e-9
            and profile.caution_ms <= profile.ceiling_ms + 1e-9,
            f"...and the {name} profile never plans above its own ceiling",
        )

    fast = profiles.PROFILES[profiles.FAST]
    check(
        abs(fast.ceiling_ms - limit) < 1e-9,
        f"the fast profile goes all the way to the limit and no further "
        f"({fast.ceiling_kn:.2f} kn)",
    )
    survey = profiles.PROFILES[profiles.SURVEY]
    check(
        abs(survey.ceiling_ms - config.CAREFUL_SPEED_MS) < 1e-9,
        f"the survey profile is careful mode's 1 kn ({survey.ceiling_ms:.2f} m/s)",
    )

    # The property Task 2 depends on, stated as a test rather than as a comment.
    # A gate is a red/green pair 5 m apart (NJORD 9.2), so the boat has 2.5 m to
    # play with either side of the centreline; a speed term on the normal profile
    # would eat it and the boat would refuse a gate it is meant to drive through.
    for name in (profiles.NORMAL, profiles.SURVEY):
        check(
            profiles.PROFILES[name].clearance_per_ms == 0.0,
            f"the {name} profile adds no speed term to clearance, so a 5 m gate "
            f"stays passable",
        )
    gate_half = 5.0 / 2.0
    normal_room = config.BUOY_CLEARANCE_M + config.TRACK_SIGMA_M
    check(
        normal_room < gate_half,
        f"a freshly seen gate buoy claims {normal_room:.2f} m of the {gate_half:.1f} m "
        f"half-gate",
    )
    check(fast.clearance_per_ms > 0.0, "the fast profile does add one")

    # What that term is worth where it is switched on.
    at_speed = min(
        config.CLEARANCE_SPEED_MAX_M, fast.clearance_per_ms * fast.ceiling_ms
    )
    check(
        at_speed > 1.5,
        f"at full speed the fast profile buys {at_speed:.1f} m of extra water "
        f"round every mark",
    )

    # Selection, and refusing to select nonsense.
    mode = profiles.RunMode(config)
    check(mode.profile.name == profiles.NORMAL, "a fresh node is in the normal profile")
    check(not mode.alternation, "...with the alternation prior off")
    ok, message = mode.set_profile("fast")
    check(ok and mode.profile.name == profiles.FAST, f"fast selects: {message}")
    ok, message = mode.set_profile("ludicrous")
    check(
        not ok and "fast" in message and mode.profile.name == profiles.FAST,
        f"an unknown profile is refused and changes nothing: {message}",
    )
    ok, _message = mode.set_profile("survey")
    check(ok and mode.careful, "the survey profile IS careful mode, not a rival to it")

    # ...and that the ceiling is real on the wire, not just in the object.
    class _Link:
        def __init__(self):
            self.sent = []

        def control(self, **fields):
            self.sent.append(fields)

    def wire_speed(profile_name):
        link = _Link()
        commander = commander_module.Commander(link, config)
        commander.set_profile(profile_name)
        state = BoatState(
            {"origin": ORIGIN, "boat": {"position": [0.0, 0.0], "heading_deg": 0.0}}
        )
        commander.send(commander_module.goto((100.0, 0.0), 99.0, "attack"), state)
        return max(
            float(m.get("speed") or 0.0)
            for m in link.sent
            if m.get("cmd") == "position_target"
        )

    check(
        abs(wire_speed("fast") - limit) < 1e-6,
        f"in the fast profile a 99 m/s request goes out at {wire_speed('fast'):.3f} m/s "
        f"- the vessel limit, not more",
    )
    check(
        wire_speed("normal") <= config.MAX_SPEED_MS + 1e-6,
        "in the normal profile it is still held to the tuned 1.6 m/s",
    )
    check(
        wire_speed("survey") <= config.CAREFUL_SPEED_MS + 1e-6,
        "and in the survey profile to 1 kn",
    )


def test_corner_speed():
    section("how fast a corner allows")

    from nodes.self_driving.behaviours.base import Context, corner_speed_limit
    from nodes.self_driving import profiles

    payload, points = real_course()

    def limit_at(index, distance_out, profile=profiles.FAST):
        """The speed allowed `distance_out` metres before waypoint `index`."""
        plan = Plan.parse(payload, ORIGIN)
        plan.index = index
        start, end = points[index - 1], points[index]
        bearing = geo.bearing_to(start, end)
        boat = geo.offset_point(end, bearing + 180.0, distance_out)
        state = BoatState(
            {
                "origin": ORIGIN,
                "boat": {"position": list(boat), "heading_deg": bearing},
            }
        )
        run_mode = profiles.RunMode(config)
        run_mode.set_profile(profile)
        ctx = Context(
            state=state, world=None, plan=plan, config=config, now=0.0,
            waypoint=plan.current, leg=(start, end), task="buoys",
            ceiling=run_mode.profile.ceiling_ms, run=run_mode,
        )
        return corner_speed_limit(ctx, ctx.cruise_speed)

    # Waypoint 9 is `3.2`, where the course turns 123 degrees - the tightest
    # corner on it. Probed from just short of the mark, which is where the
    # limiter has to have finished its work.
    tight, note = limit_at(9, 0.3)
    check(
        tight < config.FAST_CRUISE_SPEED_MS * 0.8,
        f"the 123 deg corner at 3.2 holds the fast profile down to "
        f"{tight:.2f} m/s ({tight / config.KNOT_MS:.1f} kn): {note}",
    )
    check("deg turn" in note, f"...and says why, in words: {note}")

    # The same corner from far enough back that the boat can still brake into it.
    far, _note = limit_at(9, 11.0)
    check(
        far > tight,
        f"from 11 m out it may still run at {far:.2f} m/s and brake late, rather "
        f"than crawling the whole leg",
    )

    # A gentle corner does not limit at all. Waypoint 3 (`1.2` -> `1.3`) turns 13
    # degrees, which is a kink.
    gentle, note = limit_at(3, 2.0)
    check(
        abs(gentle - config.FAST_CRUISE_SPEED_MS) < 1e-9 and note == "",
        f"a 13 deg kink is not a corner and is not slowed ({gentle:.2f} m/s)",
    )

    # The survey profile is already below every corner's limit, so the limiter
    # never fires - the slow attempt is not made slower still.
    slow, note = limit_at(10, 2.0, profile=profiles.SURVEY)
    check(
        abs(slow - config.CAREFUL_SPEED_MS) < 1e-9,
        f"at 1 kn the corner limiter has nothing to do ({slow:.2f} m/s)",
    )

    # A plan that doubles back on itself is a 180 degree turn, and `sec(90)` is
    # where a naive version divides by zero.
    reversing = Plan.parse(
        {
            "waypoints": [
                {"name": "out", "x": 0.0, "y": 20.0, "role": "transit"},
                {"name": "back", "x": 0.0, "y": 0.0, "role": "transit"},
            ]
        },
        ORIGIN,
    )
    state = BoatState(
        {"origin": ORIGIN, "boat": {"position": [0.0, 20.0], "heading_deg": 0.0}}
    )
    ctx = Context(
        state=state, world=None, plan=reversing, config=config, now=0.0,
        waypoint=reversing.current, leg=((0.0, 0.0), (0.0, 20.0)), task="transit",
    )
    about_turn, note = corner_speed_limit(ctx, 2.5)
    check(
        about_turn <= config.CORNER_MIN_SPEED_MS + 0.05,
        f"a 180 deg about-turn limits to the floor rather than dividing by zero "
        f"({about_turn:.2f} m/s): {note}",
    )


def test_fast_course():
    """Monday's actual course, flown fast. The question the whole plan rests on.

    The course is a slalom - three corners over 100 degrees on legs of 10-17 m -
    and the failure this is written to catch is the specific one that a fast
    attempt invites: the boat holds its speed into a corner, turns as hard as it
    can, and sweeps past the waypoint several metres wide. It finishes the plan,
    it looks confident, and it has missed the marks it is scored on passing.
    """
    section("the real Task 1 course, flown fast")

    from nodes.self_driving.behaviours import transit as transit_module
    from nodes.self_driving import profiles

    payload, points = real_course()
    radius = config.WAYPOINT_RADIUS_M

    _pilot, _boat, slow_track, _ticks = run(
        Plan.parse(payload, ORIGIN), [], seconds=600.0, profile=profiles.SURVEY
    )
    index, missed = worst_miss(slow_track, points)
    check(
        missed <= radius,
        f"at 1 kn every waypoint is passed inside the {radius:.0f} m radius "
        f"(worst was #{index} at {missed:.1f} m)",
    )
    slow_s = seconds_to_finish(slow_track, points, config.ARRIVAL_RADIUS_M)
    check(slow_s is not None, f"...and it reaches GPS 4, in {slow_s} s")

    _pilot, _boat, fast_track, _ticks = run(
        Plan.parse(payload, ORIGIN), [], seconds=600.0, profile=profiles.FAST
    )
    index, missed = worst_miss(fast_track, points)
    check(
        missed <= radius,
        f"and in the fast profile too (worst was #{index} at {missed:.1f} m)",
    )
    fast_s = seconds_to_finish(fast_track, points, config.ARRIVAL_RADIUS_M)
    check(
        fast_s is not None and slow_s is not None and fast_s < slow_s,
        f"the fast attempt is actually faster: {fast_s:.0f} s against "
        f"{slow_s:.0f} s to GPS 4, a "
        f"{1.0 - (fast_s / slow_s if slow_s else 1.0):.0%} saving",
    )

    # What the pacing is actually worth, and it is not what it looks like.
    #
    # With the hull as capable as `TURN_LATERAL_ACCEL_MS2` assumes, pure pursuit's
    # own lookahead clamp already pulls the aim in on the run-up to a mark and the
    # pacing barely changes the trace - so a straight paced/unpaced comparison at
    # the nominal hull proves nothing either way. The pacing is *margin*, and
    # margin only shows up when something is worse than assumed.
    #
    # So the control degrades the simulated hull to below what the config believes,
    # which is the failure actually worth insuring against: nobody has measured
    # this boat's lateral acceleration yet, the number in `config.py` is a guess,
    # and a guess that turns out 25 % optimistic is entirely likely.
    #
    # None of this is evidence about the real boat. It is evidence that the pacing
    # buys margin against the parameter being wrong, which is the only claim the
    # simulation can support.
    from nodes.self_driving.behaviours import base as base_module
    from nodes.self_driving.behaviours import buoys as buoys_module

    def fly_with_hull(lateral_accel, paced):
        was = FakeBoat.MAX_LATERAL_ACCEL
        FakeBoat.MAX_LATERAL_ACCEL = lateral_accel
        saved = (
            transit_module.corner_speed_limit,
            buoys_module.corner_speed_limit,
            base_module.heading_speed_limit,
        )
        if not paced:
            unpaced = lambda ctx, speed, *rest: (speed, "")  # noqa: E731
            transit_module.corner_speed_limit = unpaced
            buoys_module.corner_speed_limit = unpaced
            base_module.heading_speed_limit = unpaced
        eased = [0]

        def count(pilot, _world, _boat, _now):
            if "deg turn at" in pilot.reason or "off the aim" in pilot.reason:
                eased[0] += 1

        try:
            _p, _b, flown, _t = run(
                Plan.parse(payload, ORIGIN), [], seconds=600.0,
                profile=profiles.FAST, watch=count,
            )
        finally:
            (
                transit_module.corner_speed_limit,
                buoys_module.corner_speed_limit,
                base_module.heading_speed_limit,
            ) = saved
            FakeBoat.MAX_LATERAL_ACCEL = was
        return flown, eased[0]

    _flown, eased = fly_with_hull(FakeBoat.MAX_LATERAL_ACCEL, True)
    check(
        eased > 0,
        f"the pacing is live on this course, not dead code - it eased the boat on "
        f"{eased} ticks of the fast run",
    )

    weak = config.TURN_LATERAL_ACCEL_MS2 * 0.75
    limp_paced, _eased = fly_with_hull(weak, True)
    limp_loose, _eased = fly_with_hull(weak, False)
    paced_s = seconds_to_finish(limp_paced, points, config.ARRIVAL_RADIUS_M)
    loose_s = seconds_to_finish(limp_loose, points, config.ARRIVAL_RADIUS_M)
    check(
        paced_s is not None,
        f"a hull 25 % less capable than assumed ({weak:.2f} m/s^2) still reaches "
        f"GPS 4 when the boat paces itself, in {paced_s} s",
    )
    check(
        loose_s is None or loose_s > (paced_s or 0.0),
        f"...and does not, unpaced (reached GPS 4 at {loose_s} s)",
    )
    _index, limp_missed = worst_miss(limp_paced, points)
    _index, loose_missed = worst_miss(limp_loose, points)
    check(
        limp_missed < loose_missed,
        f"and it holds a tighter line doing it: {limp_missed:.1f} m worst miss "
        f"against {loose_missed:.1f} m",
    )


def test_fast_clearance():
    section("clearance at speed")

    from nodes.self_driving import profiles

    # One buoy sitting beside a straight 40 m leg. The only thing that changes
    # between the two runs is the profile.
    def pass_distance(profile):
        mark = Obstacle((2.2, 20.0), ObstacleType.RED, 0.2)
        plan = make_plan([(0.0, 40.0)], role="buoys")
        _pilot, _boat, track, _ticks = run(
            plan, [mark], seconds=300.0, profile=profile
        )
        return min_distance(track, mark.xy)

    careful = pass_distance(profiles.SURVEY)
    quick = pass_distance(profiles.FAST)
    check(
        careful > config.BUOY_CLEARANCE_M * 0.75,
        f"at survey speed the mark is cleared by {careful:.1f} m",
    )
    check(
        quick > careful + 0.5,
        f"at speed the same mark is given {quick:.1f} m instead of {careful:.1f} m "
        f"- clearance is a time budget, and speed spends it",
    )


def test_alternation():
    """The optional prior: what it says, when it refuses, and what it never does."""
    section("the cardinal alternation prior (optional, off by default)")

    from nodes.self_driving.behaviours import alternation
    from nodes.self_driving.behaviours.base import Context
    from nodes.self_driving import profiles

    class _Mark:
        def __init__(self, mark_id, kind, pos):
            self.id = mark_id
            self.kind = kind
            self.pos = pos

    class _World:
        def __init__(self, marks):
            self._marks = marks

        def marks(self):
            return list(self._marks)

    def context(marks, leg=((0.0, 0.0), (0.0, 60.0)), on=True):
        state = BoatState(
            {"origin": ORIGIN, "boat": {"position": [0.0, 0.0], "heading_deg": 0.0}}
        )
        plan = make_plan([(leg[1][0], leg[1][1])], role="buoys")
        mode = profiles.RunMode(config)
        mode.alternation = on
        return Context(
            state=state, world=_World(marks), plan=plan, config=config, now=0.0,
            waypoint=plan.current, leg=leg, task="buoys", run=mode,
        )

    # A leg running due north. A red mark 10 m up it: sailing with the buoyage,
    # red is kept to port, so the mark sits to port of the line and the boat goes
    # up its starboard side. The next mark in the run should therefore be the one
    # that pushes the other way - which on a northbound leg is a WEST cardinal
    # (safe water to its west puts the mark to starboard of us).
    red = _Mark(1, ObstacleType.RED, (0.0, 10.0))
    unknown = _Mark(2, ObstacleType.CARDINAL, (0.0, 25.0))
    ctx = context([red, unknown])
    guess, why = alternation.resolve(ctx, unknown, outbound=True)
    check(
        guess == ObstacleType.WEST,
        f"a red mark before it implies the next cardinal is WEST, got "
        f"{guess.name if guess else None}: {why}",
    )
    check("#1" in why, f"...and it names the mark it reasoned from: {why}")

    # Switch the lateral sense and the answer must switch with it. This is the
    # sign error the whole file is exposed to, so it gets its own check.
    flipped, _why = alternation.resolve(ctx, unknown, outbound=False)
    check(
        flipped == ObstacleType.EAST,
        f"sailing against the buoyage the same pair implies EAST, got "
        f"{flipped.name if flipped else None}",
    )

    # Off by default is not a comment, it is behaviour.
    quiet = context([red, unknown], on=False)
    silent, _why = alternation.resolve(quiet, unknown, outbound=True)
    check(silent is None, "with the prior switched off it says nothing at all")

    # Nothing to reason from.
    alone = context([unknown])
    nothing, why = alternation.resolve(alone, unknown, outbound=True)
    check(
        nothing is None and "says nothing" in why,
        f"with no settled mark before it, it declines and explains: {why}",
    )

    # A mark too far back is not "the previous mark in the run".
    distant = _Mark(3, ObstacleType.RED, (0.0, -60.0))
    far = context([distant, unknown])
    none_yet, _why = alternation.resolve(far, unknown, outbound=True)
    check(
        none_yet is None,
        f"a mark {config.ALTERNATION_MAX_GAP_M:.0f} m back is a different part of "
        f"the course and is not reasoned from",
    )

    # The geometric refusal: on a leg running north, north and south cardinals
    # say nothing about which way to go round, and the prior must not invent an
    # answer. Rig it so the only candidates would be N/S by asking directly.
    kind, margin = alternation.best_cardinal(+1.0, 0.0, config)
    check(
        kind == ObstacleType.WEST and margin > 0.99,
        f"on a northbound leg 'keep the mark to starboard' means WEST, squarely "
        f"({margin:.2f})",
    )
    side, margin = alternation.cardinal_side(
        ObstacleType.NORTH, 0.0, config.ALTERNATION_MIN_SIN
    )
    check(
        side is None and margin < 1e-9,
        "...and a NORTH cardinal on a northbound leg is refused, not guessed",
    )

    # It never overrides the camera. A committed EAST where the pattern wanted
    # WEST is obeyed, and the disagreement is reported instead.
    committed = _Mark(2, ObstacleType.EAST, (0.0, 25.0))
    ctx = context([red, committed])
    clash = alternation.disagreement(ctx, committed, outbound=True)
    check(
        "obeying the camera" in clash and "west" in clash,
        f"a committed vote that contradicts the pattern is reported, not "
        f"overruled: {clash}",
    )
    agreed = _Mark(2, ObstacleType.WEST, (0.0, 25.0))
    check(
        alternation.disagreement(context([red, agreed]), agreed, outbound=True) == "",
        "...and agreement is not worth a sentence",
    )

    # End to end: an uncommitted cardinal beside a red mark, flown. With the
    # prior off the boat holds the planned line; with it on it goes round.
    def fly(on):
        marks = [
            Obstacle((0.0, 12.0), ObstacleType.RED, 0.2),
            Obstacle((0.0, 26.0), ObstacleType.CARDINAL, 0.2),
        ]
        plan = Plan.parse(
            {
                "channel_bearing": 0,
                "waypoints": [{"name": "n", "x": 0.0, "y": 50.0, "role": "buoys"}],
            },
            ORIGIN,
        )
        _pilot, _boat, track, _ticks = run(
            plan, marks, seconds=300.0, alternation=on
        )
        return track, marks[1].xy

    track_on, cardinal_xy = fly(True)
    track_off, _xy = fly(False)
    closest_on = min(track_on, key=lambda p: math.dist(p, cardinal_xy))
    closest_off = min(track_off, key=lambda p: math.dist(p, cardinal_xy))
    check(
        closest_on[0] < closest_off[0],
        f"with the prior on the boat commits to the west side of the unresolved "
        f"cardinal (x={closest_on[0]:.1f}) rather than holding the line "
        f"(x={closest_off[0]:.1f})",
    )
    check(
        min_distance(track_on, cardinal_xy) > 1.0,
        "and it does not touch it either way",
    )


def _fake_cluster(centre, colour, width=0.4):
    """One `Cluster` of the given colour at the given boat-frame point."""
    from nodes.self_driving.perception.cluster import Cluster

    rgb = {
        "green": (20, 200, 60),
        "red": (210, 30, 30),
        "white": (230, 230, 230),
    }[colour]
    n = 6
    range_m = math.hypot(*centre)
    return Cluster(
        centre=(float(centre[0]), float(centre[1])),
        nearest=(float(centre[0]), float(centre[1])),
        range_m=range_m,
        bearing_deg=math.degrees(math.atan2(centre[0], centre[1])),
        width_m=width,
        n=n,
        rgb=np.tile(np.array(rgb, dtype=np.int64), (n, 1)),
        source="front_lidar",
    )


def main():
    start = time.time()
    for test in (
        test_geo,
        test_colour,
        test_loose_marks,
        test_colour_age_weighting,
        test_masks,
        test_plan,
        test_transit,
        test_avoidance,
        test_buoy_rules,
        test_cardinal,
        test_colregs,
        test_hold,
        test_dock,
        test_safety,
        test_commander,
        test_recorder,
        test_memory,
        test_speed_limit,
        test_profiles,
        test_corner_speed,
        test_fast_course,
        test_fast_clearance,
        test_alternation,
        test_real_jetson_sample,
    ):
        try:
            test()
        except Exception as exc:  # noqa: BLE001 - report and carry on
            import traceback

            print(f"  ERROR in {test.__name__}: {exc}")
            traceback.print_exc()
            FAILURES.append(f"{test.__name__} raised {exc!r}")

    print(f"\n{'=' * 60}")
    if FAILURES:
        print(f"{len(FAILURES)} FAILURE(S) in {time.time() - start:.1f}s:")
        for failure in FAILURES:
            print(f"  - {failure}")
        return 1
    print(f"all checks passed in {time.time() - start:.1f}s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
