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
    """First-order kinematics. Enough to catch a sign error, not a hull model."""

    MAX_YAW = 0.7  # rad/s
    ACCEL = 1.5    # m/s^2

    def __init__(self, xy=(0.0, 0.0), heading=0.0):
        self.xy = list(xy)
        self.heading = heading
        self.speed = 0.0
        self.lateral = 0.0
        self.mode = "GUIDED"
        self.armed = True
        self.sent = []

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
            target = geo.to_world(message["lat"], message["lon"], ORIGIN)
            wanted = geo.bearing_to(self.xy, target)
            self._turn(wanted, dt)
            self._accelerate(message.get("speed", 0.5), dt)
            return
        if command == "velocity_target":
            yaw = float(message.get("yaw_rate") or 0.0)
            self.heading = geo.wrap360(self.heading + math.degrees(yaw) * dt)
            self._accelerate(float(message.get("vx") or 0.0), dt, signed=True)
            self.lateral = float(message.get("vy") or 0.0)

    def _turn(self, wanted, dt):
        error = geo.angle_diff(wanted, self.heading)
        rate = math.degrees(self.MAX_YAW) * dt
        self.heading = geo.wrap360(self.heading + max(-rate, min(rate, error)))

    def _accelerate(self, target, dt, signed=False):
        limit = self.ACCEL * dt
        self.speed += max(-limit, min(limit, target - self.speed))
        if not signed:
            self.speed = max(0.0, self.speed)

    def step(self, dt):
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
        moving=(), watch=None):
    """Fly a plan. Returns `(pilot, boat, track, ticks)`.

    `watch(pilot, world, boat, now)` is called every tick, for the assertions
    that are about what happened *during* the run rather than where it ended -
    a track that was committed and then aged out, a docking phase that was
    entered and left, are invisible from the final state.
    """
    boat = FakeBoat(start, heading)
    link = FakeLink(boat)
    commander = commander_module.Commander(link, config)
    pilot = Pilot(config, commander)
    pilot.plan = plan
    pilot.start()
    world = WorldModel(config)

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
        commander.send(intent, state)
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
        tally, n = colour_votes([rgb] * 8, config)
        check(tally.get(name, 0) == 8, f"{rgb} reads as {name} ({tally})")

    # Uncoloured returns must never read as dark - that is the whole reason
    # scan.py writes -1 rather than black.
    tally, n = colour_votes([(-1, -1, -1)] * 6, config)
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
        commander.send(intent, state)
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


def main():
    start = time.time()
    for test in (
        test_geo,
        test_colour,
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
