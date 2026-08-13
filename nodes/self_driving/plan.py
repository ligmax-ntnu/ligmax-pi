"""The route, as data: waypoints that each carry a **role**.

    plan = Plan.parse(payload)          # from the operator, over the command channel
    plan.save()                         # survives a node restart
    plan = Plan.load()

Why roles exist
---------------
A Njord course is not a list of places, it is a list of *places plus what to do
between them*. The same GPS point means "drive here ignoring everything" on one
leg and "drive here, but a red buoy on this leg must be left to port" on the
next, and no amount of coordinate is going to say which. Njord's own tasks are
laid out exactly that way (see `njord.md`): part 1 of Task 1 is blind GNSS,
part 2 of the same task adds cardinal marks, Task 2 adds a vessel, Task 3 is a
dock.

So the role rides on the waypoint, and `pilot.py` swaps behaviour when it
advances. The operator lays a course once and the boat does the right thing on
each leg without anyone reloading anything.

    transit         drive to it. Ignore everything but not hitting things.
    buoys           drive to it, obeying the lateral marks and the cardinals.
    avoid           drive to it, watching for a vessel and giving way per COLREG.
    hold            arrive, then hold station. `hold_s` 0 means "until told".
    dock            bow-in docking: find the berth, enter, hold, REVERSE out.
    dock_parallel   come alongside, hold, then continue forward.
    park            park in the middle of three lines, hold 10 s, REVERSE out.
    park_parallel   the same, alongside, then continue forward.

`park` versus `dock`
--------------------
Two ways of finding the same Njord berth, kept apart on purpose because they fail
differently and there is no way to know which is better before the water.

`dock` looks for a **gap between two structures** - two clusters the right
distance apart (`perception/cluster.split_by_gap`) - and it obeys the ordinary
obstacle machinery on the way in.

`park` looks for **three lines making a rectangle with open corners**
(`perception/lines.py`, `perception/parking.py`), aims at the middle of it plus a
static per-type depth offset, and ignores the world model entirely: no buoy
colours, no clearances, no avoidance. That is what the parking task actually is -
the marks around it are scenery, and a buoy in the mouth is not a reason to refuse
to park.

Two fields exist for the parking roles alone: `park_offset_m`, how deep into the
space to sit, and `park_probe_deg`, **which way the docks are** from a waypoint
laid just outside them. The second one is there because the boat has one
forward-looking lidar (the aft unit is broken), so a berth a few metres further in
is not visible from the waypoint at all, and the honest answer is to creep that
way and look rather than to declare failure standing still.

Two fields that are the course's, not a waypoint's
--------------------------------------------------
`buoyage` and `cardinal_rule` ride on the plan rather than on each point, because
they describe **how the course was laid** rather than what to do at a place. See
`BUOYAGE_ROUTE` and `CARDINAL_INSIDE`: between them they are the difference between
a channel run up and down and a ring run once, and both default to the channel.

Coordinates
-----------
A waypoint may be given as `lat`/`lon` or as `x`/`y` grid metres - the dashboard
lays points on a metre chart, and the morning's handout will be degrees, so both
have to work. **Latitude and longitude is the canonical storage**, and grid
metres are converted on the way in, because the grid origin can be re-zeroed by
the operator (`recentre_origin`) and a plan stored in metres would silently move
with it. Everything is converted back to metres per tick against whatever origin
is current, so a recentre moves the boat and the course together.

The rule this file exists to serve
----------------------------------
NJORD §8.2: a boat that goes off course must re-enter **behind the last
successfully passed waypoint**. So `last_passed` is tracked explicitly, is
persisted, and survives a node restart - because the moment it matters is the
moment something has just gone wrong.
"""

from __future__ import annotations

import json
import logging
import math
import os
import time

from . import geo
from .config import PLAN_FILE, SPEED_LIMIT_KNOTS, SPEED_LIMIT_MS

log = logging.getLogger("self_driving.plan")

TRANSIT = "transit"
BUOYS = "buoys"
AVOID = "avoid"
HOLD = "hold"
DOCK = "dock"
DOCK_PARALLEL = "dock_parallel"
PARK = "park"
PARK_PARALLEL = "park_parallel"
#: The same two manoeuvres, found with the dock's AR tags instead of the lidar.
#: **These are the working docking roles as of 2026-08-11**: both lidars are down,
#: so `dock`, `dock_parallel`, `park` and `park_parallel` have no sensor and every
#: one of them will sit in SEARCH until the operator takes over. Kept rather than
#: deleted because a repaired lidar brings them straight back.
PARK_TAG = "park_tag"
PARK_TAG_PARALLEL = "park_tag_parallel"

#: NJORD §9.2, declared rather than classified. Which side the Otter comes from
#: is in the briefing and there are only two cases, so the operator picks the
#: manoeuvre on the dock instead of the boat inferring it from a monocular
#: bearing thirty seconds before it matters. `AVOID` still exists and still
#: classifies; these are what the Njord runs should actually be laid with.
#:
#: The `_BACKUP` pair consult **no sensor at all** and fire on the boat's own
#: progress along the leg. They are the answer to "what happens if the cameras
#: see nothing", which on a stack that has never been in the water is not a
#: hypothetical. See `behaviours/collision.py`.
COLLISION_FRONT = "collision_front"
COLLISION_RIGHT = "collision_right"
COLLISION_FRONT_BACKUP = "collision_front_backup"
COLLISION_RIGHT_BACKUP = "collision_right_backup"

ROLES = (TRANSIT, BUOYS, AVOID, HOLD, DOCK, DOCK_PARALLEL, PARK, PARK_PARALLEL,
         PARK_TAG, PARK_TAG_PARALLEL,
         COLLISION_FRONT, COLLISION_RIGHT,
         COLLISION_FRONT_BACKUP, COLLISION_RIGHT_BACKUP)

#: The four above, for anything that needs to ask "is this leg a Task 2 leg".
COLLISION_ROLES = frozenset({COLLISION_FRONT, COLLISION_RIGHT,
                             COLLISION_FRONT_BACKUP, COLLISION_RIGHT_BACKUP})

#: Roles that find their berth from the AR tags rather than from a lidar.
TAG_ROLES = frozenset({PARK_TAG, PARK_TAG_PARALLEL})

#: Every role `behaviours/parking.Parking` runs, whichever sensor finds the space.
#: What they have in common, and the reason this set exists, is that each one has a
#: berth it enters and an exit it can be told to skip (`park_no_exit`).
PARKING_ROLES = frozenset({PARK, PARK_PARALLEL, PARK_TAG, PARK_TAG_PARALLEL})

# Roles whose waypoint is a place to *arrive at and settle*, rather than a point
# to sweep through. They get the tighter acceptance radius and they are not
# allowed to be passed by the passing-plane test - "stop at GPS point 4" is not
# satisfied by driving past it.
SETTLE_ROLES = frozenset({HOLD, DOCK, DOCK_PARALLEL, PARK, PARK_PARALLEL,
                          PARK_TAG, PARK_TAG_PARALLEL})

#: Default seconds stationary, from the rules, per role (NJORD §9.3). Both parking
#: roles hold ten, which is what the team asked for and is the stricter of the
#: two rulebook figures (§9.3 asks 5 s of the alongside berth).
DEFAULT_HOLD_S = {
    DOCK: 10.0,
    DOCK_PARALLEL: 5.0,
    PARK: 10.0,
    PARK_PARALLEL: 10.0,
    # Ten for the alongside tag berth as well, for the same reason as its lidar
    # twin: §9.3 asks five and holding longer than the rules require cannot lose a
    # point, while a clock that stops a second early can lose the task.
    PARK_TAG: 10.0,
    PARK_TAG_PARALLEL: 10.0,
    HOLD: 0.0,
}

#: Direction of buoyage at this venue, degrees true (NJORD §10.2).
#:
#: The entrance is defined as true north, so this is a property of the *water*
#: and not of the course laid on it. A plan cannot be right about it, only
#: wrong - so it is not something an operator types in at 08:15. `Plan.parse`
#: accepts a `channel_bearing` for backwards compatibility and says loudly that
#: it is being ignored; `bearing_of_buoyage` returns this and only this.
BUOYAGE_BEARING_DEG = 0.0

#: How the lateral rule reads the direction of buoyage. Plan-level, not per
#: waypoint, because it is a fact about **how the course was laid** and a plan that
#: mixed the two would be right about four legs and wrong about one - which is the
#: failure `buoys.BUOYAGE_INVERTS_BEYOND_DEG` exists to avoid, reintroduced by the
#: back door.
#:
#:  "venue"  The default and everything before 2026-08-13: the direction of
#:           buoyage is the water's, `BUOYAGE_BEARING_DEG`, and a leg running more
#:           than `buoys.BUOYAGE_INVERTS_BEYOND_DEG` back down it inverts red and
#:           green. Right for Task 1 and Task 2, which run up and down a channel.
#:  "route"  The direction of buoyage is **the direction of travel**: red to port
#:           and green to starboard on every leg, never inverted.
#:
#: "route" exists for a **ring**, and a ring is why "venue" cannot describe it. The
#: surprise task's marks are laid as a closed loop run once in one direction, so
#: every compass bearing occurs somewhere on it - the 2026-08-13 course runs 185,
#: 291, 179, 141 and 020 - and no fixed reference plus a window can call all of
#: them "with the buoyage". Under "venue" the 179 and 141 deg legs both invert, and
#: those are TWO OF THE THREE scored ring legs: the boat would pass every mark on
#: them the wrong side, confidently and consistently. See docs/findings.md item 41.
BUOYAGE_VENUE = "venue"
BUOYAGE_ROUTE = "route"
BUOYAGE_MODES = (BUOYAGE_VENUE, BUOYAGE_ROUTE)

#: How a cardinal mark is passed.
#:
#:  "safe_side"  The default, NJORD 10.3: a cardinal says which side of *itself* is
#:               safe water, so route through a via-point on that side. Needs the
#:               camera to have committed to which cardinal it is, and no lidar can
#:               tell a north mark from a south one - so an uncommitted mark buys a
#:               slow-down and the planned line (`behaviours/buoys._cardinal`).
#:  "inside"     Pass **inside** the mark: between it and the interior of the loop
#:               the course itself traces. What the marks mean is not consulted.
#:
#: "inside" is not a relaxation of "safe_side", it is a different statement about a
#: different course. It is right when the cardinals ring the *outside* of a closed
#: circuit and the jury's instruction is "stay inside them" - the 2026-08-13
#: surprise task, whose five marks 4.1-4.5 sit outside a loop the boat runs once.
#:
#: What it buys is the thing "safe_side" cannot have on this boat: **an unresolved
#: cardinal is fully actionable.** Which side is inside comes from the geometry of
#: the plan (`interior_side`), so the topmark, the second-stage classifier and the
#: alternation prior are all out of the loop, and a mark the camera will never
#: commit to is routed round exactly as well as one it does.
CARDINAL_SAFE_SIDE = "safe_side"
CARDINAL_INSIDE = "inside"
CARDINAL_RULES = (CARDINAL_SAFE_SIDE, CARDINAL_INSIDE)

#: `interior_side` returns one of these: which hand the inside of the course is on.
SIDE_PORT = -1
SIDE_STARBOARD = +1

#: The smallest loop `interior_side` will read a hand off. Ten times smaller than
#: the 2026-08-13 surprise course encloses (~730 m2), and far larger than the few
#: square metres of GNSS noise a course laid straight comes out with.
MIN_LOOP_AREA_M2 = 70.0

#: Sentinel for "not computed yet", so that a genuine `None` - a course with no
#: loop - is cached instead of being recomputed and re-warned about every tick.
_UNSET = object()


class Waypoint:
    """One point on the course, and what to do on the way to it.

    `index` is its position in the plan and is what the operator's "skip" and
    "go back one" buttons address, so it is stable for the life of a plan.
    """

    __slots__ = (
        "index", "name", "lat", "lon", "role", "speed", "radius", "hold_s",
        "channel_bearing", "berth_width_m", "park_offset_m", "park_probe_deg",
        "berth", "park_no_exit", "notes",
    )

    def __init__(self, index, name, lat, lon, role, **kwargs):
        self.index = index
        self.name = name
        self.lat = lat
        self.lon = lon
        self.role = role
        self.speed = kwargs.get("speed")
        self.radius = kwargs.get("radius")
        self.hold_s = kwargs.get("hold_s")
        self.channel_bearing = kwargs.get("channel_bearing")
        self.berth_width_m = kwargs.get("berth_width_m")
        # How deep into a parking space to sit, metres from the middle, positive
        # towards the closed end. Overrides the per-type figure in `config.py` for
        # this waypoint only - which is what a space that turns out to be shorter
        # than the handbook says needs, without touching the other parking type.
        self.park_offset_m = kwargs.get("park_offset_m")
        # Which way the docks are from this waypoint, as a true bearing. Used only
        # when the parking space is not in view from the waypoint itself, which
        # with one forward-looking lidar is the ordinary case rather than a fault:
        # the boat creeps along this bearing until it finds three lines
        # (`behaviours/parking.py:_probe`). Per waypoint because it is a fact about
        # a berth, not about the boat - Havet arena's is about 120 degrees, in
        # towards land, and `config.PARK_PROBE_BEARING_DEG` is that default.
        self.park_probe_deg = kwargs.get("park_probe_deg")
        # Which berth to take, by name ("berth 1" / "berth 2" / "alongside"), for
        # the tag roles only. Normally left unset: two bow-in berths are laid side
        # by side with one occupied, and the boat picks the free one by which
        # closed-end tag it can see (`perception/artags.py`). This is the override
        # for when it has picked wrong, or when the tags disagree with what the
        # jury said - a name the operator types rather than an argument.
        self.berth = kwargs.get("berth")
        # Stay in the berth when the hold is over, instead of reversing out of it.
        # For the parking roles alone, and it exists because of one sentence in the
        # organisers' 2026-08-12 clarification of the surprise task: "The task ends
        # inside the final dock. You do not need to exit the dock after completing
        # the final docking manoeuvre."
        #
        # Default False, i.e. every other course keeps the behaviour NJORD 9.3
        # describes - the berth is left the way it was entered, and the next
        # waypoint does the forward part. This is not a shortcut for a boat that
        # cannot get out: an exit that is skipped is an exit nobody watched, so it
        # is per waypoint and typed by an operator rather than inferred from a
        # waypoint being last in the plan. See `behaviours/parking.py:_hold`.
        self.park_no_exit = bool(kwargs.get("park_no_exit") or False)
        self.notes = kwargs.get("notes") or ""

    # ------------------------------------------------------------------ query

    @property
    def settles(self):
        return self.role in SETTLE_ROLES

    def hold_seconds(self):
        if self.hold_s is not None:
            return float(self.hold_s)
        return DEFAULT_HOLD_S.get(self.role, 0.0)

    def world(self, origin):
        """`(east, north)` metres against the current origin, or None."""
        return geo.to_world(self.lat, self.lon, origin)

    def to_dict(self):
        out = {
            "index": self.index,
            "name": self.name,
            "lat": round(self.lat, 8),
            "lon": round(self.lon, 8),
            "role": self.role,
        }
        for key in (
            "speed", "radius", "hold_s", "channel_bearing", "berth_width_m",
            "park_offset_m", "park_probe_deg", "berth",
        ):
            value = getattr(self, key)
            if value is not None:
                out[key] = value
        # Only when set. It is a bool with a default rather than an optional, so
        # the `is not None` loop above would put `"park_no_exit": false` on every
        # waypoint of every course - and this dict is what the dashboard draws, what
        # `save()` writes and what a trip header carries.
        if self.park_no_exit:
            out["park_no_exit"] = True
        if self.notes:
            out["notes"] = self.notes
        return out

    def __repr__(self):
        return f"<Waypoint {self.index} {self.name!r} {self.role}>"


class PlanError(ValueError):
    """A plan that cannot be flown. The message goes straight to the operator."""


class Plan:
    """An ordered list of `Waypoint`, plus where the boat has got to.

    The cursor (`index`) and `last_passed` are part of the plan rather than of
    the pilot, so that a node restart mid-run resumes at the right waypoint
    instead of driving back to the start of the course.
    """

    def __init__(self, waypoints, name="plan", channel_bearing=0.0, created=None,
                 buoyage=BUOYAGE_VENUE, cardinal_rule=CARDINAL_SAFE_SIDE):
        self.waypoints = waypoints
        self.name = name
        # The direction of buoyage: sailing this way, red is to port. Njord lays
        # the course with seaward = north (§10.2), which is the default, but a
        # leg that runs back down the course inverts the sense and a boat that
        # does not know it passes every gate on the wrong side.
        self.channel_bearing = channel_bearing
        # How the lateral rule and the cardinals are read on THIS course. Both
        # default to what every plan before 2026-08-13 did, so a course that says
        # nothing behaves exactly as it always has.
        self.buoyage = buoyage
        self.cardinal_rule = cardinal_rule
        self.created = created or time.time()
        self.index = 0
        self.last_passed = -1
        self._interior_side = _UNSET

    # ------------------------------------------------------------------ parse

    @classmethod
    def parse(cls, payload, origin=None):
        """Build a plan from the operator's JSON. Raises `PlanError` with a
        message meant to be shown verbatim in the dashboard's command list.

        Deliberately strict. A plan is typed in under time pressure on a
        competition morning, and a silently-dropped waypoint is far worse than a
        refused upload: the boat would run a course nobody laid.
        """
        if not isinstance(payload, dict):
            raise PlanError("plan must be an object")
        raw = payload.get("waypoints")
        if not isinstance(raw, list) or not raw:
            raise PlanError("plan needs a non-empty 'waypoints' list")
        if len(raw) > 200:
            raise PlanError(f"{len(raw)} waypoints is more than a Njord course has")

        default_bearing = _float(payload.get("channel_bearing"), BUOYAGE_BEARING_DEG)
        waypoints = []
        for position, item in enumerate(raw):
            waypoints.append(_waypoint(position, item, origin, default_bearing))

        # The direction of buoyage is hardcoded (`BUOYAGE_BEARING_DEG`). A plan
        # that still carries one is not refused - refusing the course over a
        # field that no longer does anything would be the more expensive
        # failure - but it is not accepted quietly either, because an operator
        # who typed a number is entitled to know the boat is not reading it.
        carried = {default_bearing} | {
            float(point.channel_bearing)
            for point in waypoints
            if point.channel_bearing is not None
        }
        stray = sorted(
            bearing
            for bearing in carried
            if abs(geo.angle_diff(bearing, BUOYAGE_BEARING_DEG)) > 0.5
        )
        if stray:
            log.warning(
                "plan carries channel_bearing %s - ignored; buoyage is fixed at "
                "%.0f deg true, the venue's entrance (NJORD 10.2)",
                ", ".join(f"{bearing:.0f}" for bearing in stray),
                BUOYAGE_BEARING_DEG,
            )

        plan = cls(
            waypoints,
            name=str(payload.get("name") or "plan")[:64],
            channel_bearing=default_bearing,
            created=payload.get("created"),
            buoyage=_choice(payload.get("buoyage"), BUOYAGE_MODES, BUOYAGE_VENUE,
                            "buoyage"),
            cardinal_rule=_choice(payload.get("cardinal_rule"), CARDINAL_RULES,
                                  CARDINAL_SAFE_SIDE, "cardinal_rule"),
        )
        # Said out loud at upload, and both halves of it. A course laid as a ring
        # behaves differently on every leg from the same course laid as a channel,
        # the difference is invisible on the chart, and the ack is the one place an
        # operator sees which of the two the boat thinks it has.
        if plan.buoyage != BUOYAGE_VENUE or plan.cardinal_rule != CARDINAL_SAFE_SIDE:
            side = plan.interior_side()
            log.info(
                "plan %r: buoyage=%s, cardinals=%s, inside is to %s",
                plan.name, plan.buoyage, plan.cardinal_rule,
                "port" if side == SIDE_PORT
                else "starboard" if side == SIDE_STARBOARD
                else "NOWHERE - the course encloses nothing",
            )
            # Refused, not warned about. `cardinal_rule: "inside"` on a course with
            # no inside is a plan whose two halves contradict each other, and the
            # run-time behaviour - hold the planned line past every cardinal - is
            # indistinguishable from the rule working. That is exactly the kind of
            # silence this file is strict to avoid.
            if plan.cardinal_rule == CARDINAL_INSIDE and side is None:
                raise PlanError(
                    "cardinal_rule 'inside' needs a course that encloses "
                    f"something, and this one encloses under {MIN_LOOP_AREA_M2:.0f} "
                    "m2 - lay the loop, or use 'safe_side'"
                )
        # A plan may name where to resume, for the §8.2 re-entry case: the
        # operator drives the boat back behind the last good waypoint by hand
        # and re-uploads with `start_at` set rather than watching it run the
        # whole course again.
        start_at = payload.get("start_at")
        if start_at is not None:
            plan.index = max(0, min(len(waypoints) - 1, int(start_at)))
            plan.last_passed = plan.index - 1
        return plan

    # ---------------------------------------------------------------- cursor

    @property
    def finished(self):
        return self.index >= len(self.waypoints)

    @property
    def current(self):
        if self.finished:
            return None
        return self.waypoints[self.index]

    @property
    def previous(self):
        """The waypoint the current leg starts from, or None at the first one."""
        if self.index <= 0 or self.index > len(self.waypoints):
            return None
        return self.waypoints[self.index - 1]

    def advance(self, why=""):
        """Mark the current waypoint passed and move on. Returns the new one."""
        if self.finished:
            return None
        self.last_passed = self.index
        self.index += 1
        log.info(
            "waypoint %s passed%s; next is %s",
            self.waypoints[self.last_passed].name,
            f" ({why})" if why else "",
            self.current.name if self.current else "END OF PLAN",
        )
        return self.current

    def rewind(self):
        """Back up one waypoint. The §8.2 recovery, from the operator's button."""
        self.index = max(0, self.index - 1)
        self.last_passed = self.index - 1
        return self.current

    def jump_to(self, index):
        self.index = max(0, min(len(self.waypoints), int(index)))
        self.last_passed = self.index - 1
        return self.current

    def reset(self):
        self.index = 0
        self.last_passed = -1

    # ------------------------------------------------------------------ legs

    def leg(self, origin, boat_xy=None):
        """`(start_xy, end_xy)` of the current leg in world metres, or None.

        The leg's start is the previous waypoint - or, at the very first one,
        the boat's own position. Starting the first leg at the boat is what lets
        cross-track control work from the moment autonomy is engaged, rather
        than needing one waypoint of history first.
        """
        current = self.current
        if current is None:
            return None
        end = current.world(origin)
        if end is None:
            return None
        previous = self.previous
        start = previous.world(origin) if previous is not None else boat_xy
        if start is None:
            start = boat_xy
        if start is None:
            return None
        return start, end

    def reference_path(self, origin):
        """Every waypoint in grid metres, for the chart's amber ideal-route layer.

        NJORD §11.4 asks for the course over ground "compared against the ideal
        route from GNSS waypoints" - this is the ideal route half of that
        comparison, and the dashboard already knows how to draw a `path` with
        `kind: "reference"`.
        """
        points = []
        for waypoint in self.waypoints:
            xy = waypoint.world(origin)
            if xy is not None:
                points.append([round(xy[0], 2), round(xy[1], 2)])
        return points

    def reference_layer(self, origin):
        """The route *and what each leg is for*, for the operator's chart.

        `reference_path` alone gives the dashboard a line and nothing else, so a
        course drawn from it cannot show that waypoint 5 is a dock and waypoint 3
        obeys the buoy rules - which is the one thing about a Njord plan worth
        seeing before it is run, and the thing a mis-typed role hides.

        The three arrays run in lockstep with `points`, and `indices` says which
        waypoint each entry came from: a waypoint that will not convert is
        dropped from all four together rather than silently shifting the roles
        one place along, which would paint a plausible and completely wrong
        course. (Nothing drops in practice - `world()` only fails without an
        origin, which the caller has already checked - but a mirrored world is
        the failure most likely to survive a casual glance, so it is made
        impossible rather than unlikely.)
        """
        points, roles, names, indices = [], [], [], []
        for waypoint in self.waypoints:
            xy = waypoint.world(origin)
            if xy is None:
                continue
            points.append([round(xy[0], 2), round(xy[1], 2)])
            roles.append(waypoint.role)
            names.append(waypoint.name)
            indices.append(waypoint.index)
        return {
            "points": points,
            "roles": roles,
            "names": names,
            "indices": indices,
            # Where the boat is up to *as of this upload*. It goes stale as the
            # boat advances, and the dashboard prefers the live cursor out of
            # `telemetry.autopilot.plan.index` - which is published at 2 Hz and
            # costs nothing extra - rather than this being re-sent every tick.
            "target_index": self.index,
            "passed_index": self.last_passed,
        }

    def follows_route_buoyage(self):
        """Whether the direction of buoyage is this course's own direction of travel.

        True only for `buoyage: "route"`. `behaviours/buoys._with_the_buoyage` reads
        it and stops asking the compass; see `BUOYAGE_ROUTE` for why a ring needs it.
        """
        return self.buoyage == BUOYAGE_ROUTE

    def passes_cardinals_inside(self):
        """Whether cardinals are passed inside the loop rather than on their safe side."""
        return self.cardinal_rule == CARDINAL_INSIDE

    def interior_side(self):
        """Which hand the inside of this course is on: `SIDE_PORT`/`SIDE_STARBOARD`.

        The signed area of the waypoint polygon, closed back to the first point. A
        counter-clockwise loop is entered with its interior to **port**, a clockwise
        one to starboard, and that is the whole derivation - `cardinal_rule:
        "inside"` needs a side and this is where it comes from.

        Computed from the plan and nothing else, which is the point. The alternative
        was an operator typing "port" on a competition morning, and the two ways of
        being wrong are not comparable: a mis-derived side is a course whose own
        shape contradicts it and which `commander` can therefore refuse, while a
        mis-typed one is a boat that rounds every mark on the wrong side and looks
        entirely deliberate doing it.

        `None` when there is no loop to speak of - fewer than three convertible
        points, or a polygon so thin that its own sign is noise. A course laid
        straight has no inside, and saying so is better than picking a hand out of
        rounding error. `behaviours/buoys` falls back to holding the planned line.

        In local flat metres about the first waypoint rather than against the grid
        origin: handedness is scale-free, an origin the operator may since have
        re-zeroed is one more thing to be wrong, and `geo.to_world` would make this
        depend on a fix the boat may not have yet.
        """
        if self._interior_side is not _UNSET:
            return self._interior_side
        self._interior_side = self._compute_interior_side()
        return self._interior_side

    def _compute_interior_side(self):
        points = [(point.lat, point.lon) for point in self.waypoints]
        if len(points) < 3:
            return None
        lat0 = points[0][0]
        per_lat = geo.METRES_PER_DEGREE_LAT
        per_lon = per_lat * math.cos(math.radians(lat0))
        flat = [
            ((lon - points[0][1]) * per_lon, (lat - lat0) * per_lat)
            for lat, lon in points
        ]
        twice_area = 0.0
        for (east, north), (next_east, next_north) in zip(flat, flat[1:] + flat[:1]):
            twice_area += east * next_north - next_east * north
        # A threshold on the area itself, not on its sign. A course of six points
        # laid nearly in a line has a signed area of a few square metres made
        # entirely of GNSS noise, and its sign flips between two uploads of what an
        # operator would call the same course. `MIN_LOOP_AREA_M2` is a tenth of the
        # smallest circuit anyone would call a ring.
        if abs(twice_area) < 2.0 * MIN_LOOP_AREA_M2:
            log.warning(
                "plan %r encloses only %.0f m2 - too thin to say which side is "
                "inside; cardinals will hold the planned line",
                self.name, abs(twice_area) / 2.0,
            )
            return None
        return SIDE_PORT if twice_area > 0.0 else SIDE_STARBOARD

    def bearing_of_buoyage(self, waypoint):
        """The direction of buoyage on the leg into `waypoint`, degrees true.

        Always true north - `BUOYAGE_BEARING_DEG`, the venue's own definition of
        its entrance. Whatever the plan carries is ignored, so the boat cannot be
        talked onto the wrong side of a red by a number in a file. `waypoint` is
        still taken: the caller has one in hand, and a venue whose buoyage turns
        partway up the channel is a plausible future for this to key off.
        """
        return BUOYAGE_BEARING_DEG

    # ------------------------------------------------------------ persistence

    def to_dict(self):
        return {
            "name": self.name,
            "created": self.created,
            "channel_bearing": self.channel_bearing,
            "buoyage": self.buoyage,
            "cardinal_rule": self.cardinal_rule,
            "index": self.index,
            "last_passed": self.last_passed,
            "waypoints": [waypoint.to_dict() for waypoint in self.waypoints],
        }

    def save(self, path=PLAN_FILE):
        """Write the plan out, cursor included. Best effort - never raises.

        Losing the file costs a re-upload; refusing to run because it could not
        be written would cost the attempt.
        """
        try:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            temporary = f"{path}.tmp"
            with open(temporary, "w", encoding="utf-8") as handle:
                json.dump(self.to_dict(), handle, indent=1)
            os.replace(temporary, path)  # atomic: never a half-written plan
        except OSError as exc:
            log.warning("could not save the plan to %s: %s", path, exc)

    @classmethod
    def load(cls, path=PLAN_FILE):
        """The stored plan, cursor and all, or None. A bad file is no file."""
        try:
            with open(path, "r", encoding="utf-8") as handle:
                stored = json.load(handle)
        except (OSError, ValueError):
            return None
        try:
            plan = cls.parse(stored)
        except PlanError as exc:
            log.warning("ignoring the stored plan in %s: %s", path, exc)
            return None
        plan.index = max(0, min(len(plan.waypoints), int(stored.get("index", 0))))
        plan.last_passed = int(stored.get("last_passed", plan.index - 1))
        log.info(
            "restored plan %r: %d waypoint(s), resuming at %s",
            plan.name,
            len(plan.waypoints),
            plan.current.name if plan.current else "END",
        )
        return plan

    # ------------------------------------------------------------- telemetry

    def telemetry(self):
        current = self.current
        return {
            "name": self.name,
            "waypoints": len(self.waypoints),
            "index": self.index,
            "current": current.name if current else None,
            "role": current.role if current else None,
            "last_passed": (
                self.waypoints[self.last_passed].name
                if 0 <= self.last_passed < len(self.waypoints)
                else None
            ),
            "finished": self.finished,
            # On the wire every tick, because the two of them decide which side of
            # every mark on the course the boat goes and there is nothing else on the
            # dashboard that would show it. `interior_side` only for the rule that
            # uses one.
            "buoyage": self.buoyage,
            "cardinal_rule": self.cardinal_rule,
            **(
                {"inside": (
                    "port" if self.interior_side() == SIDE_PORT
                    else "starboard" if self.interior_side() == SIDE_STARBOARD
                    else None
                )}
                if self.passes_cardinals_inside() else {}
            ),
        }

    def __len__(self):
        return len(self.waypoints)


# ------------------------------------------------------------------ parsing

def _waypoint(position, item, origin, default_bearing):
    if not isinstance(item, dict):
        raise PlanError(f"waypoint {position + 1} is not an object")

    role = str(item.get("role") or TRANSIT).strip().lower()
    if role not in ROLES:
        raise PlanError(
            f"waypoint {position + 1}: '{role}' is not a role "
            f"({', '.join(ROLES)})"
        )

    lat, lon = _coordinates(position, item, origin)
    name = str(item.get("name") or item.get("id") or position + 1)[:32]

    return Waypoint(
        position,
        name,
        lat,
        lon,
        role,
        # Upper bound is the vessel's 5 kn limit, not an arbitrary 3.0 m/s -
        # which was 5.83 kn and would have let a plan ask for more than the boat
        # is allowed to do. Refused at upload rather than quietly clamped: the
        # operator finds out on the dock, with the ack in front of them, instead
        # of wondering later why the boat would not hold the speed they typed.
        speed=_optional_float(
            item.get("speed"), 0.05, SPEED_LIMIT_MS, position, "speed"
        ),
        radius=_optional_float(item.get("radius"), 0.3, 50.0, position, "radius"),
        hold_s=_optional_float(item.get("hold_s"), 0.0, 600.0, position, "hold_s"),
        channel_bearing=(
            _float(item["channel_bearing"], default_bearing)
            if "channel_bearing" in item
            else None
        ),
        berth_width_m=_optional_float(
            item.get("berth_width_m"), 0.5, 10.0, position, "berth_width_m"
        ),
        # Signed, because "sit half a metre short of the middle" is as ordinary a
        # request as "half a metre deeper". Bounded at +-3 m because the spaces are
        # 2-4 m across and an offset bigger than the space is a typo, not a plan -
        # `behaviours/parking.py` clamps it to the measured space as well and says
        # on the panel when it had to.
        park_offset_m=_optional_float(
            item.get("park_offset_m"), -3.0, 3.0, position, "park_offset_m"
        ),
        # A compass bearing, so 0..360. Bounded rather than wrapped, because a
        # bearing of 480 in a plan is a typo and wrapping it to 120 would hide
        # that - and this is the number the boat drives blind towards a dock on.
        park_probe_deg=_optional_float(
            item.get("park_probe_deg"), 0.0, 360.0, position, "park_probe_deg"
        ),
        berth=_berth(position, item, role),
        # Refused on a role that has no berth to stay in, rather than ignored: a
        # `park_no_exit` an operator typed onto the wrong waypoint is a boat they
        # expect to stop and which drives on, and silence is how that is found out
        # too late. The parking roles are the only ones with an exit to skip.
        park_no_exit=_park_no_exit(position, item, role),
        notes=str(item.get("notes") or "")[:120],
    )


def _park_no_exit(position, item, role):
    if "park_no_exit" not in item:
        return False
    if not bool(item["park_no_exit"]):
        return False
    if role not in PARKING_ROLES:
        raise PlanError(
            f"waypoint {position + 1}: 'park_no_exit' is only for the parking "
            f"roles ({', '.join(sorted(PARKING_ROLES))}), and this one is '{role}'"
        )
    return True


def _berth(position, item, role):
    """The named berth override, checked against the ones that exist for this role.

    Refused at upload rather than ignored at run time. A misspelt berth name would
    otherwise fall through to "let the tags decide" and look identical to not having
    asked - which is the worst possible outcome for an override whose whole purpose
    is overruling the tags.
    """
    raw = item.get("berth")
    if raw is None or str(raw).strip() == "":
        return None
    name = str(raw).strip().lower()
    if role not in TAG_ROLES:
        raise PlanError(
            f"waypoint {position + 1}: 'berth' only means something for the tag "
            f"roles ({', '.join(sorted(TAG_ROLES))}), and this one is '{role}'"
        )
    # Imported here, not at module scope. The berth names belong to the tag
    # geometry - it is the thing that knows which ids make which berth - and a
    # top-level import would put `perception` on `plan`'s import path, which is
    # backwards: a plan is data and has no business needing a sensor to load.
    # Deferring it also keeps one source of truth instead of a fourth copy of a
    # list of names.
    from .perception.artags import BOW_IN_BERTHS, PARALLEL_BERTHS

    allowed = sorted(PARALLEL_BERTHS if role == PARK_TAG_PARALLEL
                     else BOW_IN_BERTHS)
    if name not in allowed:
        raise PlanError(
            f"waypoint {position + 1}: '{raw}' is not a berth for {role} "
            f"({', '.join(allowed)})"
        )
    return name


def _coordinates(position, item, origin):
    """`(lat, lon)` from either form. Grid metres need an origin to convert."""
    if "lat" in item and "lon" in item:
        lat = _float(item["lat"], None)
        lon = _float(item["lon"], None)
        if lat is None or lon is None or abs(lat) > 90.0 or abs(lon) > 180.0:
            raise PlanError(f"waypoint {position + 1}: lat/lon is not a position")
        return lat, lon

    if "x" in item and "y" in item:
        if not origin:
            raise PlanError(
                f"waypoint {position + 1} is in grid metres, but the boat has no "
                "GPS origin yet - send lat/lon, or wait for a fix"
            )
        x = _float(item["x"], None)
        y = _float(item["y"], None)
        if x is None or y is None:
            raise PlanError(f"waypoint {position + 1}: x/y is not a position")
        converted = geo.to_global(x, y, origin)
        if converted is None:
            raise PlanError(f"waypoint {position + 1}: could not georeference x/y")
        return converted

    raise PlanError(f"waypoint {position + 1} has neither lat/lon nor x/y")


def _choice(value, allowed, default, field):
    """One of `allowed`, or `default` when absent. Anything else is refused.

    Refused rather than defaulted, because both of these fields have a *safe-looking*
    default that is the wrong answer for the course that bothered to set them: a
    misspelt `"rout"` would silently fly a ring under the channel rule and invert
    red and green on the legs that run back down it.
    """
    if value is None or (isinstance(value, str) and not value.strip()):
        return default
    text = str(value).strip().lower()
    if text not in allowed:
        raise PlanError(
            f"{field} '{value}' is not one of {', '.join(allowed)}"
        )
    return text


def _float(value, default):
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    if out != out or out in (float("inf"), float("-inf")):  # NaN / inf
        return default
    return out


def _optional_float(value, low, high, position, field):
    if value is None:
        return None
    out = _float(value, None)
    if out is None:
        raise PlanError(f"waypoint {position + 1}: {field} is not a number")
    if not (low <= out <= high):
        if field == "speed" and out > high:
            # Named explicitly, because "outside 0.05..2.5722" tells an operator
            # under time pressure nothing at all, and the number they typed is
            # almost certainly in knots or was copied from a faster boat.
            raise PlanError(
                f"waypoint {position + 1}: speed {out} m/s is over the "
                f"{SPEED_LIMIT_KNOTS:.0f} knot limit "
                f"({SPEED_LIMIT_MS:.2f} m/s) - lower it"
            )
        raise PlanError(
            f"waypoint {position + 1}: {field} of {out} is outside {low}..{high}"
        )
    return out
