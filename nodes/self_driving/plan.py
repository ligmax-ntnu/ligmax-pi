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

ROLES = (TRANSIT, BUOYS, AVOID, HOLD, DOCK, DOCK_PARALLEL, PARK, PARK_PARALLEL)

# Roles whose waypoint is a place to *arrive at and settle*, rather than a point
# to sweep through. They get the tighter acceptance radius and they are not
# allowed to be passed by the passing-plane test - "stop at GPS point 4" is not
# satisfied by driving past it.
SETTLE_ROLES = frozenset({HOLD, DOCK, DOCK_PARALLEL, PARK, PARK_PARALLEL})

#: Default seconds stationary, from the rules, per role (NJORD §9.3). Both parking
#: roles hold ten, which is what the team asked for and is the stricter of the
#: two rulebook figures (§9.3 asks 5 s of the alongside berth).
DEFAULT_HOLD_S = {
    DOCK: 10.0,
    DOCK_PARALLEL: 5.0,
    PARK: 10.0,
    PARK_PARALLEL: 10.0,
    HOLD: 0.0,
}


class Waypoint:
    """One point on the course, and what to do on the way to it.

    `index` is its position in the plan and is what the operator's "skip" and
    "go back one" buttons address, so it is stable for the life of a plan.
    """

    __slots__ = (
        "index", "name", "lat", "lon", "role", "speed", "radius", "hold_s",
        "channel_bearing", "berth_width_m", "park_offset_m", "notes",
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
            "park_offset_m",
        ):
            value = getattr(self, key)
            if value is not None:
                out[key] = value
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

    def __init__(self, waypoints, name="plan", channel_bearing=0.0, created=None):
        self.waypoints = waypoints
        self.name = name
        # The direction of buoyage: sailing this way, red is to port. Njord lays
        # the course with seaward = north (§10.2), which is the default, but a
        # leg that runs back down the course inverts the sense and a boat that
        # does not know it passes every gate on the wrong side.
        self.channel_bearing = channel_bearing
        self.created = created or time.time()
        self.index = 0
        self.last_passed = -1

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

        default_bearing = _float(payload.get("channel_bearing"), 0.0)
        waypoints = []
        for position, item in enumerate(raw):
            waypoints.append(_waypoint(position, item, origin, default_bearing))

        plan = cls(
            waypoints,
            name=str(payload.get("name") or "plan")[:64],
            channel_bearing=default_bearing,
            created=payload.get("created"),
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

    def bearing_of_buoyage(self, waypoint):
        """The direction of buoyage on the leg into `waypoint`, degrees."""
        if waypoint is not None and waypoint.channel_bearing is not None:
            return float(waypoint.channel_bearing)
        return float(self.channel_bearing)

    # ------------------------------------------------------------ persistence

    def to_dict(self):
        return {
            "name": self.name,
            "created": self.created,
            "channel_bearing": self.channel_bearing,
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
        notes=str(item.get("notes") or "")[:120],
    )


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
