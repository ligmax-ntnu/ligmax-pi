"""Frames, angles and distances. Pure functions, no state, no I/O.

Everything in the autonomy stack lives in **one** metric frame and this module
is the only place that converts into or out of it:

    WORLD   metres from the grid origin. +x EAST, +y NORTH. This is the same
            frame `nodes/io_manager/navigation.py` publishes the boat's position
            in and the same one the dashboard draws its chart in, deliberately -
            a point can be moved between the planner, the telemetry and the map
            without being reinterpreted.

    BOAT    metres from the boat's datum. +x STARBOARD, +y FORWARD. This is the
            frame `nodes/io_manager/scan.py` puts both lidars into, and the
            frame every sensor return arrives in.

    COMPASS degrees, 0 = north, 90 = east, increasing clockwise. Every heading
            and bearing in this codebase. NOT mathematical radians-from-east,
            which is the convention numpy's atan2 hands you and the source of
            every sign error in this kind of code.

The world frame is a flat-earth tangent plane about the origin's latitude, using
the same 111320 m/deg constant as `navigation.py` and `ligmax-server`'s
`web/js/geo.js`. Over a Njord course a few hundred metres across the error is
well under a metre; using a different constant here would put the planner and
the map in two slightly different worlds, which is far worse than the
approximation.

Why compass degrees rather than radians throughout: every number that crosses a
link - MAVLink's `hdg`, the dashboard's `heading_deg`, the operator's idea of
"pass east of it" - is already compass degrees. Converting at every boundary is
how a sign gets lost.
"""

import math

# Must equal METRES_PER_DEGREE_LAT in nodes/io_manager/navigation.py and in
# ligmax-server/web/js/geo.js. See the module docstring.
METRES_PER_DEGREE_LAT = 111320.0


# ------------------------------------------------------------------- angles

def wrap360(degrees):
    """Any angle onto [0, 360)."""
    return degrees % 360.0


def wrap180(degrees):
    """A signed difference onto (-180, 180]. Positive is clockwise/starboard."""
    return ((degrees + 540.0) % 360.0) - 180.0


def angle_diff(a, b):
    """`a - b`, wrapped. Positive means `a` is clockwise of `b`."""
    return wrap180(a - b)


def bearing_to(from_xy, to_xy):
    """Compass bearing from one world point to another, degrees.

    atan2(east, north), not atan2(north, east): compass zero is north and it
    increases clockwise, which is the transpose of the mathematical convention.
    """
    dx = to_xy[0] - from_xy[0]  # east
    dy = to_xy[1] - from_xy[1]  # north
    if dx == 0.0 and dy == 0.0:
        return 0.0
    return wrap360(math.degrees(math.atan2(dx, dy)))


def unit(bearing_deg):
    """A compass bearing as a world-frame unit vector `(east, north)`."""
    rad = math.radians(bearing_deg)
    return (math.sin(rad), math.cos(rad))


def distance(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


# -------------------------------------------------------------- frame changes

def boat_to_world(starboard, forward, heading_deg):
    """A BOAT-frame offset as a WORLD-frame offset, given the boat's heading.

    Rotation only - add the boat's own position to get an absolute point.
    Check it against the two headings you can do in your head: at heading 0
    (north) forward is north and starboard is east; at heading 90 (east)
    forward is east and starboard is south.
    """
    rad = math.radians(heading_deg)
    cos_h, sin_h = math.cos(rad), math.sin(rad)
    east = starboard * cos_h + forward * sin_h
    north = -starboard * sin_h + forward * cos_h
    return east, north


def world_to_boat(east, north, heading_deg):
    """The inverse of `boat_to_world`: a WORLD offset in BOAT axes."""
    rad = math.radians(heading_deg)
    cos_h, sin_h = math.cos(rad), math.sin(rad)
    starboard = east * cos_h - north * sin_h
    forward = east * sin_h + north * cos_h
    return starboard, forward


def relative_bearing(target_xy, boat_xy, heading_deg):
    """Where a world point lies relative to the bow, degrees.

    0 is dead ahead, +90 is abeam to starboard, 180 is astern. This is the
    number every COLREG rule is written in terms of.
    """
    return angle_diff(bearing_to(boat_xy, target_xy), heading_deg)


# ----------------------------------------------------------------- lat / lon

def to_world(lat, lon, origin):
    """`(lat, lon)` degrees -> world `(east, north)` metres. None without an origin.

    `origin` is `{"lat": ..., "lon": ...}` exactly as `navigation.py` publishes
    it, so the conversion is the same one the dashboard does in the other
    direction.
    """
    if not origin:
        return None
    north = (lat - origin["lat"]) * METRES_PER_DEGREE_LAT
    east = (
        (lon - origin["lon"])
        * METRES_PER_DEGREE_LAT
        * math.cos(math.radians(origin["lat"]))
    )
    return east, north


def to_global(east, north, origin):
    """World `(east, north)` metres -> `(lat, lon)` degrees. None without an origin.

    The exact inverse of `to_world`, and of `navigation.to_global`. This is the
    conversion that runs on every GUIDED target, because MAVLink's position
    targets are global and the planner's are not.
    """
    if not origin:
        return None
    lat = origin["lat"] + north / METRES_PER_DEGREE_LAT
    lon = origin["lon"] + east / (
        METRES_PER_DEGREE_LAT * math.cos(math.radians(origin["lat"]))
    )
    return lat, lon


# ------------------------------------------------------------ leg geometry

def project_onto_leg(point, leg_start, leg_end):
    """Where `point` falls along a leg. Returns `(t, along_m, cross_m)`.

    `t` is 0 at the start and 1 at the end, unclamped - **greater than 1 means
    the boat is past the end**, which is the passing-plane test that stops a
    waypoint laid slightly off the line trapping the boat in a circle around it.

    `along_m` is metres from the start along the leg, `cross_m` is signed
    distance off it: **positive means the point is to starboard of the leg
    direction**, which is the same sign convention as `relative_bearing`, so
    "positive is to my right" holds everywhere in this codebase.

    A zero-length leg returns `(1.0, 0.0, distance)` - already arrived, and the
    cross-track is just the range. That degenerate case is real: two waypoints
    can be laid on top of each other by a shaky hand on a phone.
    """
    dx = leg_end[0] - leg_start[0]
    dy = leg_end[1] - leg_start[1]
    length_sq = dx * dx + dy * dy
    px = point[0] - leg_start[0]
    py = point[1] - leg_start[1]
    if length_sq <= 1e-9:
        return 1.0, 0.0, math.hypot(px, py)
    t = (px * dx + py * dy) / length_sq
    length = math.sqrt(length_sq)
    along = t * length
    # 2-D cross product, then flipped: with +x east and +y north, (dx,dy) x
    # (px,py) is positive when the point is to the LEFT of the leg direction.
    cross = -(dx * py - dy * px) / length
    return t, along, cross


def lookahead_point(boat_xy, leg_start, leg_end, lookahead_m):
    """The pure-pursuit aim point: `lookahead_m` further along the leg.

    Steering straight at a waypoint makes a boat weave across the track, because
    the further off the line it is the more it turns, and it arrives with all
    that turn still wound in. Steering at a point a fixed distance ahead *on the
    line* makes it converge onto the line instead - the error and the correction
    go to zero together.

    Clamped to the leg's end, so the last few metres of a leg aim at the
    waypoint itself and the boat actually arrives rather than aiming past it.
    """
    dx = leg_end[0] - leg_start[0]
    dy = leg_end[1] - leg_start[1]
    length = math.hypot(dx, dy)
    if length <= 1e-9:
        return leg_end
    _t, along, _cross = project_onto_leg(boat_xy, leg_start, leg_end)
    target = min(length, max(0.0, along) + lookahead_m)
    return (leg_start[0] + dx * target / length, leg_start[1] + dy * target / length)


def offset_point(point, bearing_deg, distance_m):
    """A world point `distance_m` away from `point` on a compass bearing."""
    east, north = unit(bearing_deg)
    return (point[0] + east * distance_m, point[1] + north * distance_m)


# ---------------------------------------------------------------------- CPA

def closest_point_of_approach(own_xy, own_vel, other_xy, other_vel):
    """`(time_s, distance_m)` of the closest approach of two moving points.

    Straight-line extrapolation of both, which is what a CPA *is* - it answers
    "if neither of us does anything, how close do we get and when". A negative
    time means the closest approach is already behind us and the range is
    opening; callers must check that before reacting, or the boat will manoeuvre
    to avoid a vessel it has already passed.

    Velocities are world-frame `(east, north)` m/s.
    """
    rx = other_xy[0] - own_xy[0]
    ry = other_xy[1] - own_xy[1]
    vx = other_vel[0] - own_vel[0]
    vy = other_vel[1] - own_vel[1]
    speed_sq = vx * vx + vy * vy
    if speed_sq <= 1e-9:
        # No relative motion: the range now is the range forever.
        return 0.0, math.hypot(rx, ry)
    t = -(rx * vx + ry * vy) / speed_sq
    cx = rx + vx * t
    cy = ry + vy * t
    return t, math.hypot(cx, cy)
