"""Position, heading, course and distance-to-waypoint, from MAVLink.

Feeds the four navigation figures the Njord GUI requirement asks for by name -
latitude, longitude, heading, course over ground with speed over ground - plus the
distance to the next waypoint, and the cross-track error that says how far off the
ideal route the boat actually is.

Where each number comes from, and why it matters which
------------------------------------------------------
    GPS_RAW_INT             lat, lon, fix quality, satellites, HDOP, and the
                            receiver's own COG and ground speed. This is the
                            GNSS module talking, before the autopilot's EKF has
                            an opinion.
    GLOBAL_POSITION_INT     the fused position and the NED velocity, i.e. the
                            EKF's answer. Better filtered, and wrong in a
                            different way when the filter is unhappy.
    VFR_HUD                 heading in whole degrees and ground speed. Cheap,
                            and the fallback when hdg is not being sent.
    NAV_CONTROLLER_OUTPUT   wp_dist and xtrack_error - the autopilot's own view
                            of how the mission is going.
    MISSION_CURRENT         which waypoint that distance is *to*.
    ATTITUDE                roll, pitch and their rates, from the EKF.

**Why attitude is in a navigation module at all.** It is not a navigation figure
and it is here because it is a *geometric* one, and this is the module that owns
turning the autopilot's messages into geometry. Two consumers need it and neither
existed when this file was written:

  * **anything measured off a camera.** A marker's pose comes out of `solvePnP` in
    the camera frame, and turning that into "the berth is 3 m ahead and 0.4 m to
    port" needs to know which way is down. Assume level and the cross-track error
    is `range * sin(roll)`: at 4 m and 5 degrees that is 0.35 m, inside a 2 m
    berth. The camera calibration cannot help with this - a Kannala-Brandt fit is
    intrinsics, it says what the lens does and nothing about where it points.
  * **the stabiliser, judged rather than trusted.** This hull's whole claim is
    that it holds roll and pitch; until this was published, nothing on shore could
    say by how much, and `SCR_USER5`/`SCR_USER6` were being tuned from the
    dashboard against no readback of the thing they move.

Published as its own `attitude` sub-block rather than folded into `motion`,
because `motion` is where-the-boat-is-going and this is how-the-hull-is-lying;
they go wrong independently and an operator reads them for different reasons.

**Heading and COG are different numbers and the dashboard shows both on purpose.**
Heading is where the bow points; COG is where the boat is actually going. On a
light trimaran in a Trondheim tide the gap between them is several degrees, and
that gap - `crab_deg` - is the thing that explains an otherwise baffling
cross-track error. Publishing only one of them would hide it.

COG is deliberately **not** published below COG_MIN_SPEED. The direction of a
near-zero velocity is noise, and a course readout spinning while the boat sits
alongside teaches the operator to distrust the panel.

Preference order is the receiver's own figure first, the EKF second. That is the
opposite of what you would want for control and the right way round for a display:
the operator is being shown what the sensor measured, and the sim, the dashboard
and this module all agree on that so a discrepancy means something.

The grid, and why this module owns it
-------------------------------------
The dashboard's chart is not drawn in degrees. It draws a local metre grid and
lays map imagery under it, which needs two things the raw GNSS figures are not:
``origin`` - the lat/lon of grid (0, 0) - and ``boat.position`` in metres from
it. Publishing `telemetry.gps.lat/lon` alone puts numbers in the figure list and
leaves the map empty, because nothing else in the fleet converts one to the
other (`ligmax-server/web/js/geo.js` does it client-side, but only for the
cursor readout).

So this module captures the first usable fix as the origin - the same thing
`Boat.original_gps_position` means - and reports position relative to it as a
tangent plane, +x east, +y north, which is the protocol's default grid. Over a
Njord course a few hundred metres across the flat-earth error is well under a
metre, and the constant is deliberately the same 111320 m/deg the dashboard
uses, so the two ends cannot disagree about where the boat is.

The origin outlives a MAVLink dropout and a node restart (it is cached in
`ORIGIN_FILE`) because it is a georeference, not a measurement: re-zeroing it
mid-run would silently shift the whole chart, the track history and every
obstacle under the boat. `recentre()` - the dashboard's `recentre_origin`
command - is the only way to move it, and a reboot clears the cache.

Nothing here blocks or raises; a missing message means a missing field, and the
dashboard shows a gap rather than a stale number.
"""

import json
import logging
import math
import os

log = logging.getLogger("io_manager.navigation")

# GPS_FIX_TYPE. Names match the `goodValues`/`warnValues` the dashboard's GNSS
# widget already tests against (`ligmax-server/web/js/telemetry.js`).
FIX_TYPES = {
    0: "NO_GPS",
    1: "NO_FIX",
    2: "2D",
    3: "3D",
    4: "DGPS",
    5: "RTK_FLOAT",
    6: "RTK_FIXED",
    7: "STATIC",
    8: "PPP",
}

# Below this, course over ground is not reported at all. Roughly a slow drift:
# fast enough that the direction means something, slow enough not to blank the
# field during a gentle station-keep.
COG_MIN_SPEED = float(os.environ.get("LIGMAX_COG_MIN_SPEED", "0.15"))

# MAVLink "not known" sentinels for the fields this module reads.
UINT16_MAX = 0xFFFF
INT16_MAX = 0x7FFF

# Metres per degree of latitude. Must stay equal to METRES_PER_DEGREE_LAT in
# `ligmax-server/web/js/geo.js`, which converts back the other way for the
# cursor readout - a different constant here would put the boat and the mouse
# pointer in two slightly different worlds.
METRES_PER_DEGREE_LAT = 111320.0

# Fixes too coarse to zero a grid on. A 2D fix can sit tens of metres out, and
# the offset would move the satellite imagery under the whole course, not just
# the boat. lat/lon are still reported while we wait.
UNUSABLE_FIXES = ("NO_GPS", "NO_FIX", "2D")

# Where the captured origin is cached, so an io_manager restart rejoins the grid
# it left rather than re-zeroing under the operator. /run is tmpfs: cleared by a
# reboot, which is exactly when "the fix it booted at" should mean a new one.
ORIGIN_FILE = os.environ.get("LIGMAX_GRID_ORIGIN_FILE", "/run/ligmax/grid-origin.json")


def _wrap360(degrees):
    return degrees % 360.0


def _wrap180(degrees):
    """Signed difference, wrapped to (-180, 180]. Used for the crab angle."""
    return ((degrees + 540.0) % 360.0) - 180.0


def _load_origin():
    """The cached grid origin, or None. Never raises - a bad cache is no cache."""
    try:
        with open(ORIGIN_FILE, "r", encoding="utf-8") as handle:
            cached = json.load(handle)
        lat, lon = float(cached["lat"]), float(cached["lon"])
    except (OSError, ValueError, TypeError, KeyError):
        return None
    if abs(lat) > 90.0 or abs(lon) > 180.0:
        log.warning("ignoring nonsense cached grid origin %s", ORIGIN_FILE)
        return None
    log.info("grid origin restored from cache: %.7f, %.7f", lat, lon)
    return {"lat": lat, "lon": lon}


def _save_origin(origin):
    """Cache the origin so a node restart rejoins the same grid. Best effort."""
    try:
        os.makedirs(os.path.dirname(ORIGIN_FILE) or ".", exist_ok=True)
        with open(ORIGIN_FILE, "w", encoding="utf-8") as handle:
            json.dump(origin, handle)
    except OSError as exc:
        # Not fatal: the grid is still correct for as long as this process runs,
        # and the operator would rather have a map than an exception.
        log.warning("could not cache grid origin to %s: %s", ORIGIN_FILE, exc)


def _forget_origin():
    try:
        os.remove(ORIGIN_FILE)
    except OSError:
        pass


class Navigation:
    """Accumulates MAVLink navigation messages into one telemetry block.

    `handle()` is called from the MAVLink pump for every message; unrecognised
    types are ignored. `telemetry()` returns `{"gps": {...}, "motion": {...}}`
    ready to merge into a frame - the dashboard merges telemetry two levels deep
    (`ligmax-server/ligmax_gui/state.py`), so this can own those two sub-blocks
    without clobbering what other tasks put in them.
    """

    def __init__(self):
        self._gps_raw = None
        self._global = None
        self._hud = None
        self._nav = None
        self._attitude = None
        self._mission_seq = None
        self._warned_no_position = False
        self._origin = _load_origin()

    # -- fed by the MAVLink pump -------------------------------------------

    def handle(self, message):
        """Absorb one MAVLink message. Returns True if it was one of ours."""
        kind = message.get_type()
        if kind == "GPS_RAW_INT":
            self._gps_raw = message
        elif kind == "GLOBAL_POSITION_INT":
            self._global = message
        elif kind == "VFR_HUD":
            self._hud = message
        elif kind == "NAV_CONTROLLER_OUTPUT":
            self._nav = message
        elif kind == "ATTITUDE":
            self._attitude = message
        elif kind == "MISSION_CURRENT":
            self._mission_seq = getattr(message, "seq", None)
        else:
            return False
        return True

    def link_down(self):
        """Forget everything. A dropped link must not leave a stale position up.

        This matters more here than anywhere else on the boat: a chart showing the
        vessel where it was thirty seconds ago is worse than a chart showing
        nothing, because it looks correct.

        The origin is deliberately kept. It is a georeference rather than a
        measurement, and dropping it would move the grid - and with it every
        track and the operator's track history - the moment the link came back.
        """
        self._gps_raw = None
        self._global = None
        self._hud = None
        self._nav = None
        self._attitude = None
        self._mission_seq = None

    # -- derived figures ----------------------------------------------------

    @property
    def position(self):
        """`(lat, lon)` in degrees, or None. Receiver first, then the EKF."""
        for source in (self._gps_raw, self._global):
            if source is None:
                continue
            lat = getattr(source, "lat", None)
            lon = getattr(source, "lon", None)
            # 0/0 is the null island the autopilot reports before a fix, not a
            # position in the Gulf of Guinea.
            if not lat or not lon:
                continue
            return lat / 1e7, lon / 1e7
        return None

    @property
    def sog(self):
        """Speed over ground, m/s. The receiver's own figure, then the EKF's."""
        if self._gps_raw is not None:
            vel = getattr(self._gps_raw, "vel", None)
            if vel is not None and vel != UINT16_MAX:
                return vel / 100.0  # cm/s
        if self._global is not None:
            vx = getattr(self._global, "vx", None)
            vy = getattr(self._global, "vy", None)
            if vx is not None and vy is not None:
                return math.hypot(vx, vy) / 100.0
        if self._hud is not None:
            speed = getattr(self._hud, "groundspeed", None)
            if speed is not None and math.isfinite(speed):
                return float(speed)
        return None

    @property
    def cog(self):
        """Course over ground in degrees, or None when too slow to mean anything."""
        speed = self.sog
        if speed is None or speed < COG_MIN_SPEED:
            return None

        if self._gps_raw is not None:
            course = getattr(self._gps_raw, "cog", None)
            # 65535 is "not available"; the field is centidegrees.
            if course is not None and course != UINT16_MAX:
                return _wrap360(course / 100.0)
        if self._global is not None:
            vx = getattr(self._global, "vx", None)
            vy = getattr(self._global, "vy", None)
            if vx is not None and vy is not None and (vx or vy):
                # NED: vx is north, vy is east, so the compass bearing is atan2(E, N).
                return _wrap360(math.degrees(math.atan2(vy, vx)))
        return None

    @property
    def heading(self):
        """Heading in degrees - where the bow points, not where the boat goes."""
        if self._global is not None:
            hdg = getattr(self._global, "hdg", None)
            if hdg is not None and hdg != UINT16_MAX:
                return _wrap360(hdg / 100.0)  # centidegrees
        if self._hud is not None:
            hdg = getattr(self._hud, "heading", None)
            if hdg is not None:
                return _wrap360(float(hdg))  # whole degrees
        return None

    # -- the grid the map is drawn in ---------------------------------------

    @property
    def fix(self):
        """The fix type as a name (`3D`, `RTK_FIXED`, ...), or None if unknown."""
        if self._gps_raw is None:
            return None
        raw = getattr(self._gps_raw, "fix_type", None)
        if raw is None:
            return None
        return FIX_TYPES.get(int(raw), f"FIX_{raw}")

    @property
    def origin(self):
        """`{"lat": ..., "lon": ...}` of grid (0, 0), or None until a fix lands."""
        return dict(self._origin) if self._origin else None

    def recentre(self):
        """Drop the origin so the next usable fix re-zeros the grid.

        Wired to the dashboard's `recentre_origin` command. Everything on the
        chart moves when this takes effect, which is why it is an explicit
        operator action and not something a reconnect does on its own.
        """
        self._origin = None
        _forget_origin()
        log.warning("grid origin cleared; the next usable fix will re-zero it")

    def _ensure_origin(self, position):
        """Capture `position` as the origin if we have not got one yet."""
        if self._origin is not None:
            return
        if (fix := self.fix) in UNUSABLE_FIXES:
            return  # lat/lon still get published; the grid can wait for a 3D fix
        self._origin = {"lat": position[0], "lon": position[1]}
        _save_origin(self._origin)
        log.info(
            "grid origin set to %.7f, %.7f on a %s fix",
            position[0],
            position[1],
            fix or "unreported",
        )

    @property
    def grid_position(self):
        """`[east, north]` metres from the origin, or None.

        Flat-earth about the origin's latitude - the same approximation, and the
        same constant, the dashboard uses in the other direction.
        """
        position = self.position
        if position is None:
            return None
        self._ensure_origin(position)
        if self._origin is None:
            return None
        north = (position[0] - self._origin["lat"]) * METRES_PER_DEGREE_LAT
        east = (
            (position[1] - self._origin["lon"])
            * METRES_PER_DEGREE_LAT
            * math.cos(math.radians(self._origin["lat"]))
        )
        return [round(east, 2), round(north, 2)]

    def to_global(self, x, y):
        """Grid metres `(x east, y north)` -> `(lat, lon)` degrees, or None.

        The exact inverse of `grid_position`, for the one thing that needs to
        go the other way: an admin lays a waypoint on the chart in grid
        metres, and the autopilot's mission protocol wants lat/lon
        (`mission.py`). None until a fix has set the origin - a mission
        cannot be georeferenced onto a grid that has no origin yet.

        Like `grid_position`, this ignores `grid_bearing` because nothing in
        this fleet ever sets it to anything but 0 (see `world()`) - if that
        changes, both conversions need the same rotation added.
        """
        if self._origin is None:
            return None
        lat = self._origin["lat"] + y / METRES_PER_DEGREE_LAT
        lon = self._origin["lon"] + x / (
            METRES_PER_DEGREE_LAT * math.cos(math.radians(self._origin["lat"]))
        )
        return lat, lon

    def world(self):
        """The map fields: `{"origin": ..., "boat": ...}`, or `{"boat": None}`.

        `boat` is explicitly null when there is no position, because the server
        merges frames: leaving the key out would keep the last known position on
        the chart for the rest of the run, which is the one failure mode this
        module exists to avoid. `grid_bearing` is left alone - the grid is
        north-aligned, which is the protocol's default.
        """
        grid = self.grid_position
        if grid is None:
            return {"boat": None}

        boat = {"position": grid}
        if (heading := self.heading) is not None:
            # Compass degrees; the server turns it into a grid unit vector.
            boat["heading_deg"] = round(heading, 1)
        if (course := self.cog) is not None and (speed := self.sog) is not None:
            # Only with a COG: below COG_MIN_SPEED the direction is noise, and
            # the chart's velocity arrow would spin on the spot.
            rad = math.radians(course)
            boat["velocity"] = [
                round(speed * math.sin(rad), 3),
                round(speed * math.cos(rad), 3),
            ]
        return {"origin": self.origin, "boat": boat}

    # -- output -------------------------------------------------------------

    def telemetry(self):
        """`{"gps": {...}, "motion": {...}}`, omitting anything not measured."""
        gps = {}
        motion = {}

        if (position := self.position) is not None:
            gps["lat"] = round(position[0], 7)
            gps["lon"] = round(position[1], 7)
        elif not self._warned_no_position and (self._gps_raw or self._global):
            self._warned_no_position = True
            log.warning("autopilot is reporting no GNSS position yet")

        if self._gps_raw is not None:
            if (fix := self.fix) is not None:
                gps["fix"] = fix
            satellites = getattr(self._gps_raw, "satellites_visible", None)
            if satellites is not None and satellites != 255:
                gps["satellites"] = int(satellites)
            # eph is HDOP * 100, UINT16_MAX when unknown.
            eph = getattr(self._gps_raw, "eph", None)
            if eph is not None and eph != UINT16_MAX:
                gps["hdop"] = round(eph / 100.0, 2)

        altitude = None
        if self._global is not None:
            # relative_alt is above the home point, which for a boat is the number
            # that means something. Absolute altitude over the geoid is not useful
            # here and is the noisier of the two.
            relative = getattr(self._global, "relative_alt", None)
            if relative is not None:
                altitude = relative / 1000.0  # mm
        if altitude is not None:
            gps["altitude"] = round(altitude, 2)

        if (speed := self.sog) is not None:
            motion["sog"] = round(speed, 3)
        if (heading := self.heading) is not None:
            motion["heading_deg"] = round(heading, 1)
        if (course := self.cog) is not None:
            motion["cog_deg"] = round(course, 1)
            # Only meaningful with both, and only published then - a crab angle
            # against a missing heading would be a number made of one number.
            if heading is not None:
                motion["crab_deg"] = round(_wrap180(heading - course), 1)

        if self._hud is not None:
            speed = getattr(self._hud, "groundspeed", None)
            if speed is not None and math.isfinite(speed):
                # The EKF's speed, kept beside the GNSS one: the planner steers on
                # this, and the two disagreeing is a symptom worth being able to see.
                motion["speed"] = round(float(speed), 3)

        if self._nav is not None:
            # wp_dist is metres, uint16. 0 is a real value (arrived), so it is not
            # a sentinel - unlike most of MAVLink, this one means what it says.
            distance = getattr(self._nav, "wp_dist", None)
            if distance is not None and distance != UINT16_MAX:
                motion["distance_to_target"] = float(distance)
            error = getattr(self._nav, "xtrack_error", None)
            if error is not None and math.isfinite(error):
                motion["cross_track_error"] = round(float(error), 3)
            bearing = getattr(self._nav, "target_bearing", None)
            if bearing is not None and bearing != INT16_MAX:
                motion["bearing_to_target"] = round(_wrap360(float(bearing)), 1)

        attitude = {}
        if self._attitude is not None:
            # ATTITUDE is radians, and every consumer of this wants degrees: the
            # panel shows degrees, `rig.json`'s mounting angles are degrees, and
            # the tuning parameters an operator sets against this readback are
            # degrees. Converted once, here, rather than in each of them.
            for name, field in (
                ("roll_deg", "roll"),
                ("pitch_deg", "pitch"),
                ("yaw_deg", "yaw"),
            ):
                value = getattr(self._attitude, field, None)
                if value is not None and math.isfinite(value):
                    degrees = math.degrees(float(value))
                    # Yaw is a compass bearing and wraps; roll and pitch are
                    # signed deflections from level and must NOT be wrapped to
                    # 0-360, or a 2 degree list to port reads as 358.
                    attitude[name] = (
                        round(_wrap360(degrees), 1)
                        if name == "yaw_deg"
                        else round(_wrap180(degrees), 2)
                    )
            for name, field in (
                ("roll_rate_dps", "rollspeed"),
                ("pitch_rate_dps", "pitchspeed"),
            ):
                value = getattr(self._attitude, field, None)
                if value is not None and math.isfinite(value):
                    # The rates are what tell a stabiliser that is *fighting* the
                    # sea from one that is riding it: a hull held at 1 degree and
                    # a hull passing through 1 degree at 20 deg/s read the same
                    # from the angles alone.
                    attitude[name] = round(math.degrees(float(value)), 1)

        out = {}
        if gps:
            out["gps"] = gps
        if motion:
            out["motion"] = motion
        if attitude:
            out["attitude"] = attitude
        if self._mission_seq is not None:
            # Which waypoint `distance_to_target` is measured to. Without it the
            # distance is a number with no referent.
            out["autonomy"] = {"waypoint": int(self._mission_seq)}
        return out
