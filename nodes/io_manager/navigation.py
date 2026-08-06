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

Nothing here blocks or raises; a missing message means a missing field, and the
dashboard shows a gap rather than a stale number.
"""

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


def _wrap360(degrees):
    return degrees % 360.0


def _wrap180(degrees):
    """Signed difference, wrapped to (-180, 180]. Used for the crab angle."""
    return ((degrees + 540.0) % 360.0) - 180.0


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
        self._mission_seq = None
        self._warned_no_position = False

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
        """
        self._gps_raw = None
        self._global = None
        self._hud = None
        self._nav = None
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
            fix = getattr(self._gps_raw, "fix_type", None)
            if fix is not None:
                gps["fix"] = FIX_TYPES.get(int(fix), f"FIX_{fix}")
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

        out = {}
        if gps:
            out["gps"] = gps
        if motion:
            out["motion"] = motion
        if self._mission_seq is not None:
            # Which waypoint `distance_to_target` is measured to. Without it the
            # distance is a number with no referent.
            out["autonomy"] = {"waypoint": int(self._mission_seq)}
        return out
