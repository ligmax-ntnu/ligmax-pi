"""Every number the autonomy stack can be wrong about, in one file.

Two rules for this file:

  * **Nothing here is a magic constant.** Each value says where it came from -
    a Njord rule, a hardware measurement, or a judgement call - because at 08:00
    on a competition morning somebody will have to change one of them from a
    phone on a dock, and they need to know which ones are safe to touch.
  * **Everything is overridable from the environment**, so that change does not
    need a commit, a push and a `git pull` on the boat. Put the override in
    `/etc/ligmax/node.env` if it should survive a reboot.

The values that are *rules* (buoy diameter, gate width, task speed) are marked
NJORD and should only change if the handbook changes. The ones marked TUNE are
ours and are expected to move on the water.
"""

import os


def _f(name, default):
    """A float from the environment, or the default. Never raises."""
    try:
        return float(os.environ[name])
    except (KeyError, ValueError):
        return default


def _i(name, default):
    try:
        return int(float(os.environ[name]))
    except (KeyError, ValueError):
        return default


def _b(name, default):
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() not in ("0", "false", "no", "off", "")


def _s(name, default):
    value = os.environ.get(name, "").strip()
    return value or default


# ---------------------------------------------------------------- loop timing

# The autonomy tick. 10 Hz matches the lidar's own rotation rate, so every tick
# gets exactly one new sweep and no tick does the same work twice. Faster buys
# nothing - the sensor is the limit, not the CPU.
TICK_HZ = _f("LIGMAX_AP_TICK_HZ", 10.0)
TICK_PERIOD = 1.0 / TICK_HZ

# How often a GUIDED target is re-sent even when it has not changed. ArduPilot
# times out a guided command and falls back to holding, so this is a keepalive,
# not a correction.
TARGET_REFRESH_S = _f("LIGMAX_AP_TARGET_REFRESH_S", 0.5)


# ------------------------------------------------------------------- safety

# NJORD §7.3 requires autonomous movement to stop within 60 s of losing radio
# contact. We use 10: at the 2 kn task speed 60 s is ten metres of uncommanded
# travel, and the rule is a ceiling rather than a target. This counts from the
# last state frame received from io_manager, which is itself proof that the
# MAVLink link, the node bus and the process are all alive.
LOSS_OF_COMMS_STOP_S = _f("LIGMAX_AP_COMMS_TIMEOUT_S", 10.0)

# Stale state is worse than no state. If the newest navigation fix is older than
# this the pilot stops rather than steering on a remembered position.
MAX_NAV_AGE_S = _f("LIGMAX_AP_MAX_NAV_AGE_S", 2.0)

# NJORD §8.2: a boat with a problem gets a 20 s autonomous search window before
# the team must take over. A behaviour that has not made progress for this long
# says so loudly, so the operator can use those 20 s deliberately rather than
# discovering the situation at second 19.
STUCK_WARN_S = _f("LIGMAX_AP_STUCK_WARN_S", 12.0)

# The absolute ceiling on commanded speed, whatever a behaviour asks for. This
# is the last line between a planner bug and a boat crossing the course at full
# throttle.
MAX_SPEED_MS = _f("LIGMAX_AP_MAX_SPEED_MS", 1.6)


# -------------------------------------------------------------------- speeds

# NJORD §9.2 sets the collision-avoidance task speed at 2 knots and requires the
# boat to accelerate to it immediately at the start of an attempt.
KNOT_MS = 0.514444
TASK_SPEED_MS = _f("LIGMAX_AP_TASK_SPEED_MS", 2.0 * KNOT_MS)  # NJORD, 2 kn

# Cruise for the blind GNSS legs, where nothing is being scored on speed except
# the 9 % time multiplier. Deliberately close to the task speed: a fast leg that
# overshoots a waypoint costs more than it saves.
CRUISE_SPEED_MS = _f("LIGMAX_AP_CRUISE_SPEED_MS", 1.2)

# Around buoys and cardinal marks, where a misread mark has to be recoverable.
CAUTION_SPEED_MS = _f("LIGMAX_AP_CAUTION_SPEED_MS", 0.8)

# The final approach into a 2 m berth. TUNE: slow enough that a 0.5 s reaction
# is 15 cm of travel.
DOCK_SPEED_MS = _f("LIGMAX_AP_DOCK_SPEED_MS", 0.3)
DOCK_REVERSE_SPEED_MS = _f("LIGMAX_AP_DOCK_REVERSE_SPEED_MS", 0.25)

# Below this the boat is treated as stopped, for the "stay stationary" scores.
STATIONARY_SPEED_MS = _f("LIGMAX_AP_STATIONARY_SPEED_MS", 0.15)


# ---------------------------------------------------------------- navigation

# How close counts as reaching a waypoint. The Njord marks are laid by hand from
# a boat, and an RTK-fixed position is centimetre-accurate, so the limit here is
# the accuracy of the *course*, not of the boat. TUNE: 3 m is forgiving enough
# for a 3D fix and tight enough that the jury sees the boat pass the point.
WAYPOINT_RADIUS_M = _f("LIGMAX_AP_WAYPOINT_RADIUS_M", 3.0)

# A tighter one for the points that are scored as arrivals rather than as
# transits - GPS point 4 ("stop and stay stationary"), and the dock approach.
ARRIVAL_RADIUS_M = _f("LIGMAX_AP_ARRIVAL_RADIUS_M", 1.5)

# A waypoint is also considered passed once the boat is beyond the plane through
# it perpendicular to the leg, even if the radius was never satisfied. Without
# this a waypoint laid slightly off the line traps the boat in a circle around
# it forever - the single most common failure mode of naive waypoint following.
USE_PASSING_PLANE = _b("LIGMAX_AP_PASSING_PLANE", True)

# How far ahead on the leg to aim. Pure pursuit: steering at the waypoint itself
# makes the boat oscillate around the track, steering at a point a fixed
# distance ahead on the track makes it converge onto it. TUNE.
LOOKAHEAD_M = _f("LIGMAX_AP_LOOKAHEAD_M", 6.0)
LOOKAHEAD_MIN_M = _f("LIGMAX_AP_LOOKAHEAD_MIN_M", 2.5)


# ------------------------------------------------------------------ obstacles

# NJORD §10.2: the buoys are 40 cm across at the waterline. Anything the lidar
# reports much wider than this is not a buoy.
BUOY_DIAMETER_M = _f("LIGMAX_AP_BUOY_DIAMETER_M", 0.40)  # NJORD

# NJORD §9.2: the Otter is 2.0 x 1.08 m. A cluster longer than the widest buoy
# and shorter than a pier is a vessel, and that is the size cue that survives the
# handbook's warning that its colour and form may vary.
OTTER_LENGTH_M = _f("LIGMAX_AP_OTTER_LENGTH_M", 2.0)  # NJORD

# How wide to give a mark that must simply not be hit. Half the boat's beam plus
# the buoy's radius plus a margin for position error. TUNE on the water.
BUOY_CLEARANCE_M = _f("LIGMAX_AP_BUOY_CLEARANCE_M", 2.0)

# The same for a vessel. Larger, because it moves.
VESSEL_CLEARANCE_M = _f("LIGMAX_AP_VESSEL_CLEARANCE_M", 6.0)

# Returns beyond this are ignored for planning. The C1's datasheet range is 12 m
# and its returns get sparse and noisy well before that; a "buoy" at 11 m made of
# three points is a reflection off a wave.
MAX_OBSTACLE_RANGE_M = _f("LIGMAX_AP_MAX_RANGE_M", 10.0)

# Below this a return is the boat's own hull, its own ama, or spray.
MIN_OBSTACLE_RANGE_M = _f("LIGMAX_AP_MIN_RANGE_M", 0.45)


# ------------------------------------------------------------------ clustering

# Two returns further apart than this are different objects. At 10 m the C1's
# 0.9 deg step is 16 cm between neighbouring beams, so this has to exceed that or
# a single buoy at range splits into a scatter of one-point objects.
CLUSTER_GAP_M = _f("LIGMAX_AP_CLUSTER_GAP_M", 0.45)

# The gap grows with range, because the angular step does. gap = base + k*range.
CLUSTER_GAP_PER_M = _f("LIGMAX_AP_CLUSTER_GAP_PER_M", 0.03)

# Fewer returns than this and it is noise. A 40 cm buoy at 10 m subtends 2.3 deg,
# which is two or three beams - so this cannot be raised much without going blind
# to buoys at the range where seeing them first matters.
MIN_CLUSTER_POINTS = _i("LIGMAX_AP_MIN_CLUSTER_POINTS", 2)

# A cluster wider than this is a wall, a pier or the shore, not a mark.
MAX_MARK_WIDTH_M = _f("LIGMAX_AP_MAX_MARK_WIDTH_M", 1.2)


# ------------------------------------------------------------------- colour

# Colour thresholds in HSV, applied to the front lidar's per-point RGB. These are
# sensor-native values straight off the OV5647 - the colour matrix runs at the
# receiver, so they are NOT calibrated colours. TUNE THESE ON THE WATER, in the
# day's actual light: the same buoy reads differently at 09:00 and at 15:00.
#
# Hue is degrees 0-360. Red wraps, so it is two ranges.
HUE_RED_LOW_MAX = _f("LIGMAX_AP_HUE_RED_LOW_MAX", 20.0)
HUE_RED_HIGH_MIN = _f("LIGMAX_AP_HUE_RED_HIGH_MIN", 335.0)
HUE_GREEN_MIN = _f("LIGMAX_AP_HUE_GREEN_MIN", 75.0)
HUE_GREEN_MAX = _f("LIGMAX_AP_HUE_GREEN_MAX", 175.0)
HUE_YELLOW_MIN = _f("LIGMAX_AP_HUE_YELLOW_MIN", 35.0)
HUE_YELLOW_MAX = _f("LIGMAX_AP_HUE_YELLOW_MAX", 70.0)

# Below this saturation a point has no usable hue: it is white, grey or black.
#
# MEASURED, 2026-08-08, on 6879 coloured returns from a real capture indoors:
# median saturation 0.42, p90 0.60, and the whole scene's mean RGB was
# (80, 48, 44) - a strong warm cast, because these are sensor-native values with
# NO white balance applied (`edge_protocol.py`). At the old threshold of 0.28
# every return in that scene passed as "a colour", and 46 of 49 clusters
# classified as RED. Ordinary warm-lit surfaces are not red buoys.
#
# RAL 3001 red, neon green and RAL 1003 yellow are all strongly saturated - well
# above 0.7 in daylight - so raising this separates a painted mark from a tinted
# grey while costing nothing on the marks themselves. The failure mode of too
# high is UNKNOWN, which is avoided on both sides; the failure mode of too low is
# a confident wrong-side pass. Prefer too high.
MIN_SATURATION = _f("LIGMAX_AP_MIN_SATURATION", 0.55)

# Grey-world white balance, per sweep, before the hue is taken: divide each
# channel by its own mean across the sweep, which cancels a global colour cast.
#
# OFF by default, and this is a judgement call worth understanding. On the water
# it is the right correction - a sweep is mostly sea, sky and land, so their
# average IS approximately grey and a buoy is a few returns out of hundreds.
# But it fails badly in the degenerate case where one coloured object fills the
# view, because the correction then neutralises the very thing being measured.
# Turn it on if the day's captures show a cast that `MIN_SATURATION` alone is
# not handling; the real fix is white balance on the Jetson, where the sensor
# and its colour matrix actually are.
WHITE_BALANCE = _b("LIGMAX_AP_WHITE_BALANCE", False)

# How far a grey-world gain may go. A nearly monochrome sweep would otherwise
# produce enormous gains and invent colour out of noise.
WHITE_BALANCE_MAX_GAIN = _f("LIGMAX_AP_WHITE_BALANCE_MAX_GAIN", 1.8)

# White (a dock, a hull) vs dark (water, a shadow, an unlit object).
WHITE_MIN_VALUE = _f("LIGMAX_AP_WHITE_MIN_VALUE", 0.55)
DARK_MAX_VALUE = _f("LIGMAX_AP_DARK_MAX_VALUE", 0.22)

# What fraction of a cluster's coloured points must agree before the cluster is
# given that colour. Below it the cluster is UNKNOWN, which is safe: an unknown
# obstacle is avoided on both sides rather than passed on the wrong one.
COLOUR_VOTE_FRACTION = _f("LIGMAX_AP_COLOUR_VOTE_FRACTION", 0.5)

# A cluster needs at least this many coloured returns to be classified at all.
MIN_COLOURED_POINTS = _i("LIGMAX_AP_MIN_COLOURED_POINTS", 2)


# ------------------------------------------------------------------ tracking

# How much a new measurement moves a track. Low means smooth and laggy, high
# means twitchy. Marks are nailed to the seabed so they can be smoothed hard;
# a vessel cannot.
TRACK_ALPHA_STATIC = _f("LIGMAX_AP_TRACK_ALPHA_STATIC", 0.35)
TRACK_ALPHA_VESSEL = _f("LIGMAX_AP_TRACK_ALPHA_VESSEL", 0.6)

# Association gate: a detection matches a track within this distance, growing
# with range because a bearing error is an arc.
TRACK_GATE_M = _f("LIGMAX_AP_TRACK_GATE_M", 1.5)
TRACK_GATE_PER_M = _f("LIGMAX_AP_TRACK_GATE_PER_M", 0.12)

# Confidence gained per hit and lost per second unseen. A track has to be seen
# on several consecutive sweeps before it is steered around, and survives a few
# seconds of occlusion - a buoy hidden behind the Otter has not ceased to exist.
TRACK_CONFIRM_HITS = _i("LIGMAX_AP_TRACK_CONFIRM_HITS", 3)
TRACK_DECAY_PER_S = _f("LIGMAX_AP_TRACK_DECAY_PER_S", 0.35)
TRACK_DROP_AFTER_S = _f("LIGMAX_AP_TRACK_DROP_AFTER_S", 6.0)

# How many independent camera votes before a cardinal mark's direction is
# believed. The detector is known to be weak, so this is deliberately high: the
# cost of waiting is a slower pass, the cost of being wrong is passing on the
# wrong side of a mark, which is the whole point of the task.
CARDINAL_VOTES_REQUIRED = _i("LIGMAX_AP_CARDINAL_VOTES", 4)
CARDINAL_MIN_CONF = _f("LIGMAX_AP_CARDINAL_MIN_CONF", 0.55)


# ------------------------------------------------------------------- COLREG

# NJORD §9.2: the Otter closes at 2.5 kn from anywhere in +-100 deg.
# Start thinking about a vessel at this range.
COLREG_DETECT_RANGE_M = _f("LIGMAX_AP_COLREG_DETECT_M", 25.0)

# Closest point of approach we are willing to accept before acting.
COLREG_MIN_CPA_M = _f("LIGMAX_AP_COLREG_MIN_CPA_M", 8.0)

# ...and how far ahead in time to look for it. Beyond this the Otter's track is
# extrapolation, not prediction.
COLREG_HORIZON_S = _f("LIGMAX_AP_COLREG_HORIZON_S", 25.0)

# Rule 14 head-on: reciprocal courses within this of dead ahead.
COLREG_HEADON_DEG = _f("LIGMAX_AP_COLREG_HEADON_DEG", 15.0)

# Rule 8: an alteration must be "large enough to be readily apparent to another
# vessel observing visually". A jury on the dock is that observer.
COLREG_TURN_DEG = _f("LIGMAX_AP_COLREG_TURN_DEG", 45.0)

# Absolute last resort, whatever the rules say: Rule 2, the ordinary practice of
# seamen. Inside this range everything else is abandoned and the boat backs off.
COLREG_PANIC_M = _f("LIGMAX_AP_COLREG_PANIC_M", 4.0)


# ------------------------------------------------------------------- docking

# NJORD §9.3 berth sizes.
DOCK_BERTH_WIDTH_M = _f("LIGMAX_AP_BERTH_WIDTH_M", 2.0)  # NJORD, normal
DOCK_BERTH_PARALLEL_M = _f("LIGMAX_AP_BERTH_PARALLEL_M", 4.0)  # NJORD, parallel

# How long to hold, per the rules.
DOCK_HOLD_S = _f("LIGMAX_AP_DOCK_HOLD_S", 10.0)  # NJORD §9.3, normal
DOCK_PARALLEL_HOLD_S = _f("LIGMAX_AP_DOCK_PARALLEL_HOLD_S", 5.0)  # NJORD, parallel

# Where the approach starts: hold station here, square up, then run in.
DOCK_STANDOFF_M = _f("LIGMAX_AP_DOCK_STANDOFF_M", 6.0)

# How far into the berth to drive, measured from the berth mouth.
DOCK_ENTRY_DEPTH_M = _f("LIGMAX_AP_DOCK_ENTRY_DEPTH_M", 1.2)

# How far back out. Slightly more than the entry, so the boat is clear.
DOCK_EXIT_M = _f("LIGMAX_AP_DOCK_EXIT_M", 4.0)

# Berth detection: a gap between two structures this wide, plus or minus, is a
# candidate berth. The tolerance is generous because a floating dock moves.
DOCK_GAP_TOLERANCE_M = _f("LIGMAX_AP_DOCK_GAP_TOL_M", 0.7)

# The minimum extent of a structure either side of the gap for it to be a berth
# wall rather than two buoys with a coincidental spacing.
DOCK_WALL_MIN_M = _f("LIGMAX_AP_DOCK_WALL_MIN_M", 0.8)

# How square to the berth the boat must be before it commits to entering.
DOCK_ALIGN_TOLERANCE_DEG = _f("LIGMAX_AP_DOCK_ALIGN_DEG", 12.0)

# How far off the berth centreline is acceptable on entry. A 2 m berth and a
# boat with amas leaves very little; this is the number to measure the hull
# against before the first attempt.
DOCK_LATERAL_TOLERANCE_M = _f("LIGMAX_AP_DOCK_LATERAL_M", 0.25)


# ------------------------------------------------------- lateral thruster

# The boat has two main thrusters (one per ama) and a third, sideways-only unit
# for slow horizontal movement. There are two ways to reach it and which one is
# right depends on how the flight controller is configured:
#
#   "mavlink"  ArduPilot owns it as a lateral motor output, so the `vy` term of
#              a GUIDED body-frame velocity command drives it. Costs nothing if
#              the autopilot is NOT configured that way - it ignores vy.
#   "rc"       the Pi drives it directly with an RC override on
#              LIGMAX_LATERAL_RC_CHAN, the way `io_manager/pixhalwk.py` drives
#              the ride height on channel 16.
#   "none"     no lateral thrust. Parallel docking falls back to an angled
#              approach and a pivot, which is slower and less tidy but works.
#
# Default is "mavlink" because it is the only one that is safe when unverified:
# an unconfigured autopilot drops the term, whereas a wrong RC channel drives
# something else on the boat.
LATERAL_MODE = _s("LIGMAX_LATERAL_MODE", "mavlink").lower()
LATERAL_RC_CHAN = _i("LIGMAX_LATERAL_RC_CHAN", 0)  # 0 = unset, refuse to guess
LATERAL_RC_CENTRE = _i("LIGMAX_LATERAL_RC_CENTRE", 1500)
LATERAL_RC_SPAN = _i("LIGMAX_LATERAL_RC_SPAN", 400)
LATERAL_MAX_MS = _f("LIGMAX_AP_LATERAL_MAX_MS", 0.35)


# ------------------------------------------------------------------ steering

# Heading controller for the body-velocity behaviours (docking, station keeping,
# avoidance). Transit does not use these - ArduPilot's own L1 controller steers
# a position target, and it is better tuned than anything written here would be.
YAW_P = _f("LIGMAX_AP_YAW_P", 0.030)  # rad/s per degree of error
YAW_MAX_RATE = _f("LIGMAX_AP_YAW_MAX_RATE", 0.6)  # rad/s
YAW_DEADBAND_DEG = _f("LIGMAX_AP_YAW_DEADBAND_DEG", 3.0)

# Station keeping: how far the boat may drift before it is pulled back.
HOLD_TOLERANCE_M = _f("LIGMAX_AP_HOLD_TOLERANCE_M", 1.0)
HOLD_P = _f("LIGMAX_AP_HOLD_P", 0.35)  # m/s per metre of error


# ------------------------------------------------------------------- links

# ZeroMQ. io_manager publishes the boat's state on IO_PORT; this node publishes
# control requests and its own telemetry on SELF_DRIVING_PORT. Both are
# loopback-only - there is no route to either from off the vessel.
IO_STATE_PORT = _i("LIGMAX_IO_STATE_PORT", 5557)
SELF_DRIVING_PORT = _i("LIGMAX_SELF_DRIVING_PORT", 5559)

# This node binds TCP 3401 and takes the Jetson's feed directly, rather than
# having io_manager relay it: it is the only consumer that needs the full 10 Hz
# coloured cloud and the detections, and a relay would serialise ~10 kB ten
# times a second for nothing. io_manager still gets a copy for the operator's
# plot, pushed back over SELF_DRIVING_PORT.
# LIGMAX_EDGE_OWNER is read on both sides and only one of them may bind: see
# `nodes/io_manager/edge_link.py`, which reads the same variable and stays off
# unless it says "io_manager".
EDGE_PORT = _i("LIGMAX_EDGE_PORT", 3401)
OWN_EDGE_LINK = _s("LIGMAX_EDGE_OWNER", "self_driving") == "self_driving"


# ------------------------------------------------------------------ recording

# Every run is recorded. Two attempts and fifteen minutes means the only way to
# fix attempt two is to know exactly what attempt one did.
RECORD_ENABLED = _b("LIGMAX_AP_RECORD", True)
RECORD_DIR = _s("LIGMAX_AP_RECORD_DIR", "/home/admin/ligmax-trips")

# Full state, including every lidar point, at this rate. The cloud is most of
# the bytes, so it gets its own (slower) rate than the pose and decision log.
RECORD_HZ = _f("LIGMAX_AP_RECORD_HZ", 10.0)
RECORD_SCAN_HZ = _f("LIGMAX_AP_RECORD_SCAN_HZ", 2.0)

# Keep at most this many trips on the Pi's SD card.
RECORD_KEEP_TRIPS = _i("LIGMAX_AP_RECORD_KEEP", 40)


# --------------------------------------------------------------- persistence

# The plan survives a node restart. At 08:55, having typed the morning's course
# in once, nobody wants to do it again because a process bounced.
PLAN_FILE = _s("LIGMAX_AP_PLAN_FILE", "/home/admin/.ligmax/plan.json")


def snapshot():
    """Everything above, for the trip recording's header.

    A trip is only reviewable if you know what the boat believed at the time,
    and half of that is which numbers it was running.
    """
    return {
        name: value
        for name, value in sorted(globals().items())
        if name.isupper() and isinstance(value, (int, float, str, bool))
    }
