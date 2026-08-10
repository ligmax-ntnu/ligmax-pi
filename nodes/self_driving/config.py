"""Every number the autonomy stack can be wrong about, in one file.

Two rules for this file:

  * **Nothing here is a magic constant.** Each value says where it came from -
    a Njord rule, a hardware measurement, or a judgement call - because at 08:00
    on a competition morning somebody will have to change one of them from a
    phone on a dock, and they need to know which ones are safe to touch.
  * **Almost everything is overridable from the environment**, so that change
    does not need a commit, a push and a `git pull` on the boat. Put the
    override in `/etc/ligmax/node.env` if it should survive a reboot.

The values that are *rules* (buoy diameter, gate width, task speed) are marked
NJORD and should only change if the handbook changes. The ones marked TUNE are
ours and are expected to move on the water.

The one exception, and why
--------------------------
`SPEED_LIMIT_KNOTS` is **5 knots and is not overridable from the environment.**
It is a limit imposed on the vessel rather than a number we tune, and a limit
that can be raised from a phone on a dock is not a limit. Every speed below is
clamped to it as it is read (`_speed`), so no amount of environment fiddling can
raise any of them past it. See the block where it is defined.
"""

import os


def _f(name, default):
    """A float from the environment, or the default. Never raises.

    **NaN and infinity are rejected**, and that is not pedantry. `float("nan")`
    does not raise, so without this check `LIGMAX_AP_MAX_SPEED_MS=nan` sets the
    speed cap to NaN - and every clamp downstream is a `min()`, which
    *propagates* NaN rather than rejecting it: `min(nan, 2.57)` is `nan` in
    Python (the comparison is False, so the first value stands). A single typo
    in `/etc/ligmax/node.env` would therefore disable every speed limit in the
    stack silently. `plan.py`'s `_float` has always rejected both; this is the
    same rule, in the file where it matters most.
    """
    try:
        value = float(os.environ[name])
    except (KeyError, ValueError):
        return default
    if value != value or value in (float("inf"), float("-inf")):
        return default
    return value


def _i(name, default):
    try:
        value = float(os.environ[name])
    except (KeyError, ValueError):
        return default
    if value != value or value in (float("inf"), float("-inf")):
        return default
    return int(value)


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

KNOT_MS = 0.514444


# ------------------------------------------------------- THE SPEED LIMIT
#
# **5 knots. Autonomous mode may never exceed this, and it is not a preference.**
#
# It is a limit on the vessel, so unlike everything else in this file it is NOT
# overridable from the environment. There is deliberately no `_f` call and no
# LIGMAX_* name for it: a limit somebody can raise from a phone on a dock at
# 08:00 is not a limit, and the whole point of this file being environment-driven
# is that the *tuning* can move on the water. This cannot.
#
# It is enforced in five independent places, because one clamp is a single point
# of failure and this is the kind of thing that must not have one:
#
#   1. here          every speed constant below is clamped to it as it is read,
#                    so nothing downstream can even be *asked* for more;
#   2. plan.py       a per-waypoint `speed` above it is refused at upload, with a
#                    readable reason, rather than silently clamped - the operator
#                    should learn about it on the dock, not from the telemetry;
#   3. commander.py  every value is clamped again on its way out of this node,
#                    including the **resultant** of forward and lateral rather
#                    than each axis alone;
#   4. io_manager/autopilot_bridge.py
#                    clamped a third time in the last function before the value
#                    becomes a MAVLink message. That layer is what makes the
#                    limit true rather than merely intended: the control bus is a
#                    loopback socket any process on the Pi can publish to, and a
#                    limit enforced only by the process being limited is not one;
#   5. the tests     `test_speed_limit` drives the whole stack and asserts.
#
# The number itself lives in the repo-root `config.py`, imported below, because
# io_manager enforces the same limit and the two must not drift apart.
#
# **What this does NOT cover:** a MAVLink MISSION run in AUTO. That path never
# passes a speed through this software at all - ArduPilot flies it from its own
# WP_SPEED/CRUISE_SPEED parameters. If the boat is ever run from an uploaded
# mission rather than from this node's GUIDED targets, the limit has to be set on
# the Pixhawk as well. See `next_step.md`.
#
# Raising it means editing the root config, in a commit, with a reason - which is
# the correct amount of friction for a change of this kind.
# Imported rather than restated, so this node and io_manager cannot drift apart
# on the one number they both enforce. See the block in the root `config.py`.
from config import VESSEL_SPEED_LIMIT_KNOTS as SPEED_LIMIT_KNOTS  # noqa: E402
from config import VESSEL_SPEED_LIMIT_MS as SPEED_LIMIT_MS  # noqa: E402


def _speed_value(value):
    """One speed, clamped into `[0, SPEED_LIMIT_MS]`. The limit, applied.

    The clamp is written `min(SPEED_LIMIT_MS, value)` with the **limit first**
    even though `_f` now rejects NaN. If a NaN ever reached here again, that
    order returns the limit where `min(value, SPEED_LIMIT_MS)` would return the
    NaN - Python's `min` keeps the first argument when the comparison is False,
    and every comparison against NaN is False. Two independent guards against one
    silent, total loss of the speed limit is the right number.
    """
    try:
        out = float(value)
    except (TypeError, ValueError):
        return 0.0
    if out != out:  # NaN
        return 0.0
    return max(0.0, min(SPEED_LIMIT_MS, out))


def _speed(name, default):
    """A speed from the environment, clamped into `[0, SPEED_LIMIT_MS]`.

    Every speed in this file goes through here rather than through `_f`, so the
    limit applies to values nobody thought to check as well as the obvious one.
    """
    return _speed_value(_f(name, default))


# ------------------------------------------------------------- careful mode
#
# A second, much lower ceiling the operator can switch on and off while the boat
# is running: **1 knot**. For a first pass down an unfamiliar course, a crowded
# basin, a shakedown after changing a threshold, or any moment where somebody
# wants the boat slow enough to watch and to take over from.
#
# 1 kn is 0.51 m/s, which is about a boat length every four seconds - slow enough
# that a person on the dock can walk alongside it, and slow enough that the 0.5 s
# between one autonomy tick's decision and the next is 26 cm of travel.
#
# This is a *ceiling*, not a speed: everything already slower than it stays as it
# is. Docking (0.30 m/s) and the lateral thruster (0.35 m/s) are both below it,
# so careful mode changes nothing about a berth approach - which is correct,
# because that manoeuvre is already at the speed it should be.
#
# Unlike SPEED_LIMIT_KNOTS this one IS overridable, because it is ours rather
# than a limit on the vessel. It can never rise above the 5 kn limit, though:
# `_speed` clamps it like every other speed here.
CAREFUL_SPEED_KNOTS = _f("LIGMAX_AP_CAREFUL_KNOTS", 1.0)
CAREFUL_SPEED_MS = _speed_value(CAREFUL_SPEED_KNOTS * KNOT_MS)

# Whether the boat boots into careful mode. Off by default - careful mode is a
# deliberate act, and a boat that is quietly slow for a reason nobody remembers
# setting is its own kind of confusing. Set LIGMAX_AP_CAREFUL=1 for a day of
# shakedown runs where it should be the default.
CAREFUL_DEFAULT = _b("LIGMAX_AP_CAREFUL", False)


# The absolute ceiling on commanded speed, whatever a behaviour asks for. This
# is the last line between a planner bug and a boat crossing the course at full
# throttle - and it can never be raised past the 5 kn limit above.
MAX_SPEED_MS = _speed("LIGMAX_AP_MAX_SPEED_MS", 1.6)


# ------------------------------------------------------------- the fast run
#
# NJORD gives two attempts (§8.2), and the marks do not move between them. So
# the two attempts are not the same run twice: the first one is slow enough for
# the lidar to survey every mark properly, and the second one is driven off that
# survey (`survey.py`) at whatever speed the course geometry actually allows.
#
# That is a real trade and it is worth being explicit about which way it goes.
# Going faster costs sightings - a mark 10 m off subtends 2.3 deg, and at 2.5 m/s
# it is inside the C1's useful range for a couple of seconds rather than ten, so
# `TRACK_ESTABLISH_HITS` sweeps of it may simply never happen. The fast attempt
# is therefore only honest when there IS a survey to fall back on, which is why
# it is a mode an operator selects rather than a default.
#
# **The 5 kn limit still applies**, through `_speed` here and through all five
# enforcement points listed above it. What the fast profile changes is which of
# the *lower* ceilings the boat holds itself to, never the vessel limit itself.
FAST_CEILING_MS = _speed("LIGMAX_AP_FAST_CEILING_MS", SPEED_LIMIT_MS)

# What the fast profile asks for on an open leg and around marks. Both are below
# the ceiling on purpose: the ceiling is what the boat may not exceed, these are
# what it aims for, and leaving room between the two is what lets the corner
# limiter below give speed back on a straight without ever touching the limit.
FAST_CRUISE_SPEED_MS = _speed("LIGMAX_AP_FAST_CRUISE_MS", 2.2)
FAST_CAUTION_SPEED_MS = _speed("LIGMAX_AP_FAST_CAUTION_MS", 1.6)

# Extra metres of clearance per m/s the boat is doing, on top of the static
# `BUOY_CLEARANCE_M` and the mark's own position uncertainty.
#
# A clearance is really a *time* budget wearing metres: it is the room needed to
# notice a mark is not where it was believed to be and to steer off it. At the
# 0.8 m/s caution speed 2 m is two and a half seconds; at 2.5 m/s the same 2 m is
# eight tenths of a second, which is less than one autonomy tick plus the
# thrusters' response. So the static figure alone means the boat is progressively
# less safe the faster it goes, while the number on the dashboard says otherwise.
#
# 1.0 m per m/s restores roughly the same time budget across the range: 2.5 m/s
# buys 2.5 m of extra water, and the total clearance at full speed is about
# 4.5 m plus uncertainty.
#
# **Zero on every profile but `fast`**, and that is deliberate rather than
# cautious defaulting. Task 2's gates are red/green pairs 5 m apart (NJORD §9.2)
# - half of that is 2.5 m, and `BUOY_CLEARANCE_M` at 2.0 m plus the 0.35 m
# tracking sigma already very nearly fills it. Any speed term at all would make
# the boat refuse a gate it is supposed to drive through. Task 1's buoys are
# standalone (NJORD §9.1: "no red/green gate pairs are used in this task"), so
# the wide berth is free there and only there.
FAST_CLEARANCE_PER_MS = _f("LIGMAX_AP_FAST_CLEARANCE_PER_MS", 1.0)

# ...and a ceiling on that term, so a runaway speed reading cannot make the boat
# claim half the course as its own.
CLEARANCE_SPEED_MAX_M = _f("LIGMAX_AP_CLEARANCE_SPEED_MAX_M", 3.0)

# Which profile the node boots into. "normal" unless somebody says otherwise:
# booting into `fast` would mean a boat that comes up after a mid-competition
# reboot doing 5 kn on a course it has not surveyed.
DEFAULT_PROFILE = _s("LIGMAX_AP_PROFILE", "normal").strip().lower()


# -------------------------------------------------------------------- speeds

# NJORD §9.2 sets the collision-avoidance task speed at 2 knots and requires the
# boat to accelerate to it immediately at the start of an attempt.
TASK_SPEED_MS = _speed("LIGMAX_AP_TASK_SPEED_MS", 2.0 * KNOT_MS)  # NJORD, 2 kn

# Cruise for the blind GNSS legs, where nothing is being scored on speed except
# the 9 % time multiplier. Deliberately close to the task speed: a fast leg that
# overshoots a waypoint costs more than it saves.
CRUISE_SPEED_MS = _speed("LIGMAX_AP_CRUISE_SPEED_MS", 1.2)

# Around buoys and cardinal marks, where a misread mark has to be recoverable.
CAUTION_SPEED_MS = _speed("LIGMAX_AP_CAUTION_SPEED_MS", 0.8)

# The final approach into a 2 m berth. TUNE: slow enough that a 0.5 s reaction
# is 15 cm of travel.
DOCK_SPEED_MS = _speed("LIGMAX_AP_DOCK_SPEED_MS", 0.3)
DOCK_REVERSE_SPEED_MS = _speed("LIGMAX_AP_DOCK_REVERSE_SPEED_MS", 0.25)

# Below this the boat is treated as stopped, for the "stay stationary" scores.
STATIONARY_SPEED_MS = _speed("LIGMAX_AP_STATIONARY_SPEED_MS", 0.15)


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

# ...and the same distance expressed as time, which is the form that actually
# governs whether pure pursuit is stable. A fixed 6 m lookahead is five seconds
# of travel at the 1.2 m/s cruise and two and a third at 2.5 m/s, and a lookahead
# that short relative to the speed is the classic pure-pursuit oscillation: the
# boat corrects harder than it can turn, overshoots, and weaves down the leg with
# the jury watching the trace (NJORD §11.4). The lookahead used is the LARGER of
# the two, so nothing changes at survey speed and the fast run gets the longer
# rein it needs. TUNE.
LOOKAHEAD_TIME_S = _f("LIGMAX_AP_LOOKAHEAD_TIME_S", 4.0)


# ---------------------------------------------------- how fast a corner allows
#
# The Monday Task 1 course is a slalom: five of its twelve corners are over 85
# degrees and three are over 100, on legs of 10-17 m (`plans/README.md`). A turn
# radius is `speed / yaw rate`, so speed is what decides whether the boat can get
# round a corner at all - and, more to the point, whether it passes inside the
# acceptance radius of the waypoint at the corner, which is what it is scored on.
#
# Rather than hand-tuning a speed per waypoint on a competition morning, the
# geometry is read off the plan and the speed follows from it: fast where the
# course is straight, slow where it is not, with no plan field to get wrong.
#
# How hard the boat can turn, as lateral acceleration in m/s^2. **This is the one
# number here that must be measured on the water**, and everything the corner
# limiter does rests on it.
#
# Lateral acceleration rather than a yaw rate, and the difference is not
# academic. A turn of radius `R` at speed `v` needs `v^2 / R` of lateral
# acceleration, and a hull can only supply so much before it stops turning and
# starts sliding sideways. So the radius a boat can hold grows with the SQUARE of
# its speed:
#
#     R = v^2 / A          v = sqrt(A * R)
#
# Model it as a constant yaw rate instead - `R = v / omega` - and you have a boat
# whose turning circle only doubles when its speed doubles, which is far too
# flattering and which will happily plan a 5 knot pass through a corner it cannot
# physically make. That mistake was in this file's first version of the limiter
# and the simulation caught it: at a plausible hull capability the paced run
# missed the same waypoints as the unpaced one, because the pacing had been
# computed from a law that let it keep almost all of its speed.
#
# **How to measure it**: put the boat in a full-lock turn at a known speed and
# time one revolution. `A = v * 2*pi / period`. Do it at the fast profile's speed,
# not at idle - the number falls off as the hull loads up. 0.8 m/s^2 is a
# conservative guess for a light trimaran on differential thrust; guessing low
# costs a slow corner, guessing high costs the waypoint. TUNE.
TURN_LATERAL_ACCEL_MS2 = _f("LIGMAX_AP_TURN_LATERAL_ACCEL", 0.8)

# The yaw rate the hull can hold at low speed, rad/s, which is the OTHER end of
# the same curve. The lateral-acceleration law above says a boat doing 0.3 m/s
# could hold a 0.11 m radius, i.e. spin on the spot, and it cannot - below a knot
# or so what limits the turn is how much yaw moment the thrusters make, not grip.
# The limiter takes whichever of the two laws is tighter, so this governs the slow
# end and the acceleration governs the fast one. 0.5 rad/s is 29 deg/s. TUNE.
TURN_YAW_RATE = _f("LIGMAX_AP_TURN_YAW_RATE", 0.5)

# How hard the boat can shed speed, m/s^2, for working out how early to start
# slowing for a corner. A displacement hull with reversible thrusters stops
# faster than it accelerates; this is the conservative half of that. TUNE.
TURN_DECEL_MS2 = _f("LIGMAX_AP_TURN_DECEL_MS2", 0.5)

# The corner limiter may never ask for less than this. A boat that creeps round
# every bend is its own failure mode, and below this speed the steering authority
# of a differential-thrust hull starts falling away anyway.
CORNER_MIN_SPEED_MS = _speed("LIGMAX_AP_CORNER_MIN_SPEED_MS", 0.4)


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
# ---- The provenance scare of 2026-08-09, settled by git. Read this. --------
# For a day this number was suspected of being calibrated against the wrong
# distribution: the Jetson colour-corrects before sending (the OV5647 matrix in
# `ligmax-edge/fusion.py::_correct`, on top of the ISP chroma gain in the frame
# header's `saturation` field, default 2.0), and the note below was written in
# the belief that the wire carried sensor-native values. If the capture had
# predated that change, 0.55 would have been derived from a distribution the
# boat no longer sends.
#
# It did not predate it. The capture below contains the `age_ms` field, and
# `age_ms` first appears in ligmax-edge at 1b70dc4, 2026-08-08 18:04, whereas
# `_correct` landed an hour earlier at 15e8d7d, 2026-08-08 17:19. So the capture
# was taken AFTER the correction and the 6879 returns were already corrected.
#
# The measurement stands; only its explanation was wrong. The warm cast in the
# numbers below is a real indoor scene under warm light, not an uncorrected
# sensor - which is, if anything, the more reassuring of the two, because it is
# the cast the boat will actually be handed. Confirm against a red and a green
# mark in the day's light (`telemetry.autopilot.sees`) like any other threshold,
# but this no longer needs re-deriving from scratch before the first run.
# ---------------------------------------------------------------------------
#
# MEASURED, 2026-08-08, on 6879 coloured returns from a real capture indoors,
# post-correction (see above): median saturation 0.42, p90 0.60, and the whole
# scene's mean RGB was (80, 48, 44) - a strong warm cast off warm indoor light.
# At the old threshold of 0.28 every return in that scene passed as "a colour",
# and 46 of 49 clusters classified as RED. Ordinary warm-lit surfaces are not
# red buoys.
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

# How much a return's colour is worth, given how mistimed it is.
#
# The Jetson colours each return from the nearest buffered camera frame, not from
# one frame per sweep, and tells us how far off that frame was in `age_ms`
# (`edge_protocol.py`). In the 2026-08-08 capture 179 of 269 coloured points were
# `stale` - outside the edge's own 40 ms freshness line - so treating every
# coloured return as equally good evidence is throwing away the field that says
# which ones to distrust. A boat doing 2.5 m/s moves 60 cm in 250 ms, and a buoy
# 10 m off subtends about 2.3 deg, so a quarter-second-old frame can genuinely be
# sampling the water beside a mark rather than the mark.
#
# Full weight to FRESH_MS (the edge's --lidar-max-skew default), then a linear
# ramp down to MIN_WEIGHT at STALE_MS (its --lidar-max-age default, which is where
# `age_ms` is capped). The floor is deliberately NOT zero: a mistimed colour is
# weaker evidence, not no evidence, and the edge ships it for exactly that reason.
#
# This only decides WHICH colour wins a cluster's vote and how confident it is.
# It never decides whether there is enough evidence to classify at all -
# `MIN_COLOURED_POINTS` still counts raw returns - so a sweep full of stale
# colour cannot make objects disappear, only make the boat less sure what they
# are. With no `age_ms` on the wire (the aft unit, an older Jetson) every weight
# is 1.0 and the vote is bit-identical to the unweighted one.
COLOUR_AGE_FRESH_MS = _f("LIGMAX_AP_COLOUR_AGE_FRESH_MS", 40.0)
COLOUR_AGE_STALE_MS = _f("LIGMAX_AP_COLOUR_AGE_STALE_MS", 250.0)
COLOUR_AGE_MIN_WEIGHT = _f("LIGMAX_AP_COLOUR_AGE_MIN_WEIGHT", 0.25)


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

# The upper limit on the association gate, however much uncertainty a remembered
# track has accumulated. Without it a track at the sigma ceiling would have a
# gate wide enough to swallow the *other* buoy of a gate 5 m away, and the pair
# would collapse into one.
TRACK_GATE_MAX_M = _f("LIGMAX_AP_TRACK_GATE_MAX_M", 4.0)


# ------------------------------------------------- how sure we are WHERE it is
#
# A track carries a position uncertainty, `sigma_m`, and every consumer that
# needs room around an object adds it to that object's clearance. The point is
# that "there is a buoy here" and "there is a buoy somewhere around here" are
# different statements and the boat should drive differently for each.
#
# Freshly measured. The C1 itself is +-3 cm, so this is almost entirely the
# boat's own pose error: a 3D fix, a compass a degree or two out, and a lever arm
# from the datum to the sensor.
TRACK_SIGMA_M = _f("LIGMAX_AP_TRACK_SIGMA_M", 0.35)

# How fast that grows once the object goes out of view. **We do not know how a
# Njord mark drifts** - it is on a mooring of unknown scope, in tide, and nobody
# has measured it - so this is deliberately a guess with a hard ceiling rather
# than a model. Linear rather than the random walk's sqrt(t) because a linear
# ramp is the one shape somebody can re-tune on a dock from a phone without
# doing arithmetic.
TRACK_SIGMA_GROWTH_M_S = _f("LIGMAX_AP_TRACK_SIGMA_GROWTH_M_S", 0.05)

# ...and where it stops. Past this the answer is "somewhere in this circle" and
# waiting longer does not make it worse: a moored buoy has a watch circle, it
# does not wander off. 6 m at 0.05 m/s is reached after about two minutes unseen.
TRACK_SIGMA_MAX_M = _f("LIGMAX_AP_TRACK_SIGMA_MAX_M", 6.0)

# What a mark restored from the survey file is worth, before the lidar has seen
# it again. **Not `TRACK_SIGMA_MAX_M`**, which is what it used to get by falling
# through the growth ramp with an hour-old `last_seen`, and that was wrong in a
# way that quietly defeated the whole point of surveying.
#
# The two cases are not the same uncertainty. A mark that went out of view during
# a run has been unobserved for an unknown reason and could be anywhere in its
# watch circle; a mark in the survey file was measured deliberately, over
# `TRACK_ESTABLISH_HITS` sweeps, against an RTK fix, and has since been sitting
# on the same mooring. Its error is the survey's own accuracy plus how far a
# moored buoy swings - not two minutes of "we have no idea".
#
# The number matters because every consumer adds it to a clearance. At the old
# 6 m every remembered mark claimed 8 m of water, which on a course whose legs
# are 10-17 m long meant the second attempt would swerve round its own map and
# very likely fail to thread it at all. At 1.2 m the boat gives a remembered mark
# a sane berth, and the instant the lidar actually sees it the figure collapses
# to `TRACK_SIGMA_M` like any other measurement. TUNE against how far the marks
# had moved between attempt one and attempt two, which the trip files will show.
SURVEY_SIGMA_M = _f("LIGMAX_AP_SURVEY_SIGMA_M", 1.2)


# --------------------------------------------- what earns permanent memory
#
# NJORD gives two attempts (§8.2). The marks, the dock and the shore do not move
# between them, so a mark surveyed properly on attempt one is worth keeping - and
# keeping it is the difference between arriving at a gate knowing where it is and
# arriving hoping to see it.
#
# But only a mark surveyed *properly*. One stray coloured return, seen once and
# never again, is far more likely to have been a bad read than a real object, and
# a phantom remembered for ever is worse than no memory at all. So a track has to
# earn permanence, and these three thresholds together are what it has to earn:
#
#   hits      seen on this many sweeps. At 10 Hz that is just over a second of
#             continuous sight, which noise does not survive.
#   span      ...spread over at least this long. The one that actually kills a
#             stray: a burst of returns off a single wave crest can reach the hit
#             count inside 300 ms, and cannot reach it across two seconds.
#   conf      and the accumulated confidence has to have got there.
#
# Only STATIC types qualify at all (`obsticales.is_static`), which is what stops
# the Otter being remembered where it was two minutes ago - the one object on the
# course that is guaranteed to have moved.
TRACK_ESTABLISH_HITS = _i("LIGMAX_AP_TRACK_ESTABLISH_HITS", 12)
TRACK_ESTABLISH_SPAN_S = _f("LIGMAX_AP_TRACK_ESTABLISH_SPAN_S", 2.0)
TRACK_ESTABLISH_CONF = _f("LIGMAX_AP_TRACK_ESTABLISH_CONF", 0.80)

# Once established, a track's confidence stops decaying here rather than at zero.
# That is what makes the memory effectively permanent: `_age` drops a track whose
# confidence falls through 0.15, and without a floor an established mark would
# still evaporate after half a minute out of view. Its *position* uncertainty
# goes on growing, which is the honest part - we still believe it is there, we
# are just less and less sure exactly where.
TRACK_ESTABLISH_FLOOR = _f("LIGMAX_AP_TRACK_ESTABLISH_FLOOR", 0.55)

# An operator who deletes an object means it. Without this a phantom deleted from
# the dashboard is re-created by the very next sweep that produced it, the button
# looks broken, and the operator presses it repeatedly during the one minute they
# could have spent doing something useful. So the spot is refused new tracks for
# this long. Not for ever: if something really is there, it should come back.
FORGET_SUPPRESS_S = _f("LIGMAX_AP_FORGET_SUPPRESS_S", 30.0)


# --------------------------------------------------------------- the survey
#
# The established static tracks, written to disk so attempt two starts with
# attempt one's map. **Stored as lat/lon, never as grid metres**, because the
# grid origin is cached in /run (tmpfs) by `io_manager/navigation.py` and is
# therefore re-zeroed by every reboot - and by the dashboard's `recentre_origin`
# button. A map in metres from an origin that moved is a map of somewhere else.
SURVEY_ENABLED = _b("LIGMAX_AP_SURVEY", True)
SURVEY_FILE = _s("LIGMAX_AP_SURVEY_FILE", "/home/admin/.ligmax/survey.json")

# How often it is rewritten while running. Cheap (a few kB, atomic replace) and
# frequent enough that yanking the power between attempts costs at most this.
SURVEY_SAVE_PERIOD_S = _f("LIGMAX_AP_SURVEY_SAVE_PERIOD_S", 10.0)

# A survey older than this is not this competition, and quietly steering around
# yesterday's marks is exactly the failure this whole feature could introduce.
# Two days covers a Friday practice and a Sunday final.
SURVEY_MAX_AGE_S = _f("LIGMAX_AP_SURVEY_MAX_AGE_S", 2 * 86400.0)

# A ceiling on the file, so a run along a pier that clusters the whole shore into
# LAND tracks cannot grow it without bound.
SURVEY_MAX_TRACKS = _i("LIGMAX_AP_SURVEY_MAX_TRACKS", 200)

# How many independent camera votes before a cardinal mark's direction is
# believed. The detector is known to be weak, so this is deliberately high: the
# cost of waiting is a slower pass, the cost of being wrong is passing on the
# wrong side of a mark, which is the whole point of the task.
CARDINAL_VOTES_REQUIRED = _i("LIGMAX_AP_CARDINAL_VOTES", 4)
CARDINAL_MIN_CONF = _f("LIGMAX_AP_CARDINAL_MIN_CONF", 0.55)


# ------------------------------------------- the alternation prior, OFF by default
#
# A fallback for the case the cardinal vote is designed around and cannot fix: the
# camera never commits, the mark is 15 m away and closing, and the boat has to
# pass it on one side or the other.
#
# The prior is a general fact about how marks are laid in a channel, not a fact
# about any Njord course: **consecutive marks along a run alternate the side you
# pass them on.** A mark that pushes you the same way as the one before it does
# not constrain anything the previous one had not already settled, so nobody lays
# one there. Two marks in a row, and the second is far more likely to be the
# opposite hand than the same one.
#
# `behaviours/alternation.py` applies exactly that and nothing else. It reads the
# sides the boat has *already established for itself* - a red or green mark's
# lateral rule, or an earlier cardinal the camera did commit - and expects the
# next mark to be the other hand. It is not permitted to name a specific course,
# a specific mark, or a specific task, and it never overrides evidence: a
# committed camera vote always wins, and disagreement is reported rather than
# resolved.
#
# **Off unless switched on**, by `LIGMAX_AP_ALTERNATION=1` or the operator's
# `alternation` command. It is an inference from a pattern, the pattern can be
# wrong, and a boat that quietly guesses sides is not one whose telemetry can be
# trusted at the moment it matters. Switched on it is worth roughly the
# difference between a coin flip and a considered guess - which is a great deal
# when the alternative is a coin flip, and nothing at all when the camera works.
ALTERNATION_DEFAULT = _b("LIGMAX_AP_ALTERNATION", False)

# How far off the leg's axis a cardinal's safe side has to point before the prior
# will name it. The prior can only ever say "pass this one to port" or "...to
# starboard"; turning that back into a compass direction needs the safe bearing
# to have a real sideways component. On a leg running north, east and west are
# unambiguous and north and south say nothing at all - 0.5 is sin(30 deg), so a
# safe side within 30 degrees of straight up or down the leg is declined rather
# than guessed at.
ALTERNATION_MIN_SIN = _f("LIGMAX_AP_ALTERNATION_MIN_SIN", 0.5)

# How far apart two marks may be and still count as consecutive. Beyond this the
# second one is not "the next mark in the run", it is a mark on another part of
# the course that happens to lie ahead, and the alternation says nothing about
# it. The Monday course's marks sit on legs of 10-17 m; 40 m is generous enough
# to survive a mark being missed entirely and tight enough to not reach across
# the basin.
ALTERNATION_MAX_GAP_M = _f("LIGMAX_AP_ALTERNATION_MAX_GAP_M", 40.0)


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


# --------------------------------------------------------------- line fitting
#
# `perception/lines.py` turns a sweep into straight edges. These are the numbers
# that decide what counts as an edge, and they are TUNE rather than NJORD: they
# describe the C1 and the things it is pointed at, not a rule.

# How far a return may sit off a straight line and still belong to it.
#
# The window is wide: the C1's +-3 cm means a run of thirty returns off one flat
# wall can have a 9 cm outlier in it, and the corners this has to *separate* are
# right angles, which deviate from the chord by tens of centimetres. So 10 cm sits
# comfortably above the noise and comfortably below any corner in a rectangle.
#
# Set it near the sensor noise instead and a straight wall shatters at whichever
# return happens to sit furthest off the chord - which is why the length filter
# runs after the merge (`lines.fit_segments`) rather than before it.
LINE_TOLERANCE_M = _f("LIGMAX_AP_LINE_TOL_M", 0.10)

# Slack on the above when *accepting* a fitted piece. The split loop stops at
# `LINE_TOLERANCE_M` measured from the chord, and a fit's RMS from the
# least-squares line is a different (smaller) figure - so the acceptance test
# needs headroom or it throws away pieces the splitter was happy with.
LINE_RMS_SLACK = _f("LIGMAX_AP_LINE_RMS_SLACK", 1.6)

# The shortest thing that is a wall. Below this it is a buoy, a bollard or a bird:
# a Njord berth wall is metres long, and 0.5 m is short enough to still catch the
# part of one that a lidar inside the berth can see.
LINE_MIN_M = _f("LIGMAX_AP_LINE_MIN_M", 0.5)

# ...and the fewest returns it may be fitted from. Two points are always exactly
# a line, which is the whole problem with two points.
LINE_MIN_POINTS = _i("LIGMAX_AP_LINE_MIN_POINTS", 5)

# How far out to bother fitting. Deliberately its own number rather than
# `MAX_OBSTACLE_RANGE_M`: a dock wall is worth fitting further away than a buoy is
# worth avoiding, and the far end of the C1's useful range is where a shoreline
# starts arriving as a plausible berth wall.
LINE_MAX_RANGE_M = _f("LIGMAX_AP_LINE_MAX_RANGE_M", 12.0)

# Rejoining one wall that arrived as several pieces - a shadow across it, a cleat
# in front of it, or just noise deciding where the splitter cut. Nearly parallel,
# nearly collinear, and nearly touching. The angle is generous because two short
# noisy pieces of one wall genuinely disagree about its direction by several
# degrees, and the whole point of merging them is that the joined line is better
# than either.
LINE_MERGE_DEG = _f("LIGMAX_AP_LINE_MERGE_DEG", 12.0)
LINE_MERGE_OFFSET_M = _f("LIGMAX_AP_LINE_MERGE_OFFSET_M", 0.15)

# How much daylight two pieces of one wall may have between them. Sized for the
# shadow a floating object casts on the wall behind it - a 40 cm buoy sitting in
# the mouth of a berth hides most of a metre of the back wall from a lidar 5 m
# away, and the two halves either side of it are the only view of that wall there
# is. The cost of being generous is that two collinear walls of two adjacent
# berths can be joined into one long face; that is harmless here, because a face
# is allowed to overhang the space it closes (`perception/parking.py`).
LINE_MERGE_GAP_M = _f("LIGMAX_AP_LINE_MERGE_GAP_M", 0.80)

# Each pass makes at most one join, so this bounds how fragmented a scene can be
# put back together. Twelve is far more than three walls need and costs nothing:
# the pass is a double loop over a handful of segments.
LINE_MERGE_PASSES = _i("LIGMAX_AP_LINE_MERGE_PASSES", 12)


# ------------------------------------------------------------------- parking
#
# NJORD Task 3 as three lines rather than as a gap: the parking space is three
# sides of a rectangle whose corners do not meet, and the boat parks on the middle
# of it (`behaviours/parking.py`, `perception/parking.py`). Everything here is
# per-parking-type, because bow-in and alongside are not the same manoeuvre.

# The way in, and how far back the closed end is. NJORD §9.3's berths: 2 m x 2 m
# bow-in, 2 m deep by 4 m along the dock alongside. Measured on the day beats
# both - a waypoint's `berth_width_m` overrides the mouth.
PARK_MOUTH_M = _f("LIGMAX_AP_PARK_MOUTH_M", 2.0)
PARK_DEPTH_M = _f("LIGMAX_AP_PARK_DEPTH_M", 2.0)
PARK_PARALLEL_MOUTH_M = _f("LIGMAX_AP_PARK_PARALLEL_MOUTH_M", 4.0)
PARK_PARALLEL_DEPTH_M = _f("LIGMAX_AP_PARK_PARALLEL_DEPTH_M", 2.0)

# **The static depth offset, one per parking type.** How far off the middle of the
# space to sit, in metres, measured along the depth axis: **positive is deeper in**
# - towards the lone line, the side of the space with no partner, which is the
# only one whose distance means "how far into the space am I".
#
# Zero is the middle of the space and is the default for both. They are separate
# numbers because the two manoeuvres want different things from the same box: a
# bow-in park usually wants the bow held off the back wall (a small negative),
# alongside usually wants the hull square in the middle. Clamped so the dot stays
# inside the space; a waypoint's `park_offset_m` overrides either.
PARK_DEPTH_OFFSET_M = _f("LIGMAX_AP_PARK_OFFSET_M", 0.0)
PARK_PARALLEL_DEPTH_OFFSET_M = _f("LIGMAX_AP_PARK_PARALLEL_OFFSET_M", 0.0)

# How much of the space to leave between the dot and a wall when clamping the
# offset above. A dot on the wall is not a parking position.
PARK_OFFSET_MARGIN_M = _f("LIGMAX_AP_PARK_OFFSET_MARGIN_M", 0.35)

# How long to sit there. Ten seconds for both, which is what the team asked for;
# NJORD §9.3 asks 10 s of the bow-in park and 5 s of the alongside one, so the
# alongside figure is deliberately the stricter of the two. A waypoint's `hold_s`
# overrides either.
PARK_HOLD_S = _f("LIGMAX_AP_PARK_HOLD_S", 10.0)
PARK_PARALLEL_HOLD_S = _f("LIGMAX_AP_PARK_PARALLEL_HOLD_S", 10.0)

# How close to the dot counts as being on it, and how far the boat may drift
# before the countdown restarts. The rule wants a continuous stretch stationary in
# the middle, so drifting out of the second figure does not count towards it.
PARK_TARGET_TOLERANCE_M = _f("LIGMAX_AP_PARK_TARGET_TOL_M", 0.20)
PARK_HOLD_TOLERANCE_M = _f("LIGMAX_AP_PARK_HOLD_TOL_M", 0.40)

# How far off the space's centreline the boat may be and still go in. Outside this
# the entry is abandoned and the boat **backs out and runs the approach again** -
# it may not steer inside the space (it would arrive crooked) and it may not crab
# across it (see PARK_TRIM_LATERAL_MS), so lining up again outside is the only way
# left. The approach runs along the space's normal or it does not run.
PARK_CENTRE_TOLERANCE_M = _f("LIGMAX_AP_PARK_CENTRE_TOL_M", 0.20)

# What the sideways thruster is allowed to do **while the boat is going somewhere**,
# m/s. A trim, not a way of travelling: it holds the line the approach established
# against a light set, and that is all.
#
# The reason is the hull. Pushed sideways this trimaran presents three hulls
# broadside to the water, so lateral drag is nothing like the drag it was shaped
# around, and a sideways translation that looks fine on a bench is slow, weak and
# at the mercy of any tide on the water. Holding station is a different job - a few
# centimetres against a set, with time to do it - and it gets the full
# `LATERAL_MAX_MS` (`behaviours/parking.py:_approach_move`, `travel=False`).
PARK_TRIM_LATERAL_MS = _speed("LIGMAX_AP_PARK_TRIM_LATERAL_MS", 0.12)

# How far off the parking angle the boat may be during the hold. The countdown
# needs the *angle* as well as the position for the whole of its ten seconds,
# because a boat on the right spot at thirty degrees to the walls is not parked.
# Tighter than the entry tolerance: by this point the boat is stationary and there
# is nothing left for it to be doing except pointing the right way.
PARK_HOLD_ANGLE_DEG = _f("LIGMAX_AP_PARK_HOLD_ANGLE_DEG", 10.0)

# How many times the boat may back out and re-run the approach before saying it
# cannot line up. Without this a boat that cannot centre itself shuttles in and out
# of the mouth for the rest of the run, looking busy.
PARK_MAX_REAPPROACHES = _i("LIGMAX_AP_PARK_MAX_REAPPROACHES", 2)

# The depth of space this hull needs to rotate 90 degrees in, metres - which an
# alongside park does **on the dot, inside the space**.
#
# **Zero, meaning not measured and not checked.** The turning circle of this boat
# is recorded nowhere in git and this file does not invent hardware numbers. Set it
# to a measured figure and a space shallower than it is refused with a reason
# instead of attempted; leave it at 0 and rotating in a 2 m box is the operator's
# judgement, which is where it currently belongs. Worth measuring early: a trimaran
# with amas sweeps more than its length suggests.
PARK_TURN_CLEARANCE_M = _f("LIGMAX_AP_PARK_TURN_CLEARANCE_M", 0.0)

# Below this the boat is close enough on an axis to stop pushing. Thruster wear
# and a visibly hunting boat, for centimetres nobody is scoring.
PARK_DEADBAND_M = _f("LIGMAX_AP_PARK_DEADBAND_M", 0.06)

# Metres per second per metre of error, entering and holding. Tighter than
# `HOLD_P` because the space is 2 m wide and the boat is most of that.
PARK_ENTRY_P = _f("LIGMAX_AP_PARK_ENTRY_P", 0.45)

# Speeds. The creep in and out is the docking creep; the run to the operator's
# parking waypoint is allowed to be brisker because it happens in open water.
PARK_SPEED_MS = _speed("LIGMAX_AP_PARK_SPEED_MS", 0.3)
PARK_REVERSE_SPEED_MS = _speed("LIGMAX_AP_PARK_REVERSE_SPEED_MS", 0.25)
PARK_APPROACH_SPEED_MS = _speed("LIGMAX_AP_PARK_APPROACH_SPEED_MS", 0.8)

# Where the approach starts: hold station this far out from the dot, on the
# centreline, and square up before committing to a space the boat barely fits.
PARK_STANDOFF_M = _f("LIGMAX_AP_PARK_STANDOFF_M", 3.0)

# How square to the space the boat must be before it commits. A 2 m space entered
# 15 deg crooked is a collision.
PARK_ALIGN_TOLERANCE_DEG = _f("LIGMAX_AP_PARK_ALIGN_DEG", 12.0)

# How far to get away from the dot before the waypoint is finished.
PARK_EXIT_M = _f("LIGMAX_AP_PARK_EXIT_M", 3.0)

# How long to look before saying so. NJORD §8.2 gives the crew 20 s to take over,
# so saying it at 15 leaves them all 20.
PARK_SEARCH_TIMEOUT_S = _f("LIGMAX_AP_PARK_SEARCH_TIMEOUT_S", 15.0)

# Finding the space. How far the measured mouth and depth may sit from the figures
# above, how far from parallel/perpendicular three lines may be and still be a
# rectangle, and how much of a nominal dimension a line has to cover to count.
#
# The span fraction is well under 1 on purpose: a lidar *inside* a 2 m space sees
# part of each wall and often not the far end of either, and a finder that
# insisted on whole walls would find nothing from the one position where finding
# something matters most.
PARK_BOX_TOLERANCE_M = _f("LIGMAX_AP_PARK_BOX_TOL_M", 0.6)
PARK_BOX_ANGLE_DEG = _f("LIGMAX_AP_PARK_BOX_ANGLE_DEG", 18.0)
PARK_BOX_SPAN_FRACTION = _f("LIGMAX_AP_PARK_BOX_SPAN", 0.45)

# Whether the aft lidar's returns may be fitted into a parking space. **Off**,
# and not as a preference: the aft unit's mounting geometry is hand-measured and a
# flipped `LIGMAX_AFT_LIDAR_ANGLE_DIR` produces a complete, plausible and
# MIRRORED world astern (docs/testing.md 7c). A mirrored parking space is a
# parking space on the wrong side of the boat, and this behaviour would drive into
# it with confidence. Switch it on once the port-quarter check has passed.
PARK_USE_AFT_LIDAR = _b("LIGMAX_AP_PARK_USE_AFT_LIDAR", False)


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
#              the ride height on channel 14.
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
LATERAL_MAX_MS = _speed("LIGMAX_AP_LATERAL_MAX_MS", 0.35)


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

# The clusters, with the sentence explaining how each one classified. This is the
# layer that answers "why did it call that buoy red", which is the question the
# colour thresholds are re-tuned from, and it is the reason a trip file is worth
# more than the dashboard's scrollback. Same rate as the cloud it came from -
# they are only useful read side by side.
RECORD_CLUSTER_HZ = _f("LIGMAX_AP_RECORD_CLUSTER_HZ", 2.0)

# The io_manager telemetry snapshot (battery, BMS, RTK, trim, tuning, lights,
# propulsion, safety). It rides down the node bus at 1 Hz and costs nothing to
# write, and without it a trip file cannot answer "did the pack sag?" - which is
# a real cause of a boat that stopped and is unanswerable from the autonomy
# node's own state.
RECORD_BOAT_TELEMETRY = _b("LIGMAX_AP_RECORD_BOAT_TELEMETRY", True)

# Keep at most this many trips on the Pi's SD card.
RECORD_KEEP_TRIPS = _i("LIGMAX_AP_RECORD_KEEP", 40)

# ...and at most this many megabytes of them, which is the cap that actually
# matters. A file COUNT is not a disk budget: forty ordinary runs are 80 MB and
# forty pathological ones - a lidar spraying returns off rain, at ten times the
# expected cluster count - are several gigabytes, and the card is 32 GB with the
# OS on it. Whichever limit bites first wins.
RECORD_MAX_TOTAL_MB = _f("LIGMAX_AP_RECORD_MAX_TOTAL_MB", 3072.0)

# One run's own ceiling. Reaching it closes the file and says so loudly rather
# than filling the card: a truncated recording of a 15 minute run is a nuisance,
# a full root filesystem is a boat that cannot write its logs, cannot self-update
# and probably cannot finish the day.
RECORD_MAX_TRIP_MB = _f("LIGMAX_AP_RECORD_MAX_TRIP_MB", 512.0)

# Never start a recording, and stop one already running, with less free space
# than this. Checked against the filesystem, not against our own accounting,
# because the journal and apt and everything else are writing to the same card.
RECORD_MIN_FREE_MB = _f("LIGMAX_AP_RECORD_MIN_FREE_MB", 750.0)

# gzip buffers, and the recording most worth having is the one from the run that
# ended with someone pulling the battery. Flushing on this period bounds what a
# power cut costs to the last few seconds instead of the whole file. It is a
# Z_SYNC_FLUSH, so the file stays a valid gzip stream at every instant.
RECORD_FLUSH_PERIOD_S = _f("LIGMAX_AP_RECORD_FLUSH_PERIOD_S", 5.0)

#: **Camera frames are never written to disk on the Pi.** Not a switch - there is
#: no code path that does it and there must not be one. The Jetson pushes preview
#: JPEGs straight to shore over HTTPS (`ligmax-edge/cloud_camera.py`) and they do
#: not pass through this machine at all. The per-point `rgb` that IS recorded is
#: three bytes per lidar return, not an image, and it is the only evidence of why
#: the colour classifier decided what it decided.
RECORD_IMAGES = False


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
