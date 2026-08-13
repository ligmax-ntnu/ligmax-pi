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
    does not raise, so without this check `LIGMAX_AP_SPEED_MS=nan` sets the
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


def _list(name, default):
    """A comma-separated environment list, lowercased, as a tuple.

    An empty variable means the empty tuple and not the default - `MARK_SOURCES=`
    is how a run says "no camera marks at all", and falling back to the default
    there would be the opposite of what was typed.
    """
    raw = os.environ.get(name)
    if raw is None:
        return tuple(default)
    return tuple(part for part in (p.strip().lower() for p in raw.split(",")) if part)


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


# ------------------------------------------------------------- the one speed
#
# **There is one speed setting and the operator owns it.** `set_speed_limit`
# from the dashboard sets it, in m/s, and it is two things at once:
#
#   * what the boat runs an open leg at, and
#   * the ceiling nothing may exceed - every behaviour, docking included.
#
# Anything the code already holds *below* it stays below it: the docking creep
# (`PARK_SPEED_MS`, `DOCK_SPEED_MS`), the caution speed among marks, the corner
# limiter. So setting 0.1 m/s slows the whole boat including a berth approach,
# which is exactly what a first parking test wants, and setting 2.5 m/s speeds
# up the transits without touching the creep.
#
# What used to be here instead: careful mode (a 1 kn toggle) and three run
# profiles (survey/normal/fast). Both are gone - one number an operator sets is
# the whole of it, and `run_profile` never reached this node anyway because
# `io_manager/autopilot_bridge.py` did not forward it.
#
# The boot value. NOT the limit - `SPEED_LIMIT_MS` above is the limit, it is
# NJORD's 5 knots, it lives in the repo-root `config.py` and nothing here or on
# the dashboard can raise it. 1.2 m/s is the old normal-profile cruise, so a
# boat that comes up after a mid-run reboot behaves as it always did.
SPEED_MS = _speed("LIGMAX_AP_SPEED_MS", 1.2)

# The slowest setting worth accepting. Below this the boat cannot hold a heading
# against any wind at all, so a smaller number is a typo rather than a request.
# 0.1 m/s is deliberately low enough for a dockside parking test - the same floor
# `io_manager/guided.py` and the dashboard use, so all three agree.
SPEED_MIN_MS = _speed("LIGMAX_AP_SPEED_MIN_MS", 0.1)

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
# **Zero by default**, and that is deliberate rather than cautious defaulting.
# Task 2's gates are red/green pairs 5 m apart (NJORD §9.2) - half of that is
# 2.5 m, and `BUOY_CLEARANCE_M` at 2.0 m plus the 0.35 m tracking sigma already
# very nearly fills it. Any speed term at all would make the boat refuse a gate
# it is supposed to drive through. Task 1's buoys are standalone (NJORD §9.1:
# "no red/green gate pairs are used in this task"), so the wide berth is free
# there and only there - set this from the environment for a Task 1 pass being
# run fast, and leave it at zero for anything with a gate in it.
CLEARANCE_PER_MS = _f("LIGMAX_AP_CLEARANCE_PER_MS", 0.0)

# ...and a ceiling on that term, so a runaway speed reading cannot make the boat
# claim half the course as its own.
CLEARANCE_SPEED_MAX_M = _f("LIGMAX_AP_CLEARANCE_SPEED_MAX_M", 3.0)


# -------------------------------------------------------------------- speeds

# NJORD §9.2 sets the collision-avoidance task speed at 2 knots and requires the
# boat to accelerate to it immediately at the start of an attempt.
TASK_SPEED_MS = _speed("LIGMAX_AP_TASK_SPEED_MS", 2.0 * KNOT_MS)  # NJORD, 2 kn

# Cruise for the blind GNSS legs is `SPEED_MS` above - the operator's one
# setting - and there is deliberately no second knob for it here. Two names for
# "how fast does it run a leg" is how the panel and the boat end up disagreeing.

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

# ...unless the returns are the colour of a mark, in which case ONE is enough.
#
# The two cases are not the same evidence and were being held to the same bar.
# An uncoloured return is a range and nothing else: two of them agreeing is the
# cheapest test that separates an object from a single noisy beam. A return the
# camera has painted signal red or neon green is a *measurement of the thing the
# task is about*, and there is nothing else on the water that colour.
#
# One dot is not a rare case, either. The C1's plane is fixed and the boat
# pitches: catch a 40 cm mark on the shoulder of its dome and a whole rotation
# leaves one or two returns on it, which is precisely when seeing it early
# matters most. Requiring two was throwing those away, and `TRACK_CONFIRM_HITS`
# already refuses to steer for anything that does not come back on the next
# sweep - so the sweep is the wrong place to be strict.
MIN_MARK_CLUSTER_POINTS = _i("LIGMAX_AP_MIN_MARK_CLUSTER_POINTS", 1)

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
# Green runs from yellow-green to teal, and it is deliberately the widest band of
# the three. Two things push a real green mark off 120 deg and they push it in
# opposite directions: the warm cast measured below lifts the red channel, which
# drags the hue DOWN towards yellow, and water and sky reflected off a wet dome
# lift the blue channel, which drags it UP towards cyan. 62-200 covers both
# without reaching either red or the sky's own blue.
#
# Nothing on the Njord course is teal, so the cost of the width is close to zero;
# the cost of the old 75-175 was a mark whose hue had been dragged eight degrees
# reading as no colour at all.
HUE_GREEN_MIN = _f("LIGMAX_AP_HUE_GREEN_MIN", 62.0)
HUE_GREEN_MAX = _f("LIGMAX_AP_HUE_GREEN_MAX", 200.0)
# Yellow gives up its top eight degrees to green. RAL 1003 sits near 45 deg and
# the warm cast moves it towards orange, never towards green, so nothing real is
# lost above 62 - whereas a green mark dragged to 70 was being called a cardinal,
# which is the more expensive mistake: the boat routes a via-point around a
# cardinal and merely passes a lateral mark on one side.
HUE_YELLOW_MIN = _f("LIGMAX_AP_HUE_YELLOW_MIN", 35.0)
HUE_YELLOW_MAX = _f("LIGMAX_AP_HUE_YELLOW_MAX", 62.0)

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
#
# This is now the line between "has a hue at all" and "is a grey" - what separates
# a blue fender from a white hull. Each of the three MARK colours has its own bar
# below, and the asymmetry between them is the whole point. Read the next two
# comments before touching any of the four.
MIN_SATURATION = _f("LIGMAX_AP_MIN_SATURATION", 0.55)

# RED's own bar. Deliberately LOOSE, and this is a decision rather than an
# oversight - the crew was shown the measurement below and chose looseness.
#
# MEASURED on the 2026-08-08 capture (6879 returns, warm indoor light, no mark
# anywhere in the scene), running the whole pipeline and counting the clusters that
# came out RED at each bar:
#
#     red bar   0.55   0.60   0.65   0.70   0.75
#     RED        129     85     52      8      0
#
# Every one of those is a false positive indoors, and the cliff at 0.70 is real:
# that scene's warm-grey population runs out there (saturation p90 0.60) whereas
# RAL 3001 does not - a signal red buoy reads about 0.82 in sun, the same in deep
# shade (saturation is chroma over value, so it does not care how dark the light
# is), and higher still under the very cast that makes the greys dangerous.
#
# We stay at 0.55 anyway, because the two failure modes are not equally priced. An
# over-detected mark costs a wider berth, a marker on the chart the operator can
# delete with one tap (`world.forget_track`), and nothing else - the buoy rules
# only shift the corridor, they do not stop the boat. A mark that reads UNKNOWN
# costs the pass that the task is scored on. Indoors among warm wood the detector
# is *supposed* to light up; on the water in daylight the greys that trip it
# largely are not there.
#
# **Raise this towards 0.70 if red marks are being invented on the water** - the
# table says what each step buys, and it cannot cost green anything, because the
# two no longer share a threshold.
MIN_SATURATION_RED = _f("LIGMAX_AP_MIN_SATURATION_RED", 0.55)

# ---- why green needs a much lower bar than red, and why that is still safe ----
#
# MEASURED on the same 6879-return capture, 2026-08-08: the whole sweep averages
# RGB (80, 48, 44). That is a global warm cast, and a warm cast is not symmetric
# in HSV - it does opposite things to the two colours the task is scored on.
#
# Saturation is `(high - low) / high`. Lifting the red channel:
#
#   on a RED mark   raises `high`. Chroma grows, saturation grows. In that
#                   capture 6083 of 6879 returns landed in the red hue band and
#                   at a 0.28 bar 46 of 49 clusters came out RED - warm-lit grey
#                   masquerading as a signal-red buoy. Hence the 0.55 bar, and it
#                   stays.
#   on a GREEN mark raises `low` - red is the *minimum* channel on a green
#                   object. Chroma shrinks and saturation FALLS. The cast is
#                   actively erasing the evidence, and 0.55 was erasing the mark
#                   with it: a neon dome reading (100, 200, 72) is 0.64 and
#                   passes, but the same dome in shade, or a return sampled at
#                   its edge where the camera pixel is half water, lands at
#                   0.3-0.45 and was being called "grey". That is the reported
#                   symptom - green never detected at all.
#
# So a low bar for green is not a loosening of the same test, it is the same
# strictness applied to a channel the cast works against. For a return to land in
# the green hue band at all, GREEN must be the maximum channel - and under a cast
# that multiplies red by about 1.65, green beating red means the true scene had
# green beating red by 1.65 times over. That inequality is the real detector, and
# it is one no amount of warm light can fake; the saturation figure is then only
# there to keep genuine greys out. 0.22 does that.
#
# The remaining false-green source is foliage on the shore, which is why the
# buoys task drops wide clutter (`BUOY_TASK_CLUTTER_RANGE_M`) rather than trying
# to solve it in the colour space, where it cannot be solved.
MIN_SATURATION_GREEN = _f("LIGMAX_AP_MIN_SATURATION_GREEN", 0.22)

# Yellow sits between the two: the cast lifts a yellow mark's red channel, which
# is already near its maximum, so saturation moves little either way. 122 of the
# 123 yellow returns in the capture were above 0.7, so this costs nothing there
# and buys a cardinal seen in shade.
MIN_SATURATION_YELLOW = _f("LIGMAX_AP_MIN_SATURATION_YELLOW", 0.45)

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


# ---------------------------------- a mark colour is not outvoted by background
#
# The vote above counts all five colour names against each other, and that was
# losing marks for a reason that has nothing to do with disagreement.
#
# A cluster on a buoy is not a cluster of buoy-coloured returns. The beams that
# strike the dome squarely get its paint; the ones that clip its shoulder, or that
# pass a few centimetres wide and come back off the water behind it, get "dark".
# Tilt the sensor's plane a little - which the boat's own pitch does continuously -
# and a 40 cm mark gives one or two painted returns and half a dozen dark ones.
# On a five-way vote that is green 1, dark 6: a fraction of 0.14 against a bar of
# 0.5, so the mark comes out UNKNOWN and the boat gives it room on both sides
# instead of passing it on the side it is scored on.
#
# But "dark" is not a competing claim about what the object is. It is water,
# spray and shadow - the *background* the mark is standing in front of, and the
# absence of evidence rather than evidence of absence. So the vote is taken among
# the three MARK colours (red, green, yellow) whenever any of them is present, and
# white and dark are left out of the denominator entirely.
#
# What still beats a mark is another mark colour, which is the only disagreement
# that means anything here: one green against one red is a genuine conflict and
# must stay UNKNOWN. Hence a *higher* bar than `COLOUR_VOTE_FRACTION` on the
# subset - 0.6 keeps 1-vs-1 unresolved, lets 2-vs-1 through, and lets one green
# among twenty dark returns be a green buoy.
MARK_COLOUR_WINS = _b("LIGMAX_AP_MARK_COLOUR_WINS", True)
MARK_COLOUR_VOTE_FRACTION = _f("LIGMAX_AP_MARK_COLOUR_VOTE_FRACTION", 0.6)

# How many mark-coloured returns it takes to call a cluster a mark. One, and
# `MIN_MARK_CLUSTER_POINTS` above explains why. The safety of this number does not
# rest on the sweep: it rests on `TRACK_CONFIRM_HITS`, which will not steer for
# anything that fails to come back, and on `TRACK_ESTABLISH_SPAN_S`, which will
# not remember anything that was not looked at for two seconds.
MARK_MIN_POINTS = _i("LIGMAX_AP_MARK_MIN_POINTS", 1)

# How many mark-coloured returns count as full confidence. A one-dot mark is real
# and it is also weak evidence, and the tracker is built to be told the
# difference: at 3.0 a single painted return opens at 0.33 confidence and climbs
# as the sweeps agree, rather than arriving at 1.0 and outranking a mark the boat
# has studied for a second.
MARK_CONFIDENCE_FULL_POINTS = _f("LIGMAX_AP_MARK_CONFIDENCE_POINTS", 3.0)

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


# ------------------------------------------- detection, per task the boat is on
#
# "Follow these GPS points, but obey the buoy rules" is NJORD §9.1 part 2, it is
# the `buoys` role, and it is a task about marks. There is no vessel to give way
# to in it and no dock to find; the shore is scenery. A detector that reports the
# pier as a structure and a moored dinghy as a vessel on that leg is not being
# careful, it is answering a question nobody asked - and it costs three real
# things: a chart the operator has to read past, `deconflict` corridors pushed
# about by objects the task does not mention, and a survey file that fills up
# with shoreline (`SURVEY_MAX_TRACKS`).
#
# So the classifier is told what the boat is doing and declines to name what the
# task does not need. `perception/classify.py::policy_for` is where these land,
# and there are two rules and one range:
#
#   marks_only    on the buoys leg, a wide object stops being BOAT or LAND and
#                 becomes UNKNOWN. Note what that does NOT change: an UNKNOWN
#                 track is still tracked, still drawn, still avoided on both
#                 sides, and still stops the boat through `emergency_stop_needed`.
#                 What it loses is the *name* - and with it the vessel clearance,
#                 the COLREG machinery and a permanent place in the survey. The
#                 boat still does not hit the pier; it just no longer files a
#                 report about it.
#   clutter range beyond this, a cluster too wide to be a mark is not tracked at
#                 all on a marks task. This is the "bunch of boats and land" cure.
#                 6 m is deliberately well outside anything the hull can reach
#                 before the next dozen sweeps - at the 0.8 m/s caution speed it
#                 is seven seconds of water, and at the fast profile's 2.5 m/s it
#                 is still two and a half - and everything inside it is tracked
#                 exactly as before.
#   mark-sized    a cluster that could be a mark is ALWAYS tracked, at any range,
#                 whatever colour it did or did not come out. That exception is
#                 load-bearing: `world.absorb_detections` can only refine a track
#                 that already exists, so dropping an uncoloured mark-sized
#                 cluster at 9 m would take the camera's "that is green" vote away
#                 with it.
BUOY_TASK_CLUTTER_RANGE_M = _f("LIGMAX_AP_BUOY_TASK_CLUTTER_M", 6.0)

# On a marks task, how wide a cluster may be and still be called a mark - looser
# than `MAX_MARK_WIDTH_M`, because a mark at 3 m is one cluster with the water
# behind it and reads wider than the 40 cm dome it is, and because the gate buoys
# of Task 2 are sometimes caught together with the chop between them.
#
# The width alone does not buy it: a cluster past `MAX_MARK_WIDTH_M` needs
# `BUOY_TASK_WIDE_MARK_POINTS` painted returns rather than the one a normal-sized
# cluster needs. One green dot on a 1.7 m cluster is as likely to be a green hull
# fitting or a reflection as a buoy; two is a mark.
BUOY_TASK_MARK_WIDTH_M = _f("LIGMAX_AP_BUOY_TASK_MARK_WIDTH_M", 1.8)
BUOY_TASK_WIDE_MARK_POINTS = _i("LIGMAX_AP_BUOY_TASK_WIDE_POINTS", 2)

# Note what is deliberately NOT here: how much colour it takes to call something a
# mark. That is `MARK_MIN_POINTS` and it is the same on every task, because it is a
# question about the evidence rather than about the boat's errand - one painted
# return is one painted return whether or not this leg is being scored on marks.
# The task decides what is worth NAMING and what is worth REMEMBERING; it does not
# get to decide what the returns say.


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

# A mark confirms one sweep sooner than anything else. What the confirm count is
# defending against is a cluster made of noise - two stray returns off a wave
# crest - and a cluster that came back twice *in the same place with the same
# paint on it* has already answered that: sea foam is not signal red two sweeps
# running. Meanwhile the cost of the extra sweep falls where it hurts, because a
# mark is only useful while there is still room to choose a side of it.
#
# Two, not one, and that floor stays: one sweep is a measurement, two is the
# cheapest thing that can be called agreement.
MARK_CONFIRM_HITS = _i("LIGMAX_AP_MARK_CONFIRM_HITS", 2)

# ...and a mark is remembered for the rest of the run from that same sighting.
#
# ONCE A BUOY IS SEEN, IT IS THERE FOR EVER. `TRACK_ESTABLISH_HITS` below wanted
# twelve sightings over two seconds at 0.8 confidence before it would remember
# anything, and in the abstract that is the right instinct - a phantom remembered
# for ever is worse than no memory. But a Njord mark is very often not available
# for twelve sweeps: the boat passes it, a swell takes it out of the lidar's fixed
# plane, the camera stops covering that bearing. The mark then falls out of the
# model six seconds later (`TRACK_DROP_AFTER_S`) and the boat arrives back at the
# same gate on the next leg with nothing in memory, which is the failure that
# actually costs a run.
#
# What is given up is real and is bounded, deliberately, by three things that were
# NOT given up: the position uncertainty keeps growing while a mark is unseen so a
# remembered mark is approached more cautiously than a measured one
# (`TRACK_SIGMA_GROWTH_M_S`); the operator's delete button removes it from the
# model and from the survey file together; and only marks get this - LAND still
# has to earn permanence the slow way, since the shore is not what a memory of the
# course is for.
#
# Set to 1 to remember a mark from a single return. That is a coherent choice on a
# course you have already surveyed and a bad one on an unknown shoreline, because
# at 1 there is nothing between one stray green pixel and a permanent buoy on the
# chart.
MARK_ESTABLISH_HITS = _i("LIGMAX_AP_MARK_ESTABLISH_HITS", 2)

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


# ------------------------------ the vessel comes from the CAMERA now, not lidar
#
# Both lidars died on 2026-08-12 and the Otter has to be found with the cameras
# alone. `perception/world.absorb_detections` normally refuses to create a track
# from a camera detection - the buoy detector is weak, and a phantom mark on the
# chart is worse than a missing one - so the collision-avoidance detector is
# given a single, narrow exception, and this is its switch.
#
# **ON by default**, because with no lidar the alternative is a boat that cannot
# see the one object NJORD 9.2 is scored on. Turn it OFF if a lidar is repaired:
# geometry beats a monocular box, and the old rule is the better rule whenever
# there is something else posing the questions.
CAMERA_CREATES_VESSELS = _b("LIGMAX_AP_CAMERA_CREATES_VESSELS", True)

# The bar a vessel detection must clear to become a track. **Much higher than
# the 0.4 the colour vote uses**, and the asymmetry is the point: a camera vote
# on a buoy's colour shifts a corridor, whereas a camera-invented vessel stops
# the boat dead in the middle of a scored run. Measured on
# `ligmax-ai/vessel/synth`: at 0.25 the detector found nothing at all on open
# water over the whole test set, and recall was still 90-100 % down to a 26 px
# hull. Raise it if the shoreline at Havet starts inventing vessels; the cost is
# range, and the table in that README says how much.
VESSEL_MIN_CONF = _f("LIGMAX_AP_VESSEL_MIN_CONF", 0.25)

# ------------------------------- and for the surprise task, so do the MARKS
#
# The second breach of "a camera detection never creates a track", opened
# 2026-08-12 for the surprise task, and the one that needed the most argument
# against it - `perception/world.absorb_detections`' rule exists *because* the buoy
# detector is weak, so this is the exception aimed straight at the reason for the
# rule. Three things decided it:
#
#   * the surprise task scores passing red and green marks on the correct side, and
#     with both lidars dead `behaviours/buoys.py` has an empty `world.marks()` and
#     silently degrades to blind GNSS transit. Nothing else on the boat can put a
#     mark on the chart;
#   * the marks are a *corridor* constraint, not a stop: a wrong one shifts the aim
#     line by a couple of metres (`buoys._lateral`), where a wrong vessel stops the
#     boat dead. The cost of a phantom is bounded here in a way it is not there;
#   * the colour source is not the YOLO. It is a hue/saturation test on the frame
#     below the lidar line - the same test that colours the lidar - which is a much
#     smaller thing to be wrong about than a detector trained on 40 photographs.
#
# **OFF by default, and that is deliberate.** Every other task on this boat is
# better served by the strict rule, and a course that has never been driven with
# camera-invented marks should not acquire them because somebody flashed a new
# build. `set_mark_source` on the dashboard turns it on for the run that wants it,
# which is also what makes it switchable *between* two attempts of the same task.
CAMERA_CREATES_MARKS = _b("LIGMAX_AP_CAMERA_CREATES_MARKS", False)

# Which detections may do it, as the `src` the Jetson stamps on each one.
#
# "colour" is `ligmax-edge/colour_marks.py`, the hue test below the lidar line.
# "yolo" is the buoy detector proper. Both, either, or neither - the dashboard's
# two modes are this string, and the reason it is a *list* rather than a boolean is
# that "run them both and see whether they agree" is the most useful thing an
# operator can do with fifteen minutes on the water before a run.
MARK_SOURCES = _list("LIGMAX_AP_MARK_SOURCES", ("colour",))

# The bar a mark detection must clear to become a track, per source. The colour
# test's "confidence" is the fraction of its blob that passed the hue window, so
# 0.5 means "half the blob was convincingly that colour"; the YOLO's is a YOLO
# score and gets the higher bar because it is the weaker instrument of the two on
# this boat. Neither replaces `TRACK_CONFIRM_HITS`: a single frame puts a mark on
# the chart and does NOT yet move the boat.
MARK_MIN_CONF_COLOUR = _f("LIGMAX_AP_MARK_MIN_CONF_COLOUR", 0.5)
MARK_MIN_CONF_YOLO = _f("LIGMAX_AP_MARK_MIN_CONF_YOLO", 0.6)

# How far out a camera-created mark is believed at all. Range comes from apparent
# size and degrades as the square of it (~6 % at 20 m), and beyond this the
# position is worse than the clearance the rule is trying to enforce - a mark whose
# own uncertainty is wider than the corridor cannot say which side of it the boat
# is on. Marks further away are dropped rather than shipped with a huge sigma,
# because `buoys._lateral` adds sigma to the clearance and would otherwise shove
# the aim line metres sideways for a buoy nobody can see properly yet.
MARK_MAX_RANGE_M = _f("LIGMAX_AP_MARK_MAX_RANGE_M", 25.0)

# NJORD marks are 40 cm spheres (`--buoy-diameter` on the Jetson). Used as a
# created mark's width for the same reason `OTTER_BEAM_M` is used for a vessel's:
# the range was derived from the apparent width, so measuring the width back off
# the detection would be circular.
MARK_DIAMETER_M = _f("LIGMAX_AP_MARK_DIAMETER_M", 0.40)


# NJORD §9.2: the Otter is 2.0 x 1.08 m. Its BEAM, used as a created track's
# width. Deliberately the same constant the Jetson derives the range from
# (`protocol.OTTER_BEAM_M`) - measuring the width back off the detection would
# be circular, since the range and the width come from the same pixel box.
OTTER_BEAM_M = _f("LIGMAX_AP_OTTER_BEAM_M", 1.08)


# ------------------------------------- what the boat DOES about it, per geometry
#
# NJORD 9.2 as it will actually be run (confirmed 2026-08-12): the Otter
# simulates a vessel that is **out of control** - it holds one straight line
# whatever we do - and it can only come from **ahead or from starboard**, never
# from port.
#
# Both of those change the rule that applies, and they change it in the same
# direction. COLREG Rule 18(a): a power-driven vessel underway keeps out of the
# way of a vessel not under command. So we are the give-way vessel in *every*
# case, Rule 17's stand-on branch cannot arise, and there is never a reason to
# hold course into a closing target and hope. `COLREG_STAND_ON` exists to switch
# that branch back on if the Otter is ever a normal give-way vessel again.
COLREG_STAND_ON = _b("LIGMAX_AP_COLREG_STAND_ON", False)

# Head-on (Rule 14): turn to starboard, run a track PARALLEL to the leg this far
# off it, and come back. Not a circle and not a swerve - an offset.
#
# Why an offset rather than the aim-astern manoeuvre `_give_way` does: a head-on
# target has no stern to aim behind, the leg has a **gate at each end** (NJORD
# 9.2: red/green pairs 5 m apart, 20-80 m between gates) so the boat has to be
# back on the centreline before it arrives, and a jury reading the chart sees a
# deliberate dogleg rather than a wobble.
#
# 8 m against a `COLREG_MIN_CPA_M` of 8: the two vessels pass port to port with
# the full offset between them, plus whatever the Otter's own track contributes.
# Wider is not free - the gate is 5 m wide and the boat has to fit back through
# it.
COLREG_OFFSET_M = _f("LIGMAX_AP_COLREG_OFFSET_M", 8.0)

# How far ahead of the boat to plant the offset track's aim point while running
# out to it. Small makes the turn abrupt and hard to hold; large makes it lazy
# and Rule 8 wants it "readily apparent". TUNE.
COLREG_OFFSET_LEAD_M = _f("LIGMAX_AP_COLREG_OFFSET_LEAD_M", 6.0)

# Crossing from starboard (Rule 15): **stop and let it go past.**
#
# Rule 15 says keep clear and avoid crossing ahead; Rule 8(e) says in as many
# words that slackening speed or taking all way off is a legitimate way of doing
# it. For this encounter it is also the better one. The Otter is not going to
# manoeuvre, its track is a straight line and therefore exactly predictable, and
# the boat is on a leg with a 5 m gate at each end - so a turn spends the one
# thing the geometry does not have, which is room. Standing still and letting a
# vessel that cannot steer pass ahead is what a mariner does in a channel, and it
# is unmistakable to a jury.
#
# This is the speed held while waiting rather than zero: a hull with no way on
# has no steering authority at all and will lie across the leg in any wind.
COLREG_WAIT_SPEED_MS = _speed("LIGMAX_AP_COLREG_WAIT_SPEED_MS", 0.15)

# The wait ends when the vessel is drawing clear, which is two conditions and
# both must hold - a range that has begun to open is not enough on its own,
# because it is also what a target that has just passed close ahead looks like
# one second before it is abeam.
#
#   * its bearing has gone abaft this angle, i.e. it is level with us or behind;
#   * and the projected CPA is no longer a problem, or the range is opening.
COLREG_CLEAR_ASTERN_DEG = _f("LIGMAX_AP_COLREG_CLEAR_ASTERN_DEG", 100.0)

# A ceiling on the wait, so a detector that latches onto the pontoon does not
# park the boat on the course for the rest of the attempt. NJORD §8.2 gives a
# 20 s autonomous window before the team must take over; this expires first and
# says so loudly, leaving the operator time to use those seconds.
COLREG_WAIT_MAX_S = _f("LIGMAX_AP_COLREG_WAIT_MAX_S", 45.0)


# --------------------------------------------- the four declared collision roles
#
# `behaviours/collision.py`, and the whole point of them is that **the operator
# declares which encounter this is before the boat moves**. Which side the Otter
# comes from is in the briefing and there are only two cases, so it is a waypoint
# role picked on the dock rather than something classified from a monocular
# bearing thirty seconds before it matters.
#
#   collision_front         vessel in -45..+45 deg  -> alter to starboard, rejoin
#   collision_right         vessel in +45..+110     -> hold, let it cross ahead
#   collision_front_backup  the same alteration, larger, on geometry alone
#   collision_right_backup  the same hold, longer, on geometry alone
#
# The sectors below do NOT decide what the boat does - the role already did that.
# They decide which bearings count as "the vessel we were told to expect", so a
# hull at the quay 120 deg off the bow cannot trigger a manoeuvre laid on for a
# target ahead.
COLLISION_FRONT_SECTOR = (
    _f("LIGMAX_AP_COLLISION_FRONT_LOW_DEG", -45.0),
    _f("LIGMAX_AP_COLLISION_FRONT_HIGH_DEG", 45.0),
)
COLLISION_RIGHT_SECTOR = (
    _f("LIGMAX_AP_COLLISION_RIGHT_LOW_DEG", 45.0),
    _f("LIGMAX_AP_COLLISION_RIGHT_HIGH_DEG", 110.0),
)

# How close a tracked vessel has to be, inside the sector, to fire the manoeuvre.
# The same 25 m `COLREG_DETECT_RANGE_M` uses, restated rather than shared because
# these roles are allowed to move independently of the general COLREG behaviour -
# and because at 25 m the Otter is 42 px in the detector's tensor, comfortably
# inside the flat part of the recall curve (`ligmax-ai/vessel/README.md`).
COLLISION_TRIGGER_RANGE_M = _f("LIGMAX_AP_COLLISION_TRIGGER_M", 25.0)

# How far to starboard the detected alteration runs, as a track PARALLEL to the
# leg. 8 m against a `COLREG_MIN_CPA_M` of 8: the two pass port to port with the
# full offset between them. Wider is not free - the gate is 5 m and the boat has
# to fit back through it.
COLLISION_OFFSET_M = _f("LIGMAX_AP_COLLISION_OFFSET_M", 8.0)

# How far ahead on the offset track to aim while running out to it. Small makes
# the turn abrupt and hard to hold, large makes it lazy, and COLREG rule 8 wants
# it "readily apparent". TUNE.
COLLISION_OFFSET_LEAD_M = _f("LIGMAX_AP_COLLISION_OFFSET_LEAD_M", 6.0)

# Held while waiting for a vessel to cross ahead. NOT zero: a hull with no way on
# has no steering authority at all and lies across the leg in any wind, which is
# both a worse place to be when the Otter goes by and a mess to recover from.
COLLISION_WAIT_SPEED_MS = _speed("LIGMAX_AP_COLLISION_WAIT_SPEED_MS", 0.15)

# A detected manoeuvre ends when the vessel is abaft this AND opening. Both,
# because a range that has begun to open is also what a target looks like one
# second before it crosses close ahead.
COLLISION_CLEAR_ASTERN_DEG = _f("LIGMAX_AP_COLLISION_CLEAR_ASTERN_DEG", 100.0)

# ...and ceilings on both, so nothing can run for ever on a detector that has
# latched onto a pontoon.
COLLISION_WAIT_MAX_S = _f("LIGMAX_AP_COLLISION_WAIT_MAX_S", 45.0)
COLLISION_OFFSET_MAX_S = _f("LIGMAX_AP_COLLISION_OFFSET_MAX_S", 40.0)

# After a manoeuvre finishes, how long before the leg will trigger another one.
#
# It re-arms rather than finishing for good, and that is the safer of the two.
# A spurious early trigger - a moored hull inside the sector at the top of the
# leg - would otherwise spend the boat's one manoeuvre before the Otter ever
# appeared, and the rest of the leg would be driven blind past the thing the task
# is about. Re-arming costs a second manoeuvre in the worst case; not re-arming
# costs the run. The delay only exists to stop the boat dithering in and out of
# the same encounter.
COLLISION_REARM_S = _f("LIGMAX_AP_COLLISION_REARM_S", 5.0)


# ----------------------------------------------- and the backups, which are blind
#
# **Nothing here consults the world model.** No detector, no tracks, no camera.
# These fire on the boat's own progress along the leg and cannot be stopped by
# anything being broken, which is the entire reason they exist: nothing in this
# stack has been on the water, and a run where the detector says nothing is
# otherwise a run where the boat drives straight down the line into the Otter.
#
# Where the manoeuvre starts: this far BEFORE the midpoint of the two waypoints,
# measured along the leg rather than as a range to the midpoint, so a boat a few
# metres off the line does not start early or late.
COLLISION_BACKUP_LEAD_M = _f("LIGMAX_AP_COLLISION_BACKUP_LEAD_M", 10.0)

# How far past the midpoint the scripted alteration holds its offset before
# rejoining. Symmetric with the lead, so the dogleg is centred on the midpoint -
# which is where the two vessels were always going to meet.
COLLISION_BACKUP_RUN_M = _f("LIGMAX_AP_COLLISION_BACKUP_RUN_M", 10.0)

# The scripted alteration is bigger than the detected one, and the scripted hold
# is longer. That is not timidity, it is the price of being blind: a manoeuvre
# timed off our own progress rather than off where the Otter actually is has to
# be wide enough to be right despite not knowing. 12 m against the detected 8.
COLLISION_BACKUP_OFFSET_M = _f("LIGMAX_AP_COLLISION_BACKUP_OFFSET_M", 12.0)

# The scripted hold, in seconds. At the Otter's 2.5 kn this is about 26 m of its
# travel, which clears a crossing track with room to spare. It is a fixed count
# and not a look, because a backup that waits for something to be seen is not a
# backup for a boat that cannot see.
COLLISION_BACKUP_WAIT_S = _f("LIGMAX_AP_COLLISION_BACKUP_WAIT_S", 20.0)


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

# **Centre-to-centre between the two side AR tags**, for the `park_tag` roles. A
# DIFFERENT NUMBER from the mouth above, and that is the whole reason it exists: the
# markers are taped to walls 0.13 m thick, so tag-to-tag is the clear opening plus
# about one wall. The team's measurements are 2 m for the bow-in berth and 4.13 m
# for the alongside one, and 4.13 = 4.00 + 0.13 reads as exactly that.
#
# Only ever used to CHECK a pair of tags and to draw the box, never to place the
# boat: the dot is the midpoint of the two tags, and a symmetric error in this
# figure does not move a midpoint. Park once, read `parking.mouth_m` off the panel,
# and set this to what the dock actually is.
PARK_TAG_SPAN_M = _f("LIGMAX_AP_PARK_TAG_SPAN_M", 2.0)
PARK_TAG_SPAN_PARALLEL_M = _f("LIGMAX_AP_PARK_TAG_SPAN_PARALLEL_M", 4.13)

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
#
# **Half the forward creep, and it has to stay under it.** This was 0.12 while the
# entry creep was 0.3; the creep came down to 0.1 on 2026-08-11 and a 0.12 trim
# against a 0.1 creep would have made the sideways term the LARGER of the two - a
# boat moving more across the berth than into it, which is the opposite of every
# rule in `behaviours/parking.py`. The two axes are scaled together rather than
# clipped separately, so this does not bend the approach; it bounds how much of the
# commanded motion may be sideways, and at 0.05 against 0.1 that is at most about
# 27 degrees off the normal.
PARK_TRIM_LATERAL_MS = _speed("LIGMAX_AP_PARK_TRIM_LATERAL_MS", 0.05)

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

# Speeds, and they are deliberately **very** slow inside the berth: 0.1 m/s, the
# floor of the whole speed system (`MIN_SPEED_LIMIT_MS`), asked for by the team on
# 2026-08-11. There is nothing to gain from entering faster and a great deal to
# lose - the berth is 2 m across, the hull is most of that, and the only things
# between it and the pontoon are the operator and the physical E-stop
# (`behaviours/parking.py` turns the ordinary avoidance off in here on purpose).
#
# 0.1 m/s is 10 cm a second: crossing a 2 m berth takes 20 s, and the whole
# bow-in manoeuvre - align, enter, hold 10 s, reverse out - is comfortably inside
# the 15 minute slot even repeated. A contact is a deduction *and* the completion
# points, so slow is not the cautious choice here, it is the fast one.
#
# These are caps, never floors: `Parking._speed` takes the lesser of the figure
# here and the operator's `set_speed_limit`, so a setting *below* these still wins.
# Raising `set_speed_limit` cannot make the berth creep faster - to do that you
# would have to change the number here, which is the point of it being here.
PARK_SPEED_MS = _speed("LIGMAX_AP_PARK_SPEED_MS", 0.1)
PARK_REVERSE_SPEED_MS = _speed("LIGMAX_AP_PARK_REVERSE_SPEED_MS", 0.1)
# SEARCH (running to the operator's waypoint, in open water) and ALIGN (getting
# onto the centreline one 3 m standoff out, and squaring up) share this one. It is
# 0.3 rather than 0.1 because it covers real distance - GPS 7 to the docking
# waypoint is about 15 m, which is 50 s at 0.3 and 150 s at 0.1 - and rather than
# the 0.8 it was until 2026-08-11, because half of what it governs happens three
# metres from a pontoon. Nothing here is inside the berth.
PARK_APPROACH_SPEED_MS = _speed("LIGMAX_AP_PARK_APPROACH_SPEED_MS", 0.3)

# Where the approach starts: hold station this far out from the dot, on the
# centreline, and square up before committing to a space the boat barely fits.
PARK_STANDOFF_M = _f("LIGMAX_AP_PARK_STANDOFF_M", 3.0)

# How square to the space the boat must be before it commits. A 2 m space entered
# 15 deg crooked is a collision.
PARK_ALIGN_TOLERANCE_DEG = _f("LIGMAX_AP_PARK_ALIGN_DEG", 12.0)

# How far to get away from the dot before the waypoint is finished.
PARK_EXIT_M = _f("LIGMAX_AP_PARK_EXIT_M", 3.0)

# How long to look from the waypoint before probing forward. NJORD §8.2 gives the
# crew 20 s to take over, so moving at 15 still leaves them all 20 once the probe
# below has also run out.
PARK_SEARCH_TIMEOUT_S = _f("LIGMAX_AP_PARK_SEARCH_TIMEOUT_S", 15.0)

# ---- the probe: what to do when the waypoint sees nothing --------------------
#
# The waypoint before a park is laid *just outside* the docks, and the boat only
# has a forward-looking lidar (the aft unit is broken - see PARK_* below and
# docs/hardware.md), so a space a couple of metres further in is simply not in
# view from there. Rather than sit at the waypoint declaring failure, the boat
# creeps along a fixed bearing until it either finds the three lines or runs out
# of probe.
#
# **120 degrees is Havet arena's answer, not a general one**: at that berth, in
# towards land is east and a little south. It is a *true* bearing (degrees from
# north, clockwise), it is the direction the bow points while probing, and a
# waypoint's own `park_probe_deg` overrides it - which is what a different dock
# needs rather than an environment variable.
PARK_PROBE_BEARING_DEG = _f("LIGMAX_AP_PARK_PROBE_DEG", 120.0)

# How far past the waypoint the probe may go, metres. Bounded because the whole
# point of the waypoint is that somebody laid it in the right place: 8 m of probe
# finds a berth that was 5 m further in than the GPS point suggested, and does not
# quietly turn into a boat crossing the basin looking for a wall.
PARK_PROBE_M = _f("LIGMAX_AP_PARK_PROBE_M", 8.0)

# How fast to probe. The berth creep, 0.1 m/s, because this is the one move on the
# whole boat that drives deliberately **towards a hard structure it cannot see** -
# a forward-looking sensor outside the docks cannot make out a berth a few metres
# further in, so the boat goes and looks. 8 m of probe is therefore 80 s, which is
# the price of the fallback path and is why the waypoint being laid in the right
# place matters: with the tags visible from the waypoint this never runs at all.
PARK_PROBE_SPEED_MS = _speed("LIGMAX_AP_PARK_PROBE_SPEED_MS", 0.1)

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

# How much the space is allowed to move before the boat stops believing a new
# measurement of it, once it is committed. See `behaviours/parking.py:_measure`:
# from the turn onwards the box is a *memory*, and a fresh fit is only accepted
# when it agrees with that memory on all three of the dot, the mouth width and
# the way in. It has to be a tolerance rather than a flat refusal because a
# floating dock moves; it has to be tight because a "space" fitted from a boat
# lying across the berth - which is what an alongside park does, with the lone
# line out of view - is a different space wearing the same name.
PARK_LATCH_TOLERANCE_M = _f("LIGMAX_AP_PARK_LATCH_TOL_M", 0.4)

# There is **no aft-lidar switch here any more.** The aft unit is broken (2026-08-11,
# docs/hardware.md), and it was already refused for parking anyway: its mounting
# geometry is hand-measured and a flipped `LIGMAX_AFT_LIDAR_ANGLE_DIR` produces a
# complete, plausible and MIRRORED world astern (docs/testing.md 7c), which is a
# parking space on the wrong side of the boat that this behaviour would drive into
# with confidence. Parking fits lines to the front unit and nothing else.


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
    out = {
        name: value
        for name, value in sorted(globals().items())
        if name.isupper() and isinstance(value, (int, float, str, bool))
    }
    # A tuple is not a scalar, so the comprehension above skips it - and which mark
    # sources were live is the first thing a post-mortem of a buoy leg has to know,
    # since it decides whether `world.marks()` could have held anything at all.
    out["MARK_SOURCES"] = ",".join(MARK_SOURCES)
    return out
