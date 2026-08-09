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
# does not wander off. 6 m at 0.05 m/s is reached after about two minutes unseen,
# and is also what a track restored from a previous run starts at.
TRACK_SIGMA_MAX_M = _f("LIGMAX_AP_TRACK_SIGMA_MAX_M", 6.0)


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
