"""What a cluster is, from the colour of its returns and how big it is.

    kind, confidence, why = classify(cluster, config, context="buoys")

**The colour comes off the lidar, not off the detector.** The front C1's returns
arrive already coloured by the Jetson's two cameras (`edge_protocol.py`), so a
cluster carries a measured geometry *and* a colour, and neither of those had to
go through the YOLO to get here. That matters on this boat, because the detector
is weak and the lidar is not.

The colour map, which is the whole classifier
---------------------------------------------
    red             a red navigation buoy      -> keep to PORT (seaward north)
    green           a green navigation buoy    -> keep to STARBOARD
    yellow          black-and-yellow: a cardinal mark. WHICH cardinal is a
                    topmark, so it is a camera question and is answered
                    elsewhere - here it is just CARDINAL.
    white           a hull or a dock. Which one depends on what the boat is
                    doing, hence `context`.
    blue / dark     water, spray, shadow. Not an object; discarded.

Why hue and not RGB distance
----------------------------
Absolute RGB drifts with exposure, with the time of day and between the two
cameras. Hue survives all three: a red buoy in shade and the same buoy in sun
differ enormously in *value* and barely at all in *hue*. Saturation then
separates "a colour" from "a grey", and value separates white from black among
the greys.

**What the values on the wire actually are - checked 2026-08-09, and not what
this file used to say.** The Jetson now colour-corrects before sending: it
applies the OV5647 matrix in `ligmax-edge/fusion.py::_correct`, on top of the
chroma gain Argus's ISP has already applied (the `saturation` header field,
default 2.0, because JetPack ships no ISP tuning for this sensor and its
untouched output is about a third of normal chroma). So these are *corrected*
values comparable to a corrected preview, **not** the sensor-native ones this
docstring claimed for as long as the edge repo was unavailable to check against.

That mattered for one number, `config.MIN_SATURATION`, which was raised to 0.55
on the strength of a capture whose saturation statistics were read as a
raw-sensor warm cast. The worry was that the capture predated the correction, in
which case the threshold would be calibrated against a distribution the boat no
longer receives. **It does not: settled by git 2026-08-09.** The capture carries
`age_ms`, which first appears in ligmax-edge at 1b70dc4 (2026-08-08 18:04),
while `_correct` landed at 15e8d7d (2026-08-08 17:19) - an hour earlier. The
returns were already corrected, the measurement holds, and the warm cast in it
is warm indoor light rather than an uncorrected sensor. `config.py` carries the
full argument at the value itself.

Njord's own paint codes make this workable: RAL 3001 signal red and neon green
are about as far apart in hue as two colours get, and RAL 1003 yellow sits
neatly between them. The thresholds are in `config.py` and **must be checked in
the day's light** - they are the one part of this stack that a cloud can move.

Each mark colour carries its own saturation bar, and that asymmetry is not
sloppiness
-------------------------------------------------------------------------------
A warm cast lifts the red channel. On a red mark that raises the maximum channel,
so chroma and saturation *grow*; on a green mark it raises the minimum channel,
so chroma and saturation *shrink*. One threshold for both therefore cannot be
right: the number that keeps warm-lit grey from reading as signal red (0.55, and
the capture that forced it is in `config.MIN_SATURATION`) is far above what a real
green dome survives once the cast has eaten its chroma. That is why green was
never detected at all, and why `MIN_SATURATION_GREEN` is 0.22.

The low green bar is safe for a reason worth stating: to land in the green hue
band a return must have GREEN as its maximum channel, and under a cast that
multiplies red by about 1.65 that means the real scene had green ahead of red by
1.65 times over. The channel inequality is the detector; the saturation figure
only keeps genuine greys out.

Voting, and why UNKNOWN is still a good answer
----------------------------------------------
A cluster is classified by a weighted vote of its coloured returns - but the vote
is taken among the three MARK colours (red, green, yellow) whenever any of them
is present, and white and dark are left out of the denominator.

That is the difference between seeing a buoy and not. A 40 cm dome caught on the
shoulder of the lidar's plane gives one or two painted returns and half a dozen
"dark" ones off the water behind it, and a five-way vote reads that as green 1,
dark 6 - a fraction of 0.14, comfortably UNKNOWN, and the mark is then given room
on both sides instead of being passed on the side it is scored on. But dark is
not a rival claim about what the object is; it is the background it stands in
front of. `MARK_COLOUR_WINS` in `config.py` carries the full argument.

What still beats a mark colour is another mark colour, and that is the only
disagreement here that means anything: one green against one red is a real
conflict and stays UNKNOWN, which is deliberately safe. An unknown obstacle is
given room on *both* sides, whereas a guessed one is passed confidently on
whichever side the guess implied. In a task where passing on the wrong side is
the failure being scored, a shrug beats a coin flip.

The task decides what is worth naming
-------------------------------------
`context` is what the boat is doing, and `policy_for` turns it into how hard to
look and for what. Two contexts want genuinely different answers about the same
returns:

    dock / avoid  a big white object is the thing the task is about - the dock to
                  aim at, or the Otter to give way to. Name it.
    buoys         "follow these GPS points, obeying the buoy rules" is a task
                  about marks. There is no vessel to give way to and no berth to
                  find; the shore is scenery. So a wide object is not named
                  BOAT or LAND, and wide clutter past `BUOY_TASK_CLUTTER_RANGE_M`
                  is not tracked at all.

**Declining to name something is not declining to avoid it.** An UNKNOWN track is
still tracked, still drawn, still steered around on both sides by
`behaviours/base.deconflict`, and still stops the boat through
`emergency_stop_needed`. What it loses is a vessel's clearance, the COLREG
machinery, and a permanent entry in the survey file.
"""

import numpy as np

from ..obsticales import MARK_TYPES, ObstacleType

# What `scan.py` writes into `rgb` for a return no camera could colour. Not
# black on purpose - most of a rotation is behind both lenses, and "uncoloured"
# must never be confusable with "genuinely dark".
NO_COLOUR = -1

#: The three colours that mean "this is a mark". White and dark are background -
#: a hull, a dock, water, spray, shadow - and are kept out of a mark's vote.
MARK_COLOURS = ("red", "green", "yellow")

#: The context string for "follow the GPS points, obeying the buoy rules" -
#: `behaviours/buoys.Buoys.task`, which is where it arrives from.
MARKS_TASK = "buoys"

# How many coloured returns count as full confidence in an ordinary (non-mark)
# vote. Unchanged behaviour; a mark's own figure is
# `config.MARK_CONFIDENCE_FULL_POINTS`, which is lower because a mark decided on
# one painted return is real evidence and there is only ever going to be one of it.
FULL_CONFIDENCE_POINTS = 6.0


def rgb_to_hsv(rgb):
    """`(n, 3)` 0-255 ints -> `(hue_deg, saturation, value)` arrays.

    Written out rather than pulled from colorsys or matplotlib because this runs
    on every cluster of every sweep and both of those are per-pixel Python.
    """
    arr = np.asarray(rgb, dtype=np.float64) / 255.0
    r, g, b = arr[:, 0], arr[:, 1], arr[:, 2]
    high = arr.max(axis=1)
    low = arr.min(axis=1)
    chroma = high - low

    hue = np.zeros_like(high)
    nonzero = chroma > 1e-9
    # Which channel is the maximum decides which 120 deg sector the hue is in.
    red_max = nonzero & (high == r)
    green_max = nonzero & (high == g) & ~red_max
    blue_max = nonzero & ~red_max & ~green_max
    with np.errstate(divide="ignore", invalid="ignore"):
        hue[red_max] = 60.0 * (((g[red_max] - b[red_max]) / chroma[red_max]) % 6.0)
        hue[green_max] = 60.0 * (((b[green_max] - r[green_max]) / chroma[green_max]) + 2.0)
        hue[blue_max] = 60.0 * (((r[blue_max] - g[blue_max]) / chroma[blue_max]) + 4.0)

    saturation = np.zeros_like(high)
    lit = high > 1e-9
    saturation[lit] = chroma[lit] / high[lit]
    return hue % 360.0, saturation, high


def white_balance_gains(rgb, config):
    """Grey-world gains for one sweep: `(gr, gg, gb)`, or None.

    The Jetson sends **sensor-native, uncalibrated** values - the OV5647's
    colour matrix runs at the receiver, so a warm-lit scene arrives warm. A real
    capture indoors averaged (80, 48, 44) across the whole sweep, which is a
    global cast rather than a red world.

    Dividing each channel by its own mean cancels that, on the assumption that
    the scene averages to grey. On water that assumption is good - a sweep is
    mostly sea, sky and land, and a buoy is a handful of returns out of
    hundreds. It is *not* good when one coloured object fills the view, which is
    why this is off by default (`config.WHITE_BALANCE`) and why the gains are
    clamped: the clamp bounds how wrong the degenerate case can go.
    """
    if not config.WHITE_BALANCE:
        return None
    arr = np.asarray(rgb, dtype=np.float64)
    if arr.ndim == 1:
        arr = arr.reshape(-1, 3)
    lit = ~(arr <= NO_COLOUR).any(axis=1)
    arr = arr[lit]
    if arr.shape[0] < 20:  # too few returns for the grey-world assumption
        return None
    means = arr.mean(axis=0)
    if (means <= 1.0).any():
        return None
    grey = float(means.mean())
    limit = config.WHITE_BALANCE_MAX_GAIN
    return tuple(float(min(limit, max(1.0 / limit, grey / m))) for m in means)


def age_weights(age_ms, config):
    """Per-return vote weight from `age_ms`, or None when there is no age.

    Full weight inside the edge's freshness window, ramping linearly down to
    `COLOUR_AGE_MIN_WEIGHT` by the age at which the edge stops colouring at all.
    A return the Jetson could not colour carries -1 rather than an age; it is
    dropped from the vote by the `lit` mask before this is ever consulted, so it
    is given full weight here rather than special-cased twice.
    """
    if age_ms is None:
        return None
    ages = np.asarray(age_ms, dtype=np.float64).reshape(-1)
    if ages.size == 0:
        return None
    fresh = float(config.COLOUR_AGE_FRESH_MS)
    stale = float(config.COLOUR_AGE_STALE_MS)
    floor = float(config.COLOUR_AGE_MIN_WEIGHT)
    # A misconfigured pair (stale <= fresh) must not divide by zero or invert the
    # ramp; it degrades to "everything inside fresh, full weight".
    if not (stale > fresh):
        return np.ones_like(ages)
    over = np.clip((ages - fresh) / (stale - fresh), 0.0, 1.0)
    weights = 1.0 - (1.0 - floor) * over
    # -1 is "nothing coloured this", not "infinitely fresh".
    return np.where(ages < 0.0, 1.0, weights)


def _lit_hsv(rgb, config, gains=None):
    """`(lit mask, hue, saturation, value)` for the returns a camera coloured.

    The hue arrays cover only the lit returns, so they are shorter than `rgb`
    whenever anything was uncoloured; `lit` is what maps them back. Shared by
    `colour_votes` and `mark_colour_mask` so that "is this return a mark colour"
    cannot drift apart from "which colour won this cluster" - they were the same
    test written twice for about an hour, which is long enough.
    """
    arr = np.asarray(rgb)
    if arr.size == 0:
        return None, None, None, None
    if arr.ndim == 1:
        if arr.size % 3:
            return None, None, None, None
        arr = arr.reshape(-1, 3)
    # A single -1 channel marks the whole return as uncoloured.
    lit = ~(arr <= NO_COLOUR).any(axis=1)
    if not lit.any():
        return lit, None, None, None
    arr = arr[lit]
    if gains is not None:
        # Applied before the hue is taken, and clipped back into range so a gain
        # cannot push a channel past white and invert the hue it was correcting.
        arr = np.clip(np.asarray(arr, dtype=np.float64) * np.asarray(gains), 0, 255)
    hue, saturation, value = rgb_to_hsv(arr)
    return lit, hue, saturation, value


def _colour_masks(hue, saturation, value, config):
    """The five colour masks, in the order they win ties.

    Each mark colour is tested against **its own** saturation bar, because a warm
    cast moves red's and green's in opposite directions - see the module docstring
    and `config.MIN_SATURATION_GREEN`. Red is tested first and wins any overlap
    with yellow at the seam, because a red buoy misread as a cardinal costs a
    wrong-side pass while the reverse costs a cautious detour.
    """
    red = (
        (saturation >= config.MIN_SATURATION_RED)
        & ((hue <= config.HUE_RED_LOW_MAX) | (hue >= config.HUE_RED_HIGH_MIN))
    )
    yellow = (
        (saturation >= config.MIN_SATURATION_YELLOW)
        & (hue >= config.HUE_YELLOW_MIN)
        & (hue <= config.HUE_YELLOW_MAX)
    )
    green = (
        (saturation >= config.MIN_SATURATION_GREEN)
        & (hue >= config.HUE_GREEN_MIN)
        & (hue <= config.HUE_GREEN_MAX)
    )
    yellow &= ~red
    green &= ~red & ~yellow

    painted = red | yellow | green
    # A hue that is none of the three - a blue fender, the sky in a reflection.
    # Bucketed with the water, because for planning purposes "a colour that is
    # not a mark" and "not an object" are the same answer.
    other = (saturation >= config.MIN_SATURATION) & ~painted
    grey = ~painted & ~other
    white = grey & (value >= config.WHITE_MIN_VALUE)
    return red, green, yellow, white, (grey & ~white) | other


def mark_colour_mask(rgb, config, gains=None):
    """Per-return "this is the colour of a mark", as a full-length bool array.

    Length matches `rgb`, with False wherever no camera coloured the return, so
    `cluster.py` can ask the question of a group of point indices directly. That
    is what lets a **single** painted return survive `MIN_CLUSTER_POINTS` - see
    `config.MIN_MARK_CLUSTER_POINTS` for why one dot is enough.
    """
    arr = np.asarray(rgb)
    if arr.ndim == 1:
        n = arr.size // 3
    else:
        n = arr.shape[0]
    out = np.zeros(max(0, n), dtype=bool)
    lit, hue, saturation, value = _lit_hsv(rgb, config, gains)
    if lit is None or hue is None:
        return out
    red, green, yellow, _white, _dark = _colour_masks(hue, saturation, value, config)
    out[lit] = red | green | yellow
    return out


def colour_votes(rgb, config, gains=None, age_ms=None):
    """Colour names for one cluster's returns.

    Returns `(tally, coloured, weighted)`:

        tally      `{name: count}` - raw returns of each colour, for the operator
        coloured   how many returns had any colour at all
        weighted   `{name: weight}` - the same tally with each return counted by
                   how well-timed the frame that coloured it was (`age_weights`)

    The two tallies are kept apart on purpose. `weighted` decides *which colour
    wins and how confidently*; `tally` and `coloured` are what the boat says out
    loud and what `MIN_COLOURED_POINTS` gates on, because "only 2 coloured
    returns" has to keep meaning two actual returns. With no `age_ms` the two are
    identical and nothing downstream can tell the difference.

    Names are the five the boat reasons in: "red", "green", "yellow", "white",
    "dark". Anything with a hue that is none of the three - a blue fender, the
    sky in a reflection - lands in "dark" alongside the water, because for
    planning purposes "a colour that is not a mark" and "not an object" are the
    same answer.
    """
    lit, hue, saturation, value = _lit_hsv(rgb, config, gains)
    if lit is None or hue is None:
        return {}, 0, {}

    # Sliced by the same mask as the colours, before anything is counted: an
    # age array that has slipped by one point relative to its RGB is worse than
    # no age array at all, so a length mismatch drops the weighting entirely
    # rather than mis-weighting the sweep.
    weights = age_weights(age_ms, config)
    if weights is not None and weights.shape[0] == lit.shape[0]:
        weights = weights[lit]
    else:
        weights = None

    red, green, yellow, white, dark = _colour_masks(hue, saturation, value, config)
    tally = {}

    weighted = {}
    for name, mask in (
        ("red", red),
        ("green", green),
        ("yellow", yellow),
        ("white", white),
        ("dark", dark),
    ):
        count = int(mask.sum())
        if count:
            tally[name] = count
            weighted[name] = (
                float(weights[mask].sum()) if weights is not None else float(count)
            )
    return tally, int(lit.sum()), weighted


class TaskPolicy:
    """How hard to look, and for what, given what the boat is doing.

    Built by `policy_for` from `config`, so every number in it is still a single
    documented value with an environment override - this object decides *which* of
    them applies, never what they are.
    """

    __slots__ = (
        "name", "min_coloured_points", "min_mark_points", "mark_width_m",
        "wide_mark_points", "clutter_range_m", "names_traffic",
    )

    def __init__(self, name, min_coloured_points, min_mark_points, mark_width_m,
                 wide_mark_points, clutter_range_m, names_traffic):
        self.name = name
        self.min_coloured_points = int(min_coloured_points)
        self.min_mark_points = int(min_mark_points)
        self.mark_width_m = float(mark_width_m)
        self.wide_mark_points = int(wide_mark_points)
        self.clutter_range_m = float(clutter_range_m)
        self.names_traffic = bool(names_traffic)

    @property
    def marks_only(self):
        """Whether this task declines to name vessels and structures."""
        return not self.names_traffic

    def tracks(self, kind, range_m, width_m):
        """Whether a measurement of this kind is worth keeping. `(bool, why)`.

        Four answers, in order, and the third is the one that keeps this honest:

          * **a mark** is always kept. That is the task.
          * **water** - blue or dark, which the camera positively reported rather
            than failed to cover - is kept only inside `clutter_range_m`, and is
            never drawn at all (`world.WorldModel.telemetry`). Wave crests and
            spray are not objects, and a chart covered in them is a chart nobody
            reads. Inside that range it stays tracked and avoided, because a dark
            log and dark water are the same returns and only one of the two is
            safe to drive through.
          * **anything mark-SIZED** is always kept, at any range, whatever colour
            it did or did not come out - because it may yet be a mark. Dropping an
            uncoloured 40 cm cluster at 9 m would also drop the camera's chance to
            say "that is green": `world.absorb_detections` only ever refines a
            track that already exists.
          * **wide clutter** - the pier, the shore, a moored hull - is kept only
            inside `clutter_range_m`, which off a marks task is the whole sensor
            range and on one is close enough that the boat still needs to not hit
            it. This is the "a bunch of boats and land" cure, and note what it is
            not: it is a decision about what to REMEMBER, not about what to avoid.
            Everything inside the range is tracked and steered around exactly as
            before, and `emergency_stop_needed` works off the last second of
            returns regardless.
        """
        if kind in MARK_TYPES:
            return True, ""
        if kind == ObstacleType.WATER:
            if range_m <= self.clutter_range_m:
                return True, ""
            return False, (
                f"water at {range_m:.1f} m - the camera coloured it and it is "
                f"not an object"
            )
        if width_m <= self.mark_width_m:
            return True, ""
        if range_m <= self.clutter_range_m:
            return True, ""
        return False, (
            f"{width_m:.1f} m of clutter at {range_m:.1f} m - beyond the "
            f"{self.clutter_range_m:.0f} m this task tracks non-marks to"
        )


def policy_for(context, config):
    """The detection policy for one `behaviours.*.task` string.

    Unknown contexts get the general policy, which is the cautious one: it names
    vessels and structures and tracks everything the sensor can see. Only the
    marks task opts out, and it opts out of *naming*, not of avoiding.
    """
    name = str(context or "transit")
    if name == MARKS_TASK:
        return TaskPolicy(
            name=name,
            min_coloured_points=config.MIN_COLOURED_POINTS,
            min_mark_points=config.MARK_MIN_POINTS,
            mark_width_m=max(
                config.MAX_MARK_WIDTH_M, config.BUOY_TASK_MARK_WIDTH_M
            ),
            wide_mark_points=config.BUOY_TASK_WIDE_MARK_POINTS,
            clutter_range_m=config.BUOY_TASK_CLUTTER_RANGE_M,
            names_traffic=False,
        )
    return TaskPolicy(
        name=name,
        min_coloured_points=config.MIN_COLOURED_POINTS,
        min_mark_points=config.MARK_MIN_POINTS,
        mark_width_m=config.MAX_MARK_WIDTH_M,
        wide_mark_points=config.BUOY_TASK_WIDE_MARK_POINTS,
        # Unbounded, rather than `MAX_OBSTACLE_RANGE_M`: off a marks task nothing
        # is dropped, and the range a return is worth considering at all is
        # `cluster_sweep`'s decision, made once, on the sweep. Repeating it here
        # would quietly re-filter anything handed in from a replay tool or a test
        # at a range the live sweep would never have produced.
        clutter_range_m=float("inf"),
        names_traffic=True,
    )


def classify(cluster, config, context="transit", gains=None):
    """`(ObstacleType, confidence, why)` for one cluster.

    `context` is what the boat is currently doing (`behaviours/*.task`), and
    `policy_for` above turns it into how loose to be and what is worth naming: a
    big white object is the dock while docking and the Otter while giving way,
    and on a buoy leg it is scenery.

    `confidence` is the fraction of the vote the winner took, scaled down when
    there was little of it - a 1-of-1 agreement is unanimous and still weak
    evidence. `why` is a sentence for the operator's log, because NJORD §11.4
    scores the boat explaining itself.
    """
    policy = policy_for(context, config)
    tally, coloured, weighted = (
        colour_votes(cluster.rgb, config, gains, age_ms=cluster.age_ms)
        if cluster.rgb is not None
        else ({}, 0, {})
    )
    painted = sum(tally.get(name, 0) for name in MARK_COLOURS)

    # Too wide to be a mark. Size is a measurement and it beats colour - with one
    # exception, and only on a task about marks: a cluster that is *painted* like
    # a mark is allowed to be a wide one, because a dome at 3 m arrives welded to
    # the water behind it and a gate's two buoys sometimes arrive welded to each
    # other. Two painted returns rather than one, because at this width one is as
    # likely to be a green deck fitting as a buoy.
    if cluster.width_m > config.MAX_MARK_WIDTH_M:
        wide_mark = (
            cluster.width_m <= policy.mark_width_m
            and painted >= policy.wide_mark_points
        )
        if not wide_mark:
            return _large(cluster, config, policy, tally)

    if cluster.rgb is None:
        return (
            ObstacleType.UNKNOWN,
            0.0,
            f"{cluster.width_m:.2f} m object at {cluster.range_m:.1f} m, "
            f"no camera colour ({cluster.source})",
        )

    # How much colour it takes depends on what claim is being made, not on what
    # the boat is doing. `MIN_COLOURED_POINTS` exists because one white return
    # among the water could be anything - so it still governs the white/dark vote,
    # in every context. But one *painted* return is the whole of the evidence there
    # is going to be for a mark caught on the shoulder of the sweep, and holding it
    # to the same bar is what "does not detect buoys that are clearly on the radar"
    # looks like from inside this function.
    if not tally or (painted < policy.min_mark_points
                     and coloured < policy.min_coloured_points):
        return (
            ObstacleType.UNKNOWN,
            0.0,
            f"{cluster.width_m:.2f} m object at {cluster.range_m:.1f} m, "
            f"only {coloured} coloured return(s)",
        )

    winner, fraction, evidence, required, contested = _winner(
        tally, weighted, coloured, config, policy
    )
    if winner is None or fraction < required:
        return (
            ObstacleType.UNKNOWN,
            0.0,
            f"object at {cluster.range_m:.1f} m, colours disagree "
            f"({_tally_text(tally)})",
        )

    # Less evidence is weaker evidence even at 100 % agreement. A mark counts
    # against its own, lower, full-confidence figure - there is only ever going to
    # be one painted return off the shoulder of a dome, and calling that 0.17
    # confident (six being full) made a real mark look like noise.
    full = (
        config.MARK_CONFIDENCE_FULL_POINTS
        if winner in MARK_COLOURS
        else FULL_CONFIDENCE_POINTS
    )
    confidence = fraction * min(1.0, evidence / max(1e-9, full))

    if winner == "red":
        kind = ObstacleType.RED
    elif winner == "green":
        kind = ObstacleType.GREEN
    elif winner == "yellow":
        kind = ObstacleType.CARDINAL
    elif winner == "white":
        # Small and white: at this size it is a fender, a bird or the corner of
        # a pontoon rather than a hull. Give it room, do not name it.
        kind = (
            ObstacleType.LAND
            if policy.name == "dock" and policy.names_traffic
            else ObstacleType.UNKNOWN
        )
    else:  # "dark" - the sea, spray, a shadow
        # WATER rather than UNKNOWN, and the difference is the whole reason the
        # type exists: this is the camera having looked and reported blue water,
        # not the camera having missed the bearing. `TaskPolicy.tracks` keeps it
        # off the chart and out of the memory while still avoiding it close in,
        # which nothing could do while both answers arrived as UNKNOWN.
        return (
            ObstacleType.WATER,
            0.0,
            f"dark return at {cluster.range_m:.1f} m, most likely water",
        )

    # Say so when the timing changed the answer. A cluster that reads green on
    # weight but red on a straight count is the single most useful thing this
    # module can tell an operator, because it is the case where the boat is about
    # to pass on a side that the raw pixel counts do not support. Measured within
    # the same set of names the vote was taken over, or every mark won against a
    # dark background would carry a note saying the raw count said "dark".
    counted = {name: tally[name] for name in contested if name in tally}
    plurality = max(counted, key=counted.get) if counted else winner
    aside = (
        f", on freshness (raw count says {plurality})" if plurality != winner else ""
    )
    # A mark that won against the water says so, because "1/7 returns agree" reads
    # like a bad detection and "1 painted return, 6 background" reads like what it
    # is: a dome the sweep only grazed.
    if winner in MARK_COLOURS and coloured > painted:
        support = (
            f"{tally[winner]} of {painted} painted return(s), "
            f"{coloured - painted} background"
        )
    else:
        support = f"{tally[winner]}/{coloured} returns agree"
    return (
        kind,
        confidence,
        f"{winner} {cluster.width_m:.2f} m at {cluster.range_m:.1f} m, "
        f"{support}{aside}",
    )


def _winner(tally, weighted, coloured, config, policy):
    """Who won the colour vote. `(name, fraction, evidence, required, contested)`.

    The winner is decided on WEIGHT - a colour sampled from a well-timed frame
    outvotes one sampled from a frame a quarter-second away - but the fraction is
    measured against the weight actually cast, so a cluster whose returns are
    uniformly stale is not penalised for it. Only disagreement is.

    `contested` is the set of names that were allowed to compete, and it is the
    whole of the change here. When any mark colour is present the vote runs among
    the mark colours alone: white and dark are the background a mark stands in
    front of, not a rival claim about what it is. The bar on that subset is
    *higher* than the ordinary one (`MARK_COLOUR_VOTE_FRACTION`), because the one
    disagreement that matters - a red and a green in the same cluster - has to
    stay unresolved. See `config.MARK_COLOUR_WINS`.
    """
    marks = {name: weighted[name] for name in MARK_COLOURS if name in weighted}
    painted = sum(tally.get(name, 0) for name in MARK_COLOURS)
    if config.MARK_COLOUR_WINS and marks and painted >= policy.min_mark_points:
        total = sum(marks.values())
        winner = max(marks, key=marks.get)
        fraction = (marks[winner] / total) if total > 0 else 0.0
        return (
            winner, fraction, painted, config.MARK_COLOUR_VOTE_FRACTION,
            tuple(marks),
        )

    if not weighted:
        return None, 0.0, 0, config.COLOUR_VOTE_FRACTION, ()
    total = sum(weighted.values())
    winner = max(weighted, key=weighted.get)
    fraction = (weighted[winner] / total) if total > 0 else 0.0
    return (
        winner, fraction, coloured, config.COLOUR_VOTE_FRACTION, tuple(weighted),
    )


def _large(cluster, config, policy, tally=None):
    """Something wider than a mark: a vessel, a structure, or nobody's business.

    `tally` is the colour count the caller has already paid for; None means work
    it out here, which is what a direct call from a test or a replay tool does.
    """
    if tally is None:
        tally, _coloured, _weighted = (
            colour_votes(cluster.rgb, config, None, age_ms=cluster.age_ms)
            if cluster.rgb is not None
            else ({}, 0, {})
        )
    hint = f" ({_tally_text(tally)})" if tally else ""

    if policy.marks_only:
        # A marks task. The shore, the pier and a moored hull are all real and all
        # beside the point, and naming them buys three things the task does not
        # want: a vessel's 6 m clearance, the COLREG machinery, and a permanent
        # entry in the survey. UNKNOWN keeps every one of them out while leaving
        # the object tracked, drawn and avoided on both sides.
        return (
            ObstacleType.UNKNOWN,
            0.0,
            f"{cluster.width_m:.1f} m object at {cluster.range_m:.1f} m - too "
            f"wide for a mark; not named, this task is about the marks{hint}",
        )

    # NJORD §9.2 puts the Otter at 2.0 x 1.08 m, which is between a buoy and a
    # pier. The handbook warns its colour and form may vary, so this is decided
    # on SIZE, and colour only sharpens the confidence.
    vessel_like = cluster.width_m <= config.OTTER_LENGTH_M * 1.6

    if policy.name == "dock":
        # In a berth everything solid is structure. Calling the dock a vessel
        # here would make the boat give way to the thing it is trying to reach.
        return (
            ObstacleType.LAND,
            0.7,
            f"{cluster.width_m:.1f} m structure at {cluster.range_m:.1f} m{hint}",
        )

    if vessel_like:
        return (
            ObstacleType.BOAT,
            0.6,
            f"{cluster.width_m:.1f} m object at {cluster.range_m:.1f} m - "
            f"vessel-sized{hint}",
        )
    return (
        ObstacleType.LAND,
        0.8,
        f"{cluster.width_m:.1f} m of structure at {cluster.range_m:.1f} m{hint}",
    )


def _tally_text(tally):
    return ", ".join(f"{name} {count}" for name, count in sorted(tally.items()))


class CardinalVote:
    """Accumulates the camera's opinion of which cardinal a mark is.

    This is the one classification the lidar cannot make: north/south/east/west
    is the topmark's two black cones, not a colour and not a size. The Jetson's
    second-stage classifier reports it as `card` with a `card_conf`
    (`edge_protocol.py`), and that model is not trusted on this boat.

    So it is treated as a poll rather than an answer. Every observation above
    `CARDINAL_MIN_CONF` is a vote, votes accumulate per track, and the direction
    is only *committed* once the leader has `CARDINAL_VOTES_REQUIRED` votes and
    a clear margin over the runner-up. Until then the mark is a plain CARDINAL
    and `behaviours/buoys.py` falls back to the planned route's own side.

    The margin matters as much as the count. A detector that flickers
    east/west/east/west accumulates four votes quickly and has told you nothing;
    requiring the leader to be ahead by two is what distinguishes agreement from
    noise.
    """

    def __init__(self, config):
        self._config = config
        self._votes = {}
        self.committed = None

    def add(self, name, confidence):
        """One observation. `name` is "north"/"south"/"east"/"west"."""
        if self.committed is not None:
            return
        if not name or confidence is None:
            return
        if confidence < self._config.CARDINAL_MIN_CONF:
            return
        key = str(name).strip().lower()
        if key not in ("north", "south", "east", "west"):
            return
        self._votes[key] = self._votes.get(key, 0) + 1
        self._settle()

    def _settle(self):
        if len(self._votes) == 0:
            return
        ranked = sorted(self._votes.items(), key=lambda item: item[1], reverse=True)
        leader, count = ranked[0]
        runner_up = ranked[1][1] if len(ranked) > 1 else 0
        if count >= self._config.CARDINAL_VOTES_REQUIRED and count - runner_up >= 2:
            self.committed = leader

    @property
    def tally(self):
        return dict(self._votes)

    def describe(self):
        if self.committed:
            # `.get`, not `[]`: a vote tally restored from the survey file can in
            # principle name a committed direction it has no votes for, and this
            # string is built on the 10 Hz tick inside `Track.telemetry()`.
            return (
                f"{self.committed} cardinal "
                f"({self._votes.get(self.committed, 0)} votes)"
            )
        if not self._votes:
            return "cardinal, no camera vote yet"
        return "cardinal, votes " + _tally_text(self._votes) + " - not committed"
