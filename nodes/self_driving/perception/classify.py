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

Voting, and why UNKNOWN is a good answer
----------------------------------------
A cluster is classified by majority vote of its coloured returns, and needs
`COLOUR_VOTE_FRACTION` of them to agree. Falling short returns UNKNOWN, which is
deliberately safe: an unknown obstacle is given room on *both* sides, whereas a
guessed one is passed confidently on whichever side the guess implied. In a task
where passing on the wrong side is the failure being scored, a shrug beats a
coin flip.
"""

import numpy as np

from ..obsticales import ObstacleType

# What `scan.py` writes into `rgb` for a return no camera could colour. Not
# black on purpose - most of a rotation is behind both lenses, and "uncoloured"
# must never be confusable with "genuinely dark".
NO_COLOUR = -1


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
    arr = np.asarray(rgb)
    if arr.size == 0:
        return {}, 0, {}
    if arr.ndim == 1:
        arr = arr.reshape(-1, 3)
    # A single -1 channel marks the whole return as uncoloured.
    lit = ~(arr <= NO_COLOUR).any(axis=1)
    if not lit.any():
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

    arr = arr[lit]

    if gains is not None:
        # Applied before the hue is taken, and clipped back into range so a gain
        # cannot push a channel past white and invert the hue it was correcting.
        arr = np.clip(np.asarray(arr, dtype=np.float64) * np.asarray(gains), 0, 255)

    hue, saturation, value = rgb_to_hsv(arr)
    tally = {}

    strong = saturation >= config.MIN_SATURATION
    red = strong & (
        (hue <= config.HUE_RED_LOW_MAX) | (hue >= config.HUE_RED_HIGH_MIN)
    )
    yellow = strong & (hue >= config.HUE_YELLOW_MIN) & (hue <= config.HUE_YELLOW_MAX)
    green = strong & (hue >= config.HUE_GREEN_MIN) & (hue <= config.HUE_GREEN_MAX)
    # Red is tested first and wins any overlap with yellow at the seam, because
    # a red buoy misread as a cardinal costs a wrong-side pass while the reverse
    # costs a cautious detour.
    yellow &= ~red
    green &= ~red & ~yellow

    grey = ~strong
    white = grey & (value >= config.WHITE_MIN_VALUE)
    dark = grey & ~white

    # Everything that had a hue but not one of the three - blues, purples.
    other = strong & ~red & ~yellow & ~green

    weighted = {}
    for name, mask in (
        ("red", red),
        ("green", green),
        ("yellow", yellow),
        ("white", white),
        ("dark", dark | other),
    ):
        count = int(mask.sum())
        if count:
            tally[name] = count
            weighted[name] = (
                float(weights[mask].sum()) if weights is not None else float(count)
            )
    return tally, int(arr.shape[0]), weighted


def classify(cluster, config, context="transit", gains=None):
    """`(ObstacleType, confidence, why)` for one cluster.

    `context` is what the boat is currently doing, and it exists for exactly one
    ambiguity: a big white object. During docking that is the dock; during
    collision avoidance it is the Otter. Nothing else consults it.

    `confidence` is the fraction of coloured returns that voted for the winner,
    scaled down when there were few of them - a 2-of-2 agreement is unanimous
    and still weak evidence. `why` is a sentence for the operator's log, because
    NJORD §11.4 scores the boat explaining itself.
    """
    # Too wide to be a mark. Size is a measurement and it beats colour.
    if cluster.width_m > config.MAX_MARK_WIDTH_M:
        return _large(cluster, config, context, gains)

    if cluster.rgb is None:
        return (
            ObstacleType.UNKNOWN,
            0.0,
            f"{cluster.width_m:.2f} m object at {cluster.range_m:.1f} m, "
            f"no camera colour ({cluster.source})",
        )

    tally, coloured, weighted = colour_votes(
        cluster.rgb, config, gains, age_ms=cluster.age_ms
    )
    if coloured < config.MIN_COLOURED_POINTS or not tally:
        return (
            ObstacleType.UNKNOWN,
            0.0,
            f"{cluster.width_m:.2f} m object at {cluster.range_m:.1f} m, "
            f"only {coloured} coloured return(s)",
        )

    # The winner is decided on WEIGHT - a colour sampled from a well-timed frame
    # outvotes one sampled from a frame a quarter-second away - but the fraction
    # is measured against the weight actually cast, so a cluster whose returns
    # are uniformly stale is not penalised for it. Only disagreement is.
    winner = max(weighted, key=weighted.get)
    total_weight = sum(weighted.values())
    fraction = (weighted[winner] / total_weight) if total_weight > 0 else 0.0
    votes = tally[winner]
    if fraction < config.COLOUR_VOTE_FRACTION:
        return (
            ObstacleType.UNKNOWN,
            0.0,
            f"object at {cluster.range_m:.1f} m, colours disagree "
            f"({_tally_text(tally)})",
        )

    # Fewer returns is weaker evidence even at 100 % agreement.
    confidence = fraction * min(1.0, coloured / 6.0)

    if winner == "red":
        kind = ObstacleType.RED
    elif winner == "green":
        kind = ObstacleType.GREEN
    elif winner == "yellow":
        kind = ObstacleType.CARDINAL
    elif winner == "white":
        # Small and white: at this size it is a fender, a bird or the corner of
        # a pontoon rather than a hull. Give it room, do not name it.
        kind = ObstacleType.LAND if context == "dock" else ObstacleType.UNKNOWN
    else:  # "dark" - water, spray, shadow
        return (
            ObstacleType.UNKNOWN,
            0.0,
            f"dark return at {cluster.range_m:.1f} m, most likely water",
        )

    # Say so when the timing changed the answer. A cluster that reads green on
    # weight but red on a straight count is the single most useful thing this
    # module can tell an operator, because it is the case where the boat is about
    # to pass on a side that the raw pixel counts do not support.
    plurality = max(tally, key=tally.get)
    aside = (
        f", on freshness (raw count says {plurality})" if plurality != winner else ""
    )
    return (
        kind,
        confidence,
        f"{winner} {cluster.width_m:.2f} m at {cluster.range_m:.1f} m, "
        f"{votes}/{coloured} returns agree{aside}",
    )


def _large(cluster, config, context, gains=None):
    """Something wider than a mark: a vessel, or a structure."""
    # Colour is only a hint at this size - the decision below is made on width -
    # so the raw tally is what gets shown, unweighted.
    tally, _coloured, _weighted = (
        colour_votes(cluster.rgb, config, gains, age_ms=cluster.age_ms)
        if cluster.rgb is not None
        else ({}, 0, {})
    )
    hint = f" ({_tally_text(tally)})" if tally else ""

    # NJORD §9.2 puts the Otter at 2.0 x 1.08 m, which is between a buoy and a
    # pier. The handbook warns its colour and form may vary, so this is decided
    # on SIZE, and colour only sharpens the confidence.
    vessel_like = cluster.width_m <= config.OTTER_LENGTH_M * 1.6

    if context == "dock":
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
