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
The values on the wire are **sensor-native, uncalibrated** - the OV5647's colour
matrix runs at the receiver, so these are the raw numbers the detector's boxes
were drawn from. Absolute RGB therefore drifts with exposure, with the time of
day and between the two cameras. Hue survives all three: a red buoy in shade and
the same buoy in sun differ enormously in *value* and barely at all in *hue*.
Saturation then separates "a colour" from "a grey", and value separates white
from black among the greys.

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


def colour_votes(rgb, config, gains=None):
    """Per-return colour names, as a `{name: count}` tally. Uncoloured ignored.

    Names are the five the boat reasons in: "red", "green", "yellow", "white",
    "dark". Anything with a hue that is none of the three - a blue fender, the
    sky in a reflection - lands in "dark" alongside the water, because for
    planning purposes "a colour that is not a mark" and "not an object" are the
    same answer.
    """
    arr = np.asarray(rgb)
    if arr.size == 0:
        return {}, 0
    if arr.ndim == 1:
        arr = arr.reshape(-1, 3)
    # A single -1 channel marks the whole return as uncoloured.
    lit = ~(arr <= NO_COLOUR).any(axis=1)
    if not lit.any():
        return {}, 0
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
    return tally, int(arr.shape[0])


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

    tally, coloured = colour_votes(cluster.rgb, config, gains)
    if coloured < config.MIN_COLOURED_POINTS or not tally:
        return (
            ObstacleType.UNKNOWN,
            0.0,
            f"{cluster.width_m:.2f} m object at {cluster.range_m:.1f} m, "
            f"only {coloured} coloured return(s)",
        )

    winner, votes = max(tally.items(), key=lambda item: item[1])
    fraction = votes / coloured
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

    return (
        kind,
        confidence,
        f"{winner} {cluster.width_m:.2f} m at {cluster.range_m:.1f} m, "
        f"{votes}/{coloured} returns agree",
    )


def _large(cluster, config, context, gains=None):
    """Something wider than a mark: a vessel, or a structure."""
    tally, coloured = (
        colour_votes(cluster.rgb, config, gains) if cluster.rgb is not None else ({}, 0)
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
            return f"{self.committed} cardinal ({self._votes[self.committed]} votes)"
        if not self._votes:
            return "cardinal, no camera vote yet"
        return "cardinal, votes " + _tally_text(self._votes) + " - not committed"
