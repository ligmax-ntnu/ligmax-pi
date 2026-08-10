"""Lidar returns -> straight edges. The one place a point cloud becomes a line.

    segments = fit_sweeps(ctx.sweeps, config=config)   # both lidars
    segments = fit_segments(points_stbd_fwd, config=config)   # one sweep

Input is the BOAT frame, `[starboard, forward]` metres, exactly as
`nodes/io_manager/scan.py` builds it and exactly what `cluster.py` takes - so the
same function works on the coloured front unit and the uncoloured aft one with no
special cases.

Why this is not `cluster.py`
---------------------------
A cluster answers "is there a thing there, and how wide is it". That is the right
question for a buoy and the wrong question for a wall: a 2 m plank of dock reads
as one enormous cluster whose centroid sits in the middle of the plank and whose
`nearest` is wherever the beams happened to land. Neither of those is the thing
the parking task is about, which is the plank's **line** - its direction, and
which side of it the water is on.

So this module fits lines and `cluster.py` fits objects, and the parking
behaviours use only this one. `perception/parking.py` is what turns three of
these into a berth.

How the fit works, and why this shape
-------------------------------------
Split-and-merge on the sensor's own angular order, which is the same trick
`cluster.py` plays and for the same reason: a rotating 2-D lidar hands you
returns **in angular order**, so neighbouring array entries are neighbouring
points in space, and no general-purpose line detector has to rediscover that.

    1. cut the sweep wherever consecutive returns jump apart, exactly as
       `cluster.py` does, with the same range-dependent gap. A parking box whose
       corners do not meet is cut into three runs here, for free, by the corner
       gaps themselves.
    2. split each run recursively at its worst deviation from the chord until
       every piece is straight to `LINE_TOLERANCE_M`. This is what separates two
       walls that *do* meet at a corner.
    3. fit each piece by total least squares (the principal axis of its
       covariance), which is the right fit for a line with error in both
       coordinates - an ordinary y-on-x regression cannot represent a wall the
       boat is looking down the length of, because that wall is vertical in
       sensor coordinates and the slope is infinite.
    4. merge pieces that are collinear and nearly touching, because a wall with
       a shadow across it arrives as two runs.

Why not a Hough transform: it needs a parameter grid, its resolution is a
compromise between finding a 0.6 m plank and separating two walls 2 m apart, and
it answers with infinite lines when what the geometry needs is **endpoints** -
the box finder's whole job is deciding which line ends where the corner gap
starts.

An axis, not a direction
------------------------
`Segment.axis_deg` is folded onto **[0, 180)**. A wall has no front: the same
plank scanned from either end is the same plank, and code that carries a
direction for it spends its life wondering which of two answers it has. Compare
two of them with `axis_diff`, which returns 0-90 and knows that 179 deg and
1 deg are two degrees apart.
"""

import math

import numpy as np


class Segment:
    """One straight edge, as measured on one sweep. BOAT frame, metres.

    Attributes:
        a, b        the endpoints, `(starboard, forward)`, ordered along `axis_deg`
        midpoint    halfway between them
        axis_deg    the line's direction as a relative bearing folded onto
                    [0, 180): 0 runs fore-and-aft, 90 runs athwartships
        length_m    distance from `a` to `b`
        range_m     range to `midpoint` - how far away this edge is
        rms_m       how straight it actually was. The honest quality figure
        n           returns in it
        source      which sensor saw it, for the operator's log
    """

    __slots__ = ("a", "b", "midpoint", "axis_deg", "length_m", "range_m", "rms_m",
                 "n", "source")

    def __init__(self, a, b, axis_deg, length_m, rms_m, n, source):
        self.a = a
        self.b = b
        self.midpoint = (0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1]))
        self.axis_deg = fold_axis(axis_deg)
        self.length_m = length_m
        self.range_m = math.hypot(*self.midpoint)
        self.rms_m = rms_m
        self.n = n
        self.source = source

    # -- geometry ----------------------------------------------------------

    @property
    def direction(self):
        """Unit vector along the line, `(starboard, forward)`.

        The relative-bearing convention this repo uses everywhere: 0 deg is
        forward, 90 deg is starboard, so a bearing `t` is `(sin t, cos t)` in
        `(starboard, forward)`. See `geo.py`.
        """
        rad = math.radians(self.axis_deg)
        return (math.sin(rad), math.cos(rad))

    @property
    def normal(self):
        """Unit vector across the line - `direction` turned 90 deg to starboard."""
        rad = math.radians(self.axis_deg)
        return (math.cos(rad), -math.sin(rad))

    def offset_of(self, point):
        """Signed distance from the line to `point`, along `normal`."""
        return dot(subtract(point, self.midpoint), self.normal)

    def along_of(self, point):
        """Where `point` falls along the line, metres from `midpoint`."""
        return dot(subtract(point, self.midpoint), self.direction)

    def __repr__(self):
        return (
            f"<Segment {self.source} {self.length_m:.2f}m @{self.axis_deg:.0f}deg "
            f"r={self.range_m:.1f}m rms={self.rms_m * 100:.0f}cm n={self.n}>"
        )


# ------------------------------------------------------------------ vectors
#
# Two-element tuples rather than numpy arrays: the box finder does a few hundred
# of these per tick on a Pi 5 sharing a 10 Hz loop with everything else, and a
# numpy call costs more in overhead than this arithmetic costs in total.

def dot(u, v):
    return u[0] * v[0] + u[1] * v[1]


def subtract(u, v):
    return (u[0] - v[0], u[1] - v[1])


def add(u, v):
    return (u[0] + v[0], u[1] + v[1])


def scale(u, k):
    return (u[0] * k, u[1] * k)


def fold_axis(degrees):
    """Any direction onto [0, 180). A line has no front - see the module notes."""
    return degrees % 180.0


def axis_diff(a_deg, b_deg):
    """Angle between two undirected axes, 0-90 degrees.

    The fold is the whole point: two walls at 179 deg and 1 deg are two degrees
    from parallel, and a naive difference calls them 178 and rejects the pair.
    """
    difference = abs((a_deg - b_deg) % 180.0)
    return min(difference, 180.0 - difference)


def mean_axis(segments):
    """Length-weighted mean axis of several segments, degrees in [0, 180).

    Averaged as **doubled angles on the unit circle**, which is the standard way
    to average an undirected direction: doubling maps [0, 180) onto a full circle,
    so 179 deg and 1 deg average to 0 rather than to 90. Averaging the folded
    degrees directly gets that case exactly wrong, and "exactly wrong" here means
    a berth axis at right angles to the real one.
    """
    x = y = 0.0
    for segment in segments:
        weight = max(segment.length_m, 1e-6)
        doubled = math.radians(segment.axis_deg * 2.0)
        x += weight * math.cos(doubled)
        y += weight * math.sin(doubled)
    if abs(x) < 1e-12 and abs(y) < 1e-12:
        return segments[0].axis_deg if segments else 0.0
    return fold_axis(math.degrees(math.atan2(y, x)) * 0.5)


def unit_of(axis_deg):
    """`(starboard, forward)` unit vector along an axis bearing."""
    rad = math.radians(axis_deg)
    return (math.sin(rad), math.cos(rad))


def normal_of(axis_deg):
    """`(starboard, forward)` unit vector across an axis bearing."""
    rad = math.radians(axis_deg)
    return (math.cos(rad), -math.sin(rad))


def bearing_of(vector):
    """A `(starboard, forward)` vector as a relative bearing in [0, 360)."""
    if abs(vector[0]) < 1e-12 and abs(vector[1]) < 1e-12:
        return 0.0
    return math.degrees(math.atan2(vector[0], vector[1])) % 360.0


# --------------------------------------------------------------------- fitting

def fit_sweeps(sweeps, *, config, sources=None):
    """Every sweep -> one list of `Segment`. The entry point a behaviour uses.

    `sweeps` is what `main.py` already has: a list of scan dicts, each with a
    `source` and boat-frame `points`. `sources` optionally restricts which
    sensors are trusted - the parking behaviours use it to leave the aft unit out
    until its mounting geometry has been checked against a tape, because a
    flipped `LIGMAX_AFT_LIDAR_ANGLE_DIR` produces a complete and **mirrored**
    world astern (docs/testing.md 7c).

    Each sweep is fitted **separately and then concatenated**, which is not an
    optimisation but a correctness requirement: step 1 relies on the array being
    in one sensor's angular order, and two sweeps merged into one array
    interleave at every shared bearing. A single 2 m wall then arrives as a
    dozen fragments, none of them long enough to be a berth wall.
    """
    out = []
    for sweep in sweeps or ():
        if not isinstance(sweep, dict):
            continue
        source = str(sweep.get("source") or "lidar")
        if sources is not None and source not in sources:
            continue
        out.extend(fit_segments(sweep.get("points"), config=config, source=source))
    return out


def fit_segments(points, *, config, source="lidar"):
    """One sweep -> a list of `Segment`. Never raises on odd input.

    The length test is applied **after** the merge, not before it, and that
    ordering is not cosmetic. Range noise makes the splitter cut a straight wall
    at whichever return happens to sit furthest off the chord, so a 2 m wall
    measured with 3 cm of noise arrives as three or four short pieces. Filtering
    on length first throws all of them away individually and the wall disappears -
    which is exactly the failure that "no parking space found" looks like from the
    dock, with a wall plainly in front of the boat.
    """
    pts = _prepare(points, config)
    if pts is None:
        return []

    pieces = []
    for lo, hi in _runs(pts, config):
        if hi - lo < config.LINE_MIN_POINTS:
            continue
        for start, end in _split(pts, lo, hi, config.LINE_TOLERANCE_M):
            piece = _fit(pts, start, end, config, source)
            if piece is not None:
                pieces.append(piece)

    merged = _merge(pieces, config)
    return [segment for segment in merged if segment.length_m >= config.LINE_MIN_M]


def _prepare(points, config):
    """Finite, in-range returns in angular order, or None if there are too few."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] < 2 or pts.shape[1] != 2:
        return None
    pts = pts[np.isfinite(pts).all(axis=1)]
    if pts.shape[0] < 2:
        return None

    ranges = np.hypot(pts[:, 0], pts[:, 1])
    keep = (ranges >= config.MIN_OBSTACLE_RANGE_M) & (ranges <= config.LINE_MAX_RANGE_M)
    if keep.sum() < 2:
        return None
    pts = pts[keep]

    # `arctan2(starboard, forward)`: the relative-bearing convention used
    # everywhere in this repo. 0 ahead, positive to starboard.
    order = np.argsort(np.degrees(np.arctan2(pts[:, 0], pts[:, 1])))
    return pts[order]


def _runs(pts, config):
    """`[lo, hi)` index ranges of returns that are angular neighbours in space.

    The same range-dependent gap as `cluster.py`, and deliberately the same
    numbers: the beams diverge, so a fixed gap either shatters a distant wall or
    welds two nearby ones together, and there is no reason for the two modules to
    disagree about where one object stops.

    Unlike `cluster.py` this does **not** re-join the +-180 deg seam. A wall
    crossing dead astern is fitted as two pieces and `_merge` puts them back
    together if they really are collinear - which is the same answer by a route
    that cannot accidentally fit one line through two walls that merely happen to
    sit either side of the seam.
    """
    ranges = np.hypot(pts[:, 0], pts[:, 1])
    steps = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    gaps = config.CLUSTER_GAP_M + config.CLUSTER_GAP_PER_M * np.minimum(
        ranges[:-1], ranges[1:]
    )
    cuts = [0, *(int(i) + 1 for i in np.flatnonzero(steps > gaps)), pts.shape[0]]
    return [(cuts[i], cuts[i + 1]) for i in range(len(cuts) - 1)]


def _split(pts, lo, hi, tolerance):
    """Cut `[lo, hi)` at its worst deviation until every piece is straight.

    Iterative rather than recursive: a sweep is up to a few thousand returns and
    a pathological one (a curved shoreline at 12 m) would otherwise recurse once
    per point. The pivot is included in **both** halves, which is what keeps the
    corner itself on both walls rather than orphaning it.
    """
    pieces = []
    stack = [(lo, hi)]
    while stack:
        start, end = stack.pop()
        if end - start < 2:
            continue
        worst, pivot, supported = _max_deviation(pts, start, end, tolerance)
        if (
            worst <= tolerance
            or not supported
            or pivot <= start
            or pivot >= end - 1
        ):
            pieces.append((start, end))
            continue
        stack.append((start, pivot + 1))
        stack.append((pivot, end))
    pieces.sort()
    return pieces


def _max_deviation(pts, start, end, tolerance):
    """`(worst distance, index, supported)` from the chord across `[start, end)`.

    `supported` is whether a **neighbour** of the worst point also stands off the
    chord, and it is what keeps one bad return from cutting a wall in half.

    Splitting on a lone outlier is the classic weakness of this family of
    algorithms and it bites here for a specific reason: at 0.9 deg the beams are
    8 cm apart at 5 m, so a single return 10 cm off a flat wall is one sample of
    range noise and not a corner. Cutting there leaves two pieces of eight points
    each, both of which then fail the minimum-points test, and a wall plainly in
    front of the boat is reported as no wall at all.

    A real corner has every point after it off the chord, so it is supported many
    times over and this costs it nothing.
    """
    a = pts[start]
    b = pts[end - 1]
    along = b - a
    length = math.hypot(along[0], along[1])
    if length < 1e-9:
        return 0.0, start, False
    # Perpendicular distance of every member from the chord, by 2-D cross product.
    member = pts[start:end] - a
    cross = np.abs(member[:, 0] * along[1] - member[:, 1] * along[0]) / length
    index = int(np.argmax(cross))
    neighbours = [
        cross[i] for i in (index - 1, index + 1) if 0 <= i < cross.shape[0]
    ]
    supported = any(value > tolerance * 0.5 for value in neighbours)
    return float(cross[index]), start + index, supported


def _fit(pts, start, end, config, source):
    """Total-least-squares fit of `[start, end)`, or None if it is not a line.

    Does **not** test the length - `fit_segments` does that after the merge, for
    the reason its docstring gives.
    """
    member = pts[start:end]
    n = member.shape[0]
    if n < config.LINE_MIN_POINTS:
        return None

    centroid = member.mean(axis=0)
    centred = member - centroid
    # Principal axis of the covariance: the direction that leaves the least
    # perpendicular residual, which is what a line through noisy points in both
    # coordinates means. `eigh` on a 2x2 symmetric matrix, largest eigenvalue last.
    covariance = centred.T @ centred / n
    values, vectors = np.linalg.eigh(covariance)
    direction = vectors[:, int(np.argmax(values))]
    normal = np.array([direction[1], -direction[0]])

    residuals = centred @ normal
    rms = float(np.sqrt(float(np.mean(residuals * residuals))))
    if rms > config.LINE_TOLERANCE_M * config.LINE_RMS_SLACK:
        return None

    along = centred @ direction
    low, high = float(along.min()), float(along.max())
    length = high - low
    if length <= 1e-6:
        return None

    a = centroid + direction * low
    b = centroid + direction * high
    axis = math.degrees(math.atan2(float(direction[0]), float(direction[1])))
    return Segment(
        a=(float(a[0]), float(a[1])),
        b=(float(b[0]), float(b[1])),
        axis_deg=axis,
        length_m=length,
        rms_m=rms,
        n=n,
        source=source,
    )


def _merge(segments, config):
    """Join pieces that are one edge with a gap in it. Bounded passes.

    A wall with a mooring cleat in front of it, or a shadow across it, arrives as
    two runs that step 1 has already decided are separate objects. Left alone
    each piece may be under `LINE_MIN_M` and the wall is invisible.

    The merge is **geometric, not a refit**: the member points are long gone by
    here, so the joined segment takes the length-weighted axis and the extreme
    endpoints, and its `rms_m` is the worse of the two rather than a recomputed
    figure. That is an approximation and it is stated in `rms_m`'s docstring, but
    it is the conservative direction - a merged segment never claims to be
    straighter than either half was.
    """
    out = list(segments)
    for _pass in range(config.LINE_MERGE_PASSES):
        joined = _merge_once(out, config)
        if joined is None:
            break
        out = joined
    return sorted(out, key=lambda segment: segment.range_m)


def _merge_once(segments, config):
    """One merge, or None if nothing could be merged."""
    for i, first in enumerate(segments):
        for j in range(i + 1, len(segments)):
            second = segments[j]
            if first.source != second.source:
                # Two sensors looking at one wall genuinely see it in two places
                # if the mounting geometry is off, and merging across that would
                # hide exactly the error `docs/testing.md` 7c is hunting for.
                continue
            if axis_diff(first.axis_deg, second.axis_deg) > config.LINE_MERGE_DEG:
                continue
            longer, shorter = (
                (first, second) if first.length_m >= second.length_m else (second, first)
            )
            if max(
                abs(longer.offset_of(shorter.a)), abs(longer.offset_of(shorter.b))
            ) > config.LINE_MERGE_OFFSET_M:
                continue
            if _span_gap(longer, shorter) > config.LINE_MERGE_GAP_M:
                continue
            merged = _join(first, second)
            return [merged, *(s for k, s in enumerate(segments) if k not in (i, j))]
    return None


def _span_gap(longer, shorter):
    """Metres of daylight between two segments along the longer one's axis."""
    a_lo, a_hi = sorted((longer.along_of(longer.a), longer.along_of(longer.b)))
    b_lo, b_hi = sorted((longer.along_of(shorter.a), longer.along_of(shorter.b)))
    if b_lo > a_hi:
        return b_lo - a_hi
    if a_lo > b_hi:
        return a_lo - b_hi
    return 0.0  # they overlap


def _join(first, second):
    axis = mean_axis([first, second])
    direction = unit_of(axis)
    points = (first.a, first.b, second.a, second.b)
    weight = first.n + second.n
    centroid = (
        sum(p[0] for p in points) / 4.0,
        sum(p[1] for p in points) / 4.0,
    )
    along = [dot(subtract(p, centroid), direction) for p in points]
    low, high = min(along), max(along)
    return Segment(
        a=add(centroid, scale(direction, low)),
        b=add(centroid, scale(direction, high)),
        axis_deg=axis,
        length_m=high - low,
        rms_m=max(first.rms_m, second.rms_m),
        n=weight,
        source=first.source,
    )
