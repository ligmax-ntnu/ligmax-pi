"""Lidar returns -> objects. The one place a point cloud becomes a thing.

    clusters = cluster_sweep(points_stbd_fwd, rgb=rgb_or_none)

Input is the BOAT frame, `[starboard, forward]` metres, exactly as
`nodes/io_manager/scan.py` builds it - so the same function works on the front
unit (coloured, forward-looking) and the aft one (no colour, astern) with no
special cases.

Why angular clustering rather than DBSCAN
-----------------------------------------
A rotating 2-D lidar does not produce a general point cloud. It produces returns
**in angular order**, one per beam, and neighbouring beams that hit the same
object arrive next to each other in the array. So sorting by bearing and cutting
wherever consecutive returns jump apart is a single O(n log n) pass, and it is
exactly right for this sensor - whereas DBSCAN would spend milliseconds
rediscovering an ordering the hardware already gave us. At 10 Hz on a Pi 5,
sharing a loop with everything else, that matters.

The gap that splits two objects has to grow with range, because the beams
diverge: the C1 steps 0.9 deg, which is 3 cm apart at 2 m and 16 cm at 10 m. A
fixed gap either shatters a distant buoy into single-point fragments or welds
two nearby ones into one. Hence `gap = CLUSTER_GAP_M + CLUSTER_GAP_PER_M *
range`.

What a cluster is, and what it is not
-------------------------------------
A cluster is a **measurement**, not an object with an identity - it exists for
one sweep and is thrown away. Identity, history and velocity are
`world.py`'s job, and keeping them apart is what lets this module stay a pure
function of one sweep and be tested against a captured file with no boat
attached.

The width figure is the honest one to threshold on: `width_m` is the chord
across the cluster, so a 40 cm Njord buoy reads 0.2-0.4 m depending on how much
of it the beams caught, a 2 m Otter reads 1-2 m, and a pier reads as far as the
sensor can see. `MAX_MARK_WIDTH_M` is the line between "a mark" and "a
structure", and it is drawn well above the buoy's real diameter because a
cluster at close range catches the near face of two adjacent objects as one.
"""

import math

import numpy as np

from .classify import white_balance_gains


class Cluster:
    """One object, as measured on one sweep. BOAT frame, metres.

    Attributes:
        centre      `(starboard, forward)` of the cluster's centroid
        nearest     `(starboard, forward)` of its closest return - the point
                    that matters for not hitting it
        range_m     range to `centre`
        bearing_deg relative bearing to `centre`: 0 ahead, + to starboard
        width_m     chord across the cluster
        n           returns in it
        rgb         `(n, 3)` uint8 array, or None for an uncoloured sensor
        age_ms      `(n,)` of how mistimed each return's colour was, or None
        source      which sensor saw it, for the operator's log
    """

    __slots__ = (
        "centre", "nearest", "range_m", "bearing_deg", "width_m", "n", "rgb", "source",
        "gains", "age_ms",
    )

    def __init__(self, centre, nearest, range_m, bearing_deg, width_m, n, rgb, source,
                 gains=None, age_ms=None):
        self.centre = centre
        self.nearest = nearest
        self.range_m = range_m
        self.bearing_deg = bearing_deg
        self.width_m = width_m
        self.n = n
        self.rgb = rgb
        # How far the camera frame that coloured each return sat from the return
        # itself, in milliseconds (`edge_protocol.py`). Carried per point rather
        # than per sweep because the Jetson colours each return from the nearest
        # of several buffered frames, so one sweep genuinely holds a spread of
        # ages. None for a sensor that sends no age, and for the aft unit, which
        # sends no colour at all.
        self.age_ms = age_ms
        self.source = source
        # The sweep's white-balance gains, carried here rather than recomputed
        # per cluster: a colour cast is a property of the whole scene, and a
        # handful of returns off one buoy is far too small a sample to estimate
        # it from. None when white balance is off (the default).
        self.gains = gains

    def __repr__(self):
        return (
            f"<Cluster {self.source} {self.range_m:.1f}m @{self.bearing_deg:+.0f}deg "
            f"w={self.width_m:.2f}m n={self.n}>"
        )


def cluster_sweep(points, rgb=None, source="lidar", *, config, coloured_mask=None,
                  age_ms=None):
    """One sweep -> a list of `Cluster`. Never raises on odd input.

    `points` is an `(n, 2)` array-like of `[starboard, forward]` metres.
    `rgb` is `(n, 3)`, or a flat `3n` list as it arrives on the wire, or None.
    `coloured_mask` is an optional length-n boolean saying which points a camera
    actually saw - `scan.py` marks the rest `-1, -1, -1` rather than black,
    precisely so "no camera covered this" cannot be read as "this is dark".
    `age_ms` is the optional length-n per-point colour age, which rides along
    through every filter and reorder below so that a cluster's ages stay lined up
    with its own returns; a mismatched length is dropped rather than carried,
    because an age array off by one point is worse than none at all.
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] == 0 or pts.shape[1] != 2:
        return []

    colours = _as_rgb(rgb, pts.shape[0])
    ages = _as_ages(age_ms, pts.shape[0])
    # Estimated once, over the whole sweep, before anything is split up.
    gains = white_balance_gains(colours, config) if colours is not None else None
    if colours is not None and coloured_mask is not None:
        mask = np.asarray(coloured_mask, dtype=bool)
        if mask.shape[0] == colours.shape[0]:
            colours = colours.copy()
            colours[~mask] = -1

    ranges = np.hypot(pts[:, 0], pts[:, 1])
    keep = (ranges >= config.MIN_OBSTACLE_RANGE_M) & (
        ranges <= config.MAX_OBSTACLE_RANGE_M
    )
    if not keep.any():
        return []
    pts, ranges = pts[keep], ranges[keep]
    if colours is not None:
        colours = colours[keep]
    if ages is not None:
        ages = ages[keep]

    # Bearings, then sort into angular order. `arctan2(starboard, forward)` is
    # the relative-bearing convention used everywhere in this repo: 0 ahead,
    # positive to starboard. See geo.py.
    bearings = np.degrees(np.arctan2(pts[:, 0], pts[:, 1]))
    order = np.argsort(bearings)
    pts, ranges, bearings = pts[order], ranges[order], bearings[order]
    if colours is not None:
        colours = colours[order]
    if ages is not None:
        ages = ages[order]

    # Distance between angular neighbours, and the range-dependent gap that
    # decides whether they belong to the same thing.
    steps = np.hypot(np.diff(pts[:, 0]), np.diff(pts[:, 1]))
    gaps = config.CLUSTER_GAP_M + config.CLUSTER_GAP_PER_M * np.minimum(
        ranges[:-1], ranges[1:]
    )
    cuts = np.flatnonzero(steps > gaps) + 1
    groups = np.split(np.arange(pts.shape[0]), cuts)

    # The sweep is a circle, so the first and last groups are angular neighbours
    # too. An object straddling the +-180 deg seam - which for the aft unit is
    # dead astern, the direction that matters most to it - would otherwise
    # always be reported as two half-objects.
    if len(groups) > 1 and _wraps(pts, ranges, groups[0], groups[-1], config):
        groups = [np.concatenate([groups[-1], groups[0]])] + list(groups[1:-1])

    out = []
    for index in groups:
        if index.size < config.MIN_CLUSTER_POINTS:
            continue
        member = pts[index]
        centre = (float(member[:, 0].mean()), float(member[:, 1].mean()))
        near_i = int(np.argmin(ranges[index]))
        nearest = (float(member[near_i, 0]), float(member[near_i, 1]))
        # Chord across the cluster: the distance between its two extreme
        # returns, which is what a width threshold should actually test.
        width = float(
            math.hypot(
                member[:, 0].max() - member[:, 0].min(),
                member[:, 1].max() - member[:, 1].min(),
            )
        )
        out.append(
            Cluster(
                centre=centre,
                nearest=nearest,
                range_m=float(math.hypot(*centre)),
                bearing_deg=float(math.degrees(math.atan2(centre[0], centre[1]))),
                width_m=width,
                n=int(index.size),
                rgb=(colours[index] if colours is not None else None),
                age_ms=(ages[index] if ages is not None else None),
                source=source,
                gains=gains,
            )
        )
    return out


def _wraps(pts, ranges, first, last, config):
    """Whether the first and last angular groups are really one object."""
    a, b = last[-1], first[0]
    step = math.hypot(pts[a, 0] - pts[b, 0], pts[a, 1] - pts[b, 1])
    gap = config.CLUSTER_GAP_M + config.CLUSTER_GAP_PER_M * min(ranges[a], ranges[b])
    return step <= gap


def _as_rgb(rgb, n):
    """`(n, 3)` int array from whatever shape the colour arrived in, or None.

    The wire format is a flat `r,g,b,r,g,b...` list three times as long as the
    point list (`edge_protocol.py`), so both shapes turn up depending on whether
    the caller reshaped it first.
    """
    if rgb is None:
        return None
    arr = np.asarray(rgb)
    if arr.size == 0:
        return None
    if arr.ndim == 1:
        if arr.size != 3 * n:
            return None
        arr = arr.reshape(-1, 3)
    if arr.ndim != 2 or arr.shape[0] != n or arr.shape[1] != 3:
        return None
    return arr.astype(np.int16)


def _as_ages(age_ms, n):
    """`(n,)` float array of per-point colour ages, or None.

    Length is checked rather than trusted: this array only ever *weights* a vote,
    so a silent misalignment would tilt colours towards whichever returns happen
    to sit at the same index in a shorter array - a bias with no symptom. None is
    the safe answer and costs only the weighting.
    """
    if age_ms is None:
        return None
    arr = np.asarray(age_ms, dtype=np.float64).reshape(-1)
    if arr.size != n:
        return None
    return arr


def split_by_gap(clusters, gap_m, tolerance_m):
    """Pairs of clusters `gap_m` apart, nearest first. The berth/gate finder.

    Used twice and for the same geometric reason each time:

      * **docking** - a berth is a gap of a known width between two structures
        (NJORD §9.3: 2 m for bow-in, 4 m for parallel);
      * **gates** - a Njord gate is a red and a green buoy 5 m apart (§9.2).

    The gap is measured between the two clusters' *nearest* returns rather than
    their centroids, because that is the opening the hull has to fit through -
    a centroid-to-centroid figure quietly includes half of each object and reads
    a 2 m berth as 2.4 m wide.

    Returned nearest-first, since the candidate in front of the boat is the one
    it is being asked about.
    """
    pairs = []
    for i, a in enumerate(clusters):
        for b in clusters[i + 1:]:
            separation = math.hypot(
                a.nearest[0] - b.nearest[0], a.nearest[1] - b.nearest[1]
            )
            if abs(separation - gap_m) <= tolerance_m:
                midpoint = (
                    0.5 * (a.nearest[0] + b.nearest[0]),
                    0.5 * (a.nearest[1] + b.nearest[1]),
                )
                pairs.append((math.hypot(*midpoint), a, b, separation, midpoint))
    pairs.sort(key=lambda item: item[0])
    return [(a, b, separation, midpoint) for _r, a, b, separation, midpoint in pairs]
