"""Three lines -> a parking space, and the dot in the middle of it.

    box = find_box(segments, mouth_m=2.0, depth_m=2.0, ...)
    dot = box.point_at_depth(offset_m)        # the thing the boat drives to

Input is `lines.Segment` in the BOAT frame; output is one `ParkingBox`, also in
the boat frame. Both parking behaviours use this and nothing else uses it.

What the parking space is
-------------------------
**Three sides of a rectangle whose corners do not meet.** The fourth side is the
water, and that is the way in. Two of the three lines are a *pair*: parallel to
each other, one either side of the mouth, running from the mouth to the closed
end. The third is *alone*: perpendicular to the pair, joining them across the
closed end. It is the lone line that the depth offset is measured from, because
it is the only one whose distance means "how deep into the space am I".

    normal parking                     parallel parking
    (mouth 2 m, depth 2 m)             (mouth 4 m, depth 2 m)

      |             |                    |               |
      |      .      |   <- the dot       |       .       |   <- the dot
      |             |                    |               |
      +-----------  +                    +-------------  +
        the lone line                      the lone line

Same structure in both, which is why one finder serves both: only the two
dimensions differ, and they arrive as arguments.

Corners that do not meet are the point, not a tolerance
-------------------------------------------------------
Nothing here asks the lines to touch. A corner gap is what makes each side arrive
as its own run out of `lines.py` in the first place - it is the feature that
makes the three lines separable, and requiring closed corners would reject every
real one of these. `corner_gap_m` is reported for the operator and used for
nothing.

Why lines and not clusters, and why no buoys anywhere in this file
------------------------------------------------------------------
`cluster.py` + `classify.py` answer "what kind of thing is that", and the answer
is worth having on a buoy course. Here it is worth nothing and costs something: a
buoy that drifts into the mouth of the berth is not a reason to refuse to park,
and a berth wall classified as `LAND` is a berth wall either way. So this file
sees geometry and only geometry - no colours, no track identities, no obstacle
kinds, and no way for any of them to hold the boat out of the space it has been
told to park in.

The measured dimension versus the nominal one
---------------------------------------------
The mouth is measured, because the gap between two lines is exactly what the
lidar is good at (+-3 cm on a C1) and because the handbook's figure is nominal
while the thing in the water is what the hull has to fit into.

The depth is measured **when the measurement is credible** and taken from the
argument when it is not, and `depth_source` says which. A lidar sitting inside a
2 m box sees the near end of each side wall and often not the far end, so a
measured depth can be short by half a metre for no reason worse than geometry -
and half a metre of error on a 2 m box moves the dot a quarter of the way to a
wall. The nominal figure is the safer answer when the two disagree, and saying
which one was used is what lets the operator tell "the box is smaller than we
thought" from "the lidar only saw half of it".
"""

import math

from . import lines


class ParkingBox:
    """A parking space measured on one tick. BOAT frame, metres.

    Attributes:
        centre          `(starboard, forward)` of the geometric middle
        into_deg        relative bearing pointing from the mouth into the space,
                        i.e. from the water towards the lone line. This is the
                        heading a bow-in approach holds
        mouth_m         measured gap between the two paired lines
        depth_m         the depth actually used for the geometry
        depth_measured_m  what the lidar saw, whether or not it was used
        depth_source    "measured" or "nominal" - which of the two `depth_m` is
        corners         four `(starboard, forward)` points: mouth, closed end,
                        closed end, mouth. Drawn in that order it is an open U
                        with the way in left open, which is the honest picture
        back            the lone `Segment`
        sides           the two paired `Segment`s, low side of the mouth first
        corner_gap_m    the widest gap between the lone line and a side. Reported
                        for the operator; nothing depends on it
        score           how well this candidate fits. Lower is better
    """

    __slots__ = ("centre", "into_deg", "mouth_m", "depth_m", "depth_measured_m",
                 "depth_source", "corners", "back", "sides", "corner_gap_m",
                 "score", "_into", "_width")

    def __init__(self, centre, into_vector, width_vector, mouth_m, depth_m,
                 depth_measured_m, depth_source, corners, back, sides,
                 corner_gap_m, score):
        self.centre = centre
        self._into = into_vector      # unit, from the mouth towards the lone line
        self._width = width_vector    # unit, across the mouth
        self.into_deg = lines.bearing_of(into_vector)
        self.mouth_m = mouth_m
        self.depth_m = depth_m
        self.depth_measured_m = depth_measured_m
        self.depth_source = depth_source
        self.corners = corners
        self.back = back
        self.sides = sides
        self.corner_gap_m = corner_gap_m
        self.score = score

    # -- the dot -----------------------------------------------------------

    def point_at_depth(self, offset_m):
        """The centre, shifted `offset_m` deeper into the space.

        Positive is **towards the lone line** - deeper in - because that is the
        direction the operator means by "how far into the berth do we sit". The
        offset is the caller's, per parking type, and this does not clamp it:
        `behaviours/parking.py` clamps against `depth_m` and says so on the
        panel, which is where a number the operator typed belongs.
        """
        return lines.add(self.centre, lines.scale(self._into, offset_m))

    def depth_of(self, point):
        """How deep into the space `point` sits, metres from the lone line.

        0 is on the lone line, `depth_m` is at the mouth. This is the readback
        that makes the static offset tunable from what the operator sees rather
        than from a guess: park once, read this, adjust the offset by the
        difference.
        """
        along = lines.dot(lines.subtract(point, self.centre), self._into)
        return self.depth_m * 0.5 - along

    def __repr__(self):
        return (
            f"<ParkingBox {self.mouth_m:.2f}x{self.depth_m:.2f}m "
            f"centre=({self.centre[0]:.2f},{self.centre[1]:.2f}) "
            f"into={self.into_deg:.0f}deg {self.depth_source} score={self.score:.2f}>"
        )


def find_box(segments, *, mouth_m, depth_m, tolerance_m, angle_deg,
             span_fraction, min_line_m, max_range_m):
    """The best parking space in `segments`, or None.

    Every unordered pair of segments is tried as the sides and every remaining
    segment as the lone line, which is O(n^3) in the worst case and irrelevant in
    practice: `lines.py` returns a handful of segments from a sweep, not hundreds,
    and the pair test rejects almost all of the pairs on the first comparison.

    `span_fraction` is how much of a nominal dimension a line has to cover to
    count. It is well below 1 on purpose - a lidar inside a 2 m box sees part of
    each wall, and a finder that insisted on whole walls would find nothing from
    the one position where finding something matters most.
    """
    best = None
    count = len(segments)
    for i in range(count):
        for j in range(i + 1, count):
            pair = _pair(segments[i], segments[j], mouth_m=mouth_m, depth_m=depth_m,
                         tolerance_m=tolerance_m, angle_deg=angle_deg,
                         span_fraction=span_fraction, min_line_m=min_line_m)
            if pair is None:
                continue
            for k in range(count):
                if k in (i, j):
                    continue
                candidate = _assemble(
                    pair, segments[k], segments,
                    mouth_m=mouth_m, depth_m=depth_m, tolerance_m=tolerance_m,
                    angle_deg=angle_deg, span_fraction=span_fraction,
                    min_line_m=min_line_m, max_range_m=max_range_m,
                    skip=(i, j, k),
                )
                if candidate is None:
                    continue
                if best is None or candidate.score < best.score:
                    best = candidate
    return best


# ------------------------------------------------------------------ the pair

class _Pair:
    """Two candidate side lines and the frame they define."""

    __slots__ = ("low", "high", "axis_deg", "depth_dir", "width_dir",
                 "low_w", "high_w", "separation", "overlap")

    def __init__(self, low, high, axis_deg, depth_dir, width_dir, low_w, high_w,
                 separation, overlap):
        self.low = low
        self.high = high
        self.axis_deg = axis_deg
        self.depth_dir = depth_dir   # unit, along the sides: mouth <-> lone line
        self.width_dir = width_dir   # unit, across the mouth: low side -> high side
        self.low_w = low_w
        self.high_w = high_w
        self.separation = separation
        self.overlap = overlap


def _pair(first, second, *, mouth_m, depth_m, tolerance_m, angle_deg,
          span_fraction, min_line_m):
    """Two segments as the sides of a space, or None.

    Three tests, in the order that rejects fastest:

      * **parallel** to each other, within `angle_deg`;
      * **long enough** to be a side wall rather than a plank end-on;
      * **the right distance apart**, which is the mouth width - and measured
        perpendicular to their common axis, not between their midpoints, because
        two walls the boat is looking down at an angle have midpoints much
        further apart than the gap the hull has to fit through.

    Then they must **overlap** along that axis. Two walls of two different berths
    twenty metres apart are parallel and correctly separated and are not a berth;
    what makes a pair the two sides of one space is that they are alongside each
    other.
    """
    if lines.axis_diff(first.axis_deg, second.axis_deg) > angle_deg:
        return None
    needed = max(min_line_m, span_fraction * depth_m)
    if min(first.length_m, second.length_m) < needed:
        return None

    axis = lines.mean_axis([first, second])
    depth_dir = lines.unit_of(axis)
    width_dir = lines.normal_of(axis)

    first_w = lines.dot(first.midpoint, width_dir)
    second_w = lines.dot(second.midpoint, width_dir)
    separation = abs(second_w - first_w)
    if abs(separation - mouth_m) > tolerance_m:
        return None

    low, high = (first, second) if first_w <= second_w else (second, first)
    low_w, high_w = min(first_w, second_w), max(first_w, second_w)

    low_lo, low_hi = _span(low, depth_dir)
    high_lo, high_hi = _span(high, depth_dir)
    overlap = min(low_hi, high_hi) - max(low_lo, high_lo)
    if overlap < span_fraction * depth_m:
        return None

    return _Pair(low, high, axis, depth_dir, width_dir, low_w, high_w,
                 separation, overlap)


def _span(segment, direction):
    """`(low, high)` extent of a segment along `direction`, absolute coordinates."""
    a = lines.dot(segment.a, direction)
    b = lines.dot(segment.b, direction)
    return (a, b) if a <= b else (b, a)


# --------------------------------------------------------------- the assembly

def _assemble(pair, back, segments, *, mouth_m, depth_m, tolerance_m, angle_deg,
              span_fraction, min_line_m, max_range_m, skip):
    """A pair plus a lone line -> a `ParkingBox`, or None."""
    # Perpendicular to the sides. `axis_diff` returns 0-90, so perpendicular is
    # 90 and the slack comes off that end.
    if lines.axis_diff(back.axis_deg, pair.axis_deg) < 90.0 - angle_deg:
        return None
    if back.length_m < max(min_line_m, span_fraction * mouth_m):
        return None

    # The lone line has to cross the gap rather than sit beside it.
    back_lo, back_hi = _span(back, pair.width_dir)
    covered = min(back_hi, pair.high_w) - max(back_lo, pair.low_w)
    if covered < span_fraction * pair.separation:
        return None
    # A dock face may run on past the berth, so overhang is allowed - it is only
    # scored, and the score is what breaks the tie when two assemblies fit.
    overhang = max(0.0, pair.low_w - back_lo) + max(0.0, back_hi - pair.high_w)

    # Which end of the sides is closed. The lone line is at one of them, and the
    # space runs from the other one - the mouth - towards it.
    low_lo, low_hi = _span(pair.low, pair.depth_dir)
    high_lo, high_hi = _span(pair.high, pair.depth_dir)
    sides_lo, sides_hi = min(low_lo, high_lo), max(low_hi, high_hi)
    back_d = 0.5 * (
        lines.dot(back.a, pair.depth_dir) + lines.dot(back.b, pair.depth_dir)
    )
    if abs(back_d - sides_hi) <= abs(back_d - sides_lo):
        into_sign = 1.0            # deeper in means larger depth coordinate
        mouth_d = sides_lo
    else:
        into_sign = -1.0
        mouth_d = sides_hi
    depth_measured = abs(back_d - mouth_d)

    # A lone line that is nowhere near either end of the sides is a line across
    # the middle of something, not the closed end of a space.
    if min(abs(back_d - sides_hi), abs(back_d - sides_lo)) > tolerance_m + depth_m * 0.5:
        return None

    if abs(depth_measured - depth_m) <= tolerance_m:
        depth_used, depth_source = depth_measured, "measured"
    else:
        depth_used, depth_source = depth_m, "nominal"

    into_dir = lines.scale(pair.depth_dir, into_sign)

    # The middle. Across the mouth it is halfway between the two side lines,
    # which is the well-determined half. Along the space it is half the depth out
    # from the lone line, because that is the side the offset is measured from.
    centre_w = 0.5 * (pair.low_w + pair.high_w)
    centre_d = back_d - into_sign * depth_used * 0.5
    centre = lines.add(
        lines.scale(pair.width_dir, centre_w), lines.scale(pair.depth_dir, centre_d)
    )
    if math.hypot(*centre) > max_range_m:
        return None

    mouth_edge = back_d - into_sign * depth_used
    corners = [
        _corner(pair, pair.low_w, mouth_edge),
        _corner(pair, pair.low_w, back_d),
        _corner(pair, pair.high_w, back_d),
        _corner(pair, pair.high_w, mouth_edge),
    ]

    if not _mouth_is_open(
        segments, pair, mouth_edge, skip,
        tolerance_m=tolerance_m, angle_deg=angle_deg, span_fraction=span_fraction,
    ):
        return None

    corner_gap = max(
        _gap_between(back, pair.low), _gap_between(back, pair.high)
    )

    score = (
        abs(pair.separation - mouth_m) / max(tolerance_m, 0.05)
        + abs(depth_measured - depth_m) / max(tolerance_m, 0.05)
        + lines.axis_diff(pair.low.axis_deg, pair.high.axis_deg) / max(angle_deg, 1.0)
        + (90.0 - lines.axis_diff(back.axis_deg, pair.axis_deg)) / max(angle_deg, 1.0)
        + overhang * 0.5
        + math.hypot(*centre) / 20.0
    )

    return ParkingBox(
        centre=centre,
        into_vector=into_dir,
        width_vector=pair.width_dir,
        mouth_m=pair.separation,
        depth_m=depth_used,
        depth_measured_m=depth_measured,
        depth_source=depth_source,
        corners=corners,
        back=back,
        sides=(pair.low, pair.high),
        corner_gap_m=corner_gap,
        score=score,
    )


def _corner(pair, width_coord, depth_coord):
    return lines.add(
        lines.scale(pair.width_dir, width_coord),
        lines.scale(pair.depth_dir, depth_coord),
    )


def _mouth_is_open(segments, pair, mouth_edge, skip, *, tolerance_m, angle_deg,
                   span_fraction):
    """Whether the fourth side really is water.

    With exactly three lines in view this can only answer yes, and it earns its
    place the moment there is a fourth: a closed rectangle offers two assemblies,
    one with each end taken as the closed end, and the wrong one sends the boat
    at a wall. Refusing the assembly whose mouth is barred leaves the right one
    standing, or nothing - which is the correct answer for a box with no way in.
    """
    for index, segment in enumerate(segments):
        if index in skip:
            continue
        if lines.axis_diff(segment.axis_deg, pair.axis_deg) < 90.0 - angle_deg:
            continue
        centre_d = 0.5 * (
            lines.dot(segment.a, pair.depth_dir) + lines.dot(segment.b, pair.depth_dir)
        )
        if abs(centre_d - mouth_edge) > tolerance_m:
            continue
        lo, hi = _span(segment, pair.width_dir)
        covered = min(hi, pair.high_w) - max(lo, pair.low_w)
        if covered >= span_fraction * pair.separation:
            return False
    return True


def _gap_between(first, second):
    """Shortest distance between two segments' endpoints. The corner gap."""
    return min(
        math.hypot(a[0] - b[0], a[1] - b[1])
        for a in (first.a, first.b)
        for b in (second.a, second.b)
    )
