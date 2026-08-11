"""The dock's AR tags -> a `ParkingBox`. NJORD §9.3, and the only way in now.

    berth, report = find_berth(tags, mouth_m=2.0, depth_m=2.0, parallel=False,
                               prior_into_deg=leg_bearing_relative)

`tags` is `nodes/io_manager/edge_link.EdgeLink.tags()`: both cameras' markers,
already measured in the rig frame by the Jetson (`artags.py` over there). What
comes back is the same `perception/parking.ParkingBox` the lidar's line fitter
produces, so `behaviours/parking.py` drives to it without knowing which sensor
found it - that is the whole point of doing it this way rather than writing a
second docking behaviour.

Why this file exists
--------------------
**Both lidars are down (2026-08-11).** The aft unit had already failed and the
front one has now followed it, and every way this boat had of finding a berth went
with them: `perception/lines.py` fits edges to lidar returns, `dock.py` looks for a
gap between two lidar clusters. So the docking task has one sensor left - two
fisheye cameras looking at three sheets of A4 - and this is the module that turns
them into geometry.

That is less of a downgrade than it sounds. A tag is *identified*, which no lidar
return ever was: the marker on a berth's closed end says which berth it is, and its
absence says the berth is occupied. The lidar could measure a gap to ±3 cm and
never tell you whose gap it was.

Three tags, and what each one is for
------------------------------------
From the organisers' own files (`ArUco_tags_on_dock.zip`, decoded: DICT_4X4_50,
ids 0-7) and the layout drawing that came with them. Each berth is marked at
**both mouth corners and the middle of its closed end**:

    bow-in, berth 1     sides 0 (right) and 1 (left),   closed end 7
    bow-in, berth 2     sides 1 (right) and 3 (left),   closed end 2
    alongside           sides 4 (right) and 6 (left),   closed end 5

**Id 1 is on both berths**, because it is the shared finger pontoon between them -
berth 1's left wall is berth 2's right wall. So a lone id 1 identifies nothing, and
which berth is in view is decided by the *other* tags.

Left and right are **as the boat sees them coming in**, which is how the drawing is
laid out: left is to port on a bow-first entry. That is not decoration. It is what
fixes the sign of the way in when the closed end is not visible - see
`_axis_from_sides`.

Occupancy, which is the one thing the tags do that nothing else could
--------------------------------------------------------------------
Two bow-in berths are laid out side by side and **one of them is occupied**. The
boat in it hides that berth's closed-end marker, and the free berth's is in plain
view. So:

    a berth whose closed-end tag is visible is FREE
    a berth whose closed-end tag is missing is OCCUPIED -- or its paper fell off

Both halves of that matter. The test is worth having because it is the only
occupancy cue on the boat, and it is not trusted absolutely because a sheet of A4
taped to a pontoon in Trondheim in August is not a reliable object - one had
already come off the far berth in the photographs this was written from. So:

  * exactly one closed-end tag in view -> that berth, and say why;
  * both in view -> the nearer one, and say that both looked free;
  * neither in view, but side tags are -> **do not guess.** The berth geometry is
    reported with `occupancy: "unknown"` and the behaviour still parks, because a
    berth found from two side tags is still a berth; what it must not do is claim
    to have checked. An operator watching the panel can see the difference.

Positions, never one tag's normal
---------------------------------
The Jetson ships a full 6-DOF pose per tag and **the rotation half of it is not
usable**. Measured by round-tripping exact synthetic corners through that module:
positions come back to 0.09 deg of bearing and 3 mm of range, while the face normal
is out by 28 deg at 15 deg of tag tilt and 49 deg at 25 deg, because the mirrored
planar-pose solution fits a small square just as well. Its `ambiguity` figure was
0.01 while that was happening, so it does not catch it either.

Every axis in this file therefore comes from the line between two tag *centres*,
or from the operator's waypoint, and never from `normal_rig`. Two tags 2 m apart,
each placed to 5 cm, fix that line to about 1.5 deg. The normals are read for
exactly one purpose - a sanity flag, `facing_disagrees` - and nothing steers on
them.

What is measured and what is assumed
------------------------------------
The **mouth** is measured: it is the distance between the two side tags, and it is
reported beside the nominal figure because a disagreement is the most useful
warning this module can produce. `rig.json`'s ±75 deg camera yaws are hand-described
and unverified, the two side tags of a 2 m berth land **one in each camera** (the
pair's fields overlap only ~24 deg across the bow), and a yaw error therefore shows
up first as a berth that measures the wrong width. Watch `mouth_m` against
`mouth_nominal_m` on the panel.

One reassurance about that: the boat drives to the **midpoint** of the two side
tags, and a symmetric width error does not move a midpoint. A yaw error has to be
*asymmetric* between the two cameras before it moves the dot.

The **depth** is measured when the closed-end tag is visible and nominal otherwise,
which is the same `depth_source` distinction the lidar version already reports.

And what the tag separation means in metres is itself an assumption worth stating:
the tags sit on walls 0.13 m thick, so centre-to-centre is the clear opening plus
about a wall. `TAG_SPAN_M` is that figure, separately configurable, and the honest
answer is to park once and read `mouth_m` off the panel.
"""

from __future__ import annotations

import math

from . import lines

#: Which tags mark which berth. `(right, left, closed_end)`, using the drawing's
#: left and right - i.e. as the boat sees them entering bow first.
#:
#: Ids from the organisers' files, decoded as DICT_4X4_50. **Not published in the
#: handbook**, so if the tags on the day are different this dict and
#: `ligmax-edge/artags.py`'s dictionary are the two places to change.
BOW_IN_BERTHS = {
    "berth 1": (0, 1, 7),
    "berth 2": (1, 3, 2),
}
PARALLEL_BERTHS = {
    "alongside": (4, 6, 5),
}

#: Centre-to-centre distance between the two side tags, per berth type, metres.
#:
#: NOT the same number as the clear opening the boat has to fit through: the tags
#: are on the walls and the walls are 0.13 m thick, so this is the opening plus
#: about one wall. The team's own figures are 2 m for the bow-in berth and 4.13 m
#: for the alongside one, which is 4.00 + 0.13 and reads as exactly that.
#:
#: Used only to CHECK a measurement, never to place the boat: the dot is the
#: midpoint of the two tags and a symmetric error in this figure does not move a
#: midpoint. Park once, read `mouth_m` off the panel, and set this to what the dock
#: actually is.
TAG_SPAN_M = 2.0
TAG_SPAN_PARALLEL_M = 4.13

#: How far the measured tag separation may differ from `TAG_SPAN_M` before the two
#: tags are refused as a pair. Generous, because it is guarding against picking up
#: two tags from *different* berths (which on this dock are 2 m apart, so the wrong
#: pair reads about 4 m) rather than against calibration error.
SPAN_TOLERANCE_M = 0.9

#: Furthest a tag may be and still be used. Past this an 18 cm marker is under
#: 20 px on the sensor and its range error is growing as z**2; the berth is worked
#: from 3 m in.
MAX_RANGE_M = 12.0

#: Tags seen past this angle off square are dropped. Not because the position is
#: bad - it is not - but because a marker this oblique is a marker whose corners
#: are nearly collinear, and it is usually the *other* berth's tag seen edge-on
#: from outside, which is exactly the tag that must not join this berth's fit.
MAX_INCIDENCE_DEG = 72.0

#: How far two cameras' sightings of one tag id may differ before the pair is
#: distrusted. This is the yaw check in its live form: the overlap across the bow
#: is only ~24 deg, so when both cameras do see one tag, their disagreement is
#: `rig.json`'s error with nothing else in it.
CROSS_CAM_TOLERANCE_M = 0.5


class TagBerth:
    """What `find_berth` decided, beside the box. For the operator's panel.

    Everything here is *reported*. The only field anything steers on is the box.
    """

    __slots__ = ("name", "box", "occupancy", "why", "ids", "sides_seen",
                 "back_seen", "mouth_m", "mouth_nominal_m", "depth_source",
                 "axis_source", "cross_cam_m", "facing_disagrees", "candidates")

    def __init__(self, name, box, occupancy, why, ids, sides_seen, back_seen,
                 mouth_m, mouth_nominal_m, depth_source, axis_source,
                 cross_cam_m=None, facing_disagrees=False, candidates=()):
        self.name = name
        self.box = box
        self.occupancy = occupancy          # "free" / "occupied" / "unknown"
        self.why = why
        self.ids = ids
        self.sides_seen = sides_seen
        self.back_seen = back_seen
        self.mouth_m = mouth_m
        self.mouth_nominal_m = mouth_nominal_m
        self.depth_source = depth_source
        self.axis_source = axis_source      # "two sides" / "end + side" / "waypoint"
        self.cross_cam_m = cross_cam_m
        self.facing_disagrees = facing_disagrees
        self.candidates = tuple(candidates)

    def telemetry(self):
        block = {
            "berth": self.name,
            "occupancy": self.occupancy,
            "why": self.why,
            "tag_ids": list(self.ids),
            "sides_seen": self.sides_seen,
            "end_tag_seen": self.back_seen,
            "mouth_m": None if self.mouth_m is None else round(self.mouth_m, 2),
            "mouth_nominal_m": round(self.mouth_nominal_m, 2),
            "depth_source": self.depth_source,
            "axis_source": self.axis_source,
        }
        if self.cross_cam_m is not None:
            block["cross_cam_m"] = round(self.cross_cam_m, 3)
        if self.facing_disagrees:
            block["facing_disagrees"] = True
        if self.candidates:
            block["candidates"] = list(self.candidates)
        return block

    def __repr__(self):
        return (f"<TagBerth {self.name} {self.occupancy} "
                f"ids={sorted(self.ids)} axis={self.axis_source}>")


# --------------------------------------------------------------------- reading

def _point(tag):
    """A tag's centre in the boat frame, `(starboard, forward)`.

    The rig frame is `+x starboard, +y down, +z forward` and this repo's boat frame
    is `(starboard, forward)`, so the vertical component is simply dropped. That is
    correct rather than lazy: a berth is a shape on the water, and how high up a
    pontoon somebody taped the paper is not part of it.
    """
    pos = tag.get("pos_rig")
    if not pos or len(pos) < 3:
        return None
    return (float(pos[0]), float(pos[2]))


def collapse(tags):
    """One entry per tag id, averaging what both cameras said. `(by_id, spread)`.

    A tag within ~12 deg of the bow is inside both cameras' cones and arrives
    twice. Averaging halves the noise, and the *disagreement* is worth more than
    the average: it is a direct reading of the ±75 deg yaw error in `rig.json`,
    measured by the boat, on the water, with no bench required.

    A pair that disagrees by more than `CROSS_CAM_TOLERANCE_M` is still averaged -
    refusing it would throw away the berth over a calibration fault the operator
    can see and correct - but the spread is returned so it can be said out loud.
    """
    grouped = {}
    for tag in tags or ():
        tag_id = tag.get("id")
        if not isinstance(tag_id, int):
            continue
        point = _point(tag)
        if point is None:
            continue
        if float(tag.get("range_m") or 0.0) > MAX_RANGE_M:
            continue
        if abs(float(tag.get("incidence_deg") or 0.0)) > MAX_INCIDENCE_DEG:
            continue
        grouped.setdefault(tag_id, []).append((tag, point))

    by_id, spread = {}, {}
    for tag_id, seen in grouped.items():
        xs = [p[0] for _, p in seen]
        ys = [p[1] for _, p in seen]
        n = float(len(seen))
        by_id[tag_id] = {
            "point": (sum(xs) / n, sum(ys) / n),
            "n": len(seen),
            "cams": sorted({int(t.get("cam", -1)) for t, _ in seen}),
            "range_m": min(float(t.get("range_m") or 0.0) for t, _ in seen),
            "edge_px": max(float(t.get("edge_px") or 0.0) for t, _ in seen),
            "facing_deg": seen[0][0].get("facing_deg"),
            "incidence_deg": min(float(t.get("incidence_deg") or 0.0)
                                 for t, _ in seen),
        }
        if len(seen) > 1:
            points = [p for _, p in seen]
            spread[tag_id] = max(
                math.dist(points[i], points[j])
                for i in range(len(points)) for j in range(i + 1, len(points))
            )
    return by_id, spread


# ------------------------------------------------------------------- the berth

def find_berth(tags, *, mouth_m, depth_m, parallel=False, prior_into_deg=None,
               tag_span_m=None, prefer=None):
    """Pick a berth out of what the cameras can see. `(TagBerth | None, dict)`.

    `mouth_m` and `depth_m` are the nominal berth from `config.py`, `parallel`
    picks which set of tag ids to look for, and `prior_into_deg` is the operator's
    own idea of which way the berth faces - the bearing of the leg into the docking
    waypoint, **relative to the boat's heading**. That last one is the "and GPS
    points" half of doing this without a lidar: it is what lets a berth be found
    from a single tag, and it costs nothing when better evidence exists.

    `prefer` names a berth to take when both look free, which is how an operator
    overrides the choice from the dashboard.

    The second return value is a report dict even when no berth was found, because
    "which tags can you see, and why is that not a berth" is the question somebody
    will be asking at the time.
    """
    table = PARALLEL_BERTHS if parallel else BOW_IN_BERTHS
    span = tag_span_m if tag_span_m is not None else (
        TAG_SPAN_PARALLEL_M if parallel else TAG_SPAN_M)

    by_id, spread = collapse(tags)
    report = {
        "tags_used": sorted(by_id),
        "tags_seen": len(tags or ()),
        "span_nominal_m": round(span, 2),
    }
    if spread:
        worst = max(spread.values())
        report["cross_cam_m"] = round(worst, 3)
        if worst > CROSS_CAM_TOLERANCE_M:
            report["cross_cam_warning"] = (
                f"the two cameras disagree by {worst:.2f} m about one tag -- "
                f"rig.json's camera yaws are the suspect"
            )
    if not by_id:
        report["why"] = "no tags in view"
        return None, report

    candidates = []
    for name, (right_id, left_id, back_id) in table.items():
        cand = _assemble(name, by_id, right_id, left_id, back_id,
                         mouth_m=mouth_m, depth_m=depth_m, span_m=span,
                         prior_into_deg=prior_into_deg)
        if cand is not None:
            candidates.append(cand)

    report["candidates"] = [
        {"berth": c.name, "occupancy": c.occupancy, "ids": sorted(c.ids),
         "axis_source": c.axis_source,
         "range_m": round(math.hypot(*c.box.centre), 2)}
        for c in candidates
    ]
    if not candidates:
        report["why"] = (
            f"tags {sorted(by_id)} are not a berth: "
            + _why_not(by_id, table, span)
        )
        return None, report

    chosen = _choose(candidates, prefer)
    chosen.candidates = tuple(c.name for c in candidates if c is not chosen)
    report["why"] = chosen.why
    report["berth"] = chosen.name
    report["occupancy"] = chosen.occupancy
    return chosen, report


def _choose(candidates, prefer):
    """Which berth to take. Free ones first, then the nearest."""
    if prefer:
        named = [c for c in candidates if c.name == prefer]
        if named:
            best = named[0]
            best.why = f"{best.name}: the operator asked for it"
            return best

    free = [c for c in candidates if c.occupancy == "free"]
    pool = free or candidates
    pool = sorted(pool, key=lambda c: math.hypot(*c.box.centre))
    best = pool[0]

    if len(free) == 1:
        others = [c.name for c in candidates if c is not best]
        best.why = (
            f"{best.name}: its end tag is in view and "
            + (f"{', '.join(others)}'s is not -- taken" if others
               else "it is the only berth in view")
        )
    elif len(free) > 1:
        best.why = (f"{best.name}: {len(free)} berths look free, taking the "
                    f"nearest at {math.hypot(*best.box.centre):.1f} m")
    else:
        best.why = (
            f"{best.name}: no end tag in view, so nothing here says whether it is "
            f"occupied -- geometry from {best.axis_source}"
        )
    return best


def _why_not(by_id, table, span):
    """A sentence saying what is missing. This is read during a run, so it matters."""
    known = set()
    for ids in table.values():
        known.update(ids)
    seen = set(by_id) & known
    if not seen:
        return (f"none of them belong to this task's berths "
                f"({sorted(known)}) -- wrong dictionary, or the other task's tags")
    # Which berth each visible tag could belong to. Said this way round because the
    # common partial view is two tags from *different* berths, and "one tag each
    # from berth 1 and berth 2" is a much more useful sentence than "berth 1 is
    # missing two tags".
    owners = {}
    for name, ids in table.items():
        for tag_id in ids:
            if tag_id in by_id:
                owners.setdefault(tag_id, []).append(name)
    if len(seen) == 1:
        only = next(iter(seen))
        return (f"tag {only} alone fixes no axis -- it belongs to "
                f"{' and '.join(owners.get(only, ['nothing here']))}, and one side "
                f"tag says where a wall is but not which way the berth faces")
    per_berth = {}
    for tag_id, names in owners.items():
        for name in names:
            per_berth.setdefault(name, set()).add(tag_id)
    spread_out = ", ".join(f"{name} has {sorted(ids)}"
                           for name, ids in sorted(per_berth.items()))
    return (f"{spread_out} -- no berth has a usable pair, or its two side tags "
            f"measured further apart than {span:.2f}±{SPAN_TOLERANCE_M:.2f} m "
            f"(which is how two tags from different berths are refused)")


def _assemble(name, by_id, right_id, left_id, back_id, *, mouth_m, depth_m,
              span_m, prior_into_deg):
    """One berth's tags -> a `TagBerth`, or None if they do not make one."""
    right = by_id.get(right_id)
    left = by_id.get(left_id)
    back = by_id.get(back_id)
    if right is None and left is None and back is None:
        return None

    axis = None
    if right is not None and left is not None:
        axis = _axis_from_sides(right["point"], left["point"], span_m)
    if axis is None and back is not None and (right is not None or left is not None):
        side = right if right is not None else left
        axis = _axis_from_end_and_side(
            back["point"], side["point"], is_right=right is not None,
            span_m=span_m, depth_m=depth_m)
    if axis is None and back is not None and prior_into_deg is not None:
        axis = ("waypoint", lines.unit_of(prior_into_deg), None)
    if axis is None:
        return None

    axis_source, into, measured_span = axis
    width = lines.normal_of(lines.bearing_of(into))   # into turned 90 deg to stbd

    # The mouth's midpoint. With both side tags it is measured; with one side and
    # the closed end it is derived; with only the closed end it is the end tag
    # pushed out by the nominal depth.
    if right is not None and left is not None:
        mouth_mid = lines.scale(lines.add(right["point"], left["point"]), 0.5)
    elif back is not None:
        mouth_mid = lines.add(back["point"], lines.scale(into, -depth_m))
        if right is not None or left is not None:
            side = right if right is not None else left
            # Slide the derived midpoint across so it sits on the side tag's own
            # line: the end tag fixes the centreline and the side tag fixes how far
            # along it the mouth is, which is better than either alone.
            across = lines.dot(lines.subtract(side["point"], mouth_mid), width)
            half = 0.5 * span_m * (1.0 if right is not None else -1.0)
            mouth_mid = lines.add(mouth_mid, lines.scale(width, across - half))
    else:
        return None

    depth_measured = None
    if back is not None:
        depth_measured = abs(lines.dot(lines.subtract(back["point"], mouth_mid),
                                       into))
    # A measured depth is only believed when it is anywhere near the berth the
    # rules describe. A tag read at the wrong range - the failure mode of a marker
    # printed at the wrong size - would otherwise silently deepen the berth and put
    # the dot inside the pontoon.
    if (depth_measured is not None
            and abs(depth_measured - depth_m) <= max(0.5, 0.4 * depth_m)):
        depth_used, depth_source = depth_measured, "measured"
    else:
        depth_used, depth_source = depth_m, "nominal"

    centre = lines.add(mouth_mid, lines.scale(into, 0.5 * depth_used))
    # The box's mouth is the TAG-TO-TAG span, measured where both side tags are
    # visible and `span_m` where they are not -- never the caller's `mouth_m`, which
    # is the clear opening and a different number by about a wall thickness. It has
    # to be the span, because the corners below are drawn at +-mouth/2 and those
    # corners are the tags. Mixing the two would make the berth change width by
    # 13 cm the moment a side tag went out of view, which `parking._agrees` would
    # then have to forgive - and forgiving 13 cm is forgiving a real jump too.
    mouth_used = measured_span if measured_span is not None else span_m

    # The three walls the tags imply, as segments, so the operator's chart draws the
    # same open U the lidar version draws. Marked `artag` rather than `front_lidar`,
    # because they are inferred from three points and not measured along their length
    # -- an operator looking at that plot has to be able to tell.
    half_w = 0.5 * mouth_used
    corner_r = lines.add(mouth_mid, lines.scale(width, half_w))
    corner_l = lines.add(mouth_mid, lines.scale(width, -half_w))
    back_r = lines.add(corner_r, lines.scale(into, depth_used))
    back_l = lines.add(corner_l, lines.scale(into, depth_used))
    back_seg = _segment(back_l, back_r, "artag")
    side_seg_l = _segment(corner_l, back_l, "artag")
    side_seg_r = _segment(corner_r, back_r, "artag")

    from .parking import ParkingBox
    box = ParkingBox(
        centre=centre, into_vector=into, width_vector=width,
        mouth_m=mouth_used, depth_m=depth_used,
        depth_measured_m=depth_measured, depth_source=depth_source,
        # Mouth, closed end, closed end, mouth -- the order `ParkingBox` documents,
        # drawn as an open U with the way in left open.
        corners=[corner_l, back_l, back_r, corner_r],
        back=back_seg, sides=[side_seg_l, side_seg_r],
        # Nothing here measures a corner gap: the tags ARE the corners. Reported as
        # 0.0 rather than None so the panel's field keeps its type.
        corner_gap_m=0.0,
        score=0.0,
    )

    ids = {i for i, t in ((right_id, right), (left_id, left), (back_id, back))
           if t is not None}
    sides_seen = sum(1 for t in (right, left) if t is not None)
    occupancy = "free" if back is not None else "unknown"

    # A weak cross-check, and weak on purpose: the tag normals are unreliable (see
    # the module docstring), so a disagreement is worth a flag on the panel and
    # nothing more. If the end tag claims to face somewhere other than back down
    # the way in, either it is the mirrored pose solution or the tags are not the
    # berth we think they are.
    facing_disagrees = False
    if back is not None and back.get("facing_deg") is not None:
        want = lines.bearing_of(lines.scale(into, -1.0))
        off = abs(_angle_diff(float(back["facing_deg"]), want))
        facing_disagrees = off > 50.0

    return TagBerth(
        name=name, box=box, occupancy=occupancy, why="", ids=ids,
        sides_seen=sides_seen, back_seen=back is not None,
        mouth_m=measured_span, mouth_nominal_m=span_m,
        depth_source=depth_source, axis_source=axis_source,
        facing_disagrees=facing_disagrees,
    )


def _axis_from_sides(right_pt, left_pt, span_m):
    """The way in, from the two mouth corners. `(source, into, measured)` or None.

    **This is where knowing left from right earns its keep.** Two points give a line
    across the mouth and two possible normals to it, and the boat has to travel down
    the right one - the other is straight back out to sea. With no closed-end tag
    there is nothing geometric to break the tie.

    The tag *ids* break it. The drawing's left and right are as the boat sees them
    coming in, so the way in is the direction that puts the right-hand tag on the
    boat's starboard: `into` is `left -> right` turned 90 degrees to port.
    """
    across = lines.subtract(right_pt, left_pt)
    measured = math.hypot(*across)
    if measured < 1e-6:
        return None
    if abs(measured - span_m) > SPAN_TOLERANCE_M:
        # Two tags from different berths, most likely: on this dock the berths are
        # 2 m wide, so the wrong pairing reads about double.
        return None
    unit = lines.scale(across, 1.0 / measured)
    # A bearing turned 90 deg to PORT. `lines.normal_of` turns to starboard, so this
    # is the other one -- and it is the whole sign convention, so it is spelled out
    # rather than folded into a helper.
    into = lines.normal_of(lines.bearing_of(unit) + 180.0)
    return "two sides", into, measured


def _axis_from_end_and_side(back_pt, side_pt, *, is_right, span_m, depth_m):
    """The way in, from the closed end and ONE mouth corner.

    The two points are a *diagonal* of the berth rather than an edge, so this needs
    the nominal shape to interpret: from the middle of the closed end, a mouth
    corner lies `depth_m` back along the way in and `span_m / 2` across it. Which
    way across is what the tag id says, and it is the reason this case can be solved
    at all rather than being two mirror-image answers.
    """
    vec = lines.subtract(side_pt, back_pt)
    if math.hypot(*vec) < 1e-6:
        return None
    # How far off "straight back out of the berth" a mouth corner sits, seen from
    # the middle of the closed end. The right-hand corner is that much CLOCKWISE of
    # the way out, so recovering the way out means adding it back for the right tag
    # and subtracting it for the left one. Signs verified numerically both ways --
    # getting this backwards yields a berth rotated by twice this angle, which for a
    # 2 m berth is 53 degrees and a collision.
    corner_off = math.degrees(math.atan2(0.5 * span_m, max(depth_m, 1e-6)))
    out_bearing = lines.bearing_of(vec) + (corner_off if is_right else -corner_off)
    # `out` points from the closed end towards the mouth; the way IN is its reverse.
    return "end + side", lines.unit_of(out_bearing + 180.0), None


def _segment(a, b, source):
    """A `lines.Segment` between two points, for drawing only."""
    length = math.dist(a, b)
    axis = lines.bearing_of(lines.subtract(b, a)) if length > 1e-9 else 0.0
    return lines.Segment(a=a, b=b, axis_deg=axis, length_m=length,
                         # Not fitted to anything, so there is no residual to
                         # report and no return count. Zeroes rather than invented
                         # figures; `source` is what says why.
                         rms_m=0.0, n=0, source=source)


def _angle_diff(a, b):
    """`a - b` folded onto [-180, 180)."""
    return (a - b + 180.0) % 360.0 - 180.0
