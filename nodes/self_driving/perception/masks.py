"""Throw away the returns that are the boat looking at itself.

The aft C1 is bolted 0.5 m astern of the datum facing backwards, so everything
it reports in the *forward* direction is the vessel: the hull, the mast, the
battery slider rail, whichever ama is nearest. Those returns are real - the
sensor is not wrong - but they are not obstacles, and left in they do three bad
things:

  * they cluster into a permanent 1-2 m "structure" a metre from the boat, which
    `emergency_stop_needed()` reads as something dead ahead;
  * they are the closest returns in every sweep, so a berth-gap search finds the
    gap between two pieces of the boat first;
  * they never move relative to the hull, so the tracker gives them enormous
    confidence and they outlive every real detection.

Two shapes, because they fail differently
-----------------------------------------
    box     a corridor `AFT_MASK_HALF_WIDTH_M` either side of the centreline,
            from the sensor forwards. This is the shape of the actual
            obstruction, so it removes the hull and keeps a buoy that happens to
            be broad on the bow at 8 m. **The default.**
    sector  everything within `AFT_MASK_SECTOR_DEG` of dead ahead, at any range.
            Blunter, and worth having because it needs no measurement of the
            boat: if the box is letting something through, this will not.
    both    the union.

The front unit is filtered on the Jetson (`ligmax-edge`), where the rig geometry
lives, so its mask is off by default here - two places correcting one occlusion
is how a rig ends up corrected twice.

Everything is measured in the BOAT frame, `[starboard, forward]` metres, the
frame `scan.py` delivers - so the numbers below are the ones you get off the
hull with a tape measure, not sensor-frame angles that have to be worked out.
"""

import math
import os

import numpy as np


def _f(name, default):
    try:
        return float(os.environ[name])
    except (KeyError, ValueError):
        return default


def _s(name, default):
    return (os.environ.get(name, "").strip() or default).lower()


# "box", "sector", "both", or "none".
AFT_MASK_MODE = _s("LIGMAX_AFT_MASK_MODE", "box")

# The corridor. Half-width either side of the centreline, and where it starts -
# the aft unit sits 0.5 m astern of the datum, so anything forward of that is
# looking up the length of the boat.
AFT_MASK_HALF_WIDTH_M = _f("LIGMAX_AFT_MASK_HALF_WIDTH_M", 0.5)
AFT_MASK_FROM_M = _f("LIGMAX_AFT_MASK_FROM_M", -0.6)

# The sector, for "sector"/"both". Half-angle either side of dead ahead.
AFT_MASK_SECTOR_DEG = _f("LIGMAX_AFT_MASK_SECTOR_DEG", 60.0)

# The front unit is masked on the Jetson. Off here unless somebody says
# otherwise, in which case the same three knobs apply.
FRONT_MASK_MODE = _s("LIGMAX_FRONT_MASK_MODE", "none")
FRONT_MASK_HALF_WIDTH_M = _f("LIGMAX_FRONT_MASK_HALF_WIDTH_M", 0.5)
FRONT_MASK_FROM_M = _f("LIGMAX_FRONT_MASK_FROM_M", 0.6)
FRONT_MASK_SECTOR_DEG = _f("LIGMAX_FRONT_MASK_SECTOR_DEG", 60.0)


def keep_mask(points, mode, half_width_m, from_m, sector_deg, behind=False):
    """Boolean array: True for the returns to KEEP. Never raises.

    `behind` flips the corridor to point astern instead of ahead, so the same
    function masks a forward-facing unit's view of the stern.
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] == 0 or pts.shape[1] != 2:
        return np.zeros(0, dtype=bool)

    keep = np.ones(pts.shape[0], dtype=bool)
    if mode in ("none", "off", ""):
        return keep

    starboard, forward = pts[:, 0], pts[:, 1]

    if mode in ("box", "both"):
        in_corridor = np.abs(starboard) <= half_width_m
        beyond = forward <= from_m if behind else forward >= from_m
        keep &= ~(in_corridor & beyond)

    if mode in ("sector", "both"):
        # Relative bearing: 0 is dead ahead, +-180 is astern. Same convention as
        # geo.relative_bearing and cluster.py.
        bearing = np.degrees(np.arctan2(starboard, forward))
        if behind:
            blocked = np.abs(np.abs(bearing) - 180.0) <= sector_deg
        else:
            blocked = np.abs(bearing) <= sector_deg
        keep &= ~blocked

    return keep


def mask_aft(points):
    """Keep-mask for the aft unit: drop its view up the length of the boat."""
    return keep_mask(
        points,
        AFT_MASK_MODE,
        AFT_MASK_HALF_WIDTH_M,
        AFT_MASK_FROM_M,
        AFT_MASK_SECTOR_DEG,
        behind=False,
    )


def mask_front(points):
    """Keep-mask for the front unit. Off by default - masked on the Jetson."""
    return keep_mask(
        points,
        FRONT_MASK_MODE,
        FRONT_MASK_HALF_WIDTH_M,
        FRONT_MASK_FROM_M,
        FRONT_MASK_SECTOR_DEG,
        behind=True,
    )


def apply(points, rgb, keep):
    """`(points, rgb)` with the masked returns removed. `rgb` may be None.

    The two are filtered together and by the same mask, which is the only thing
    that matters here: `rgb` is a flat `r,g,b,...` array three times as long as
    the point list (`edge_protocol.py`), so dropping a point without dropping
    its triple shifts every colour after it by one and silently recolours the
    entire rest of the sweep.
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[0] == 0:
        return pts, rgb
    if keep is None or keep.shape[0] != pts.shape[0]:
        return pts, rgb
    if keep.all():
        return pts, rgb

    out_points = pts[keep]
    if rgb is None:
        return out_points, None

    colours = np.asarray(rgb)
    if colours.ndim == 1:
        if colours.size != 3 * pts.shape[0]:
            return out_points, None
        colours = colours.reshape(-1, 3)
    if colours.shape[0] != pts.shape[0]:
        return out_points, None
    return out_points, colours[keep]


def describe():
    """What the masks are set to, for telemetry and the trip header."""
    return {
        "aft_mode": AFT_MASK_MODE,
        "aft_half_width_m": AFT_MASK_HALF_WIDTH_M,
        "aft_from_m": AFT_MASK_FROM_M,
        "aft_sector_deg": AFT_MASK_SECTOR_DEG,
        "front_mode": FRONT_MASK_MODE,
        "note": "the front unit is masked on the Jetson (ligmax-edge)",
    }
