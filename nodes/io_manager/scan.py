"""Both lidars, in one frame, as the `scans` the dashboard plots.

    scans = ScanPublisher(edge_link, aft_reader)
    frame_fields = scans.publish_fields()      # {"scans": [...]} or {"scans": []}

Two sensors, one boat
---------------------
The vessel carries two RPLidar C1s and they arrive here by completely different
routes:

    front   on the Jetson. Fused with the two cameras there (`ligmax-edge`),
            so its returns arrive **already coloured** where a lens covered
            them, in the RIG frame - origin at the front lidar itself, +x
            starboard, +y down, +z forward. It reaches us over TCP 3401
            (`edge_link.py`).
    aft     on this Pi, on its own USB serial port (`lidar.py`). No cameras
            look aft, so its returns have no colour and never will. It is
            mounted facing astern.

Both are put into the BOAT frame - same axis convention, origin at the boat's
datum - and shipped as two entries in one `scans` list, so the operator gets one
plot of everything the boat can see rather than two charts to reconcile.

GEOMETRY. Hand-measured, exactly like `ligmax-edge/rig.json`, and the numbers to
edit when something is re-bolted:

    front lidar   0.50 m FORWARD of the datum, axis-aligned
    aft lidar     0.50 m AFT of the datum, yawed 180 deg (facing astern)

The front cloud needs a translation and nothing else, because `rig.json` has
already rotated that unit onto the boat's axes (its own 0 mark is bolted 45 deg
to port, and `rig.json`'s `yaw_deg: -45` takes that out). If the front returns
come out rotated, the fix belongs in `rig.json` on the Jetson, not here - there
is deliberately no second yaw knob for the front, because two places to correct
one rotation is how a rig ends up corrected twice.

The aft unit's own convention IS set here, since nothing else owns it:

    a = LIGMAX_AFT_LIDAR_ANGLE_DIR * (reported_deg - LIGMAX_AFT_LIDAR_ANGLE_ZERO_DEG)
    p_lidar = (d*sin(a), 0, d*cos(a))          # a = 0 is the sensor's own forward

then yawed by LIGMAX_AFT_LIDAR_YAW_DEG and translated aft. **Verify the sign on
hardware before trusting it** - the failure mode of a flipped `angle_dir` is a
plausible but mirrored world astern, which is the kind of wrong that survives a
casual glance. `docs/testing.md` is the check.

Every one of these is an environment override with a hand-measured default, so
the boat can be corrected without a commit and the commit records what was
measured. None of them has been checked against hardware yet.

WHAT GOES ON THE WIRE. Points are `[starboard, forward]` metres in the boat
frame, tagged `frame: "boat"`, and the dashboard rotates them onto the chart
with the vessel's own position and heading. Sending boat-relative rather than
grid metres is deliberate: what the sensor measured is a range and a bearing
from the hull, and converting to grid here would bake this second's heading into
a cloud the map redraws at 60 Hz - the points would swing a frame behind the
boat on every turn. It also means the plot still works before the grid has an
origin, which is most of a bench session.

The front scan carries a flat `rgb` array, 3 entries per point, in the same
order as `points`. A point no camera could colour is `-1, -1, -1` rather than
black: most of a rotation is behind both lenses, so "uncoloured" is the normal
case and must not be confusable with a genuinely dark object. The aft scan has
no `rgb` at all.

STALENESS. A sweep older than MAX_SWEEP_AGE_S is dropped rather than
republished. A point cloud that stopped updating but stays on the chart looks
exactly like a sea that stopped changing, and this is the same rule
`navigation.py` follows when it sends `boat: null` rather than leave the vessel
drawn where it was thirty seconds ago.

Both ages are measured on THIS machine's clock: the aft sweep from its own
`t_end`, stamped here, and the front sweep from the instant it landed on 3401.
The front cloud carries the Jetson's `t_start`/`t_end` too, in the same epoch
seconds, and they are not used for this on purpose - see `_front()`. Clock
disagreement between the two machines is reported as `clock_offset_s` and
never allowed to decide whether a sensor exists.
"""

import logging
import math
import os
import time

import numpy as np

log = logging.getLogger("io_manager.scan")


def _env_float(name, default):
    try:
        return float(os.environ[name])
    except (KeyError, ValueError):
        return default


# Where each unit sits along the hull, metres from the datum, positive forward.
# HAND-MEASURED, not fitted. See the module docstring.
FRONT_FORWARD_M = _env_float("LIGMAX_FRONT_LIDAR_FORWARD_M", 0.5)
AFT_FORWARD_M = _env_float("LIGMAX_AFT_LIDAR_FORWARD_M", -0.5)

# The aft unit faces astern. 180 deg about +y (down); positive yaw swings to
# starboard, the same convention as `ligmax-edge/rig.json` and `fusion.py`.
AFT_YAW_DEG = _env_float("LIGMAX_AFT_LIDAR_YAW_DEG", 180.0)

# How the aft C1's reported heading becomes a direction in its own frame.
# +1 is the C1's clockwise-from-above convention; -1 if this unit or its
# mounting runs the other way.
AFT_ANGLE_DIR = 1.0 if _env_float("LIGMAX_AFT_LIDAR_ANGLE_DIR", 1.0) >= 0 else -1.0
AFT_ANGLE_ZERO_DEG = _env_float("LIGMAX_AFT_LIDAR_ANGLE_ZERO_DEG", 0.0)

# Older than this and a sweep is not published at all. Both units settle at
# 10 Hz, so two seconds is two hundred rotations of grace - long enough that a
# hiccup does not blink the plot, short enough that a dead sensor stops drawing.
MAX_SWEEP_AGE_S = _env_float("LIGMAX_LIDAR_MAX_AGE_S", 2.0)

# Centimetre resolution on the wire. The C1 is +-3 cm, so a third decimal would
# be transmitting noise about 400 points at a time.
_DECIMALS = 2

# What an uncoloured return carries in `rgb`. Not black: most of a rotation is
# behind both lenses, and "no camera saw this" must not look like "this is dark".
NO_COLOUR = -1


def _to_boat(x, z, forward_m, yaw_deg):
    """Lidar-frame (x starboard, z forward) -> boat frame, as (starboard, forward).

    Yaw is about +y (down), positive to starboard - the convention `fusion.py`
    and `rig.json` already use, kept identical so a pose can be moved between
    them without being reinterpreted.
    """
    yaw = math.radians(yaw_deg)
    cy, sy = math.cos(yaw), math.sin(yaw)
    stbd = cy * x + sy * z
    fwd = -sy * x + cy * z + forward_m
    return stbd, fwd


def _round(values):
    return np.round(values, _DECIMALS).tolist()


def front_scan(cloud):
    """The Jetson's sweep -> one `scans` entry, colour included. None if unusable.

    `cloud` is the columnar payload straight off the wire (`edge_protocol.py`):
    parallel `x`/`y`/`z` arrays in the rig frame, a flat `rgb` three times as
    long, and `cam` saying which camera coloured each point, or -1 for none.
    """
    if not cloud:
        return None
    xs = np.asarray(cloud.get("x") or (), dtype=np.float64)
    zs = np.asarray(cloud.get("z") or (), dtype=np.float64)
    if xs.size == 0 or xs.size != zs.size:
        return None

    # The rig frame's origin IS the front lidar, and rig.json has already put
    # its axes on the boat's, so this is a translation and nothing more.
    stbd, fwd = _to_boat(xs, zs, FRONT_FORWARD_M, 0.0)
    scan = {
        "source": "front_lidar",
        "frame": "boat",
        "points": [[s, f] for s, f in zip(_round(stbd), _round(fwd))],
    }

    # Colour, where a camera actually saw the point. `cam` is the authority on
    # that, not the RGB triple: the Jetson writes (0, 0, 0) for uncoloured, and
    # a genuinely black buoy would be indistinguishable from it.
    cams = np.asarray(cloud.get("cam") or (), dtype=np.int64)
    rgb = np.asarray(cloud.get("rgb") or (), dtype=np.int64)
    if cams.size == xs.size and rgb.size == 3 * xs.size:
        rgb = rgb.reshape(-1, 3).copy()
        rgb[cams < 0] = NO_COLOUR
        scan["rgb"] = rgb.reshape(-1).tolist()
        scan["coloured"] = int((cams >= 0).sum())
    return scan


def aft_scan(sweep):
    """An aft `Sweep` -> one `scans` entry. No colour: nothing looks astern."""
    if sweep is None or len(sweep) == 0:
        return None
    a = np.radians(AFT_ANGLE_DIR * (sweep.angle_deg - AFT_ANGLE_ZERO_DEG))
    d = sweep.dist_m
    stbd, fwd = _to_boat(d * np.sin(a), d * np.cos(a), AFT_FORWARD_M, AFT_YAW_DEG)
    return {
        "source": "aft_lidar",
        "frame": "boat",
        "points": [[s, f] for s, f in zip(_round(stbd), _round(fwd))],
    }


class ScanPublisher:
    """Turns whatever both lidars last said into the frame's `scans` list.

    Owns no thread and no socket: `edge_link` and `aft` each run their own, and
    this only ever reads their newest answer. Called from the publish tick in
    `main.py`, which is on the loop that owes the autopilot its heartbeat.
    """

    def __init__(self, edge_link=None, aft=None, max_age=MAX_SWEEP_AGE_S):
        self.edge_link = edge_link
        self.aft = aft
        self.max_age = max_age
        # The front sweep as relayed by the autonomy node, which owns TCP 3401
        # by default (`edge_link.EDGE_OWNER`). `(scan, arrived_at, seq)` - our
        # own counter, because the relay carries no sequence and the cache below
        # keys on one. Only consulted when `edge_link` is None, i.e. when this
        # node is not the one bound to the port.
        self._relayed = None
        self._relayed_seq = 0
        self._published = 0
        self._front_points = 0
        self._aft_points = 0
        self._errors = 0
        # Last built payload per sensor, keyed by the sweep it was built from, so
        # a rotation that has to be re-sent is not re-converted. At 10 Hz that
        # matters: building ~700 points costs a millisecond or two, on the loop
        # that owes the autopilot its heartbeat.
        self._cache = {"front_lidar": (None, None), "aft_lidar": (None, None)}
        # What the dashboard is currently showing. `scans` is a list, and lists
        # REPLACE on merge, so every publish must carry the complete current set
        # - and we can stay silent whenever that set has not changed.
        self._sent = ()

    def _cached(self, source, key, build):
        """`(scan, is_new)` — build once per sweep, hand back the same object after."""
        seen, scan = self._cache[source]
        if key is not None and key == seen:
            return scan, False
        scan = build()
        self._cache[source] = (key, scan)
        return scan, True

    def relay_front(self, scans):
        """Take a front sweep built by the autonomy node. Already boat frame.

        The autonomy node holds TCP 3401, so without this the coloured front
        cloud would disappear from the operator's chart the moment autonomy
        started - which is exactly when someone most wants to see it. The points
        arrive already converted and already masked, so nothing is recomputed
        here; this is a hand-over, not a second pipeline.
        """
        if not scans:
            return
        front = next(
            (s for s in scans if isinstance(s, dict) and s.get("source") == "front_lidar"),
            None,
        )
        if front is None:
            return
        self._relayed_seq += 1
        self._relayed = (front, time.time(), self._relayed_seq)

    def aft_scan_for_planner(self):
        """The newest fresh aft sweep as a plain dict, or None.

        The autonomy node needs it and cannot read the port - this node owns the
        serial link. The front sweep goes the other way, so between them each
        sensor crosses the bus exactly once and in one direction.
        """
        if self.aft is None:
            return None
        sweep = self.aft.latest()
        if sweep is None or time.time() - sweep.t_end > self.max_age:
            return None
        scan, _is_new = self._cached("aft_lidar", sweep.seq, lambda: aft_scan(sweep))
        return scan

    def _front(self):
        """The Jetson's newest fresh sweep, and whether it is one we have not sent."""
        if self.edge_link is None:
            # Not our port: the autonomy node is bound to 3401 and relays it.
            if self._relayed is None:
                return None, False
            scan, arrived_at, seq = self._relayed
            if time.time() - arrived_at > self.max_age:
                return None, False
            return self._cached("front_lidar", seq, lambda: scan)
        cloud, seq, arrived_at = self.edge_link.front_cloud()
        if not cloud or seq == 0:
            return None, False
        # Aged on OUR clock, from the instant the sweep landed - deliberately
        # NOT from the `t_end` the Jetson stamped into the cloud.
        #
        # Both are epoch seconds and subtracting one from the other looks
        # right; it measures the whole pipeline, sensor to here, instead of
        # just the link. But they come from two machines' wall clocks, and any
        # disagreement bigger than max_age makes every sweep read as ancient -
        # so a perfectly healthy front lidar is dropped on every single tick,
        # forever, while the aft unit stamped on this machine carries on fine.
        # That is not a hypothetical: it is what the first hardware run looked
        # like (3701 sweeps received, 0 points plotted), and it is worse than
        # the problem it was guarding against, because a link so backed up that
        # it hands us two-second-old rotations announces itself in `sweep_hz`
        # anyway.
        #
        # The cross-machine offset is still worth knowing, so `edge_link`
        # measures and publishes it as `clock_offset_s` rather than quietly
        # dying of it.
        if arrived_at is None or time.time() - arrived_at > self.max_age:
            return None, False
        return self._cached("front_lidar", seq, lambda: front_scan(cloud))

    def _aft(self):
        if self.aft is None:
            return None, False
        sweep = self.aft.latest()
        if sweep is None or time.time() - sweep.t_end > self.max_age:
            return None, False
        return self._cached("aft_lidar", sweep.seq, lambda: aft_scan(sweep))

    def publish_fields(self):
        """`{"scans": [...]}` when the plot has changed, `{}` when it has not.

        Returning nothing at all is the common case at 10 Hz and is the point of
        this method. Both units turn at about 10 Hz and neither is synchronised
        to our tick, so plenty of ticks find the same rotation they saw last
        time; re-sending it would burn a frame to redraw an identical picture.

        When something HAS changed, the complete current set goes out - both
        sweeps, including one that was already sent. That is forced by `scans`
        being a list: lists replace rather than merge on the server
        (`ligmax_gui/state.py`), so publishing only the sensor that happened to
        turn would take the other one's points off the chart ten times a second.

        The same replace rule is what clears a sensor that has stopped: when a
        sweep ages out the set shrinks, that counts as a change, and one publish
        takes its returns off the chart rather than leaving them there looking
        like water.
        """
        built = []
        fresh = False
        for build in (self._front, self._aft):
            try:
                scan, is_new = build()
            except Exception as exc:  # noqa: BLE001 - geometry must not kill the loop
                self._errors += 1
                log.warning("could not build a scan: %s", exc)
                continue
            if scan:
                built.append(scan)
                fresh = fresh or is_new

        sources = tuple(scan["source"] for scan in built)
        if not fresh and sources == self._sent:
            return {}       # same rotations, same sensors: nothing to say

        self._sent = sources
        self._published += 1
        self._front_points = next(
            (len(s["points"]) for s in built if s["source"] == "front_lidar"), 0
        )
        self._aft_points = next(
            (len(s["points"]) for s in built if s["source"] == "aft_lidar"), 0
        )
        return {"scans": built}

    def telemetry(self):
        """`telemetry.lidar` - which of the two is answering, and how.

        Deliberately reports the two sensors separately. They fail in unrelated
        ways and by completely different routes: the front one goes away when
        the Jetson's link drops or when `ligmax-edge/run.sh` was started without
        `LIDAR=1`, the aft one when a USB serial port does not come up. A single
        "lidar ok" flag would hide which.
        """
        block = {
            "front_points": self._front_points,
            "aft_points": self._aft_points,
            "published": self._published,
            "geometry": {
                "front_forward_m": FRONT_FORWARD_M,
                "aft_forward_m": AFT_FORWARD_M,
                "aft_yaw_deg": AFT_YAW_DEG,
            },
        }
        if self._errors:
            block["errors"] = self._errors
        if self.edge_link is not None:
            block["front"] = self.edge_link.telemetry()
        if self.aft is not None:
            block["aft"] = self.aft.stats()
        return block
