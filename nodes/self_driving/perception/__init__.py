"""Sensors -> objects.

Three steps, deliberately kept apart so each can be tested against a captured
file with no boat attached:

    masks.py      drop the returns that are the boat looking at itself - the
                  aft unit faces up the length of the hull
    cluster.py    a lidar sweep -> `Cluster` measurements, boat frame
    classify.py   a `Cluster` -> what kind of thing it is, from its colour, its
                  size, and what the boat is currently doing (`policy_for`)
    world.py      measurements over time -> `Track`s that persist, world frame

The order matters and so does the direction of trust: geometry first (the C1 is
+-3 cm), colour second (measured, but by an uncalibrated camera), and the YOLO
detector last and only ever as a refinement - see `world.absorb_detections`.

The task is an input, not a filter bolted on afterwards. A leg that follows GPS
points under buoy rules is a task about marks, and `classify.policy_for` is what
lets it decline to name the shore and the moored hulls without declining to avoid
them.

One branch off that chain, for the parking task:

    lines.py      a lidar sweep -> `Segment` straight edges, boat frame
    parking.py    three `Segment`s -> a `ParkingBox` and the dot in the middle

It is a branch and not a step: nothing in it classifies, tracks or remembers
anything, and the parking behaviours use it *instead of* the chain above rather
than after it - not even `policy_for`, because a berth is geometry and the marks
around it are scenery. See `parking.py`.
"""

from .cluster import Cluster, cluster_sweep, split_by_gap
from .classify import (
    CardinalVote,
    TaskPolicy,
    classify,
    colour_votes,
    mark_colour_mask,
    policy_for,
)
from .lines import Segment, fit_segments, fit_sweeps
from .masks import apply as apply_mask, mask_aft, mask_front
from .parking import ParkingBox, find_box
from .world import Track, WorldModel

__all__ = [
    "Cluster",
    "cluster_sweep",
    "split_by_gap",
    "CardinalVote",
    "TaskPolicy",
    "classify",
    "colour_votes",
    "mark_colour_mask",
    "policy_for",
    "Segment",
    "fit_segments",
    "fit_sweeps",
    "apply_mask",
    "mask_aft",
    "mask_front",
    "ParkingBox",
    "find_box",
    "Track",
    "WorldModel",
]
