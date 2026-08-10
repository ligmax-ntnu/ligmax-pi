"""Sensors -> objects.

Three steps, deliberately kept apart so each can be tested against a captured
file with no boat attached:

    masks.py      drop the returns that are the boat looking at itself - the
                  aft unit faces up the length of the hull
    cluster.py    a lidar sweep -> `Cluster` measurements, boat frame
    classify.py   a `Cluster` -> what kind of thing it is, from its colour and
                  its size
    world.py      measurements over time -> `Track`s that persist, world frame

The order matters and so does the direction of trust: geometry first (the C1 is
+-3 cm), colour second (measured, but by an uncalibrated camera), and the YOLO
detector last and only ever as a refinement - see `world.absorb_detections`.

One branch off that chain, for the parking task:

    lines.py      a lidar sweep -> `Segment` straight edges, boat frame
    parking.py    three `Segment`s -> a `ParkingBox` and the dot in the middle

It is a branch and not a step: nothing in it classifies, tracks or remembers
anything, and the parking behaviours use it *instead of* the chain above rather
than after it. See `parking.py` for why a berth is geometry and not objects.
"""

from .cluster import Cluster, cluster_sweep, split_by_gap
from .classify import CardinalVote, classify, colour_votes
from .lines import Segment, fit_segments, fit_sweeps
from .masks import apply as apply_mask, mask_aft, mask_front
from .parking import ParkingBox, find_box
from .world import Track, WorldModel

__all__ = [
    "Cluster",
    "cluster_sweep",
    "split_by_gap",
    "CardinalVote",
    "classify",
    "colour_votes",
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
