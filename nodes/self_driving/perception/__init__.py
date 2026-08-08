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
"""

from .cluster import Cluster, cluster_sweep, split_by_gap
from .classify import CardinalVote, classify, colour_votes
from .masks import apply as apply_mask, mask_aft, mask_front
from .world import Track, WorldModel

__all__ = [
    "Cluster",
    "cluster_sweep",
    "split_by_gap",
    "CardinalVote",
    "classify",
    "colour_votes",
    "apply_mask",
    "mask_aft",
    "mask_front",
    "Track",
    "WorldModel",
]
