"""What the boat can meet on the water, as one enumeration.

**The numeric values are a wire format** - the frontend mirrors this enum so the
operator's chart can colour an obstacle the same way the planner reasons about
it. The original members therefore keep their numbers for ever: append, never
renumber, never reuse. New members since the first version are 10 and up.

The four things a type decides
------------------------------
    which side to pass    RED/GREEN carry the IALA rule; the cardinals carry
                          their own. Everything else is simply avoided, on
                          whichever side is roomier.
    how much room         a nailed-down mark needs `BUOY_CLEARANCE_M`; something
                          that moves needs `VESSEL_CLEARANCE_M`.
    how hard to smooth    a mark cannot move, so its track can be averaged hard;
                          a vessel's cannot (`config.TRACK_ALPHA_*`).
    whether to predict    only `BOAT` gets a velocity estimate and a CPA.

How a type is arrived at, and how much to trust it
--------------------------------------------------
Two independent sources, trusted very differently.

    the lidar     says an object EXISTS, where it is, and how big it is. The C1
                  is +-3 cm, and the front unit's returns arrive already
                  coloured by the Jetson's cameras - so a cluster carries a
                  colour without the detector being involved at all. Trusted.
    the YOLO      says what an object IS, and on this boat it is weak. Used only
                  to break ties, and for the one thing no lidar can do: telling
                  a north cardinal from a south one, which is a topmark rather
                  than a shape or a colour.

So the colour path is the primary classifier and it comes off the *lidar*, not
off the detector - see `perception/classify.py`:

    red             -> RED buoy
    green           -> GREEN buoy
    yellow          -> CARDINAL (a mark; *which* cardinal is a camera question)
    white           -> BOAT or LAND, decided by size and by what the boat is
                       doing: a 2 m white object during a docking task is the
                       dock, the same object during collision avoidance is the
                       Otter.
    blue / dark     -> water, spray, or a shadow. Discarded, not tracked.
"""

from enum import Enum

import numpy as np


# The frontend needs the same code
class Enviroment:
    """The direction of buoyage, which is what makes red mean "port".

    IALA region A: sailing *seaward* you keep red to port and green to
    starboard. Njord's course is laid with seaward = north (NJORD §10.2), but a
    leg that runs back down the course inverts the sense, and a boat applying
    the outbound rule on the return passes every gate on the wrong side.

    So this is carried explicitly per leg rather than assumed - `plan.py`'s
    `channel_bearing` is what fills it in.
    """

    def __init__(self, upstream_direction: np.ndarray):
        self.upstream_direction = upstream_direction


# The frontend needs the same code
class ObstacleType(Enum):
    UNKNOWN = 0

    RED = 1
    GREEN = 2
    NORTH = 3
    SOUTH = 4
    WEST = 5
    EAST = 6

    BOAT = 7
    LAND = 8
    DOCKING_CENTER = 9

    # Appended - see the module docstring on why nothing above is renumbered.
    # A mark that reads black-and-yellow, before the camera has said which of
    # the four it is. It has to be avoidable in that state, because the detector
    # may never commit, and "unknown cardinal" is a far more useful thing to put
    # in front of an operator than "unknown object".
    CARDINAL = 10


#: Marks that carry a lateral rule: which side of them the boat must pass.
BUOY_TYPES = frozenset({ObstacleType.RED, ObstacleType.GREEN})

#: The four resolved cardinals, plus the unresolved one.
CARDINAL_TYPES = frozenset(
    {
        ObstacleType.NORTH,
        ObstacleType.SOUTH,
        ObstacleType.EAST,
        ObstacleType.WEST,
        ObstacleType.CARDINAL,
    }
)

#: Anything nailed to the seabed or the shore. Smoothed hard, never predicted.
STATIC_TYPES = BUOY_TYPES | CARDINAL_TYPES | {ObstacleType.LAND}

#: The Jetson's detector classes (`edge_protocol.CLASS_NAMES`) as our types.
FROM_DETECTOR_CLASS = {
    0: ObstacleType.GREEN,
    1: ObstacleType.RED,
    2: ObstacleType.CARDINAL,
}

#: The detector's second-stage cardinal classifier (`card`) as our types.
FROM_CARDINAL_NAME = {
    "north": ObstacleType.NORTH,
    "south": ObstacleType.SOUTH,
    "east": ObstacleType.EAST,
    "west": ObstacleType.WEST,
}

#: The compass bearing you must be on, *from* the mark, to be passing it
#: legally: a north cardinal is passed to its north (NJORD §10.3, IALA).
CARDINAL_SAFE_BEARING = {
    ObstacleType.NORTH: 0.0,
    ObstacleType.EAST: 90.0,
    ObstacleType.SOUTH: 180.0,
    ObstacleType.WEST: 270.0,
}

_LABELS = {
    ObstacleType.UNKNOWN: "unknown",
    ObstacleType.RED: "red buoy",
    ObstacleType.GREEN: "green buoy",
    ObstacleType.NORTH: "north cardinal",
    ObstacleType.SOUTH: "south cardinal",
    ObstacleType.EAST: "east cardinal",
    ObstacleType.WEST: "west cardinal",
    ObstacleType.CARDINAL: "cardinal (side unknown)",
    ObstacleType.BOAT: "vessel",
    ObstacleType.LAND: "structure",
    ObstacleType.DOCKING_CENTER: "berth",
}


def is_static(obstacle_type):
    return obstacle_type in STATIC_TYPES


def clearance_for(obstacle_type, config):
    """How much room to give this thing, metres."""
    if obstacle_type == ObstacleType.BOAT:
        return config.VESSEL_CLEARANCE_M
    return config.BUOY_CLEARANCE_M


def label(obstacle_type):
    """A name for the operator's chart and for the log. Never an enum repr."""
    return _LABELS.get(obstacle_type, str(obstacle_type))
