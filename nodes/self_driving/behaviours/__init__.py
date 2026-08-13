"""One behaviour per waypoint role. `for_role()` is the only entry point.

A behaviour is a small object with `update(ctx) -> Intent` and a `done` flag,
and nothing else. It reads the world through `Context` and acts through an
intent, so every one of them runs against a recorded trip with no boat attached
- which is where the bugs should be found, rather than in a fifteen-minute slot
with a jury watching.

    transit         `transit.Transit`   blind GNSS following        NJORD §9.1 pt 1
    buoys           `buoys.Buoys`       + lateral marks, cardinals  NJORD §9.1 pt 2
    avoid           `colregs.Colregs`   + give way to a vessel      NJORD §9.2
    collision_front `collision.CollisionFront`  see it ahead -> starboard, rejoin
    collision_right `collision.CollisionRight`  see it to starboard -> stop
    collision_front_backup  `collision.CollisionFrontBackup`  the same, blind
    collision_right_backup  `collision.CollisionRightBackup`  the same, blind
    hold            `hold.Hold`         arrive and station-keep     NJORD §9.1
    dock            `dock.Dock`         bow-in, hold 10 s, reverse  NJORD §9.3.1
    dock_parallel   `dock.Dock`         alongside, hold 5 s, ahead  NJORD §9.3.2
    park            `parking.Parking`   three lines, hold 10 s, out NJORD §9.3.1
    park_parallel   `parking.Parking`   the same, alongside         NJORD §9.3.2
    park_tag        `parking.Parking`   three AR TAGS, hold, out    NJORD §9.3.1
    park_tag_parallel `parking.Parking` the same, alongside         NJORD §9.3.2

`task` on each class is what the classifier is told the boat is doing, and it
exists for one genuine ambiguity: a big white object is the dock during docking
and the Otter during collision avoidance (`perception/classify.py`).

Three ways to park, and only one of them currently has a sensor
---------------------------------------------------------------
`dock*` finds a berth as a **gap between two lidar clusters** and keeps the
ordinary obstacle avoidance switched on. `park*` finds it as **three lines making
a rectangle with open corners** fitted to the lidar, parks on the middle of that
rectangle plus a static per-type depth offset, and consults the world model not at
all - no buoy colours, no clearances, no avoidance nudge. See `parking.py`'s
docstring for why the avoidance is off in there.

`park_tag*` finds the **same rectangle out of the dock's three AR markers**
(`perception/artags.py`) and is the same `Parking` object with `source="artag"` -
the manoeuvre never depended on the lidar, only the measurement did.

**As of 2026-08-11 the tag roles are the only ones that can work**, because both
lidars are down: the other four will sit in SEARCH reporting no space until an
operator takes over. The lidar roles are kept rather than deleted because a
repaired sensor brings them back with no code change, and because on a boat where
one sensor has just died, deleting the code for the other one is how a team ends
up with neither.
"""

from .. import plan as plan_roles
from .base import Behaviour, Context
from .buoys import Buoys
from .collision import (
    Collision,
    CollisionFront,
    CollisionFrontBackup,
    CollisionRight,
    CollisionRightBackup,
)
from .colregs import Colregs
from .dock import Dock
from .hold import Hold
from .parking import Parking
from .transit import Transit

__all__ = [
    "Behaviour",
    "Context",
    "Buoys",
    "Collision",
    "CollisionFront",
    "CollisionFrontBackup",
    "CollisionRight",
    "CollisionRightBackup",
    "Colregs",
    "Dock",
    "Hold",
    "Parking",
    "Transit",
    "for_role",
]


def for_role(role, config):
    """A fresh behaviour for a waypoint's role.

    Fresh, not cached: every behaviour carries per-waypoint state (which docking
    phase, which COLREG situation it committed to, when it arrived), and reusing
    an instance across waypoints would carry that into a leg it does not belong
    to. They are a few dozen bytes each.

    An unknown role falls back to `transit`, which is the conservative choice -
    it still avoids everything it can see, it just does not apply a rule it does
    not understand. `plan.py` refuses unknown roles at upload time, so reaching
    this line means a stored plan from an older build.
    """
    if role == plan_roles.BUOYS:
        return Buoys(config)
    if role == plan_roles.AVOID:
        return Colregs(config)
    if role == plan_roles.COLLISION_FRONT:
        return CollisionFront(config)
    if role == plan_roles.COLLISION_RIGHT:
        return CollisionRight(config)
    if role == plan_roles.COLLISION_FRONT_BACKUP:
        return CollisionFrontBackup(config)
    if role == plan_roles.COLLISION_RIGHT_BACKUP:
        return CollisionRightBackup(config)
    if role == plan_roles.HOLD:
        return Hold(config)
    if role == plan_roles.DOCK:
        return Dock(config, parallel=False)
    if role == plan_roles.DOCK_PARALLEL:
        return Dock(config, parallel=True)
    if role == plan_roles.PARK:
        return Parking(config, parallel=False)
    if role == plan_roles.PARK_PARALLEL:
        return Parking(config, parallel=True)
    if role == plan_roles.PARK_TAG:
        return Parking(config, parallel=False, source="artag")
    if role == plan_roles.PARK_TAG_PARALLEL:
        return Parking(config, parallel=True, source="artag")
    return Transit(config)
