"""What the boat knows about itself, in one immutable object per tick.

`BoatState` is built fresh from each state frame `io_manager` publishes, and it
is the *only* thing the behaviours are allowed to read about the vessel. That is
the point: a behaviour is then a pure function of `(state, world, plan)`, which
means it can be run against a recorded trip on a laptop with no boat, no
Pixhawk, and no lidar. Every bug found that way is one not found on the water
with fifteen minutes on the clock.

Everything is optional
----------------------
Every field can be `None`, because every one of them can genuinely be missing:
no GPS fix yet, no heading before the compass settles, no origin until the first
usable fix. Nothing here substitutes a plausible default for a missing
measurement - `usable` is the single question a caller asks before steering, and
`why_unusable` says which piece is absent, in words, because that string ends up
in front of an operator wondering why the boat will not move.

Ages, not timestamps
--------------------
`age` is measured against this machine's clock at construction, and the frame
carries the time io_manager sampled it. Both nodes are on the same Pi, so unlike
the Jetson link there is no cross-machine clock problem here - but the age is
still what matters, because a frame that stopped arriving looks exactly like a
boat that stopped moving unless somebody checks.
"""

from __future__ import annotations

import math
import time


class BoatState:
    """One tick's view of the vessel. Cheap to build, never mutated."""

    __slots__ = (
        "t", "received_at", "origin", "position", "heading", "cog", "sog",
        "velocity", "lat", "lon", "fix", "satellites", "status", "estop",
        "mode", "armed", "aft_scan", "rc_link", "operator_link", "raw",
        "telemetry",
    )

    def __init__(self, frame, received_at=None):
        self.raw = frame or {}
        self.received_at = received_at if received_at is not None else time.time()
        self.t = _number(self.raw.get("t"), self.received_at)

        self.origin = self.raw.get("origin") or None

        boat = self.raw.get("boat") or {}
        position = boat.get("position")
        self.position = (
            (float(position[0]), float(position[1]))
            if isinstance(position, (list, tuple)) and len(position) >= 2
            else None
        )
        self.heading = _number(boat.get("heading_deg"), None)
        velocity = boat.get("velocity")
        self.velocity = (
            (float(velocity[0]), float(velocity[1]))
            if isinstance(velocity, (list, tuple)) and len(velocity) >= 2
            else None
        )

        gps = self.raw.get("gps") or {}
        self.lat = _number(gps.get("lat"), None)
        self.lon = _number(gps.get("lon"), None)
        self.fix = gps.get("fix")
        self.satellites = gps.get("satellites")

        motion = self.raw.get("motion") or {}
        self.sog = _number(motion.get("sog"), None)
        self.cog = _number(motion.get("cog_deg"), None)
        if self.heading is None:
            self.heading = _number(motion.get("heading_deg"), None)

        self.status = self.raw.get("status")
        self.estop = bool(self.raw.get("estop"))
        self.mode = self.raw.get("mode")
        self.armed = self.raw.get("armed")
        self.rc_link = bool(self.raw.get("rc_link"))
        self.operator_link = bool(self.raw.get("operator_link"))

        # The aft lidar, which io_manager owns because the serial port is on its
        # side of the boat. The front one comes to this node directly off TCP
        # 3401 and never travels through a frame.
        self.aft_scan = self.raw.get("aft_scan")

        # io_manager's whole dashboard telemetry block - battery, BMS, RTK,
        # trim, tuning, lights, propulsion, safety - forwarded at 1 Hz. **No
        # behaviour reads this**, deliberately: a planner that steers on the
        # pack voltage is a planner nobody can reason about. It is here for the
        # trip recording, which otherwise cannot answer why a boat stopped.
        # Present only on the frames that carried it, so it can be None.
        self.telemetry = self.raw.get("telemetry")

    # ------------------------------------------------------------------ query

    def age(self, now=None):
        """Seconds since this frame arrived. The comms-loss test."""
        return (now if now is not None else time.time()) - self.received_at

    @property
    def usable(self):
        """Whether there is enough here to steer on."""
        return (
            self.position is not None
            and self.heading is not None
            and self.origin is not None
        )

    @property
    def why_unusable(self):
        """Which piece is missing, in words. Empty when nothing is."""
        missing = []
        if self.origin is None:
            missing.append("no grid origin (waiting for a 3D fix)")
        if self.position is None:
            missing.append("no position")
        if self.heading is None:
            missing.append("no heading")
        return "; ".join(missing)

    @property
    def speed(self):
        """Speed over ground, or 0.0 if it is not being reported."""
        if self.sog is not None:
            return self.sog
        if self.velocity is not None:
            return math.hypot(*self.velocity)
        return 0.0

    @property
    def stationary(self):
        """For the two scored "stay stationary" requirements (NJORD §9.1, §9.3)."""
        from .config import STATIONARY_SPEED_MS

        return self.speed <= STATIONARY_SPEED_MS

    @property
    def world_velocity(self):
        """`(east, north)` m/s. Preferred from the chart velocity, then from
        course and speed over ground, then zero.

        Zero rather than None deliberately: this feeds the CPA calculation, and
        a missing own-velocity there means "treat me as stationary", which is
        the *conservative* reading - a stationary give-way vessel manoeuvres
        earlier than a moving one would.
        """
        if self.velocity is not None:
            return self.velocity
        if self.cog is not None and self.sog is not None:
            east, north = _unit(self.cog)
            return (east * self.sog, north * self.sog)
        return (0.0, 0.0)

    def describe(self):
        """One line for a log or the operator's panel."""
        if not self.usable:
            return f"state unusable: {self.why_unusable}"
        return (
            f"at ({self.position[0]:.1f}, {self.position[1]:.1f}) m "
            f"hdg {self.heading:.0f} deg {self.speed:.2f} m/s "
            f"[{self.fix or 'no fix'}, {self.mode or 'no mode'}, "
            f"{'armed' if self.armed else 'disarmed'}]"
        )


def _number(value, default):
    try:
        out = float(value)
    except (TypeError, ValueError):
        return default
    return default if out != out else out  # NaN


def _unit(bearing_deg):
    rad = math.radians(bearing_deg)
    return math.sin(rad), math.cos(rad)
