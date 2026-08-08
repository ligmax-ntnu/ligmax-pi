"""The world model: what is out there, where, and how sure we are.

    world = WorldModel(config)
    world.observe(clusters, boat_xy, heading_deg, now, context="buoys")
    world.absorb_detections(dets, boat_xy, heading_deg, now)
    for track in world.confirmed():
        ...

One sweep of the lidar is a set of *measurements*; this turns them into
*objects that persist*. That is the difference that lets the boat plan at all:
you cannot decide to pass a buoy to port if the buoy ceases to exist every
100 ms and is replaced by a stranger.

Everything here is in the WORLD frame - metres east and north of the grid origin
- for one reason: **the boat moves.** Track a buoy in the boat frame and it
appears to sweep across the bow every time the boat turns, so its "velocity" is
the boat's own rotation and any attempt to tell a moving vessel from a fixed
mark fails immediately. In the world frame a mark's position is constant, which
makes "is this thing moving?" a question with an answer.

Why association is Hungarian and not greedy
-------------------------------------------
Two buoys 5 m apart in a gate, seen at 15 m, have overlapping association gates.
Greedy nearest-neighbour will happily give both tracks to the same detection and
then invent a new track for the other one, which is how a gate turns into three
buoys and the boat steers between the wrong pair. `linear_sum_assignment` picks
the assignment that minimises total error instead, so the pair stays a pair. It
costs microseconds at these counts.

Two things are deliberately NOT done here
-----------------------------------------
No occupancy grid, and no map that outlives the run. The Njord course is a
handful of discrete marks in open water, and a grid would spend its whole time
representing empty sea. And a track that has not been seen for
`TRACK_DROP_AFTER_S` is dropped rather than remembered: on a boat with a 3D fix
the position error grows faster than the memory is worth, and a remembered buoy
that is 4 m from where it really is, is worse than no buoy at all.
"""

import math

import numpy as np

from .. import geo
from ..obsticales import (
    FROM_CARDINAL_NAME,
    FROM_DETECTOR_CLASS,
    ObstacleType,
    is_static,
    label,
)
from .classify import CardinalVote, classify


class Track:
    """One persistent object in the world frame.

    `pos` is smoothed, `velocity` is only estimated for things that can move.
    `kind` can change - a cluster first seen as UNKNOWN and later coloured green
    becomes GREEN - but a *committed* cardinal never changes back, because the
    vote that committed it required agreement the next frame cannot undo.
    """

    __slots__ = (
        "id", "kind", "confidence", "pos", "velocity", "width_m", "hits",
        "first_seen", "last_seen", "cardinal", "reason", "_kind_votes", "_last_pos",
        "_last_move_t", "source",
    )

    def __init__(self, track_id, pos, kind, confidence, width_m, now, config, source):
        self.id = track_id
        self.pos = (float(pos[0]), float(pos[1]))
        self.kind = kind
        self.confidence = float(confidence)
        self.width_m = float(width_m)
        self.hits = 1
        self.first_seen = now
        self.last_seen = now
        self.cardinal = CardinalVote(config)
        self.reason = ""
        self.velocity = (0.0, 0.0)
        self.source = source
        self._kind_votes = {kind: confidence}
        self._last_pos = self.pos
        self._last_move_t = now

    # ------------------------------------------------------------------ update

    def observe(self, pos, kind, confidence, width_m, now, config, reason=""):
        """Fold in one new measurement."""
        alpha = (
            config.TRACK_ALPHA_STATIC if is_static(self.kind)
            else config.TRACK_ALPHA_VESSEL
        )
        new_pos = (
            self.pos[0] + alpha * (pos[0] - self.pos[0]),
            self.pos[1] + alpha * (pos[1] - self.pos[1]),
        )

        # Velocity, for the things that can have one. Differenced over the whole
        # interval since the last estimate rather than tick to tick: at 10 Hz a
        # single tick's displacement for a 1.3 m/s Otter is 13 cm, which is the
        # same size as the position noise, so a per-tick derivative is mostly
        # noise. A quarter-second baseline is four times the signal.
        if not is_static(kind):
            dt = now - self._last_move_t
            if dt >= 0.25:
                vx = (new_pos[0] - self._last_pos[0]) / dt
                vy = (new_pos[1] - self._last_pos[1]) / dt
                self.velocity = (
                    0.6 * self.velocity[0] + 0.4 * vx,
                    0.6 * self.velocity[1] + 0.4 * vy,
                )
                self._last_pos = new_pos
                self._last_move_t = now
        else:
            self.velocity = (0.0, 0.0)

        self.pos = new_pos
        self.width_m = 0.7 * self.width_m + 0.3 * float(width_m)
        self.hits += 1
        self.last_seen = now
        if reason:
            self.reason = reason

        # Type by accumulated vote, not by the newest frame. One badly coloured
        # sweep must not turn a green buoy red on the tick the boat commits to
        # a side.
        if kind != ObstacleType.UNKNOWN:
            self._kind_votes[kind] = self._kind_votes.get(kind, 0.0) + confidence
            best = max(self._kind_votes.items(), key=lambda item: item[1])
            if not self._committed_cardinal():
                self.kind = best[0]
        # Probability-style accumulation: repeated weak evidence adds up, and no
        # single observation can take it to certainty.
        self.confidence = 1.0 - (1.0 - self.confidence) * (1.0 - min(0.95, confidence))

    def _committed_cardinal(self):
        return self.cardinal.committed is not None

    def note_cardinal(self, name, confidence):
        """A camera vote on which cardinal this is. Commits when convinced."""
        self.cardinal.add(name, confidence)
        if self.cardinal.committed:
            resolved = FROM_CARDINAL_NAME.get(self.cardinal.committed)
            if resolved is not None:
                self.kind = resolved

    def decay(self, dt, config):
        self.confidence = max(0.0, self.confidence - config.TRACK_DECAY_PER_S * dt)

    # ------------------------------------------------------------------ query

    @property
    def speed(self):
        return math.hypot(*self.velocity)

    def predicted(self, seconds):
        """Where it will be in `seconds`, on its current velocity."""
        return (
            self.pos[0] + self.velocity[0] * seconds,
            self.pos[1] + self.velocity[1] * seconds,
        )

    def age(self, now):
        return now - self.last_seen

    def telemetry(self):
        """One entry for the dashboard's obstacle layer.

        `type` is the enum's *number*, because the frontend mirrors
        `obsticales.py` and switches on it; `label` is the same thing in words,
        so a jury member reading the chart does not have to.
        """
        block = {
            "id": self.id,
            "type": self.kind.value,
            "label": label(self.kind),
            "position": [round(self.pos[0], 2), round(self.pos[1], 2)],
            "confidence": round(self.confidence, 3),
            "hits": self.hits,
            "width_m": round(self.width_m, 2),
            "source": self.source,
        }
        if self.speed > 0.1:
            block["velocity"] = [round(self.velocity[0], 2), round(self.velocity[1], 2)]
            block["speed"] = round(self.speed, 2)
        if self.reason:
            block["why"] = self.reason
        if self.kind in (ObstacleType.CARDINAL,) or self.cardinal.tally:
            block["cardinal"] = self.cardinal.describe()
        return block


class WorldModel:
    """Every object the boat currently believes in."""

    def __init__(self, config):
        self._config = config
        self._tracks = []
        self._next_id = 1
        self._last_update = None
        self.observations = 0

    # ---------------------------------------------------------------- ingest

    def observe(self, clusters, boat_xy, heading_deg, now, context="transit"):
        """Fold one sweep's clusters in. `clusters` are BOAT frame.

        Silently does nothing without a boat pose: a cluster cannot be placed in
        the world frame without knowing where the boat was when it was measured,
        and putting it at the origin instead would populate the map with
        phantoms clustered around grid (0, 0).
        """
        if boat_xy is None or heading_deg is None:
            return
        self._age(now)
        if not clusters:
            return

        measurements = []
        for cluster in clusters:
            kind, confidence, why = classify(
                cluster, self._config, context, cluster.gains
            )
            east, north = geo.boat_to_world(
                cluster.centre[0], cluster.centre[1], heading_deg
            )
            measurements.append(
                {
                    "pos": (boat_xy[0] + east, boat_xy[1] + north),
                    "kind": kind,
                    "confidence": confidence,
                    "width_m": cluster.width_m,
                    "why": why,
                    "range_m": cluster.range_m,
                    "source": cluster.source,
                }
            )

        matched = self._associate(measurements)
        for m_index, t_index in matched.items():
            m = measurements[m_index]
            self._tracks[t_index].observe(
                m["pos"], m["kind"], m["confidence"], m["width_m"],
                now, self._config, m["why"],
            )
        for m_index, m in enumerate(measurements):
            if m_index in matched:
                continue
            self._tracks.append(
                Track(
                    self._next_id, m["pos"], m["kind"], max(0.25, m["confidence"]),
                    m["width_m"], now, self._config, m["source"],
                )
            )
            self._tracks[-1].reason = m["why"]
            self._next_id += 1
        self.observations += len(measurements)

    def _associate(self, measurements):
        """`{measurement index: track index}`, by optimal assignment.

        The gate grows with range because a bearing error is an arc: half a
        degree is 4 cm at 5 m and 17 cm at 20 m, so a fixed gate is either too
        tight far away or too loose close in.

        Types that disagree are forbidden from matching outright, *unless* one
        of them is UNKNOWN - a track that has not been coloured yet must be
        allowed to become a green buoy, but a red buoy must never quietly
        become the Otter.
        """
        if not self._tracks or not measurements:
            return {}

        from scipy.optimize import linear_sum_assignment

        track_pos = np.array([t.pos for t in self._tracks], dtype=np.float64)
        meas_pos = np.array([m["pos"] for m in measurements], dtype=np.float64)
        cost = np.linalg.norm(meas_pos[:, None, :] - track_pos[None, :, :], axis=2)

        forbidden = 1e6
        for m_index, m in enumerate(measurements):
            gate = (
                self._config.TRACK_GATE_M
                + self._config.TRACK_GATE_PER_M * m["range_m"]
            )
            for t_index, track in enumerate(self._tracks):
                incompatible = (
                    m["kind"] != track.kind
                    and m["kind"] != ObstacleType.UNKNOWN
                    and track.kind != ObstacleType.UNKNOWN
                )
                if incompatible or cost[m_index, t_index] > gate:
                    cost[m_index, t_index] = forbidden

        rows, cols = linear_sum_assignment(cost)
        return {
            int(r): int(c) for r, c in zip(rows, cols) if cost[r, c] < forbidden
        }

    def absorb_detections(self, detections, boat_xy, heading_deg, now):
        """Fold in the Jetson's camera detections. Only ever *refines*.

        A detection never creates a track. The detector is weak, its range from
        apparent size degrades as the square of distance, and a phantom buoy on
        the chart is worse than a missing one - so its job here is to answer
        questions the lidar posed, not to pose new ones.

        It answers two:

          * **which cardinal** a yellow mark is (the topmark - no lidar can see
            it), through `Track.note_cardinal`;
          * a weak vote on red vs green, for a mark the colour path left
            UNKNOWN because no camera happened to cover it.

        The position used for matching comes from the detection's own `lidar`
        block where it has one - those returns are the same measurement this
        model was built from, so they land on the right track - and falls back
        to bearing plus range-from-apparent-size, which is why the match gate is
        widened when it does.
        """
        if not detections or boat_xy is None or heading_deg is None:
            return
        for det in detections:
            placed = self._detection_position(det, boat_xy, heading_deg)
            if placed is None:
                continue
            pos, slack = placed
            track = self._nearest(pos, slack)
            if track is None:
                continue

            kind = FROM_DETECTOR_CLASS.get(det.get("cls"))
            if kind is None:
                continue
            confidence = float(det.get("conf") or 0.0)

            if kind == ObstacleType.CARDINAL:
                track.note_cardinal(det.get("card"), float(det.get("card_conf") or 0.0))
                if track.kind == ObstacleType.UNKNOWN:
                    track.kind = ObstacleType.CARDINAL
                continue

            # A deliberately weak vote: the detector gets a say only where the
            # coloured lidar had none.
            if track.kind == ObstacleType.UNKNOWN and confidence > 0.4:
                track.observe(
                    track.pos, kind, confidence * 0.5, track.width_m, now,
                    self._config,
                    f"camera says {label(kind)} at {confidence:.0%} (no lidar colour)",
                )

    def _detection_position(self, det, boat_xy, heading_deg):
        """`(world position, extra match slack)` for a detection, or None.

        The rig frame's origin is the front lidar, and `scan.py` places that
        0.5 m forward of the boat's datum - so a rig-frame range and bearing
        becomes a boat-frame point by rotating and then shifting forward.
        """
        from nodes.io_manager.scan import FRONT_FORWARD_M

        lidar = det.get("lidar") or {}
        range_m = lidar.get("range_m")
        bearing = lidar.get("bearing_deg")
        slack = 0.5
        if range_m is None or bearing is None:
            fallback = det.get("range") or {}
            if not fallback.get("valid"):
                return None
            range_m = fallback.get("range_m")
            bearing = det.get("bearing_deg")
            # Range from apparent size degrades as z**2 - about 6 % at 20 m -
            # so a match against it needs a much wider gate than a lidar range.
            slack = 2.0 + 0.15 * float(range_m or 0.0)
        if range_m is None or bearing is None:
            return None
        if not det.get("in_valid_cone", True):
            return None

        rad = math.radians(float(bearing))
        stbd = float(range_m) * math.sin(rad)
        fwd = float(range_m) * math.cos(rad) + FRONT_FORWARD_M
        east, north = geo.boat_to_world(stbd, fwd, heading_deg)
        return (boat_xy[0] + east, boat_xy[1] + north), slack

    def _nearest(self, pos, slack):
        best, best_distance = None, None
        for track in self._tracks:
            d = geo.distance(track.pos, pos)
            gate = self._config.TRACK_GATE_M + slack
            if d <= gate and (best_distance is None or d < best_distance):
                best, best_distance = track, d
        return best

    # ----------------------------------------------------------------- ageing

    def _age(self, now):
        if self._last_update is not None:
            dt = max(0.0, now - self._last_update)
            for track in self._tracks:
                if track.last_seen < now:
                    track.decay(dt, self._config)
        self._last_update = now
        self._tracks = [
            track
            for track in self._tracks
            if track.confidence > 0.15
            and track.age(now) < self._config.TRACK_DROP_AFTER_S
        ]

    def forget(self):
        """Drop everything. Used when a new task starts on a new part of the
        course, so marks from the previous run cannot haunt this one."""
        self._tracks = []

    # ------------------------------------------------------------------ query

    def all(self):
        return list(self._tracks)

    def confirmed(self):
        """Tracks seen often enough to steer around.

        One sweep can produce a cluster out of two stray returns off a wave
        crest. Requiring `TRACK_CONFIRM_HITS` sightings is what stops the boat
        manoeuvring for foam.
        """
        return [
            track
            for track in self._tracks
            if track.hits >= self._config.TRACK_CONFIRM_HITS
        ]

    def marks(self):
        """Confirmed static marks: buoys and cardinals."""
        return [track for track in self.confirmed() if is_static(track.kind)
                and track.kind != ObstacleType.LAND]

    def vessels(self):
        """Confirmed things that can move. The Otter, and any real traffic."""
        return [track for track in self.confirmed() if track.kind == ObstacleType.BOAT]

    def structures(self):
        return [track for track in self.confirmed() if track.kind == ObstacleType.LAND]

    def nearest_to(self, xy, kinds=None, within=None):
        best, best_distance = None, None
        for track in self.confirmed():
            if kinds is not None and track.kind not in kinds:
                continue
            d = geo.distance(track.pos, xy)
            if within is not None and d > within:
                continue
            if best_distance is None or d < best_distance:
                best, best_distance = track, d
        return best

    def near_leg(self, leg_start, leg_end, corridor_m, kinds=None):
        """Confirmed tracks within `corridor_m` of a leg, in order along it.

        The order is what makes this useful: a behaviour walking a leg wants the
        next mark it will meet, not the closest one, and those differ whenever
        the boat has already passed something.
        """
        found = []
        for track in self.confirmed():
            if kinds is not None and track.kind not in kinds:
                continue
            t, along, cross = geo.project_onto_leg(track.pos, leg_start, leg_end)
            if abs(cross) <= corridor_m and -corridor_m <= along:
                found.append((along, cross, track))
        found.sort(key=lambda item: item[0])
        return found

    # ------------------------------------------------------------- telemetry

    def telemetry(self, limit=40):
        """The obstacle layer for the dashboard, most confident first."""
        tracks = sorted(
            self.confirmed(), key=lambda t: t.confidence, reverse=True
        )[:limit]
        return [track.telemetry() for track in tracks]

    def summary(self):
        """One line an operator can read at a glance."""
        counts = {}
        for track in self.confirmed():
            name = label(track.kind)
            counts[name] = counts.get(name, 0) + 1
        if not counts:
            return "nothing tracked"
        return ", ".join(f"{n}x {name}" for name, n in sorted(counts.items()))
