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

Memory, and how sure it is
--------------------------
NJORD gives two attempts at each subtask (§8.2). The marks, the dock and the
shore are in the same place for both, so a mark properly surveyed on attempt one
is worth more than almost anything else the boat can carry into attempt two -
and this model therefore remembers, which an earlier version of this file
explicitly refused to do.

The objection that refusal was based on is real and is answered rather than
ignored: **a remembered buoy 4 m from where it really is, is worse than no
buoy.** So a remembered track does not claim to be anywhere exact. Every track
carries `sigma_m`, a position uncertainty that is small while the object is in
sight and grows once it is not, up to `TRACK_SIGMA_MAX_M` - we do not know how a
mark on an unmeasured mooring drifts, so the honest ceiling is "somewhere in this
circle". Every consumer that needs room around an object adds `sigma_m` to that
object's clearance, so the boat gives a half-remembered mark a wide berth and a
mark it can see right now a tight one, automatically.

Three tiers, and what separates them
------------------------------------
    tracked      any measurement at all. Dropped after `TRACK_DROP_AFTER_S`.
    confirmed    `TRACK_CONFIRM_HITS` sightings. Steered around. This is what
                 stops the boat manoeuvring for one wave crest.
    established  `TRACK_ESTABLISH_HITS` sightings spread over
                 `TRACK_ESTABLISH_SPAN_S`, at `TRACK_ESTABLISH_CONF`, and
                 STATIC. Never dropped by time, and written to the survey file
                 so it survives a restart and the gap between two attempts.

The span requirement is the one doing the real work. A single stray coloured
return - the "one random green point, once, never again" that must never be
remembered for ever - reaches one hit and dies at the first tier. A burst off a
wave crest can reach twelve hits inside 300 ms and still cannot reach them across
two seconds. Nothing gets into permanent memory without having been looked at,
repeatedly, over time.

And **only static types are ever established**, which is what keeps the Otter out
of it: a vessel is the one object on the course guaranteed to have moved since
the boat last saw it, and remembering where it was is worse than useless.

Still deliberately NOT done here
--------------------------------
No occupancy grid. The Njord course is a handful of discrete marks in open water,
and a grid would spend its whole time representing empty sea.
"""

import logging
import math
import time

import numpy as np

from .. import geo
from ..obsticales import (
    FROM_CARDINAL_NAME,
    FROM_DETECTOR_CLASS,
    ObstacleType,
    clearance_for,
    is_static,
    label,
)
from .classify import CardinalVote, classify

log = logging.getLogger("self_driving.world")


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
        "_last_move_t", "source", "sigma_m", "established", "misses",
        "surveyed_at", "restored",
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
        # How sure we are WHERE it is, metres. Reset on every measurement, grown
        # by `refresh` while unseen. See the module docstring.
        self.sigma_m = float(config.TRACK_SIGMA_M)
        # Whether it has earned permanent memory (see `_maybe_establish`).
        self.established = False
        # Ageing steps that went by without a measurement. Purely diagnostic, but
        # it is the number that says "occluded" as opposed to "never really
        # there", and the dashboard has a field for it.
        self.misses = 0
        # Wall-clock time this was written to the survey, if it ever was, and
        # whether this instance came back off disk rather than off the lidar.
        self.surveyed_at = None
        self.restored = False

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
        self.misses = 0
        # Measured again: we know where it is to sensor accuracy once more,
        # whatever the uncertainty had grown to while it was out of sight. This
        # is the collapse that makes a remembered mark cheap to re-acquire - the
        # boat drives cautiously round a 6 m circle until it sees the mark, and
        # tightly round a 0.35 m one from the sweep after that.
        self.sigma_m = float(config.TRACK_SIGMA_M)
        self.restored = False
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
        self._maybe_establish(config)

    def _committed_cardinal(self):
        return self.cardinal.committed is not None

    def note_cardinal(self, name, confidence):
        """A camera vote on which cardinal this is. Commits when convinced."""
        self.cardinal.add(name, confidence)
        if self.cardinal.committed:
            resolved = FROM_CARDINAL_NAME.get(self.cardinal.committed)
            if resolved is not None:
                self.kind = resolved

    def _maybe_establish(self, config):
        """Promote to permanent memory, once and never back.

        Three tests, all of which must pass, and the *span* is the one that
        matters - see the module docstring. Once true this never goes false
        again: an established mark that later goes unseen is exactly the case
        the memory exists for, and un-establishing it on the first quiet second
        would defeat the whole thing.
        """
        if self.established:
            return
        if not is_static(self.kind) or self.kind == ObstacleType.UNKNOWN:
            return
        if self.hits < config.TRACK_ESTABLISH_HITS:
            return
        if (self.last_seen - self.first_seen) < config.TRACK_ESTABLISH_SPAN_S:
            return
        if self.confidence < config.TRACK_ESTABLISH_CONF:
            return
        self.established = True

    def refresh(self, now, config):
        """Recompute the position uncertainty for this instant.

        Called on every track on every tick, before anything reads `sigma_m`, so
        a behaviour never sees an uncertainty computed against a different tick's
        clock than the position it is paired with.

        A mark straight off the survey file is a special case and it has to be,
        because its `last_seen` is the *previous attempt's* wall clock. Run
        through the growth ramp that is an hour or more of ageing, which pins it
        at `TRACK_SIGMA_MAX_M` - and a 6 m uncertainty added to a 2 m clearance
        means every remembered mark claims 8 m of water on a course whose legs
        are 10 to 17 m long. The second attempt would swerve around its own map
        and very likely fail to thread it, which is the exact opposite of what
        surveying is for. It gets `SURVEY_SIGMA_M` instead: the honest error of a
        mark that was measured properly and has been sitting on its mooring
        since, rather than the error of a mark nobody can account for. The
        instant the lidar sees it, `observe` clears `restored` and this falls
        back to the ordinary ramp from a fresh measurement.
        """
        if self.restored:
            self.sigma_m = float(config.SURVEY_SIGMA_M)
            return
        age = max(0.0, now - self.last_seen)
        self.sigma_m = min(
            config.TRACK_SIGMA_MAX_M,
            config.TRACK_SIGMA_M + config.TRACK_SIGMA_GROWTH_M_S * age,
        )

    def decay(self, dt, config):
        """Lose confidence for a second unseen.

        An established track stops at `TRACK_ESTABLISH_FLOOR` rather than at
        zero. That floor is the whole mechanism behind "remembered essentially
        for ever": `WorldModel._age` culls anything under 0.15, and without a
        floor a mark the boat had studied for a minute would still evaporate
        half a minute after it went out of view.
        """
        floor = config.TRACK_ESTABLISH_FLOOR if self.established else 0.0
        self.confidence = max(
            floor, self.confidence - config.TRACK_DECAY_PER_S * dt
        )
        self.misses += 1

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

    def avoid_radius(self, config):
        """How much water this object owns, metres: clearance plus uncertainty.

        The number the dashboard draws its no-go disc from
        (`ligmax-server/web/js/nogo.js`) and the number `behaviours/base.py`
        steers around. Being the *same* number in both places is the point: what
        the operator sees on the chart is what the boat is actually avoiding, so
        a mark the boat is unsure about visibly claims more water rather than
        the boat mysteriously giving it a wider berth than the picture explains.
        """
        return round(clearance_for(self.kind, config) + self.sigma_m, 2)

    def telemetry(self, config, now=None):
        """One entry for the dashboard's obstacle layer.

        `type` is the enum's *number*, because the frontend mirrors
        `obsticales.py` and switches on it; `label` is the same thing in words,
        so a jury member reading the chart does not have to.

        Every key here is one `ligmax-server/ligmax_gui/protocol.py`
        `_normalise_track` actually keeps - it is a whitelist, and a field that
        is not on it is dropped silently somewhere over the North Sea. `radius`
        carries the position uncertainty and `age` the seconds since the object
        was last actually measured, which are the two numbers that tell an
        operator "the boat is remembering this, not seeing it".
        """
        now = now if now is not None else self.last_seen
        block = {
            "id": self.id,
            "type": self.kind.value,
            "label": label(self.kind),
            "position": [round(self.pos[0], 2), round(self.pos[1], 2)],
            "confidence": round(self.confidence, 3),
            "hits": self.hits,
            "misses": self.misses,
            "width_m": round(self.width_m, 2),
            "source": self.source,
            "avoid_radius": self.avoid_radius(config),
            "radius": round(self.sigma_m, 2),
            "age": round(max(0.0, now - self.last_seen), 1),
        }
        if self.speed > 0.1:
            block["velocity"] = [round(self.velocity[0], 2), round(self.velocity[1], 2)]
            block["speed"] = round(self.speed, 2)

        why = self.reason
        if self.established and self.age(now) > 1.0:
            # Said in words, because `why` is what the operator reads on hover
            # and "remembered" versus "seen" changes how much they should trust
            # the marker's position. NJORD §11.4 scores the boat explaining
            # itself, and this is the sentence for it.
            remembered = (
                f"remembered from {self.age(now):.0f} s ago, "
                f"position good to about {self.sigma_m:.1f} m"
            )
            why = f"{why}; {remembered}" if why else remembered
        if why:
            block["why"] = why
        if self.kind in (ObstacleType.CARDINAL,) or self.cardinal.tally:
            block["cardinal"] = self.cardinal.describe()
        return block

    def debug(self, now):
        """Everything about this track, for the trip recording only.

        Deliberately not the same shape as `telemetry`: nothing here is dropped
        to fit a whitelist or rounded to look tidy on a chart, because the
        consumer is `tools/review_trip.py` on a laptop rather than a 4G link.
        The kind votes and the cardinal tally are the two that answer "why did it
        think that", which is the question a recording exists to answer.
        """
        return {
            "id": self.id,
            "kind": self.kind.name,
            "type": self.kind.value,
            "pos": [round(self.pos[0], 3), round(self.pos[1], 3)],
            "sigma_m": round(self.sigma_m, 3),
            "confidence": round(self.confidence, 4),
            "hits": self.hits,
            "misses": self.misses,
            "width_m": round(self.width_m, 3),
            "source": self.source,
            "established": self.established,
            "restored": self.restored,
            "first_seen": round(self.first_seen, 3),
            "last_seen": round(self.last_seen, 3),
            "age_s": round(max(0.0, now - self.last_seen), 3),
            "velocity": [round(self.velocity[0], 3), round(self.velocity[1], 3)],
            "kind_votes": {k.name: round(v, 4) for k, v in self._kind_votes.items()},
            "cardinal_votes": self.cardinal.tally,
            "cardinal_committed": self.cardinal.committed,
            "why": self.reason,
        }

    # -------------------------------------------------------------- the survey

    def to_survey(self, origin):
        """This track as a survey entry, in **lat/lon**. None without an origin.

        Degrees rather than grid metres, and this is not a stylistic choice: the
        grid origin is cached in `/run` (`io_manager/navigation.py`), which is
        tmpfs and is therefore empty after every reboot, and the dashboard has a
        `recentre_origin` button that drops it deliberately. A survey in metres
        would silently become a survey of somewhere else the first time either
        happened - and it would happen overnight, between the two attempts this
        file exists to bridge.
        """
        position = geo.to_global(self.pos[0], self.pos[1], origin)
        if position is None:
            return None
        return {
            "lat": round(position[0], 8),
            "lon": round(position[1], 8),
            "kind": self.kind.name,
            "confidence": round(self.confidence, 4),
            "hits": self.hits,
            "width_m": round(self.width_m, 3),
            "source": self.source,
            "why": self.reason[:160],
            "cardinal_votes": self.cardinal.tally,
            "cardinal_committed": self.cardinal.committed,
            "first_seen": round(self.first_seen, 3),
            "last_seen": round(self.last_seen, 3),
        }

    @classmethod
    def from_survey(cls, entry, origin, config, track_id, now):
        """Rebuild a track from a survey entry. None if it cannot be trusted.

        The restored track is `established` by construction - only established
        tracks are ever written - and its `last_seen` is the *old* wall clock,
        which is exactly right: `refresh` then puts its uncertainty straight at
        the ceiling, so the boat treats it as "a green buoy somewhere in this
        6 m circle" until the lidar collapses it back down to 35 cm.
        """
        try:
            kind = ObstacleType[str(entry["kind"])]
            lat = float(entry["lat"])
            lon = float(entry["lon"])
        except (KeyError, TypeError, ValueError):
            return None
        if abs(lat) > 90.0 or abs(lon) > 180.0:
            return None
        if not is_static(kind) or kind == ObstacleType.UNKNOWN:
            # Belt and braces against a hand-edited file: a vessel has moved.
            return None
        placed = geo.to_world(lat, lon, origin)
        if placed is None:
            return None

        track = cls(
            track_id,
            placed,
            kind,
            max(config.TRACK_ESTABLISH_FLOOR, float(entry.get("confidence") or 0.8)),
            float(entry.get("width_m") or config.BUOY_DIAMETER_M),
            now,
            config,
            str(entry.get("source") or "survey"),
        )
        track.hits = max(
            config.TRACK_ESTABLISH_HITS, int(entry.get("hits") or 0)
        )
        track.first_seen = float(entry.get("first_seen") or now)
        track.last_seen = float(entry.get("last_seen") or now)
        track.established = True
        track.restored = True
        track.surveyed_at = track.last_seen
        track.reason = str(entry.get("why") or "surveyed on an earlier run")[:160]
        # The camera's cardinal poll is restored too, committed included. Working
        # out which cardinal a mark is takes `CARDINAL_VOTES_REQUIRED` agreeing
        # sightings, which is most of a slow pass; throwing that away between
        # attempts would mean paying for it twice.
        for name, count in (entry.get("cardinal_votes") or {}).items():
            for _ in range(int(count)):
                track.cardinal.add(name, 1.0)
        committed = entry.get("cardinal_committed")
        if (
            committed
            and track.cardinal.committed is None
            and str(committed) in track.cardinal.tally
        ):
            # Only ever committed to a direction the restored tally can account
            # for. `CardinalVote.describe()` looks the leader up in that tally,
            # so a hand-edited file naming a direction with no votes behind it
            # would otherwise raise inside `telemetry()` - on the 10 Hz tick.
            track.cardinal.committed = str(committed)
        if track.cardinal.committed:
            resolved = FROM_CARDINAL_NAME.get(track.cardinal.committed)
            if resolved is not None:
                track.kind = resolved
        track.refresh(now, config)
        return track


class WorldModel:
    """Every object the boat currently believes in."""

    def __init__(self, config, survey=None):
        self._config = config
        self._tracks = []
        self._next_id = 1
        self._last_update = None
        self.observations = 0
        # This tick's clusters and how each classified, for the trip recording.
        self.last_measurements = []
        # The grid origin, handed in each tick by `main.py`. Needed only to read
        # and write the survey, which is in lat/lon - see `Track.to_survey`.
        self._origin = None
        self._survey = survey
        self._survey_loaded = False
        self._survey_saved_at = 0.0
        self.restored = 0
        # Places an operator has deleted, as `(pos, kind, expires_at)`. See
        # `forget_track`: without this a deleted phantom is re-created by the
        # very next sweep and the button looks broken.
        self._suppressed = []

    # ---------------------------------------------------------------- origin

    def set_origin(self, origin):
        """Tell the model where grid (0, 0) is. Called every tick; cheap.

        The first time this arrives with a real origin, the stored survey is
        materialised into tracks - not before, because a lat/lon cannot be put on
        the grid without one, and not once at construction, because at
        construction the boat has usually not got a fix yet.
        """
        if not origin:
            return
        moved = self._origin is not None and (
            abs(origin.get("lat", 0.0) - self._origin.get("lat", 0.0)) > 1e-9
            or abs(origin.get("lon", 0.0) - self._origin.get("lon", 0.0)) > 1e-9
        )
        self._origin = dict(origin)
        if moved:
            # `recentre_origin`, or a reboot that re-zeroed the grid. Every
            # track's position is in metres from an origin that no longer
            # exists, so they are all now wrong by however far the origin moved.
            # Re-place the ones that are worth re-placing, from the survey, and
            # drop the rest rather than leave a chart full of offset ghosts.
            self._tracks = []
            self._survey_loaded = False
        if not self._survey_loaded:
            self._load_survey()

    # ---------------------------------------------------------------- ingest

    def observe(self, clusters, boat_xy, heading_deg, now, context="transit"):
        """Fold one sweep's clusters in. `clusters` are BOAT frame.

        Places nothing without a boat pose: a cluster cannot be put in the world
        frame without knowing where the boat was when it was measured, and
        putting it at the origin instead would populate the map with phantoms
        clustered around grid (0, 0). Tracks are still **aged**, though, because
        a track's world-frame position and its age are both perfectly meaningful
        while the boat's own fix is missing - and not ageing meant a fix dropout
        froze every track and its uncertainty exactly where they were.
        """
        self._age(now)
        if boat_xy is None or heading_deg is None or not clusters:
            return

        measurements = []
        # The same list, flattened for the trip recording. Built here rather than
        # re-derived by the recorder because `classify` is the expensive part of
        # the tick and this is the one place it has already been paid for - and
        # because what a recording wants is the classification the world model
        # actually acted on, not a second opinion computed later.
        self.last_measurements = []
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
            self.last_measurements.append(
                {
                    "source": cluster.source,
                    "centre": [
                        round(float(cluster.centre[0]), 3),
                        round(float(cluster.centre[1]), 3),
                    ],
                    "world": [
                        round(boat_xy[0] + east, 3),
                        round(boat_xy[1] + north, 3),
                    ],
                    "width_m": round(float(cluster.width_m), 3),
                    "range_m": round(float(cluster.range_m), 3),
                    "n": int(cluster.n),
                    "kind": kind.name,
                    "confidence": round(float(confidence), 4),
                    "why": why,
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
            if self._is_suppressed(m["pos"], m["kind"], now):
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
        self._maybe_save_survey(now)

    def _associate(self, measurements):
        """`{measurement index: track index}`, by optimal assignment.

        The gate grows with range because a bearing error is an arc: half a
        degree is 4 cm at 5 m and 17 cm at 20 m, so a fixed gate is either too
        tight far away or too loose close in.

        Types that disagree are forbidden from matching outright, *unless* one
        of them is UNKNOWN - a track that has not been coloured yet must be
        allowed to become a green buoy, but a red buoy must never quietly
        become the Otter.

        The gate also grows with the **track's own** position uncertainty, and
        that half is what makes remembering marks work at all. A buoy restored
        from the survey sits wherever it was left, with `sigma_m` at the ceiling;
        the real buoy turns up several metres away; on the old fixed gate those
        are two different objects and the boat ends up with a phantom beside
        every real mark. Widening by `sigma_m` says "this measurement is
        consistent with what I remembered", which is exactly what it is.

        Capped at `TRACK_GATE_MAX_M`, because the failure mode of an unbounded
        gate is worse than a duplicate: two buoys of a gate 5 m apart would
        become mutually eligible and the pair would collapse into one, which is
        the exact scenario `linear_sum_assignment` is here to prevent.
        """
        if not self._tracks or not measurements:
            return {}

        from scipy.optimize import linear_sum_assignment

        track_pos = np.array([t.pos for t in self._tracks], dtype=np.float64)
        meas_pos = np.array([m["pos"] for m in measurements], dtype=np.float64)
        cost = np.linalg.norm(meas_pos[:, None, :] - track_pos[None, :, :], axis=2)

        forbidden = 1e6
        for m_index, m in enumerate(measurements):
            base_gate = (
                self._config.TRACK_GATE_M
                + self._config.TRACK_GATE_PER_M * m["range_m"]
            )
            for t_index, track in enumerate(self._tracks):
                gate = min(
                    self._config.TRACK_GATE_MAX_M, base_gate + track.sigma_m
                )
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
        """The nearest track a detection could plausibly belong to.

        Widened by the track's own uncertainty for the same reason `_associate`
        is: a camera vote on a mark the boat has remembered rather than measured
        has to be able to reach it, and that mark may be several metres from
        where it is being drawn.
        """
        best, best_distance = None, None
        for track in self._tracks:
            d = geo.distance(track.pos, pos)
            gate = min(
                self._config.TRACK_GATE_MAX_M + slack,
                self._config.TRACK_GATE_M + slack + track.sigma_m,
            )
            if d <= gate and (best_distance is None or d < best_distance):
                best, best_distance = track, d
        return best

    # ----------------------------------------------------------------- ageing

    def _age(self, now):
        """Decay, re-estimate uncertainty, and cull. Once per tick, before use.

        The cull is the rule that implements everything the memory promises:

            established     kept regardless of age. Its confidence has a floor
                            (`Track.decay`) so the confidence test cannot reach
                            it either. This is the buoy the boat is sure about,
                            remembered indefinitely - with a position it is
                            steadily less certain of, which is the honest part.
            everything else dropped after `TRACK_DROP_AFTER_S`, exactly as
                            before. The Otter, because it has moved. And the one
                            stray green return that was never seen again,
                            because a single bad read must not outlive the sweep
                            it came from.
        """
        if self._last_update is not None:
            dt = max(0.0, now - self._last_update)
            for track in self._tracks:
                if track.last_seen < now:
                    track.decay(dt, self._config)
        self._last_update = now

        # Every track's uncertainty, for this instant, before any consumer reads
        # it. Done for all of them - including the ones measured this very tick,
        # whose age is ~0 and whose sigma therefore comes out at the floor.
        for track in self._tracks:
            track.refresh(now, self._config)

        self._tracks = [
            track
            for track in self._tracks
            if track.confidence > 0.15
            and (
                track.established
                or track.age(now) < self._config.TRACK_DROP_AFTER_S
            )
        ]
        self._suppressed = [s for s in self._suppressed if s[2] > now]

    # ------------------------------------------------------------- forgetting

    def forget(self, reason="operator cleared the world model"):
        """Drop everything, the stored survey included. `(count, message)`.

        The survey has to go too. Without that, "clear everything" clears the
        screen and the next restart puts it all back - which is not what anyone
        pressing that button means, and is the sort of thing that is discovered
        at the worst possible moment.
        """
        count = len(self._tracks)
        self._tracks = []
        self._suppressed = []
        if self._survey is not None:
            self._survey.clear()
        log.warning("world model cleared (%d track(s)): %s", count, reason)
        return count, f"cleared {count} tracked object(s) and the stored survey"

    def forget_track(self, track_id):
        """Delete one object by id. `(ok, message)`.

        The operator's "that is not a buoy" button. Two things have to happen
        beyond removing it from the list, or it does not stay deleted:

          * the **survey** entry goes, or it returns on the next restart;
          * the **spot** is suppressed for `FORGET_SUPPRESS_S`, or the very next
            sweep re-creates it from the same returns that produced it and the
            button looks broken.

        Suppression is deliberately temporary. If something really is there, it
        should come back - the operator's judgement is better than the sensor's
        for the next half minute, not for ever.
        """
        try:
            wanted = int(track_id)
        except (TypeError, ValueError):
            return False, f"{track_id!r} is not a track id"

        for index, track in enumerate(self._tracks):
            if track.id != wanted:
                continue
            self._tracks.pop(index)
            self._suppressed.append(
                (track.pos, track.kind, time.time() + self._config.FORGET_SUPPRESS_S)
            )
            self._write_survey()
            log.warning(
                "operator deleted %s #%d at (%.1f, %.1f)",
                label(track.kind),
                wanted,
                track.pos[0],
                track.pos[1],
            )
            return True, (
                f"deleted {label(track.kind)} #{wanted}; that spot is ignored "
                f"for {self._config.FORGET_SUPPRESS_S:.0f} s"
            )
        return False, f"no track #{wanted} - it may have already gone"

    def _is_suppressed(self, pos, kind, now):
        """Whether a new track here would be one the operator just deleted."""
        for spot, spot_kind, expires in self._suppressed:
            if expires <= now:
                continue
            if kind != spot_kind and kind != ObstacleType.UNKNOWN:
                continue
            if geo.distance(spot, pos) <= self._config.TRACK_GATE_M:
                return True
        return False

    # ---------------------------------------------------------------- survey

    def _load_survey(self):
        """Materialise the stored survey into tracks. Once per origin."""
        self._survey_loaded = True
        if self._survey is None or self._origin is None:
            return
        now = time.time()
        restored = 0
        for entry in self._survey.entries():
            track = Track.from_survey(
                entry, self._origin, self._config, self._next_id, now
            )
            if track is None:
                continue
            self._tracks.append(track)
            self._next_id += 1
            restored += 1
        self.restored = restored
        if restored:
            log.warning(
                "restored %d surveyed mark(s) from %s - each is good to about "
                "%.1f m until the lidar sees it again",
                restored,
                self._survey.path,
                self._config.TRACK_SIGMA_MAX_M,
            )

    def _maybe_save_survey(self, now):
        if self._survey is None:
            return
        if now - self._survey_saved_at < self._config.SURVEY_SAVE_PERIOD_S:
            return
        self._survey_saved_at = now
        self._write_survey()

    def _write_survey(self):
        """Persist the established static tracks. Best effort, never raises."""
        if self._survey is None or self._origin is None:
            return
        entries = []
        for track in self._tracks:
            if not track.established:
                continue
            entry = track.to_survey(self._origin)
            if entry is not None:
                entries.append(entry)
        self._survey.write(entries, self._origin)

    def save_survey(self):
        """Write the survey out now. Called when a run ends."""
        self._write_survey()

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

    def telemetry(self, limit=40, now=None):
        """The obstacle layer for the dashboard, most confident first."""
        now = now if now is not None else time.time()
        tracks = sorted(
            self.confirmed(), key=lambda t: t.confidence, reverse=True
        )[:limit]
        return [track.telemetry(self._config, now) for track in tracks]

    def debug_tracks(self, now=None):
        """**Every** track, confirmed or not, in full, for the trip recording.

        Not `telemetry`, and the difference is the point of having both. The
        dashboard gets the confirmed ones, trimmed, over 4G. A recording gets all
        of them - because the question asked of a recording afterwards is usually
        "why did the boat not see that buoy", and the answer lives in the tracks
        that never reached `confirmed()`, which the dashboard never showed anyone.
        """
        now = now if now is not None else time.time()
        return [track.debug(now) for track in self._tracks]

    def stats(self, now=None):
        """Counts for the telemetry panel and the recording's per-tick row."""
        now = now if now is not None else time.time()
        established = [t for t in self._tracks if t.established]
        return {
            "tracks": len(self._tracks),
            "confirmed": len(self.confirmed()),
            "established": len(established),
            "remembered": len([t for t in established if t.age(now) > 1.0]),
            "restored": self.restored,
            "suppressed": len(self._suppressed),
            "observations": self.observations,
        }

    def summary(self):
        """One line an operator can read at a glance."""
        counts = {}
        for track in self.confirmed():
            name = label(track.kind)
            counts[name] = counts.get(name, 0) + 1
        if not counts:
            return "nothing tracked"
        out = ", ".join(f"{n}x {name}" for name, n in sorted(counts.items()))
        remembered = self.stats()["remembered"]
        return f"{out} ({remembered} remembered)" if remembered else out
