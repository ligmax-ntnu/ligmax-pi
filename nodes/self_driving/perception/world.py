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
    confirmed    `TRACK_CONFIRM_HITS` sightings - `MARK_CONFIRM_HITS` for a mark.
                 Steered around. This is what stops the boat manoeuvring for one
                 wave crest.
    established  permanent. Never dropped by time or by confidence, and written to
                 the survey file so it survives a restart and the gap between two
                 attempts.

**A mark reaches the third tier on its second sighting** (`MARK_ESTABLISH_HITS`) -
the same instant it becomes steerable. Once the boat has seen a buoy twice it
knows about that buoy for the rest of the run, whether or not it ever sees it
again. That is a change from the old rule, which wanted twelve sightings spread
over two seconds at 0.8 confidence, and `Track._maybe_establish` sets out both
sides of the trade in full.

LAND and anything else static still has to earn permanence the slow way, and there
the span requirement is what does the work: a burst of returns off a single wave
crest can reach twelve hits inside 300 ms and cannot reach them across two seconds.

And **only static types are ever established**, which is what keeps the Otter out
of it: a vessel is the one object on the course guaranteed to have moved since
the boat last saw it, and remembering where it was is worse than useless.

What a permanent mark still is not
----------------------------------
Certain of where it is. `sigma_m` goes on growing while the mark is unseen, and
every consumer adds it to that mark's clearance, so a buoy the boat is remembering
from two minutes ago claims metres more water than the same buoy in view. "It is
there for ever" and "it is exactly there" are different claims and only the first
one is being made.

And not beyond the operator's reach: `forget_track` removes a mark from the model
and from the survey together, which is the answer to a phantom that has made
itself permanent.

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
    MARK_TYPES,
    ObstacleType,
    clearance_for,
    is_static,
    label,
)
from .classify import CardinalVote, classify, policy_for

#: Types that mean "nothing has been named here" and are therefore allowed to
#: associate with each other and to become anything later. UNKNOWN is "no camera
#: covered this"; WATER is "a camera did, and it is the sea". A cluster can slide
#: between the two as the boat's own motion moves it in and out of a lens's cone,
#: and treating that as two different objects would put a duplicate track beside
#: every real one along the edge of the camera coverage.
UNNAMED = frozenset({ObstacleType.UNKNOWN, ObstacleType.WATER})

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

        **A mark is permanent as soon as it has been seen twice.** That is a
        deliberate override of the three tests below and it is worth being explicit
        about what was traded for it.

        The old rule wanted twelve sightings spread over two seconds at 0.8
        confidence before it would remember anything, and the reasoning was sound
        in the abstract: a stray coloured return remembered for ever is worse than
        no memory at all. What it missed is that a Njord mark is very often *not*
        available for twelve sweeps at close range - the boat passes it, the mark
        goes out of the lidar's plane on a swell, the camera stops covering that
        bearing - and a mark that was seen properly for a second and then dropped
        from the model is the failure that actually costs the run. The boat arrives
        at the same gate a minute later with nothing in memory.

        So marks are kept from the second sighting (`MARK_CONFIRM_HITS`, which is
        also what `WorldModel.confirmed` uses, so a mark becomes steerable and
        permanent in the same instant). Two rather than one because one sweep is a
        measurement and two is the cheapest thing that can be called agreement:
        sea foam is not signal red in the same spot twice running.

        What still guards against a phantom, and none of it was given up:

          * `sigma_m` keeps growing while the mark is unseen, so a remembered mark
            claims more water and is approached more cautiously than a measured one
            (`refresh`, and `avoid_radius`);
          * the operator's "that is not a buoy" button removes it from the model
            AND from the survey file (`WorldModel.forget_track`);
          * only STATIC types qualify at all, so the Otter is never remembered
            where it was.

        LAND and anything else static still has to earn permanence the old way -
        the shore is not what the memory is for, and it is the thing most likely to
        fill the survey file (`SURVEY_MAX_TRACKS`).
        """
        if self.established:
            return
        if not is_static(self.kind) or self.kind == ObstacleType.UNKNOWN:
            return
        if self.kind in MARK_TYPES:
            if self.hits >= max(1, config.MARK_ESTABLISH_HITS):
                self.established = True
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

        The floor can only ever stop confidence falling - never raise it. A mark
        established on two sightings arrives here at about 0.4, and lifting it to
        0.55 the first time it went out of sight would have the boat grow *more*
        sure of a mark by not looking at it, on the panel and in the recording
        both. It keeps whatever it earned, and keeps it indefinitely.
        """
        floor = (
            min(config.TRACK_ESTABLISH_FLOOR, self.confidence)
            if self.established
            else 0.0
        )
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

        # What this task is looking for. One object per sweep rather than per
        # cluster, since it is a pure function of the context string.
        policy = policy_for(context, self._config)

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
            # Whether this task wants it remembered at all. Marks and everything
            # mark-sized always are; wide clutter only close in. See
            # `classify.TaskPolicy.tracks` - and note that a dropped measurement is
            # still written to the recording below, because "why did the boat not
            # see that" is the question a trip file exists to answer, and a silent
            # filter is how that question stops being answerable.
            keep, dropped_why = policy.tracks(kind, cluster.range_m, cluster.width_m)
            if keep:
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
            entry = {
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
            if not keep:
                entry["dropped"] = dropped_why
                entry["task"] = policy.name
            self.last_measurements.append(entry)

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
                    and m["kind"] not in UNNAMED
                    and track.kind not in UNNAMED
                )
                if incompatible or cost[m_index, t_index] > gate:
                    cost[m_index, t_index] = forbidden

        rows, cols = linear_sum_assignment(cost)
        return {
            int(r): int(c) for r, c in zip(rows, cols) if cost[r, c] < forbidden
        }

    def absorb_detections(self, detections, boat_xy, heading_deg, now):
        """Fold in the Jetson's camera detections. Refines, and for ONE class
        creates.

        For marks a detection never creates a track. The buoy detector is weak,
        its range from apparent size degrades as the square of distance, and a
        phantom buoy on the chart is worse than a missing one - so its job here
        is to answer questions the lidar posed, not to pose new ones.

        It answers two:

          * **which cardinal** a yellow mark is (the topmark - no lidar can see
            it), through `Track.note_cardinal`;
          * a weak vote on red vs green, for a mark the colour path left
            UNKNOWN because no camera happened to cover it.

        **And since 2026-08-12 it may create a VESSEL, and only a vessel.** That
        is not a loosening of the rule above, it is the rule meeting a boat with
        no lidars: class 3 comes from a separate model (`sender.py
        --vessel-engine`) with its own measured operating point, and if the
        camera may not create the track then `colregs.py` has no vessel to give
        way to and NJORD 9.2 cannot run at all. It is behind
        `CAMERA_CREATES_VESSELS` so that a boat with a working lidar can go back
        to the stricter rule with one environment variable. See the block below.

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
            # NOTE: `track is None` is no longer an early exit. It used to be,
            # and it was the mechanism of "a detection never creates a track" -
            # the vessel branch below is the single exception, and it needs to
            # see the None. Every other class still gives up on it, a few lines
            # further down.

            kind = FROM_DETECTOR_CLASS.get(det.get("cls"))
            if kind is None:
                continue
            confidence = float(det.get("conf") or 0.0)

            # ---- the one exception to "a detection never creates a track".
            #
            # A vessel, and only a vessel, and only when the config says so. The
            # rule above exists because the buoy detector is weak and a phantom
            # buoy on the chart is worse than a missing one - the lidar poses the
            # questions and the camera answers them. **Both lidars are dead**
            # (2026-08-12), so for the Otter there is no question to answer:
            # either the camera creates the track or NJORD 9.2 does not run at
            # all.
            #
            # Three things keep it honest, and none of them is optional:
            #
            #   * it is a SEPARATE model with its own measured operating point
            #     (`ligmax-ai/vessel/`) rather than the buoy detector guessing;
            #   * `VESSEL_MIN_CONF` is a much higher bar than the 0.4 the colour
            #     vote below uses, because this one moves the boat;
            #   * `TRACK_CONFIRM_HITS` still applies, so a single frame's
            #     detection is on the chart and is NOT yet steered around.
            #
            # And it is deliberately narrow: nothing here lets a camera invent a
            # buoy, a cardinal or a piece of land.
            if kind == ObstacleType.BOAT:
                if not self._config.CAMERA_CREATES_VESSELS:
                    continue
                if confidence < self._config.VESSEL_MIN_CONF:
                    continue
                if track is not None and track.kind in (ObstacleType.BOAT,) + UNNAMED:
                    track.observe(
                        pos, ObstacleType.BOAT, confidence, track.width_m, now,
                        self._config, f"camera sees a vessel at {confidence:.0%}",
                    )
                elif track is None and not self._is_suppressed(
                        pos, ObstacleType.BOAT, now):
                    # `width_m` from the Otter's own beam rather than from the
                    # box: the detection's width is in pixels and the metric
                    # figure the edge computed IS this constant (it is what the
                    # range was derived from), so measuring it back off the
                    # detection would be circular.
                    self._tracks.append(
                        Track(
                            self._next_id, pos, ObstacleType.BOAT, confidence,
                            self._config.OTTER_BEAM_M, now, self._config, "camera",
                        )
                    )
                    self._tracks[-1].reason = (
                        f"camera sees a vessel at {confidence:.0%} (no lidar)"
                    )
                    self._next_id += 1
                continue

            # ---- the second exception: a red or green MARK, for the surprise task.
            #
            # Same shape as the vessel branch above and the same three guards, and
            # it is aimed straight at the reason the rule exists, so the reasoning
            # is worth having here rather than only in `config.py`:
            #
            #   * OFF unless an operator switched it on for this run
            #     (`CAMERA_CREATES_MARKS`, `set_mark_source`), where the vessel
            #     exception is on by default. Every other course is better served
            #     by the strict rule;
            #   * only the sources named in `MARK_SOURCES`, so "colour" - a hue test
            #     below the lidar line, the same test that colours the lidar - can be
            #     trusted while the YOLO is not, which is the actual state of this
            #     boat;
            #   * `TRACK_CONFIRM_HITS` still applies, so one frame draws a mark on
            #     the chart and does NOT yet shift the corridor.
            #
            # What makes this bearable where a phantom vessel would not be: a mark is
            # a constraint on the *corridor*, not a stop. `buoys._lateral` shifts the
            # aim line a couple of metres and drives on. And range is bounded
            # (`MARK_MAX_RANGE_M`) because it comes from apparent size and degrades
            # as its square - past that the mark's own sigma is wider than the
            # clearance the rule is trying to enforce, and it would shove the line
            # sideways for a buoy nobody can properly see.
            if kind in (ObstacleType.RED, ObstacleType.GREEN) and self._create_mark(
                det, pos, kind, confidence, track, now, boat_xy
            ):
                continue

            # Everything below refines an existing track and nothing else, so a
            # detection that matched nothing is finished with here.
            if track is None:
                continue

            if kind == ObstacleType.CARDINAL:
                track.note_cardinal(det.get("card"), float(det.get("card_conf") or 0.0))
                if track.kind in UNNAMED:
                    track.kind = ObstacleType.CARDINAL
                continue

            # A deliberately weak vote: the detector gets a say only where the
            # coloured lidar had none - which includes a track the colour path read
            # as WATER, and that is not a technicality. The Jetson colours each
            # return from the nearest buffered frame and most of them are stale
            # (`config.COLOUR_AGE_FRESH_MS`), so a mark 8 m off can easily be
            # painted from a frame that was looking at the water beside it. A track
            # the lidar called water is exactly a track the camera should be
            # allowed to correct; refusing would have made naming water a way of
            # silencing the detector.
            if track.kind in UNNAMED and confidence > 0.4:
                track.observe(
                    track.pos, kind, confidence * 0.5, track.width_m, now,
                    self._config,
                    f"camera says {label(kind)} at {confidence:.0%} (no lidar colour)",
                )

    def _create_mark(self, det, pos, kind, confidence, track, now, boat_xy):
        """A camera-created red or green mark. True if this detection is finished.

        True means "handled, do not fall through" - either a track was created or
        an existing one was refined as a mark. False leaves the detection to the
        ordinary weak-vote path below, which is what keeps the strict rule's
        behaviour byte-for-byte identical while the switch is off.
        """
        config = self._config
        if not config.CAMERA_CREATES_MARKS:
            return False
        source = str(det.get("src") or "yolo").strip().lower()
        if source not in config.MARK_SOURCES:
            return False

        floor = (
            config.MARK_MIN_CONF_COLOUR
            if source == "colour"
            else config.MARK_MIN_CONF_YOLO
        )
        if confidence < floor:
            return False

        # Range, and it is the reason most detections stop here. `_detection_position`
        # has already turned bearing plus range into a world point, so the check is on
        # how far that point is from the boat rather than on the detection's own
        # `range` block - which is null on a colour blob that fell outside the
        # detector's crop.
        if boat_xy is not None:
            reach = math.dist(boat_xy, pos)
            if reach > config.MARK_MAX_RANGE_M:
                return False

        if track is not None:
            # An existing track: name it, do not duplicate it. This is a stronger
            # statement than the weak vote below - the vote only speaks where the
            # lidar's colour path had nothing to say, whereas a source the operator
            # has switched on is allowed to name a track outright. It still may not
            # overrule a mark the lidar already coloured differently: a red track
            # that the camera thinks is green is a disagreement worth keeping, not a
            # value worth overwriting.
            if track.kind in UNNAMED or track.kind == kind:
                track.observe(
                    pos, kind, confidence, track.width_m, now, config,
                    f"{source} says {label(kind)} at {confidence:.0%}",
                )
                return True
            return False

        if self._is_suppressed(pos, kind, now):
            return False

        self._tracks.append(
            Track(
                self._next_id, pos, kind, confidence,
                config.MARK_DIAMETER_M, now, config, source,
            )
        )
        self._tracks[-1].reason = (
            f"{source} sees {label(kind)} at {confidence:.0%} (no lidar)"
        )
        self._next_id += 1
        return True

    def _note_no_rig_bearing(self):
        """Say once a minute that every camera-only detection is being dropped.

        Rate-limited rather than per detection: this fires for every box in every
        frame, so at 10 fps it would be the only thing in the journal - and a log
        nobody can read is how the one line that mattered gets missed. Once a minute
        is often enough to be seen and rare enough to leave the rest legible.
        """
        now = time.time()
        if now - getattr(self, "_no_rig_at", 0.0) < 60.0:
            return
        self._no_rig_at = now
        log.warning(
            "camera detections carry no bearing_rig_deg, so none of them can be "
            "placed - the Jetson needs rig.json and a build from 2026-08-12 or "
            "later. With no lidar this means NO marks and NO vessel at all"
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
            # **`bearing_rig_deg`, not `bearing_deg`.** The plain one is measured
            # from that camera's OPTICAL AXIS and the two lenses are yawed +-75 deg
            # (`rig.json`), so using it here put a mark seen dead ahead of camera 0
            # seventy-five degrees from where it actually was. It cost nothing while
            # a detection could only refine a lidar track that already had a
            # position - the refine path uses `track.pos` and the match gate is
            # metres wide - and it became the whole answer the moment both lidars
            # died and the camera started creating the track. See
            # `ligmax-edge/estimate.py`, which computes the rig-frame angle from the
            # ray and the mount.
            bearing = det.get("bearing_rig_deg")
            if bearing is None:
                # An older Jetson build, or one running without `rig.json`. Refuse
                # rather than fall back to the camera-frame angle: a mark 75 deg
                # from where it is will be dutifully avoided, reported, and drawn on
                # the operator's chart, and nothing about it looks wrong. A missing
                # mark is recoverable and a confidently misplaced one is not.
                self._note_no_rig_bearing()
                return None
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

            established     kept, full stop - neither age nor confidence can
                            remove it, and for a mark that state is reached on the
                            second sighting (`Track._maybe_establish`). This is
                            the buoy the boat has seen, remembered for the rest of
                            the run and written to the survey for the next one -
                            with a position it is steadily less certain of, which
                            is the honest part. The only things that remove it are
                            the operator's delete button and a new grid origin.
            everything else dropped after `TRACK_DROP_AFTER_S`, exactly as
                            before. The Otter, because it has moved. Unnamed
                            clutter, because the sweep that made it is gone. And
                            the single stray return that never came back, because
                            one bad read must not outlive the sweep it came from.
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
            if track.established
            or (
                track.confidence > 0.15
                and track.age(now) < self._config.TRACK_DROP_AFTER_S
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
            # An unnamed measurement at a deleted spot is suppressed as well as a
            # matching one, and WATER counts as unnamed here for a concrete reason:
            # the camera is allowed to upgrade a water track (`absorb_detections`),
            # so letting one form on a spot the operator has just cleared is a route
            # by which the deleted phantom comes straight back.
            if kind != spot_kind and kind not in UNNAMED:
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
        """Persist the established static tracks. Best effort, never raises.

        Best-first, because `survey.write` truncates at `SURVEY_MAX_TRACKS` and
        something has to decide which 200 survive. Since a mark now earns
        permanence on its second sighting, the file fills with a wider spread of
        quality than it used to, and truncating in whatever order the tracks
        happen to sit in the list would let a mark seen twice at 8 m displace one
        the boat spent ten seconds alongside. Ranked by sightings times
        confidence, the cap drops the weakest instead of the newest.
        """
        if self._survey is None or self._origin is None:
            return
        ranked = sorted(
            (t for t in self._tracks if t.established),
            key=lambda t: t.hits * max(0.0, t.confidence),
            reverse=True,
        )
        entries = []
        for track in ranked:
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

        A **mark** confirms a sweep sooner (`MARK_CONFIRM_HITS`), because the thing
        the count is defending against has already been answered in its case: a
        cluster that came back in the same place with the same paint on it is not
        foam, and foam is not signal red twice running. The saving is a tenth of a
        second, and it is spent where it is worth most - a mark is only useful
        while there is still room to choose a side of it.
        """
        return [track for track in self._tracks if track.hits >= self._hits_for(track)]

    def _hits_for(self, track):
        """How many sightings this track needs before the boat acts on it."""
        if track.kind in MARK_TYPES:
            return max(1, self._config.MARK_CONFIRM_HITS)
        return self._config.TRACK_CONFIRM_HITS

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
        """The obstacle layer for the dashboard, most confident first.

        **Water never reaches the chart.** Spray, wave crests and the shadow under
        a jetty are all real returns and none of them is an object worth drawing;
        a dozen of them a minute is what turns a chart the operator is meant to
        read at a glance into one they scroll past. They are still tracked while
        they are close enough to matter, still avoided, and still written to the
        trip recording by `debug_tracks` - so "why did it swerve there" stays
        answerable afterwards even though nothing was drawn at the time.
        """
        now = now if now is not None else time.time()
        drawable = [
            track for track in self.confirmed()
            if track.kind != ObstacleType.WATER
        ]
        tracks = sorted(drawable, key=lambda t: t.confidence, reverse=True)[:limit]
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
        """One line an operator can read at a glance.

        Water is left out for the same reason it is left off the chart: "3x water"
        is not something anybody needed to be told.
        """
        counts = {}
        for track in self.confirmed():
            if track.kind == ObstacleType.WATER:
                continue
            name = label(track.kind)
            counts[name] = counts.get(name, 0) + 1
        if not counts:
            return "nothing tracked"
        out = ", ".join(f"{n}x {name}" for name, n in sorted(counts.items()))
        remembered = self.stats()["remembered"]
        return f"{out} ({remembered} remembered)" if remembered else out
