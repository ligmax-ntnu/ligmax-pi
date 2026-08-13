"""The four collision-avoidance roles. NJORD §9.2, and nothing else.

    collision_front         see a vessel ahead      -> go round it to STARBOARD
    collision_right         see a vessel to starboard -> STOP and let it pass
    collision_front_backup  the same manoeuvre, no detection, always
    collision_right_backup  the same stop, no detection, always

**The operator picks the case, the boat does not.** Which side the Otter comes
from is known before the attempt starts - it is in the briefing, and there are
only two of them - so it is a waypoint role chosen on the dock, not something
inferred from a monocular bearing thirty seconds before it matters. That is a
deliberate step back from `colregs.py`, which classifies the encounter at
runtime and picks a rule. Classification is the right design for a boat that
meets unknown traffic; it is the wrong one for a scored run where the answer is
already known and a misclassification turns a stop into a turn.

So the sector below is **not** used to decide what to do. It only decides which
bearings count as "the vessel we were told to expect", so that a hull at the quay
120 deg off the bow cannot trigger a manoeuvre laid on for a target ahead.

    collision_front   -45 .. +45 deg     act: offset to starboard, then rejoin
    collision_right   +45 .. +110 deg    act: hold station, let it cross ahead

How the leg works
-----------------
Two waypoints with a straight line between them. **The role goes on the second
one**, the one being driven to - a behaviour runs while the boat is heading for
its waypoint, and `ctx.leg` is the line from the previous waypoint to this one.
Putting it on both ends is harmless and makes the chart read better; putting it
only on the first does nothing.

The backups, and why they exist
-------------------------------
None of this has been on the water. Not the detector, not the world model's new
ability to make a vessel out of a camera box, not the behaviour. A run where the
detector says nothing is a run where the boat drives straight down the line and
into whatever is on it - which scores zero for the task and is the one outcome
worth engineering against when there is no test slot left.

So the backup roles **do not look at the world model at all**. They fire on
geometry: at `COLLISION_BACKUP_LEAD_M` before the midpoint of the two waypoints,
they start the manoeuvre, and they do it larger than the detected version would.
Nothing can stop them and nothing needs to be working for them to run. They are
the answer to "what does the boat do if every camera on it is blind", and the
answer is "the right shape, at roughly the right place, a bit too big".

Exaggerated is deliberate: a scripted manoeuvre is timed off the boat's own
progress rather than off where the Otter actually is, so it has to be wide
enough to be right despite being blind. `COLLISION_BACKUP_OFFSET_M` is half again
`COLLISION_OFFSET_M`, and the scripted wait is a fixed count rather than a look.
"""

from .. import geo
from ..commander import stop
from .base import has_arrived, steer_towards
from .transit import Transit


class Collision(Transit):
    """One leg, one expected encounter, one pre-declared manoeuvre.

    Subclasses set three class attributes and nothing else:

        SECTOR_CONFIG  the name of the `config` entry holding `(low, high)`
                       signed relative bearings in degrees - which bearings count
                       as the vessel we were told to expect. Named rather than
                       inlined so the sector stays overridable from
                       `/etc/ligmax/node.env` like every other number on this
                       boat. Ignored when SCRIPTED.
        ACTION         "offset" (go round to starboard) or "wait" (let it pass).
        SCRIPTED       True to ignore the world model entirely and fire on
                       geometry.
    """

    SECTOR_CONFIG = "COLLISION_FRONT_SECTOR"
    ACTION = "offset"
    SCRIPTED = False

    task = "avoid"          # what the classifier is told the boat is doing

    def sector(self, ctx):
        return getattr(ctx.config, self.SECTOR_CONFIG)

    def start(self, ctx):
        super().start(ctx)
        self._phase = "watching"     # watching | acting | past
        self._acting_since = None
        self._acting_on = None       # track id, or None for a scripted run
        self._rearm_at = None

    # ------------------------------------------------------------------ update

    def update(self, ctx):
        if ctx.target is None:
            return stop("no waypoint to run to")

        arrived, why = has_arrived(ctx)
        if arrived:
            self.done = True
            self.note(arrival=why)
            return stop(f"waypoint {ctx.waypoint.name} reached: {why}")

        # NJORD §9.2 sets the task speed and requires the boat to be at it
        # immediately, so this leg does not use the cruise speed.
        speed = ctx.speed_limit(ctx.config.TASK_SPEED_MS)

        if self._phase == "acting":
            held = ctx.now - (self._acting_since or ctx.now)
            finished, why_done = self._finished(ctx, held)
            if finished:
                self._phase = "past"
                self._rearm_at = ctx.now + ctx.config.COLLISION_REARM_S
                self.note(manoeuvre_ended=why_done)
            else:
                return self._act(ctx, speed, held)

        if self._phase == "past" and self._rearm_at is not None \
                and ctx.now >= self._rearm_at:
            # Re-armed rather than finished for good. A spurious early trigger -
            # a moored hull inside the sector at the start of the leg - would
            # otherwise spend the boat's one manoeuvre before the Otter ever
            # appeared, and the leg would then be driven blind past the thing the
            # task is about. Re-arming costs a second manoeuvre in the worst case;
            # not re-arming costs the run.
            self._phase = "watching"
            self._rearm_at = None
            self._acting_on = None

        trigger, why = self._triggered(ctx)
        if trigger is not None or why:
            self._phase = "acting"
            self._acting_since = ctx.now
            self._acting_on = getattr(trigger, "id", None)
            self.note(manoeuvre_started=why)
            return self._act(ctx, speed, 0.0)

        self._note_watching(ctx)
        return steer_towards(ctx, self._aim(ctx), speed,
                             f"running to {ctx.waypoint.name} at task speed, "
                             + self._watching_words(ctx))

    def _watching_words(self, ctx):
        """The tail of the reason sentence while nothing has fired.

        `_to_trigger` returns None on a plan with no leg to measure along, and a
        format spec on None raises - which would take down the tick, on the one
        role whose entire job is to be the thing that still works.
        """
        if not self.SCRIPTED:
            return "watching for the Otter"
        remaining = self._to_trigger(ctx)
        if remaining is None:
            return ("NO LEG to measure the midpoint along - the scripted "
                    "manoeuvre cannot fire. Lay a waypoint before this one")
        return f"{max(0.0, remaining):.0f} m to the scripted manoeuvre"

    # ----------------------------------------------------------- what starts it

    def _triggered(self, ctx):
        """`(track_or_None, reason)`. An empty reason means nothing has fired."""
        if self.SCRIPTED:
            return self._triggered_by_geometry(ctx)
        return self._triggered_by_sight(ctx)

    def _triggered_by_sight(self, ctx):
        """The nearest tracked vessel inside the sector, within trigger range.

        Deliberately NOT a CPA test. `colregs.py` uses one and it is the better
        instrument when the encounter is unknown, but it needs the other vessel's
        velocity, and a velocity differentiated from a monocular range that is
        itself +-20 % is a number to be careful with. What this needs is much
        cruder: a boat has appeared where we were told one would appear. The
        sector and the range are that test, and both are things the camera
        measures well.
        """
        boat = ctx.boat
        if boat is None:
            return None, ""
        low, high = self.sector(ctx)
        best, best_range = None, None
        for track in ctx.world.vessels():
            distance = geo.distance(boat, track.pos)
            if distance > ctx.config.COLLISION_TRIGGER_RANGE_M:
                continue
            bearing = geo.relative_bearing(track.pos, boat, ctx.heading)
            if not (low <= bearing <= high):
                continue
            if best_range is None or distance < best_range:
                best, best_range = track, distance
        if best is None:
            return None, ""
        bearing = geo.relative_bearing(best.pos, boat, ctx.heading)
        return best, (
            f"vessel #{best.id} seen {best_range:.0f} m off at "
            f"{bearing:+.0f} deg, inside the {low:+.0f}..{high:+.0f} deg sector "
            f"this leg was laid for"
        )

    def _triggered_by_geometry(self, ctx):
        """`COLLISION_BACKUP_LEAD_M` before the midpoint of the two waypoints."""
        remaining = self._to_trigger(ctx)
        if remaining is None:
            return None, ""
        if remaining > 0.0:
            return None, ""
        return None, (
            f"scripted manoeuvre - {ctx.config.COLLISION_BACKUP_LEAD_M:.0f} m "
            "before the midpoint of the leg, no detection required"
        )

    def _to_trigger(self, ctx):
        """Metres still to run before a scripted manoeuvre starts. None if no leg.

        Measured **along the leg**, not as range to a point: the boat may be
        metres off the line and the trigger should not move because of it.
        """
        if ctx.leg is None or ctx.boat is None:
            return None
        start, end = ctx.leg
        length = geo.distance(start, end)
        if length <= 1e-6:
            return None
        _t, along, _cross = geo.project_onto_leg(ctx.boat, start, end)
        return (0.5 * length - ctx.config.COLLISION_BACKUP_LEAD_M) - along

    # ------------------------------------------------------------ what it does

    def _act(self, ctx, speed, held):
        offset = (ctx.config.COLLISION_BACKUP_OFFSET_M if self.SCRIPTED
                  else ctx.config.COLLISION_OFFSET_M)
        self.note(phase="acting", detected=not self.SCRIPTED,
                  colreg=("head-on" if self.ACTION == "offset" else "crossing"),
                  action=("offset" if self.ACTION == "offset" else "wait"),
                  scripted=self.SCRIPTED,
                  acting_on=self._acting_on,
                  action_held_s=round(held, 1))
        if self.ACTION == "wait":
            return self._wait(ctx, held)
        return self._offset(ctx, speed, offset, held)

    def _wait(self, ctx, held):
        """Hold station on the leg at a crawl and let the vessel cross ahead.

        Not `stop()`. A hull with no way on has no steering authority and will
        lie across the leg in the first puff of wind, which is both a worse place
        to be when the Otter goes by and a mess to recover from - the boat then
        has to turn back onto the line before it can resume. Creeping down the leg
        keeps the bow where it belongs and makes almost no ground.
        """
        reason = (f"holding on the line to let the vessel pass ahead - "
                  f"{held:.0f} s (COLREG rules 15, 8e)")
        if self.SCRIPTED:
            reason = (f"scripted hold, {held:.0f} of "
                      f"{ctx.config.COLLISION_BACKUP_WAIT_S:.0f} s - letting the "
                      "vessel cross ahead")
        return steer_towards(ctx, self._aim(ctx),
                             ctx.config.COLLISION_WAIT_SPEED_MS, reason)

    def _offset(self, ctx, speed, offset_m, held):
        """Run a track parallel to the leg, `offset_m` to starboard."""
        aim = self._offset_aim(ctx, offset_m)
        if aim is None:
            # No leg to be parallel to - a single-waypoint plan. A committed turn
            # to starboard is the right default for every case in the book, and
            # it is better than driving on.
            aim = geo.offset_point(ctx.boat, ctx.heading + ctx.config.COLREG_TURN_DEG,
                                   max(8.0, ctx.config.LOOKAHEAD_M))
            self.note(offset_source="no leg - committed turn to starboard")
        reason = (f"{offset_m:.0f} m to starboard of the line, going round the "
                  f"vessel and rejoining ({held:.0f} s)")
        if self.SCRIPTED:
            reason = (f"scripted {offset_m:.0f} m alteration to starboard, "
                      f"rejoining after the midpoint ({held:.0f} s)")
        return steer_towards(ctx, aim, speed, reason)

    def _offset_aim(self, ctx, offset_m):
        """The pure-pursuit point on the offset track, or None if there is no leg.

        The **leg** is offset, not the boat. Offset from the boat and every tick
        moves the line it is chasing, so the track curves away indefinitely
        instead of running parallel.
        """
        if ctx.leg is None or ctx.boat is None:
            return None
        start, end = ctx.leg
        # +90 deg on the leg's own bearing is to starboard of the direction of
        # travel - the same sign convention `project_onto_leg` and
        # `relative_bearing` use, where positive is to the boat's right.
        starboard = geo.bearing_to(start, end) + 90.0
        return geo.lookahead_point(
            ctx.boat,
            geo.offset_point(start, starboard, offset_m),
            geo.offset_point(end, starboard, offset_m),
            ctx.config.COLLISION_OFFSET_LEAD_M,
        )

    # ------------------------------------------------------------ what ends it

    def _finished(self, ctx, held):
        """`(finished, why)`. Every path out of a manoeuvre is here."""
        if self.SCRIPTED:
            return self._finished_scripted(ctx, held)
        return self._finished_by_sight(ctx, held)

    def _finished_scripted(self, ctx, held):
        if self.ACTION == "wait":
            if held >= ctx.config.COLLISION_BACKUP_WAIT_S:
                return True, f"scripted hold of {held:.0f} s complete"
            return False, ""
        remaining = self._to_trigger(ctx)
        if remaining is None:
            # No leg: fall back to a time box, because a scripted manoeuvre with
            # no geometry to end it would otherwise run to the end of the leg.
            if held >= ctx.config.COLLISION_OFFSET_MAX_S:
                return True, f"scripted alteration timed out after {held:.0f} s"
            return False, ""
        run_past = -remaining - ctx.config.COLLISION_BACKUP_LEAD_M
        if run_past >= ctx.config.COLLISION_BACKUP_RUN_M:
            return True, (f"scripted alteration complete - "
                          f"{run_past:.0f} m past the midpoint, rejoining")
        return False, ""

    def _finished_by_sight(self, ctx, held):
        """The vessel has gone past, or we have waited long enough.

        Two conditions for "gone past", and both must hold. A range that has
        begun to open is **not** enough on its own: it is also what a target
        looks like one second before it crosses close ahead, because the closest
        point of approach is behind us in time while the vessel is still in front
        of us in space. Requiring the bearing to have gone abaft the beam as well
        means the thing has physically gone by.
        """
        limit = (ctx.config.COLLISION_WAIT_MAX_S if self.ACTION == "wait"
                 else ctx.config.COLLISION_OFFSET_MAX_S)
        if held >= limit:
            # Loudly: NJORD §8.2's 20 s operator window starts from noticing, and
            # this is the notice.
            self.note(stuck=True)
            return True, (f"gave up after {held:.0f} s - the vessel is still "
                          "there or the detector has latched onto something")

        track = self._find(ctx, self._acting_on)
        if track is None:
            return True, "vessel no longer tracked"
        if ctx.boat is None:
            return True, "no position"

        bearing = abs(geo.relative_bearing(track.pos, ctx.boat, ctx.heading))
        if bearing < ctx.config.COLLISION_CLEAR_ASTERN_DEG:
            return False, ""
        tcpa, cpa = geo.closest_point_of_approach(
            ctx.boat, ctx.state.world_velocity, track.pos, track.velocity
        )
        if tcpa < 0.0 or cpa > ctx.config.COLREG_MIN_CPA_M:
            return True, f"vessel is abaft the beam at {bearing:.0f} deg and opening"
        return False, ""

    def _find(self, ctx, track_id):
        if track_id is None:
            return None
        for track in ctx.world.vessels():
            if track.id == track_id:
                return track
        return None

    # ------------------------------------------------------------- telemetry

    def _note_watching(self, ctx):
        """What the panel shows while nothing has fired.

        `detected` is published FALSE rather than omitted. NJORD §9.2 wants the
        boat to signal a detection, and a field that only ever appears when
        something is seen cannot distinguish "watching, nothing there" from "this
        leg is not a collision-avoidance leg at all" - which are very different
        things to a jury reading the screen.
        """
        fields = {
            "phase": "watching",
            "action": "none",
            "scripted": self.SCRIPTED,
            "detected": False,
        }
        if self.SCRIPTED:
            remaining = self._to_trigger(ctx)
            fields["to_manoeuvre_m"] = (None if remaining is None
                                        else round(max(0.0, remaining), 1))
        else:
            fields["sector_deg"] = list(self.sector(ctx))
            fields["vessels"] = len(ctx.world.vessels())
        self.note(**fields)


class CollisionFront(Collision):
    """Otter from ahead: go round it to starboard and rejoin the line."""

    name = "collision_front"
    SECTOR_CONFIG = "COLLISION_FRONT_SECTOR"
    ACTION = "offset"
    SCRIPTED = False


class CollisionRight(Collision):
    """Otter from starboard: stop on the line and let it cross ahead."""

    name = "collision_right"
    SECTOR_CONFIG = "COLLISION_RIGHT_SECTOR"
    ACTION = "wait"
    SCRIPTED = False


class CollisionFrontBackup(Collision):
    """The starboard alteration, larger, on geometry alone. No detection."""

    name = "collision_front_backup"
    ACTION = "offset"
    SCRIPTED = True


class CollisionRightBackup(Collision):
    """The stop, longer, on geometry alone. No detection."""

    name = "collision_right_backup"
    ACTION = "wait"
    SCRIPTED = True
