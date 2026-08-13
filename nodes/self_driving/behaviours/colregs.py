"""Role `avoid`: transit while giving way to a vessel, per COLREG. Njord Task 2.

    "Go from this point to the next and watch out for a boat while you do it."

Lay two waypoints with the role `avoid` - the dashboard calls it **Collision
avoidance** - and this runs between them.

NJORD §9.2: from one GPS point to the next through two gates, at a set speed of
2 knots, while the Otter closes at 2.5 knots. COLREG compliance is explicitly
part of the score, so this is not "avoid the boat" - it is "avoid the boat *the
way a mariner would*", which is a much narrower target.

Two facts about this particular encounter, confirmed 2026-08-12, and they
between them decide everything below
------------------------------------------------------------------------------
**The Otter simulates a vessel that is out of control.** It drives one straight
line whatever we do. COLREG Rule 18(a): a power-driven vessel underway shall keep
out of the way of a vessel not under command. So we are the give-way vessel in
every geometry, Rule 17's stand-on branch cannot arise, and there is never a
moment where the correct action is to hold course into a closing target and trust
the other to move. `config.COLREG_STAND_ON` switches that branch back on if the
Otter ever becomes an ordinary vessel again; it is **off**.

**It can only come from ahead or from starboard**, never from port. Which leaves
exactly two cases to get right, and they want opposite things:

    head-on (Rule 14)          turn to STARBOARD, pass port to port, come back
    crossing from starboard    STOP and let it cross ahead
      (Rule 15, Rule 8(e))

Why stopping, and not a turn, for the crossing case
---------------------------------------------------
Rule 15 says keep clear and avoid crossing ahead; Rule 8(e) says in as many words
that slackening speed or taking all way off is a legitimate way to do it. Turning
is the textbook answer in open water and this is not open water: the leg has a
**5 m gate at each end**, 20-80 m apart, and the boat has to be back on the
centreline to thread the second one. A turn spends room the geometry does not
have. Meanwhile the target is not going to manoeuvre and its track is a straight
line, which makes "wait for it to go past" both exactly predictable and the thing
a mariner actually does in a channel. It is also unmistakable to a jury: a boat
that stops, announces why, and then carries on has visibly *decided* something.

The head-on case cannot use it. There is no stern to wait behind - waiting in
front of a vessel coming straight at you is not a manoeuvre, it is a collision at
a lower speed. So that one turns.

Why an offset track and not a swerve
------------------------------------
Rule 8 wants an alteration "large enough to be readily apparent to another vessel
observing visually", and Rule 8 also warns against a succession of small ones. So
the head-on case runs a track **parallel to the leg**, `COLREG_OFFSET_M` to
starboard, and rejoins when the vessel is astern. One turn out, one turn back,
both large; a clean dogleg on the chart rather than a wobble. A half circle would
also be visible and would cost the gate.

When to act, not just how
-------------------------
Everything keys off the closest point of approach (`geo.closest_point_of_approach`),
which answers "if neither of us does anything, how close do we get, and when".
Two numbers, and both matter: a CPA of 3 m in ninety seconds is not yet a
situation, and a CPA of 3 m in four seconds is an emergency. Acting on range
alone would have the boat manoeuvring around a vessel that is merely nearby and
opening.
"""

from .. import geo
from ..commander import move, stop
from .base import has_arrived, steer_towards
from .transit import Transit

# Rule 13's sector: a vessel more than 112.5 deg abaft the beam is overtaking.
OVERTAKING_DEG = 112.5

# How far past a crossing vessel's stern to aim, for the cases that still aim
# rather than wait. Rule 15's "avoid crossing ahead" made into a number.
ASTERN_MARGIN_M = 8.0

# The stand-on vessel holds - but not forever. Once the projected CPA falls
# below this, Rule 17(b) applies and we take action of our own. Only reachable
# with `COLREG_STAND_ON` on, which it is not for the Njord run.
STAND_ON_ABANDON_M = 4.0


class Colregs(Transit):
    """Transit to the waypoint, giving way to vessels as the rules require."""

    name = "avoid"
    task = "avoid"

    def start(self, ctx):
        super().start(ctx)
        self._committed = None       # the situation we have committed to, if any
        self._committed_at = None
        self._action = None          # "offset" | "wait" | None
        self._action_since = None
        self._acting_on = None       # the track id the action is about


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

        threat = self._threat(ctx)

        # An action, once started, is finished against the vessel that started
        # it - not re-decided from scratch every tick. `_threat` filters on CPA
        # and on TCPA, and both of those stop being alarming the instant the
        # manoeuvre begins to work, so a fresh decision each tick would cancel
        # the manoeuvre halfway and re-enter it a second later. That is exactly
        # the "succession of small alterations" Rule 8 warns about, and on the
        # water it looks like a boat that cannot make up its mind.
        if self._action is not None:
            holding = self._holding(ctx, threat, speed)
            if holding is not None:
                return holding

        if threat is None:
            self._committed = None
            self.note(colreg="clear", vessels=len(ctx.world.vessels()),
                      detected=False, action="none")
            return steer_towards(
                ctx, self._aim(ctx), speed,
                f"running to {ctx.waypoint.name} at task speed, no vessel in the way",
            )

        track, cpa_m, tcpa_s, situation = threat
        self._announce(track, situation, cpa_m, tcpa_s, ctx)

        # Rule 2, and it outranks everything above it. Astern rather than a
        # turn: inside four metres there is not room to turn out of the way, and
        # backing up buys time no alteration of course can.
        if geo.distance(ctx.boat, track.pos) <= ctx.config.COLREG_PANIC_M:
            self.note(action="backing off")
            return move(
                forward=-ctx.config.DOCK_REVERSE_SPEED_MS,
                reason=(
                    f"vessel #{track.id} {geo.distance(ctx.boat, track.pos):.1f} m "
                    "away - backing off (COLREG rule 2)"
                ),
            )

        if situation == "stand-on":
            # Rule 17, and only reachable with `COLREG_STAND_ON` on. Hold course
            # and speed. This is a decision, and it is published as one so nobody
            # watching thinks the boat has not seen the Otter.
            if cpa_m < STAND_ON_ABANDON_M:
                return self._begin(ctx, "offset", track, speed,
                                   "stand-on vessel not keeping clear - taking "
                                   "action (COLREG rule 17b)")
            self.note(action="standing on")
            return steer_towards(
                ctx, self._aim(ctx), speed,
                f"vessel #{track.id} crossing from port - standing on, holding "
                f"course and speed (COLREG rule 17); CPA {cpa_m:.0f} m in "
                f"{tcpa_s:.0f} s",
            )

        detail = f"CPA {cpa_m:.0f} m in {tcpa_s:.0f} s"
        offset_m = ctx.config.COLREG_OFFSET_M

        if situation == "crossing":
            # The whole point of the module docstring. Let it go past.
            return self._begin(
                ctx, "wait", track, speed,
                f"vessel #{track.id} crossing from starboard - giving way by "
                f"waiting for it to pass ahead (COLREG rules 15, 8e); {detail}",
            )

        if situation == "head-on":
            return self._begin(
                ctx, "offset", track, speed,
                f"vessel #{track.id} head-on - altering {offset_m:.0f} m to "
                f"starboard to pass port to port (COLREG rule 14); {detail}",
            )

        if situation == "port-crossing":
            return self._begin(
                ctx, "offset", track, speed,
                f"vessel #{track.id} off the port bow and not under command - "
                f"altering {offset_m:.0f} m to starboard, away from it "
                f"(COLREG rules 18a, 17c); {detail}",
            )

        if situation == "overtaking":
            return self._begin(
                ctx, "offset", track, speed,
                f"overtaking vessel #{track.id} - keeping clear "
                f"(COLREG rule 13); {detail}",
            )

        # Not moving, or moving too slowly to have a course: an obstacle rather
        # than a COLREG situation, and `_give_way` aims round it.
        return self._give_way(
            ctx, track, speed,
            f"vessel #{track.id} in the way - keeping clear; {detail}")

    # ------------------------------------------------------------- the threat

    def _threat(self, ctx):
        """The vessel that needs a decision, or None. `(track, cpa, tcpa, kind)`.

        Only one at a time, and it is the one with the smallest CPA rather than
        the nearest: a vessel 20 m away and closing fast is the problem, not one
        10 m away and opening.
        """
        boat = ctx.boat
        if boat is None:
            return None
        own_velocity = ctx.state.world_velocity
        worst = None
        for track in ctx.world.vessels():
            distance = geo.distance(boat, track.pos)
            if distance > ctx.config.COLREG_DETECT_RANGE_M:
                continue
            tcpa, cpa = geo.closest_point_of_approach(
                boat, own_velocity, track.pos, track.velocity
            )
            # Already past its closest point: the range is opening and nothing
            # needs doing. Without this the boat manoeuvres to avoid a vessel it
            # has just cleared, which is both wrong and alarming to watch.
            if tcpa < 0.0:
                continue
            if tcpa > ctx.config.COLREG_HORIZON_S:
                continue
            if cpa > ctx.config.COLREG_MIN_CPA_M:
                continue
            if worst is None or cpa < worst[1]:
                worst = (track, cpa, tcpa, self._situation(ctx, track))
        return worst

    def _situation(self, ctx, track):
        """Which COLREG case this is, from the geometry alone.

        Sticky once committed: the relative bearing of a crossing vessel drifts
        as the encounter develops, and a boat that reclassifies mid-manoeuvre
        turns one way and then the other, which is exactly the "succession of
        small alterations" Rule 8 warns against.
        """
        if self._committed is not None and ctx.now - self._committed_at < 20.0:
            return self._committed

        bearing = geo.relative_bearing(track.pos, ctx.boat, ctx.heading)
        # The other vessel's own course, from its tracked velocity. Below a
        # walking pace its direction is noise, and a vessel that is not moving
        # is an obstacle rather than a COLREG situation.
        if track.speed < 0.2:
            situation = "obstacle"
        elif abs(bearing) <= ctx.config.COLREG_HEADON_DEG and self._reciprocal(ctx, track):
            situation = "head-on"
        elif abs(bearing) >= OVERTAKING_DEG:
            situation = "overtaking"
        elif bearing > 0.0:
            situation = "crossing"      # on our starboard: rule 15, we give way
        elif ctx.config.COLREG_STAND_ON:
            situation = "stand-on"      # on our port: rule 17, we hold
        else:
            # It should not be on our port at all - the Otter is briefed to come
            # from ahead or starboard. If it is, the most likely cause is a
            # bearing error near the bow, and the safe reading of an out-of-
            # control vessel somewhere off the port bow is the same as any other:
            # keep clear, to starboard, away from it. Rule 17(c) says as much -
            # a vessel taking action for a target on its port side shall not
            # alter to port.
            #
            # It takes the same action as head-on and keeps its own name, because
            # the audit trail should say what the boat actually saw rather than
            # the case it borrowed a manoeuvre from.
            situation = "port-crossing"
        self._committed = situation
        self._committed_at = ctx.now
        return situation

    def _reciprocal(self, ctx, track):
        """Whether the other vessel is coming more or less straight at us."""
        course = geo.bearing_to((0.0, 0.0), track.velocity)
        return abs(geo.angle_diff(course, ctx.heading + 180.0)) <= 40.0

    # ------------------------------------------------------- running an action

    def _begin(self, ctx, action, track, speed, reason):
        """Enter `offset` or `wait` against `track`, and do the first tick of it."""
        if self._action != action or self._acting_on != track.id:
            self._action = action
            self._action_since = ctx.now
            self._acting_on = track.id
        return self._run(ctx, action, track, speed, reason)

    def _holding(self, ctx, threat, speed):
        """Continue the action in progress, or `None` if it is finished.

        Ends on one of three things, and the third is not optional:

          * the vessel we are acting on is gone from the world model;
          * it is drawing clear (`_clear`);
          * or the wait has run past `COLREG_WAIT_MAX_S`, which is the guard
            against a detector that has latched onto a pontoon parking the boat
            on the course for the rest of the attempt.
        """
        track = self._find(ctx, self._acting_on)
        if track is None:
            self.note(action_ended="vessel no longer tracked")
            return self._end()
        if self._clear(ctx, track):
            self.note(action_ended="vessel drawing clear")
            return self._end()

        held = ctx.now - (self._action_since or ctx.now)
        if self._action == "wait" and held > ctx.config.COLREG_WAIT_MAX_S:
            # Loudly, because the operator's 20 s window (NJORD §8.2) starts
            # from noticing, and this is the notice.
            self.note(action_ended="waited too long - giving up and running on",
                      stuck=True, waited_s=round(held, 1))
            return self._end()

        return self._run(ctx, self._action, track, speed,
                         self._why_still(ctx, track, held))

    def _run(self, ctx, action, track, speed, reason):
        """One tick of an action.

        The geometry is re-published every tick even while an action is being
        held, because `_announce` only runs on the deciding path and the panel
        would otherwise show the range and CPA frozen at the moment the boat
        committed - which is the exact moment they stop being interesting and
        start being misleading.
        """
        tcpa, cpa = geo.closest_point_of_approach(
            ctx.boat, ctx.state.world_velocity, track.pos, track.velocity
        )
        self.note(action=action, acting_on=track.id,
                  detected=True,
                  range_m=round(geo.distance(ctx.boat, track.pos), 1),
                  cpa_m=round(cpa, 1), tcpa_s=round(tcpa, 1),
                  vessel_speed=round(track.speed, 2),
                  vessel_bearing_deg=round(
                      geo.relative_bearing(track.pos, ctx.boat, ctx.heading), 1),
                  action_held_s=round(ctx.now - (self._action_since or ctx.now), 1))
        if action == "wait":
            return self._wait(ctx, track, reason)
        return self._offset(ctx, track, speed, reason)

    def _end(self):
        self._action = None
        self._action_since = None
        self._acting_on = None
        self._committed = None
        return None

    def _wait(self, ctx, track, reason):
        """Hold station on the leg at a crawl until the vessel is past.

        Not `stop()`. A hull with no way on has no steering authority at all and
        will lie across the leg in the first puff of wind - which is both a worse
        place to be when the Otter goes by and a mess to recover from, because
        the boat then has to turn back onto the leg before it can resume. Creeping
        at `COLREG_WAIT_SPEED_MS` down the leg keeps the bow where it belongs and
        makes almost no ground.
        """
        return steer_towards(ctx, self._aim(ctx),
                             ctx.config.COLREG_WAIT_SPEED_MS, reason)

    def _offset(self, ctx, track, speed, reason):
        """Run a track parallel to the leg, `COLREG_OFFSET_M` to starboard."""
        aim = self._offset_aim(ctx)
        if aim is None:
            # No leg to be parallel to - a single-waypoint plan, or the boat has
            # no position. Fall back to the committed turn, which is the right
            # default for every case in the book.
            return self._give_way(ctx, track, speed, reason)
        aim, _notes = self._widen(ctx, aim)
        return steer_towards(ctx, aim, speed, reason)

    def _offset_aim(self, ctx):
        """The pure-pursuit point on the offset track, or None if there is no leg."""
        if ctx.leg is None or ctx.boat is None:
            return None
        start, end = ctx.leg
        along = geo.bearing_to(start, end)
        # Perpendicular, to the right of the direction of travel. Offsetting the
        # LEG rather than the boat is what makes this a parallel track: offset
        # from the boat and every tick moves the line the boat is chasing, which
        # curves away indefinitely.
        starboard = along + 90.0
        offset = ctx.config.COLREG_OFFSET_M
        shifted = (geo.offset_point(start, starboard, offset),
                   geo.offset_point(end, starboard, offset))
        return geo.lookahead_point(ctx.boat, shifted[0], shifted[1],
                                   ctx.config.COLREG_OFFSET_LEAD_M)

    def _clear(self, ctx, track):
        """Whether the vessel is drawing clear and the action can end.

        Two conditions and both must hold. A range that has begun to open is
        **not** enough on its own: it is also what a target looks like one second
        before it crosses close ahead, because the CPA is behind us in time while
        the vessel is still in front of us in space. Requiring the bearing to
        have gone abaft the beam as well means the thing has physically gone
        past before the boat resumes.
        """
        if ctx.boat is None:
            return True
        bearing = abs(geo.relative_bearing(track.pos, ctx.boat, ctx.heading))
        if bearing < ctx.config.COLREG_CLEAR_ASTERN_DEG:
            return False
        tcpa, cpa = geo.closest_point_of_approach(
            ctx.boat, ctx.state.world_velocity, track.pos, track.velocity
        )
        return tcpa < 0.0 or cpa > ctx.config.COLREG_MIN_CPA_M

    def _find(self, ctx, track_id):
        for track in ctx.world.vessels():
            if track.id == track_id:
                return track
        return None

    def _why_still(self, ctx, track, held):
        distance = geo.distance(ctx.boat, track.pos)
        if self._action == "wait":
            return (f"holding for vessel #{track.id} to pass ahead - {distance:.0f} m "
                    f"off, waited {held:.0f} s (COLREG rules 15, 8e)")
        return (f"holding {ctx.config.COLREG_OFFSET_M:.0f} m to starboard of the "
                f"leg until vessel #{track.id} is past - {distance:.0f} m off "
                f"(COLREG rules 14, 8)")

    # ---------------------------------------------------- the older aim-astern

    def _give_way(self, ctx, track, speed, reason):
        """Rule 16: early and substantial, to starboard, passing astern.

        Kept for the cases the two headline actions do not cover - a vessel with
        no usable course, and an offset with no leg to be parallel to. The aim
        point is placed astern of the vessel's *predicted* position rather than
        beside its current one, which is what "do not cross ahead" means once
        both boats are moving.
        """
        boat = ctx.boat
        if track.speed >= 0.2:
            # Where it will be when we get there, then a margin further back
            # along its own course - i.e. behind its stern.
            distance = geo.distance(boat, track.pos)
            seconds = distance / max(0.3, ctx.state.speed)
            ahead = track.predicted(min(seconds, ctx.config.COLREG_HORIZON_S))
            course = geo.bearing_to((0.0, 0.0), track.velocity)
            aim = geo.offset_point(ahead, course + 180.0, ASTERN_MARGIN_M)
            self.note(action="passing astern", astern_of=[round(aim[0], 1), round(aim[1], 1)])
        else:
            aim = geo.offset_point(
                boat, ctx.heading + ctx.config.COLREG_TURN_DEG,
                max(8.0, ctx.config.LOOKAHEAD_M),
            )
            self.note(action=f"turning {ctx.config.COLREG_TURN_DEG:.0f} deg to starboard")

        # Extra room on top of the ordinary clearance while a situation is live.
        aim, _notes = self._widen(ctx, aim)
        return steer_towards(ctx, aim, speed, reason)

    def _widen(self, ctx, aim):
        from .base import deconflict

        return deconflict(ctx, aim, extra_clearance=2.0)

    # ------------------------------------------------------ telling the world

    def _announce(self, track, situation, cpa_m, tcpa_s, ctx):
        """Publish the detection, which NJORD §9.2 requires the boat to signal.

        "Once observing the Otter vessel, your ASV shall signal the detection,
        and safely maneuver around the marker vessel." §11.4 says what the jury
        reads: a GUI that shows "which objects are detected and based on this how
        the path is further planned". So these fields are the signal - the
        dashboard raises its banner off `detected`, and the vessel itself is
        already on the chart as a track.

        `detected` goes true the moment there is a threat, which is BEFORE the
        manoeuvre begins, because the rule asks for the signal first.
        """
        self.note(
            colreg=situation,
            detected=True,
            vessel_id=track.id,
            range_m=round(geo.distance(ctx.boat, track.pos), 1),
            cpa_m=round(cpa_m, 1),
            tcpa_s=round(tcpa_s, 1),
            vessel_speed=round(track.speed, 2),
            vessel_bearing_deg=round(
                geo.relative_bearing(track.pos, ctx.boat, ctx.heading), 1),
        )

