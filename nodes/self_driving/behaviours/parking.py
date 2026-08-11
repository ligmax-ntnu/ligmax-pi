"""Roles `park` and `park_parallel`: park in the middle of three lines and wait.

    park            drive into the space, sit in the middle for 10 s, reverse out
    park_parallel   come alongside into it, sit in the middle for 10 s, go on

Both do the same three things and differ only in dimensions, in which way the
bow points while sitting there, and in which way they leave.

One lidar, forward only, and what that costs
--------------------------------------------
**The boat has one working lidar and it looks forward.** The aft unit is broken
(2026-08-11) and parking never used it anyway - a flipped
`LIGMAX_AFT_LIDAR_ANGLE_DIR` gives a plausible, complete and *mirrored* world
astern, and a mirrored parking space is a space on the wrong side of the boat.
Either way there is nothing behind the boat now, and one phase of one manoeuvre
pays for it:

**an alongside park cannot see the space it is sitting in.** It comes down the
normal bow-first, reaches the dot, then rotates 90 degrees - and the lone line,
the closed end that every depth measurement is taken from, ends up abeam and out
of view. The two side walls are then ahead and astern of the bow instead of
either side of it, at a range where the hull's own returns live. So from the turn
onwards the boat has to **remember the space** rather than measure it: it holds
the dimensions and the pose it last measured properly, from the dot, square, with
all three lines in view, and it dead-reckons the middle of that against its own
GNSS. `_measure` latches for exactly that reason, `parking.source` on the panel
says `remembered` while it is doing it, and the hold is being flown on memory -
which is worth knowing before the countdown restarts and somebody wonders why.

Bow-in parking is unaffected: it sits facing the closed end, so it keeps all
three lines and keeps measuring.

What this behaviour looks at, and what it deliberately does not
--------------------------------------------------------------
**Lines. Only lines.** The parking space is three sides of a rectangle whose
corners do not meet (`perception/parking.py`), the target is the middle of it, and
none of that involves knowing what anything *is*. So this behaviour reads the
front lidar's sweeps and fits edges to them, and it never consults the world
model: no buoy colours, no track identities, no obstacle clearances, no avoidance
nudge.

That is a decision and not an oversight. During a parking run the boat is
deliberately driving to within tens of centimetres of structures it can see
perfectly well, and every mechanism that exists to keep it away from things it
has recognised is, in here, a mechanism for refusing to park:

  * `deconflict()` widens its berth around anything confirmed, which inside a 2 m
    space means steering into a wall to avoid a buoy;
  * `emergency_stop_needed()` fires on anything within about 1.5 m of the bow,
    which inside a 2 m space is the back of the space, every tick;
  * a remembered mark drifting into the mouth is not a reason to abandon the
    task, and it is exactly what the tracker is built to keep believing in.

So the whole of that is off in here, and what remains is not nothing: the pilot's
own checks run first and are untouched (E-stop, somebody else driving, comms
loss, a stale fix - `pilot.py`), every speed is the docking creep rather than a
task speed, and the operator has both twenty seconds and a physical switch. The
boat also cannot be *asked* to park by anything other than a waypoint role
somebody typed, which is the real gate.

Always normal to the space, and the angle is the type's
------------------------------------------------------
Three rules govern the whole manoeuvre, and every phase below is a consequence of
one of them.

**The boat travels along the space's normal, bow first.** Never diagonally, never
at an angle, and never sideways. In and out are the same line, and both types use
it: a space barely wider than the hull is entered on that line or not at all.
`_approach_move` does the work in the *space's* axes and rotates into the hull's
only at the last moment, which is what makes one piece of code correct for both
types without a branch.

**The sideways thruster holds position. It does not move the boat anywhere.**
This hull is a trimaran: pushed sideways it is presenting three hulls broadside to
the water, and the drag is nothing like the drag it was designed around. So the
third thruster is a *trim* - it holds a line against a light set, and it holds the
dot during the hold and the turn - and it is never asked to translate the boat
across a space. An offset bigger than a trim can hold is therefore fixed the only
other way there is: **back out and run the approach again**, where there is room
to turn and the main thrusters do the work. `PARK_TRIM_LATERAL_MS` is the line
between the two jobs.

**The boat's angle is the parking type's angle, and it is enforced.** Bow-in sits
square to the closed end. Alongside sits parallel to it - and gets there by
**rotating 90 degrees once it is inside**, not by crabbing in sideways from
outside. The hold clock requires the angle as well as the position, because a boat
on the right spot at thirty degrees to the walls is not parked, it is wedged.

The phases
----------
    SEARCH   drive to the operator's waypoint and fit lines until three of them
             make a space. The waypoint says where to look; the lidar says where
             the space is.
    PROBE    nothing in view from the waypoint after `PARK_SEARCH_TIMEOUT_S`, so
             creep on a fixed bearing (120 degrees at Havet arena - east and a
             little south, in towards land) for up to `PARK_PROBE_M`, still
             fitting lines. A forward-looking lidar sitting outside the docks
             cannot see a berth a few metres further in, so sitting still is not
             a plan. Its own phase because `pilot._watch_progress` has to exempt
             it: this is the one phase that deliberately drives *away* from the
             waypoint.
    ALIGN    get onto the space's centreline one standoff out, and square up.
             **A 2 m space entered crooked is a collision**, and the only cheap
             moment to be straight is before committing.
    ENTER    creep down the normal to the dot, bow first, holding the approach
             heading. Off the centreline it stops advancing and crabs back onto it.
    TURN     on the dot, rotate to the parking type's angle. A no-op for bow-in,
             which is already at it; 90 degrees for alongside.
    HOLD     sit on the dot, at that angle, for the required time. The countdown
             is published for the operator's chart, which draws it next to the boat.
    EXIT     straight back out along the normal until clear.

The dot, and the static offset
------------------------------
The dot is the middle of the space, then shifted along the depth axis by a
**static offset that is configured per parking type** (`PARK_DEPTH_OFFSET_M` and
`PARK_PARALLEL_DEPTH_OFFSET_M`, or `park_offset_m` on the waypoint). Positive is
deeper in - measured from the lone line, the side of the space that has no
partner, because that is the only line whose distance means "how far into the
space am I". The two types get their own number because they are not the same
manoeuvre: bow-in wants the hull clear of the back wall, alongside wants it
centred fore-and-aft, and one figure cannot be right for both.

The offset is clamped so the dot stays inside the space, and the panel says when
it was clamped. `parking.depth_m` next to it is the readback that makes the
number tunable from what the operator sees: park once, read how deep the boat
actually sat, adjust by the difference.

Why the box is latched once the boat is committed
-------------------------------------------------
Up to and including ENTER the space is re-measured every tick, because a floating
dock moves and the boat's own position error grows. **From the turn onwards it is
a memory**, and a fresh fit has to earn its way in: it is accepted only if it
agrees with the remembered box on all three of where the dot is
(`PARK_LATCH_TOLERANCE_M`), how wide the mouth is, and which way the way in
faces. That still tracks a floating dock, and it cannot

  * restart a countdown that is eight seconds through because one sweep clipped a
    wall differently, or
  * hand an alongside park a *different* space fitted from the two returns it can
    still see while lying across the berth with the closed end out of view.

The second is the one that arrived with the aft lidar's death, and it is why the
test is on the whole geometry rather than on the dot alone: from that pose the
front unit sees one side wall and part of the other, and three lines fitted from
that can be a plausible box with its mouth 90 degrees from the real one.
"""

import logging
import math

from .. import geo
from ..commander import goto, station_keep, stop
from ..config import LATERAL_MAX_MS, LATERAL_MODE, LATERAL_RC_CHAN
from ..perception import lines as line_fit
from ..perception import parking as parking_geometry
from .base import Behaviour, creep, next_leg

log = logging.getLogger("self_driving.parking")

SEARCH = "search"
PROBE = "probe"
ALIGN = "align"
ENTER = "enter"
TURN = "turn"
HOLD = "hold"
EXIT = "exit"


class Parking(Behaviour):
    """Line-based parking. `parallel=True` switches to the alongside variant."""

    name = "park"
    # What the classifier is told the boat is doing. Nothing in this file reads a
    # classification, but the world model keeps building tracks for the chart and
    # the recording while parking runs, and "dock" is the honest answer for what
    # the big white things around the boat are.
    task = "dock"

    def __init__(self, config, parallel=False):
        super().__init__(config)
        self.parallel = parallel
        self.name = "park_parallel" if parallel else "park"
        self.phase = SEARCH
        self.box = None          # the space in WORLD metres, or None
        self._phase_at = None
        self._hold_from = None
        self._hold_restarts = 0
        self._along_sign = None  # which way along the space the bow points
        self._segments = 0
        self._reapproaches = 0
        # The turn in progress: which heading, and what to do when it is reached.
        self._turn_to = None
        self._turn_next = HOLD
        self._turned_back = False
        # Whether the box is a memory rather than a measurement, and how long it
        # has been one. Set when the boat commits to the turn - see `_measure`.
        self._latched_at = None
        self._ignored = 0
        # Where the probe started from, and how far along it the boat has gone.
        # None until the search gives up on the waypoint itself.
        self._probe_from = None

    def start(self, ctx):
        super().start(ctx)
        self.phase = SEARCH
        self.box = None
        self._phase_at = ctx.now
        self._hold_from = None
        self._hold_restarts = 0
        self._along_sign = None
        self._segments = 0
        self._reapproaches = 0
        self._turn_to = None
        self._turn_next = HOLD
        self._turned_back = False
        self._latched_at = None
        self._ignored = 0
        self._probe_from = None

    # ------------------------------------------------------------------ tick

    def update(self, ctx):
        if ctx.boat is None or ctx.heading is None:
            return stop("parking needs a position and a heading")

        self._measure(ctx)

        handler = {
            SEARCH: self._search,
            PROBE: self._probe,
            ALIGN: self._align,
            ENTER: self._enter,
            TURN: self._turn,
            HOLD: self._hold,
            EXIT: self._exit,
        }[self.phase]
        intent = handler(ctx)

        self.note(
            phase=self.phase,
            phase_s=round(_since(self._phase_at, ctx.now), 1),
            segments=self._segments,
            parking=self._telemetry(ctx),
        )
        if self.phase != HOLD:
            # `status` accumulates across ticks, so a countdown left in it from the
            # hold would still be sitting on the chart next to the boat while the
            # boat reversed out. None rather than a stale zero: the chart draws the
            # timer only while there is one.
            self.note(hold_remaining_s=None)
        return intent

    def _to(self, phase, ctx):
        if self.phase == phase:
            return
        self.phase = phase
        self._phase_at = ctx.now
        # The moment the box stops being a measurement and becomes a memory. It is
        # stamped on the way *into* the turn rather than when the lines are lost,
        # because that is the last tick on which the boat was on the dot, square,
        # with all three lines in view - which is the measurement worth keeping.
        if phase in (TURN, HOLD, EXIT):
            if self._latched_at is None:
                self._latched_at = ctx.now
        else:
            # Back outside: re-measure freely again. A re-approach is a fresh look
            # at the space, not a boat second-guessing a memory.
            self._latched_at = None
            self._ignored = 0

    # ---------------------------------------------------------------- phases

    def _search(self, ctx):
        """Get to the waypoint and fit lines until three of them are a space."""
        if self.box is not None:
            self._to(ALIGN, ctx)
            return self._align(ctx)

        target = ctx.target
        if target is None:
            return stop("no parking waypoint")

        distance = geo.distance(ctx.boat, target)
        elapsed = _since(self._phase_at, ctx.now)
        if self._probe_from is not None:
            # A probe that has already started owns the boat, even if the box it
            # found has since been lost: the waypoint is astern by now, and going
            # back out to it to look again from the one place already known not to
            # see the space is a shuttle, not a search.
            self._to(PROBE, ctx)
            return self._probe(ctx)
        if distance <= ctx.config.ARRIVAL_RADIUS_M:
            if elapsed > ctx.config.PARK_SEARCH_TIMEOUT_S:
                # Its own phase, and not only for tidiness: `pilot._watch_progress`
                # exempts it, so a probe that is deliberately driving *away* from
                # the waypoint does not raise the STUCK badge on the panel. A badge
                # during a working probe would spend the operator's attention on the
                # one thing that is going right.
                self._to(PROBE, ctx)
                return self._probe(ctx)
            return station_keep(
                ctx.state, target, ctx.heading, ctx.config,
                f"at the parking waypoint, looking for three lines "
                f"({self._segments} in view, {elapsed:.0f} s)",
                ceiling=ctx.ceiling,
            )

        # A plain position target, deliberately not `steer_towards`: that one
        # deconflicts against the world model, and the world model's opinion is
        # exactly what a parking run is told to ignore. The waypoint is laid a
        # few metres off the space by the operator, so this leg is short.
        speed = self._speed(ctx, ctx.config.PARK_APPROACH_SPEED_MS)
        return goto(
            target, speed,
            f"approaching the parking waypoint, {distance:.0f} m, "
            f"looking for three lines",
        )

    def _probe(self, ctx):
        """Creep in towards the docks on a fixed bearing, still looking. `(intent)`.

        **The waypoint is outside the docks and the boat only sees forwards**, so
        "nothing in view from the waypoint" is the expected case rather than a
        failure: a 2 m space a few metres further in is behind a dock face as far
        as one forward lidar is concerned. Sitting on the waypoint declaring itself
        stuck would waste the crew's twenty seconds (NJORD §8.2) on a situation one
        metre of travel fixes.

        So the boat drives the bearing it has been told the docks are on -
        `park_probe_deg` on the waypoint, else `PARK_PROBE_BEARING_DEG`, which is
        **120 degrees for Havet arena: east and a little south, straight in towards
        land** - bow first, at the docking creep, for at most `PARK_PROBE_M`. Every
        tick of it is still fitting lines, so the probe ends the moment a space
        appears (`_measure` sets the box and `update` hands over to ALIGN).

        Bow first and no faster than the creep, both deliberately: this is a blind
        move towards a structure, and the only sensor that will see the structure is
        the one pointing where the boat is going.
        """
        if self.box is not None:
            # Found one. Hand over from wherever the probe has got to - not from the
            # waypoint, which is behind the boat now and was the wrong place to look
            # from in the first place.
            self._to(ALIGN, ctx)
            return self._align(ctx)

        if self._probe_from is None:
            self._probe_from = ctx.boat
            log.warning(
                "no parking space at the waypoint after %.0f s - probing %.0f deg "
                "for up to %.1f m at %.2f m/s",
                ctx.config.PARK_SEARCH_TIMEOUT_S,
                self._probe_bearing(ctx),
                ctx.config.PARK_PROBE_M,
                self._speed(ctx, ctx.config.PARK_PROBE_SPEED_MS),
            )

        bearing = self._probe_bearing(ctx)
        gone = geo.distance(self._probe_from, ctx.boat)
        limit = ctx.config.PARK_PROBE_M
        self.note(
            probe_bearing_deg=round(bearing, 1),
            probe_m=round(gone, 2),
            probe_limit_m=round(limit, 1),
        )

        if gone >= limit:
            # Out of probe. Said in the words the operator needs, and `stuck` as
            # well, because this is the point where the twenty seconds start
            # mattering: the waypoint was wrong, or the space is not where anybody
            # thought, and neither is something this behaviour can fix.
            message = (
                f"no {self._mouth(ctx):.1f} m x {self._depth():.1f} m parking space "
                f"at the waypoint or {gone:.1f} m along {bearing:.0f} deg "
                f"({self._segments} line(s) in view of the three it needs) - take "
                f"over and reposition"
            )
            self.note(stuck=message)
            return stop(message)

        return creep(
            ctx,
            forward=self._speed(ctx, ctx.config.PARK_PROBE_SPEED_MS),
            desired_heading=bearing,
            reason=(
                f"nothing in view from the waypoint - probing {bearing:.0f} deg "
                f"towards the docks, {gone:.1f}/{limit:.1f} m "
                f"({self._segments} line(s) in view)"
            ),
        )

    def _probe_bearing(self, ctx):
        """Which way the docks are. The waypoint's figure beats the config's."""
        if ctx.waypoint is not None and ctx.waypoint.park_probe_deg is not None:
            return geo.wrap360(float(ctx.waypoint.park_probe_deg))
        return geo.wrap360(ctx.config.PARK_PROBE_BEARING_DEG)

    def _align(self, ctx):
        """Get onto the space's centreline, one standoff out, and square up.

        Two steps, because they are two different problems solved by two different
        sets of thrusters.

        **Coarse.** Further out than `HOLD_TOLERANCE_M` from the approach point, it
        is a plain position target with the heading left free: the boat is outside
        the space, where there is room to turn, and steering three metres with the
        main thrusters takes a third of the time that crabbing them with the
        sideways one would. This is also the step that recovers a big lateral
        offset, and the only one that can do it without a lateral thruster at all.

        **Fine.** Inside that, station-keep on the approach point holding the
        parking type's own heading, until the boat is **on the centreline**, square,
        and stopped.

        The gate is the centreline error and *not* the distance to the approach
        point, and that is the whole point of this phase: the approach into the
        space has to run along the space's normal, so the boat has to start from the
        normal. How far out it starts barely matters - a metre either way is a
        second of creeping - so `align_along_m` is allowed the loose tolerance and
        `align_across_m` gets the tight one.
        """
        if self.box is None:
            self._to(SEARCH, ctx)
            return self._search(ctx)

        approach = geo.offset_point(
            self.box["target"], self.box["into_deg"] + 180.0,
            ctx.config.PARK_STANDOFF_M,
        )
        desired = self._desired_heading(ctx)
        across, along = self._space_error(ctx, approach)
        misalignment = abs(geo.angle_diff(desired, ctx.heading))
        self.note(
            align_across_m=round(across, 2),
            align_along_m=round(along, 2),
            align_error_deg=round(misalignment, 1),
            reapproaches=self._reapproaches,
        )

        offset = geo.distance(ctx.boat, approach)
        if offset > ctx.config.HOLD_TOLERANCE_M:
            speed = self._speed(ctx, ctx.config.PARK_APPROACH_SPEED_MS)
            return goto(
                approach, speed,
                f"running to the approach point, {offset:.1f} m off the "
                f"centreline one standoff out",
            )

        if (
            abs(across) <= ctx.config.PARK_CENTRE_TOLERANCE_M
            and abs(along) <= ctx.config.HOLD_TOLERANCE_M
            and misalignment <= ctx.config.PARK_ALIGN_TOLERANCE_DEG
            and ctx.state.speed < ctx.config.STATIONARY_SPEED_MS * 2.0
        ):
            self._to(ENTER, ctx)
            return self._enter(ctx)

        return station_keep(
            ctx.state, approach, desired, ctx.config,
            f"squaring up on the centreline ({across:+.2f} m across, "
            f"{misalignment:.0f} deg out)",
            ceiling=ctx.ceiling,
        )

    def _enter(self, ctx):
        """Creep onto the dot, **along the space's normal and nothing else**.

        The boat travels straight in. It never aims diagonally at the dot, it never
        leans the heading to cut a corner, and it never crabs across the space:

          * the space is barely wider than the hull, so a diagonal path clips a
            wall that a straight one clears;
          * the heading is the approach heading throughout, so a boat that steered
            to close a lateral error would arrive crooked - and crooked in a 2 m
            gap is a collision;
          * and sideways is not a direction this hull travels in (see the module
            notes), so crabbing across the mouth is not on the table either.

        Which leaves one honest answer to being off the centreline: **stop, and go
        back out and line up again**. The trim thruster holds the line the approach
        established; anything bigger than a trim is a failed approach and is
        treated as one.
        """
        if self.box is None:
            # The remembered space is exactly as stale as the reason it vanished.
            self._to(ALIGN, ctx)
            return stop("lost the parking space mid-entry - stopping to re-acquire")

        target = self.box["target"]
        across, along = self._space_error(ctx, target)
        error = geo.distance(ctx.boat, target)
        offset = abs(across) > ctx.config.PARK_CENTRE_TOLERANCE_M
        self.note(
            dot_error_m=round(error, 2),
            across_m=round(across, 2),
            along_m=round(along, 2),
            realigning=offset,
        )

        if error <= ctx.config.PARK_TARGET_TOLERANCE_M:
            # On the dot. Now take up the parking angle - which for a bow-in park
            # is the angle it already has, and for an alongside one is 90 degrees
            # away. `_begin_turn` sends it straight on to the hold when there is
            # nothing to turn.
            return self._begin_turn(ctx, self._park_heading(ctx), HOLD)

        if offset:
            return self._reapproach(
                ctx, f"{abs(across):.2f} m off the centreline"
            )

        return self._approach_move(
            ctx, target,
            f"entering along the normal, {along:.2f} m to the dot",
            travel=True,
        )

    def _reapproach(self, ctx, why):
        """Give up on this entry and square up again from outside. `(intent)`.

        The only way to fix a real offset, given that the boat may not steer inside
        the space and may not crab across it. Outside there is room to turn, the
        main thrusters do the work, and `_align`'s coarse leg is a plain position
        target that puts the boat back on the centreline properly.

        Counted, because a boat that cannot centre itself will otherwise shuttle in
        and out of the mouth for the rest of the run looking busy. After
        `PARK_MAX_REAPPROACHES` it says so in the words the operator needs.
        """
        self._reapproaches += 1
        self._to(ALIGN, ctx)
        if self._reapproaches > ctx.config.PARK_MAX_REAPPROACHES:
            message = (
                f"cannot line up on the space: {why}, after "
                f"{self._reapproaches} attempts - take over"
            )
            self.note(stuck=message)
            return stop(message)
        return stop(
            f"{why} - backing out to re-approach "
            f"({self._reapproaches}/{ctx.config.PARK_MAX_REAPPROACHES})"
        )

    def _begin_turn(self, ctx, heading, then):
        """Start rotating to `heading`, or go straight to `then` if already there.

        Refuses the turn outright when the space is known to be too small for it -
        see `PARK_TURN_CLEARANCE_M`, which is **0 by default because nobody has
        measured this hull's turning circle**. With 0 it does not check, and
        rotating in a 2 m space is then the operator's judgement rather than
        something this code has quietly signed off.
        """
        if self.box is None:
            return stop("lost the parking space")

        error = abs(geo.angle_diff(heading, ctx.heading))
        if error <= ctx.config.PARK_ALIGN_TOLERANCE_DEG:
            self._to(then, ctx)
            if then == HOLD:
                self._hold_from = ctx.now
            return self._dispatch(ctx, then)

        clearance = ctx.config.PARK_TURN_CLEARANCE_M
        if clearance > 0.0 and self.box["depth_m"] < clearance:
            message = (
                f"will not turn {error:.0f} deg in a {self.box['depth_m']:.2f} m "
                f"space - it needs {clearance:.2f} m to swing - take over"
            )
            self.note(stuck=message)
            return stop(message)

        self._turn_to = heading
        self._turn_next = then
        self._to(TURN, ctx)
        return self._turn(ctx)

    def _dispatch(self, ctx, phase):
        return {
            SEARCH: self._search,
            PROBE: self._probe,
            ALIGN: self._align,
            ENTER: self._enter,
            TURN: self._turn,
            HOLD: self._hold,
            EXIT: self._exit,
        }[phase](ctx)

    def _turn(self, ctx):
        """Rotate on the spot to the parking angle, holding the dot underneath.

        This is where an alongside park becomes an alongside park: it comes down
        the normal bow-first like the other type, and the 90 degrees happen **in
        the space**, on the dot, with the position held by the same controller that
        brought it in. Turning outside and crabbing in would need the sideways
        thruster to cover the whole depth of the space at 0.35 m/s, and would spend
        the entire entry with the hull broadside to a gap it has to fit into.

        The position is held throughout rather than after, because a boat that
        rotates first and corrects afterwards has spent the rotation drifting.
        """
        if self.box is None:
            self._to(ALIGN, ctx)
            return stop("lost the parking space mid-turn - stopping to re-acquire")

        wanted = self._turn_to if self._turn_to is not None else ctx.heading
        error = geo.angle_diff(wanted, ctx.heading)
        self.note(
            turn_to_deg=round(wanted % 360.0, 1),
            turn_error_deg=round(error, 1),
        )

        if abs(error) <= ctx.config.PARK_ALIGN_TOLERANCE_DEG:
            nxt = self._turn_next
            self._to(nxt, ctx)
            if nxt == HOLD:
                self._hold_from = ctx.now
            return self._dispatch(ctx, nxt)

        return self._approach_move(
            ctx, self.box["target"],
            f"turning to the parking angle, {error:+.0f} deg to go",
        )

    def _hold(self, ctx):
        """Sit on the dot, **at the parking type's own angle**, for the duration.

        The clock needs *both* to be true, and the heading half is not decoration:
        what the task asks for is a boat parked in a berth, and a boat sitting on
        the right spot at thirty degrees to the walls is not parked, it is wedged.
        Bow-in means square to the closed end; alongside means parallel to it.

        Time spent with either out of tolerance is not counted, because the rule
        wants a continuous stretch. Restarting is loud - `hold_restarts` and a yellow
        countdown on the chart - rather than silent, because a timer that keeps going
        back to ten in a tide is the operator's cue to take over rather than
        something to hide.
        """
        required = self._required_hold(ctx)
        target = self.box["target"] if self.box else ctx.boat
        desired = self._desired_heading(ctx)
        error = geo.distance(ctx.boat, target)
        misalignment = abs(geo.angle_diff(desired, ctx.heading))

        off_position = error > ctx.config.PARK_HOLD_TOLERANCE_M
        off_angle = misalignment > ctx.config.PARK_HOLD_ANGLE_DEG
        if off_position or off_angle:
            if self._hold_from is not None and ctx.now - self._hold_from > 0.5:
                self._hold_restarts += 1
            self._hold_from = ctx.now
            self.note(
                hold_restart_why=(
                    f"{error:.2f} m off the dot" if off_position
                    else f"{misalignment:.0f} deg off the parking angle"
                )
            )

        held = _since(self._hold_from, ctx.now)
        remaining = max(0.0, required - held)
        self.note(
            hold_elapsed_s=round(held, 1),
            hold_required_s=round(required, 1),
            # What the chart draws next to the boat. One number, already counted
            # down on the vessel, so the browser never has to guess how old the
            # frame it is holding is.
            hold_remaining_s=round(remaining, 1),
            hold_restarts=self._hold_restarts,
            dot_error_m=round(error, 2),
            hold_error_deg=round(misalignment, 1),
            stationary=ctx.state.stationary,
        )
        if not _lateral_available():
            # Holding a spot is the one job the sideways thruster does have, and
            # without it the boat can only hold along its own fore-and-aft axis and
            # its heading. For an alongside park that is the axis that does *not*
            # keep it off the dock face.
            self.note(
                hold_warning=(
                    "no sideways thruster configured - the dot can only be held "
                    "on one axis"
                )
            )
        if self._blind(ctx):
            # Said every tick of the hold, on the panel, because it changes what the
            # operator should be watching: the boat is not looking at the berth any
            # more, it is holding a remembered middle on GNSS, and the thing that
            # will go wrong is drift rather than a bad fit.
            self.note(
                hold_blind=(
                    f"lying across the space - holding the remembered middle of a "
                    f"{self.box['mouth_m']:.2f} m mouth on GNSS, the closed end is "
                    f"out of the front lidar's view"
                    if self.box
                    else "no space in view"
                )
            )

        if held >= required:
            self._to(EXIT, ctx)
            return self._exit(ctx)

        detail = f"{held:.0f}/{required:.0f} s"
        if off_angle:
            detail = f"{detail}, squaring up {misalignment:.0f} deg"
        return self._approach_move(
            ctx, target,
            f"parked - holding {detail}"
            f"{'' if ctx.state.stationary else ' (still moving)'}",
        )

    def _exit(self, ctx):
        """Straight back out along the normal, astern, until clear.

        Both types leave the way they came in, down the same line and on the main
        thrusters. An alongside park is lying across that line by the time the hold
        is over, so it **turns back to the approach angle first** - because coming
        out sideways would be asking the trim thruster to move the boat, which is
        the one thing it is not for.

        Leaving *ahead* is what an alongside park cannot do, and the arithmetic is
        why: the space is 4 m long and this boat is most of that, so from the middle
        of it the bow reaches the end wall long before the hull is clear. NJORD
        §9.3's "moves forward toward the next GPS point" is still satisfied - the
        next waypoint does the forward part, once the boat is out of the box.
        """
        if self.box is None:
            self.done = True
            return stop("clear of the parking space")

        distance = geo.distance(ctx.boat, self.box["target"])
        target = ctx.config.PARK_EXIT_M
        self.note(exit_distance_m=round(distance, 2))

        if distance >= target:
            self.done = True
            return stop(f"clear of the parking space ({distance:.1f} m)")

        # Lying across the way out: turn back onto the approach heading before
        # retreating, once.
        if not self._turned_back and self.parallel:
            self._turned_back = True
            return self._begin_turn(ctx, self.box["into_deg"], EXIT)

        out = geo.offset_point(
            self.box["target"], self.box["into_deg"] + 180.0, target + 0.5
        )
        return self._approach_move(
            ctx, out,
            f"reversing out along the normal, {distance:.1f}/{target:.1f} m",
            travel=True,
            speed=self._speed(ctx, ctx.config.PARK_REVERSE_SPEED_MS),
        )

    # ------------------------------------------------------------- the driving

    def _speed(self, ctx, wanted):
        """`wanted`, held under the operator's speed setting. Every speed in here.

        One method rather than a `min` at each site, because there are seven of
        them - the run to the waypoint, the probe, the approach, the entry, the
        turn, the hold and the way out - and the one that gets forgotten is the one
        that makes a 0.1 m/s test a 0.3 m/s test.

        A cap, never a floor: `ctx.ceiling` is what the boat *may* do and these
        figures are what a berth manoeuvre *should* do, so the slower of the two
        always wins.
        """
        return min(float(wanted), ctx.ceiling)

    def _space_error(self, ctx, target):
        """`(across, along)` from the boat to `target`, in the SPACE's own axes.

        `along` is positive deeper into the space, `across` positive to the
        starboard side of the way in. Both are independent of which way the boat
        happens to be pointing, which is exactly why the controller works in them:
        "am I on the centreline" and "how deep am I" are questions about the space,
        and answering them in the hull's frame is what produces a diagonal approach.
        """
        into = self.box["into_deg"] if self.box else ctx.heading
        return geo.world_to_boat(
            target[0] - ctx.boat[0], target[1] - ctx.boat[1], into
        )

    def _approach_move(self, ctx, target, reason, *, travel=False, speed=None):
        """Move onto `target` along the space's axes, under the phase's heading.

        Not `station_keep`: that one commands nothing inside `HOLD_TOLERANCE_M`,
        which is a metre, and a metre of slack inside a 2 m space is half a space.
        This closes on `PARK_ENTRY_P` with a centimetre-scale deadband.

        The work is done in the **space's** frame and only then rotated into the
        hull's, which is what keeps the motion square to the walls whatever the boat
        is pointing at. One piece of code, both types, no branch.

        `travel` is the important argument, and it is the sideways thruster's job
        description:

            travel=True     going somewhere - in, or out. The sideways term is held
                            to `PARK_TRIM_LATERAL_MS`, a trim: enough to hold the
                            line the approach established against a light set, not
                            enough to be a way of getting anywhere. This hull is
                            three hulls broadside when pushed sideways and it does
                            not go there usefully; anything bigger than a trim means
                            the approach failed and `_enter` backs out.
            travel=False    holding - the hold and the turn. The thruster gets its
                            full authority, because holding a spot against a tide is
                            exactly what it is for.

        The two axes are scaled **together** when they hit those limits, never
        clipped separately. Clipping one axis rotates the commanded motion, and a
        motion that was square to the walls before the clamp and diagonal after it
        is the exact failure this method exists to avoid.

        Both limits are then held under the **operator's speed setting**
        (`ctx.ceiling`), which is what makes "set it to 0.1 m/s for the first
        parking test" mean what it says. It is a cap and never a floor: at the
        ordinary 1.2 m/s setting nothing here changes, because every figure in
        this method is already far below it.
        """
        desired = self._desired_heading(ctx)
        across, along = self._space_error(ctx, target)
        gain = ctx.config.PARK_ENTRY_P
        deadband = ctx.config.PARK_DEADBAND_M

        v_along = 0.0 if abs(along) < deadband else along * gain
        v_across = 0.0 if abs(across) < deadband else across * gain

        # Space axes -> world -> hull axes.
        east, north = geo.boat_to_world(
            v_across, v_along, self.box["into_deg"] if self.box else ctx.heading
        )
        starboard, forward = geo.world_to_boat(east, north, ctx.heading)

        limit = self._speed(
            ctx, ctx.config.PARK_SPEED_MS if speed is None else speed
        )
        sideways = self._speed(
            ctx, ctx.config.PARK_TRIM_LATERAL_MS if travel else LATERAL_MAX_MS
        )
        scale = 1.0
        if abs(forward) > limit:
            scale = min(scale, limit / abs(forward))
        if abs(starboard) > sideways:
            scale = min(scale, sideways / abs(starboard))

        # ...and the **resultant** against the operator's setting, which is a
        # different test from either axis: 0.02 m/s ahead plus 0.10 m/s sideways is
        # legal on both axes and 0.102 m/s through the water. `commander._limit_pair`
        # would scale it on the way out anyway, so without this the behaviour plans
        # at a speed it does not get - and then reports that speed on the panel.
        resultant = math.hypot(forward * scale, starboard * scale)
        if resultant > ctx.ceiling > 0.0:
            scale *= ctx.ceiling / resultant

        return creep(
            ctx,
            forward=forward * scale,
            desired_heading=desired,
            starboard=starboard * scale,
            reason=reason,
        )

    def _desired_heading(self, ctx):
        """The heading to hold **in this phase**. One place, so it cannot diverge.

            PROBE                the bearing the docks are on. The bow points where
                                 the boat is going, because the only sensor that
                                 will see the dock points forwards.
            SEARCH/ALIGN/ENTER   the approach heading: square to the space, bow
                                 pointing in. Both types, because both come down
                                 the normal.
            TURN                 whatever the turn is turning to.
            HOLD                 the parking type's angle - and it is gated there.
            EXIT                 whatever the boat is lying at now, which is the
                                 parking angle unless `_exit` turned it back.
        """
        if self.phase == PROBE:
            return self._probe_bearing(ctx)
        if self.box is None:
            return ctx.heading
        if self.phase == TURN and self._turn_to is not None:
            return self._turn_to
        if self.phase == HOLD:
            return self._park_heading(ctx)
        if self.phase == EXIT:
            return (
                self.box["into_deg"] if self._turned_back
                else self._park_heading(ctx)
            )
        return self.box["into_deg"]

    def _park_heading(self, ctx):
        """The angle to sit at for the hold. The type decides.

        Bow-in: square to the closed end, which is the heading it arrived on.

        Alongside: 90 degrees off that - parallel to the closed end - and **which
        of the two ways round is latched the first time it is asked**, so a
        wandering compass cannot make the boat change its mind halfway through the
        turn.

        The tie-break is the leg *after* this waypoint: the bow ends up pointing
        the way the boat is about to leave, which is what NJORD §9.3's "moves
        forward toward the next GPS point" wants and saves a second rotation
        outside the space. With no next waypoint it keeps whichever side the boat
        is already nearer, which is the cheaper turn.
        """
        if self.box is None:
            return ctx.heading
        into = self.box["into_deg"]
        if not self.parallel:
            return into

        if self._along_sign is None:
            one, other = geo.wrap360(into + 90.0), geo.wrap360(into - 90.0)
            following = next_leg(ctx)
            reference = following[0] if following is not None else ctx.heading
            self._along_sign = (
                1.0
                if abs(geo.angle_diff(one, reference))
                <= abs(geo.angle_diff(other, reference))
                else -1.0
            )
        return geo.wrap360(into + 90.0 * self._along_sign)

    # ------------------------------------------------------------ the geometry

    def _measure(self, ctx):
        """Fit lines, find the space, and keep it in world metres.

        Two regimes, and which one is in force is the difference between a boat
        that is looking at the berth and a boat that is remembering it:

        **Before the turn** (SEARCH, ALIGN, ENTER) every fit replaces the box. The
        boat is outside or on the way in, facing the closed end, so all three lines
        are in front of the one lidar there is, and a floating dock that has moved
        since the last tick should move the dot with it.

        **From the turn onwards** (TURN, HOLD, EXIT) the box is latched and a fit
        has to agree with it to be accepted - see `_agrees`. This is what carries an
        alongside park through the part of the manoeuvre where the closed end is
        abeam and out of view: the boat holds the width, the depth and the pose it
        measured from the dot while it could still see all three lines, and holds
        the middle of *that* on GNSS.
        """
        segments = line_fit.fit_sweeps(
            ctx.sweeps, config=ctx.config, sources=self._sources(ctx)
        )
        self._segments = len(segments)
        if not segments:
            return

        box = parking_geometry.find_box(
            segments,
            mouth_m=self._mouth(ctx),
            depth_m=self._depth(),
            tolerance_m=ctx.config.PARK_BOX_TOLERANCE_M,
            angle_deg=ctx.config.PARK_BOX_ANGLE_DEG,
            span_fraction=ctx.config.PARK_BOX_SPAN_FRACTION,
            min_line_m=ctx.config.LINE_MIN_M,
            max_range_m=ctx.config.LINE_MAX_RANGE_M,
        )
        if box is None:
            return

        found = self._to_world(ctx, box)
        if self._latched() and self.box is not None:
            agrees, why = self._agrees(ctx, found)
            if not agrees:
                self._ignored += 1
                self.note(box_ignored=why, boxes_ignored=self._ignored)
                return
        self.box = found

    def _latched(self):
        """Whether the box is a memory. True from the turn onwards."""
        return self.phase in (TURN, HOLD, EXIT)

    def _blind(self, ctx):
        """Whether the closed end is out of the front lidar's view right now.

        True for an **alongside** park from the moment it starts turning until it
        has turned back to leave: that is the stretch where the boat lies across
        the space with the lone line abeam, and with no aft lidar there is nothing
        looking at it. Not a guess about what the sweep contains - it is a
        statement about where the hull is pointing, which is the thing that is
        actually known.

        False for a bow-in park throughout. It sits square to the closed end, so
        the one lidar there is has all three lines in front of it the whole time.
        """
        if not self.parallel or self.box is None:
            return False
        if not self._latched():
            return False
        if self.phase == EXIT and self._turned_back:
            return False
        if ctx.heading is None:
            return True
        # Square to the way in means looking at the closed end; ninety degrees off
        # it means looking down the space's width at a side wall.
        off = abs(geo.angle_diff(self.box["into_deg"], ctx.heading))
        return off > 45.0

    def _agrees(self, ctx, found):
        """Whether a fresh fit is the *same* space as the latched one. `(ok, why)`.

        Three tests rather than one, and the two extra ones arrived with the aft
        lidar's death. Distance alone was enough while the boat always faced the
        closed end: any real re-measurement of the same box lands within a few
        centimetres of the same dot, so a jump meant a different box.

        Lying across a berth with one forward lidar, that is no longer true. From
        that pose the front unit sees one side wall and part of the other, plus
        whatever is beyond the mouth, and three lines fitted from that can produce a
        box whose *centre* is close to the right one and whose mouth faces ninety
        degrees away. Driving out along that box's normal would be driving into a
        wall. So the mouth width and the way in have to match as well - all three,
        or the fit is a different space wearing the same name.
        """
        moved = geo.distance(self.box["target"], found["target"])
        if moved > ctx.config.PARK_LATCH_TOLERANCE_M:
            return False, f"the dot jumped {moved:.2f} m"
        widened = abs(found["mouth_m"] - self.box["mouth_m"])
        if widened > ctx.config.PARK_BOX_TOLERANCE_M:
            return False, f"the mouth changed by {widened:.2f} m"
        turned = abs(geo.angle_diff(found["into_deg"], self.box["into_deg"]))
        if turned > ctx.config.PARK_BOX_ANGLE_DEG:
            return False, f"the way in swung {turned:.0f} deg"
        return True, ""

    def _sources(self, ctx):
        """Which lidars to believe: **the front one, and there is no other.**

        The aft unit is broken (2026-08-11) and it was never fitted into a parking
        space even when it worked, because its mounting geometry is hand-measured
        and a flipped `LIGMAX_AFT_LIDAR_ANGLE_DIR` gives a complete and **mirrored**
        world astern (docs/testing.md 7c) - and a mirrored parking space is a
        parking space on the wrong side of the boat, which this behaviour would
        drive into with confidence.

        A tuple with one entry rather than nothing at all: `fit_sweeps` filters the
        sweeps it is given by source, and being explicit is what stops an aft sweep
        that reappears on the bus one day from silently joining in.
        """
        return ("front_lidar",)

    def _to_world(self, ctx, box):
        """A boat-frame `ParkingBox` as the world-frame dict this behaviour keeps."""
        offset, clamped = self._offset(ctx, box)
        dot = box.point_at_depth(offset)
        return {
            "centre": self._point(ctx, box.centre),
            "target": self._point(ctx, dot),
            "into_deg": geo.wrap360(box.into_deg + ctx.heading),
            "mouth_m": box.mouth_m,
            "depth_m": box.depth_m,
            "depth_measured_m": box.depth_measured_m,
            "depth_source": box.depth_source,
            "offset_m": offset,
            "offset_clamped": clamped,
            # How deep the dot sits, measured from the lone line. The readback the
            # offset is tuned against.
            "dot_depth_m": box.depth_of(dot),
            "corner_gap_m": box.corner_gap_m,
            "corners": [self._point(ctx, corner) for corner in box.corners],
            "lines": [
                [self._point(ctx, segment.a), self._point(ctx, segment.b)]
                for segment in (box.back, box.sides[0], box.sides[1])
            ],
            "seen_at": ctx.now,
        }

    def _point(self, ctx, boat_point):
        east, north = geo.boat_to_world(boat_point[0], boat_point[1], ctx.heading)
        return (ctx.boat[0] + east, ctx.boat[1] + north)

    def _offset(self, ctx, box):
        """The static depth offset to use, and whether it had to be clamped.

        Clamped to leave the dot inside the space with a little to spare. An
        offset deeper than the space is an operator error - almost always a figure
        meant for the other parking type - and honouring it literally would drive
        the boat through the lone line.
        """
        wanted = self._configured_offset(ctx)
        limit = max(0.0, box.depth_m * 0.5 - ctx.config.PARK_OFFSET_MARGIN_M)
        offset = _clamp(wanted, -limit, limit)
        return offset, abs(offset - wanted) > 1e-6

    def _configured_offset(self, ctx):
        if ctx.waypoint is not None and ctx.waypoint.park_offset_m is not None:
            return float(ctx.waypoint.park_offset_m)
        return (
            ctx.config.PARK_PARALLEL_DEPTH_OFFSET_M
            if self.parallel
            else ctx.config.PARK_DEPTH_OFFSET_M
        )

    def _mouth(self, ctx):
        """How wide the way in is. The waypoint's figure beats the rulebook's."""
        if ctx.waypoint is not None and ctx.waypoint.berth_width_m is not None:
            return float(ctx.waypoint.berth_width_m)
        return (
            self.config.PARK_PARALLEL_MOUTH_M
            if self.parallel
            else self.config.PARK_MOUTH_M
        )

    def _depth(self):
        return (
            self.config.PARK_PARALLEL_DEPTH_M
            if self.parallel
            else self.config.PARK_DEPTH_M
        )

    def _required_hold(self, ctx):
        if ctx.waypoint is not None and ctx.waypoint.hold_s is not None:
            return float(ctx.waypoint.hold_s)
        return (
            self.config.PARK_PARALLEL_HOLD_S
            if self.parallel
            else self.config.PARK_HOLD_S
        )

    # ------------------------------------------------------------- telemetry

    def _telemetry(self, ctx):
        """The space, the dot and the countdown, for the operator's chart.

        World metres, which is the frame the chart draws in, so the browser adds
        nothing to it but a colour. Rounded here rather than there: this rides up
        a 4G link twice a second.
        """
        if self.box is None:
            return {
                "seen": False,
                "segments": self._segments,
                "speed_cap_ms": round(ctx.ceiling, 2),
                # Only while probing, so a panel that is not showing a probe is
                # showing nothing about one.
                **(
                    {
                        "probing_deg": round(self._probe_bearing(ctx), 1),
                        "probed_m": round(
                            geo.distance(self._probe_from, ctx.boat), 2
                        ),
                    }
                    if self._probe_from is not None and ctx.boat is not None
                    else {}
                ),
            }
        box = self.box
        required = self._required_hold(ctx)
        held = _since(self._hold_from, ctx.now)
        return {
            "seen": True,
            "kind": self.name,
            # Whether these figures are being measured right now or remembered
            # from the last tick that could see all three lines. The single most
            # important field on this block during an alongside park: everything
            # below it is either a measurement or a dead-reckoned memory, and the
            # operator cannot tell which from the numbers themselves.
            "source": "remembered" if self._latched() else "measured",
            "blind": self._blind(ctx),
            "remembered_s": (
                round(_since(self._latched_at, ctx.now), 1)
                if self._latched_at is not None
                else None
            ),
            "boxes_ignored": self._ignored,
            # The speed everything in this manoeuvre is held under - the operator's
            # setting. On the block because "why is it creeping" and "why is it not
            # creeping" are both answered by this number.
            "speed_cap_ms": round(ctx.ceiling, 2),
            "target": _round_point(box["target"]),
            "centre": _round_point(box["centre"]),
            "corners": [_round_point(point) for point in box["corners"]],
            "lines": [
                [_round_point(point) for point in line] for line in box["lines"]
            ],
            # Rounded then re-wrapped: a bearing a hair under zero wraps to
            # 359.99 and rounds to "360.0", which reads on a panel like a bug.
            "into_deg": round(box["into_deg"], 1) % 360.0,
            # The angle the boat has to sit at for the hold to count - square to
            # the closed end bow-in, 90 degrees off it alongside. Published so the
            # chart can draw it through the dot: "is the boat on the spot" and "is
            # the boat at the right angle" are two questions and the second one is
            # the one that is invisible without a reference to compare against.
            "park_heading_deg": round(self._park_heading(ctx), 1) % 360.0,
            "heading_error_deg": (
                round(abs(geo.angle_diff(self._park_heading(ctx), ctx.heading)), 1)
                if ctx.heading is not None
                else None
            ),
            "mouth_m": round(box["mouth_m"], 2),
            "depth_m": round(box["depth_m"], 2),
            "depth_measured_m": round(box["depth_measured_m"], 2),
            "depth_source": box["depth_source"],
            "offset_m": round(box["offset_m"], 2),
            "offset_clamped": box["offset_clamped"],
            "dot_depth_m": round(box["dot_depth_m"], 2),
            "corner_gap_m": round(box["corner_gap_m"], 2),
            "age_s": round(max(0.0, ctx.now - box["seen_at"]), 1),
            "hold_required_s": round(required, 1),
            "hold_remaining_s": (
                round(max(0.0, required - held), 1) if self.phase == HOLD else None
            ),
            "segments": self._segments,
        }


def _lateral_available():
    """Whether there is sideways thrust to correct an offset with.

    `"rc"` with no channel counts as **no**: `commander._lateral` refuses to guess
    a channel number, because the wrong one drives something else on the boat, so
    in that state the sideways command goes nowhere and the behaviour needs to know
    that rather than waiting for a correction that cannot arrive.

    `"mavlink"` counts as yes, which is the one hopeful answer here: if ArduPilot
    has no lateral output configured it silently drops the term, and nothing on this
    side can tell. That is the documented cost of the default
    (`commander.py`), and on the water it looks like a boat that will not come off
    the centreline - see docs/testing.md 7j.
    """
    if LATERAL_MODE == "none":
        return False
    if LATERAL_MODE == "rc" and not LATERAL_RC_CHAN:
        return False
    return True


def _since(stamp, now):
    """Seconds since `stamp`, or 0 if there is no stamp yet.

    `is None` rather than the shorter `stamp or now`, because a stamp of **zero**
    is a real value: `main.py` ticks on `time.time()`, but a replay or a
    simulation ticks from zero, and `0.0 or now` quietly returns `now` - which
    makes a ten-second hold sit at nought point nought for ever without anything
    looking wrong. That is a bug that only appears off the boat, which is the one
    place it is most expensive to have.
    """
    return 0.0 if stamp is None else max(0.0, now - stamp)


def _round_point(point):
    return [round(point[0], 2), round(point[1], 2)]


def _clamp(value, low, high):
    return max(low, min(high, value))
