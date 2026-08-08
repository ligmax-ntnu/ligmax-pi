"""The pilot: runs the plan through the behaviours, and refuses to when it must.

    pilot = Pilot(config, commander)
    intent = pilot.tick(state, world, clusters, now)

Two jobs, and the order they are done in is the whole design:

  1. **Decide whether the boat may be driven at all.** Every tick, before any
     behaviour is consulted. If the answer is no, nothing downstream gets a say.
  2. Run the current waypoint's behaviour, advance the plan when it finishes.

Putting the safety checks in one place, ahead of everything, is what makes them
reviewable. A check buried inside a behaviour protects that behaviour; a check
here protects the boat, including from a behaviour nobody has written yet.

The checks, and what each one is really for
--------------------------------------------
    engaged?        autonomy is opt-in. The node runs from boot and observes
                    continuously - building the world model, recording, feeding
                    the chart - and commands nothing until an operator says so.
                    That is what makes it safe to leave running.
    E-stop          the relay is open, propulsion is cut. Commanding anything is
                    pointless and pretending otherwise is misleading.
    somebody else   the autopilot is in MANUAL, or the mode left GUIDED. A human
                    has taken the boat. NJORD §8.2 gives the crew twenty seconds
                    to do exactly that, and the *instant* they do, this must stop
                    fighting them for the controls.
    comms           no state frame for `LOSS_OF_COMMS_STOP_S`. NJORD §7.3 requires
                    autonomous movement to stop within 60 s of losing radio
                    contact; we use ten, because sixty seconds at task speed is
                    ten metres of uncommanded boat.
    stale fix       the newest position is older than `MAX_NAV_AGE_S`. Steering
                    on a remembered position is worse than stopping, for the same
                    reason `navigation.py` publishes `boat: null` rather than
                    leaving the vessel drawn where it was.
    plan            no plan, or the plan is finished.

Anything that fails puts the boat in HOLD and says why, in a sentence, on the
dashboard - because NJORD §11.4 scores the boat explaining itself and because
the operator's next decision depends on knowing which of those seven it was.

Progress, and the twenty seconds
--------------------------------
A behaviour that is running but getting nowhere is the case the rules are
written around: §8.2 gives the boat twenty autonomous seconds to sort itself out
before the crew must take over by remote. So distance-to-target is watched, and
`STUCK_WARN_S` (twelve seconds) of no improvement raises a warning - early
enough that the crew still have all twenty.
"""

from __future__ import annotations

import logging

from . import behaviours as behaviour_registry
from . import commander as commander_module
from . import geo
from .behaviours.base import Context
from .plan import Plan, PlanError

log = logging.getLogger("self_driving.pilot")

IDLE = "IDLE"
RUNNING = "RUNNING"
PAUSED = "PAUSED"
BLOCKED = "BLOCKED"
FINISHED = "FINISHED"

# How much closer the boat has to get for it to count as progress. Below this it
# is noise on the position, not movement.
PROGRESS_M = 0.75

# Modes that mean somebody else is driving. Seeing one of these is an immediate
# hand-back, not a fight for the controls.
PILOTED_MODES = frozenset({"MANUAL", "ACRO", "STEERING", "SIMPLE"})


class Pilot:
    """Owns the plan cursor, the current behaviour, and the decision to drive."""

    def __init__(self, config, commander):
        self.config = config
        self.commander = commander
        self.plan = Plan.load()
        self.mode = IDLE
        self.behaviour = None
        self.reason = "not engaged"
        self.blocked_reason = ""
        self._best_distance = None
        self._progress_at = None
        self._last_waypoint_index = None
        self.stuck = False
        self.started_at = None

    # ------------------------------------------------------------------ tick

    def tick(self, state, world, clusters, now):
        """One decision. Returns the `Intent` to send. Never raises."""
        allowed, why = self._may_drive(state, now)
        if not allowed:
            self.blocked_reason = why
            if self.mode == RUNNING:
                self.mode = BLOCKED
                log.warning("autonomy blocked: %s", why)
            if self.commander.engaged:
                self.commander.disengage(why)
            self.reason = why
            return commander_module.idle(why)

        self.blocked_reason = ""
        if self.mode == PAUSED:
            self.reason = "paused by the operator"
            return commander_module.stop(self.reason)

        if self.mode != RUNNING:
            self.reason = "not engaged"
            return commander_module.idle(self.reason)

        self.commander.engage(state)

        waypoint = self.plan.current
        if waypoint is None:
            self.mode = FINISHED
            self.reason = f"plan {self.plan.name!r} complete"
            log.info("%s", self.reason)
            return commander_module.stop(self.reason)

        # The context is built before the behaviour, because `start()` takes one
        # - a behaviour that initialises its state there (which docking, COLREG
        # and station keeping all do) is broken if it is only ever `update()`d.
        ctx = Context(
            state=state,
            world=world,
            plan=self.plan,
            config=self.config,
            now=now,
            waypoint=waypoint,
            leg=self.plan.leg(state.origin, state.position),
            task="transit",
            clusters=clusters,
        )

        # A new waypoint means a new behaviour, and a fresh progress clock.
        if self._last_waypoint_index != waypoint.index or self.behaviour is None:
            self._enter(waypoint, ctx, now)
        ctx.task = getattr(self.behaviour, "task", "transit")

        try:
            intent = self.behaviour.update(ctx)
        except Exception as exc:  # noqa: BLE001 - a planner bug must not run the boat on
            log.exception("behaviour %s failed: %s", self.behaviour.name, exc)
            self.mode = BLOCKED
            self.blocked_reason = f"{self.behaviour.name} raised {exc!r}"
            self.commander.disengage(self.blocked_reason)
            return commander_module.stop(self.blocked_reason)

        self._watch_progress(ctx, now)

        if self.behaviour.done:
            self.plan.advance(why=self.behaviour.name)
            self.plan.save()
            self.behaviour = None
            self._last_waypoint_index = None

        self.reason = intent.reason
        return intent

    def _enter(self, waypoint, ctx, now):
        self.behaviour = behaviour_registry.for_role(waypoint.role, self.config)
        self.behaviour.start(ctx)
        self._last_waypoint_index = waypoint.index
        self._best_distance = None
        self._progress_at = now
        self.stuck = False
        log.info(
            "waypoint %s (%s): %s", waypoint.name, waypoint.role, self.behaviour.name
        )

    # ---------------------------------------------------------------- safety

    def _may_drive(self, state, now):
        """`(bool, why)`. The seven checks, in priority order."""
        if self.mode not in (RUNNING, BLOCKED):
            return True, ""  # nothing to block; the mode check happens above

        if state is None:
            return False, "no state from io_manager yet"

        age = state.age(now)
        if age > self.config.LOSS_OF_COMMS_STOP_S:
            return False, (
                f"no contact with io_manager for {age:.0f} s - stopping "
                f"(NJORD 7.3 requires this inside 60 s)"
            )

        if state.estop:
            return False, "emergency stop engaged - propulsion power is cut"

        if state.mode and str(state.mode).upper() in PILOTED_MODES:
            return False, (
                f"the autopilot is in {state.mode} - a pilot has the boat, "
                "standing down"
            )

        if state.armed is False:
            return False, "the vehicle is disarmed"

        if not state.usable:
            return False, f"cannot navigate: {state.why_unusable}"

        if age > self.config.MAX_NAV_AGE_S:
            return False, f"the position fix is {age:.1f} s old - stopping"

        if self.plan is None or len(self.plan) == 0:
            return False, "no plan loaded"

        return True, ""

    def _watch_progress(self, ctx, now):
        """Raise `stuck` when the boat stops getting closer. NJORD §8.2's window.

        Deliberately measured against the *best* distance so far rather than the
        last tick's: a boat that is going backwards and a boat that is holding
        station both fail to improve, and both are the same problem here.

        A behaviour that is legitimately not closing - holding station, sitting
        in a berth for ten seconds, standing on for a crossing vessel - says so
        by having no target distance, and is exempt.
        """
        distance = ctx.distance_to_target
        if distance is None or self.behaviour is None:
            return
        if getattr(self.behaviour, "phase", None) in ("hold", "exit"):
            self._progress_at = now
            return
        if self._best_distance is None or distance < self._best_distance - PROGRESS_M:
            self._best_distance = distance
            self._progress_at = now
            self.stuck = False
            return
        if now - (self._progress_at or now) > self.config.STUCK_WARN_S:
            if not self.stuck:
                log.warning(
                    "no progress towards %s for %.0f s (%.1f m to run) - the crew "
                    "have about %.0f s before NJORD 8.2 says take over",
                    ctx.waypoint.name,
                    now - self._progress_at,
                    distance,
                    max(0.0, 20.0 - (now - self._progress_at)),
                )
            self.stuck = True

    # -------------------------------------------------------------- commands

    def set_plan(self, payload, origin):
        """Load a plan from the operator. `(ok, message)` for the ack."""
        try:
            plan = Plan.parse(payload, origin)
        except PlanError as exc:
            return False, str(exc)
        was_running = self.mode == RUNNING
        self.plan = plan
        self.plan.save()
        self.behaviour = None
        self._last_waypoint_index = None
        if was_running:
            # A plan swapped in mid-run stops the boat rather than silently
            # continuing onto a course nobody has looked at yet.
            self.mode = PAUSED
            self.commander.disengage("new plan loaded")
            return True, (
                f"loaded {len(plan)} waypoint(s) - autonomy PAUSED, "
                "start it again when ready"
            )
        return True, f"loaded {len(plan)} waypoint(s) as {plan.name!r}"

    def start(self):
        if self.plan is None or len(self.plan) == 0:
            return False, "no plan loaded"
        if self.plan.finished:
            self.plan.reset()
        self.mode = RUNNING
        self.stuck = False
        self.started_at = None
        log.warning(
            "AUTONOMY ENGAGED: plan %r, %d waypoint(s), starting at %s",
            self.plan.name,
            len(self.plan),
            self.plan.current.name if self.plan.current else "?",
        )
        return True, f"running {self.plan.name!r} from {self.plan.current.name}"

    def stop(self, why="operator stopped autonomy"):
        self.mode = IDLE
        self.behaviour = None
        self._last_waypoint_index = None
        self.commander.disengage(why)
        log.warning("autonomy stopped: %s", why)
        return True, why

    def pause(self):
        if self.mode != RUNNING:
            return False, f"not running (mode is {self.mode})"
        self.mode = PAUSED
        return True, "paused - holding station"

    def resume(self):
        if self.mode != PAUSED:
            return False, f"not paused (mode is {self.mode})"
        self.mode = RUNNING
        return True, "resumed"

    def skip(self):
        """Advance past the current waypoint. The operator's "it is done" button."""
        if self.plan is None:
            return False, "no plan loaded"
        current = self.plan.current
        self.plan.advance(why="operator skipped")
        self.plan.save()
        self.behaviour = None
        self._last_waypoint_index = None
        return True, (
            f"skipped {current.name if current else '?'}; next is "
            f"{self.plan.current.name if self.plan.current else 'END'}"
        )

    def back(self):
        """Step back one waypoint - NJORD §8.2's re-entry behind the last good one."""
        if self.plan is None:
            return False, "no plan loaded"
        current = self.plan.rewind()
        self.plan.save()
        self.behaviour = None
        self._last_waypoint_index = None
        return True, f"back to {current.name if current else 'the start'}"

    def jump(self, index):
        if self.plan is None:
            return False, "no plan loaded"
        try:
            current = self.plan.jump_to(int(index))
        except (TypeError, ValueError):
            return False, "index must be a number"
        self.plan.save()
        self.behaviour = None
        self._last_waypoint_index = None
        return True, f"jumped to {current.name if current else 'END'}"

    # ------------------------------------------------------------- telemetry

    def telemetry(self, state, world):
        """`telemetry.autopilot` - the block NJORD §11.4 is really asking for.

        Everything here is chosen so a jury member reading it can answer "what
        is the boat doing and why" without asking anyone: the mode, the current
        waypoint and its role, the sentence the behaviour produced this tick,
        and what it can see.
        """
        block = {
            "mode": self.mode,
            "reason": self.reason,
            "stuck": self.stuck,
            "sees": world.summary() if world is not None else "no world model",
        }
        if self.blocked_reason:
            block["blocked"] = self.blocked_reason
        if self.plan is not None:
            block["plan"] = self.plan.telemetry()
            waypoint = self.plan.current
            if waypoint is not None and state is not None and state.origin:
                target = waypoint.world(state.origin)
                if target is not None and state.position is not None:
                    block["distance_to_waypoint"] = round(
                        geo.distance(state.position, target), 1
                    )
                    block["bearing_to_waypoint"] = round(
                        geo.bearing_to(state.position, target), 1
                    )
        if self.behaviour is not None:
            block.update(self.behaviour.telemetry())
        return block
