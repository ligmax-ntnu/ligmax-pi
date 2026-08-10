"""How hard the boat is being driven, as one switchable object.

    run = RunMode(config)
    run.set_profile("fast")          # from the operator, mid-run if they like
    run.profile.ceiling_ms           # what commander.py holds every command to
    run.profile.cruise_ms            # what a behaviour asks for on an open leg

Why there are profiles at all
-----------------------------
NJORD gives two attempts at each subtask (§8.2) and the marks do not move
between them. That makes the attempts *different problems*, and driving them the
same way wastes one of them:

    survey   1 kn. Slow enough that every mark gets the dozen sweeps
             `TRACK_ESTABLISH_HITS` wants and the camera gets the four agreeing
             frames `CARDINAL_VOTES_REQUIRED` wants. The output of this attempt
             is not just a score, it is `survey.json` - the map attempt two
             starts from.
    normal   the tuned defaults. What the boat does when nobody has said
             otherwise, and what a mid-competition reboot comes back up in.
    fast     up to the 5 kn vessel limit, driven off the survey. The lidar will
             see less at this speed - a 40 cm mark at 10 m subtends 2.3 degrees
             and goes past in a couple of seconds - and that is an accepted cost,
             because the map already knows where everything is. In exchange the
             marks get a wider berth (`clearance_per_ms`), because clearance is a
             time budget wearing metres and speed spends it.

Three properties this file is built to have
-------------------------------------------
**A profile can never raise the vessel limit.** Every speed here comes through
`config._speed`, which clamps to `SPEED_LIMIT_MS` as it reads, and
`commander.py` clamps again on the way out. The fast profile raises the boat's
*self-imposed* ceiling from `MAX_SPEED_MS` to the limit; it cannot go past it,
and there is no name in the environment that would let it.

**Selecting one is an act, not a default.** `DEFAULT_PROFILE` is "normal" and a
boat that reboots between attempts comes back up in it rather than in whatever
was selected before the power went. Coming back up at 5 kn on a course it has
not surveyed is the accident this prevents.

**The name is on the wire.** `telemetry()` puts the profile, its ceiling in
knots, and the switches on the operator's panel and into the trip recording,
because "why was it going that fast" has to be answerable afterwards from the
file rather than from somebody's memory of which button they pressed.
"""

from __future__ import annotations

import logging

from .config import (
    ALTERNATION_DEFAULT,
    CAREFUL_SPEED_MS,
    CRUISE_SPEED_MS,
    CAUTION_SPEED_MS,
    DEFAULT_PROFILE,
    FAST_CAUTION_SPEED_MS,
    FAST_CEILING_MS,
    FAST_CLEARANCE_PER_MS,
    FAST_CRUISE_SPEED_MS,
    KNOT_MS,
    MAX_SPEED_MS,
    SPEED_LIMIT_MS,
)

log = logging.getLogger("self_driving.profiles")

SURVEY = "survey"
NORMAL = "normal"
FAST = "fast"


class Profile:
    """One set of speeds, plus how much extra room they cost.

    Immutable on purpose. A profile is selected, never edited: an operator who
    wants different numbers changes the environment and restarts, which leaves a
    record, rather than nudging a live value nothing writes down.
    """

    __slots__ = ("name", "ceiling_ms", "cruise_ms", "caution_ms",
                 "clearance_per_ms", "description")

    def __init__(self, name, ceiling_ms, cruise_ms, caution_ms,
                 clearance_per_ms, description):
        # Every one of these is clamped to the vessel limit a second time here.
        # `config._speed` already did it; this is the cheap belt to that braces,
        # and it also covers the `min` against the ceiling below.
        self.name = name
        self.ceiling_ms = min(SPEED_LIMIT_MS, float(ceiling_ms))
        # A profile cannot ask for more than its own ceiling. Without this a
        # mistyped override could have a behaviour planning at a speed the
        # commander would then silently clamp - and a behaviour that plans at a
        # speed it does not get reasons about arrival times that never happen.
        self.cruise_ms = min(self.ceiling_ms, float(cruise_ms))
        self.caution_ms = min(self.ceiling_ms, float(caution_ms))
        self.clearance_per_ms = max(0.0, float(clearance_per_ms))
        self.description = description

    @property
    def ceiling_kn(self):
        return self.ceiling_ms / KNOT_MS

    def telemetry(self):
        return {
            "profile": self.name,
            "ceiling_ms": round(self.ceiling_ms, 3),
            "ceiling_kn": round(self.ceiling_kn, 2),
            "cruise_ms": round(self.cruise_ms, 2),
            "caution_ms": round(self.caution_ms, 2),
            "clearance_per_ms": round(self.clearance_per_ms, 2),
            "description": self.description,
        }

    def __repr__(self):
        return f"<Profile {self.name} ceiling={self.ceiling_ms:.2f} m/s>"


PROFILES = {
    SURVEY: Profile(
        SURVEY,
        ceiling_ms=CAREFUL_SPEED_MS,
        cruise_ms=CAREFUL_SPEED_MS,
        caution_ms=CAREFUL_SPEED_MS,
        # No speed term: at 1 kn the static clearance is already four seconds of
        # water, and the survey attempt wants the marks *close* enough to be
        # measured well, not held at arm's length.
        clearance_per_ms=0.0,
        description="1 kn - survey the course and build the map",
    ),
    NORMAL: Profile(
        NORMAL,
        ceiling_ms=MAX_SPEED_MS,
        cruise_ms=CRUISE_SPEED_MS,
        caution_ms=CAUTION_SPEED_MS,
        # Zero, and see `config.FAST_CLEARANCE_PER_MS` for why: Task 2's gates
        # are 5 m wide and any speed term at all would make the boat refuse one.
        clearance_per_ms=0.0,
        description="the tuned defaults",
    ),
    FAST: Profile(
        FAST,
        ceiling_ms=FAST_CEILING_MS,
        cruise_ms=FAST_CRUISE_SPEED_MS,
        caution_ms=FAST_CAUTION_SPEED_MS,
        clearance_per_ms=FAST_CLEARANCE_PER_MS,
        description="up to 5 kn off the surveyed map, wide berths",
    ),
}

#: Careful mode, the operator's existing one-knot switch, is the survey profile
#: under its older name. Kept as an alias rather than as a second mechanism,
#: because two things that both mean "hold it to a knot" eventually disagree.
CAREFUL = SURVEY


def _resolve(name):
    key = str(name or "").strip().lower()
    return PROFILES.get(key)


class RunMode:
    """The profile in force, plus the optional behaviours switched on with it.

    Lives on the `Commander` because that is where the speed ceiling is actually
    enforced, and a flag that lives anywhere other than its enforcement point
    eventually disagrees with it. The non-speed switches ride along because they
    are the same kind of thing - a decision about how this attempt is being run,
    made once by an operator and needing to be visible in the telemetry
    afterwards - and because one object is one thing to get right.
    """

    def __init__(self, config=None):
        self._config = config
        self.profile = _resolve(DEFAULT_PROFILE) or PROFILES[NORMAL]
        if _resolve(DEFAULT_PROFILE) is None and DEFAULT_PROFILE != NORMAL:
            log.warning(
                "LIGMAX_AP_PROFILE=%r is not a profile (%s) - running %s",
                DEFAULT_PROFILE,
                ", ".join(sorted(PROFILES)),
                NORMAL,
            )
        # The cardinal alternation prior. See `behaviours/alternation.py`; off
        # unless deliberately switched on.
        self.alternation = bool(ALTERNATION_DEFAULT)

    # ------------------------------------------------------------- selection

    def set_profile(self, name):
        """Switch profile. `(ok, message)` for the operator's ack."""
        profile = _resolve(name)
        if profile is None:
            return False, (
                f"{name!r} is not a run profile - "
                f"pick one of {', '.join(sorted(PROFILES))}"
            )
        was = self.profile
        self.profile = profile
        if was.name == profile.name:
            return True, f"already running the {profile.name} profile"
        # WARNING rather than INFO for all of them, not just for `fast`. The
        # question this line answers after the fact is "what was it set to when
        # that happened", and an answer that is only in the log at the default
        # level for one of the three values is not an answer.
        log.warning(
            "RUN PROFILE %s -> %s: %.2f m/s (%.1f kn) ceiling, %s",
            was.name,
            profile.name,
            profile.ceiling_ms,
            profile.ceiling_kn,
            profile.description,
        )
        return True, (
            f"{profile.name} profile - {profile.ceiling_kn:.1f} kn "
            f"({profile.ceiling_ms:.2f} m/s) maximum, {profile.description}"
        )

    def set_alternation(self, on):
        """Switch the cardinal alternation prior. `(ok, message)`."""
        was = self.alternation
        self.alternation = bool(on)
        if was == self.alternation:
            return True, (
                f"the alternation prior was already "
                f"{'on' if self.alternation else 'off'}"
            )
        if self.alternation:
            log.warning(
                "ALTERNATION PRIOR ON - an uncommitted cardinal may now be "
                "passed on the side the previous mark implies"
            )
            return True, (
                "alternation prior ON - a cardinal the camera has not committed "
                "will be passed on the opposite side to the mark before it, and "
                "said so on the panel"
            )
        log.warning("alternation prior OFF")
        return True, (
            "alternation prior OFF - an uncommitted cardinal holds the planned "
            "line and slows down instead of guessing"
        )

    # -------------------------------------------------------------- careful

    @property
    def careful(self):
        """Whether the one-knot ceiling is in force. The older switch's name."""
        return self.profile.name == SURVEY

    # ------------------------------------------------------------- telemetry

    def telemetry(self):
        block = dict(self.profile.telemetry())
        block["careful"] = self.careful
        block["alternation"] = self.alternation
        return block
