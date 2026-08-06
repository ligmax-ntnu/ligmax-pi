"""Notice when the physical E-stop has cut propulsion, by watching the VESCs.

Why this exists
---------------
The physical E-stop is a mushroom button in series with the relay on the *power*
side (`emergency_stop.py:7`). The Pi drives that relay but has no input that
senses the button, so pressing it cuts propulsion while every piece of software on
board carries on believing propulsion is permitted. `status.py` left the door open
for exactly this: `evaluate()` takes a `propulsion_permitted` argument whose
docstring says it "exists so a caller that knows better - a physical E-stop the Pi
cannot see, say - can say so". This is the caller that knows better.

Measured on the bench, 2026-08-06
---------------------------------
The two VESC 6.7s are on the Pixhawk's CAN bus, not the Pi's, so the Pi cannot
watch them directly - it reads ArduPilot's `ESC_TELEMETRY_1_TO_4` over MAVLink.
Pressing the button produced this, at 1 Hz:

    t=0..34s   count +48..50/s   ESC input 47.9 V / 48.5 V
    t=34s      count +49         32.6 V / 32.5 V     <- rail collapsing
    t=35s      count +18/+17     6.4 V / 7.1 V       <- dying mid-second
    t=42s      no ESC_TELEMETRY at all
    then       the Pixhawk's own USB disconnected

**The Pixhawk is on the same rail as the VESCs and dies with them.** That is a
wiring fault, not a design: the autopilot should survive the E-stop so it can keep
reporting, keep logging, and keep the RC link up while propulsion is cut. It is
expected to be rewired. This module is built so that either way works, and so that
fixing the wiring makes it *better* rather than breaking it:

  * The **ESC counters freezing** is the real signal - specific, and it arrives
    first (about 1 s in, versus 7 s for the link). It is the only one that fires if
    the autopilot survives, and it is the one to keep.
  * The **MAVLink link dropping** is the fallback, and it only exists because of
    the shared rail. Once the Pixhawk is on its own supply this stops firing for
    E-stops, and goes back to meaning what it should mean - a dead USB cable.
    Leaving it in costs nothing and covers the boat as it is wired today.

Relying only on the second would be a detector that goes blind at the moment it
matters most; relying only on the first would miss nothing once the rewire is done,
but today it would be racing a Pixhawk that is about to vanish.

What it is honest about
-----------------------
Frozen counters mean "the VESCs are not talking". The button is the *likeliest*
reason, not the only one - a pulled CAN plug, a blown fuse, a VESC that faulted
mid-run all look identical from here. So nothing here claims to have detected the
button. It reports that propulsion is not *confirmed available*, which is the part
that is true and also the part that matters: under every one of those causes the
thrusters are not going to answer.

Three states, not two
---------------------
    unknown   no ESC telemetry has ever arrived. Reported as None, which
              `status.py` reads as "no opinion" and falls back to the relay.
    alive     at least one VESC's counter advanced within FROZEN_TIMEOUT_S.
    lost      it was alive, and every VESC that had been reporting has now been
              frozen for CONFIRM_S - or the MAVLink link went away.

The unknown state is the important one. Treating "never seen it" as "E-stop
pressed" would put the hull in KILLED red on every bench boot with no autopilot
attached, and a warning that fires when nothing is wrong is one the crew learns to
ignore. This only ever contradicts the relay after it has positively heard the
VESCs and then lost them.

Why *every* VESC has to go quiet
--------------------------------
The E-stop cuts both rails at once, so requiring all of them is the tighter test
for the button specifically and does not cry wolf when one CAN node drops. The
cost is that a single genuine motor failure is not reported here - `telemetry()`
names the frozen ESCs individually so the operator can still see it.

Note on `count`
---------------
It is ArduPilot's cumulative tally of telemetry packets received from that ESC,
and it is a **uint16 that wraps**: the capture above rolled 65530 -> 43 between
two samples. So advance is detected by inequality, never by `>`. A `>` test reads
every wrap as a freeze, which on a 50 Hz counter is a false E-stop about every 22
minutes.
"""

import logging
import time

log = logging.getLogger("io_manager.propulsion")

# ESC_TELEMETRY_1_TO_4 arrives at about 1 Hz, and the counters advance ~50 per
# second, so one stale sample is already unambiguous. Two seconds is two missed
# messages - generous, because this decides whether the boat declares itself
# killed.
FROZEN_TIMEOUT_S = 2.0

# How long the freeze has to persist before it is believed. The E-stop is a
# mechanical switch cutting a power rail, so the real thing is permanent and
# waiting costs nothing - whereas a single dropped message flipping the boat to
# KILLED and back would strobe the hull and the dashboard both.
CONFIRM_S = 1.0

UNKNOWN = "unknown"
ALIVE = "alive"
LOST = "lost"


class PropulsionWatch:
    """Whether propulsion is confirmed available, from the VESCs' CAN telemetry.

    Fed from the MAVLink pump in `main.py`, like `StatusMachine`. Nothing here
    talks to hardware or owns a thread, which is what makes it testable without a
    boat - `clock` is injectable for exactly that.
    """

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        # esc index -> [last count value, when it last changed]
        self._counts = {}
        self._ever_seen = False
        self._link_down = False
        self._link_down_at = None
        self._state = UNKNOWN
        self._reason = "no ESC telemetry from the autopilot yet"

    # -- fed by the MAVLink pump -------------------------------------------

    def note_esc_telemetry(self, message):
        """An ESC_TELEMETRY_1_TO_4. Records which ESCs' counters moved.

        Only ESCs that have actually reported are tracked: the message always
        carries four slots and this boat has two VESCs, so the trailing zeros are
        empty slots rather than dead controllers. An empty slot must never count
        as a frozen one, or the detector would sit in LOST for ever.
        """
        counts = getattr(message, "count", None) or ()
        voltages = getattr(message, "voltage", None) or ()
        now = self._clock()
        self._link_down = False
        self._link_down_at = None

        for index, count in enumerate(counts):
            voltage = voltages[index] if index < len(voltages) else 0
            known = index in self._counts
            if not known:
                # An empty slot reads as count 0 and voltage 0 for ever. Requiring
                # one of them to be non-zero is what tells a real controller from
                # a slot this airframe does not use.
                if not count and not voltage:
                    continue
                self._counts[index] = [count, now]
                self._ever_seen = True
                log.warning(
                    "ESC %d is reporting (%.1f V) - physical E-stop detection is "
                    "now armed", index + 1, voltage / 100.0
                )
                continue

            # Inequality, not `>`: this counter is a uint16 and wraps. See the
            # module docstring.
            if count != self._counts[index][0]:
                self._counts[index] = [count, now]

    def note_link_down(self):
        """The MAVLink link dropped.

        On this boat that is itself an E-stop symptom - the Pixhawk shares the rail
        the button cuts and dies a few seconds after the VESCs - so once the
        detector has been armed, losing the link counts as losing propulsion.

        It is deliberately not treated that way before the first ESC telemetry has
        ever arrived: on the bench, with no autopilot plugged in, that would be a
        permanent red hull.
        """
        if not self._link_down:
            self._link_down_at = self._clock()
        self._link_down = True

    # -- the decision -------------------------------------------------------

    def evaluate(self):
        """UNKNOWN / ALIVE / LOST. Cheap enough to call every loop pass.

        Both timeouts are measured against recorded timestamps - when the counter
        last moved, when the link went down - rather than against when this method
        first happened to notice. A debounce keyed off observation time silently
        depends on how often the caller polls: call it once every five seconds and
        the confirm window restarts on every call, so the state never settles.
        """
        now = self._clock()

        if not self._ever_seen:
            self._transition(UNKNOWN, "no ESC telemetry from the autopilot yet")
            return UNKNOWN

        if self._link_down:
            # No counter timestamp to lean on here, so the link-down moment is the
            # start of the window.
            quiet_for = now - (self._link_down_at or now)
            reason = (
                "the MAVLink link went away, and the Pixhawk shares the rail the "
                "E-stop cuts"
            )
            state = LOST if quiet_for >= CONFIRM_S else ALIVE
            if state == ALIVE:
                reason = f"holding alive before believing it: {reason}"
            self._transition(state, reason)
            return state

        # `max` because every controller has to be quiet for this to be the E-stop:
        # the most recently heard one is the one that decides. See the module
        # docstring on why the test is the strict one.
        quiet_for = now - max(changed_at for _, changed_at in self._counts.values())
        frozen = [
            index + 1
            for index, (_, changed_at) in sorted(self._counts.items())
            if (now - changed_at) >= FROZEN_TIMEOUT_S
        ]

        if quiet_for >= (FROZEN_TIMEOUT_S + CONFIRM_S):
            state = LOST
            reason = (
                f"ESC telemetry frozen on all {len(self._counts)} controllers for "
                f"{quiet_for:.1f} s - propulsion power is not confirmed"
            )
        elif quiet_for >= FROZEN_TIMEOUT_S:
            # Frozen, but not yet for long enough to believe. One dropped message
            # must not strobe the hull.
            state = ALIVE
            reason = (
                f"holding alive for another {FROZEN_TIMEOUT_S + CONFIRM_S - quiet_for:.1f} s: "
                f"ESC telemetry frozen on all {len(self._counts)} controllers"
            )
        else:
            state = ALIVE
            reason = (
                f"{len(self._counts) - len(frozen)} of {len(self._counts)} "
                f"controllers reporting"
            )
            if frozen:
                reason += f", frozen on ESC {', '.join(str(i) for i in frozen)}"

        self._transition(state, reason)
        return state

    def _transition(self, state, reason):
        self._reason = reason
        if state == self._state:
            return
        previous, self._state = self._state, state
        if state == LOST:
            log.critical(
                "PROPULSION LOST: %s. The physical E-stop is the likeliest cause; "
                "a pulled CAN plug or a blown fuse look the same from here.",
                reason,
            )
        elif state == ALIVE and previous == LOST:
            log.warning("propulsion is back: %s", reason)

    @property
    def propulsion_permitted(self):
        """What to pass to `StatusMachine.evaluate()`.

        `None` means "no opinion, use the relay" and is what a detector that has
        never heard the VESCs must say. `False` is a positive assertion that
        propulsion is gone, and is only ever returned after they have been heard
        and lost.
        """
        state = self.evaluate()
        if state == UNKNOWN:
            return None
        return state == ALIVE

    # -- what goes up the link ---------------------------------------------

    def telemetry(self):
        """The `telemetry.propulsion` block: why propulsion is believed gone.

        An operator looking at an unexplained red hull needs to tell a pressed
        E-stop from a CAN cable somebody kicked, and this module cannot tell those
        apart - so it publishes the evidence and lets them. `frozen` names the
        individual controllers, which is the only place a single dead VESC shows
        up at all, since the status test requires all of them.
        """
        state = self.evaluate()
        now = self._clock()
        block = {
            "state": state,
            "reason": self._reason,
            "controllers": len(self._counts),
        }
        frozen = [
            index + 1
            for index, (_, changed_at) in sorted(self._counts.items())
            if (now - changed_at) >= FROZEN_TIMEOUT_S
        ]
        if frozen:
            block["frozen"] = frozen
        if not self._ever_seen:
            # Says plainly that the detector is not armed, so nobody reads a quiet
            # bus as a confirmed all-clear.
            block["note"] = "no ESC telemetry yet - no E-stop detection"
        return block
