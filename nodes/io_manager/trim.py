"""What the stabilisation actuators are being told to do: the slider and the amas.

Both of the "nice to have" items on the GUI list - battery slider position, and
what the ama motors are doing - come from the same place: the Pixhawk's servo rail.
Neither actuator reports anything back.

    battery slider   `battery_slider.ino` reads a position demand as a PWM pulse
                     on its GPIO 34 and drives a DM542C stepper. It has three
                     endstops and it knows exactly where the rail is. It tells
                     nobody: there is no return path to the Pi at all, only the
                     homing input the Pi pulses (docs/hardware.md).
    amas             `amas.lua` runs a roll PD controller on the flight controller
                     and writes two servo outputs; the translator ESP32 measures
                     those pulse widths and drives an H-bridge. Also no return path.

So everything here is **commanded, not measured**, and every field says so through
`rail_source`. That distinction is the point of this module rather than a caveat on
it: an operator watching a rail position that is really a demand needs to know that
a stuck slider would show as perfectly obedient.

Why the servo channels are configuration and not constants
----------------------------------------------------------
`amas.lua` claims outputs by *function ID* - `SERVOn_FUNCTION` 94 and 95, Scripting1
and Scripting2 - and which physical MAIN/AUX pin carries each is a flight-controller
parameter that **exists nowhere in git** (docs/findings.md item 10). The slider's
channel is not recorded either.

Rather than guess, this module publishes nothing until it is told:

    LIGMAX_AMA_PORT_CH        servo output carrying SERVO_FUNCTION 94 (Scripting1)
    LIGMAX_AMA_STARBOARD_CH   servo output carrying SERVO_FUNCTION 95 (Scripting2)
    LIGMAX_SLIDER_CH          servo output feeding battery_slider.ino's GPIO 34

Read them off the flight controller once, put them in `/etc/ligmax/node.env`, and
record them in `docs/hardware.md` while you are there. Unset means the field is
absent, which reads on the dashboard as "not wired up" - the honest answer, and
better than a number pulled off whichever channel happened to be first.

Turning pulses into millimetres
-------------------------------
`battery_slider.ino` counts *steps*: `MAX_TRAVEL_BACK` 3200 aft of the optical
centre and `MAX_TRAVEL_FRONT` 5000 forward. The steps-per-millimetre figure is a
property of the leadscrew and is not recorded anywhere in git either, so millimetres
cannot be derived. What can be derived, entirely from the sketch, is the fraction of
travel the demand corresponds to - so `battery_rail_pct` is always published and
`battery_rail_mm` only when someone supplies LIGMAX_SLIDER_MM_PER_STEP.
"""

import logging
import os

log = logging.getLogger("io_manager.trim")


def _channel(name):
    """A servo output number from the environment, or None if unset."""
    raw = os.environ.get(name, "").strip()
    if not raw:
        return None
    try:
        channel = int(raw)
    except ValueError:
        log.error("%s=%r is not a channel number, ignoring it", name, raw)
        return None
    if not 1 <= channel <= 16:
        log.error("%s=%s is not in 1..16, ignoring it", name, channel)
        return None
    return channel


AMA_PORT_CH = _channel("LIGMAX_AMA_PORT_CH")
AMA_STARBOARD_CH = _channel("LIGMAX_AMA_STARBOARD_CH")
SLIDER_CH = _channel("LIGMAX_SLIDER_CH")

# Straight from the sketches, and the reason this module can say anything useful
# without a calibration of its own.
#
#   amas.lua:41-46      output = 1500 -/+ roll_output + height_offset, clamped
#                       1000..2000. So 1500 is neutral and the two outputs are
#                       anti-symmetric in roll, common-mode in height.
#   battery_slider.ino  1500 us neutral with a +/-30 us deadband (PWM_DEADBAND),
#                       and 3200 steps of aft travel against 5000 forward.
PWM_NEUTRAL_US = 1500.0
PWM_MIN_US = 1000.0
PWM_MAX_US = 2000.0
SLIDER_DEADBAND_US = 30.0
SLIDER_STEPS_BACK = 3200
SLIDER_STEPS_FRONT = 5000

# The rail is not centred in its travel, so "50 %" is not the middle: the optical
# home sits 3200 steps from the aft limit and 5000 from the forward one.
SLIDER_HOME_FRACTION = SLIDER_STEPS_BACK / (SLIDER_STEPS_BACK + SLIDER_STEPS_FRONT)

# Optional, because it cannot be derived from anything in the repos. Set it after
# measuring the leadscrew and the number appears in millimetres.
MM_PER_STEP = os.environ.get("LIGMAX_SLIDER_MM_PER_STEP", "").strip()
try:
    MM_PER_STEP = float(MM_PER_STEP) if MM_PER_STEP else None
except ValueError:
    log.error("LIGMAX_SLIDER_MM_PER_STEP=%r is not a number, ignoring it", MM_PER_STEP)
    MM_PER_STEP = None

# Output within this of a rail counts as saturated. `amas.lua` clamps to
# 1000..2000 and a full-travel height command uses all of it, leaving no roll
# authority - which looks exactly like the roll loop having died
# (docs/findings.md item 10), so it gets flagged rather than left to be diagnosed.
SATURATION_MARGIN_US = 8.0


class Trim:
    """Reads SERVO_OUTPUT_RAW and reports what the trim actuators were told.

    Fed from the MAVLink pump like `Navigation`. Publishes nothing at all when no
    channels are configured, so an un-mapped flight controller produces an absent
    panel rather than a wrong one.
    """

    def __init__(self):
        self._servo = None
        self._homing = False
        self._announced = False

    @property
    def configured(self):
        return any((AMA_PORT_CH, AMA_STARBOARD_CH, SLIDER_CH))

    def handle(self, message):
        """Absorb one MAVLink message. Returns True if it was one of ours."""
        if message.get_type() != "SERVO_OUTPUT_RAW":
            return False
        # Only the first servo bank. A second SERVO_OUTPUT_RAW instance covers
        # channels 9-16 and would otherwise overwrite the first one's values.
        if getattr(message, "port", 0) != 0:
            return False
        self._servo = message
        if not self._announced and self.configured:
            self._announced = True
            log.info(
                "trim readback: ama port=ch%s starboard=ch%s slider=ch%s "
                "(commanded values, nothing measures these actuators)",
                AMA_PORT_CH,
                AMA_STARBOARD_CH,
                SLIDER_CH,
            )
        return True

    def note_homing(self, homing):
        """Told by `main.py` when the slider homing line has been pulsed.

        Worth surfacing because while the slider is homing it ignores the
        Pixhawk's position demand entirely (`battery_slider.ino:152-155`), so the
        commanded figure below is not even what the rail is chasing.
        """
        self._homing = bool(homing)

    def link_down(self):
        self._servo = None

    def _pulse(self, channel):
        """`servoN_raw` in microseconds, or None. 0 means the channel is unused."""
        if channel is None or self._servo is None:
            return None
        value = getattr(self._servo, f"servo{channel}_raw", None)
        if value is None or value == 0:
            return None
        return float(value)

    def telemetry(self):
        """The `telemetry.trim` block. `{}` when nothing is configured or seen."""
        out = {}

        port = self._pulse(AMA_PORT_CH)
        starboard = self._pulse(AMA_STARBOARD_CH)
        if port is not None or starboard is not None:
            out.update(self._amas(port, starboard))

        slider = self._pulse(SLIDER_CH)
        if slider is not None:
            out.update(self._slider(slider))
        if out:
            out["rail_source"] = "commanded"
            out["rail_homing"] = self._homing
        return out

    def _amas(self, port, starboard):
        """Split the two outputs back into the roll and height they were mixed from.

        `amas.lua` writes `1500 - roll + height` and `1500 + roll + height`, so the
        half-difference recovers the roll correction and the mean recovers the ride
        height. That is the pair worth showing: the raw microseconds say nothing,
        and "levelling hard while lifting" is a sentence an operator can act on.
        """
        out = {}
        if port is not None:
            out["ama_port_us"] = round(port)
        if starboard is not None:
            out["ama_starboard_us"] = round(starboard)

        if port is None or starboard is None:
            # One channel alone cannot be decomposed - roll and height are only
            # separable from the pair. Report what there is and stop.
            return out

        roll_us = (starboard - port) / 2.0
        height_us = (starboard + port) / 2.0 - PWM_NEUTRAL_US
        out["ama_roll_us"] = round(roll_us)
        out["ama_height_us"] = round(height_us)

        saturated = any(
            value <= PWM_MIN_US + SATURATION_MARGIN_US
            or value >= PWM_MAX_US - SATURATION_MARGIN_US
            for value in (port, starboard)
        )
        out["ama_saturated"] = saturated

        levelling = abs(roll_us) > 25.0
        lifting = abs(height_us) > 20.0
        if saturated:
            # The one case that needs saying outright, because the symptom is that
            # the roll loop appears to have stopped working.
            out["ama_doing"] = "output at the rail — no roll authority left"
        elif levelling and lifting:
            out["ama_doing"] = (
                f"levelling {'to port' if roll_us > 0 else 'to starboard'}, "
                f"{'lifting' if height_us > 0 else 'lowering'}"
            )
        elif levelling:
            out["ama_doing"] = f"levelling {'to port' if roll_us > 0 else 'to starboard'}"
        elif lifting:
            out["ama_doing"] = "holding ride height"
        else:
            out["ama_doing"] = "neutral"
        return out

    def _slider(self, pulse):
        """The rail demand as a fraction of travel, and mm if that is calibrated."""
        out = {}
        offset = pulse - PWM_NEUTRAL_US
        if abs(offset) <= SLIDER_DEADBAND_US:
            # Inside PWM_DEADBAND the sketch holds position rather than moving, so
            # the demand really is "stay at home", not "a bit off centre".
            offset = 0.0

        # Signed fraction of the pulse range, -1 aft to +1 forward.
        span = PWM_MAX_US - PWM_NEUTRAL_US
        fraction = max(-1.0, min(1.0, offset / span))

        steps = fraction * (SLIDER_STEPS_FRONT if fraction >= 0 else SLIDER_STEPS_BACK)
        out["battery_rail_steps"] = round(steps)
        # 0 % is the aft limit and 100 % the forward one, so the bar on the
        # dashboard matches the physical rail rather than the pulse.
        travel = SLIDER_HOME_FRACTION + fraction * (
            (1.0 - SLIDER_HOME_FRACTION) if fraction >= 0 else SLIDER_HOME_FRACTION
        )
        out["battery_rail_pct"] = round(100.0 * travel, 1)
        if MM_PER_STEP is not None:
            out["battery_rail_mm"] = round(steps * MM_PER_STEP, 1)
        return out
