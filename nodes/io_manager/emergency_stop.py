"""The Pi's two GPIO lines: the E-stop relay, and the battery-slider homing pulse.

This is the *only* GPIO in ligmax-pi - everything else it touches is SocketCAN
(`can1`) or MAVLink (`/dev/ttyACM0`).

    BCM 24  out  homing trigger  ->  slider ESP32 GPIO 35 (INPUT_PULLDOWN, RISING)
    BCM 25  out  E-stop relay    ->  relay in series with the physical E-stop button

Polarity matters on both lines, and it runs the opposite way on each:

  * **Homing is edge-triggered, so it wants a pulse, not a level.**
    `battery_slider.ino:78-83` arms on a RISING edge (`:115`), then hunts for the
    optical centre endstop. One second high is far more than the ESP32 needs and
    is long enough to see on a meter.

  * **The E-stop relay is fail-safe, so it is active-high.** HIGH = relay closed
    = propulsion permitted. LOW = relay open = propulsion power cut. The relay
    sits in series with the physical button, so either one opening cuts power
    (technical report §2.6, docs/hardware.md). A crashed process, a wedged Pi or
    a pulled plug all leave this pin unpowered, which is the safe state - that is
    the entire reason for this polarity. Nothing here may ever fail *closed*.

    The corollary is worth knowing before you meet it on the water: when this
    node exits, the relay opens. The supervisor restarting the node therefore
    bounces propulsion power. That is deliberate - if nothing is running, nothing
    is watching - but it means an update that restarts main.py cuts thrust for as
    long as the restart takes (../../update.py).

Pin numbers are **BCM**, which is both what gpiozero takes and what "GPIO 24"
means on a pinout diagram: BCM 24 is physical header pin 18, BCM 25 is physical
pin 22. Override either with `LIGMAX_HOMING_PIN` / `LIGMAX_ESTOP_PIN` if the
harness says otherwise - measure before you believe a number in a docstring.

**Raspberry Pi 5:** RPi.GPIO does not work on a Pi 5 at all. gpiozero on the
lgpio backend does, which is why those two are the GPIO entries in
requirements.txt. On a machine with no GPIO at all (a laptop, or before the
libraries are installed) every write here degrades to a logged no-op, so the
node still runs - and `available` reports False rather than pretending there is
a safety loop.
"""

import logging
import os
import threading
import time

log = logging.getLogger("io_manager.gpio")

HOMING_PIN = int(os.environ.get("LIGMAX_HOMING_PIN", "24"))
ESTOP_RELAY_PIN = int(os.environ.get("LIGMAX_ESTOP_PIN", "25"))

# The ESP32 only needs an edge; a second is for the human with the multimeter.
HOMING_PULSE_S = float(os.environ.get("LIGMAX_HOMING_PULSE_S", "1.0"))
# Every rising edge restarts the search, so a double-click would leave the rail
# hunting instead of homing. Refuse the second press.
HOMING_COOLDOWN_S = float(os.environ.get("LIGMAX_HOMING_COOLDOWN_S", "5.0"))

try:
    from gpiozero import DigitalOutputDevice
except ImportError:  # not a Pi, or the libraries are not installed yet
    DigitalOutputDevice = None


def _output(pin, initial_value):
    """A gpiozero output on `pin`, or None if this machine has no GPIO.

    gpiozero raises on *construction* rather than on import when there is no pin
    factory, so this is where a laptop finds out.
    """
    if DigitalOutputDevice is None:
        log.warning(
            "gpiozero is not installed - BCM %s is a no-op "
            "(pip install gpiozero lgpio)",
            pin,
        )
        return None
    try:
        return DigitalOutputDevice(pin, active_high=True, initial_value=initial_value)
    except Exception as exc:  # noqa: BLE001 - BadPinFactory, busy pin, permissions
        log.error("could not claim BCM %s (%s) - it will be a no-op", pin, exc)
        return None


class _Line:
    """Shared plumbing: one output pin that tolerates not existing."""

    def __init__(self, pin, initial_value):
        self.pin = pin
        self._device = _output(pin, initial_value)

    @property
    def available(self):
        """False when there is no real pin behind this object."""
        return self._device is not None

    def _write(self, high):
        if self._device is None:
            return False
        try:
            self._device.value = 1 if high else 0
            return True
        except Exception as exc:  # noqa: BLE001 - a GPIO fault must not kill the node
            log.error("write to BCM %s failed: %s", self.pin, exc)
            return False

    def close(self):
        device, self._device = self._device, None
        if device is not None:
            try:
                device.close()  # releases the pin; it reverts to an input
            except Exception:  # noqa: BLE001
                pass


class EstopRelay(_Line):
    """The software half of the safety loop. HIGH permits propulsion, LOW cuts it.

    Latching by design: once engaged it stays engaged until `clear()` is called,
    so a dropped uplink or a restarted dashboard cannot quietly restore power.
    """

    def __init__(self, pin=ESTOP_RELAY_PIN):
        # Closed on start: propulsion is permitted until someone stops it. If you
        # would rather the vessel come up stopped and require an explicit clear,
        # this True is the one value to change.
        super().__init__(pin, initial_value=True)
        self.engaged = False
        if self.available:
            log.info("E-stop relay ready on BCM %s (high = propulsion permitted)", pin)

    def engage(self, reason="operator"):
        """Cut propulsion power. Safe to call when already engaged.

        Returns `(ok, message)`, and `ok` is False when the pin did not move -
        an E-stop that only updated a variable must not report success.
        """
        already = self.engaged
        self.engaged = True  # true regardless, so telemetry reflects the intent
        wrote = self._write(False)
        log.critical(
            "E-STOP ENGAGED (%s): BCM %s driven low, propulsion power cut%s",
            reason,
            self.pin,
            "" if wrote else " - NO GPIO, THE RELAY DID NOT MOVE",
        )
        if not wrote:
            return False, f"NO GPIO on BCM {self.pin} - the relay did not move"
        return True, "already engaged" if already else "relay open, propulsion power cut"

    def clear(self, reason="operator"):
        """Re-close the relay. Propulsion becomes possible again."""
        self.engaged = False
        wrote = self._write(True)
        log.warning(
            "E-stop cleared (%s): BCM %s driven high, propulsion permitted again%s",
            reason,
            self.pin,
            "" if wrote else " - NO GPIO, the relay did not move",
        )
        if not wrote:
            return False, f"NO GPIO on BCM {self.pin} - the relay did not move"
        return True, "relay closed, propulsion permitted"

    def close(self):
        """Open the relay on the way out - see the fail-safe note in the docstring."""
        if self.available:
            log.warning(
                "io_manager exiting: opening the E-stop relay on BCM %s", self.pin
            )
            self._write(False)
        super().close()


class BatteryHoming(_Line):
    """One-shot homing pulse to the battery-slider ESP32.

    The slider homes itself once on boot (`battery_slider.ino:118`); this is how
    it gets told to do it again. While it is homing it ignores the Pixhawk's
    position demand entirely (`:152-155`), so pulsing this mid-mission suspends
    pitch trim until the search finishes.
    """

    def __init__(self, pin=HOMING_PIN, pulse_s=HOMING_PULSE_S):
        super().__init__(pin, initial_value=False)
        self.pulse_s = pulse_s
        self._lock = threading.Lock()
        self._timer = None
        self._last_trigger = 0.0
        if self.available:
            log.info("homing line ready on BCM %s (%.1f s pulse)", pin, pulse_s)

    def trigger(self, reason="operator"):
        """Pulse the line high, then drop it. Returns `(ok, message)`.

        Non-blocking: the drop happens on a timer, so the caller's control loop
        keeps running through the pulse.
        """
        with self._lock:
            now = time.monotonic()
            waited = now - self._last_trigger
            if self._last_trigger and waited < HOMING_COOLDOWN_S:
                message = f"ignored, homing was triggered {waited:.1f} s ago"
                log.warning("battery homing %s", message)
                return False, message

            # The cooldown starts only once a pulse really goes out, so a failed
            # attempt does not lock out the retry.
            if not self._write(True):
                return False, f"NO GPIO on BCM {self.pin} - no pulse was sent"
            self._last_trigger = now

            if self._timer is not None:
                self._timer.cancel()
            self._timer = threading.Timer(self.pulse_s, self._drop)
            self._timer.daemon = True
            self._timer.start()

        log.info(
            "battery homing triggered (%s): BCM %s high for %.1f s",
            reason,
            self.pin,
            self.pulse_s,
        )
        return True, f"{self.pulse_s:.1f} s pulse sent on BCM {self.pin}"

    def _drop(self):
        with self._lock:
            self._timer = None
            self._write(False)
        log.info("battery homing pulse ended, BCM %s low", self.pin)

    def close(self):
        with self._lock:
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
        self._write(False)
        super().close()


if __name__ == "__main__":
    # Bench check on the Pi:  python -m nodes.io_manager.emergency_stop
    #   BCM 24 pulses high for a second (the slider should start hunting),
    #   then BCM 25 goes low for two seconds (the contactor should drop out).
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s %(message)s")
    relay = EstopRelay()
    homing = BatteryHoming()
    try:
        print(f"homing line available: {homing.available}")
        print(homing.trigger(reason="bench test"))
        time.sleep(HOMING_PULSE_S + 0.5)
        print(f"estop relay available: {relay.available}")
        print(relay.engage(reason="bench test"))
        time.sleep(2.0)
        print(relay.clear(reason="bench test"))
        time.sleep(0.5)
    finally:
        homing.close()
        relay.close()
