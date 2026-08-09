"""Ride-height command and a raw MAVLink dump. Not wired into `main.py` yet.

`amas.lua` reads its ride-height demand off **RC input channel 14** and adds the
remote's channel 3 to it (`HEIGHT_RC_CHAN`, `HEIGHT_RC_CHAN_CONTROLLER`), so an
override from here has to land on 14 and nowhere else. Channels 1..8 are the
sticks and the aux switches on this boat: an override there fights the pilot for
the same channel, which is why nothing in the fleet uses one.

There is a third way into the same sum, and it is the one the dashboard uses:
`SCR_USER6`, written as an ordinary parameter by `tuning.py`. The difference is
persistence, not effect. An override stops the moment this process does - the
autopilot times it out and the amas hold - whereas a parameter is stored on the
flight controller and the amas resume creeping after a reboot. Use the override
for "move it now while I watch", the parameter for a trim meant to stay set.
"""

import logging
import time

from pymavlink import mavutil

log = logging.getLogger(__name__)

# amas.lua's HEIGHT_RC_CHAN. RC channel numbers are 1-based and the override
# message is an array, hence the index below.
#
# 14 since 2026-08-09; it was 16, and the remote's was 15. The radio link puts
# its own telemetry on the top channels - 16 sat at a steady ~2006 us with
# nothing mapped to it, RSSI being the suspect - and amas.lua reads that channel
# as a VELOCITY, so an unmapped channel at full scale is a boat driving itself.
# This number and amas.lua's HEIGHT_RC_CHAN are two hardcoded copies of one fact
# in two repos: change one and you must change the other, and a mismatch is
# silent (the override lands on a channel the script does not read).
HEIGHT_CHANNEL = 14
CHANNEL_COUNT = 18

# RC_CHANNELS_OVERRIDE has three cases per channel and only two are obvious:
#
#   1000..2000   override the channel with this pulse
#   0            clear the override - the receiver's own input takes over again
#   65535        leave this channel's override state exactly as it is
#
# UNCHANGED is what every channel but ours gets, so this message can never take
# the sticks away. It is *not* a release: an override already standing on some
# other channel keeps standing. Clearing one takes an explicit CLEAR, or the
# autopilot's own RC_OVERRIDE_TIME timeout (3 s by default) once the refreshes
# stop. `autopilot_bridge.RC_RELEASE` is this same 65535 under a name that
# suggests otherwise; it works because it relies on that timeout.
UNCHANGED = 65535
CLEAR = 0

NEUTRAL_US = 1500
MIN_US = 1000
MAX_US = 2000

# An override the autopilot stops hearing about expires, so a standing command
# has to be re-sent. Same figure autopilot_bridge.py uses for the lateral
# thruster, and for the same reason: much longer and the actuator stutters.
REFRESH_S = 0.25


def set_ride_height(master, pwm_value, channel=HEIGHT_CHANNEL):
    """
    Sends a MAVLink RC Override on `channel`, `HEIGHT_CHANNEL` (14) by default.
    1000 = Pull Amas Up (Lower boat)
    1500 = Stop / Hold position
    2000 = Push Amas Down (Raise boat)

    The translator ESP32 reads this pulse as a *velocity*, so 1500 really does
    mean hold and anything else keeps moving for as long as it is sent.

    One shot. It expires unless something keeps calling it - that something is
    `RideHeight` below, which is what the node actually uses.
    """
    rc_channels = [UNCHANGED] * CHANNEL_COUNT
    rc_channels[channel - 1] = pwm_value

    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_channels
    )
    log.debug("ride height: %s us to channel %d", pwm_value, channel)


class RideHeight:
    """The ride-height override on channel 14, refreshed, and never abandoned mid-travel.

    `amas.lua` sums channel 14, the remote's channel 3 and `SCR_USER6` and
    hands the total to the translator ESP32, which reads it as a **velocity**.
    Two consequences shape this class, and neither is obvious:

    * **Stopping is not the same as letting go.** A command that simply stops
      being refreshed does not stop the amas - it hands channel 14 back to the
      receiver, and whatever the transmitter has parked there becomes the new
      command. On 2026-08-09 that was ~2006 us on a boat nobody was flying:
      a full-speed creep, indefinitely. So `stop()` holds 1500, which is the
      translator's own STOP, rather than releasing.

    * **Nothing is claimed until an operator asks.** Before the first command
      this node does not write channel 14 at all, so a boat that never uses the
      feature behaves exactly as it did before this existed. `release()` is the
      deliberate way back out, and it is a separate action from `stop()`
      precisely because it returns the channel to a resting value this node
      cannot see.

    What this cannot do anything about: if the process dies mid-travel the
    override expires in ~3 s and the receiver takes the channel back. There is
    no way to hold a wire from a dead process - that is the E-stop's job, and
    the reason the transmitter's channels 15/16 need to be centred regardless.
    """

    def __init__(self, channel=HEIGHT_CHANNEL):
        self.channel = channel
        self._pwm = None  # None: this node is not driving the channel at all
        self._last_sent = 0.0
        self._clear_pending = False

    @property
    def active(self):
        return self._pwm is not None

    @property
    def pwm(self):
        return self._pwm

    def command(self, pwm):
        """Drive the amas at `pwm`. Returns `(ok, message)` for the ack."""
        try:
            value = int(pwm)
        except (TypeError, ValueError):
            return False, f"'pwm' must be a number from {MIN_US} to {MAX_US}"
        if not MIN_US <= value <= MAX_US:
            return False, f"'pwm' must be {MIN_US}..{MAX_US}, not {value}"

        self._pwm = value
        self._clear_pending = False
        self._last_sent = 0.0  # goes out on the next tick, not in 250 ms
        if value == NEUTRAL_US:
            return True, f"channel {self.channel} held at {value} (stop)"
        direction = "down" if value > NEUTRAL_US else "up"
        # WARN, not INFO: this is a velocity, so the interesting part of the log
        # line is that something is moving and will keep moving.
        log.warning(
            "ride height: driving %s at %d us on channel %d until told otherwise",
            direction, value, self.channel,
        )
        return True, f"amas driving {direction} at {value} us - send 1500 to stop"

    def stop(self):
        """1500: the translator's STOP, held rather than released."""
        return self.command(NEUTRAL_US)

    def release(self):
        """Hand the channel back to the receiver. Deliberate, and not the same as stop."""
        if self._pwm is None:
            return True, f"channel {self.channel} was not being driven from here"
        self._pwm = None
        self._clear_pending = True
        self._last_sent = 0.0
        log.warning(
            "ride height: releasing channel %d - the receiver's own value applies "
            "from here, and amas.lua reads it as a velocity",
            self.channel,
        )
        return True, f"channel {self.channel} released to the receiver"

    def refresh(self, master, now=None):
        """Re-send the standing command. Call every loop tick; cheap when idle."""
        if master is None:
            return
        now = time.time() if now is None else now

        if self._clear_pending:
            value = CLEAR
        elif self._pwm is None or now - self._last_sent < REFRESH_S:
            return
        else:
            value = self._pwm

        try:
            set_ride_height(master, value, self.channel)
        except Exception:
            # The link dropping is not a reason to take down the loop that owes
            # the autopilot its heartbeat. The next tick retries.
            log.exception("ride height: override send failed")
            return
        self._clear_pending = False
        self._last_sent = now

def stream_data(master, hz=10):
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")


    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        hz, # Rate in Hertz
        1   # 1 = start sending, 0 = stop sending
    )

    last_heartbeat_time = time.time()

    while True:
        try:
            current_time = time.time()
            msg = master.recv_match(blocking=False)

            if msg:
                # Ignore raw BAD_DATA to keep the console clean
                if msg.get_type() != 'BAD_DATA':
                    # Print the message type. (Change to print(msg.to_dict()) to see the actual data)
                    print(f"RECEIVED: {msg.get_type()}")
                    print(msg.to_dict())

            if current_time - last_heartbeat_time >= 1.0:
                master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                last_heartbeat_time = current_time


        except Exception as e:
            print("Error", e)
