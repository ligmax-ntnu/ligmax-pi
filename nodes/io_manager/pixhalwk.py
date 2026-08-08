"""Ride-height command and a raw MAVLink dump. Not wired into `main.py` yet.

`amas.lua` reads its ride-height demand off **RC input channel 16** and adds the
remote's channel 15 to it (`HEIGHT_RC_CHAN`, `HEIGHT_RC_CHAN_CONTROLLER`), so an
override from here has to land on 16 and nowhere else. Channels 1..8 are the
sticks and the aux switches on this boat: an override there fights the pilot for
the same channel, which is why nothing in the fleet uses one.

There is a third way into the same sum, and it is the one the dashboard uses:
`SCR_USER6`, written as an ordinary parameter by `tuning.py`. The difference is
persistence, not effect. An override stops the moment this process does - the
autopilot times it out and the amas hold - whereas a parameter is stored on the
flight controller and the amas resume creeping after a reboot. Use the override
for "move it now while I watch", the parameter for a trim meant to stay set.
"""

import time

from pymavlink import mavutil

# amas.lua's HEIGHT_RC_CHAN. RC channel numbers are 1-based and the override
# message is an array, hence the index below.
HEIGHT_CHANNEL = 16
CHANNEL_COUNT = 18
RELEASE = 65535  # "leave this channel to whoever else is driving it"


def set_ride_height(master, pwm_value):
    """
    Sends a MAVLink RC Override command to Virtual Channel 16.
    1000 = Pull Amas Up (Lower boat)
    1500 = Stop / Hold position
    2000 = Push Amas Down (Raise boat)

    The translator ESP32 reads this pulse as a *velocity*, so 1500 really does
    mean hold and anything else keeps moving for as long as it is sent.
    """
    # Every channel released except ours, so this cannot take the sticks away.
    rc_channels = [RELEASE] * CHANNEL_COUNT
    rc_channels[HEIGHT_CHANNEL - 1] = pwm_value

    # Send the override command
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_channels
    )
    print(f"Sent Ride Height Command: {pwm_value}us to Channel {HEIGHT_CHANNEL}")

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
