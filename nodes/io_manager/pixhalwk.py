import time

def set_ride_height(master, pwm_value):
    """
    Sends a MAVLink RC Override command to Virtual Channel 6.
    1000 = Pull Amas Up (Lower boat)
    1500 = Stop / Hold position
    2000 = Push Amas Down (Raise boat)
    """
    # Create an array of 18 channels filled with 65535 (ignore/release)
    rc_channels = [65535] * 18
    
    # Set Channel 6 (index 5) to our desired PWM value
    rc_channels[5] = pwm_value 
    
    # Send the override command
    master.mav.rc_channels_override_send(
        master.target_system,
        master.target_component,
        *rc_channels
    )
    print(f"Sent Ride Height Command: {pwm_value}us to Channel 6")

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