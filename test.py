import time
from pymavlink import mavutil

def main():
    # 1. Start the one "master" connection
    # Use 'udpin:localhost:14550' for SITL/Companion computer or '/dev/ttyUSB0' for serial
    print("Connecting to autopilot...")
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    # 2. Wait for the first heartbeat
    # This sets up the target_system and target_component so we know who to talk to
    print("Waiting for heartbeat...")
    master.wait_heartbeat()
    print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")

    # 3. Request ALL data streams at 10 Hz
    print("Requesting all data streams...")
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 
        10, # Rate in Hertz
        1   # 1 = start sending, 0 = stop sending
    )

    # Track time for our injection loops
    last_heartbeat_time = time.time()
    last_injection_time = time.time()

    print("Entering main loop (Press Ctrl+C to stop)...")
    try:
        while True:
            msg = master.recv_match(blocking=False)
            
            if msg:
                # Ignore raw BAD_DATA to keep the console clean
                if msg.get_type() != 'BAD_DATA':
                    # Print the message type. (Change to print(msg.to_dict()) to see the actual data)
                    print(f"RECEIVED: {msg.get_type()}")
                    print(msg.to_dict())
            

            # ==========================================
            # PART B: INJECT DATA / COMMANDS
            # ==========================================
            current_time = time.time()

            # Task 1: Inject a Heartbeat every 1 second
            # Ground stations and companion computers MUST send heartbeats or the drone may failsafe
            if current_time - last_heartbeat_time >= 1.0:
                master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 
                    0, 0, 0
                )
                last_heartbeat_time = current_time

            # Task 2: Inject your custom commands every 5 seconds
            # (e.g., overriding RC channels, sending GPS data, moving a gimbal)
            if current_time - last_injection_time >= 5.0:
                print("--> INJECTING: Custom Command")
                
                # Example: Sending an RC Override command (setting channel 1 to 1500us)
                # master.mav.rc_channels_override_send(
                #     master.target_system,
                #     master.target_component,
                #     1500, 0, 0, 0, 0, 0, 0, 0
                # )
                
                last_injection_time = current_time

            # Sleep briefly to prevent the loop from maxing out your CPU at 100%
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nScript stopped by user.")

if __name__ == "__main__":
    main()