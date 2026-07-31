import can
import json
import time

def get_battery_data(channel='can1', bitrate=250000, cell_count=12):
    """
    Fetches all available telemetry from a Daly Smart BMS on SocketCAN 
    and returns a structured JSON string.
    """
    # Structure for the output data
    battery_data = {
        "pack": {
            "voltage_v": None,
            "current_a": None,
            "soc_percent": None,
            "capacity_ah": None,
            "cycle_count": None
        },
        "mosfet_status": {
            "charge_enabled": None,
            "discharge_enabled": None
        },
        "cell_extremes": {
            "max_cell_mv": None,
            "max_cell_num": None,
            "min_cell_mv": None,
            "min_cell_num": None,
            "delta_mv": None
        },
        "temp_extremes": {
            "max_temp_c": None,
            "min_temp_c": None
        },
        "cell_voltages_mv": [],
        "temperatures_c": [],
        "balancing_cells": [],
        "alarm_flags_hex": None
    }
    
    # List of Daly commands to request
    commands = [0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98]
    raw_cells = {}
    
    try:
        bus = can.interface.Bus(channel=channel, interface='socketcan', bitrate=bitrate)
        
        # 1. Send request packets for all commands with brief delay
        for cmd in commands:
            request_id = 0x18000140 | (cmd << 16)
            msg = can.Message(
                arbitration_id=request_id,
                data=[0x00] * 8,
                is_extended_id=True
            )
            bus.send(msg)
            time.sleep(0.01)  # 10ms spacing between command requests
            
        # 2. Collect incoming frames for ~0.8 seconds
        start_time = time.time()
        while time.time() - start_time < 0.8:
            message = bus.recv(timeout=0.05)
            if message is None:
                continue
                
            cmd = (message.arbitration_id >> 16) & 0xFF
            d = message.data
            
            # --- 0x90: Main Pack Stats ---
            if cmd == 0x90:
                battery_data["pack"]["voltage_v"] = round(((d[0] << 8) | d[1]) / 10.0, 1)
                battery_data["pack"]["current_a"] = round((((d[4] << 8) | d[5]) - 30000) / 10.0, 1)
                battery_data["pack"]["soc_percent"] = round(((d[6] << 8) | d[7]) / 10.0, 1)
                
            # --- 0x91: Cell Volt Extremes ---
            elif cmd == 0x91:
                max_v = (d[0] << 8) | d[1]
                min_v = (d[3] << 8) | d[4]
                battery_data["cell_extremes"]["max_cell_mv"] = max_v
                battery_data["cell_extremes"]["max_cell_num"] = d[2]
                battery_data["cell_extremes"]["min_cell_mv"] = min_v
                battery_data["cell_extremes"]["min_cell_num"] = d[5]
                battery_data["cell_extremes"]["delta_mv"] = max_v - min_v
                
            # --- 0x92: Temperature Extremes ---
            elif cmd == 0x92:
                battery_data["temp_extremes"]["max_temp_c"] = d[0] - 40
                battery_data["temp_extremes"]["min_temp_c"] = d[2] - 40
                
            # --- 0x93: MOSFETs & Health ---
            elif cmd == 0x93:
                battery_data["mosfet_status"]["charge_enabled"] = bool(d[1] == 1)
                battery_data["mosfet_status"]["discharge_enabled"] = bool(d[2] == 1)
                battery_data["pack"]["cycle_count"] = d[3]
                cap_mah = (d[4] << 24) | (d[5] << 16) | (d[6] << 8) | d[7]
                battery_data["pack"]["capacity_ah"] = round(cap_mah / 1000.0, 2)
                
            # --- 0x95: Individual Cell Voltages ---
            elif cmd == 0x95:
                frame_no = d[0]
                base_idx = (frame_no - 1) * 3
                v1, v2, v3 = (d[1] << 8) | d[2], (d[3] << 8) | d[4], (d[5] << 8) | d[6]
                
                if 0 < v1 < 65000: raw_cells[base_idx + 1] = v1
                if 0 < v2 < 65000: raw_cells[base_idx + 2] = v2
                if 0 < v3 < 65000: raw_cells[base_idx + 3] = v3
                
            # --- 0x96: Individual Temperatures ---
            elif cmd == 0x96:
                temps = [b - 40 for b in d[1:8] if 0 < b < 255]
                if temps:
                    battery_data["temperatures_c"] = temps
                    
            # --- 0x97: Active Cell Balancing Bitmask ---
            elif cmd == 0x97:
                balancing = []
                for byte_idx in range(min(6, len(d))):
                    for bit_idx in range(8):
                        c_num = byte_idx * 8 + bit_idx + 1
                        if (d[byte_idx] >> bit_idx) & 1:
                            balancing.append(c_num)
                battery_data["balancing_cells"] = [c for c in balancing if c <= cell_count]
                
            # --- 0x98: Fault / Alarm Bytes ---
            elif cmd == 0x98:
                battery_data["alarm_flags_hex"] = [f"0x{b:02X}" for b in d]

        # Assemble the 12 cell voltages sequentially
        battery_data["cell_voltages_mv"] = [raw_cells[i] for i in range(1, cell_count + 1) if i in raw_cells]

    except Exception as e:
        return json.dumps({"error": str(e)}, indent=2)
    finally:
        if 'bus' in locals():
            bus.shutdown()
            
    return json.dumps(battery_data, indent=2)


if __name__ == "__main__":
    json_output = get_battery_data()
    print(json_output)