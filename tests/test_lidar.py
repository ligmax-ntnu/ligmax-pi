"""Capture RPLidar C1 scans to CSV.

    python3 tests/test_lidar.py            # until Ctrl+C
    python3 tests/test_lidar.py 20         # 20 rotations, then stop

Three things about the C1 that the `rplidar` library on PyPI gets wrong, all of
which look like a broken sensor:

  * **SET_PWM is not a C1 command.** `RPLidar.__init__` calls `start_motor()`,
    which sends SET_PWM (0xF0) for the A2's motor driver. The C1 spins its own
    motor and answers 0xF0 with nothing at all - and, worse, swallows the
    *next* command too, so the GET_INFO that follows times out and the library
    reports `Descriptor length mismatch`. `C1Lidar` below overrides the motor
    methods to touch DTR only.
  * **Scan mode survives the process.** The C1 keeps streaming after the port
    closes, so a run that was killed with SIGKILL - or any run that did not
    reach its `finally` - leaves the next one reading measurement bytes where
    it expects a response descriptor. That surfaces as `Wrong body size`, or as
    a scan that yields nothing while the raw port is clearly busy. Hence the
    STOP-and-flush before the first command, and again on the way out.
  * **The first measurement is ~2 s behind the SCAN command.** The C1 answers
    SCAN with its descriptor immediately and then sends nothing at all while
    the motor spins up. The library reads measurements with the port's own
    timeout, which defaults to one second, so the very first read times out and
    raises `Wrong body size` again - on a sensor that is about to work fine.
    READ_TIMEOUT below is the fix and is why it is not 1.
"""

import csv
import os
import sys
import time

from rplidar import RPLidar, RPLidarException

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import lidar_port  # noqa: E402 - needs the path above

# Never a bare /dev/ttyUSB0: that name belongs to whichever USB serial device
# the kernel probed first. See config.py and 99-ligmax-serial.rules.
PORT_NAME = lidar_port()

# Fixed for the C1. The A-series' 115200 gets you a silent port, not an error.
BAUDRATE = 460800

STOP_SETTLE = 0.1  # the C1 finishes the rotation in flight before going idle
BOOT_SETTLE = 2.0  # after RESET, before it answers again
# Every read the library makes uses this, so it has to cover the worst one: the
# motor spin-up between SCAN and the first measurement, measured at ~2s. Once
# the stream is running each read returns in microseconds, so a generous value
# costs nothing and only ever applies to a link that has actually gone quiet.
READ_TIMEOUT = 5.0


class C1Lidar(RPLidar):
    """`RPLidar` with the A2 motor commands taken out. See the module docstring."""

    def start_motor(self):
        # DTR low is the A1's motor enable and harmless here; the C1 ignores it.
        # No SET_PWM - that is the command that wedges the C1.
        self._serial_port.dtr = False
        self.motor_running = True

    def stop_motor(self):
        self._serial_port.dtr = True
        self.motor_running = False

    def quiesce(self):
        """Leave the sensor idle with an empty buffer, from any prior state.

        Safe to call on a device that is mid-scan, that has just booted, or
        that a previous crashed run left streaming.
        """
        self.stop()  # sends STOP, then clear_input()
        time.sleep(STOP_SETTLE)
        self._serial_port.reset_input_buffer()


def open_lidar():
    """Connected, idle, verified sensor - or an exception naming the port.

    One retry through RESET: a C1 that was left in a state STOP alone does not
    clear answers the first GET_INFO with garbage and the second one properly.
    """
    lidar = C1Lidar(PORT_NAME, baudrate=BAUDRATE, timeout=READ_TIMEOUT)
    for attempt in (1, 2):
        try:
            lidar.quiesce()
            info = lidar.get_info()
            health = lidar.get_health()
        except RPLidarException as exc:
            if attempt == 2:
                lidar.disconnect()
                raise
            print(f"{exc} - resetting the sensor and retrying")
            lidar.reset()
            time.sleep(BOOT_SETTLE)
            continue
        return lidar, info, health
    raise AssertionError("unreachable")


def capture_data(max_rotations=None):
    print(f"Connecting to RPLidar on {PORT_NAME} at {BAUDRATE} baud...")
    lidar, info, health = open_lidar()

    print(f"Device Info: {info}")
    print(f"Health Status: {health}")
    if health[0] != "Good":
        print(f"Warning: LiDAR health is {health[0]!r} (error code {health[1]}).")

    filename = os.path.abspath(f"lidar_c1_data_{int(time.time())}.csv")
    print(f"\nSaving to {filename}")
    print("Press Ctrl+C to stop.\n")

    scan_index = 0
    try:
        with open(filename, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(
                ["Timestamp", "Scan_Index", "Quality", "Angle_Deg", "Distance_mm"]
            )

            # One list of measurements per 360-degree rotation.
            for scan in lidar.iter_scans():
                current_time = time.time()
                for quality, angle, distance in scan:
                    writer.writerow(
                        [current_time, scan_index, quality, angle, distance]
                    )

                scan_index += 1
                if scan_index % 10 == 0:
                    # Flush, so a Ctrl+C or a pulled plug still leaves a
                    # readable file rather than a truncated last buffer.
                    csvfile.flush()
                    print(f"Captured {scan_index} full rotations...")
                if max_rotations and scan_index >= max_rotations:
                    break
    except KeyboardInterrupt:
        print("\nCapture interrupted by user.")
    finally:
        # Order matters: STOP and drain *before* closing, or the C1 keeps
        # streaming into a closed port and the next run inherits the mess.
        print("Stopping motor and disconnecting...")
        try:
            lidar.quiesce()
            lidar.stop_motor()
        except Exception as exc:  # noqa: BLE001 - teardown must not mask the real error
            print(f"(cleanup: {exc})")
        lidar.disconnect()
        print(f"Disconnected. {scan_index} rotations in {filename}")


if __name__ == "__main__":
    rotations = int(sys.argv[1]) if len(sys.argv) > 1 else None
    capture_data(rotations)
