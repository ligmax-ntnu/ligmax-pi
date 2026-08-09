"""ZMQ ports between the nodes, and the boat's serial devices.

The serial half exists because /dev/ttyUSB0 and /dev/ttyACM0 are *enumeration
order*, not identity: which USB device gets which number depends on the order
the kernel probed the hubs, so a reboot - or one device powering up slower than
the other - can hand the lidar's name to the autopilot. Every port here is
therefore resolved through a udev symlink first (99-ligmax-serial.rules, which
keys off the USB vendor/product, so the name follows the hardware), then
/dev/serial/by-id, and only then the raw ttyUSB*/ttyACM* name as a last resort.

Install the rules once per Pi:

    sudo cp 99-ligmax-serial.rules /etc/udev/rules.d/
    sudo udevadm control --reload && sudo udevadm trigger --subsystem-match=tty
    ls -l /dev/ligmax-*
"""

import glob
import os

LOGGING_PORT = 5555
MAIN_PORT = 5556
IO_PORT = 5557
BALANCING_PORT = 5558
SELF_DRIVING_PORT = 5559
LIGHT_SYSTEM_PORT = 5560


# ---------------------------------------------------- the vessel's speed limit
#
# **5 knots, under autonomy. This is the one number both nodes have to agree on**
# and it therefore lives here, in the module they both already import, rather
# than in either of their own configs.
#
# It is enforced twice on purpose, at two different layers:
#
#   nodes/self_driving/config.py   clamps every configured speed to it, and
#                                  `commander.py` clamps every command to it on
#                                  the way out. That is the layer that makes the
#                                  boat *behave*.
#   nodes/io_manager/autopilot_bridge.py
#                                  clamps again immediately before the value
#                                  becomes a MAVLink message. That is the layer
#                                  that makes the limit *true*.
#
# The second one is not redundancy for its own sake. io_manager takes control
# messages off a loopback PUB/SUB socket and, until this existed, passed the
# `speed`, `vx` and `vy` fields into MAVLink verbatim - so the limit held only
# for as long as the autonomy node was the only thing on that bus and had no
# bugs. A limit that depends on the correctness of the process it is limiting is
# not a limit. The last hop before the wire enforces it independently.
#
# Not overridable from the environment, in either place. Raising it is a commit.
KNOT_MS = 0.514444
VESSEL_SPEED_LIMIT_KNOTS = 5.0
VESSEL_SPEED_LIMIT_MS = VESSEL_SPEED_LIMIT_KNOTS * KNOT_MS  # 2.5722 m/s


def resolve_serial(env_var, *candidates):
    """First candidate that exists, or the last one if none do.

    `candidates` may contain globs; they are tried in order, so put the udev
    symlink first and the bare kernel name last. An `env_var` that is set wins
    outright and is returned unchecked - it is also how a SITL endpoint
    (`udpin:127.0.0.1:14550`) gets in, which is not a path at all.

    Returning the last candidate rather than raising is deliberate: the caller
    is opening a device that may legitimately be unplugged right now, and its
    own retry loop reports that far better than an ImportError-time crash.
    """
    override = os.environ.get(env_var, "").strip()
    if override:
        return override
    for candidate in candidates:
        if any(char in candidate for char in "*?["):
            matches = sorted(glob.glob(candidate))
            if matches:
                return matches[0]
        elif os.path.exists(candidate):
            return candidate
    return candidates[-1]


# Holybro Pixhawk 6C. It presents two CDC-ACM interfaces - if00 is MAVLink,
# if02 is the SLCAN/debug one - so the symlink and the by-id path both have to
# name the first, or the link opens fine and never sees a heartbeat.
def pixhawk_port():
    return resolve_serial(
        "LIGMAX_MAVLINK_DEVICE",
        "/dev/ligmax-pixhawk",
        "/dev/serial/by-id/*Pixhawk*-if00",
        "/dev/ttyACM0",
    )


# RPLidar C1, on a Silicon Labs CP2102N bridge.
def lidar_port():
    return resolve_serial(
        "LIGMAX_LIDAR_PORT",
        "/dev/ligmax-lidar",
        "/dev/serial/by-id/*CP2102N*-if00-port0",
        "/dev/ttyUSB0",
    )
