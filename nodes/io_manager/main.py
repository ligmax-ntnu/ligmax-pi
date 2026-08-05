"""io_manager: owns the links to the autopilot and to the ground station.

Started by the supervisor as `python -m nodes.io_manager.main` from the repo
root, and restarted by it if this process exits (../../main.py).

What it does:

  * keeps the MAVLink link to the Pixhawk alive, including the 1 Hz GCS
    heartbeat a companion computer is expected to send - without it the
    autopilot may failsafe (test.py:48).
  * turns the autopilot's BATTERY_STATUS / SYS_STATUS into the dashboard's
    `telemetry.battery` block and uploads it once a second.
  * forwards log lines to the dashboard's log panel: this node's own, the
    ZeroMQ node bus on LOGGING_PORT, and the autopilot's STATUSTEXT.
  * executes the operator's commands, which arrive in the reply to each
    telemetry POST: `estop`, `estop_clear` and `home_battery` drive the two
    GPIO lines in `emergency_stop.py`. Every command is acked, so the
    dashboard's command list shows what actually happened on the vessel.

What it does not do yet: ride-height control (`pixhalwk.set_ride_height`), and
nothing here disarms the autopilot - the E-stop cuts propulsion *power* at the
contactor rather than asking the Pixhawk nicely.

Battery data here is the autopilot's view of the pack; `battery.py` reads the
Daly BMS directly over CAN for far more detail, but it blocks for ~0.8 s per
call, so wiring it in needs a worker thread rather than this loop.
"""

import json
import logging
import os
import time

from pymavlink import mavutil

from config import LOGGING_PORT

from .emergency_stop import BatteryHoming, EstopRelay
from .upload import Uploader

# /dev/ttyACM0 is not stable across a replug - it can come back as ttyACM1, so
# a udev symlink is the real fix. The override also takes a SITL endpoint, e.g.
# LIGMAX_MAVLINK_DEVICE=udpin:127.0.0.1:14550
MAVLINK_DEVICE = os.environ.get("LIGMAX_MAVLINK_DEVICE", "/dev/ttyACM0")
MAVLINK_BAUD = int(os.environ.get("LIGMAX_MAVLINK_BAUD", "115200"))

STREAM_RATE_HZ = 4  # only battery and status are consumed today
HEARTBEAT_PERIOD = 1.0
PUBLISH_PERIOD = 1.0
LOOP_SLEEP = 0.01
MAX_MESSAGES_PER_TICK = 200  # never let a backlog starve the heartbeat
LINK_FAIL_DELAY = 5.0  # the supervisor restarts us with no backoff of its own

UINT16_MAX = 0xFFFF
INT16_MAX = 0x7FFF

# MAV_SEVERITY -> the dashboard's five levels (ligmax_gui/protocol.py:481).
STATUSTEXT_LEVELS = {
    0: "CRITICAL",  # EMERGENCY
    1: "CRITICAL",  # ALERT
    2: "CRITICAL",  # CRITICAL
    3: "ERROR",
    4: "WARN",
    5: "INFO",  # NOTICE
    6: "INFO",
    7: "DEBUG",
}

# MAV_BATTERY_CHARGE_STATE. 0 is UNDEFINED, which is not worth displaying.
CHARGE_STATES = {
    1: "OK",
    2: "LOW",
    3: "CRITICAL",
    4: "EMERGENCY",
    5: "FAILED",
    6: "UNHEALTHY",
    7: "CHARGING",
}

log = logging.getLogger("io_manager")


def cell_voltages(battery):
    """Per-cell volts from a BATTERY_STATUS, honouring its two "unknown" markers.

    `voltages` covers cells 1-10 and marks absent cells with UINT16_MAX;
    `voltages_ext` covers cells 11-14 and marks them with 0 instead. Our pack is
    12S, so a smart monitor puts cells 11 and 12 in the second array.

    A monitor that cannot see individual cells puts the *pack* total in
    voltages[0], which is why the caller only treats this as cell data when it
    returns more than one entry. Either way the sum is the pack voltage.
    """
    cells = [
        mv / 1000.0
        for mv in (getattr(battery, "voltages", None) or ())
        if 0 < mv < UINT16_MAX
    ]
    cells += [
        mv / 1000.0
        for mv in (getattr(battery, "voltages_ext", None) or ())
        if 0 < mv < UINT16_MAX
    ]
    return cells


def _field(source, name, unknown):
    """A MAVLink field, or None if it carries its "not measured" sentinel."""
    if source is None:
        return None
    value = getattr(source, name, None)
    if value is None or value == unknown:
        return None
    return value


def battery_telemetry(sys_status=None, battery=None):
    """BATTERY_STATUS + SYS_STATUS -> the dashboard's `telemetry.battery` block.

    Names and units are the ones ligmax-server already has widgets for
    (web/js/telemetry.js:19-29): volts, amps, watts, degrees C - and `soc` as a
    *fraction*, not a percentage. Fields the autopilot does not measure are left
    out entirely rather than sent as a sentinel, so the dashboard shows a gap
    instead of a plausible lie.
    """
    telemetry = {}
    cells = []

    if battery is not None:
        cells = cell_voltages(battery)
        if len(cells) > 1:  # one entry is the pack total, not a cell - see above
            telemetry["cell_min"] = round(min(cells), 3)
            telemetry["cell_max"] = round(max(cells), 3)
            telemetry["cells"] = len(cells)

        # centidegrees C, INT16_MAX when there is no temperature sensor.
        if (temperature := _field(battery, "temperature", INT16_MAX)) is not None:
            telemetry["temperature"] = round(temperature / 100.0, 1)

        if (consumed := _field(battery, "current_consumed", -1)) is not None:
            telemetry["consumed_mah"] = int(consumed)

        # energy_consumed is in hJ (100 J), so Wh = hJ * 100 / 3600 = hJ / 36.
        if (energy := _field(battery, "energy_consumed", -1)) is not None:
            telemetry["consumed_wh"] = round(energy / 36.0, 1)

        # 0 means "not provided" for time_remaining, not "empty now".
        if (remaining := _field(battery, "time_remaining", 0)) is not None:
            telemetry["time_remaining_s"] = int(remaining)

        # The autopilot's own fault view of the pack, not the Daly BMS's -
        # battery.py is the source that talks to the BMS.
        faults = getattr(battery, "fault_bitmask", None)
        if faults is not None:
            telemetry["bms_ok"] = faults == 0
            if faults:
                telemetry["faults_hex"] = f"0x{faults:04X}"

        if state := CHARGE_STATES.get(getattr(battery, "charge_state", 0)):
            telemetry["charge_state"] = state

    voltage_mv = _field(sys_status, "voltage_battery", UINT16_MAX)
    if voltage_mv:
        voltage = voltage_mv / 1000.0
    else:
        voltage = sum(cells) if cells else None
    if voltage is not None:
        telemetry["voltage"] = round(voltage, 2)

    # centiamps. Exactly -1 means "not sent"; other negatives mean charging.
    current = _field(battery, "current_battery", -1)
    if current is None:
        current = _field(sys_status, "current_battery", -1)
    if current is not None:
        current /= 100.0
        telemetry["current"] = round(current, 2)

    if voltage is not None and current is not None:
        telemetry["power"] = round(voltage * current, 1)

    percent = _field(battery, "battery_remaining", -1)
    if percent is None:
        percent = _field(sys_status, "battery_remaining", -1)
    if percent is not None:
        telemetry["soc"] = round(percent / 100.0, 4)  # the dashboard wants 0-1

    return telemetry


class LogBus:
    """Subscriber on the ZeroMQ node log bus, so its output stops going nowhere.

    The supervisor binds the PUB side on LOGGING_PORT (../../main.py:69) and
    publishes every node start, stop and error there, but nothing has ever
    subscribed (docs/findings.md item 4). Degrades to a no-op if pyzmq is
    missing or the socket will not open - a broken log path must not stop this
    node from flying the boat.
    """

    def __init__(self, port=LOGGING_PORT):
        self._zmq = None
        self._socket = None
        try:
            import zmq
        except ImportError as exc:
            log.warning("log bus unavailable, pyzmq not installed (%s)", exc)
            return
        try:
            socket = zmq.Context.instance().socket(zmq.SUB)
            socket.connect(f"tcp://127.0.0.1:{port}")
            socket.setsockopt_string(zmq.SUBSCRIBE, "")
        except Exception as exc:  # noqa: BLE001 - never fatal
            log.warning("could not subscribe to the log bus on %s: %s", port, exc)
            return
        self._zmq = zmq
        self._socket = socket
        log.info("subscribed to the node log bus on 127.0.0.1:%s", port)

    def drain(self, limit=200):
        """Every log line waiting on the bus, as protocol log dicts."""
        out = []
        if self._socket is None:
            return out
        while len(out) < limit:
            try:
                raw = self._socket.recv_string(flags=self._zmq.NOBLOCK)
            except self._zmq.Again:
                break
            except Exception as exc:  # noqa: BLE001 - drop the line, keep the node
                log.warning("log bus read failed: %s", exc)
                break
            try:
                entry = json.loads(raw)
            except ValueError:
                entry = {"message": raw}
            if isinstance(entry, dict):
                out.append(entry)
        return out

    def close(self):
        if self._socket is not None:
            self._socket.close(linger=0)
            self._socket = None


def forward_log_bus(bus, uploader):
    for entry in bus.drain():
        message = entry.get("message") or entry.get("msg") or ""
        # The supervisor puts the traceback in its own key (../../main.py:57).
        if exception := entry.get("Exception"):
            message = f"{message} ({exception})"
        uploader.log(
            entry.get("level", "INFO"), message, name=entry.get("name", "node")
        )


def handle_commands(uploader, relay, homing):
    """Run the operator's queued commands and ack each one.

    Commands ride back in the reply to a telemetry POST, so this is only as
    prompt as PUBLISH_PERIOD - about a second. The physical E-stop button is in
    series with the relay and needs none of this to work; that is the link that
    has to be trusted, and this one is the convenience.

    Anything not implemented here is acked `failed` on purpose, so the dashboard
    says "failed: not implemented" instead of leaving the operator watching a
    command sit at "sent" until it expires.
    """
    for command in uploader.commands():
        name = str(command.get("name", ""))
        command_id = command.get("id")
        log.info("command %s: %s %s", command_id, name, command.get("args") or "")

        if name == "estop":
            ok, result = relay.engage(reason=f"dashboard command {command_id}")
        elif name == "estop_clear":
            ok, result = relay.clear(reason=f"dashboard command {command_id}")
        elif name == "home_battery":
            if relay.engaged:
                # The slider draws from the same pack the relay just isolated.
                ok, result = False, "refused: clear the emergency stop first"
                log.warning("home_battery %s", result)
            else:
                ok, result = homing.trigger(reason=f"dashboard command {command_id}")
        else:
            ok, result = False, f"'{name}' is not implemented on the vessel"
            log.warning("command %s ignored: %s", command_id, result)

        if command_id is not None:
            uploader.ack(command_id, "acked" if ok else "failed", result)


def safety_telemetry(relay, homing):
    """What the operator needs in order to trust - or distrust - the buttons."""
    return {
        "estop_engaged": relay.engaged,
        "estop_relay_line": relay.available,
        "homing_line": homing.available,
    }


def connect():
    """Open the MAVLink link and wait for the autopilot to introduce itself."""
    log.info("opening %s at %s baud", MAVLINK_DEVICE, MAVLINK_BAUD)
    master = mavutil.mavlink_connection(MAVLINK_DEVICE, baud=MAVLINK_BAUD)
    master.wait_heartbeat()
    log.info(
        "autopilot up: system %s component %s",
        master.target_system,
        master.target_component,
    )
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        STREAM_RATE_HZ,
        1,  # 1 = start sending, 0 = stop
    )
    return master


def main():
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s %(message)s"
    )

    uploader = Uploader.from_env()
    uploader.attach_logging()  # everything logged from here on also goes up
    log.info(
        "telemetry uplink: %s (%s)",
        uploader.url,
        "authenticated" if uploader.key else "no LIGMAX_BOAT_KEY set",
    )

    bus = LogBus()

    # Claim the GPIO before the autopilot link, so the safety line is up even if
    # the Pixhawk is unplugged. EstopRelay() closes the relay as it opens the
    # pin: propulsion is permitted from here until someone stops it.
    relay = EstopRelay()
    homing = BatteryHoming()
    for line, what in ((relay, "E-stop relay"), (homing, "homing line")):
        if not line.available:
            log.error(
                "%s on BCM %s is NOT driving a pin - the dashboard button will "
                "report failed, not silently do nothing",
                what,
                line.pin,
            )

    try:
        master = connect()
    except Exception as exc:  # noqa: BLE001 - report it upward, then let us restart
        log.error("MAVLink link to %s failed: %s", MAVLINK_DEVICE, exc)
        time.sleep(LINK_FAIL_DELAY)
        homing.close()
        relay.close()
        bus.close()
        uploader.close()
        raise SystemExit(1)

    sys_status = None
    battery = None
    last_heartbeat = 0.0
    last_publish = 0.0

    try:
        while True:
            now = time.time()

            for _ in range(MAX_MESSAGES_PER_TICK):
                message = master.recv_match(blocking=False)
                if message is None:
                    break
                kind = message.get_type()
                if kind == "SYS_STATUS":
                    sys_status = message
                elif kind == "BATTERY_STATUS":
                    # Instance 0 only. A second monitor would otherwise
                    # overwrite the same telemetry keys as the main pack.
                    if getattr(message, "id", 0) == 0:
                        battery = message
                elif kind == "STATUSTEXT":
                    uploader.log(
                        STATUSTEXT_LEVELS.get(message.severity, "INFO"),
                        str(message.text).strip(),
                        name="pixhawk",
                    )

            if now - last_heartbeat >= HEARTBEAT_PERIOD:
                master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0,
                )
                last_heartbeat = now

            forward_log_bus(bus, uploader)
            handle_commands(uploader, relay, homing)

            if now - last_publish >= PUBLISH_PERIOD:
                # Published even when the battery block is empty: a frame
                # arriving at all is how the dashboard knows we are on the air,
                # and `estop` is what lights its banner.
                telemetry = {"safety": safety_telemetry(relay, homing)}
                if battery_block := battery_telemetry(sys_status, battery):
                    telemetry["battery"] = battery_block
                uploader.publish(telemetry=telemetry, estop=relay.engaged)
                last_publish = now

            time.sleep(LOOP_SLEEP)

    except KeyboardInterrupt:
        log.info("io_manager stopping")
    except Exception as exc:  # noqa: BLE001 - get it into the log panel first
        log.exception("io_manager died: %s", exc)
        raise
    finally:
        # Order matters: drop the GPIO first (the relay opens, cutting propulsion
        # power), then tell the dashboard why, then flush the uplink.
        homing.close()
        relay.close()
        bus.close()
        uploader.close()  # flushes what is still queued


if __name__ == "__main__":
    main()
