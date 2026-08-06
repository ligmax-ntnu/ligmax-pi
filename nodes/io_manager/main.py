"""io_manager: owns the links to the autopilot and to the ground station.

Started by the supervisor as `python -m nodes.io_manager.main` from the repo
root, and restarted by it if this process exits (../../main.py).

What it does:

  * keeps the MAVLink link to the Pixhawk alive, including the 1 Hz GCS
    heartbeat a companion computer is expected to send - without it the
    autopilot may failsafe (test.py:48).
  * decides, once, **who is in charge of the boat** - autonomous, remote,
    standby, out of control, or killed (`status.py`) - and drives both consumers
    of that answer from the one decision: the operator's status indicator, over
    the telemetry link, and the colour of the lights on the hull (`lights.py`).
  * publishes the navigation figures the operator's GUI needs: position, heading,
    course and speed over ground, and the distance to the next waypoint
    (`navigation.py`).
  * publishes the battery, **read off the Daly BMS over CAN** rather than taken
    from the autopilot (`bms.py`). The autopilot's own estimate is kept as a
    labelled fallback and `telemetry.battery.source` says which is answering.
  * publishes what the trim actuators were told to do - battery-slider position
    and the two ama outputs (`trim.py`), commanded values, since neither
    actuator reports anything back.
  * forwards log lines to the dashboard's log panel: this node's own, the
    ZeroMQ node bus on LOGGING_PORT, and the autopilot's STATUSTEXT.
  * executes the operator's commands, which arrive in the reply to each
    telemetry POST: `estop`, `estop_clear` and `home_battery` drive the two
    GPIO lines in `emergency_stop.py`. Every command is acked, so the
    dashboard's command list shows what actually happened on the vessel.

What it does not do yet: ride-height control (`pixhalwk.set_ride_height`), and
nothing here disarms the autopilot - the E-stop cuts propulsion *power* at the
contactor rather than asking the Pixhawk nicely.

Everything added to the loop here is non-blocking by construction. The BMS read
takes ~0.85 s and the lights write can stall on a dead cable, so both live on
their own worker threads and this loop only ever reads their latest answer. The
heartbeat is the thing being protected.
"""

import json
import logging
import os
import time

from pymavlink import mavutil

from config import LOGGING_PORT

from .bms import BmsReader
from .emergency_stop import BatteryHoming, EstopRelay
from .lights import Lights
from .navigation import Navigation
from .selfupdate import NAME as REPO_NAME, SelfUpdate, request_restart
from .status import StatusMachine
from .trim import Trim
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
LINK_FAIL_DELAY = 5.0  # how long to wait before retrying a dead/missing link
HEARTBEAT_WAIT_S = float(os.environ.get("LIGMAX_MAVLINK_HEARTBEAT_TIMEOUT_S", "3.0"))

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

    **This is the fallback, not the source of record.** The operator's battery
    figures are supposed to come off the Daly BMS over CAN (`bms.py`), which has
    the shunt and the cell taps; this is the autopilot's estimate, used when the
    BMS is not answering. `source` marks which one the dashboard is showing, and
    it is the field to look at first when the SOC looks wrong.

    Names and units are the ones ligmax-server already has widgets for
    (web/js/telemetry.js): volts, amps, watts, degrees C - and `soc` as a
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

    if telemetry:
        # Only labelled when there is something to label. The dashboard turns this
        # into "autopilot estimate" beside the percentage.
        telemetry["source"] = "pixhawk"
    return telemetry


def merge_battery(bms_block, autopilot_block):
    """The battery block to publish: the BMS if it answered, the autopilot if not.

    Not a field-by-field merge. Mixing a BMS state of charge with an autopilot
    voltage would produce a block that is internally inconsistent - a power figure
    computed from two different sensors' idea of the same pack - and no one reading
    the panel could tell. One source answers, and `source` names it.

    The exception is the autopilot's per-cell view, which it only has when a smart
    monitor is fitted, and its consumed-energy counter. Neither is something the
    Daly reports in the same terms, and both are additive rather than conflicting,
    so they ride along when the BMS is in charge.
    """
    if not bms_block.get("bms_ok") and "voltage" not in bms_block:
        # The BMS is not answering. Fall back, but keep its error so the operator
        # can see *why* they are looking at an estimate.
        out = dict(autopilot_block)
        if out:
            for key in ("last_error", "read_errors"):
                if key in bms_block:
                    out[key] = bms_block[key]
            out["bms_ok"] = False
        return out or bms_block

    out = dict(bms_block)
    for key in ("consumed_mah", "consumed_wh", "time_remaining_s"):
        if key in autopilot_block:
            out[key] = autopilot_block[key]
    return out


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


def handle_commands(uploader, relay, homing, updater, machine=None, trim=None):
    """Run the operator's queued commands and ack each one.

    Commands ride back in the reply to a telemetry POST, so this is only as
    prompt as PUBLISH_PERIOD - about a second. The physical E-stop button is in
    series with the relay and needs none of this to work; that is the link that
    has to be trusted, and this one is the convenience.

    Anything not implemented here is acked `failed` on purpose, so the dashboard
    says "failed: not implemented" instead of leaving the operator watching a
    command sit at "sent" until it expires.

    `update` is the exception to acking here: it starts a `git pull` on a worker
    thread and is acked later by `finish_update()`, because a pull can outlast
    the autopilot's heartbeat timeout and must not run in this loop.
    """
    for command in uploader.commands():
        name = str(command.get("name", ""))
        command_id = command.get("id")
        args = command.get("args") or {}
        log.info("command %s: %s %s", command_id, name, args)

        # An operator command arriving is proof the shore link is alive, whatever
        # the command turns out to be - including one we go on to refuse. That
        # makes it a control source, which is what keeps a boat under manual
        # supervision out of OUT_OF_CONTROL when the RC is off.
        if machine is not None:
            machine.note_operator()

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
                if ok and trim is not None:
                    # While it hunts for the optical centre the slider ignores the
                    # Pixhawk's position demand entirely, so the commanded rail
                    # figure is not even what it is chasing. Say so in telemetry.
                    trim.note_homing(True)
        elif name == "update":
            # Every node reads the same command queue, so check this one is ours
            # before pulling somebody else's repo into our checkout.
            repo = str(args.get("repo") or "")
            if repo != REPO_NAME:
                ok, result = False, f"'{repo}' is not this node's repo ({REPO_NAME})"
                log.warning("update %s", result)
            elif command_id is None:
                ok, result = False, "update needs a command id to ack against"
            else:
                started, why = updater.start(str(command_id))
                if started:
                    log.warning("update: %s", why)
                    continue  # acked by finish_update() when the pull lands
                ok, result = False, f"refused: {why}"
        else:
            ok, result = False, f"'{name}' is not implemented on the vessel"
            log.warning("command %s ignored: %s", command_id, result)

        if command_id is not None:
            uploader.ack(command_id, "acked" if ok else "failed", result)


def finish_update(uploader, updater):
    """Ack a finished pull. Returns True when the node tree should restart.

    The ack goes out before anything is torn down - `uploader.close()` in the
    caller's `finally` flushes it - because once the process group is signalled
    there is nothing left to report with, and the operator's row would sit at
    "Waiting" through a restart that actually worked.
    """
    outcome = updater.take()
    if outcome is None:
        return False

    uploader.ack(
        outcome.command_id,
        "acked" if outcome.ok else "failed",
        outcome.message,
        head=outcome.head,
    )
    if not outcome.ok:
        # A refused fast-forward is not a reason to bounce propulsion: the boat
        # keeps running the code it has, and the operator reads git's own message.
        return False
    if not outcome.changed:
        log.info("update: already up to date, nothing to restart for")
        return False
    return True


def safety_telemetry(relay, homing, mavlink_up, lights=None):
    """What the operator needs in order to trust - or distrust - the buttons."""
    block = {
        "estop_engaged": relay.engaged,
        "estop_relay_line": relay.available,
        "homing_line": homing.available,
        "mavlink_link": mavlink_up,
    }
    if lights is not None:
        # Whether the hull can be *told* anything, as opposed to what it is
        # showing - that is `telemetry.lights`. A boat whose lights are stuck on
        # the wrong colour is a scrutineering problem, so it belongs here too.
        block["lights_line"] = lights.available
    return block


def connect():
    """Open the MAVLink link and wait for the autopilot to introduce itself.

    Raises on failure - either the port would not open, or nothing answered
    the heartbeat within HEARTBEAT_WAIT_S. Callers decide what "not connected"
    should mean; this function never blocks forever.
    """
    log.info("opening %s at %s baud", MAVLINK_DEVICE, MAVLINK_BAUD)
    master = mavutil.mavlink_connection(MAVLINK_DEVICE, baud=MAVLINK_BAUD)
    if master.wait_heartbeat(timeout=HEARTBEAT_WAIT_S) is None:
        master.close()
        raise TimeoutError(f"no heartbeat within {HEARTBEAT_WAIT_S:.0f}s")
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
    # python-can logs "Created a socket" at INFO every time a Bus is opened, and
    # `battery.py` opens one per BMS poll - about every 3 s. `attach_logging()`
    # below forwards everything to the operator's log, so at INFO this third-party
    # chatter is the only thing anyone can see there. Warnings still come through.
    logging.getLogger("can").setLevel(logging.WARNING)

    uploader = Uploader.from_env()
    uploader.attach_logging()  # everything logged from here on also goes up
    log.info(
        "telemetry uplink: %s (%s)",
        uploader.url,
        "authenticated" if uploader.key else "no LIGMAX_BOAT_KEY set",
    )

    bus = LogBus()
    updater = SelfUpdate()
    restart_for_update = False

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

    # Who is in charge, and the two things that answer to it. The status machine is
    # pure bookkeeping; the lights own a worker thread and a serial port.
    machine = StatusMachine()
    lights = Lights()
    # Say something immediately rather than waiting for the first publish: until
    # the autopilot is heard from, the honest colour is not green.
    lights.set_status(machine.evaluate(relay.engaged))

    # Navigation and trim are fed from the MAVLink pump below; the BMS runs itself.
    navigation = Navigation()
    trim = Trim()
    if not trim.configured:
        log.warning(
            "no servo channels configured for the trim readback, so the battery "
            "slider and ama figures will be absent from the dashboard. Read the "
            "SERVOn_FUNCTION mapping off the flight controller and set "
            "LIGMAX_AMA_PORT_CH / LIGMAX_AMA_STARBOARD_CH / LIGMAX_SLIDER_CH"
        )
    bms = BmsReader()

    # No Pixhawk yet is not fatal - it is common on the bench, and even on the
    # water a disconnected autopilot is exactly when the E-stop and dashboard
    # link matter most. `master` is None until connect() succeeds, and drops
    # back to None on any link error; the loop below just keeps retrying.
    master = None
    next_connect_attempt = 0.0

    sys_status = None
    battery = None
    last_heartbeat = 0.0
    last_publish = 0.0

    try:
        while True:
            now = time.time()

            if master is None and now >= next_connect_attempt:
                try:
                    master = connect()
                except Exception as exc:  # noqa: BLE001 - Pixhawk may not be plugged in
                    log.error("MAVLink link to %s failed: %s", MAVLINK_DEVICE, exc)
                    next_connect_attempt = now + LINK_FAIL_DELAY

            if master is not None:
                try:
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
                        elif kind == "HEARTBEAT":
                            # The autopilot's own heartbeat, not the one we send.
                            # `flightmode` is mavutil decoding custom_mode for us,
                            # which is the only reliable way to get the mode *name*
                            # - the number means different things per vehicle type.
                            if getattr(message, "autopilot", 0) != (
                                mavutil.mavlink.MAV_AUTOPILOT_INVALID
                            ):
                                machine.note_heartbeat(
                                    mode=getattr(master, "flightmode", None),
                                    armed=bool(master.motors_armed()),
                                )
                        elif kind == "RC_CHANNELS":
                            # FRSky is still delivering. This shares no hardware
                            # with the 5G link, so it counts as an independent
                            # control source (docs/architecture.md, safety layer 3).
                            machine.note_rc()
                        elif kind == "STATUSTEXT":
                            uploader.log(
                                STATUSTEXT_LEVELS.get(message.severity, "INFO"),
                                str(message.text).strip(),
                                name="pixhawk",
                            )
                        elif not navigation.handle(message):
                            # Position, course and mission progress, then the servo
                            # rail. Both return False for anything they do not
                            # want, so a message nobody reads costs two lookups.
                            trim.handle(message)

                    if now - last_heartbeat >= HEARTBEAT_PERIOD:
                        master.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                            0,
                            0,
                            0,
                        )
                        last_heartbeat = now
                except Exception as exc:  # noqa: BLE001 - a dropped link must not kill the node
                    log.error("MAVLink link to %s dropped: %s", MAVLINK_DEVICE, exc)
                    master.close()
                    master = None
                    sys_status = None
                    battery = None
                    # Drop everything the autopilot was the only source of. A
                    # chart still showing the vessel where it was thirty seconds
                    # ago is worse than one showing nothing, because it looks right.
                    navigation.link_down()
                    trim.link_down()
                    machine.note_link_down()
                    next_connect_attempt = now + LINK_FAIL_DELAY

            forward_log_bus(bus, uploader)
            handle_commands(uploader, relay, homing, updater, machine, trim)
            if finish_update(uploader, updater):
                # Leave the loop the ordinary way: the `finally` below flushes the
                # ack and drops the GPIO before anything is signalled.
                restart_for_update = True
                break

            # Who is in charge, evaluated every loop rather than every publish: the
            # lights should follow the boat's actual state at loop rate, not at the
            # 1 Hz the uplink happens to run at. `evaluate()` is pure bookkeeping.
            status = machine.evaluate(relay.engaged)
            lights.set_status(status)

            if now - last_publish >= PUBLISH_PERIOD:
                # Published even when every block is empty: a frame arriving at all
                # is how the dashboard knows we are on the air, and `estop` is what
                # lights its banner.
                telemetry = {
                    "safety": safety_telemetry(
                        relay, homing, master is not None, lights
                    ),
                    # Why the status is what it is - which of the three control
                    # links went away. The status itself is a top-level field.
                    "control": machine.telemetry(),
                    "lights": lights.telemetry(),
                }

                # The pack's own figures if the BMS answered, the autopilot's
                # estimate if not, and `source` says which. Never a blend.
                battery_block = merge_battery(
                    bms.telemetry(), battery_telemetry(sys_status, battery)
                )
                if battery_block:
                    telemetry["battery"] = battery_block

                # Position, heading, COG/SOG, distance to the next waypoint. Owns
                # `gps`, `motion` and `autonomy.waypoint`; the dashboard merges
                # telemetry two levels deep, so this cannot clobber a sibling.
                for key, block in navigation.telemetry().items():
                    telemetry.setdefault(key, {}).update(block)
                if trim_block := trim.telemetry():
                    telemetry["trim"] = trim_block

                uploader.publish(
                    status=status,
                    telemetry=telemetry,
                    estop=relay.engaged,
                )
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
        # The relay is now open, so the boat is KILLED whatever it was doing a
        # moment ago. Say so on the hull before the serial port goes: this is the
        # one status change worth spending a few milliseconds on during shutdown,
        # because anyone standing next to the boat is about to see the thrusters
        # go dead and should see red at the same moment.
        lights.set_status("KILLED")
        time.sleep(0.15)  # long enough for the worker to get one write out
        lights.close()
        bms.close()
        if master is not None:
            master.close()
        bus.close()
        uploader.close()  # flushes what is still queued, including the update ack

    # Only reached on a clean exit, and only true after a pull that changed HEAD.
    # The ack is already on its way out, so the supervisor can take the tree down.
    if restart_for_update:
        request_restart()


if __name__ == "__main__":
    main()
