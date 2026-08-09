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
    (`navigation.py`) - **and the grid the map is drawn in**: `origin`, the
    lat/lon of grid (0, 0), plus the vessel's position in metres from it. The
    chart is metres, not degrees, so without those two the map stays empty
    however good the fix is.
  * publishes the battery, **read off the Daly BMS over CAN** rather than taken
    from the autopilot (`bms.py`). The autopilot's own estimate is kept as a
    labelled fallback and `telemetry.battery.source` says which is answering.
  * publishes what the trim actuators were told to do - battery-slider position
    and the two ama outputs (`trim.py`), commanded values, since neither
    actuator reports anything back.
  * reads the **stabilisation tuning** off the flight controller and writes back
    what the operator changes (`tuning.py`): the roll gains and trim in
    `amas.lua`'s SCR_USER1..6 and the pitch PID and trim in
    `battery_slider.lua`'s BSLD_*. Reading is automatic and repeated, writing is
    ArduPilot's own set-and-save, so a tune survives a reboot of everything in
    the chain without anyone having to write it down.
  * receives the **Jetson's feed on TCP 3401** (`edge_link.py`) - detections and
    the front lidar, the latter already coloured by the cameras - reads this
    Pi's own **aft lidar** off its serial port (`lidar.py`), and publishes both
    as one boat-frame point cloud for the operator's chart (`scan.py`). Until
    this existed nothing on the vessel bound 3401 at all, which is why the
    Jetson still needs `LIDAR=1 ./run.sh` passed by hand.
  * pulls **RTK corrections** from the caster on the ground station and injects
    them into the autopilot, which forwards them to the GNSS (`rtk.py`). The
    base station is on 4G like the boat, so both ends dial out to the ground
    station and meet there. Optional: without it the fix is ordinary 3D.
  * pushes finished **trip recordings** off the card and up to the dashboard
    (`trip_upload.py`). The autonomy node writes them; this node owns every link
    to shore, so it is the one that walks them up - chunked and resumable,
    because a 60 MB file on 4G does not always arrive first time, and never
    while the vessel is armed or a recording is open, because the same uplink
    carries the command channel.
  * forwards log lines to the dashboard's log panel: this node's own, the
    ZeroMQ node bus on LOGGING_PORT, and the autopilot's STATUSTEXT.
  * executes the operator's commands, which arrive in the reply to each
    telemetry POST: `estop`, `estop_clear` and `home_battery` drive the two
    GPIO lines in `emergency_stop.py`; `set_mode` and `arm`/`disarm` ask the
    autopilot directly; `set_mission` uploads an admin-laid route of grid
    waypoints as a real MAVLink mission (`mission.py`) for AUTO to run, and
    `clear_waypoints` empties it; `set_param` writes one stabilisation gain or
    trim and `get_params` re-reads the lot (`tuning.py`); `set_lights_mode`,
    `set_lights_pattern` and `set_lights_fps` are `/led_control`'s
    standard/custom switch, its pattern, and its refresh rate (`lights.py`);
    `set_ride_height` walks the amas up or down as an RC override on channel 14
    (`pixhalwk.py`), refreshed here every loop because the autopilot expires
    one that goes quiet, and `release_ride_height` hands that channel back to
    the receiver - a separate command because it is not a stop.
    Every command is acked, so the dashboard's
    command list shows what actually happened on the vessel - though
    `set_mode`/`arm`/`disarm` can only ack that the message was *sent*, not
    that the vehicle obeyed; watch `mode` and `telemetry.control.armed` on the
    next heartbeat for that.

What it does not do yet: the single-point `goto` command still has no
GUIDED-mode implementation - only a laid mission (AUTO) runs today. Nothing here
disarms the autopilot on its
own initiative either - the E-stop cuts propulsion *power* at the contactor
rather than asking the Pixhawk nicely.

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

from config import LOGGING_PORT, pixhawk_port

from .autopilot_bridge import AUTOPILOT_COMMANDS, AutopilotBridge
from .bms import BmsReader
from .edge_link import ENABLED as EDGE_LINK_ENABLED, EdgeLink
from .emergency_stop import BatteryHoming, EstopRelay
from .lidar import LidarReader
from .lights import Lights
from .mission import MissionUploader, parse_waypoints
from .navigation import Navigation
from .pixhalwk import RideHeight
from .propulsion import PropulsionWatch
from .rtk import ENABLED as RTK_ENABLED, RtkClient, inject as inject_rtcm
from .scan import ScanPublisher
from .selfupdate import NAME as REPO_NAME, SelfUpdate, request_restart
from .status import StatusMachine
from .trim import Trim
from .trip_upload import TripUploader
from .tuning import TUNABLES, Tuning
from .upload import Uploader

# /dev/ttyACM0 is not stable across a replug - it can come back as ttyACM1, and
# on a reboot the number depends on probe order. `pixhawk_port()` resolves the
# udev symlink instead (config.py, 99-ligmax-serial.rules) and is re-read on
# every connect() so a device that appears late still gets found.
# LIGMAX_MAVLINK_DEVICE still overrides, and still takes a SITL endpoint, e.g.
# LIGMAX_MAVLINK_DEVICE=udpin:127.0.0.1:14550
MAVLINK_BAUD = int(os.environ.get("LIGMAX_MAVLINK_BAUD", "115200"))

STREAM_RATE_HZ = 4  # only battery and status are consumed today
HEARTBEAT_PERIOD = 1.0
PUBLISH_PERIOD = 1.0

# The lidar plot runs on its own tick, an order of magnitude faster than the rest
# of the telemetry, because it is the only block anyone watches in real time -
# you steer by it and you debug the mounting geometry with it, and at 1 Hz both
# of those are guesswork. Battery, trim and status do not change meaningfully
# inside a second and stay on PUBLISH_PERIOD.
#
# Frames still coalesce per key in the uploader, so this does NOT multiply the
# whole frame by ten: nine of every ten POSTs carry only `scans`, and the tenth
# carries everything. What it does cost is real, and measured rather than
# guessed: a 270-point sweep from each unit, the front one carrying colour,
# serialises to **9.5 kB**, so 10 Hz is about **95 kB/s (0.77 Mbit/s)** on the
# same 4G uplink as the camera and the command channel. That is affordable and
# it is not free - it is roughly what the camera stream costs. Turn it down with
# LIGMAX_SCAN_PUBLISH_HZ if the link is tight; 0 disables the fast tick entirely
# and the plot falls back to riding the 1 Hz publish.
SCAN_PUBLISH_HZ = float(os.environ.get("LIGMAX_SCAN_PUBLISH_HZ", "10"))
SCAN_PERIOD = (1.0 / SCAN_PUBLISH_HZ) if SCAN_PUBLISH_HZ > 0 else None

# How often the autonomy node is handed the boat's pose over the node bus.
# Matched to its own 10 Hz tick, and unrelated to SCAN_PUBLISH_HZ: this one goes
# over loopback to another process on the same Pi, costs no 4G, and is what the
# planner steers on. Turning the operator's plot down must not blind the boat.
STATE_PERIOD = 1.0 / float(os.environ.get("LIGMAX_STATE_PUBLISH_HZ", "10"))

# How long `telemetry.autopilot` may be absent before this node stops believing
# what it last said. Only the trip uploader's "a recording is open" gate reads
# it, and that gate must not latch: an autonomy node that died mid-run would
# otherwise hold the recording on the card forever, and the run that killed the
# node is the one worth reading. Several times the bridge's 0.5 s publish, so an
# ordinary hiccup never trips it.
AUTOPILOT_TELEMETRY_STALE_S = 10.0

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


def apply_mode(master, mode_name):
    """Ask the autopilot to switch flight mode. Returns `(ok, message)`.

    This acks "requested", not "confirmed": `SET_MODE` gets no reliable ack
    across MAVLink dialects, so the honest claim is that the message went out,
    not that the vehicle is now in that mode. The real confirmation is the
    `mode` field in the next HEARTBEAT, which already rides up as the frame's
    top-level `mode` and `telemetry.control.autopilot_mode` - watch those, not
    this ack, before trusting the boat is doing what was asked.
    """
    mapping = master.mode_mapping() or {}
    if mode_name not in mapping:
        known = ", ".join(sorted(mapping)) or "none seen yet - no heartbeat?"
        return False, f"'{mode_name}' is not a mode this vehicle offers ({known})"
    master.set_mode(mapping[mode_name])
    return True, f"mode change to {mode_name} sent"


def apply_arm(master, arm):
    """Ask the autopilot to arm or disarm. Same "requested, not confirmed" caveat
    as `apply_mode()` - `telemetry.control.armed` is fed from the next HEARTBEAT.
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,  # confirmation
        1.0 if arm else 0.0,  # param1: 1 = arm, 0 = disarm
        0, 0, 0, 0, 0, 0,
    )
    return True, f"{'arm' if arm else 'disarm'} sent"


def handle_commands(
    uploader,
    relay,
    homing,
    updater,
    machine=None,
    trim=None,
    navigation=None,
    master=None,
    mission=None,
    tuning=None,
    lights=None,
    bridge=None,
    ride_height=None,
):
    """Run the operator's queued commands and ack each one.

    Commands ride back in the reply to a telemetry POST, so this is only as
    prompt as PUBLISH_PERIOD - about a second. The physical E-stop button is in
    series with the relay and needs none of this to work; that is the link that
    has to be trusted, and this one is the convenience.

    Anything not implemented here is acked `failed` on purpose, so the dashboard
    says "failed: not implemented" instead of leaving the operator watching a
    command sit at "sent" until it expires.

    The exception is the autopilot's own commands (`AUTOPILOT_COMMANDS`), which
    belong to `nodes/self_driving`. They are collected and returned rather than
    handled, so the caller can put them on the node bus - and they are NOT acked
    here, because the autonomy node acks them itself and two answers to one
    command is worse than none.

    Four commands are the exception to acking here, because each rides on an
    exchange that can span several loop ticks: `update` starts a `git pull` on a
    worker thread and is acked later by `finish_update()`; `set_mission` and
    `clear_waypoints` start a MAVLink mission exchange and are acked later by
    `finish_mission()`; `set_param` waits for the autopilot to echo back the
    value it stored and is acked by `finish_tuning()`. None of them may block the
    loop that owes the autopilot its 1 Hz heartbeat.
    """
    for_autopilot = []
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

        if name in AUTOPILOT_COMMANDS:
            # The autonomy node's, not ours. Collected here and put on the node
            # bus by the caller; it acks them itself.
            if bridge is None or not bridge.available:
                ok, result = False, "the autopilot node bus is not available"
            else:
                for_autopilot.append(command)
                continue
        elif name == "estop":
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
        elif name == "recentre_origin":
            # Re-zeroing the grid moves everything on the chart - the boat, the
            # track history, every obstacle - so it only ever happens when the
            # operator asks. The next usable fix supplies the new origin, which
            # is a second or so away at 1 Hz.
            if navigation is None:
                ok, result = False, "no navigation source on this node"
            else:
                navigation.recentre()
                ok, result = True, "grid origin cleared; re-zeroing on the next fix"
        elif name == "set_mode":
            mode_name = str(args.get("mode", "")).strip().upper()
            if master is None:
                ok, result = False, "no autopilot link"
            elif not mode_name:
                ok, result = False, "'mode' is required"
            else:
                ok, result = apply_mode(master, mode_name)
        elif name in ("arm", "disarm"):
            if master is None:
                ok, result = False, "no autopilot link"
            else:
                ok, result = apply_arm(master, arm=(name == "arm"))
        elif name == "clear_waypoints":
            if master is None:
                ok, result = False, "no autopilot link"
            elif mission is None:
                ok, result = False, "no mission handler on this node"
            elif mission.busy:
                ok, result = False, "refused: another mission command is already in flight"
            elif command_id is None:
                ok, result = False, "clear_waypoints needs a command id to ack against"
            else:
                mission.clear(master, str(command_id))
                continue  # acked by finish_mission() once MISSION_ACK lands
        elif name == "set_mission":
            # Points arrive in grid metres - the same frame `goto` uses and the
            # one the dashboard's map is drawn in - and are converted to lat/lon
            # here, against whatever origin `navigation` has captured, because
            # the autopilot's mission protocol only speaks global coordinates.
            points = parse_waypoints(args.get("points"))
            if master is None:
                ok, result = False, "no autopilot link"
            elif mission is None:
                ok, result = False, "no mission handler on this node"
            elif navigation is None or navigation.origin is None:
                ok, result = False, "no GPS origin yet - the grid is not georeferenced"
            elif points is None:
                ok, result = False, "'points' must be a non-empty list of [x, y] pairs"
            elif mission.busy:
                ok, result = False, "refused: another mission command is already in flight"
            elif command_id is None:
                ok, result = False, "set_mission needs a command id to ack against"
            else:
                global_points = [navigation.to_global(x, y) for x, y in points]
                mission.upload(master, str(command_id), points, global_points)
                continue  # acked by finish_mission() once MISSION_ACK lands
        elif name == "set_param":
            # One stabilisation gain or trim, straight into the flight
            # controller's own storage. `tuning.py` holds the whitelist and the
            # ranges: a name that is not on it, or a value outside its bounds, is
            # refused here rather than written and regretted. The ack waits for
            # the autopilot to echo the value it actually stored.
            if master is None:
                ok, result = False, "no autopilot link"
            elif tuning is None:
                ok, result = False, "no tuning handler on this node"
            elif command_id is None:
                ok, result = False, "set_param needs a command id to ack against"
            else:
                queued, why = tuning.queue_write(
                    str(command_id), args.get("name"), args.get("value")
                )
                if queued:
                    continue  # acked by finish_tuning() once PARAM_VALUE lands
                ok, result = False, why
        elif name in ("set_ride_height", "release_ride_height"):
            # The amas' travel, as an RC override on channel 14. `amas.lua` adds
            # that channel to the remote's 3 and to SCR_USER6, and the
            # translator ESP32 reads the sum as a VELOCITY: 1500 is stop, and
            # anything else keeps both amas moving for as long as it is sent.
            # This is the "move it now while I watch" route; SCR_USER6 via
            # `set_param` is the trim meant to stay set across a reboot.
            #
            # Releasing is its OWN command rather than a flag, because it is not
            # a stop: 1500 holds the channel at the translator's own STOP, while
            # releasing hands channel 14 back to the receiver - and if the
            # transmitter has that channel parked off centre, letting go is what
            # starts the creep rather than what ends it. Two buttons, two audit
            # entries.
            #
            # A `release` flag on `set_ride_height` is still honoured, and that
            # is deliberate rather than leftover: the flag was the shape this
            # command had first, and anything still sending it means "let go".
            # Ignoring it would drive the amas instead - the exact inversion
            # worth being defensive about on a velocity command.
            releasing = name == "release_ride_height" or bool(args.get("release"))
            if master is None:
                ok, result = False, "no autopilot link"
            elif ride_height is None:
                ok, result = False, "no ride-height handler on this node"
            elif relay.engaged:
                # Same rule as home_battery: the amas draw from the pack the
                # relay just isolated, so this could only pretend to work.
                ok, result = False, "refused: clear the emergency stop first"
                log.warning("%s %s", name, result)
            elif releasing:
                ok, result = ride_height.release()
            else:
                ok, result = ride_height.command(args.get("pwm"))
        elif name == "set_lights_mode":
            # The /led_control switch: standard (status-driven) vs. an admin's
            # authored test pattern. `lights.py` itself refuses to honour this
            # while the boat is KILLED, whatever it is set to - that guarantee
            # lives there, not here, so it holds even if this handler is wrong.
            if lights is None:
                ok, result = False, "no lights driver on this node"
            else:
                custom = bool(args.get("custom"))
                lights.set_override(custom)
                ok, result = True, "custom pattern" if custom else "standard status"
        elif name == "set_lights_pattern":
            # The pattern itself: a solid colour, a per-pixel array, or a
            # looping multi-frame animation authored on /led_control. Shown
            # only once the switch above is also on. `set_pattern()` never
            # raises - a malformed payload is just refused.
            if lights is None:
                ok, result = False, "no lights driver on this node"
            elif lights.set_pattern(args.get("frames")):
                ok, result = True, "pattern loaded"
            else:
                ok, result = False, "pattern rejected: see the log for why"
        elif name == "set_lights_fps":
            # How often the worker in lights.py redraws - the breathe and
            # strobe as well as a loaded pattern's playhead. `set_fps()`
            # clamps rather than refuses, so this always acks ok.
            if lights is None:
                ok, result = False, "no lights driver on this node"
            else:
                lights.set_fps(args.get("fps"))
                ok, result = True, "fps updated"
        elif name == "get_params":
            # Re-read the lot. Automatic on connect and once a minute anyway;
            # this is the button for after someone has been in Mission Planner.
            if master is None:
                ok, result = False, "no autopilot link"
            elif tuning is None:
                ok, result = False, "no tuning handler on this node"
            else:
                tuning.request_all(f"dashboard command {command_id}")
                ok, result = True, f"re-reading {len(TUNABLES)} parameters"
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
    return for_autopilot


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


def finish_mission(uploader, mission):
    """Ack a finished mission upload or clear, mirroring `finish_update()`.

    A successful upload is echoed back as a `path` with `kind: "reference"` -
    the dashboard already draws exactly this as the amber "ideal route" layer
    (`ligmax-server/web/js/map.js`), which is what an admin-laid mission *is*:
    the course as laid out, for the boat to compare its actual track against.
    A successful clear removes it the same way `clear_waypoints` promises to -
    sending `paths: []` replaces the list outright rather than merging, since
    frames merge dicts but not lists (`ligmax_gui/state.py`).
    """
    outcome = mission.take()
    if outcome is None:
        return
    command_id, ok, message, kind, grid_points = outcome
    uploader.ack(command_id, "acked" if ok else "failed", message)
    if not ok:
        return
    if kind == "upload":
        uploader.publish(path={"points": grid_points, "kind": "reference", "label": "mission"})
    elif kind == "clear":
        uploader.publish(paths=[])


def finish_tuning(uploader, tuning):
    """Ack every parameter write that has settled, mirroring `finish_mission()`.

    A write is only acked once the autopilot has echoed the value it stored, so
    "acked" here means the number is in the flight controller's own storage and
    will still be there after a power cycle - which is the whole reason the
    dashboard can present this as saving rather than as sending.
    """
    while (outcome := tuning.take()) is not None:
        command_id, ok, message = outcome
        uploader.ack(command_id, "acked" if ok else "failed", message)


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
    # Resolved per attempt, not at import: on a cold boot this node can be up
    # before the autopilot has finished enumerating, and a path captured at
    # import time would then be the fallback name forever.
    device = pixhawk_port()
    log.info("opening %s at %s baud", device, MAVLINK_BAUD)
    master = mavutil.mavlink_connection(device, baud=MAVLINK_BAUD)
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

    # The POST floor has to sit below the scan tick, or it becomes the thing
    # limiting the plot rather than the network. The uploader sleeps
    # `min_interval` AFTER each POST, so the achieved rate is 1/(RTT +
    # min_interval): at the default 0.1 s floor a 10 Hz tick could never do
    # better than ~7 Hz even on a perfect link, and on 4G rather less. Half the
    # scan period leaves the round trip as the only real limit, which is the
    # honest one - and coalescing means overshooting the link's capacity costs
    # dropped duplicates, not a growing queue.
    uploader = Uploader.from_env(
        min_interval=0.1 if SCAN_PERIOD is None else min(0.1, SCAN_PERIOD / 2)
    )
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
    # The only way the Pi learns that the physical E-stop went in: the VESCs are on
    # the Pixhawk's CAN bus, so their telemetry stops. Pure bookkeeping, fed below.
    propulsion = PropulsionWatch()
    lights = Lights()
    # Say something immediately rather than waiting for the first publish: until
    # the autopilot is heard from, the honest colour is not green.
    lights.set_status(machine.evaluate(relay.engaged, propulsion.propulsion_permitted))

    # Navigation and trim are fed from the MAVLink pump below; the BMS runs itself.
    navigation = Navigation()
    trim = Trim()
    # Admin-laid waypoint missions and mode/arm changes - see mission.py. Owns no
    # thread or socket of its own; every call takes `master` explicitly, like the
    # heartbeat send below, so it can never race that connection object.
    mission = MissionUploader()
    # The roll and pitch gains and trims, which live on the flight controller as
    # ArduPilot parameters. Reads itself in as soon as there is a link; writes
    # only what the operator asks for. Owns no thread either - see tuning.py.
    tuning = Tuning()
    if not trim.configured:
        log.warning(
            "no servo channels configured for the trim readback, so the battery "
            "slider and ama figures will be absent from the dashboard. Read the "
            "SERVOn_FUNCTION mapping off the flight controller and set "
            "LIGMAX_AMA_PORT_CH / LIGMAX_AMA_STARBOARD_CH / LIGMAX_SLIDER_CH"
        )
    bms = BmsReader()

    # The two lidars, and the plot they become. Both own their own thread and
    # this loop only ever reads their newest answer, exactly like the BMS above:
    #
    #   edge_link  binds TCP 3401 and takes the Jetson's stream - detections and
    #              the front lidar, already coloured by the cameras. Until this
    #              existed nothing on the vessel listened there at all, which is
    #              why `ligmax-edge/run.sh` still needs `LIDAR=1` to be passed by
    #              hand. It can be turned off with LIGMAX_EDGE_LINK_ENABLED=0.
    #   aft        this Pi's own C1 on its USB serial port, facing astern. A
    #              missing sensor costs a log line a minute and nothing else.
    #   scans      puts both into the boat frame and hands the publish tick one
    #              list of points - see scan.py for the hand-measured geometry.
    edge_link = EdgeLink() if EDGE_LINK_ENABLED else None
    if edge_link is not None:
        edge_link.start()
    else:
        log.info(
            "not binding the Jetson port - nodes/self_driving owns it and relays "
            "the front sweep back (LIGMAX_EDGE_OWNER=io_manager to take it back)"
        )
    aft_lidar = LidarReader()
    aft_lidar.start()
    scans = ScanPublisher(edge_link, aft_lidar)

    # The seam to `nodes/self_driving`: this node's pose and mode go out, its
    # control requests come back and go onto the MAVLink link here. Binding
    # both sockets costs nothing when the autonomy node is not running, and
    # everything below degrades to exactly today's behaviour if pyzmq is
    # missing (`autopilot_bridge.py`).
    bridge = AutopilotBridge()

    # The amas' travel, as an RC override on channel 14 (`pixhalwk.py`). Inert
    # until an operator commands it: until then this node does not write channel
    # 14 at all and the receiver owns it, exactly as before this was wired up.
    ride_height = RideHeight()

    # RTK corrections, pulled from the caster on the ground station and injected
    # into the autopilot for forwarding to the GNSS. Its own thread and its own
    # socket; the loop only drains a deque. Off if LIGMAX_RTK_ENABLED=0, and a
    # caster that never answers costs nothing but a log line.
    rtk = RtkClient() if RTK_ENABLED else None
    if rtk is None:
        log.info("RTK corrections disabled (LIGMAX_RTK_ENABLED=0)")

    # The autonomy node's trip recordings, walked up to the ground station on
    # their own thread and their own socket. Chunked, resumable and gated on the
    # vessel being idle - see `trip_upload.py`. It costs one directory listing a
    # minute when there is nothing to send, and nothing at all while the boat is
    # armed. LIGMAX_TRIP_UPLOAD=0 turns it off.
    trip_uploads = TripUploader.from_env()
    log.info(
        "trip recordings: %s -> %s (%s)",
        trip_uploads.directory,
        trip_uploads.url,
        "enabled" if trip_uploads.enabled else "disabled",
    )

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
    last_scan = 0.0
    last_state = 0.0
    for_autopilot = []
    # Mirrors of the autonomy node's recorder, for the trip uploader's gate.
    autopilot_recording = False
    autopilot_recording_file = None
    last_autopilot_block = 0.0

    try:
        while True:
            now = time.time()

            if master is None and now >= next_connect_attempt:
                try:
                    master = connect()
                    # The vehicle-type-specific mode table, straight from
                    # pymavlink's static tables keyed off the HEARTBEAT this
                    # connection just saw - not a guess, and not the same thing
                    # as `status.py`'s AUTONOMOUS_MODES/PILOTED_MODES, which is
                    # this vehicle's *behaviour* classification of them. Sent
                    # once per connection: the set does not change mid-session,
                    # and the dashboard's mode dropdown only needs it once to
                    # stop reading "vessel has not reported its modes".
                    modes = sorted((master.mode_mapping() or {}).keys())
                    if modes:
                        uploader.publish(available_modes=modes)
                    # Load the tuning table without being asked, so the
                    # dashboard's fields are filled in by the time anyone
                    # opens the page. Nothing is sent to the autopilot
                    # until `pump()` below paces it out.
                    tuning.request_all("autopilot link up")
                except Exception as exc:  # noqa: BLE001 - Pixhawk may not be plugged in
                    log.error("MAVLink link to %s failed: %s", pixhawk_port(), exc)
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
                        elif kind == "ESC_TELEMETRY_1_TO_4":
                            # The VESCs are on the autopilot's CAN bus, not ours,
                            # so this is the Pi's only sight of them - and the
                            # physical E-stop is only visible as their silence.
                            propulsion.note_esc_telemetry(message)
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
                        elif kind == "PARAM_VALUE":
                            # The stabilisation gains and trims, both when we
                            # asked for them and when a write is echoed back.
                            # Its own branch rather than the chain below: the
                            # autopilot answers a full param fetch from any GCS
                            # on this link, so this is the one message type that
                            # can arrive in bulk from something else entirely.
                            tuning.handle(message)
                        elif not navigation.handle(message):
                            # Position, course and mission progress, then a mission
                            # upload/clear exchange in flight, then the servo rail.
                            # All three return False for anything they do not want,
                            # so a message nobody reads costs three lookups.
                            if not mission.handle(master, message):
                                trim.handle(message)

                    # A mission exchange that never gets its next
                    # MISSION_REQUEST_INT or its final MISSION_ACK must not sit
                    # queued forever - cheap to check even when nothing is
                    # pending.
                    mission.check_timeout()

                    # One parameter message per tick at most, paced inside: the
                    # background re-read, and whatever the operator has just
                    # asked to be saved. Cheap when there is nothing to do.
                    tuning.pump(master)

                    # RTCM straight back down the same link. Bounded per tick, so
                    # a burst after a caster reconnect cannot delay the heartbeat
                    # below - corrections are worth having, the heartbeat is worth
                    # keeping the autopilot out of failsafe for.
                    if rtk is not None:
                        injected = 0
                        for chunk in rtk.take():
                            inject_rtcm(master, chunk)
                            injected += len(chunk)
                        if injected:
                            rtk.note_injected(injected)

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
                    log.error("MAVLink link to %s dropped: %s", pixhawk_port(), exc)
                    master.close()
                    master = None
                    sys_status = None
                    battery = None
                    # Drop everything the autopilot was the only source of. A
                    # chart still showing the vessel where it was thirty seconds
                    # ago is worse than one showing nothing, because it looks right.
                    navigation.link_down()
                    trim.link_down()
                    mission.link_down()
                    tuning.link_down()
                    machine.note_link_down()
                    # Not just lost telemetry: the Pixhawk shares the rail the
                    # E-stop cuts, so on this boat a dropped link is itself
                    # evidence propulsion is gone. Measured 2026-08-06 - see
                    # `propulsion.py`.
                    propulsion.note_link_down()
                    next_connect_attempt = now + LINK_FAIL_DELAY

            # The autonomy node's control requests, straight onto the MAVLink
            # link. Every loop rather than every publish: a docking command at
            # 10 Hz arriving at 1 Hz is a boat that creeps in ten times slower
            # than it thinks it is. Cheap when the bus is quiet.
            bridge.pump(master)

            # A standing ride-height command, re-sent before the autopilot
            # expires it. Every loop for the same reason as bridge.pump() above,
            # and a no-op on every tick where nothing is being commanded.
            ride_height.refresh(master)

            forward_log_bus(bus, uploader)
            for entry in bridge.take_logs():
                uploader.log(
                    entry.get("level", "INFO"),
                    entry.get("msg", ""),
                    name=entry.get("name", "self_driving"),
                )
            for ack in bridge.take_acks():
                # The autonomy node answering an operator command it was handed
                # below. Passed through verbatim so the dashboard's command list
                # shows what actually happened.
                uploader.ack(
                    ack.get("id"), ack.get("status", "acked"), ack.get("result")
                )
            if relayed := bridge.take_scan():
                # The front lidar, which the autonomy node owns the port for.
                scans.relay_front(relayed)

            for_autopilot = handle_commands(
                uploader,
                relay,
                homing,
                updater,
                machine,
                trim,
                navigation,
                master,
                mission,
                tuning,
                lights,
                bridge,
                ride_height,
            )
            if finish_update(uploader, updater):
                # Leave the loop the ordinary way: the `finally` below flushes the
                # ack and drops the GPIO before anything is signalled.
                restart_for_update = True
                break
            finish_mission(uploader, mission)
            finish_tuning(uploader, tuning)

            # Who is in charge, evaluated every loop rather than every publish: the
            # lights should follow the boat's actual state at loop rate, not at the
            # 1 Hz the uplink happens to run at. `evaluate()` is pure bookkeeping.
            # `propulsion_permitted` is None until ESC telemetry has arrived at
            # least once, and `evaluate()` reads None as "no opinion, trust the
            # relay" - so on a bench with no autopilot this changes nothing.
            status = machine.evaluate(relay.engaged, propulsion.propulsion_permitted)
            lights.set_status(status)

            # The state frame the autonomy node steers on, at STATE_PERIOD - NOT
            # at the 1 Hz publish tick. A planner running at 10 Hz on a 1 Hz
            # pose is working from a position up to a second stale, which at
            # task speed is a metre of error it cannot see. An operator command
            # bypasses the timer entirely, so pressing stop is never up to a
            # tenth of a second late.
            if for_autopilot or now - last_state >= STATE_PERIOD:
                last_state = now
                bridge.publish_state(
                    navigation=navigation,
                    machine=machine,
                    relay=relay,
                    scans=scans,
                    master=master,
                    commands=for_autopilot,
                )

            # The lidar plot, on its own tick. Nothing else rides this one: the
            # uploader coalesces per key, so these POSTs carry `scans` and the
            # frame's own timestamp and nothing more, and the slow tick below
            # still carries everything. Publishing is non-blocking - a link that
            # cannot keep up coalesces sweeps rather than queueing them, so the
            # operator sees the newest picture at whatever rate the 4G allows
            # instead of a lengthening backlog of stale ones.
            if SCAN_PERIOD is not None and now - last_scan >= SCAN_PERIOD:
                last_scan = now
                # Empty means neither lidar has turned since the last tick, which
                # at 10 Hz is a good fraction of them. Skip the publish entirely
                # rather than pass no fields: `publish()` with nothing in it is a
                # deliberate keepalive, and one of those ten times a second is
                # both pointless and indistinguishable from a real frame in the
                # dashboard's rate counter.
                if scan_fields := scans.publish_fields():
                    uploader.publish(**scan_fields)

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
                    # Why propulsion is believed gone, so an operator can tell a
                    # pressed E-stop from a kicked CAN cable.
                    "propulsion": propulsion.telemetry(),
                    "lights": lights.telemetry(),
                }
                if rtk is not None:
                    # This link only: bytes in, bytes injected, correction age.
                    # Whether it *worked* is `gps.fix` going to RTK_FIXED, and
                    # corrections flowing with the fix stuck at 3D is the
                    # signature of a receiver that is not taking RTCM at all.
                    telemetry["rtk"] = rtk.telemetry()

                # Both lidars, reported separately: they fail for unrelated
                # reasons and by different routes, and "no returns" astern looks
                # exactly like "clear water" astern unless the panel says which.
                telemetry["lidar"] = scans.telemetry()
                if bridge.relayed_lidar is not None:
                    # The front unit's own health, as reported by whichever node
                    # is actually bound to 3401.
                    telemetry["lidar"]["front"] = bridge.relayed_lidar

                # Whether the node bus itself is working. Its own block, because
                # "the autonomy node is not running" and "the bus is not
                # delivering" look identical from the dashboard otherwise.
                telemetry["autopilot_bridge"] = bridge.telemetry()

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

                # The gains and trims themselves, as opposed to what the
                # actuators are doing with them. Always published, even before
                # anything has answered: `known`/`of` and `missing` are how the
                # dashboard says "still reading" or "battery_slider.lua is not
                # loaded" instead of showing empty fields with no explanation.
                telemetry["tuning"] = tuning.telemetry()

                # `origin` and `boat` are what put the vessel on the chart: the
                # dashboard draws a metre grid, not degrees, so a GNSS fix in
                # `telemetry.gps` alone fills the figures and leaves the map
                # empty. `boat` comes back explicitly null when the position is
                # gone, because frames merge (navigation.world()).
                frame = {
                    "status": status,
                    "telemetry": telemetry,
                    "estop": relay.engaged,
                    **navigation.world(),
                }

                # Whatever the autonomy node has published since the last frame:
                # `telemetry.autopilot` (what it is doing and why - NJORD §11.4
                # scores exactly this), `tracks` for the chart's obstacle layer,
                # and `path` for the amber ideal-route layer. Merged rather than
                # nested, because each of those is a top-level protocol field.
                for key, value in bridge.take_telemetry().items():
                    if key == "autopilot":
                        telemetry["autopilot"] = value
                    else:
                        frame[key] = value

                # The trip uploader's two gates, refreshed from what this tick
                # actually knows: whether the vehicle is armed (first-hand, off
                # the HEARTBEAT) and whether the autonomy node has a recording
                # open (second-hand, off the bridge - so it expires rather than
                # latching, see AUTOPILOT_TELEMETRY_STALE_S).
                if autopilot_block := telemetry.get("autopilot"):
                    last_autopilot_block = now
                    recording_block = autopilot_block.get("recording") or {}
                    was_recording = autopilot_recording
                    autopilot_recording = bool(recording_block.get("recording"))
                    autopilot_recording_file = recording_block.get("file")
                    if was_recording and not autopilot_recording:
                        # Do not wait out the sweep period: the crew is standing
                        # on the dock wanting this file, and it is the moment the
                        # boat is least busy.
                        trip_uploads.request("a recording just closed")
                elif now - last_autopilot_block > AUTOPILOT_TELEMETRY_STALE_S:
                    autopilot_recording = False
                    autopilot_recording_file = None
                trip_uploads.note_vessel(
                    machine.armed, autopilot_recording, autopilot_recording_file
                )
                # Published after the frame is built, which is safe because
                # `frame["telemetry"]` is this same dict - the uploader only
                # serialises it at send time.
                telemetry["trips"] = trip_uploads.telemetry()

                if SCAN_PERIOD is None:
                    # The fast tick is switched off, so the plot rides this one
                    # rather than never going out at all.
                    frame.update(scans.publish_fields())
                uploader.publish(**frame)

                # The same block down the node bus, for the autonomy node's trip
                # recording. It goes out on the *next* state frame rather than
                # now, so this tick is not made any longer - and the recording
                # gets the battery, the BMS, the RTK link and the tuning, none of
                # which the autonomy node can see any other way.
                bridge.offer_telemetry(telemetry)
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
        # Both lidars before the autopilot link: the aft one holds a USB serial
        # port that the next run has to be able to reopen, and a C1 left
        # streaming into a closed port is what makes the following start fail
        # with `Wrong body size` (lidar.py).
        aft_lidar.close()
        bridge.close()
        if edge_link is not None:
            edge_link.close()
        if rtk is not None:
            rtk.close()
        # Abandons whatever chunk is in flight rather than finishing the file.
        # That costs nothing: the server keeps the `.part`, and the next start
        # resumes from exactly where this left off.
        trip_uploads.close()
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
