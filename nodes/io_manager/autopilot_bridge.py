"""The io_manager end of the node bus: state out, control in, MAVLink between.

`nodes/self_driving` does the thinking and this puts the result on the wire.
The split exists for one hard reason and one good one:

  * **hard** - only one process can hold `/dev/ttyACM0`. Two MAVLink connections
    on one serial port interleave and neither gets a clean exchange.
  * **good** - the 1 Hz GCS heartbeat must never be queued behind a planner. It
    stays on this node's loop, where the only things that can delay it are the
    ones that have always been there.

    io_manager  --PUB IO_PORT----------->  self_driving   pose, mode, commands
    io_manager  <--SUB SELF_DRIVING_PORT-  self_driving   control, telemetry, acks

Both directions are PUB/SUB and never block. A self_driving that has crashed
simply stops sending control, this stops relaying, and ArduPilot's own guided
timeout stops the boat a moment later - which is the correct outcome and needs
no code here to arrange.

The two control messages, and their type masks
-----------------------------------------------
    position_target   SET_POSITION_TARGET_GLOBAL_INT, position bits only.
                      ArduPilot's L1 controller steers to it with the vehicle's
                      own tune. Used for every transit leg.
    velocity_target   SET_POSITION_TARGET_LOCAL_NED in MAV_FRAME_BODY_NED,
                      velocity and yaw-rate bits. Used for docking, station
                      keeping and reversing - anything a position target's
                      "drive there" shape cannot express.

The type masks are the fiddly part and getting one wrong fails silently: the
autopilot honours whichever fields the mask says are valid and quietly ignores
the rest, so a mask with the velocity bits set to "ignore" produces a boat that
receives commands at 10 Hz and does not move. They are spelled out below as
named bits rather than written as a magic number, for exactly that reason.

Frames of reference
-------------------
`MAV_FRAME_BODY_NED` is +x forward, +y right, +z down - the same two axes the
rest of this codebase uses (`self_driving/geo.py`), so `vx` and `vy` pass
straight through unflipped. Yaw rate is rad/s, positive to starboard.
"""

from __future__ import annotations

import json
import logging
import math
import time

from pymavlink import mavutil

from config import (
    IO_PORT,
    KNOT_MS,
    SELF_DRIVING_PORT,
    VESSEL_SPEED_LIMIT_KNOTS,
    VESSEL_SPEED_LIMIT_MS,
)

log = logging.getLogger("io_manager.autopilot")

# Commands the dashboard sends that belong to the autonomy node, not to us.
# They are forwarded and acked there; `handle_commands` in main.py must NOT ack
# them, or the dashboard sees two answers to one command.
AUTOPILOT_COMMANDS = frozenset(
    {
        "set_plan",
        "clear_plan",
        "autopilot_start",
        "autopilot_stop",
        "autopilot_pause",
        "autopilot_resume",
        "autopilot_skip",
        "autopilot_back",
        "autopilot_goto",
        "record_start",
        "record_stop",
        "forget_world",
        # Delete one tracked object, by the `track_id` the chart shows. Belongs
        # to the autonomy node for the same reason `forget_world` does: it owns
        # the world model, and it is the only thing that can answer whether the
        # id existed.
        "forget_object",
        # The cardinal alternation prior (`self_driving/behaviours/alternation.py`).
        # Listed since 2026-08-11: the dashboard has offered it since the day the
        # prior was written and it was never in this set, so every press was acked
        # `not implemented` by `handle_commands` and the autonomy node never saw
        # it. Same class of bug as `goto` (docs/findings.md).
        "alternation",
    }
)

# Commands **both** nodes act on, off one press. Forwarded to the autonomy node
# like the set above, but NOT skipped in `handle_commands` - io_manager runs its
# own half as well, and io_manager is the one that acks.
#
# There is exactly one, and it is one on purpose: `set_speed_limit` is the
# operator's single speed. On this node it is the hand-flown go-to's cap and the
# AUTO mission speed (`guided.py`); on the autonomy node it is the ceiling every
# behaviour plans under, docking included (`self_driving/commander.py`). Two
# commands for "how fast may the boat go" is how a dashboard ends up showing one
# figure while the planner uses another - careful mode and `run_profile` were
# exactly that, and both are gone.
SHARED_COMMANDS = frozenset({"set_speed_limit"})

# POSITION_TARGET_TYPEMASK bits. A set bit means IGNORE that field.
_IGNORE_POS = 0b0000000000000111       # x, y, z
_IGNORE_VEL = 0b0000000000111000       # vx, vy, vz
_IGNORE_ACC = 0b0000000111000000       # afx, afy, afz
_IGNORE_YAW = 0b0000010000000000
_IGNORE_YAW_RATE = 0b0000100000000000

# Position only: use x/y/z, ignore everything else.
POSITION_MASK = _IGNORE_VEL | _IGNORE_ACC | _IGNORE_YAW | _IGNORE_YAW_RATE
# Velocity plus yaw rate: use vx/vy/vz and yaw_rate, ignore position and accel.
VELOCITY_MASK = _IGNORE_POS | _IGNORE_ACC | _IGNORE_YAW

# A control message older than this is not acted on. The autonomy node ticks at
# 10 Hz and refreshes its target twice a second; anything this stale means the
# bus backed up, and steering to a two-second-old target is worse than holding.
MAX_CONTROL_AGE_S = 2.0

# RC overrides time out in the autopilot, so the lateral thruster's channel has
# to be refreshed. Anything longer than about a second and it stutters.
RC_REFRESH_S = 0.25

CHANNEL_COUNT = 18
RC_RELEASE = 65535  # "leave this channel to whoever else is driving it"


def _limited(value, what):
    """One speed off the node bus, held to the vessel's limit. Never raises.

    **This is the last thing between a number on the loopback bus and a number on
    the MAVLink wire**, and it exists because everything upstream of it is only
    as trustworthy as the process that wrote it. The autonomy node clamps its own
    commands (`self_driving/commander.py`), but this socket is a PUB/SUB bind on
    127.0.0.1 that any process on the Pi can publish to, and `_absorb` checks
    only the message's *kind* and *age* - not its numbers. Until this existed, a
    `{"cmd": "position_target", "speed": 9.0}` from anywhere on the machine, or
    one bug in the autonomy node's clamp, put 9 m/s into DO_CHANGE_SPEED.

    A limit enforced only by the process being limited is not a limit.

    NaN becomes zero rather than being clamped: ArduPilot's behaviour on a NaN
    parameter is not something to discover during a scored run, and "stop" is the
    only safe reading of "not a number".
    """
    try:
        out = float(value)
    except (TypeError, ValueError):
        return 0.0
    if out != out:  # NaN
        log.warning("refusing a NaN %s from the node bus - commanding 0", what)
        return 0.0
    if abs(out) > VESSEL_SPEED_LIMIT_MS:
        # Loud, every time. If this ever fires in the field it means something
        # upstream is wrong, and it is the kind of wrong that is invisible from
        # the dashboard because the boat simply goes the speed it was allowed.
        log.error(
            "node bus asked for %s %.2f m/s (%.2f kn) - over the %.0f kn vessel "
            "limit, clamping to %.2f m/s. Something upstream is not clamping.",
            what,
            out,
            out / KNOT_MS,
            VESSEL_SPEED_LIMIT_KNOTS,
            VESSEL_SPEED_LIMIT_MS,
        )
        return math.copysign(VESSEL_SPEED_LIMIT_MS, out)
    return out

# How often the dashboard's telemetry block is forwarded to the autonomy node for
# its trip recording. Matched to io_manager's own PUBLISH_PERIOD, which is the
# rate the block is assembled at - sending it faster would just repeat one.
SNAPSHOT_PERIOD = 1.0


class AutopilotBridge:
    """State out, control in. Degrades to a no-op without pyzmq."""

    def __init__(self, state_port=IO_PORT, control_port=SELF_DRIVING_PORT):
        self.available = False
        self._zmq = None
        self._pub = None
        self._sub = None

        self.telemetry_blocks = {}   # merged into the next dashboard frame
        self.acks = []               # forwarded to the dashboard
        self.logs = []
        self.relayed_scan = None     # the front lidar, for the operator's chart
        self.relayed_lidar = None
        self.frames_in = 0
        self.controls_applied = 0
        self.last_control = None
        self.last_control_at = None
        self.last_error = None
        self._rc = None              # (channel, pwm, last sent)
        self._snapshot = None        # the dashboard's telemetry, for the recorder
        self._snapshot_sent = 0.0

        try:
            import zmq
        except ImportError as exc:
            log.warning("autopilot bridge disabled, pyzmq is not installed (%s)", exc)
            return
        try:
            context = zmq.Context.instance()
            pub = context.socket(zmq.PUB)
            pub.setsockopt(zmq.SNDHWM, 8)   # newest state only; never a backlog
            pub.setsockopt(zmq.LINGER, 0)
            pub.bind(f"tcp://127.0.0.1:{state_port}")

            sub = context.socket(zmq.SUB)
            sub.setsockopt(zmq.RCVHWM, 200)
            sub.bind(f"tcp://127.0.0.1:{control_port}")
            sub.setsockopt_string(zmq.SUBSCRIBE, "")
        except Exception as exc:  # noqa: BLE001 - never fatal
            log.error("could not open the autopilot bridge: %s", exc)
            self.last_error = str(exc)[:160]
            return

        self._zmq = zmq
        self._pub = pub
        self._sub = sub
        self.available = True
        log.info(
            "autopilot bridge up: state on 127.0.0.1:%s, control on 127.0.0.1:%s",
            state_port,
            control_port,
        )

    # ------------------------------------------------------------- state out

    def offer_telemetry(self, snapshot):
        """Hand over the dashboard's telemetry block for the autonomy node.

        Called on io_manager's 1 Hz publish tick with the block it has just
        assembled. It is not sent to the planner because a planner needs it - it
        does not - but because the **trip recording** does: "the boat stopped
        because the pack sagged under load" is unprovable from the autonomy
        node's own state, and the autonomy node is what writes the recording.
        """
        self._snapshot = snapshot

    def publish_state(self, *, navigation, machine, relay, scans, master,
                      commands=()):
        """One state frame for the autonomy node. Called on the publish tick.

        Deliberately small: pose, who is in charge, the aft sweep, and any
        operator commands aimed at the autopilot. Everything else the dashboard
        gets - the battery, the tuning, the lights - is of no use to a planner
        and would cost 4G-shaped CPU to serialise ten times a second.

        The one exception is the telemetry snapshot from `offer_telemetry`, and
        it rides at **its own** rate rather than this frame's: at most once every
        `SNAPSHOT_PERIOD`, which is the rate it is produced at anyway, so a
        10 Hz state frame does not carry nine redundant copies of it.
        """
        if self._pub is None:
            return
        frame = {
            "t": time.time(),
            "status": machine._status if machine is not None else None,
            "estop": relay.engaged if relay is not None else None,
            "mode": getattr(master, "flightmode", None) if master is not None else None,
            "armed": machine.armed if machine is not None else None,
            "rc_link": machine.rc_up if machine is not None else False,
            "operator_link": machine.operator_up if machine is not None else False,
        }
        if navigation is not None:
            frame.update(navigation.world())
            for key, block in navigation.telemetry().items():
                frame[key] = block
        # The aft sweep only. The front one comes to the autonomy node straight
        # off TCP 3401 and must not make the round trip.
        if scans is not None:
            frame["aft_scan"] = scans.aft_scan_for_planner()
        if commands:
            frame["commands"] = list(commands)
        now = time.time()
        if self._snapshot is not None and now - self._snapshot_sent >= SNAPSHOT_PERIOD:
            self._snapshot_sent = now
            # Stamped, so the recorder can tell a fresh snapshot from the same
            # one arriving again and write each exactly once.
            frame["telemetry"] = dict(self._snapshot, t=now)
        try:
            self._pub.send_string(
                json.dumps(frame, separators=(",", ":"), allow_nan=False),
                flags=self._zmq.NOBLOCK,
            )
        except Exception as exc:  # noqa: BLE001
            self.last_error = str(exc)[:160]

    # -------------------------------------------------------------- control in

    def pump(self, master, mission=None):
        """Drain the bus and apply what came in. Call every loop tick.

        `master` may be None - the autonomy node still gets its state frames and
        still publishes what it *would* do, which is exactly what you want on a
        bench with no autopilot plugged in.
        """
        if self._sub is None:
            return
        for _ in range(200):
            try:
                raw = self._sub.recv_string(flags=self._zmq.NOBLOCK)
            except self._zmq.Again:
                break
            except Exception as exc:  # noqa: BLE001
                self.last_error = str(exc)[:160]
                break
            try:
                message = json.loads(raw)
            except ValueError:
                continue
            if not isinstance(message, dict):
                continue
            self.frames_in += 1
            self._absorb(message, master)

        # RC overrides expire in the autopilot, so the lateral thruster's
        # channel is refreshed rather than sent once.
        self._refresh_rc(master)

    def _absorb(self, message, master):
        kind = message.get("type")
        if kind == "telemetry":
            for key, value in message.items():
                if key in ("type", "t"):
                    continue
                self.telemetry_blocks[key] = value
            return
        if kind == "scan":
            self.relayed_scan = message.get("scans")
            if message.get("lidar") is not None:
                self.relayed_lidar = message["lidar"]
            return
        if kind == "ack":
            self.acks.append(message)
            return
        if kind == "log":
            self.logs.append(message)
            return
        if kind != "control":
            return

        age = time.time() - float(message.get("t") or 0.0)
        if age > MAX_CONTROL_AGE_S:
            # Steering to a stale target is worse than holding. Say so once
            # rather than silently discarding, since a bus this backed up is a
            # symptom worth seeing.
            log.warning("dropping a %.1f s old control message", age)
            return
        self.last_control = message.get("cmd")
        self.last_control_at = time.time()
        if master is None:
            return
        try:
            self._apply(master, message)
            self.controls_applied += 1
        except Exception as exc:  # noqa: BLE001 - a bad command must not drop the link
            self.last_error = str(exc)[:160]
            log.warning("could not apply control %s: %s", message.get("cmd"), exc)

    def _apply(self, master, message):
        command = message.get("cmd")
        if command == "position_target":
            self._position_target(master, message)
        elif command == "velocity_target":
            self._velocity_target(master, message)
        elif command == "set_mode":
            self._set_mode(master, str(message.get("mode", "")).upper())
        elif command == "arm":
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1.0 if message.get("arm") else 0.0,
                0, 0, 0, 0, 0, 0,
            )
        elif command == "rc_override":
            channel = int(message.get("channel") or 0)
            if 1 <= channel <= CHANNEL_COUNT:
                self._rc = (channel, int(message.get("pwm") or 1500), 0.0)

    def _position_target(self, master, message):
        """Drive to a lat/lon. Speed goes separately - the target has no field
        for it, and DO_CHANGE_SPEED is how Rover is told."""
        lat = message.get("lat")
        lon = message.get("lon")
        if lat is None or lon is None:
            return
        master.mav.set_position_target_global_int_send(
            0,  # time_boot_ms, unused by ArduPilot here
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            POSITION_MASK,
            int(round(float(lat) * 1e7)),
            int(round(float(lon) * 1e7)),
            0.0,           # altitude - meaningless for a surface vessel
            0.0, 0.0, 0.0,  # velocity, ignored by the mask
            0.0, 0.0, 0.0,  # acceleration, ignored by the mask
            0.0, 0.0,       # yaw, yaw rate, ignored by the mask
        )
        speed = message.get("speed")
        if speed is not None:
            self._change_speed(master, _limited(speed, "a groundspeed"))

    def _change_speed(self, master, speed):
        """MAV_CMD_DO_CHANGE_SPEED, at most once a second.

        Re-sent rather than set once because a mode change resets it, and
        rate-limited because it is a COMMAND_LONG - each one gets an ACK back,
        and ten a second is a hundred extra messages a second on a 115200 link
        that also carries the heartbeat.
        """
        now = time.time()
        previous = getattr(self, "_speed", None)
        if previous is not None and abs(previous[0] - speed) < 0.05 and now - previous[1] < 1.0:
            return
        self._speed = (speed, now)
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,
            0,      # param1: speed type - 0 is groundspeed, which is what a boat has
            speed,  # param2: m/s
            -1,     # param3: throttle %, -1 = no change
            0, 0, 0, 0,
        )

    def _velocity_target(self, master, message):
        """Body-frame velocity and yaw rate. Docking, holding, reversing."""
        vx = _limited(message.get("vx") or 0.0, "a forward velocity")
        vy = _limited(message.get("vy") or 0.0, "a lateral velocity")
        resultant = math.hypot(vx, vy)
        if resultant > VESSEL_SPEED_LIMIT_MS:
            # Scaled together, so the commanded *direction* survives: a crab at
            # 30 degrees stays a crab at 30 degrees, just slower. Scaling one
            # axis alone would quietly rotate the manoeuvre, which is a worse
            # failure than being slow.
            scale = VESSEL_SPEED_LIMIT_MS / resultant
            log.error(
                "node bus asked for %.2f m/s resultant (%.2f kn) - over the "
                "%.0f kn vessel limit, scaling both axes by %.3f",
                resultant,
                resultant / KNOT_MS,
                VESSEL_SPEED_LIMIT_KNOTS,
                scale,
            )
            vx, vy = vx * scale, vy * scale
        master.mav.set_position_target_local_ned_send(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            VELOCITY_MASK,
            0.0, 0.0, 0.0,  # position, ignored by the mask
            # Clamped as a resultant, not per axis: speed through the water is
            # the magnitude of the vector, and two individually-legal components
            # can still put the boat over the limit.
            vx,        # forward
            vy,        # starboard
            0.0,
            0.0, 0.0, 0.0,  # acceleration, ignored by the mask
            0.0,
            float(message.get("yaw_rate") or 0.0),  # rad/s, + to starboard
        )

    def _set_mode(self, master, mode_name):
        mapping = master.mode_mapping() or {}
        if mode_name not in mapping:
            log.warning(
                "autonomy asked for mode %r, which this vehicle does not offer (%s)",
                mode_name,
                ", ".join(sorted(mapping)) or "no heartbeat yet",
            )
            return
        master.set_mode(mapping[mode_name])

    def _refresh_rc(self, master):
        """Keep the lateral thruster's channel alive; release it when idle.

        Every other channel is released on every send, so this can never take
        the sticks away from a pilot - the same rule `pixhalwk.set_ride_height`
        follows for channel 16.
        """
        if master is None or self._rc is None:
            return
        channel, pwm, sent_at = self._rc
        now = time.time()
        if now - sent_at < RC_REFRESH_S:
            return
        # An override that stops being refreshed times out in the autopilot, so
        # a centred command is dropped entirely rather than held: that hands the
        # channel back instead of pinning it at neutral.
        if pwm == 1500 and now - sent_at > 1.0:
            self._rc = None
            channels = [RC_RELEASE] * CHANNEL_COUNT
        else:
            self._rc = (channel, pwm, now)
            channels = [RC_RELEASE] * CHANNEL_COUNT
            channels[channel - 1] = pwm
        master.mav.rc_channels_override_send(
            master.target_system, master.target_component, *channels
        )

    # ------------------------------------------------------------ take/report

    def take_telemetry(self):
        """The autonomy node's telemetry blocks, once."""
        blocks, self.telemetry_blocks = self.telemetry_blocks, {}
        return blocks

    def take_acks(self):
        acks, self.acks = self.acks, []
        return acks

    def take_logs(self):
        logs, self.logs = self.logs, []
        return logs

    def take_scan(self):
        """The relayed front sweep, once. None if nothing new has arrived."""
        scan, self.relayed_scan = self.relayed_scan, None
        return scan

    def telemetry(self):
        """`telemetry.autopilot_bridge` - whether the node bus is working.

        Its own block rather than folded into the autonomy node's, because the
        two fail independently: an autonomy node that is not running and a node
        bus that is not delivering look identical from the dashboard unless
        something says which.
        """
        block = {
            "available": self.available,
            "frames_in": self.frames_in,
            "controls_applied": self.controls_applied,
            "last_control": self.last_control,
        }
        if self.last_control_at is not None:
            block["last_control_age_s"] = round(time.time() - self.last_control_at, 2)
        if self.last_error:
            block["last_error"] = self.last_error
        return block

    def close(self):
        for socket in (self._pub, self._sub):
            if socket is not None:
                try:
                    socket.close(linger=0)
                except Exception:  # noqa: BLE001
                    pass
        self._pub = self._sub = None
        self.available = False
