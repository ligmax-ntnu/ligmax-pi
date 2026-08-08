"""The seam between this node and `io_manager`, over ZeroMQ on loopback.

    link = NodeLink()
    state = link.latest_state()          # newest BoatState, or None
    for command in link.commands(): ...  # operator commands aimed at us
    link.control(cmd="position_target", lat=..., lon=...)
    link.telemetry(autopilot={...})
    link.ack(command_id, "acked", "under way")

Why a seam at all, when both processes are on the same Pi
---------------------------------------------------------
Because **only one process can hold `/dev/ttyACM0`**. `io_manager` owns the
MAVLink link, the shore uplink and the E-stop GPIO; if this node opened its own
autopilot connection the two would interleave on the same serial port and
neither would get a clean mission exchange. So control requests go *through*
io_manager, which is also what guarantees the heartbeat keeps going out while
this node is thinking.

The split is the other way round for the Jetson: **this node binds TCP 3401
directly**, because it is the only consumer that needs the full 10 Hz coloured
cloud and the detections, and relaying ~10 kB ten times a second through
io_manager would cost real 4G-shaped CPU for nothing. io_manager still gets a
copy for the operator's chart, pushed back over this link as a `scan` message -
see `main.py`.

Directions
----------
    io_manager  --PUB IO_STATE_PORT-->    this node    (pose, mode, commands)
    this node   --PUB SELF_DRIVING_PORT-> io_manager   (control, telemetry, acks)

Both are PUB/SUB rather than REQ/REP on purpose. PUB/SUB never blocks and never
waits for a reply, so a stalled or restarting io_manager cannot wedge the
autonomy tick, and a restarting autonomy node cannot wedge the heartbeat. The
cost is that a message sent while the other end is not connected is simply
dropped - which is correct for a control stream where the newest command
supersedes the last one anyway, and is handled explicitly for acks by having
io_manager retry nothing and the operator see the command time out.

Degrading
---------
If pyzmq is missing or a socket will not open, every method here becomes a
no-op and `available` is False. A boat with a broken node bus must still run its
io_manager, its heartbeat and its E-stop.
"""

from __future__ import annotations

import json
import logging
import time

from .config import IO_STATE_PORT, SELF_DRIVING_PORT
from .state import BoatState

log = logging.getLogger("self_driving.link")

# Never let a backlog build if io_manager is slow to drain: the newest state is
# the only one worth having, and an old one is actively misleading.
RECV_LIMIT = 50


class NodeLink:
    """SUB on io_manager's state, PUB for our control and telemetry."""

    def __init__(self, state_port=IO_STATE_PORT, control_port=SELF_DRIVING_PORT):
        self.available = False
        self._zmq = None
        self._sub = None
        self._pub = None
        self._state = None
        self._commands = []
        self.frames = 0
        self.last_error = None

        try:
            import zmq
        except ImportError as exc:
            log.error("node bus unavailable, pyzmq is not installed (%s)", exc)
            return
        try:
            context = zmq.Context.instance()
            sub = context.socket(zmq.SUB)
            # Drop old state rather than queue it. A conflate-like depth of 2
            # (rather than zmq.CONFLATE, which also drops the command list that
            # rides on some frames) keeps latency down without losing a frame
            # that carried an operator command.
            sub.setsockopt(zmq.RCVHWM, 4)
            sub.connect(f"tcp://127.0.0.1:{state_port}")
            sub.setsockopt_string(zmq.SUBSCRIBE, "")

            pub = context.socket(zmq.PUB)
            pub.setsockopt(zmq.SNDHWM, 200)
            pub.setsockopt(zmq.LINGER, 0)
            pub.connect(f"tcp://127.0.0.1:{control_port}")
        except Exception as exc:  # noqa: BLE001 - a broken bus is not fatal
            log.error("could not open the node bus: %s", exc)
            self.last_error = str(exc)[:160]
            return

        self._zmq = zmq
        self._sub = sub
        self._pub = pub
        self.available = True
        log.info(
            "node bus up: state on 127.0.0.1:%s, control on 127.0.0.1:%s",
            state_port,
            control_port,
        )

    # ------------------------------------------------------------------ input

    def poll(self):
        """Drain everything waiting, keeping the newest state. Never blocks.

        Commands accumulate rather than being overwritten - a frame that carried
        `set_plan` must not be lost because a pose frame arrived behind it in
        the same tick.
        """
        if self._sub is None:
            return
        for _ in range(RECV_LIMIT):
            try:
                raw = self._sub.recv_string(flags=self._zmq.NOBLOCK)
            except self._zmq.Again:
                return
            except Exception as exc:  # noqa: BLE001 - drop the frame, keep the node
                self.last_error = str(exc)[:160]
                return
            try:
                frame = json.loads(raw)
            except ValueError:
                continue
            if not isinstance(frame, dict):
                continue
            queued = frame.pop("commands", None)
            if queued:
                self._commands.extend(c for c in queued if isinstance(c, dict))
            self._state = BoatState(frame)
            self.frames += 1

    def latest_state(self):
        """The newest `BoatState`, or None if nothing has ever arrived."""
        return self._state

    def commands(self):
        """Operator commands aimed at the autopilot, drained. Never blocks."""
        out, self._commands = self._commands, []
        return out

    # ----------------------------------------------------------------- output

    def _send(self, payload):
        if self._pub is None:
            return False
        try:
            self._pub.send_string(
                json.dumps(payload, separators=(",", ":"), allow_nan=False),
                flags=self._zmq.NOBLOCK,
            )
            return True
        except Exception as exc:  # noqa: BLE001 - a dropped message is not fatal
            self.last_error = str(exc)[:160]
            return False

    def control(self, **fields):
        """One control request for io_manager to put on the MAVLink link."""
        fields["type"] = "control"
        fields.setdefault("t", time.time())
        return self._send(fields)

    def telemetry(self, **blocks):
        """Telemetry for io_manager to merge into the next dashboard frame."""
        return self._send({"type": "telemetry", "t": time.time(), **blocks})

    def scan(self, scans, lidar=None):
        """The front lidar, relayed so the operator's chart keeps its plot.

        This node holds TCP 3401, so without this the dashboard would lose the
        coloured front cloud the moment autonomy started - which is exactly when
        someone most wants to see it.
        """
        payload = {"type": "scan", "t": time.time(), "scans": scans}
        if lidar is not None:
            payload["lidar"] = lidar
        return self._send(payload)

    def ack(self, command_id, status="acked", result=None):
        """Report an operator command's outcome back through io_manager."""
        payload = {"type": "ack", "id": str(command_id), "status": str(status)}
        if result is not None:
            payload["result"] = str(result)
        return self._send(payload)

    def log(self, level, message, name="self_driving"):
        """A log line for the dashboard's panel, via io_manager."""
        return self._send(
            {
                "type": "log",
                "level": str(level).upper(),
                "msg": str(message),
                "name": name,
                "t": time.time(),
            }
        )

    # ---------------------------------------------------------------- teardown

    def close(self):
        for socket in (self._sub, self._pub):
            if socket is not None:
                try:
                    socket.close(linger=0)
                except Exception:  # noqa: BLE001
                    pass
        self._sub = self._pub = None
        self.available = False

    def stats(self):
        block = {"available": self.available, "frames": self.frames}
        if self.last_error:
            block["last_error"] = self.last_error
        return block
