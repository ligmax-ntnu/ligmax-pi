"""The Jetson's feed, received: detections and the front lidar, on TCP 3401.

This is the listener that `ligmax-edge` has been dialling into all along and
that nothing on this end ever bound (docs/findings.md: "nothing listens on
3401"). `ligmax-edge/run.sh` keeps its lidar behind `LIDAR=1` for exactly one
reason - that a consumer which reads `header["cam"]` without checking `kind`
first files every sweep as camera 0 and blanks that feed. `receiver.py` in that
repo had precisely that bug. So:

    DISPATCH ON `kind` BEFORE TOUCHING `cam`.

`_absorb()` below does, and that is the whole contract this module has to
honour. A header with no `kind` at all is a camera frame - that is what every
sender predating the lidar emitted, and it is read that way rather than
rejected (`edge_protocol.py`).

Shape of it
-----------
The Jetson dials out, because it is the Jetson that knows when it has something
to send and the Pi that is the fixed address on the vessel LAN. So this is a
server socket that accepts one connection at a time, and a Jetson restart just
reconnects.

    link = EdgeLink(); link.start()
    cloud, seq, at = link.front_cloud()   # newest sweep, rig frame, + OUR arrival stamp

Everything is read on this thread and handed over under a lock, because the
caller is the loop that owes the autopilot a 1 Hz heartbeat and must never
block on a socket (`main.py`).

What is deliberately NOT done here
----------------------------------
The detections are received, counted and kept, but they are not turned into
`tracks` for the dashboard. Doing that honestly means deciding where a bearing
and a range put a buoy on the grid, how a cam0 track id relates to a cam1 one
(they are unique per camera and not across the pair - `edge_protocol.py`), and
how they age out. That is the world model, it belongs with the planner in
`nodes/self_driving/`, and inventing half of it here would put buoys on the
operator's chart that nothing on the boat is actually steering around. The
front *lidar* is different: a return is a measured range and bearing, it needs
no interpretation, and it is what `scan.py` publishes.

The preview JPEGs are read off the wire and dropped. The picture an operator
sees comes straight from the Jetson to shore over HTTPS and never passes
through here (`ligmax-edge/cloud_camera.py`); the payload is only consumed
because the stream is length-prefixed and skipping it would desynchronise it.
"""

import logging
import os
import socket
import threading
import time

from . import edge_protocol

log = logging.getLogger("io_manager.edge_link")

# 3401, not 3338: the dashboard binds 3338 on the ground station and
# live.ligmax.no is forwarded there, so the edge feed moved off it
# (docs/findings.md item 1). Matches `ligmax-edge/run.sh`'s default PORT.
DEFAULT_PORT = int(os.environ.get("LIGMAX_EDGE_PORT", "3401"))

# Bind address. 0.0.0.0 because the Jetson reaches us over the vessel LAN; this
# port is not forwarded anywhere and there is no route in from shore anyway
# (docs/comms.md), so there is nothing here to expose.
BIND_HOST = os.environ.get("LIGMAX_EDGE_BIND", "0.0.0.0")

ENABLED = os.environ.get("LIGMAX_EDGE_LINK_ENABLED", "1").strip().lower() not in (
    "0", "false", "no", "off",
)

# The Jetson sends ~14 frames a second per camera. Silence for this long is a
# dead link, not a quiet one - and because `read_message` does partial reads, a
# timeout that fires mid-message leaves the stream misaligned. So a timeout is
# always treated as "drop the connection and wait for it to dial back in",
# never as something to resume from.
IDLE_TIMEOUT = 10.0

# How long the accept loop blocks before checking whether it is being shut down.
ACCEPT_TIMEOUT = 1.0


class EdgeLink(threading.Thread):
    """Accepts the Jetson's stream and keeps the newest of each thing it sends."""

    daemon = True

    def __init__(self, host=BIND_HOST, port=DEFAULT_PORT):
        super().__init__(name="edge-link")
        self.host = host
        self.port = port
        self._lock = threading.Lock()
        self._stopping = threading.Event()
        self._server = None

        self._cloud = None          # newest KIND_LIDAR payload, rig frame
        self._cloud_seq = 0         # our own counter, not the Jetson's
        self._cloud_at = None       # when it landed, OUR clock
        self._cloud_t_end = None    # when the JETSON says the rotation ended

        self._frames = 0
        self._sweeps = 0
        self._dets = [0, 0]         # detections in the newest frame, per camera
        self._frame_at = [None, None]
        self._fps = None
        self._peer = None
        self._bound = False
        self.last_error = None

    # ------------------------------------------------------------------ access
    def front_cloud(self):
        """`(cloud, seq, arrived_at)` - the newest front sweep, or `(None, 0, None)`.

        `cloud` is the columnar payload exactly as it came off the wire: rig
        frame, +x starboard, +y down, +z forward, origin at the front lidar.
        `seq` counts sweeps received here, so a caller can tell a new sweep
        from the same one seen twice without trusting the sender's numbering.

        `arrived_at` is OUR `time.time()` at the moment the sweep landed, and it
        is the only timestamp a caller may age it against. The cloud also
        carries the Jetson's own `t_start`/`t_end`, in the same units, which is
        exactly what makes them dangerous: they are stamped by a different
        machine's wall clock, and judging freshness across that boundary turns
        any clock disagreement into a sensor that is silently never fresh. See
        `clock_offset_s` in `telemetry()`, which measures that gap instead of
        being destroyed by it.
        """
        with self._lock:
            return self._cloud, self._cloud_seq, self._cloud_at

    @property
    def connected(self):
        return self._peer is not None

    def telemetry(self):
        """What the operator needs to tell "no buoys" from "no Jetson"."""
        with self._lock:
            cloud = self._cloud
            age = None if self._cloud_at is None else round(time.time() - self._cloud_at, 2)
            block = {
                "listening": self._bound,
                "port": self.port,
                "connected": self._peer is not None,
                "frames": self._frames,
                "sweeps": self._sweeps,
                "detections": list(self._dets),
                "sweep_age_s": age,
            }
            if self._peer:
                block["peer"] = self._peer
            if self._fps is not None:
                block["jetson_fps"] = self._fps
            if cloud:
                # The two numbers worth watching on the Jetson's own fusion:
                # how many returns a camera could colour, and how far apart in
                # capture time the sweep and the frame that coloured it were.
                # Colour is only meaningful while `skew_ms` stays small.
                block["sweep_points"] = cloud.get("n")
                block["sweep_coloured"] = cloud.get("coloured")
                block["sweep_hz"] = cloud.get("hz")
                skew = cloud.get("skew_ms")
                if isinstance(skew, (list, tuple)):
                    block["skew_ms"] = [s for s in skew if s is not None]
                # How far apart the two machines' wall clocks are: our arrival
                # instant minus the Jetson's end-of-rotation instant. The true
                # flight time is a LAN hop, single-digit milliseconds, so
                # anything past a second is the clocks disagreeing and not the
                # link being slow.
                #
                # Reported, never acted on. Freshness is judged on our own clock
                # (`front_cloud`), so an offset here costs the sweep timestamps
                # their meaning without costing the operator the sensor.
                if self._cloud_t_end is not None and self._cloud_at is not None:
                    block["clock_offset_s"] = round(
                        self._cloud_at - float(self._cloud_t_end), 2)
            if self.last_error:
                block["last_error"] = self.last_error
        return block

    # -------------------------------------------------------------------- loop
    def run(self):
        while not self._stopping.is_set():
            try:
                self._serve()
            except Exception as exc:  # noqa: BLE001 - a bad socket must not kill the node
                self.last_error = str(exc)[:160]
                self._bound = False
                log.warning("edge link on %s:%s: %s; retrying", self.host, self.port, exc)
                if self._stopping.wait(2.0):
                    return

    def _serve(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # Without this a restart inside TIME_WAIT cannot rebind, and the node
        # would sit refusing the Jetson for a minute after every update.
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.settimeout(ACCEPT_TIMEOUT)
        try:
            server.bind((self.host, self.port))
            server.listen(1)
            self._server = server
            self._bound = True
            log.info("listening for the Jetson on %s:%s", self.host, self.port)
            while not self._stopping.is_set():
                try:
                    conn, addr = server.accept()
                except socket.timeout:
                    continue
                self._session(conn, addr)
        finally:
            self._bound = False
            self._server = None
            try:
                server.close()
            except OSError:
                pass

    def _session(self, conn, addr):
        peer = f"{addr[0]}:{addr[1]}"
        log.info("Jetson connected from %s", peer)
        with self._lock:
            self._peer = peer
        try:
            conn.settimeout(IDLE_TIMEOUT)
            while not self._stopping.is_set():
                message = edge_protocol.read_message(conn)
                if message is None:
                    log.info("Jetson %s disconnected", peer)
                    return
                header, _jpeg = message      # the preview is not ours - see above
                self._absorb(header)
        except socket.timeout:
            self.last_error = f"no data from {peer} for {IDLE_TIMEOUT:.0f}s"
            log.warning("Jetson %s went quiet for %.0fs; dropping the connection",
                        peer, IDLE_TIMEOUT)
        except (OSError, ValueError) as exc:
            # ValueError covers a header that is not JSON, which is what a
            # desynchronised stream looks like once read_message has resynced
            # onto a magic that was really payload bytes.
            self.last_error = str(exc)[:160]
            log.warning("Jetson %s: %s", peer, exc)
        finally:
            with self._lock:
                self._peer = None
            try:
                conn.close()
            except OSError:
                pass

    def _absorb(self, header):
        """One message. **Dispatches on `kind` before reading `cam`.**

        A KIND_LIDAR header has no `cam` field, because most of a rotation is
        behind both cameras. Reading `cam` first - or defaulting it to 0 - is
        the bug this whole module is written around.
        """
        kind = header.get("kind", edge_protocol.KIND_FRAME)

        if kind == edge_protocol.KIND_LIDAR:
            cloud = header.get("lidar")
            if not isinstance(cloud, dict):
                return
            t_end = cloud.get("t_end")
            with self._lock:
                self._cloud = cloud
                self._cloud_seq += 1
                self._cloud_at = time.time()
                # Kept only to measure how far apart the two clocks are. A
                # sender that omits it, or sends something that is not a number,
                # costs us the measurement and nothing else.
                self._cloud_t_end = t_end if isinstance(t_end, (int, float)) else None
                self._sweeps += 1
            return

        if kind != edge_protocol.KIND_FRAME:
            # Something newer than this build understands. Counting it and
            # moving on is right: the framing is self-describing, so an unknown
            # message costs nothing, and refusing the connection over one would
            # take the detections down with it.
            return

        cam = header.get("cam")
        if cam not in (0, 1):
            return
        with self._lock:
            self._frames += 1
            self._dets[cam] = len(header.get("dets") or ())
            self._frame_at[cam] = time.time()
            if (fps := header.get("fps")) is not None:
                self._fps = fps

    def shutdown(self):
        self._stopping.set()

    def close(self):
        """Stop accepting and release the port."""
        self.shutdown()
        server = self._server
        if server is not None:
            try:
                server.close()   # unblocks a thread parked in accept()
            except OSError:
                pass
        if self.is_alive():
            self.join(timeout=3.0)
