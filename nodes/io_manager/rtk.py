"""Pull RTCM3 corrections off the caster and feed them to the autopilot's GNSS.

The chain this is the last link of:

    LC29H base ──▶ ligmax-subsystems/rtk/base_station.py
                         │  NTRIP SOURCE, outbound over 4G
                         ▼
                   rtk.ligmax.no:2101   (ligmax-server/ligmax_gui/rtk.py)
                         │  NTRIP GET, outbound over 4G  ── this module
                         ▼
                   io_manager ──MAVLink GPS_RTCM_DATA──▶ Pixhawk ──UART──▶ LC29H rover

Both ends of the RTK link are on 4G, so neither can be dialled into; the ground
station is the only box with a forwarded port and sits in the middle. This end
therefore *connects out*, exactly like the telemetry uplink.

Why via the autopilot rather than straight at the receiver
----------------------------------------------------------
The rover GNSS is wired to the Pixhawk, not to the Pi (docs/comms.md). ArduPilot
forwards the payload of every GPS_RTCM_DATA message to its GPS UART unmodified,
so injecting over the existing MAVLink link needs no new cable and no second
claim on the receiver's port. It also means the fix type the operator sees and
the corrections that produced it travel the same path, so `telemetry.gps.fix`
going to RTK_FIXED is real end-to-end proof that this module works.

Corrections are sent **unfragmented, in chunks of at most 180 bytes**. The
message supports a fragmented mode for RTCM frames longer than that, but the
receiver parses a byte stream and does not care where the packet boundaries
fall, while a lost fragment stalls reassembly until the autopilot times it out.
Chunking a stream is the failure-free version of the same thing.

Design rules, the same ones the rest of io_manager follows:

  * **Never blocks the MAVLink loop.** All socket work is on a worker thread.
    The loop only ever calls `take()`, which pops from a deque.
  * **Never raises.** No network, no caster, a wrong password: every one of them
    degrades to a logged retry and telemetry that says the link is down. RTK is
    an accuracy improvement, not a requirement - the boat runs without it.
  * **Never buffers.** Corrections that arrived while MAVLink was down are worse
    than nothing by the time it returns: the receiver would apply an old
    atmosphere to a new epoch. The queue is small and drops the oldest.

`telemetry.rtk` is deliberately about *this* link only - bytes in, bytes
injected, how old the last correction is. Whether it worked is `telemetry.gps.fix`,
and the two disagreeing (corrections flowing, fix stuck at 3D) is the signature
of a rover that is not accepting RTCM at all.
"""

import base64
import logging
import os
import socket
import threading
import time
from collections import deque

log = logging.getLogger("io_manager.rtk")

# Same port as the caster, and the same conventional NTRIP number every survey
# app defaults to. `rtk.ligmax.no` is DNS-only in Cloudflare - NTRIP is not
# HTTPS, so it cannot be proxied (docs/hosting.md).
# LIGMAX_RTK_CASTER, not LIGMAX_RTK_HOST: on the ground station that name means
# the address the caster *binds*, and the two ending up in one /etc/ligmax/node.env
# would be a confusing half-hour.
HOST = os.environ.get("LIGMAX_RTK_CASTER", "rtk.ligmax.no")
PORT = int(os.environ.get("LIGMAX_RTK_PORT", "2101"))
MOUNT = os.environ.get("LIGMAX_RTK_MOUNT", "LIGMAX1")
USER = os.environ.get("LIGMAX_RTK_USER", "")
PASSWORD = os.environ.get("LIGMAX_RTK_PASSWORD", "")
ENABLED = os.environ.get("LIGMAX_RTK_ENABLED", "1").strip().lower() not in (
    "0",
    "false",
    "no",
    "off",
)

# GPS_RTCM_DATA carries a fixed 180-byte array; `len` says how much of it is real.
RTCM_CHUNK = 180

# About four seconds of a single-base stream. Deep enough that a busy MAVLink
# tick does not drop corrections, shallow enough that nothing stale survives a
# real outage - see the module docstring.
QUEUE_DEPTH = 64

RECONNECT_MIN = 2.0
RECONNECT_MAX = 60.0
# The caster drops a source that goes quiet, so a client that has heard nothing
# for this long is looking at a dead base or a dead socket either way.
DATA_TIMEOUT = 45.0
CONNECT_TIMEOUT = 10.0
READ_SIZE = 4096


class RtkClient:
    """NTRIP client on a worker thread. `take()` is the whole interface.

    Owns one socket and one thread. Nothing here touches MAVLink: the caller
    injects, because the MAVLink connection belongs to the main loop and sharing
    it across threads is how a serial link gets interleaved garbage.
    """

    def __init__(self, host=HOST, port=PORT, mount=MOUNT, user=USER, password=PASSWORD):
        self.host, self.port, self.mount = host, port, mount
        self.user, self.password = user, password

        self._lock = threading.Lock()
        self._queue = deque(maxlen=QUEUE_DEPTH)
        self._closed = False
        self._socket = None

        self._connected_at = 0.0
        self._last_data = 0.0
        self._received = 0
        self._dropped = 0
        self._injected = 0
        self._connects = 0
        self._last_error = None

        self._thread = threading.Thread(target=self._run, daemon=True, name="rtk")
        self._thread.start()

    # -- public API ---------------------------------------------------------

    @property
    def connected(self):
        with self._lock:
            return self._socket is not None

    def take(self, limit=16):
        """Up to `limit` chunks of at most RTCM_CHUNK bytes, oldest first.

        Bounded because this is called from the MAVLink loop: a burst of
        corrections after a reconnect must not turn into a burst of MAVLink
        writes that delays the heartbeat.
        """
        out = []
        with self._lock:
            while self._queue and len(out) < limit:
                out.append(self._queue.popleft())
        return out

    def note_injected(self, count):
        """Tell telemetry how many bytes actually reached the autopilot."""
        self._injected += count

    def telemetry(self):
        """The `telemetry.rtk` block. Ages in seconds; absent means never seen."""
        now = time.monotonic()
        with self._lock:
            connected = self._socket is not None
            queued = len(self._queue)
        block = {
            "link": connected,
            "source": f"{self.host}:{self.port}/{self.mount}",
            "bytes": self._received,
            "injected": self._injected,
        }
        if self._last_data:
            # The number that matters. A connected socket with a correction age
            # climbing past a few seconds means the *base* has gone quiet, which
            # looks like a healthy link from here.
            block["correction_age_s"] = round(now - self._last_data, 1)
        if connected and self._connected_at:
            block["uptime_s"] = round(now - self._connected_at, 1)
        if queued:
            block["queued"] = queued
        if self._dropped:
            block["dropped"] = self._dropped
        if self._connects > 1:
            block["reconnects"] = self._connects - 1
        if self._last_error:
            block["last_error"] = str(self._last_error)[:120]
        return block

    def close(self):
        self._closed = True
        with self._lock:
            sock, self._socket = self._socket, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        self._thread.join(2.0)

    # -- worker -------------------------------------------------------------

    def _run(self):
        backoff = RECONNECT_MIN
        while not self._closed:
            sock = self._connect()
            if sock is None:
                # Sleep in slices so close() does not wait out the whole backoff.
                deadline = time.monotonic() + backoff
                while not self._closed and time.monotonic() < deadline:
                    time.sleep(0.25)
                backoff = min(backoff * 2, RECONNECT_MAX)
                continue

            backoff = RECONNECT_MIN
            self._pump(sock)

    def _connect(self):
        try:
            sock = socket.create_connection((self.host, self.port), CONNECT_TIMEOUT)
        except OSError as exc:
            self._note_error(f"cannot reach the caster: {exc}")
            return None

        request = [
            f"GET /{self.mount} HTTP/1.1",
            f"Host: {self.host}:{self.port}",
            "Ntrip-Version: Ntrip/2.0",
            "User-Agent: NTRIP ligmax-pi/1.0",
            "Connection: close",
        ]
        if self.password:
            token = base64.b64encode(
                f"{self.user}:{self.password}".encode("utf-8")
            ).decode("ascii")
            request.append(f"Authorization: Basic {token}")
        try:
            sock.settimeout(CONNECT_TIMEOUT)
            sock.sendall(("\r\n".join(request) + "\r\n\r\n").encode("ascii"))
            reply = sock.recv(1024)
        except OSError as exc:
            self._note_error(f"handshake failed: {exc}")
            sock.close()
            return None

        # A v1 caster answers "ICY 200 OK", a v2 one "HTTP/1.1 200 OK", and a
        # caster that does not know the mountpoint answers with its sourcetable
        # - which is a 200 as well, so check for that explicitly rather than
        # streaming a catalogue into the receiver.
        head = reply[:64].upper()
        if b"SOURCETABLE" in head:
            self._note_error(f"the caster has no mountpoint /{self.mount}")
            sock.close()
            return None
        if b"200" not in head:
            detail = reply.decode("latin-1", "replace").split("\r\n")[0]
            self._note_error(f"caster refused: {detail or 'no reply'}")
            sock.close()
            return None

        with self._lock:
            self._socket = sock
        self._connected_at = time.monotonic()
        self._connects += 1
        self._last_error = None
        log.info("RTK corrections from %s:%s/%s", self.host, self.port, self.mount)

        # The body can start in the same segment as the response headers.
        _, marker, body = reply.partition(b"\r\n\r\n")
        if marker and body:
            self._enqueue(body)
        return sock

    def _pump(self, sock):
        sock.settimeout(1.0)
        while not self._closed:
            try:
                data = sock.recv(READ_SIZE)
            except socket.timeout:
                if time.monotonic() - (self._last_data or self._connected_at) > (
                    DATA_TIMEOUT
                ):
                    self._note_error(
                        f"no corrections for {DATA_TIMEOUT:.0f}s - the base "
                        "station is down, not this link"
                    )
                    break
                continue
            except OSError as exc:
                self._note_error(f"connection lost: {exc}")
                break
            if not data:
                self._note_error("the caster closed the connection")
                break
            self._enqueue(data)

        with self._lock:
            self._socket = None
        try:
            sock.close()
        except OSError:
            pass

    def _enqueue(self, data):
        """Split into GPS_RTCM_DATA-sized chunks and queue them."""
        self._received += len(data)
        self._last_data = time.monotonic()
        with self._lock:
            for start in range(0, len(data), RTCM_CHUNK):
                if len(self._queue) == self._queue.maxlen:
                    # deque(maxlen=) drops the oldest silently; count it, because
                    # "RTK never fixes and nothing looks wrong" is the failure
                    # this counter exists to explain.
                    self._dropped += 1
                self._queue.append(data[start : start + RTCM_CHUNK])

    def _note_error(self, message):
        # Only log a change: a caster that is down would otherwise fill the
        # operator's log panel at the reconnect rate.
        if str(self._last_error) != message:
            log.warning("RTK: %s", message)
        self._last_error = message


def inject(master, chunk):
    """Send one chunk of RTCM to the autopilot for forwarding to the GNSS.

    `flags` is 0: not fragmented, sequence 0. ArduPilot passes the first `len`
    bytes of the payload straight to the GPS port, so a chunk boundary in the
    middle of an RTCM frame is invisible to the receiver - it is reassembling a
    byte stream either way. The 180-byte array is fixed width in the message
    definition, so a short chunk is zero-padded and `len` says how much is real.
    """
    payload = bytes(chunk[:RTCM_CHUNK])
    master.mav.gps_rtcm_data_send(
        0, len(payload), payload + b"\x00" * (RTCM_CHUNK - len(payload))
    )
