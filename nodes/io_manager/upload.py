"""Telemetry uplink: push frames from the vessel to the operator dashboard.

`Uploader` POSTs JSON frames to ``/api/ingest`` on ``live.ligmax.no`` and hands
back any commands the operator had queued, because the reply to an ingest
carries them - the boat never has to poll a second endpoint. The frame format
is `ligmax-server/ligmax_gui/protocol.py`; every field is optional and the
server merges each frame into its live snapshot, so a node can push whichever
part of the state it owns and ignore the rest.

Why HTTPS and not the UDP ingest on 8771: 8771 is never port-forwarded and is
unauthenticated by default, so from the water the only route to the dashboard
is 443 -> Cloudflare -> Caddy -> Flask on 127.0.0.1:3338 (docs/hosting.md).
A ``http://<host>:3338`` target also works, for a bench test on the same LAN.

Design rules, because this is imported by the node that drives actuators:

  * `publish()` and `log()` never block and never raise. Both hand off to one
    daemon thread, so a 5G dropout cannot back-pressure the MAVLink loop.
  * Frames coalesce per key (newest value wins) and log lines queue, so a
    frame superseded before it went out loses nothing but stale telemetry.
  * The TLS connection is kept alive between frames. A fresh handshake per
    POST costs more on a mobile uplink than the frame it carries.
  * Nothing here uses `logging` itself - `attach_logging()` installs a handler
    on the root logger, and a log call inside the uploader would feed itself.

Usage:

    from .upload import Uploader

    up = Uploader.from_env()
    up.attach_logging()                    # mirror this node's logs upward
    up.publish(telemetry={"battery": {"soc": 0.87, "voltage": 48.2}})
    for command in up.commands():          # operator input, never blocks
        handle(command)
    up.close()
"""

from __future__ import annotations

import http.client
import json
import logging
import os
import queue
import ssl
import threading
import time
from typing import Any, Callable
from urllib.parse import urlparse

DEFAULT_URL = "https://live.ligmax.no"

# The server refuses a bigger body (ligmax-server/ligmax_gui/server.py:48).
MAX_FRAME_BYTES = 4 * 1024 * 1024
MAX_QUEUED_LOGS = 400
MAX_LOGS_PER_FRAME = 200  # normalise_frame() keeps 500; stay well inside it
REQUEST_TIMEOUT = 6.0
ERROR_BACKOFF = 0.5
REJECTED_BACKOFF = 15.0  # a wrong key will not become right by retrying at 10 Hz
IDLE_TICK = 0.5
CLOSE_TIMEOUT = 3.0


def _jsonable(value: Any) -> Any:
    """numpy -> list, Enum -> value, everything else left alone."""
    if hasattr(value, "tolist"):  # numpy array or scalar
        return value.tolist()
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, (str, bool, int, float)) or value is None:
        return value
    if hasattr(value, "value") and hasattr(type(value), "__members__"):
        return value.value  # an Enum member
    return value


class UploadLogHandler(logging.Handler):
    """Feeds stdlib log records to an `Uploader`, dropping them if it backs up."""

    def __init__(self, uploader: "Uploader") -> None:
        super().__init__()
        self._uploader = uploader

    def emit(self, record: logging.LogRecord) -> None:
        try:
            self._uploader.log(
                record.levelname,
                self.format(record) if record.exc_info else record.getMessage(),
                name=record.name,
                t=record.created,
            )
        except Exception:  # a logging handler must never break its caller
            pass


class Uploader:
    """Push telemetry to the dashboard and pull operator commands back."""

    def __init__(
        self,
        target: str = DEFAULT_URL,
        key: str | None = None,
        *,
        min_interval: float = 0.1,
        verify_tls: bool = True,
        on_command: Callable[[dict[str, Any]], None] | None = None,
    ) -> None:
        """`target` is the dashboard root - ``https://live.ligmax.no``.

        `min_interval` is the floor between POSTs; publishing faster than that
        coalesces rather than queues. `on_command` is called on the uploader's
        thread if given, otherwise commands wait in `commands()`.
        """
        parsed = urlparse(target if "://" in target else f"https://{target}")
        self.scheme = parsed.scheme.lower()
        if self.scheme not in ("http", "https"):
            raise ValueError(f"upload target must be http or https, got {target!r}")

        self.host = parsed.hostname or "127.0.0.1"
        self.port = parsed.port or (443 if self.scheme == "https" else 80)
        self.path = parsed.path.rstrip("/") + "/api/ingest"
        self.url = f"{self.scheme}://{self.host}:{self.port}{self.path}"

        self.key = (key or "").strip() or None
        self.min_interval = max(0.0, min_interval)
        self.verify_tls = verify_tls

        self.sent_frames = 0
        self.dropped_frames = 0
        self.dropped_logs = 0
        self.last_error: str | None = None
        self.last_status: int | None = None

        self._on_command = on_command
        self._seq = 0
        self._outbox: dict[str, Any] = {}
        self._acks: list[dict[str, Any]] = []
        self._logs: queue.Queue[dict[str, Any]] = queue.Queue(MAX_QUEUED_LOGS)
        self._commands: queue.Queue[dict[str, Any]] = queue.Queue(256)
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._closed = False
        self._connection: http.client.HTTPConnection | None = None
        self._log_handler: UploadLogHandler | None = None

        self._thread = threading.Thread(target=self._run, daemon=True, name="uploader")
        self._thread.start()

    @classmethod
    def from_env(cls, **kwargs: Any) -> "Uploader":
        """Build from the environment `update.py` and the service unit already set.

        ``LIGMAX_UPLOAD_URL``, else ``LIGMAX_DEPLOY_URL`` (both are the dashboard
        root, and the second is already in /etc/ligmax/node.env), else
        ``https://live.ligmax.no``. ``LIGMAX_BOAT_KEY`` is the ingest secret from
        ligmax-server/.env - without it the server still accepts our frames, but
        it would accept anyone else's too (docs/hosting.md).

        ``LIGMAX_UPLOAD_INSECURE=1`` skips certificate verification. That is only
        for a LAN test straight at Caddy, whose Cloudflare origin certificate is
        trusted by Cloudflare and by nothing else; over the real path the cert is
        Cloudflare's and verifies normally.
        """
        target = (
            os.environ.get("LIGMAX_UPLOAD_URL")
            or os.environ.get("LIGMAX_DEPLOY_URL")
            or DEFAULT_URL
        )
        insecure = os.environ.get("LIGMAX_UPLOAD_INSECURE", "").strip().lower()
        kwargs.setdefault("verify_tls", insecure not in ("1", "true", "yes", "on"))
        return cls(target, os.environ.get("LIGMAX_BOAT_KEY"), **kwargs)

    # -- public API ---------------------------------------------------------

    def publish(self, **fields: Any) -> None:
        """Merge fields into the next frame. Never blocks, never raises.

        Any key the protocol defines works: `telemetry`, `boat`, `tracks`,
        `path`, `mode`, `estop`, `status_text`, `origin`... Calling this with no
        fields still sends a frame, which is how the dashboard sees that the
        link is alive when nothing has changed.
        """
        if self._closed:
            return
        try:
            frame = {key: _jsonable(value) for key, value in fields.items()}
        except Exception as exc:  # a weird object in telemetry is not fatal
            self.last_error = f"frame not convertible: {exc}"
            self.dropped_frames += 1
            return
        frame["t"] = time.time()  # boat clock at sample time, not at send time
        with self._lock:
            self._outbox.update(frame)
        self._wake.set()

    def log(
        self, level: str, message: str, name: str = "boat", t: float | None = None
    ) -> None:
        """Queue a log line for the dashboard's log panel. Never blocks."""
        entry = {
            "level": str(level).upper(),
            "msg": str(message),
            "name": str(name),
            "t": t if t is not None else time.time(),
        }
        try:
            self._logs.put_nowait(entry)
        except queue.Full:
            self.dropped_logs += 1
        else:
            self._wake.set()

    def attach_logging(
        self, logger: logging.Logger | None = None, level: int = logging.INFO
    ) -> UploadLogHandler:
        """Mirror a logger (the root by default) into the dashboard."""
        handler = UploadLogHandler(self)
        handler.setLevel(level)
        (logger or logging.getLogger()).addHandler(handler)
        self._log_handler = handler
        return handler

    def commands(self) -> list[dict[str, Any]]:
        """Drain operator commands received since the last call. Never blocks."""
        out: list[dict[str, Any]] = []
        while True:
            try:
                out.append(self._commands.get_nowait())
            except queue.Empty:
                return out

    def ack(
        self, command_id: str, status: str = "acked", result: str | None = None
    ) -> None:
        """Report a command's outcome; rides along on the next frame."""
        ack: dict[str, Any] = {"id": str(command_id), "status": str(status)}
        if result is not None:
            ack["result"] = str(result)
        with self._lock:
            self._acks.append(ack)
        self._wake.set()

    def stats(self) -> dict[str, Any]:
        """Uplink health, for a log line or for `telemetry.uplink`."""
        return {
            "url": self.url,
            "authenticated": self.key is not None,
            "sent_frames": self.sent_frames,
            "dropped_frames": self.dropped_frames,
            "dropped_logs": self.dropped_logs,
            "last_status": self.last_status,
            "last_error": self.last_error,
        }

    def close(self) -> None:
        """Stop accepting frames, flush what is already queued, drop the socket."""
        if self._closed:
            return
        self._closed = True
        if self._log_handler is not None:
            logging.getLogger().removeHandler(self._log_handler)
            self._log_handler = None
        self._wake.set()
        self._thread.join(CLOSE_TIMEOUT)
        self._drop_connection()

    def __enter__(self) -> "Uploader":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    # -- sender thread ------------------------------------------------------

    def _run(self) -> None:
        while True:
            # Cleared before taking, so a publish() that lands during the POST
            # leaves the event set and we come straight back round.
            self._wake.clear()
            frame = self._take()
            if frame is None:
                if self._closed:
                    return
                self._wake.wait(IDLE_TICK)
                continue
            self._post(frame)
            if self.min_interval and not self._closed:
                time.sleep(self.min_interval)

    def _take(self) -> dict[str, Any] | None:
        """Everything queued, as one frame. None if there is nothing to send."""
        with self._lock:
            frame, self._outbox = self._outbox, {}
            if self._acks:
                frame["acks"] = self._acks
                self._acks = []

        logs: list[dict[str, Any]] = []
        while len(logs) < MAX_LOGS_PER_FRAME:
            try:
                logs.append(self._logs.get_nowait())
            except queue.Empty:
                break
        if logs:
            frame.setdefault("logs", []).extend(logs)

        if not frame:
            return None
        frame.setdefault("t", time.time())
        frame["seq"] = self._seq
        self._seq += 1
        return frame

    def _encode(self, frame: dict[str, Any]) -> bytes | None:
        try:
            payload = json.dumps(frame, separators=(",", ":"), allow_nan=False).encode()
        except (TypeError, ValueError) as exc:
            # One unserialisable telemetry value must not kill the stream. NaN
            # is the usual culprit: allow_nan=False because JSON has no NaN and
            # the dashboard's parser would reject the whole frame.
            self.last_error = f"frame not serialisable: {exc}"
            self.dropped_frames += 1
            return None
        if len(payload) > MAX_FRAME_BYTES:
            self.last_error = f"frame was {len(payload)} B, over the server's limit"
            self.dropped_frames += 1
            return None
        return payload

    def _post(self, frame: dict[str, Any]) -> None:
        payload = self._encode(frame)
        if payload is None:
            return

        headers = {
            "Content-Type": "application/json",
            "Connection": "keep-alive",
            "User-Agent": "ligmax-pi/upload",
        }
        if self.key:
            headers["Authorization"] = f"Bearer {self.key}"

        # Two attempts: a kept-alive socket the far end has since closed fails
        # on the write, and that failure says nothing about the next try.
        for attempt in (1, 2):
            connection = self._get_connection()
            try:
                connection.request("POST", self.path, body=payload, headers=headers)
                response = connection.getresponse()
                body = response.read()  # must drain before the socket is reused
            except (http.client.HTTPException, OSError, ssl.SSLError) as exc:
                self._drop_connection()
                if attempt == 2 or self._closed:
                    self.last_error = str(exc) or exc.__class__.__name__
                    self.dropped_frames += 1
                    self._backoff(ERROR_BACKOFF)
                continue

            self.last_status = response.status
            if response.status == 200:
                self.sent_frames += 1
                self.last_error = None
                self._absorb(body)
            else:
                self.dropped_frames += 1
                self.last_error = f"HTTP {response.status} {body[:200]!r}"
                if response.status in (401, 403):
                    # Wrong or missing LIGMAX_BOAT_KEY. Back off hard.
                    self._backoff(REJECTED_BACKOFF)
                else:
                    self._backoff(ERROR_BACKOFF)
            return

    def _backoff(self, seconds: float) -> None:
        """Sleep, but give up the moment `close()` is called."""
        deadline = time.time() + seconds
        while not self._closed and time.time() < deadline:
            time.sleep(min(0.1, deadline - time.time()))

    def _get_connection(self) -> http.client.HTTPConnection:
        if self._connection is None:
            if self.scheme == "https":
                self._connection = http.client.HTTPSConnection(
                    self.host,
                    self.port,
                    timeout=REQUEST_TIMEOUT,
                    context=self._ssl_context(),
                )
            else:
                self._connection = http.client.HTTPConnection(
                    self.host, self.port, timeout=REQUEST_TIMEOUT
                )
        return self._connection

    def _ssl_context(self) -> ssl.SSLContext:
        context = ssl.create_default_context()
        if not self.verify_tls:
            context.check_hostname = False
            context.verify_mode = ssl.CERT_NONE
        return context

    def _drop_connection(self) -> None:
        connection, self._connection = self._connection, None
        if connection is not None:
            try:
                connection.close()
            except OSError:
                pass

    def _absorb(self, body: bytes) -> None:
        """Pick the operator's queued commands out of the ingest reply."""
        try:
            payload = json.loads(body or b"{}")
        except ValueError:
            return
        if not isinstance(payload, dict):
            return
        for command in payload.get("commands") or []:
            if not isinstance(command, dict):
                continue
            if self._on_command is not None:
                try:
                    self._on_command(command)
                except Exception as exc:  # callback bugs stay contained
                    self.last_error = f"on_command raised: {exc}"
                continue
            try:
                self._commands.put_nowait(command)
            except queue.Full:
                pass


if __name__ == "__main__":
    # Smoke test: python -m nodes.io_manager.upload
    uploader = Uploader.from_env()
    print(f"posting one frame to {uploader.url}")
    uploader.log("INFO", "upload.py smoke test", name="io_manager")
    uploader.publish(telemetry={"uplink": uploader.stats()}, status_text="upload test")
    time.sleep(1.5)
    print(json.dumps(uploader.stats(), indent=2))
    uploader.close()
