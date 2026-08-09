"""Push finished trip recordings off the boat, onto the ground station.

    trips = TripUploader.from_env()
    trips.note_vessel(armed=machine.armed, recording=recording)   # every loop
    trips.request("a recording just closed")                      # or on demand
    telemetry["trips"] = trips.telemetry()
    trips.close()

`nodes/self_driving/recorder.py` writes one gzipped JSONL file per run to the
Pi's own card, and those files are the only thing that can answer "why did it do
that" after the fact. They are also stuck on a 32 GB card, on a boat, and **the
run most worth reviewing is very often the one where the boat had to be carried
back** - which is precisely when nobody is going to plug an Ethernet cable into
it in time to look before attempt two.

So this walks them up to `live.ligmax.no`, which has ~200 GB free, over the same
4G the telemetry uses. The server end is `ligmax-server/ligmax_gui/trips.py`;
that module names this file in a comment, and this is that file.

The one rule the protocol has
------------------------------
Each piece must start exactly where the file on the server currently ends.
Nothing is stitched, nothing is reordered, and the sender needs no durable state
of its own to obey it: ask `GET /api/trip` how many bytes are already held and
send from there. A refusal carries `bytes_held` on the error body as well, so
even losing our place costs one round trip rather than a whole file.

    GET  /api/trip?boat=ligmax                 -> {"trips": [...],
                                                   "pending": {name: bytes}, ...}
    POST /api/trip/<name>?boat=ligmax
         Authorization: Bearer $LIGMAX_BOAT_KEY
         Content-Type:  application/gzip
         Content-Range: bytes 2097152-4194303/62914560
                                               -> {"ok": true, "complete": ...,
                                                   "bytes_held": ...}

Why it is chunked at all, and why the chunk is small
-----------------------------------------------------
A 60 MB POST from a boat on 4G will not always finish, and a scheme that starts
again from zero on every drop can fail forever on a link that is merely bad
rather than absent. The server allows 8 MB a request; this sends **2 MB** by
default, because the cost of a dropped chunk is the chunk, and on a link whose
throughput is measured in hundreds of kB/s an 8 MB piece is half a minute of
exposure to a handover that will take it out.

Why it stays out of the way, and how
-------------------------------------
This is the lowest-priority thing on the uplink and it must behave like it. The
same 4G carries the operator's command channel, the camera and ~95 kB/s of lidar
plot; a recording that arrives an hour late costs nothing, and one that arrives
during a scored run at the expense of the command channel could cost the run.
So, three gates, in order of how much they matter:

  * **Never while the vessel is armed.** Armed is the honest proxy for "this
    boat is doing something": it covers an autonomous attempt, a remote leg and
    a dock trial alike, and it is the one bit io_manager already knows
    first-hand from every HEARTBEAT. `LIGMAX_TRIP_UPLOAD_WHILE_ARMED=1` lifts
    this for a bench test and nothing else.
  * **Never while a recording is open**, and never a file the card has touched
    in the last `QUIET_S` seconds. The first is what the autonomy node tells us;
    the second is the backstop for when it cannot - a half-written gzip stream
    uploaded as though it were finished is a recording that *looks* complete and
    is not, which is worse than one that is visibly missing.
  * **Newest first.** Attempt two is decided from attempt one, so the file the
    crew is waiting for is the one that just closed, not the oldest arrears.

Everything happens on one daemon thread. `note_vessel()` and `request()` only
set fields and an event, so the loop that owes the autopilot its 1 Hz heartbeat
can call them every tick for free.

What it deliberately does not do
---------------------------------
**It never deletes anything.** Not after a successful upload, not to make room.
The recorder's own prune owns the card's budget (`RECORD_KEEP_TRIPS`,
`RECORD_MAX_TOTAL_MB`), and a second thing removing files behind its back is how
the one recording somebody wanted disappears between being asked for and being
fetched. A file that is safely on shore and still on the boat costs nothing.
"""

from __future__ import annotations

import http.client
import json
import logging
import os
import re
import ssl
import threading
import time
from typing import Any
from urllib.parse import urlparse

from nodes.self_driving.config import RECORD_DIR

from .upload import DEFAULT_URL

log = logging.getLogger("io_manager.trips")

#: What the server will accept as a recording name, mirrored from
#: `ligmax-server/ligmax_gui/trips.py::_NAME_RE` so a name it would refuse is
#: skipped here instead of costing a round trip and an error nobody reads. The
#: recorder produces `20260810-091455-task1.jsonl.gz`, which fits easily.
_NAME_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,120}$")

#: Same, for the vessel segment of the path.
_BOAT_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,40}$")

#: Fallbacks for the two limits the server reports in `GET /api/trip`
#: (`max_mb`, `chunk_mb`). The reported values win whenever a listing has
#: succeeded, so these are only what the first attempt of a cold start assumes -
#: which is why it is not a problem that they are a second copy of a number that
#: lives in ligmax-server. If the two ever disagree, the server's answer is the
#: one that decides, because it is the one enforcing it.
SERVER_MAX_TRIP_BYTES = 256 * 1024 * 1024
SERVER_MAX_CHUNK_BYTES = 8 * 1024 * 1024

#: Long enough for a 2 MB body to cross a bad 4G link, short enough that a dead
#: socket does not hold the sweep for a minute. The listing gets its own,
#: shorter, timeout: it is a few hundred bytes and a slow one means the link is
#: not worth attempting a chunk on anyway.
CHUNK_TIMEOUT = 45.0
LIST_TIMEOUT = 10.0

#: Backoffs. A wrong `LIGMAX_BOAT_KEY` will not become right by retrying, and a
#: full disk on the ground station needs a human, so both wait a long time.
ERROR_BACKOFF = 20.0
REJECTED_BACKOFF = 300.0
FULL_BACKOFF = 600.0

#: How long the thread sleeps when it has nothing to do and nobody has asked.
IDLE_TICK = 1.0

CLOSE_TIMEOUT = 3.0


def _b(name: str, default: bool) -> bool:
    raw = os.environ.get(name)
    if raw is None:
        return default
    return raw.strip().lower() in ("1", "true", "yes", "on")


def _f(name: str, default: float) -> float:
    try:
        value = float(os.environ.get(name, default))
    except (TypeError, ValueError):
        return default
    # NaN and infinity are rejected for the same reason `self_driving/config.py`
    # rejects them: every comparison below is a `<` or a `>=`, and NaN makes all
    # of them False, which silently disables the gate rather than failing loudly.
    if value != value or value in (float("inf"), float("-inf")):
        return default
    return value


class TripUploader:
    """Walks finished recordings up to the dashboard. Never raises at the caller."""

    def __init__(
        self,
        target: str = DEFAULT_URL,
        key: str | None = None,
        *,
        directory: str = RECORD_DIR,
        boat: str = "ligmax",
        enabled: bool = True,
        verify_tls: bool = True,
        chunk_bytes: int = 2 * 1024 * 1024,
        period_s: float = 60.0,
        quiet_s: float = 30.0,
        while_armed: bool = False,
    ) -> None:
        parsed = urlparse(target if "://" in target else f"https://{target}")
        self.scheme = parsed.scheme.lower()
        if self.scheme not in ("http", "https"):
            raise ValueError(f"upload target must be http or https, got {target!r}")
        self.host = parsed.hostname or "127.0.0.1"
        self.port = parsed.port or (443 if self.scheme == "https" else 80)
        self.root = parsed.path.rstrip("/") + "/api/trip"
        self.url = f"{self.scheme}://{self.host}:{self.port}{self.root}"

        self.key = (key or "").strip() or None
        self.directory = directory
        self.boat = (boat or "ligmax").strip()
        self.enabled = bool(enabled)
        self.verify_tls = verify_tls
        self.chunk_bytes = max(64 * 1024, int(chunk_bytes))
        self.period_s = max(5.0, period_s)
        self.quiet_s = max(0.0, quiet_s)
        self.while_armed = bool(while_armed)

        if not _BOAT_RE.match(self.boat):
            log.error(
                "LIGMAX_BOAT_NAME=%r is not a name the server will accept - "
                "falling back to 'ligmax'",
                self.boat,
            )
            self.boat = "ligmax"

        # What the server told us it will accept, once a listing has succeeded.
        self.max_trip_bytes = SERVER_MAX_TRIP_BYTES
        self.max_chunk_bytes = SERVER_MAX_CHUNK_BYTES

        # Vessel state, fed from the io_manager loop. `None` means "not known
        # yet", which is treated as *not* armed: on a bench with no autopilot
        # there is no heartbeat and never will be, and refusing to upload until
        # one arrives would make this feature dead on exactly the machine people
        # test it on. The recording gate below is the one that protects the file.
        self._armed: bool | None = None
        self._recording = False
        self._recording_file: str | None = None

        self.sent = 0
        self.sent_bytes = 0
        self.held = 0
        self.queued = 0
        self.local = 0
        self.server_free_mb: float | None = None
        self.last_error: str | None = None
        self.last_sweep: float | None = None
        self.paused_because: str | None = None
        self.sending: str | None = None
        self.progress: float | None = None
        #: Recordings this process will not try again, and why. A file the server
        #: refuses on its own terms - too big, badly named - would otherwise be
        #: retried every minute forever, and the log would say so every minute.
        self.refused: dict[str, str] = {}

        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._closed = False
        self._next_sweep = 0.0
        self._backoff_until = 0.0
        self._connection: http.client.HTTPConnection | None = None

        self._thread = threading.Thread(
            target=self._run, daemon=True, name="trip-upload"
        )
        if self.enabled:
            self._thread.start()

    @classmethod
    def from_env(cls, **kwargs: Any) -> "TripUploader":
        """Build from `/etc/ligmax/node.env`, sharing the ingest link's settings.

        The target and the key are deliberately the *same* ones `upload.py` uses -
        `LIGMAX_UPLOAD_URL`/`LIGMAX_DEPLOY_URL` and `LIGMAX_BOAT_KEY` - because
        they are the same dashboard and the same secret, and a second pair of
        variables to keep in step is a second thing to get wrong at 08:00.
        `LIGMAX_TRIP_UPLOAD_URL` overrides for the case where recordings should
        go somewhere the telemetry does not.

        The recording directory is imported from
        `nodes/self_driving/config.py` rather than read here, so
        `LIGMAX_AP_RECORD_DIR` moves both the writer and the reader together.
        Two copies of a path is how an uploader ends up watching an empty
        directory and reporting, truthfully, that there is nothing to send.
        """
        target = (
            os.environ.get("LIGMAX_TRIP_UPLOAD_URL")
            or os.environ.get("LIGMAX_UPLOAD_URL")
            or os.environ.get("LIGMAX_DEPLOY_URL")
            or DEFAULT_URL
        )
        insecure = os.environ.get("LIGMAX_UPLOAD_INSECURE", "").strip().lower()
        kwargs.setdefault("verify_tls", insecure not in ("1", "true", "yes", "on"))
        kwargs.setdefault("enabled", _b("LIGMAX_TRIP_UPLOAD", True))
        kwargs.setdefault("boat", os.environ.get("LIGMAX_BOAT_NAME", "ligmax"))
        kwargs.setdefault(
            "chunk_bytes",
            int(_f("LIGMAX_TRIP_UPLOAD_CHUNK_MB", 2.0) * 1024 * 1024),
        )
        kwargs.setdefault("period_s", _f("LIGMAX_TRIP_UPLOAD_PERIOD_S", 60.0))
        kwargs.setdefault("quiet_s", _f("LIGMAX_TRIP_UPLOAD_QUIET_S", 30.0))
        kwargs.setdefault("while_armed", _b("LIGMAX_TRIP_UPLOAD_WHILE_ARMED", False))
        return cls(target, os.environ.get("LIGMAX_BOAT_KEY"), **kwargs)

    # -- fed from the io_manager loop ---------------------------------------

    def note_vessel(
        self,
        armed: bool | None,
        recording: bool,
        recording_file: str | None = None,
    ) -> None:
        """The gates, refreshed. Never blocks; safe to call every loop tick.

        **All three arguments are the current truth, not a patch.** Nothing is
        remembered from the last call, deliberately: the failure this avoids is
        a gate that latches. `machine.armed` goes back to None when the MAVLink
        link drops (`status.py::link_down`), and a `note_vessel` that only
        applied non-None values would hold "armed" forever after a boat that was
        armed when its Pixhawk fell off the USB - which is §1.1, the exact
        failure this vessel has, and which is also precisely when somebody wants
        the recording off the card.

        `recording_file` is the basename the autonomy node says it is writing,
        from `telemetry.autopilot.recording.file`. It is used on top of the
        `recording` flag rather than instead of it, because the flag can be a
        second stale (it rides the 1 Hz bridge telemetry) and the name cannot:
        a file that is named as the open one is never a candidate, whatever the
        flag says about it.
        """
        self._armed = None if armed is None else bool(armed)
        self._recording = bool(recording)
        self._recording_file = (
            os.path.basename(recording_file) or None if recording_file else None
        )

    def request(self, reason: str = "asked") -> None:
        """Sweep now rather than at the next period. Never blocks."""
        if self._closed or not self.enabled:
            return
        log.info("trip upload: sweeping now (%s)", reason)
        self._next_sweep = 0.0
        # A pending backoff is dropped: the operator asking is new information
        # about the link, and making them wait out a five-minute rejection
        # backoff after they have fixed the key is the wrong behaviour.
        self._backoff_until = 0.0
        self._wake.set()

    # -- what the operator sees ---------------------------------------------

    def telemetry(self) -> dict[str, Any]:
        """`telemetry.trips` - what is on the card, what is on shore, what is moving."""
        block: dict[str, Any] = {
            "enabled": self.enabled,
            "authenticated": self.key is not None,
            "dir": self.directory,
            "local": self.local,
            "held": self.held,
            "queued": self.queued,
            "sent": self.sent,
            "sent_mb": round(self.sent_bytes / 1048576.0, 1),
        }
        if self.sending:
            block["sending"] = self.sending
            if self.progress is not None:
                block["progress"] = round(self.progress, 3)
        if self.paused_because:
            block["paused"] = self.paused_because
        if self.server_free_mb is not None:
            block["server_free_mb"] = round(self.server_free_mb)
        if self.last_sweep is not None:
            block["sweep_age_s"] = round(time.time() - self.last_sweep, 1)
        if self.refused:
            block["refused"] = dict(self.refused)
        if self.last_error:
            block["last_error"] = self.last_error
        return block

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._wake.set()
        if self._thread.is_alive():
            self._thread.join(CLOSE_TIMEOUT)
        self._drop_connection()

    def __enter__(self) -> "TripUploader":
        return self

    def __exit__(self, *_exc: object) -> None:
        self.close()

    # -- the worker ---------------------------------------------------------

    def _run(self) -> None:
        while not self._closed:
            self._wake.clear()
            now = time.time()
            if now < self._backoff_until or now < self._next_sweep:
                self._wake.wait(IDLE_TICK)
                continue

            self._next_sweep = now + self.period_s
            try:
                self._sweep()
            except Exception as exc:  # noqa: BLE001 - a sweep must never kill the thread
                self.last_error = f"{exc.__class__.__name__}: {exc}"
                log.warning("trip upload sweep failed: %s", exc)
                self._drop_connection()
                self._backoff_until = time.time() + ERROR_BACKOFF

    def _blocked(self) -> str | None:
        """Why this is not the moment, or None. See the module docstring."""
        if self._armed and not self.while_armed:
            return "the vessel is armed"
        if self._recording:
            return "a recording is open"
        return None

    def _sweep(self) -> None:
        self.paused_because = self._blocked()
        if self.paused_because is not None:
            # Not an error and not worth a log line every minute: this is the
            # normal state of affairs for the whole of a run.
            return

        listing = self._list()
        if listing is None:
            return
        self.last_sweep = time.time()

        held = {
            str(item.get("name"))
            for item in (listing.get("trips") or [])
            if isinstance(item, dict)
        }
        pending = {
            str(name): int(size)
            for name, size in (listing.get("pending") or {}).items()
            if isinstance(size, (int, float)) and size >= 0
        }
        self.held = len(held)
        free = listing.get("free_mb")
        self.server_free_mb = float(free) if isinstance(free, (int, float)) else None
        # The server's own limits win over the mirrored constants above.
        for key, attribute in (("max_mb", "max_trip_bytes"), ("chunk_mb", "max_chunk_bytes")):
            value = listing.get(key)
            if isinstance(value, (int, float)) and value > 0:
                setattr(self, attribute, int(value) * 1024 * 1024)

        candidates = self._candidates(held)
        self.queued = len(candidates)
        if not candidates:
            return

        for name in candidates:
            if self._closed:
                return
            # Re-checked between files rather than only at the top of the sweep:
            # a 60 MB recording is minutes of uploading, and the boat can be
            # armed again for the next attempt well inside that.
            if (why := self._blocked()) is not None:
                self.paused_because = why
                log.info("trip upload: pausing, %s", why)
                return
            if not self._send(name, pending.get(name, 0)):
                # A link problem, already backed off and logged. The rest of the
                # queue is not going to fare better on the same socket.
                return

    def _candidates(self, held: set[str]) -> list[str]:
        """Recordings worth sending, newest first. Never raises.

        Sorted by name rather than by mtime, which is the same order and a
        cheaper one: every recording starts with a fixed-width `%Y%m%d-%H%M%S`
        stamp, so lexicographic order is chronological (the same property
        `recorder._prune` relies on to decide what to delete).
        """
        try:
            names = sorted(
                entry
                for entry in os.listdir(self.directory)
                if entry.endswith(".jsonl.gz")
            )
        except OSError as exc:
            # A missing directory is the ordinary case on a Pi that has never
            # recorded anything, so it is not an error - just nothing to do.
            self.local = 0
            if not isinstance(exc, FileNotFoundError):
                self.last_error = f"cannot read {self.directory}: {exc}"
            return []

        self.local = len(names)
        now = time.time()
        out = []
        for name in reversed(names):  # newest first - see the module docstring
            if name in held or name in self.refused:
                continue
            if self._recording_file and name == self._recording_file:
                continue
            if not _NAME_RE.match(name):
                self._refuse(name, "the server will not accept that name")
                continue
            try:
                stat = os.stat(os.path.join(self.directory, name))
            except OSError:
                continue
            if stat.st_size <= 0:
                # A zero-byte file is a recorder that could not open its stream.
                # The server refuses an empty body outright, so skip it here
                # rather than learning that over 4G once a minute.
                continue
            if now - stat.st_mtime < self.quiet_s:
                # Still being written, or closed a moment ago and possibly still
                # being flushed. It will be here next sweep.
                continue
            if stat.st_size > self.max_trip_bytes:
                self._refuse(
                    name,
                    f"{stat.st_size / 1048576.0:.0f} MB is over the server's "
                    f"{self.max_trip_bytes / 1048576.0:.0f} MB ceiling",
                )
                continue
            out.append(name)
        return out

    def _refuse(self, name: str, why: str) -> None:
        if name not in self.refused:
            log.error("not uploading %s: %s", name, why)
        self.refused[name] = why

    # -- one file -----------------------------------------------------------

    def _send(self, name: str, offset: int) -> bool:
        """Push one recording, resuming from `offset`. False on a link problem.

        Returns True for "this file is dealt with" - stored, already held, or
        refused on its own merits - and False only when the *link* is the
        problem, which is the caller's signal to stop the sweep rather than work
        through the queue failing identically on each one.
        """
        path = os.path.join(self.directory, name)
        try:
            total = os.path.getsize(path)
        except OSError as exc:
            self.last_error = f"{name}: {exc}"
            return True

        chunk_bytes = min(self.chunk_bytes, self.max_chunk_bytes)
        self.sending = name
        self.progress = (offset / total) if total else 0.0
        started = time.time()
        if offset:
            log.warning(
                "resuming %s at %.1f MB of %.1f MB",
                name,
                offset / 1048576.0,
                total / 1048576.0,
            )
        else:
            log.warning("uploading %s (%.1f MB)", name, total / 1048576.0)

        try:
            with open(path, "rb") as handle:
                while offset < total:
                    if self._closed:
                        return False
                    if (why := self._blocked()) is not None:
                        # Mid-file. The `.part` on the server keeps our place and
                        # the next sweep resumes from it, so this costs nothing
                        # but the wait.
                        self.paused_because = why
                        log.info("trip upload: %s paused mid-file, %s", name, why)
                        return False

                    handle.seek(offset)
                    body = handle.read(chunk_bytes)
                    if not body:
                        # The file shrank under us, which should be impossible -
                        # the recorder never rewrites a closed trip. Do not send
                        # a truncated file that would look complete.
                        self._refuse(name, "the file shrank while it was being sent")
                        return True

                    result = self._post(name, body, offset, total)
                    if result is None:
                        return False  # link problem, already backed off

                    status, payload = result
                    if status == 200:
                        if payload.get("already_held"):
                            log.info("%s is already on the ground station", name)
                            self.sending = None
                            return True
                        held = payload.get("bytes_held")
                        offset = (
                            int(held)
                            if isinstance(held, (int, float))
                            else offset + len(body)
                        )
                        self.sent_bytes += len(body)
                        self.progress = offset / total if total else 1.0
                        if payload.get("complete"):
                            self.sent += 1
                            self.sending = None
                            self.last_error = None
                            log.warning(
                                "%s is on the ground station (%.1f MB in %.0f s)",
                                name,
                                total / 1048576.0,
                                time.time() - started,
                            )
                            return True
                        continue

                    if status == 409:
                        # We sent the wrong piece. The refusal says where the
                        # server actually is, which is the whole reason recovery
                        # is one round trip rather than a fresh listing.
                        held = payload.get("bytes_held")
                        if not isinstance(held, (int, float)):
                            self.last_error = f"{name}: {payload.get('error')}"
                            return True
                        held = int(held)
                        if held > total:
                            # The server holds more of this name than we have.
                            # Only a name collision or a rewritten file can do
                            # that, and splicing two different recordings into
                            # one file is exactly what this protocol exists to
                            # prevent. Leave it: the server sweeps an idle
                            # `.part` after two hours and the next attempt after
                            # that starts cleanly.
                            self._refuse(
                                name,
                                f"the server holds {held} bytes of that name and "
                                f"the file here is {total} - not splicing them",
                            )
                            return True
                        log.info("%s: server is at %d, resuming there", name, held)
                        offset = held
                        continue

                    if status in (401, 403):
                        self.last_error = (
                            "the ground station rejected the boat key "
                            f"(HTTP {status})"
                        )
                        log.error(
                            "trip upload: %s - check LIGMAX_BOAT_KEY", self.last_error
                        )
                        self._backoff_until = time.time() + REJECTED_BACKOFF
                        return False

                    if status == 507:
                        self.last_error = str(payload.get("error") or "no space")
                        log.error("trip upload: %s", self.last_error)
                        self._backoff_until = time.time() + FULL_BACKOFF
                        return False

                    if status in (400, 413):
                        # The server's own judgement of this file, and retrying
                        # will get the same answer.
                        self._refuse(name, str(payload.get("error") or f"HTTP {status}"))
                        return True

                    self.last_error = f"{name}: HTTP {status} {payload.get('error')}"
                    log.warning("trip upload: %s", self.last_error)
                    self._backoff_until = time.time() + ERROR_BACKOFF
                    return False
        except OSError as exc:
            self.last_error = f"{name}: {exc}"
            log.warning("trip upload: could not read %s: %s", path, exc)
            return True
        finally:
            if self.sending == name:
                self.sending = None
                self.progress = None

        return True

    # -- HTTP ---------------------------------------------------------------

    def _list(self) -> dict[str, Any] | None:
        """`GET /api/trip`, or None with the backoff already set."""
        result = self._request(
            "GET", f"{self.root}?boat={self.boat}", None, {}, LIST_TIMEOUT
        )
        if result is None:
            return None
        status, payload = result
        if status == 200:
            self.last_error = None
            return payload
        if status in (401, 403):
            self.last_error = f"the ground station rejected the boat key (HTTP {status})"
            log.error("trip upload: %s - check LIGMAX_BOAT_KEY", self.last_error)
            self._backoff_until = time.time() + REJECTED_BACKOFF
            return None
        self.last_error = f"listing failed: HTTP {status} {payload.get('error')}"
        self._backoff_until = time.time() + ERROR_BACKOFF
        return None

    def _post(
        self, name: str, body: bytes, offset: int, total: int
    ) -> tuple[int, dict[str, Any]] | None:
        headers = {
            "Content-Type": "application/gzip",
            # Always sent, even when the whole file fits in one piece. The server
            # accepts a bare body as "this is all of it", but sending the range
            # unconditionally means the resume path is the only path there is,
            # so it is exercised on every upload rather than only on the ones
            # that have already gone wrong once.
            "Content-Range": f"bytes {offset}-{offset + len(body) - 1}/{total}",
        }
        return self._request(
            "POST",
            f"{self.root}/{name}?boat={self.boat}",
            body,
            headers,
            CHUNK_TIMEOUT,
        )

    def _request(
        self,
        method: str,
        path: str,
        body: bytes | None,
        headers: dict[str, str],
        timeout: float,
    ) -> tuple[int, dict[str, Any]] | None:
        """One request, retried once on a dead keep-alive. None on a link failure.

        The retry is the same one `upload.py` makes and for the same reason: a
        socket the far end closed while it was idle fails on the write, and that
        failure says nothing at all about whether the next attempt will work.
        """
        sent = dict(headers)
        sent["Connection"] = "keep-alive"
        sent["User-Agent"] = "ligmax-pi/trip-upload"
        if self.key:
            sent["Authorization"] = f"Bearer {self.key}"

        for attempt in (1, 2):
            connection = self._get_connection(timeout)
            try:
                connection.request(method, path, body=body, headers=sent)
                response = connection.getresponse()
                raw = response.read()  # drain before the socket is reused
            except (http.client.HTTPException, OSError, ssl.SSLError) as exc:
                self._drop_connection()
                if attempt == 2 or self._closed:
                    self.last_error = str(exc) or exc.__class__.__name__
                    self._backoff_until = time.time() + ERROR_BACKOFF
                    return None
                continue

            try:
                payload = json.loads(raw or b"{}")
            except ValueError:
                payload = {}
            if not isinstance(payload, dict):
                payload = {}
            return response.status, payload
        return None

    def _get_connection(self, timeout: float) -> http.client.HTTPConnection:
        # A connection of its own, not the ingest link's. Sharing one would put a
        # 2 MB body in front of the telemetry frame that carries the operator's
        # command channel - on a link where that body can take half a minute.
        if self._connection is None:
            if self.scheme == "https":
                self._connection = http.client.HTTPSConnection(
                    self.host, self.port, timeout=timeout, context=self._ssl_context()
                )
            else:
                self._connection = http.client.HTTPConnection(
                    self.host, self.port, timeout=timeout
                )
        else:
            # http.client fixes the timeout at construction, so a reused socket
            # would keep the listing's 10 s for a 2 MB chunk.
            try:
                self._connection.timeout = timeout
                if self._connection.sock is not None:
                    self._connection.sock.settimeout(timeout)
            except (AttributeError, OSError):
                pass
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
