"""Record every run, so attempt two can be better than attempt one.

    recorder = TripRecorder(config)
    recorder.start("task1")                 # a run begins
    recorder.sample(state, world, pilot, intent, scans, now)
    recorder.stop("finished")

Njord gives each subtask **two attempts and fifteen minutes** (§8.2). The only
way to spend attempt two well is to know exactly what attempt one did - which
mark the boat saw, which side it decided to pass, what the cardinal vote was
when it committed, how far off the ideal line it drifted. None of that survives
in anyone's memory between two runs on a cold dock, and none of it is in the
dashboard's live view once the frame has scrolled past.

So: one file per run, gzipped JSON lines, on the Pi's own card. Nothing here
depends on the 4G link, because the run worth reviewing is usually the one where
the link was the problem.

Format
------
Line 1 is a **header**: when, why, the git commit, the full config snapshot, the
sensor masks. A trip is only reviewable if you know which numbers the boat was
running, and those change between attempts precisely because somebody is tuning
them between attempts.

Every line after that is a **sample**:

    {"t": ..., "boat": {...}, "autopilot": {...}, "intent": {...},
     "tracks": [...], "scans": [...]}

Two rates, and that is the whole trick to the file size. Pose, decision and
tracks go at `RECORD_HZ` (10 Hz) because they are small and they are the
reasoning. The raw point clouds go at `RECORD_SCAN_HZ` (2 Hz) because they are
~9 kB a sweep and would otherwise be 95 % of the file - a 15 minute run is about
2 MB with them decimated and 35 MB without.

Why JSON lines and gzip rather than a database or rosbag
--------------------------------------------------------
A line-delimited file can be read back after a crash mid-write, by anything, on
any machine, with no schema and no library. That property is worth more than
compactness on a boat where the interesting recording is disproportionately
likely to be the one that ended badly. `gzip` gets most of the compactness back
anyway - the frames are highly repetitive - and Python reads a `.jsonl.gz` in
one line.
"""

from __future__ import annotations

import gzip
import json
import logging
import os
import subprocess
import time

from . import config as config_module
from .perception import masks

log = logging.getLogger("self_driving.recorder")


def _git_head(repo="/home/admin/ligmax-pi"):
    """The commit the boat is running, or None. Never raises, never blocks long.

    Worth the subprocess: "which build was this?" is the first question asked of
    any recording, and the answer is otherwise unrecoverable once the Pi has
    been updated - which happens between attempts, from the dashboard.
    """
    try:
        out = subprocess.run(
            ["git", "-C", repo, "rev-parse", "--short", "HEAD"],
            capture_output=True,
            text=True,
            timeout=2.0,
            check=False,
        )
        return out.stdout.strip() or None
    except (OSError, subprocess.SubprocessError):
        return None


class TripRecorder:
    """One file per run. Never raises; a failed write costs a log line."""

    def __init__(self, config=config_module):
        self.config = config
        self.path = None
        self._handle = None
        self._samples = 0
        self._scans_written = 0
        self._last_sample = 0.0
        self._last_scan = 0.0
        self._started = None
        self.last_error = None

    # ------------------------------------------------------------------ state

    @property
    def recording(self):
        return self._handle is not None

    # ------------------------------------------------------------------ start

    def start(self, label="trip", extra=None):
        """Begin a recording. Returns the path, or None if it could not open.

        Rolls the previous one first, so calling this twice can never interleave
        two runs into one file - which would be the one thing that makes a
        recording useless.
        """
        if not self.config.RECORD_ENABLED:
            return None
        self.stop("superseded")

        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
        safe = "".join(c if c.isalnum() or c in "-_" else "-" for c in str(label))[:40]
        path = os.path.join(self.config.RECORD_DIR, f"{stamp}-{safe}.jsonl.gz")
        try:
            os.makedirs(self.config.RECORD_DIR, exist_ok=True)
            handle = gzip.open(path, "wt", encoding="utf-8", compresslevel=6)
        except OSError as exc:
            self.last_error = str(exc)[:160]
            log.warning("could not start a trip recording at %s: %s", path, exc)
            return None

        self._handle = handle
        self.path = path
        self._samples = 0
        self._scans_written = 0
        self._started = time.time()
        self._write(
            {
                "kind": "header",
                "t": self._started,
                "label": label,
                "local_time": time.strftime("%Y-%m-%d %H:%M:%S"),
                "git": _git_head(),
                "config": self.config.snapshot(),
                "masks": masks.describe(),
                **(extra or {}),
            }
        )
        log.warning("recording this run to %s", path)
        self._prune()
        return path

    # ----------------------------------------------------------------- sample

    def sample(self, state, world, pilot, intent, scans=None, now=None):
        """One tick. Cheap when it is not due; never blocks the loop for long."""
        if self._handle is None:
            return
        now = now if now is not None else time.time()
        if now - self._last_sample < 1.0 / max(0.1, self.config.RECORD_HZ):
            return
        self._last_sample = now

        row = {"t": round(now, 3)}
        if state is not None:
            row["boat"] = {
                "position": state.position,
                "heading": state.heading,
                "cog": state.cog,
                "sog": state.sog,
                "lat": state.lat,
                "lon": state.lon,
                "fix": state.fix,
                "mode": state.mode,
                "armed": state.armed,
                "estop": state.estop,
                "status": state.status,
            }
            if state.origin:
                row["origin"] = state.origin
        if pilot is not None:
            row["autopilot"] = pilot.telemetry(state, world)
        if intent is not None:
            row["intent"] = intent.telemetry()
        if world is not None:
            row["tracks"] = world.telemetry(limit=30)

        # The point clouds, decimated. See the module docstring: at full rate
        # they are 95 % of the file and they are the part you least often need
        # frame by frame.
        if scans and now - self._last_scan >= 1.0 / max(0.1, self.config.RECORD_SCAN_HZ):
            self._last_scan = now
            self._scans_written += 1
            row["scans"] = scans

        self._write(row)
        self._samples += 1

    def event(self, kind, **fields):
        """A one-off worth finding later: a command, a phase change, a fault.

        These are what a reviewer scrubs to. A 15 minute run is 9000 samples and
        perhaps twenty events, and the events are where the story is.
        """
        if self._handle is None:
            return
        self._write({"kind": "event", "t": round(time.time(), 3), "event": kind, **fields})

    # ------------------------------------------------------------------- stop

    def stop(self, why="stopped"):
        if self._handle is None:
            return None
        path = self.path
        try:
            self._write(
                {
                    "kind": "footer",
                    "t": time.time(),
                    "why": why,
                    "samples": self._samples,
                    "scans": self._scans_written,
                    "duration_s": round(time.time() - (self._started or time.time()), 1),
                }
            )
            self._handle.close()
        except (OSError, ValueError) as exc:
            self.last_error = str(exc)[:160]
        self._handle = None
        size = None
        try:
            size = os.path.getsize(path)
        except OSError:
            pass
        log.warning(
            "trip recording closed: %s (%d samples%s)",
            path,
            self._samples,
            f", {size / 1024:.0f} kB" if size else "",
        )
        return path

    # --------------------------------------------------------------- internals

    def _write(self, row):
        try:
            self._handle.write(json.dumps(row, separators=(",", ":"), default=_plain))
            self._handle.write("\n")
        except (OSError, ValueError, TypeError) as exc:
            # One bad row must not end the recording, and a full card must not
            # take the boat down. Note it and carry on.
            self.last_error = str(exc)[:160]

    def _prune(self):
        """Keep the newest `RECORD_KEEP_TRIPS`. The Pi's card is not big."""
        try:
            files = sorted(
                f for f in os.listdir(self.config.RECORD_DIR)
                if f.endswith(".jsonl.gz")
            )
        except OSError:
            return
        for name in files[: max(0, len(files) - self.config.RECORD_KEEP_TRIPS)]:
            try:
                os.remove(os.path.join(self.config.RECORD_DIR, name))
                log.info("pruned old trip recording %s", name)
            except OSError:
                pass

    # ------------------------------------------------------------- telemetry

    def telemetry(self):
        block = {"recording": self.recording, "samples": self._samples}
        if self.path:
            block["file"] = os.path.basename(self.path)
        if self._started and self.recording:
            block["duration_s"] = round(time.time() - self._started, 1)
        if self.last_error:
            block["last_error"] = self.last_error
        return block


def _plain(value):
    """Last-resort JSON coercion, so one odd object cannot drop a whole row."""
    if hasattr(value, "tolist"):
        return value.tolist()
    if hasattr(value, "value"):
        return value.value
    return str(value)
