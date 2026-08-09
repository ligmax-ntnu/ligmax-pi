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

Three rates, and that is the whole trick to the file size. Pose, decision and
tracks go at `RECORD_HZ` (10 Hz) because they are small and they are the
reasoning. The raw point clouds go at `RECORD_SCAN_HZ` (2 Hz) because they are
~9 kB a sweep and would otherwise be 95 % of the file - a 15 minute run is about
2 MB with them decimated and 35 MB without. The clusters and their classification
reasoning go at `RECORD_CLUSTER_HZ`, alongside the cloud they were built from,
because reading one without the other answers nothing.

What is recorded, and why it is everything
-------------------------------------------
The rule this file follows is: **if it took part in a decision, it is in the
file.** A recording exists to answer questions asked by somebody who was not on
the boat, hours later, and every field left out is a question that cannot be
answered without running the whole day again. So a sample carries the pose, the
full io_manager telemetry snapshot (battery, BMS, RTK, trim, tuning, propulsion -
"the boat stopped because the pack sagged" is otherwise unprovable), the intent,
the pilot's and the behaviour's own reasoning, what the commander actually put on
the MAVLink wire, the edge link's health, the tick's timing, **every** track
including the unconfirmed ones, and the clusters with the sentence explaining how
each classified.

The one thing that is never written
------------------------------------
**Camera frames.** Not at any rate, not compressed, not "just the interesting
ones". The Pi's card is 32 GB with the operating system on it, JPEGs are three
orders of magnitude bigger than everything else here put together, and the
Jetson already pushes previews straight to shore over HTTPS without touching this
machine (`ligmax-edge/cloud_camera.py`). The per-point `rgb` that IS written is
three bytes per lidar return - not an image, and the only evidence of why the
colour classifier decided what it decided.

Staying inside the card
------------------------
Three independent limits, because a file COUNT is not a disk budget - forty
ordinary runs are 80 MB and forty pathological ones are gigabytes:

    RECORD_MIN_FREE_MB   refuse to start, and stop mid-run, below this much free
                         space. Checked against the filesystem, because the
                         journal and everything else share the card.
    RECORD_MAX_TRIP_MB   one run's ceiling. Hitting it closes the file loudly
                         rather than filling the card.
    RECORD_MAX_TOTAL_MB  the directory's ceiling, pruned oldest-first alongside
                         the file-count limit.

And the stream is flushed every `RECORD_FLUSH_PERIOD_S`, so a boat whose battery
is pulled - which is how an interesting run usually ends - loses seconds rather
than everything.

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
import platform
import shutil
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


def _free_mb(path):
    """Free megabytes on the filesystem holding `path`, or None.

    `shutil.disk_usage` rather than `os.statvfs`, which exists only on POSIX.
    The recorder itself only ever runs on the Pi, but `tests/test_autopilot.py`
    exercises it, and the crew runs that suite on the **Windows** ground
    station minutes before a start - where the statvfs version raised
    `AttributeError` and took the recorder test down with it. Identical figure
    on Linux, and the same call `ligmax-server/ligmax_gui/trips.py` uses.
    """
    try:
        return shutil.disk_usage(path).free / (1024.0 * 1024.0)
    except OSError:
        return None


def _environment():
    """The LIGMAX_* overrides actually in force, for the header.

    `config.snapshot()` records the values the boat ran with; this records which
    of them came from the environment rather than from the defaults in the file.
    The difference matters when a recording is compared against a later build:
    a value that matches the default today may not have been a default then.
    """
    return {
        name: value
        for name, value in sorted(os.environ.items())
        if name.startswith("LIGMAX_")
        # Never the ingest secret or the update channel's key, which are in the
        # environment beside everything else and must not end up in a file that
        # gets copied off the boat and mailed around.
        and "KEY" not in name.upper()
        and "SECRET" not in name.upper()
        and "TOKEN" not in name.upper()
        and "PASS" not in name.upper()
    }


class TripRecorder:
    """One file per run. Never raises; a failed write costs a log line."""

    def __init__(self, config=config_module):
        self.config = config
        self.path = None
        self._handle = None
        self._samples = 0
        self._scans_written = 0
        self._clusters_written = 0
        self._events = 0
        self._last_sample = 0.0
        self._last_scan = 0.0
        self._last_cluster = 0.0
        self._last_flush = 0.0
        self._last_telemetry_t = None
        self._started = None
        self._bytes = 0
        self._disk_bytes = 0
        self._stopped_for_space = False
        self.last_error = None

    # ------------------------------------------------------------------ state

    @property
    def recording(self):
        return self._handle is not None

    # ------------------------------------------------------------------ start

    def start(self, label="trip", extra=None, **fields):
        """Begin a recording. Returns the path, or None if it could not open.

        Extra header fields may be passed either as `extra={...}` or as keyword
        arguments. Both, because `main.py` has always called this as
        `start(label, plan=...)` while the signature only accepted `extra` - so
        every press of "Engage autonomy" raised `TypeError` inside the tick,
        which the loop caught, logged and turned into an immediate disengage.
        The boat refused to start and the journal blamed the tick.

        Rolls the previous one first, so calling this twice can never interleave
        two runs into one file - which would be the one thing that makes a
        recording useless.
        """
        if not self.config.RECORD_ENABLED:
            return None
        self.stop("superseded")

        # Make room before deciding there is none: the prune is what frees the
        # space this check is about to test for, so running it the other way
        # round refuses a recording the boat could easily have made.
        try:
            os.makedirs(self.config.RECORD_DIR, exist_ok=True)
        except OSError as exc:
            self.last_error = str(exc)[:160]
            log.warning("could not create %s: %s", self.config.RECORD_DIR, exc)
            return None
        self._prune()

        free = _free_mb(self.config.RECORD_DIR)
        if free is not None and free < self.config.RECORD_MIN_FREE_MB:
            self.last_error = f"only {free:.0f} MB free"
            # Loud, because the operator's assumption is that pressing start
            # records the run, and a silent failure here is discovered when
            # somebody goes looking for the file that would have explained it.
            log.error(
                "NOT recording: only %.0f MB free on the card, below the %.0f MB "
                "floor. The run will go ahead unrecorded - clear space in %s.",
                free,
                self.config.RECORD_MIN_FREE_MB,
                self.config.RECORD_DIR,
            )
            return None

        stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime())
        safe = "".join(c if c.isalnum() or c in "-_" else "-" for c in str(label))[:40]
        path = os.path.join(self.config.RECORD_DIR, f"{stamp}-{safe}.jsonl.gz")
        try:
            handle = gzip.open(path, "wt", encoding="utf-8", compresslevel=6)
        except OSError as exc:
            self.last_error = str(exc)[:160]
            log.warning("could not start a trip recording at %s: %s", path, exc)
            return None

        self._handle = handle
        self.path = path
        self._samples = 0
        self._scans_written = 0
        self._clusters_written = 0
        self._events = 0
        self._bytes = 0
        self._disk_bytes = 0
        self._stopped_for_space = False
        self._started = time.time()
        self._last_flush = self._started
        self._write(
            {
                "kind": "header",
                "t": self._started,
                "label": label,
                "local_time": time.strftime("%Y-%m-%d %H:%M:%S"),
                "git": _git_head(),
                "config": self.config.snapshot(),
                "env": _environment(),
                "masks": masks.describe(),
                "host": platform.node(),
                "python": platform.python_version(),
                "uname": " ".join(platform.uname()),
                "free_mb": round(free, 1) if free is not None else None,
                # Said explicitly in the file rather than only in this module's
                # docstring, so anyone handed a recording knows what is and is
                # not in it without reading the source that produced it.
                "records_images": False,
                **(extra or {}),
                **fields,
            }
        )
        log.warning(
            "recording this run to %s (%.0f MB free)", path, free if free else -1
        )
        return path

    # ----------------------------------------------------------------- sample

    def sample(self, state, world, pilot, intent, scans=None, now=None,
               clusters=None, edge=None, commander=None, perf=None, survey=None):
        """One tick. Cheap when it is not due; never blocks the loop for long."""
        if self._handle is None:
            return
        now = now if now is not None else time.time()
        # The epsilon matters more than it looks. The tick runs at exactly
        # RECORD_HZ, so `now - last` lands on the period every time and floating
        # point rounds it under about half the time - which recorded a 10 Hz loop
        # at 6 Hz, silently, and made every rate in the file wrong. Anything
        # smaller than a tick and larger than the rounding error works.
        if now - self._last_sample < (1.0 / max(0.1, self.config.RECORD_HZ)) - 1e-3:
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
                # Every remaining BoatState field. They were being dropped, and
                # each one is a distinct explanation for a boat that would not
                # move: `satellites` for a fix that looks fine and is not,
                # `rc_link`/`operator_link` for which of the three control links
                # went away, `velocity` for the CPA the COLREG behaviour used.
                "satellites": state.satellites,
                "velocity": state.velocity,
                "rc_link": state.rc_link,
                "operator_link": state.operator_link,
                "mode": state.mode,
                "armed": state.armed,
                "estop": state.estop,
                "status": state.status,
                "usable": state.usable,
                "why_unusable": state.why_unusable,
                "frame_age_s": round(state.age(now), 3),
            }
            if state.origin:
                row["origin"] = state.origin

            # The whole io_manager telemetry snapshot - battery, BMS, RTK, trim,
            # tuning, lights, propulsion, safety. It arrives on the node bus at
            # 1 Hz (`autopilot_bridge.publish_state`), and is written whenever a
            # newer one has turned up rather than on our own timer, so the file
            # gets each snapshot exactly once.
            snapshot = getattr(state, "telemetry", None)
            if (
                self.config.RECORD_BOAT_TELEMETRY
                and snapshot
                and snapshot.get("t") != self._last_telemetry_t
            ):
                self._last_telemetry_t = snapshot.get("t")
                row["boat_telemetry"] = snapshot

        if pilot is not None:
            row["autopilot"] = pilot.telemetry(state, world)
            # The behaviour's own working, separately from the summary the
            # dashboard sees: which phase docking is in, what the cardinal vote
            # was, which marks the lateral rule shifted the line for.
            behaviour = getattr(pilot, "behaviour", None)
            if behaviour is not None:
                row["behaviour"] = {
                    "name": behaviour.name,
                    "done": behaviour.done,
                    "phase": getattr(behaviour, "phase", None),
                    "elapsed_s": (
                        round(now - behaviour.started_at, 2)
                        if behaviour.started_at else None
                    ),
                    "status": behaviour.status,
                }
        if intent is not None:
            row["intent"] = intent.telemetry()
        if commander is not None:
            # What actually went on the MAVLink wire, as opposed to what the
            # behaviour asked for. The two differ - clamps, the lateral-thruster
            # mode, the re-send suppression - and "it commanded the right thing
            # and the boat did not do it" is only separable from "it commanded
            # the wrong thing" if both are written down.
            row["control"] = commander.telemetry()
        if world is not None:
            # EVERY track, not `telemetry`'s confirmed-only top 30. The tracks
            # that never reached `confirmed()` are the ones that explain a buoy
            # the boat drove past without seeing.
            row["tracks"] = world.debug_tracks(now)
            row["world"] = world.stats(now)
        if survey is not None:
            row["survey"] = survey
        if edge is not None:
            # The Jetson link's own health: sweeps in, drops, measured Hz, skew.
            row["edge"] = edge
        if perf is not None:
            # Tick duration and jitter. A planner that fell behind its own 10 Hz
            # is a different fault from one that decided the wrong thing, and
            # they look identical afterwards without this.
            row["perf"] = perf

        # The point clouds, decimated. See the module docstring: at full rate
        # they are 95 % of the file and they are the part you least often need
        # frame by frame.
        if scans and now - self._last_scan >= (
            1.0 / max(0.1, self.config.RECORD_SCAN_HZ)
        ) - 1e-3:
            self._last_scan = now
            self._scans_written += 1
            row["scans"] = scans

        # The clusters, with the sentence saying how each one classified. Paired
        # with the cloud above deliberately: "why is that cluster RED" is
        # answered by the returns it was built from, and a file with one and not
        # the other cannot answer it.
        if clusters and now - self._last_cluster >= (
            1.0 / max(0.1, self.config.RECORD_CLUSTER_HZ)
        ) - 1e-3:
            self._last_cluster = now
            self._clusters_written += 1
            row["clusters"] = clusters

        self._write(row)
        self._samples += 1
        self._housekeep(now)

    def event(self, kind, **fields):
        """A one-off worth finding later: a command, a phase change, a fault.

        These are what a reviewer scrubs to. A 15 minute run is 9000 samples and
        perhaps twenty events, and the events are where the story is.
        """
        if self._handle is None:
            return
        self._write({"kind": "event", "t": round(time.time(), 3), "event": kind, **fields})
        self._events += 1

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
                    "clusters": self._clusters_written,
                    "events": self._events,
                    "bytes_written": self._bytes,
                    "file_bytes": self._disk_bytes,
                    # Set when a size or free-space limit cut the recording
                    # short, so a reviewer reading a file that stops mid-run
                    # knows it was truncated on purpose rather than by a crash.
                    "truncated": self._stopped_for_space,
                    "free_mb": (
                        round(f, 1)
                        if (f := _free_mb(self.config.RECORD_DIR)) is not None
                        else None
                    ),
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
            blob = json.dumps(row, separators=(",", ":"), default=_plain)
            self._handle.write(blob)
            self._handle.write("\n")
            # Uncompressed bytes written, for the footer only. **Not** what the
            # size cap is measured against: gzip gets these frames down by about
            # ten to one, so capping on this number ends a perfectly ordinary
            # 15 minute attempt after eight minutes - which is exactly what it
            # did the first time it was tried. The cap uses the real file size,
            # in `_housekeep`.
            self._bytes += len(blob) + 1
        except (OSError, ValueError, TypeError) as exc:
            # One bad row must not end the recording, and a full card must not
            # take the boat down. Note it and carry on.
            self.last_error = str(exc)[:160]

    def _housekeep(self, now):
        """Flush periodically, and stop before the card is a problem.

        Both limits stop the *recording*, never the boat. A run that cannot be
        written down is a run with no post-mortem; a root filesystem with no free
        space is a boat that cannot log, cannot self-update and probably cannot
        finish the day, so the recording is what gives way.
        """
        if now - self._last_flush >= self.config.RECORD_FLUSH_PERIOD_S:
            self._last_flush = now
            try:
                # gzip's flush is a Z_SYNC_FLUSH, so the file is a valid gzip
                # stream at this instant and a power cut costs the last few
                # seconds instead of everything since `start()`.
                self._handle.flush()
            except (OSError, ValueError) as exc:
                self.last_error = str(exc)[:160]

            free = _free_mb(self.config.RECORD_DIR)
            if free is not None and free < self.config.RECORD_MIN_FREE_MB:
                self._stopped_for_space = True
                log.error(
                    "stopping this recording: only %.0f MB free, below the "
                    "%.0f MB floor. The run continues, unrecorded.",
                    free,
                    self.config.RECORD_MIN_FREE_MB,
                )
                self.stop("card nearly full")
                return

            # The file's real size, checked here rather than on every row: it is
            # a stat() call, and it is only meaningful just after a flush anyway.
            try:
                self._disk_bytes = os.path.getsize(self.path)
            except OSError:
                self._disk_bytes = 0
            if self._disk_bytes >= self.config.RECORD_MAX_TRIP_MB * 1024 * 1024:
                self._stopped_for_space = True
                log.error(
                    "stopping this recording: the file has reached its %.0f MB "
                    "ceiling. The run continues, unrecorded.",
                    self.config.RECORD_MAX_TRIP_MB,
                )
                self.stop("trip size ceiling reached")

    def _prune(self):
        """Keep the newest trips, under both the count and the byte ceiling.

        Sorted by name, which is chronological: every file starts with a
        fixed-width `%Y%m%d-%H%M%S` stamp, so lexicographic order is time order
        and stays that way (it survives the year rolling over, which a
        `%d-%m-%Y` stamp would not).

        The byte ceiling is the one that actually protects the card. Forty
        ordinary runs are 80 MB; forty runs in rain, with the lidar spraying
        returns and ten times the cluster count, are gigabytes - and only the
        second kind is a problem, so a count limit alone never fires when it
        matters.
        """
        try:
            names = sorted(
                f for f in os.listdir(self.config.RECORD_DIR)
                if f.endswith(".jsonl.gz")
            )
        except OSError:
            return

        sizes = {}
        for name in names:
            try:
                sizes[name] = os.path.getsize(
                    os.path.join(self.config.RECORD_DIR, name)
                )
            except OSError:
                sizes[name] = 0

        doomed = list(names[: max(0, len(names) - self.config.RECORD_KEEP_TRIPS)])
        budget = self.config.RECORD_MAX_TOTAL_MB * 1024 * 1024
        total = sum(sizes.values())
        for name in names:
            if total <= budget:
                break
            if name in doomed:
                continue
            doomed.append(name)
            total -= sizes.get(name, 0)

        for name in doomed:
            try:
                os.remove(os.path.join(self.config.RECORD_DIR, name))
                log.info(
                    "pruned old trip recording %s (%.1f MB)",
                    name,
                    sizes.get(name, 0) / 1048576.0,
                )
            except OSError:
                pass

    # ------------------------------------------------------------- telemetry

    def telemetry(self):
        block = {
            "recording": self.recording,
            "samples": self._samples,
            "scans": self._scans_written,
            "clusters": self._clusters_written,
            "events": self._events,
            "mb": round(self._disk_bytes / 1048576.0, 2),
            "uncompressed_mb": round(self._bytes / 1048576.0, 2),
        }
        if self.path:
            block["file"] = os.path.basename(self.path)
        if self._started and self.recording:
            block["duration_s"] = round(time.time() - self._started, 1)
        free = _free_mb(self.config.RECORD_DIR)
        if free is not None:
            # On the operator's panel, because "the card filled up" is a thing
            # they can act on from the dock and cannot see any other way.
            block["free_mb"] = round(free, 0)
        if self._stopped_for_space:
            block["truncated"] = True
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
