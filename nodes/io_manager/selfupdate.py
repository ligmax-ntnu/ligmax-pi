"""Fast-forward this checkout when the operator presses Update, and restart.

The dashboard's Software panel sends `update` down the same channel as `estop`
(the reply to a telemetry POST), so this needs no second secret and no second
connection - which is the whole point. The old path, a poller in `update.py`
asking `/api/deploy/ligmax-pi/pending` with `LIGMAX_NODE_KEY`, is still there as
a fallback but stays off unless that key is set: a wrong key is rejected before
the poll is recorded, so it looked exactly like a node that was switched off.

Two things here are not negotiable.

**The pull runs on a worker thread.** `handle_commands()` is called from the
MAVLink loop that also sends the 1 Hz GCS heartbeat, and `test.py:48` is explicit
that losing that heartbeat can make the autopilot failsafe. A `git pull` over 5G
can easily take longer than the timeout the autopilot allows, so it must not sit
in that loop. `start()` returns immediately; `take()` collects the outcome a tick
or many ticks later.

**`--ff-only`, and nothing else.** On this machine the uncommitted edit *is* the
field fix and the ignored files are the irreplaceable ones. git itself refuses to
fast-forward over local changes to tracked files, a detached HEAD, or commits that
are not on `origin/main`; the pull returns non-zero, the operator sees git's own
message in the panel, and the boat keeps running the code it has. No reset, no
checkout, no clean, no stash - see docs/deploy.md.
"""

from __future__ import annotations

import logging
import os
import pathlib
import signal
import subprocess
import threading
from dataclasses import dataclass

# <repo>/nodes/io_manager/selfupdate.py -> <repo>
REPO = str(pathlib.Path(__file__).resolve().parents[2])
NAME = "ligmax-pi"

# Generous, because a cold TLS handshake on a marginal uplink is slow - but
# finite, because a hung git would otherwise leave the operator's row at
# "Waiting" until it expired half an hour later with no explanation.
GIT_TIMEOUT = 120.0

log = logging.getLogger("io_manager.update")


@dataclass
class Outcome:
    """What to ack, once the worker has finished."""

    command_id: str
    ok: bool
    message: str
    head: str
    changed: bool


def _git(*args: str, timeout: float = 15.0) -> tuple[int, str]:
    """Run a git command in this checkout. Never raises; returns (rc, output)."""
    try:
        done = subprocess.run(
            ["git", "-C", REPO, *args],
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return 1, f"git {args[0]} timed out after {timeout:.0f}s"
    except OSError as exc:  # git missing, or the checkout is gone
        return 1, f"could not run git: {exc}"
    return done.returncode, (done.stdout + done.stderr).strip()


def head() -> str:
    rc, out = _git("rev-parse", "HEAD")
    return out if rc == 0 else ""


class SelfUpdate:
    """One `git pull --ff-only` at a time, off the control loop."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._outcome: Outcome | None = None

    def start(self, command_id: str) -> tuple[bool, str]:
        """Begin a pull. Returns (started, why-not) - never blocks."""
        with self._lock:
            if self._thread is not None:
                return False, "an update is already running"
            self._thread = threading.Thread(
                target=self._run, args=(command_id,), daemon=True, name="selfupdate"
            )
        self._thread.start()
        return True, f"pulling {NAME} in {REPO}"

    def take(self) -> Outcome | None:
        """The finished outcome, once and once only. None while it is running."""
        with self._lock:
            outcome, self._outcome = self._outcome, None
            if outcome is not None:
                self._thread = None
            return outcome

    def _run(self, command_id: str) -> None:
        before = head()
        rc, note = _git("pull", "--ff-only", timeout=GIT_TIMEOUT)
        after = head()
        message = " ".join(note.split()) or ("pull returned %d" % rc)
        if rc == 0:
            log.info("pull ok: %s", message)
        else:
            # Worth ERROR: this is the "someone made a field fix on this machine"
            # case, and the message is git's own explanation of it.
            log.error("pull failed: %s", message)
        with self._lock:
            self._outcome = Outcome(
                command_id=command_id,
                ok=rc == 0,
                message=message[:300],
                head=after or before,
                changed=bool(after) and after != before,
            )


def request_restart() -> bool:
    """Ask the supervisor to restart the whole node tree onto the new code.

    `update.py` starts `main.py` with `start_new_session=True`, so main.py leads
    its own process group and every node sits in it. Signalling the group is what
    `update.py` already does for a poll-driven update, and it is what avoids
    orphans: main.py has no SIGTERM handler, so killing it alone would leave its
    children running - a second io_manager driving the same E-stop GPIO.

    Only fires under the supervisor. Run `python -m nodes.io_manager.main` by hand
    and this returns False rather than signalling a process group we do not own.
    """
    if os.environ.get("LIGMAX_SUPERVISED") != "1":
        log.warning(
            "update pulled, but this node is not running under main.py/update.py - "
            "restart it by hand to pick up the new code"
        )
        return False
    log.warning("restarting the node tree to run %s", head()[:8] or "the new code")
    os.killpg(os.getpgid(0), signal.SIGTERM)
    return True
