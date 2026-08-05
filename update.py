"""Run main.py, and restart it when it exits or when an update lands.

Started at boot by ligmax-pi.service, so the vessel comes up on its own after a
power cycle with no SSH needed. Runs as the repo owner: it owns the child process
and restarts it itself, so no sudo and no systemctl.

There are two ways an update reaches this node, and this file is only involved in
one of them:

  * **As a vessel command** - the normal path. The dashboard queues `update` on
    the telemetry channel, `nodes/io_manager/selfupdate.py` fast-forwards and
    signals main.py's process group, and this loop simply sees its child exit and
    starts it again on the new code. Nothing here talks to the dashboard at all.

  * **By polling** - the fallback, and **off unless LIGMAX_NODE_KEY is set**. It
    is what still works when io_manager is the thing that is broken, but a key
    that does not match the server's is rejected before the poll is recorded, so
    a wrong one looks exactly like a node that is switched off. Leaving it unset
    is better than leaving it wrong.
"""

import json
import os
import signal
import subprocess
import sys
import time
import urllib.request

REPO = os.path.dirname(os.path.abspath(__file__))
NAME = "ligmax-pi"
START = [sys.executable, "main.py"]
DASH = os.environ.get("LIGMAX_DEPLOY_URL", "https://live.ligmax.no").rstrip("/")
KEY = os.environ.get("LIGMAX_NODE_KEY", "")
POLL = 30  # seconds between /pending checks, when the fallback is on
TICK = 1  # how often we look at the child, so a restart is not a poll behind
RESTART_DELAY = 5


def say(msg):
    # Goes to the journal: journalctl -u ligmax-pi -f
    print(f"{time.strftime('%H:%M:%S')} {msg}", flush=True)


def ask(path, body=None):
    req = urllib.request.Request(
        f"{DASH}/api/deploy/{NAME}{path}",
        data=json.dumps(body).encode() if body else None,
        headers={"Authorization": f"Bearer {KEY}", "Content-Type": "application/json"},
    )
    with urllib.request.urlopen(req, timeout=10) as response:
        return json.load(response)


def head():
    return subprocess.run(
        ["git", "-C", REPO, "rev-parse", "HEAD"], capture_output=True, text=True
    ).stdout.strip()


def wait_for_work(child):
    """Block until the child exits or the dashboard asks for a pull.

    Returns the nonce of a poll-driven request, or None if the child exited on
    its own - which is also what happens after io_manager pulls and signals the
    group, and is why that path needs nothing from this function.
    """
    next_poll = time.time() + POLL
    while child.poll() is None:
        time.sleep(TICK)
        if not KEY or time.time() < next_poll:
            continue
        next_poll = time.time() + POLL
        try:
            pending = ask("/pending")
            if pending.get("requested"):
                return pending.get("nonce")
        except Exception:
            pass  # dashboard unreachable is normal on a boat; keep running
    return None


def stop(child):
    os.killpg(os.getpgid(child.pid), signal.SIGTERM)
    try:
        child.wait(15)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(child.pid), signal.SIGKILL)


while True:
    # start_new_session so we can signal the whole tree, not just the parent -
    # and so io_manager can signal that same group to restart itself.
    child = subprocess.Popen(START, cwd=REPO, start_new_session=True)
    say(f"started {START[-1]} as pid {child.pid} at {head()[:8]}")

    nonce = wait_for_work(child)

    # Keyed off the request, not off whether the child is still up. Gating the
    # pull on `child.poll() is None` meant a main.py that had died during the
    # poll interval swallowed the request entirely: nothing was pulled, nothing
    # was reported, and the operator's row sat at "Waiting" for 30 minutes.
    if nonce is not None:
        if child.poll() is None:
            stop(child)

        before = head()
        pull = subprocess.run(
            ["git", "-C", REPO, "pull", "--ff-only"], capture_output=True, text=True
        )
        note = (pull.stdout + pull.stderr).strip().replace("\n", " ")
        say(f"pull: {note}")
        # Report, or /pending keeps saying "requested" and we restart in a loop.
        try:
            ok = pull.returncode == 0
            ask(
                "/report",
                {
                    "nonce": nonce,
                    "result": "ok" if ok else "failed",
                    "message": note[:300],
                    "head": head(),
                },
            )
        except Exception as exc:
            say(f"could not report: {exc}")
        if pull.returncode != 0:
            say("pull failed - restarting the old code")
        elif before == head():
            say("nothing new")
    else:
        # Either main.py crashed, or io_manager pulled and took the group down on
        # purpose. Both want the same thing: start it again on whatever is now in
        # the working tree.
        say(f"{START[-1]} exited with {child.returncode}; restarting in {RESTART_DELAY}s")
        time.sleep(RESTART_DELAY)
