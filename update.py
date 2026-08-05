"""Run main.py, and pull + restart it when the dashboard's Update button is pressed.

Started at boot by ligmax-pi.service, so the vessel comes up on its own after a
power cycle with no SSH needed. Runs as the repo owner: it owns the child process
and restarts it itself, so no sudo and no systemctl.
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
POLL = 30


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


while True:
    # start_new_session so we can signal the whole tree, not just the parent
    child = subprocess.Popen(START, cwd=REPO, start_new_session=True)
    say(f"started {START[-1]} as pid {child.pid} at {head()[:8]}")

    nonce = None
    while child.poll() is None:
        time.sleep(POLL)
        try:
            pending = ask("/pending")
            if pending.get("requested"):
                nonce = pending.get("nonce")
                break
        except Exception:
            pass  # dashboard unreachable is normal on a boat; keep running

    if child.poll() is None:
        os.killpg(os.getpgid(child.pid), signal.SIGTERM)
        try:
            child.wait(15)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(child.pid), signal.SIGKILL)

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
        say(f"{START[-1]} exited with {child.returncode}; restarting in 5s")
        time.sleep(5)
