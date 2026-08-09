"""The surveyed map: the marks the boat found, kept between runs.

    survey = Survey(config)
    for entry in survey.entries(): ...          # what attempt one found
    survey.write(entries, origin)               # what attempt two should start with
    survey.clear()                              # the operator's "forget it all"

Why this file exists
--------------------
NJORD gives two attempts at each subtask (§8.2). Between them the boat comes off
the water, someone changes a threshold, the node restarts - and everything it
learned about where the marks are is gone. That is the single most expensive
thing the autonomy stack forgets, because the marks have not moved: the buoys,
the dock and the shore are in exactly the same places for attempt two as they
were for attempt one. Only the Otter has moved, and `perception/world.py` refuses
to establish a vessel for precisely that reason.

So the established static tracks are written here, and read back on the next
start. Attempt two begins knowing roughly where every gate is, rather than
discovering it again at 2 knots.

Degrees, not metres, and this is not a detail
---------------------------------------------
Entries are lat/lon. The obvious alternative - grid metres, the frame everything
else in the autonomy stack uses - is **wrong**, and quietly so:

`io_manager/navigation.py` captures the grid origin from the first usable fix and
caches it at `/run/ligmax/grid-origin.json`. `/run` is **tmpfs**. Every reboot
empties it, and the next fix re-zeroes the grid somewhere else. The dashboard's
`recentre_origin` button does the same thing deliberately. A survey in metres
would therefore survive the night looking perfectly valid and describe a course
several metres from the real one - and it would be believed, because nothing
about the file would look wrong.

Degrees are absolute. `world.py` converts on the way in and on the way out, and a
moved origin costs nothing but the conversion.

Failure is not fatal
--------------------
Nothing here raises. A survey that cannot be read is no survey and the boat
starts blind, which is exactly where it was before this file existed; a survey
that cannot be written costs the next attempt its head start. Neither is worth
taking a run down for, so every path is caught and logged.
"""

from __future__ import annotations

import json
import logging
import os
import time

log = logging.getLogger("self_driving.survey")

#: Bumped when the entry shape changes in a way an older file cannot satisfy.
#: A file from a different version is discarded rather than guessed at.
FORMAT = 2


class Survey:
    """The stored map, on disk. One file, rewritten atomically."""

    def __init__(self, config):
        self.config = config
        self.path = config.SURVEY_FILE
        self.last_error = None
        self.wrote_at = None
        self.entries_written = 0

    # ------------------------------------------------------------------ read

    def entries(self):
        """The stored marks, as plain dicts. `[]` if there is nothing usable.

        Four ways a file is rejected, all of them silent-but-logged rather than
        raised, and the age one is the one that matters operationally: a survey
        from last week's practice describes marks that have since been lifted and
        re-laid, and steering round those confidently is worse than not
        remembering at all.
        """
        if not self.config.SURVEY_ENABLED:
            return []
        try:
            with open(self.path, "r", encoding="utf-8") as handle:
                stored = json.load(handle)
        except FileNotFoundError:
            return []
        except (OSError, ValueError) as exc:
            self.last_error = str(exc)[:160]
            log.warning("ignoring the stored survey %s: %s", self.path, exc)
            return []

        if not isinstance(stored, dict):
            return []
        if int(stored.get("format") or 0) != FORMAT:
            log.warning(
                "ignoring the stored survey %s: format %s, this build wants %d",
                self.path,
                stored.get("format"),
                FORMAT,
            )
            return []

        age = time.time() - float(stored.get("t") or 0.0)
        if age > self.config.SURVEY_MAX_AGE_S:
            log.warning(
                "ignoring the stored survey %s: it is %.1f h old, which is past "
                "the %.1f h limit - the marks have very likely been re-laid",
                self.path,
                age / 3600.0,
                self.config.SURVEY_MAX_AGE_S / 3600.0,
            )
            return []

        entries = stored.get("marks")
        if not isinstance(entries, list):
            return []
        good = [entry for entry in entries if isinstance(entry, dict)]
        return good[: self.config.SURVEY_MAX_TRACKS]

    # ----------------------------------------------------------------- write

    def write(self, entries, origin=None):
        """Replace the file with `entries`. Best effort; never raises.

        Atomic, the same way `plan.py` writes the plan: a temporary file and
        `os.replace`. The boat loses power without warning often enough - it is a
        battery with a relay in front of it - that a half-written map is a real
        outcome rather than a theoretical one, and a half-written map is worse
        than none because it parses.
        """
        if not self.config.SURVEY_ENABLED:
            return False
        entries = list(entries or [])[: self.config.SURVEY_MAX_TRACKS]
        payload = {
            "format": FORMAT,
            "t": time.time(),
            "local_time": time.strftime("%Y-%m-%d %H:%M:%S"),
            # Kept for the reader's benefit only - the marks themselves are
            # absolute, so this is a note about which grid they were measured
            # against, not something the conversion needs.
            "origin": origin,
            "marks": entries,
        }
        try:
            os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
            temporary = f"{self.path}.tmp"
            with open(temporary, "w", encoding="utf-8") as handle:
                json.dump(payload, handle, indent=1)
            os.replace(temporary, self.path)
        except (OSError, TypeError, ValueError) as exc:
            self.last_error = str(exc)[:160]
            log.warning("could not write the survey to %s: %s", self.path, exc)
            return False
        self.wrote_at = time.time()
        self.entries_written = len(entries)
        return True

    def clear(self):
        """Delete the stored survey. Part of the operator's "clear everything"."""
        try:
            os.remove(self.path)
            log.warning("stored survey %s deleted", self.path)
        except FileNotFoundError:
            pass
        except OSError as exc:
            self.last_error = str(exc)[:160]
            log.warning("could not delete the survey %s: %s", self.path, exc)
            return False
        self.wrote_at = None
        self.entries_written = 0
        return True

    # ------------------------------------------------------------- telemetry

    def telemetry(self):
        block = {
            "enabled": bool(self.config.SURVEY_ENABLED),
            "file": os.path.basename(self.path),
            "marks": self.entries_written,
        }
        if self.wrote_at is not None:
            block["age_s"] = round(time.time() - self.wrote_at, 1)
        if self.last_error:
            block["last_error"] = self.last_error
        return block
