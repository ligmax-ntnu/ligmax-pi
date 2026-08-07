"""Upload an admin-laid waypoint mission to the autopilot, and clear it.

Where this fits: the dashboard now lets an admin lay a route on the map (grid
metres, the same frame `goto` already uses) and send it as the `set_mission`
operator command. This module turns that into a real MAVLink mission on the
Pixhawk, so ArduPilot's own AUTO mode - not any planner here - drives the
route. That is deliberate and matches docs/architecture.md's "one idea": the
Pi is the autonomy stack, the Pixhawk is control, and MAVLink is the seam.
Running the mission is then just `set_mode` to AUTO (or GUIDED for the
single-point `goto`) plus `arm`, both implemented alongside this in
`main.py` - none of it needs the self_driving planner, which does not exist
yet.

The MAVLink mission protocol, briefly
-------------------------------------
Uploading is a three-way handshake, not one message:

    GCS  -> FC   MISSION_COUNT (how many items are coming)
    FC   -> GCS  MISSION_REQUEST_INT, once per item, in whatever order it likes
    GCS  -> FC   MISSION_ITEM_INT, one per request
    FC   -> GCS  MISSION_ACK (accepted, or a MAV_MISSION_RESULT reason)

Clearing is one message each way: MISSION_CLEAR_ALL, then MISSION_ACK.

Both are driven from `handle()`, fed one message at a time from the same
pump that runs `Navigation.handle()` and `Trim.handle()` in `main.py` - so an
upload never blocks the loop that owes the autopilot its 1 Hz heartbeat
(test.py's heartbeat warning, referenced throughout this repo). `check_timeout()`
is the escape hatch for an autopilot that never answers a request or never
sends the final ack.

Only one operation - an upload or a clear - runs at a time. A second one
mid-exchange is refused by the caller (`main.py:handle_commands`) rather than
queued, since two interleaved mission exchanges on one MAVLink link would be
indistinguishable from a corrupted one.

What this deliberately does not do: decide when to run the mission. That is
`set_mode` to AUTO, a separate operator action, so laying a route and setting
it running are always two distinct, auditable commands.
"""

from __future__ import annotations

import logging
import math
import time

from pymavlink import mavutil

log = logging.getLogger("io_manager.mission")

MAX_WAYPOINTS = 100

# No MISSION_REQUEST_INT, or no MISSION_ACK, inside this many seconds fails the
# exchange. This link is a local USB serial cable, not the 4G path to shore, so
# even a large mission should finish in well under a second - this is generous
# because the failure mode (an operator's command stuck at "sent" forever) is
# worse than waiting a little longer to be sure.
EXCHANGE_TIMEOUT_S = 8.0

MAV_MISSION_ACCEPTED = 0  # MAV_MISSION_RESULT


def parse_waypoints(raw, limit=MAX_WAYPOINTS):
    """The operator's `points` arg -> a list of `(x, y)` float pairs, or None.

    None means "reject the command" - distinguished from `[]`, which would
    mean "a mission of zero waypoints", a command `set_mission` also refuses
    but for a different reason (nothing to upload).
    """
    if not isinstance(raw, (list, tuple)) or not raw:
        return None
    points = []
    for item in raw[:limit]:
        try:
            x, y = float(item[0]), float(item[1])
        except (TypeError, ValueError, IndexError, KeyError):
            return None
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        points.append((x, y))
    return points


def _mission_result_name(value):
    """A MAV_MISSION_RESULT number as a human name, or the bare number."""
    try:
        return mavutil.mavlink.enums["MAV_MISSION_RESULT"][value].name
    except (KeyError, AttributeError, TypeError):
        return f"result {value}"


class MissionUploader:
    """Drives one MAVLink mission upload or clear at a time, from the main pump.

    Nothing here owns a thread or a socket - `master` is passed in on every
    call, exactly like the rest of the MAVLink pump in `main.py`, so this
    stays testable without a boat and cannot race the heartbeat sender for
    the same connection object.
    """

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._pending = None  # dict - see `upload`/`clear`
        self._outcome = None  # (command_id, ok, message, kind, grid_points)

    @property
    def busy(self):
        return self._pending is not None

    # -- starting an exchange ------------------------------------------------

    def upload(self, master, command_id, grid_points, global_points):
        """Start uploading a mission. `global_points` is `[(lat, lon), ...]`.

        `grid_points` is kept only so a successful upload can be echoed back
        to the dashboard as the "ideal route" reference layer
        (`main.py:finish_mission`) - it plays no part in the wire protocol.
        """
        self._pending = {
            "kind": "upload",
            "command_id": command_id,
            "items": list(global_points),
            "grid_points": list(grid_points),
            "requested": set(),
            "started": self._clock(),
        }
        master.mav.mission_count_send(
            master.target_system, master.target_component, len(global_points)
        )
        log.info(
            "mission upload started: %d waypoint(s), command %s",
            len(global_points),
            command_id,
        )

    def clear(self, master, command_id):
        """Start a MISSION_CLEAR_ALL and wait for its MISSION_ACK."""
        self._pending = {
            "kind": "clear",
            "command_id": command_id,
            "items": [],
            "grid_points": [],
            "requested": set(),
            "started": self._clock(),
        }
        master.mav.mission_clear_all_send(master.target_system, master.target_component)
        log.info("mission clear requested, command %s", command_id)

    # -- fed by the MAVLink pump ---------------------------------------------

    def handle(self, master, message):
        """Absorb one MAVLink message. Returns True if it was one of ours.

        Silently keeps absorbing traffic for an exchange that has already
        been decided (a late, stray MISSION_REQUEST_INT after a timeout, say)
        rather than answering it - `take()` has not been called yet, so the
        pending record is still here to check against.
        """
        if self._pending is None:
            return False
        kind = message.get_type()
        if kind not in ("MISSION_REQUEST_INT", "MISSION_REQUEST", "MISSION_ACK"):
            return False
        if self._outcome is not None:
            return True  # already decided; take() just has not run yet

        if kind in ("MISSION_REQUEST_INT", "MISSION_REQUEST"):
            self._answer_request(master, message)
            return True

        # MISSION_ACK
        self._finish(getattr(message, "type", None))
        return True

    def _answer_request(self, master, message):
        if self._pending["kind"] != "upload":
            return  # a clear has no items to send; a stray request is ignored
        seq = getattr(message, "seq", None)
        items = self._pending["items"]
        if seq is None or not (0 <= seq < len(items)):
            return  # out of range; the eventual MISSION_ACK will settle it
        lat, lon = items[seq]
        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            seq,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,  # current: 0, this is a plain upload, not "jump to here now"
            1,  # autocontinue: proceed to the next waypoint unattended
            0.0,  # param1: hold time at the waypoint, seconds
            0.0,  # param2: acceptance radius, 0 = the vehicle's WP_RADIUS default
            0.0,  # param3: pass radius, 0 = fly straight through
            0.0,  # param4: desired yaw, unused for a rover
            int(round(lat * 1e7)),
            int(round(lon * 1e7)),
            0.0,  # z: relative altitude - unused for a surface vessel
        )
        self._pending["requested"].add(seq)

    def check_timeout(self):
        """Fail a stuck exchange. Call once per loop tick; cheap when idle."""
        if self._pending is None or self._outcome is not None:
            return
        if self._clock() - self._pending["started"] > EXCHANGE_TIMEOUT_S:
            log.warning(
                "mission %s timed out waiting for the autopilot", self._pending["kind"]
            )
            self._finish(None, timed_out=True)

    def link_down(self):
        """The MAVLink link dropped mid-exchange. Fail it now, not silently."""
        if self._pending is not None and self._outcome is None:
            self._finish(None, timed_out=True, reason="the MAVLink link dropped")

    def take(self):
        """The finished exchange's outcome, once. `None` while still running.

        Returns `(command_id, ok, message, kind, grid_points)` - `grid_points`
        is only meaningful (and only non-empty) when `kind == "upload"`.
        """
        if self._outcome is None:
            return None
        outcome, self._outcome, self._pending = self._outcome, None, None
        return outcome

    def _finish(self, mission_result, timed_out=False, reason=None):
        pending = self._pending
        if timed_out:
            ok = False
            message = reason or f"no reply from the autopilot within {EXCHANGE_TIMEOUT_S:.0f}s"
        elif mission_result == MAV_MISSION_ACCEPTED:
            if pending["kind"] == "upload":
                missing = set(range(len(pending["items"]))) - pending["requested"]
                ok = not missing
                message = (
                    "mission accepted"
                    if ok
                    else f"accepted, but the autopilot never asked for waypoint(s) {sorted(missing)}"
                )
            else:
                ok, message = True, "mission cleared"
        else:
            ok = False
            message = f"autopilot refused: {_mission_result_name(mission_result)}"

        level = logging.INFO if ok else logging.WARNING
        log.log(level, "mission %s: %s", pending["kind"], message)
        self._outcome = (
            pending["command_id"],
            ok,
            message,
            pending["kind"],
            pending["grid_points"],
        )
