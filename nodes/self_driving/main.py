"""The autonomy node. Started by the supervisor as `python -m nodes.self_driving.main`.

What it owns
------------
    TCP 3401        the Jetson's feed - the coloured front lidar and the YOLO
                    detections - taken **directly**, not relayed through
                    io_manager. This node is the only consumer that needs the
                    full 10 Hz cloud, and a relay would serialise ~10 kB ten
                    times a second for nothing. io_manager still gets a copy for
                    the operator's chart, pushed back over the node bus.
    the world model tracks, in world metres, built from both lidars
    the plan        waypoints and their roles, persisted across restarts
    the recording   one gzipped file per run

What it deliberately does not own
---------------------------------
The MAVLink link, the shore uplink and the E-stop GPIO. All three stay with
io_manager, because only one process can hold `/dev/ttyACM0` and because the
1 Hz heartbeat must not be queued behind this node's thinking. Control goes out
as requests on the node bus and io_manager puts them on the wire (`link.py`,
`commander.py`).

Safe by default
---------------
**This node commands nothing until an operator engages it.** It runs from boot,
observing: it builds the world model, feeds the chart, records, and publishes
what it *would* do. `autopilot_start` is the only thing that lets it drive.
That is what makes it safe to leave in the supervisor's always-on list, which in
turn is what means it is already warmed up - lidar tracked, plan loaded - when
somebody does press start.

The tick
--------
10 Hz, matched to the lidar's rotation rate so every tick gets exactly one new
sweep and no tick does the same work twice. In order:

    1. drain the node bus (newest state, any operator commands)
    2. take the newest front sweep and the aft one, mask out the boat's own hull
    3. cluster, classify, fold into the world model, refine with the detections
    4. run the pilot, which decides whether it may drive at all before it
       decides anything else
    5. send the intent, publish telemetry, relay the front scan, record

Nothing in that list blocks. The edge link runs its own thread and this only
ever reads its newest answer, exactly as io_manager treats the BMS and the aft
lidar.
"""

from __future__ import annotations

import logging
import signal
import time
import traceback

import numpy as np

from nodes.io_manager.edge_link import EdgeLink
from nodes.io_manager.scan import front_scan

from . import config
from . import paths
from .commander import Commander
from .link import NodeLink
from .perception import cluster_sweep, masks
from .perception.world import WorldModel
from .pilot import Pilot
from .recorder import TripRecorder
from .survey import Survey

log = logging.getLogger("self_driving")

# How often to publish telemetry up to the dashboard. The tick is 10 Hz; the
# operator's panel does not need decisions ten times a second, and every frame
# costs 4G.
TELEMETRY_PERIOD = 0.5

# The front cloud, relayed for the operator's chart. Matches io_manager's own
# scan tick, so the plot rate is unchanged by autonomy being on.
SCAN_RELAY_PERIOD = 0.1

# One line in the journal saying what the boat can see. Rare enough not to bury
# anything, frequent enough that `journalctl -f` on a dock answers "is it
# working?" without the dashboard, which is exactly the situation you are in
# when the dashboard is the thing that is not working.
HEARTBEAT_LOG_PERIOD = 10.0

# Re-send the chart's path layers this often even when nothing about them has
# changed. Only a backstop: they are sent the moment they change. What it
# recovers from is another publisher replacing the list - `io_manager`'s mission
# echo does exactly that - which would otherwise leave the wrong lines on the
# chart until the plan next moved.
PATHS_KEEPALIVE = 5.0

_STOPPING = False


def _handle_signal(_signum, _frame):
    global _STOPPING
    _STOPPING = True


class Node:
    """Everything wired together. One instance, one process."""

    def __init__(self):
        self.link = NodeLink()
        self.commander = Commander(self.link, config)
        self.survey = Survey(config)
        self.world = WorldModel(config, survey=self.survey)
        self.pilot = Pilot(config, self.commander)
        self.recorder = TripRecorder(config)
        self.edge = EdgeLink(port=config.EDGE_PORT) if config.OWN_EDGE_LINK else None
        self._front_seq = None
        self._front_scan = None
        self._front_clusters = []
        self._aft_clusters = []
        self._last_telemetry = 0.0
        self._last_relay = 0.0
        # What the chart's path layers looked like when they were last sent, so
        # an unchanged plan is not re-sent at 2 Hz. Cleared to force a resend.
        self._paths_sig = None
        self._paths_at = 0.0
        self._last_heartbeat_log = 0.0
        self.ticks = 0
        # Loop health, recorded every sample and published on the panel. A
        # planner that fell behind its own 10 Hz and one that decided the wrong
        # thing look identical in a recording unless the timing is in it.
        self._last_tick_s = 0.0
        self._overruns = 0
        self._tick_errors = 0

    # ------------------------------------------------------------------ setup

    def start(self):
        if self.edge is not None:
            self.edge.start()
            log.info("listening for the Jetson on port %s", config.EDGE_PORT)
        else:
            log.warning(
                "LIGMAX_AP_OWN_EDGE_LINK=0 - no front lidar and no detections, so "
                "the boat can only see astern"
            )
        if self.pilot.plan is not None:
            log.info(
                "plan %r restored: %d waypoint(s), next is %s",
                self.pilot.plan.name,
                len(self.pilot.plan),
                self.pilot.plan.current.name if self.pilot.plan.current else "END",
            )
        else:
            log.info("no stored plan - waiting for the operator to send one")

    def close(self):
        self.recorder.stop("node shutting down")
        # Write the map out before going away. The node is normally stopped
        # between the two attempts - by the dashboard's update button, or by
        # somebody power-cycling the boat on the dock - which is exactly the
        # moment the survey is worth the most and the only moment it can be lost.
        self.survey_save("node shutting down")
        self.commander.disengage("node shutting down")
        if self.edge is not None:
            self.edge.close()
        self.link.close()

    def survey_save(self, why):
        """Persist the surveyed marks now, and say so. Never raises."""
        try:
            self.world.save_survey()
        except Exception as exc:  # noqa: BLE001 - losing the map is not fatal
            log.warning("could not save the survey (%s): %s", why, exc)
            return
        log.info("survey saved (%s): %s", why, self.survey.telemetry())

    # ------------------------------------------------------------------- tick

    def tick(self, now):
        started = time.monotonic()
        self.link.poll()
        state = self.link.latest_state()

        self._commands(state)
        scans = self._perceive(state, now)
        # The sweeps go through as well as the clusters: the parking behaviours fit
        # straight edges to the raw returns, which is a different question from the
        # one clustering answers (`perception/lines.py`). Both lidars are offered
        # and the behaviour decides which of them it trusts.
        # The AR tags, straight from the Jetson and NOT through the world model.
        # With both lidars down these are the docking task's only sensor, and they
        # are aged out hard (1 s) inside `EdgeLink.tags` because a tag is a claim
        # about where a pontoon was, and the boat parks tens of centimetres from it.
        tags = self.edge.tags() if self.edge is not None else ()
        intent = self.pilot.tick(
            state, self.world, self._front_clusters, now, sweeps=scans, tags=tags
        )
        if state is not None:
            self.commander.send(intent, state, now)

        self.recorder.sample(
            state,
            self.world,
            self.pilot,
            intent,
            scans,
            now,
            clusters=self._cluster_debug(),
            edge=(self.edge.telemetry() if self.edge is not None else None),
            commander=self.commander,
            perf=self._perf(started),
            survey=self.survey.telemetry(),
        )
        self._publish(state, scans, now, intent)
        self.ticks += 1
        self._last_tick_s = time.monotonic() - started

    def _perf(self, started):
        """This tick's timing. Cheap, and the only record of falling behind.

        `late_ms` is measured against the tick period rather than against the
        previous tick, so it says "this tick overran" rather than "the loop is
        running at some rate", which is the question asked afterwards.
        """
        elapsed = time.monotonic() - started
        return {
            "tick_ms": round(elapsed * 1000.0, 2),
            "last_tick_ms": round(self._last_tick_s * 1000.0, 2),
            "late_ms": round(max(0.0, elapsed - config.TICK_PERIOD) * 1000.0, 2),
            "ticks": self.ticks,
            "overruns": self._overruns,
            "tick_errors": self._tick_errors,
        }

    def _cluster_debug(self):
        """This tick's clusters and how each classified, for the recording.

        Taken straight from the world model, which computed them on this very
        tick: `classify` is the expensive part of the loop and re-running it here
        would both double that cost and risk recording a *different* answer from
        the one the boat acted on. The per-point `rgb` is deliberately not
        repeated - the sweep it came from is written alongside at the same rate.
        """
        return self.world.last_measurements

    # -------------------------------------------------------------- perception

    def _perceive(self, state, now):
        """Both lidars -> clusters -> the world model. Returns the scans to relay.

        The front and aft cluster lists are kept apart deliberately: only the
        front list is handed to the docking behaviour, because a berth is found
        from what is *in front of* the boat, and the aft unit's returns would
        offer it gaps behind. Both go into the world model, which does not care
        which sensor a measurement came from.
        """
        front = self._front(now)
        aft = self._aft(state)

        heading = state.heading if state is not None else None
        boat = state.position if state is not None else None

        # Where grid (0, 0) is, so the world model can read and write the survey
        # - which is in lat/lon, because the origin itself does not survive a
        # reboot (`survey.py`). Every tick, because the origin can genuinely
        # change under a running node.
        if state is not None and state.origin:
            self.world.set_origin(state.origin)

        self._front_clusters = self._cluster(front, "front_lidar")
        self._aft_clusters = self._cluster(aft, "aft_lidar")

        task = "transit"
        if self.pilot.behaviour is not None:
            task = getattr(self.pilot.behaviour, "task", "transit")

        self.world.observe(
            self._front_clusters + self._aft_clusters, boat, heading, now, task
        )
        if self.edge is not None:
            self.world.absorb_detections(self.edge.detections(), boat, heading, now)

        return [scan for scan in (front, aft) if scan]

    def _front(self, now):
        """The newest front sweep, boat frame, masked. None if stale or absent.

        Aged on **our** clock from the moment it landed, never against the
        Jetson's own `t_end` - the two machines' wall clocks disagree, and
        judging freshness across that boundary is what once produced "3701
        sweeps received, 0 points plotted" (`io_manager/scan.py`).
        """
        if self.edge is None:
            return None
        cloud, seq, arrived_at = self.edge.front_cloud()
        if not cloud or not seq or arrived_at is None:
            return None
        if now - arrived_at > config.MAX_NAV_AGE_S:
            return None
        if seq == self._front_seq:
            return self._front_scan  # same rotation; do not rebuild it
        self._front_seq = seq

        scan = front_scan(cloud)
        if not scan:
            self._front_scan = None
            return None

        keep = masks.mask_front(scan["points"])
        points, rgb = masks.apply(scan["points"], scan.get("rgb"), keep)
        # The per-point colour ages ride through the same mask as the colours
        # they describe. Filtering one and not the other would shift every age
        # relative to its return, which shows up as nothing at all - just a vote
        # quietly weighted by the wrong numbers.
        ages = masks.select(scan.get("age_ms"), keep, len(scan["points"]))
        scan = dict(scan, points=_listify(points))
        if rgb is not None:
            scan["rgb"] = _flatten(rgb)
        else:
            scan.pop("rgb", None)
        if ages is not None:
            scan["age_ms"] = [float(a) for a in ages]
        else:
            scan.pop("age_ms", None)
        self._front_scan = scan
        return scan

    def _aft(self, state):
        """The aft sweep out of io_manager's state frame, masked.

        io_manager owns that serial port, so the returns arrive already in the
        boat frame (`scan.py`); all that is left here is to cut out the part of
        the sweep that IS the boat, which for a stern-facing unit is everything
        ahead of it.
        """
        if state is None or not state.aft_scan:
            return None
        scan = state.aft_scan
        points = scan.get("points")
        if not points:
            return None
        kept, _rgb = masks.apply(points, None, masks.mask_aft(points))
        return dict(scan, points=_listify(kept))

    def _cluster(self, scan, source):
        if not scan or not scan.get("points"):
            return []
        try:
            return cluster_sweep(
                scan["points"],
                scan.get("rgb"),
                source=source,
                config=config,
                age_ms=scan.get("age_ms"),
            )
        except Exception as exc:  # noqa: BLE001 - a bad sweep must not stop the tick
            log.warning("could not cluster the %s sweep: %s", source, exc)
            return []

    # ---------------------------------------------------------------- commands

    def _commands(self, state):
        """Operator commands forwarded by io_manager. Each one is acked.

        Anything not listed here is left alone rather than refused: io_manager
        reads the same queue and owns `estop`, `set_mode`, `arm` and the rest,
        and acking somebody else's command would race its ack.
        """
        origin = state.origin if state is not None else None
        for command in self.link.commands():
            name = str(command.get("name", ""))
            command_id = command.get("id")
            args = command.get("args") or {}
            log.info("command %s: %s %s", command_id, name, args)
            self.recorder.event("command", name=name, args=args)

            if name == "set_plan":
                ok, result = self.pilot.set_plan(args.get("plan") or args, origin)
                if ok:
                    # Draw the new course on the next telemetry tick rather than
                    # here. `_paths_payload` sends the ideal route and the
                    # committed path as one list, and a second sender would race
                    # it - the layer replaces rather than merges. Half a second
                    # is below noticing, and it costs one special case.
                    self._paths_sig = None
            elif name == "autopilot_start":
                ok, result = self.pilot.start()
                if ok:
                    label = str(
                        args.get("label")
                        or (self.pilot.plan.name if self.pilot.plan else "run")
                    )
                    self.recorder.start(
                        label,
                        plan=self.pilot.plan.to_dict() if self.pilot.plan else None,
                    )
            elif name == "autopilot_stop":
                ok, result = self.pilot.stop(
                    str(args.get("why") or "operator stopped autonomy")
                )
                self.recorder.stop("operator stopped autonomy")
                # End of an attempt: this is the map attempt two starts from.
                self.survey_save("attempt finished")
            elif name == "set_speed_limit":
                # **The one speed setting, and the one command that sets it.**
                # io_manager owns this command as well - it is the hand-flown
                # go-to's cap and the AUTO mission speed there
                # (`io_manager/guided.py`) - so it is deliberately handled in
                # both nodes off the same press, and deliberately **not acked
                # here**: io_manager acks it, and two answers to one command is
                # worse than none (`autopilot_bridge.SHARED_COMMANDS`).
                #
                # Takes effect on the next tick and does not interrupt a run.
                # Every behaviour is held to it, docking included, which is the
                # whole point: 0.1 m/s means a berth approach at 0.1 m/s.
                ok, result = self.commander.set_speed(args.get("value"))
                if not ok:
                    log.warning("set_speed_limit refused: %s", result)
                self.recorder.event("command_result", name=name, ok=ok, result=result)
                continue
            elif name == "alternation":
                # The cardinal alternation prior, off by default. See
                # `behaviours/alternation.py` - it is an inference, and switching
                # it on is a decision somebody makes knowing that.
                ok, result = self.commander.set_alternation(
                    bool(args.get("on", True))
                )
            elif name == "autopilot_pause":
                ok, result = self.pilot.pause()
            elif name == "autopilot_resume":
                ok, result = self.pilot.resume()
            elif name == "autopilot_skip":
                ok, result = self.pilot.skip()
            elif name == "autopilot_back":
                ok, result = self.pilot.back()
            elif name == "autopilot_goto":
                ok, result = self.pilot.jump(args.get("index"))
            elif name == "clear_plan":
                self.pilot.stop("plan cleared")
                self.pilot.plan = None
                ok, result = True, "plan cleared"
            elif name == "record_start":
                path = self.recorder.start(str(args.get("label") or "manual"))
                ok, result = bool(path), path or "could not open a recording"
            elif name == "record_stop":
                path = self.recorder.stop("operator stopped the recording")
                ok, result = True, path or "was not recording"
            elif name == "forget_world":
                # The dashboard's "Clear what it has seen". Clears the live
                # tracks *and* the stored survey - see `WorldModel.forget`,
                # which explains why clearing only one of them is a trap.
                _count, result = self.world.forget(
                    f"operator command {command_id}"
                )
                ok = True
            elif name == "forget_object":
                # Delete one object by id. The id is the `track_id` the chart
                # draws next to the marker, which is `Track.id` here - the
                # server's protocol.py maps our `id` onto `track_id` on the way
                # through, so what the operator clicks is what arrives.
                ok, result = self.world.forget_track(
                    args.get("id", args.get("track_id"))
                )
            else:
                continue  # not ours - io_manager handles it and acks it

            if command_id is not None:
                self.link.ack(command_id, "acked" if ok else "failed", result)
            self.recorder.event("command_result", name=name, ok=ok, result=result)

    # --------------------------------------------------------------- telemetry

    def _publish(self, state, scans, now, intent=None):
        # The front cloud, relayed so the operator's chart keeps its plot. This
        # node holds 3401, so without this the coloured cloud would vanish from
        # the dashboard the moment autonomy started - which is exactly when
        # somebody most wants to see it.
        if scans and now - self._last_relay >= SCAN_RELAY_PERIOD:
            self._last_relay = now
            front = [scan for scan in scans if scan.get("source") == "front_lidar"]
            if front:
                self.link.scan(
                    front,
                    lidar=(self.edge.telemetry() if self.edge is not None else None),
                )

        if now - self._last_heartbeat_log >= HEARTBEAT_LOG_PERIOD:
            self._last_heartbeat_log = now
            log.info(
                "%s | %s | front %d + aft %d clusters -> %d tracks | %s | %s",
                self.pilot.mode,
                state.describe() if state is not None else "no state from io_manager",
                len(self._front_clusters),
                len(self._aft_clusters),
                len(self.world.confirmed()),
                self.world.summary(),
                self.pilot.reason[:60],
            )

        if now - self._last_telemetry < TELEMETRY_PERIOD:
            return
        self._last_telemetry = now

        blocks = {
            "autopilot": {
                **self.pilot.telemetry(state, self.world),
                "commander": self.commander.telemetry(),
                "recording": self.recorder.telemetry(),
                "perception": {
                    "front_clusters": len(self._front_clusters),
                    "aft_clusters": len(self._aft_clusters),
                    **self.world.stats(now),
                    "edge": (
                        "connected"
                        if self.edge is not None and self.edge.connected
                        else "no Jetson"
                    ),
                },
                "survey": self.survey.telemetry(),
                "bus": self.link.stats(),
                "ticks": self.ticks,
            },
            "tracks": self.world.telemetry(now=now),
        }
        if (payload := self._artag_payload()) is not None:
            blocks["artags"] = payload
        if (layers := self._paths_payload(state, intent, now)) is not None:
            blocks["paths"] = layers
        self.link.telemetry(**blocks)

    def _artag_payload(self):
        """`telemetry.artags` - the raw tag sightings, for the operator's chart.

        Its own block rather than a field on `autopilot`, because it is a *sensor
        reading* and it has to be visible whether or not a docking behaviour is
        running: on the day the first question is "can the boat see the tags from
        here", asked while driving the boat around by hand looking for somewhere to
        put the middle waypoint. `autopilot.parking.tags` is the other half - what
        was *decided* from these - and the two are deliberately at different levels
        because one of them exists without the other.

        Boat-relative metres, `[starboard, forward]`, exactly like `scans`: the
        browser already knows how to rotate that onto the chart against the boat's
        pose, and a world-frame tag would need a fix the bench does not have.

        `None` when the Jetson has never said whether it is looking, so the page can
        tell "not searching" from "searching and sees nothing" - the distinction
        `edge_protocol.py` keeps on the wire for exactly this.
        """
        if self.edge is None:
            return None
        searching = self.edge.tags_searching()
        if searching is None:
            return None
        seen = self.edge.tags()
        return {
            "searching": bool(searching),
            "n": len(seen),
            "tags": [
                {
                    "id": tag.get("id"),
                    "cam": tag.get("cam"),
                    # [starboard, forward] from [x, y=down, z=forward].
                    "point": [round(float(tag["pos_rig"][0]), 3),
                              round(float(tag["pos_rig"][2]), 3)],
                    "range_m": tag.get("range_m"),
                    "bearing_deg": tag.get("bearing_deg"),
                    # The resolution figure. On the panel because it is what says
                    # whether a tag is worth believing, and because it is the number
                    # that decides how close the boat has to get.
                    "edge_px": tag.get("edge_px"),
                    "incidence_deg": tag.get("incidence_deg"),
                }
                for tag in seen
                if isinstance(tag.get("pos_rig"), (list, tuple))
                and len(tag["pos_rig"]) >= 3
            ][:16],
        }

    def _paths_payload(self, state, intent, now):
        """The chart's two lines, or None when there is nothing new to say.

        Both layers go in one `paths` list because the list **replaces** rather
        than merges (`ligmax-server/ligmax_gui/state.py`, `_merge`): a frame that
        carried only the cyan committed path would wipe the amber ideal route off
        the chart, and one that carried only the amber would wipe the cyan. See
        `paths.py`.

        Two reasons this is not simply sent every time:

        **Silence is not the same as an empty list.** With no plan loaded, this
        node has no opinion about the layer and must not publish one - the
        `paths` `io_manager` echoes back after an admin-laid MAVLink mission
        upload (`nodes/io_manager/main.py`, `finish_mission`) is the only thing
        on the chart in that state, and a stream of empty lists from here would
        hold it permanently blank.

        **An unchanged plan is not worth 2 Hz of 4G.** The reference route is a
        dozen waypoints with their roles and names attached and it only changes
        when the plan does. The keepalive still re-sends it every few seconds, so
        a mission echo that lands on top of these layers is corrected rather than
        left standing.
        """
        if self.pilot.plan is None:
            return None
        layers = paths.layers(state, intent, self.pilot.plan)
        if not layers:
            # A plan is loaded but nothing about it can be drawn yet - no origin,
            # so no grid to place it on. Silence, not an empty list: this node
            # only ever asserts lines, it never blanks the chart, and before the
            # first fix the honest state is "I have not said anything".
            return None
        # Rounded to the centimetre by `paths.py` before it gets here, so an aim
        # point jittering below that does not count as a change.
        signature = repr(layers)
        if signature == self._paths_sig and now - self._paths_at < PATHS_KEEPALIVE:
            return None
        self._paths_sig = signature
        self._paths_at = now
        return layers


def _listify(points):
    """A numpy `(n, 2)` back to the plain lists the wire format uses."""
    array = np.asarray(points)
    if array.size == 0:
        return []
    return [[round(float(a), 2), round(float(b), 2)] for a, b in array]


def _flatten(rgb):
    array = np.asarray(rgb)
    if array.size == 0:
        return []
    return [int(v) for v in array.reshape(-1)]


def main():
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s %(message)s"
    )
    signal.signal(signal.SIGTERM, _handle_signal)
    signal.signal(signal.SIGINT, _handle_signal)

    node = Node()
    node.start()
    log.info(
        "autonomy node up, tick %.0f Hz - observing only until autopilot_start",
        config.TICK_HZ,
    )

    next_tick = time.monotonic()
    try:
        while not _STOPPING:
            try:
                node.tick(time.time())
            except Exception as exc:  # noqa: BLE001 - one bad tick is not the run
                log.exception("tick failed: %s", exc)
                node._tick_errors += 1
                # Into the recording too, with the traceback. A tick that raised
                # is the single most valuable line in a trip file and it was
                # previously only in the journal, which is not what gets copied
                # off the boat afterwards.
                node.recorder.event(
                    "tick_error",
                    error=repr(exc),
                    traceback=traceback.format_exc()[-2000:],
                )
                # Never carry on driving through an unexplained failure. The
                # supervisor restarts this node, and the boat should be stopped
                # when it does rather than still holding its last target.
                node.commander.disengage(f"tick raised {exc!r}")

            next_tick += config.TICK_PERIOD
            sleep = next_tick - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                # Fell behind - a slow tick, or the clock stepped. Re-base
                # rather than trying to catch up, which would run a burst of
                # ticks back to back on identical sensor data.
                node._overruns += 1
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        log.info("autonomy node stopping")
        node.close()


if __name__ == "__main__":
    main()
