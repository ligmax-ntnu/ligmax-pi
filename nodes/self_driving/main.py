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

import numpy as np

from nodes.io_manager.edge_link import EdgeLink
from nodes.io_manager.scan import front_scan

from . import config
from .commander import Commander
from .link import NodeLink
from .perception import cluster_sweep, masks
from .perception.world import WorldModel
from .pilot import Pilot
from .recorder import TripRecorder

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

_STOPPING = False


def _handle_signal(_signum, _frame):
    global _STOPPING
    _STOPPING = True


class Node:
    """Everything wired together. One instance, one process."""

    def __init__(self):
        self.link = NodeLink()
        self.commander = Commander(self.link, config)
        self.world = WorldModel(config)
        self.pilot = Pilot(config, self.commander)
        self.recorder = TripRecorder(config)
        self.edge = EdgeLink(port=config.EDGE_PORT) if config.OWN_EDGE_LINK else None
        self._front_seq = None
        self._front_scan = None
        self._front_clusters = []
        self._aft_clusters = []
        self._last_telemetry = 0.0
        self._last_relay = 0.0
        self._last_heartbeat_log = 0.0
        self.ticks = 0

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
        self.commander.disengage("node shutting down")
        if self.edge is not None:
            self.edge.close()
        self.link.close()

    # ------------------------------------------------------------------- tick

    def tick(self, now):
        self.link.poll()
        state = self.link.latest_state()

        self._commands(state)
        scans = self._perceive(state, now)
        intent = self.pilot.tick(state, self.world, self._front_clusters, now)
        if state is not None:
            self.commander.send(intent, state)

        self.recorder.sample(state, self.world, self.pilot, intent, scans, now)
        self._publish(state, scans, now)
        self.ticks += 1

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

        points, rgb = masks.apply(
            scan["points"], scan.get("rgb"), masks.mask_front(scan["points"])
        )
        scan = dict(scan, points=_listify(points))
        if rgb is not None:
            scan["rgb"] = _flatten(rgb)
        else:
            scan.pop("rgb", None)
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
                scan["points"], scan.get("rgb"), source=source, config=config
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
                    self._publish_route(state)
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
                self.world.forget()
                ok, result = True, "world model cleared"
            else:
                continue  # not ours - io_manager handles it and acks it

            if command_id is not None:
                self.link.ack(command_id, "acked" if ok else "failed", result)
            self.recorder.event("command_result", name=name, ok=ok, result=result)

    # --------------------------------------------------------------- telemetry

    def _publish(self, state, scans, now):
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

        self.link.telemetry(
            autopilot={
                **self.pilot.telemetry(state, self.world),
                "commander": self.commander.telemetry(),
                "recording": self.recorder.telemetry(),
                "perception": {
                    "front_clusters": len(self._front_clusters),
                    "aft_clusters": len(self._aft_clusters),
                    "tracks": len(self.world.all()),
                    "confirmed": len(self.world.confirmed()),
                    "edge": (
                        "connected"
                        if self.edge is not None and self.edge.connected
                        else "no Jetson"
                    ),
                },
                "bus": self.link.stats(),
                "ticks": self.ticks,
            },
            tracks=self.world.telemetry(),
        )

    def _publish_route(self, state):
        """The plan as the chart's ideal-route layer, roles and all. NJORD §11.4.

        The roles ride along with the points because a Njord course is a list of
        places *plus what to do between them*, and a chart that draws only the
        line cannot show that leg 5 is a dock and leg 3 obeys the buoy rules.
        That is also the cheapest place to catch a role typed into the wrong row:
        it is visible on the map the moment the upload is acked, rather than at
        the moment the boat sails past a gate on the wrong side.
        """
        if self.pilot.plan is None or state is None or not state.origin:
            return
        layer = self.pilot.plan.reference_layer(state.origin)
        if layer["points"]:
            self.link.telemetry(
                path={
                    **layer,
                    "kind": "reference",
                    "label": f"plan: {self.pilot.plan.name}",
                }
            )


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
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        log.info("autonomy node stopping")
        node.close()


if __name__ == "__main__":
    main()
