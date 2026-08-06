"""The Daly BMS as the authoritative battery source, read off the loop.

`battery.py` next door already speaks to the pack: it fires nine Daly commands at
`can1` and collects replies for 0.8 s. That 0.8 s is the whole problem. The
MAVLink loop in `main.py` owes the autopilot a 1 Hz heartbeat, and a blocking
call most of a second long inside it is a failsafe waiting to happen
(`test.py:48`). So this module runs `battery.py` on a worker thread and hands the
loop whatever the last completed read said.

**Why the BMS and not the autopilot.** The state of charge on the operator's
screen has to be the pack's own figure. The autopilot's `BATTERY_STATUS` is a
coulomb-count estimate seeded from a voltage curve, and on a 12S12P pack under a
5 kW load it sags far enough under acceleration to read low by a wide margin
exactly when someone is deciding whether there is enough left to finish a run.
The Daly has the shunt and the cell taps. `main.py` keeps the autopilot's numbers
as a labelled fallback, and `source` in the telemetry says which one is answering,
so nobody has to guess.

Energy remaining
----------------
The requirement asks for Wh, and the Daly does not report Wh. It reports SOC as a
percentage, pack voltage, and the pack's rated capacity in Ah. So:

    remaining_wh = capacity_ah * (soc / 100) * pack_voltage

Two things to know about that number. It uses the *live* pack voltage, not a
nominal 44.4 V, so it sags with load and recovers at rest - honest about the
instantaneous energy, slightly jumpy under hard acceleration. And it inherits
whatever the Daly's SOC estimate is worth; it is not an independent measurement.
It is derived from BMS-reported values only, and `remaining_wh_basis` says so.

Nothing here is a hardware constant. `capacity_ah` comes off the BMS at runtime
(Daly command 0x93), so re-cell the pack and this follows without a code change.
"""

import json
import logging
import os
import threading
import time

from .battery import get_battery_data

log = logging.getLogger("io_manager.bms")

# How often to start a read. Each one occupies its thread for ~0.85 s, so a
# period under about 1.5 s means the worker is essentially always busy for no
# extra freshness - the pack's SOC does not move that fast.
POLL_PERIOD = float(os.environ.get("LIGMAX_BMS_POLL_S", "2.0"))
# After a failed read, wait longer: the usual cause is `can1` not being up, and
# retrying at 2 s intervals just fills the log.
ERROR_PERIOD = float(os.environ.get("LIGMAX_BMS_ERROR_S", "10.0"))

CHANNEL = os.environ.get("LIGMAX_CAN_CHANNEL", "can1")
BITRATE = int(os.environ.get("LIGMAX_CAN_BITRATE", "250000"))
CELL_COUNT = int(os.environ.get("LIGMAX_PACK_CELLS", "12"))

# A reading older than this is not published as current. The dashboard would
# rather show a gap than a state of charge from two minutes ago.
STALE_AFTER = float(os.environ.get("LIGMAX_BMS_STALE_S", "30.0"))


class BmsReader:
    """Polls the Daly BMS on a worker thread. `telemetry()` never blocks.

    One thread, one in-flight read, latest result wins. There is no queue: a
    battery reading that has been superseded is of no interest to anyone.
    """

    def __init__(self, channel=CHANNEL, bitrate=BITRATE, cell_count=CELL_COUNT):
        self.channel = channel
        self.bitrate = bitrate
        self.cell_count = cell_count

        self._lock = threading.Lock()
        self._data = None  # last successful read, parsed
        self._read_at = 0.0  # monotonic, when it completed
        self._reads = 0
        self._errors = 0
        self._last_error = None
        self._closed = False
        self._wake = threading.Event()

        self._thread = threading.Thread(target=self._run, daemon=True, name="bms")
        self._thread.start()

    # -- public API ---------------------------------------------------------

    @property
    def available(self):
        """True once a reading has landed and while it is still fresh."""
        with self._lock:
            if self._data is None:
                return False
            return (time.monotonic() - self._read_at) < STALE_AFTER

    def telemetry(self):
        """The `telemetry.battery` block, or `{}` if the BMS has not answered.

        Field names and units are the ones the dashboard already has widgets for
        (`ligmax-server/web/js/telemetry.js`): volts, amps, watts, degrees C, and
        `soc` as a *fraction* rather than a percentage.
        """
        with self._lock:
            data = self._data
            read_at = self._read_at
            errors = self._errors
            last_error = self._last_error

        if data is None:
            # Still say something, so the panel can distinguish "no BMS yet" from
            # "no battery block at all".
            out = {"source": "daly_bms", "bms_ok": False}
            if last_error:
                out["last_error"] = str(last_error)[:120]
            if errors:
                out["read_errors"] = errors
            return out

        age = time.monotonic() - read_at
        pack = data.get("pack") or {}
        cells = data.get("cell_extremes") or {}
        temps = data.get("temp_extremes") or {}
        mosfets = data.get("mosfet_status") or {}
        alarms = data.get("alarm_flags_hex") or []

        out = {
            "source": "daly_bms",
            "age": round(age, 2),
        }
        if age > STALE_AFTER:
            # Publish the age and the fault, but not the figures: a stale SOC that
            # looks current is the failure mode this guards against.
            out["bms_ok"] = False
            out["stale"] = True
            return out

        voltage = pack.get("voltage_v")
        current = pack.get("current_a")
        soc_percent = pack.get("soc_percent")
        capacity_ah = pack.get("capacity_ah")

        if voltage is not None:
            out["voltage"] = voltage
        if current is not None:
            # Daly signs current negative for charging, which matches what the
            # dashboard's `battery.current` widget already expects.
            out["current"] = current
        if voltage is not None and current is not None:
            out["power"] = round(voltage * current, 1)
        if soc_percent is not None:
            out["soc"] = round(soc_percent / 100.0, 4)
        if capacity_ah is not None:
            out["capacity_ah"] = capacity_ah
        if soc_percent is not None and capacity_ah is not None and voltage is not None:
            out["remaining_wh"] = round(capacity_ah * (soc_percent / 100.0) * voltage, 0)
            # Says where the Wh came from, because it is derived rather than
            # measured and an operator planning a run deserves to know that.
            out["remaining_wh_basis"] = "capacity_ah x soc x pack_v"
        if (cycles := pack.get("cycle_count")) is not None:
            out["cycles"] = cycles

        if (max_mv := cells.get("max_cell_mv")) is not None:
            out["cell_max"] = round(max_mv / 1000.0, 3)
        if (min_mv := cells.get("min_cell_mv")) is not None:
            out["cell_min"] = round(min_mv / 1000.0, 3)
        if (delta_mv := cells.get("delta_mv")) is not None:
            out["cell_delta"] = round(delta_mv / 1000.0, 3)
        if reported := data.get("cell_voltages_mv"):
            out["cells"] = len(reported)

        # Hottest cell, because that is the one that matters. The minimum is in
        # the raw block for anyone who wants it.
        if (max_temp := temps.get("max_temp_c")) is not None:
            out["temperature"] = max_temp
        if (min_temp := temps.get("min_temp_c")) is not None:
            out["temperature_min"] = min_temp

        if (charge := mosfets.get("charge_enabled")) is not None:
            out["charge_fet"] = bool(charge)
        if (discharge := mosfets.get("discharge_enabled")) is not None:
            out["discharge_fet"] = bool(discharge)

        if balancing := data.get("balancing_cells"):
            out["balancing"] = len(balancing)

        # Any non-zero alarm byte is a fault the BMS is asserting. Report the
        # bytes rather than trying to name the bits: the Daly protocol's alarm
        # map is not documented in this repo, and inventing labels for it would
        # be worse than showing the operator the raw hex.
        if alarms:
            faulted = [byte for byte in alarms if byte not in ("0x00", "0X00")]
            out["bms_ok"] = not faulted
            if faulted:
                out["alarms_hex"] = " ".join(alarms)
        else:
            out["bms_ok"] = True

        if errors:
            out["read_errors"] = errors
        return out

    def raw(self):
        """The last full BMS read, for the log or a debug dump. May be None."""
        with self._lock:
            return self._data

    def close(self):
        self._closed = True
        self._wake.set()
        # Not joined for long: a read in flight is sitting in `bus.recv()` and
        # will finish on its own. It is a daemon thread and touches no hardware
        # that matters on the way out.
        self._thread.join(0.2)

    # -- worker -------------------------------------------------------------

    def _run(self):
        while not self._closed:
            period = self._read_once()
            self._wake.wait(period)
            self._wake.clear()

    def _read_once(self):
        """One BMS read. Returns how long to wait before the next one."""
        started = time.monotonic()
        try:
            # `get_battery_data` returns a JSON string and swallows its own
            # exceptions into {"error": ...} - it never raises, so the try here is
            # for the unexpected rather than the expected.
            payload = get_battery_data(
                channel=self.channel,
                bitrate=self.bitrate,
                cell_count=self.cell_count,
            )
            data = json.loads(payload)
        except Exception as exc:  # noqa: BLE001 - a dead CAN bus must not kill this
            self._note_error(exc)
            return ERROR_PERIOD

        if not isinstance(data, dict) or "error" in data:
            self._note_error((data or {}).get("error", "malformed BMS reply"))
            return ERROR_PERIOD

        # An empty read - bus up, nothing answering - looks like a success to
        # `get_battery_data`, which returns the skeleton with every field None.
        # Treat that as a failure, because publishing it would show 0 % charge.
        if (data.get("pack") or {}).get("voltage_v") is None:
            self._note_error(f"no reply from the BMS on {self.channel}")
            return ERROR_PERIOD

        with self._lock:
            self._data = data
            self._read_at = time.monotonic()
            self._reads += 1
            first = self._reads == 1
            recovered = self._last_error is not None
            self._last_error = None

        if first or recovered:
            pack = data.get("pack") or {}
            log.info(
                "Daly BMS on %s: %.1f V, %.1f %% SOC, %s Ah rated (read in %.2f s)",
                self.channel,
                pack.get("voltage_v") or 0.0,
                pack.get("soc_percent") or 0.0,
                pack.get("capacity_ah"),
                time.monotonic() - started,
            )
        return POLL_PERIOD

    def _note_error(self, error):
        with self._lock:
            self._errors += 1
            first = str(error) != str(self._last_error)
            self._last_error = error
        if first:
            log.error(
                "BMS read failed on %s: %s - the dashboard falls back to the "
                "autopilot's battery estimate. Is the interface up? "
                "sudo ip link set %s up type can bitrate %s",
                self.channel,
                error,
                self.channel,
                self.bitrate,
            )


if __name__ == "__main__":
    # Bench check on the Pi:  python -m nodes.io_manager.bms
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s %(message)s")
    reader = BmsReader()
    try:
        for _ in range(5):
            time.sleep(2.5)
            print(json.dumps(reader.telemetry(), indent=2))
    finally:
        reader.close()
