# Ligmax repositories — inventory

**Purpose of this file: stop the same repo being cloned twice.** Before pulling
anything down, check here. After cloning something new, add a row and write down
what is actually in it — the point is that the next person (or the next agent
session) can answer "where does X live?" without a download.

Clone siblings of this checkout, i.e. into `/home/admin/`, so the relative
paths in every docstring (`../ligmax-server/web/js/map.js`) resolve.

## Status

| Repo | Remote | Cloned here? | What it is |
|---|---|---|---|
| **ligmax-pi** | `github.com/ligmax-ntnu/ligmax-pi` | ✅ `/home/admin/ligmax-pi` | **This one.** The Raspberry Pi 5: MAVLink to the Pixhawk, telemetry uplink, aft lidar, BMS, lights, E-stop GPIO, and (new) the autonomy node. |
| **ligmax-server** | `github.com/ligmax-ntnu/ligmax-server` | ✅ `/home/admin/ligmax-server` | The ground-station dashboard at `live.ligmax.no`. Flask on 127.0.0.1:3338 behind Caddy behind Cloudflare. ~1.6 MB, no submodules. |
| **ligmax-edge** | `github.com/ligmax-ntnu/ligmax-edge` | ✅ `/home/admin/ligmax-edge` | The Jetson: two cameras, YOLO buoy detector, the **front** RPLidar C1, and the camera↔lidar fusion that colours it. |
| firmware / ArduPilot scripting | unknown | ❌ not cloned | `amas.lua`, `battery_slider.lua` (run on the Pixhawk), `battery_slider.ino` (slider ESP32), the ama translator ESP32. May live inside one of the above. |
| `andreasviner/ligmax-pi` | `github.com/andreasviner/ligmax-pi` | ✅ `/home/admin/ligmax-rpi` | An **older personal fork of this same repo**, plus `mav.parm` / `mav.tlog` captures and a `test.py` MAVLink scratchpad. Nothing unique except the parameter dump. Do not develop here. |

### Re-cloning, or cloning onto a fresh Pi

DNS works from this Pi's shell now, and both siblings were pulled down on
**2026-08-09**. Everything in the two sections below was written when they were
*not* available and was reconstructed from this repo's docstrings; the parts
that have since been checked against the real code are marked **VERIFIED**, and
the parts that turned out to be wrong are marked **WRONG**. Anything unmarked is
still only inference.

```sh
cd /home/admin
git clone https://github.com/ligmax-ntnu/ligmax-server.git
git clone https://github.com/ligmax-ntnu/ligmax-edge.git
```

Clone them as siblings of this checkout, i.e. into `/home/admin/`, or the
relative paths in every docstring (`../ligmax-server/web/js/map.js`) stop
resolving.

---

## ligmax-server — what this repo believes about it

The dashboard. Referenced from `nodes/io_manager/*` by path, so these are
real files:

* `ligmax_gui/protocol.py` — the telemetry frame format. Every field optional;
  the server merges each frame into a live snapshot. Lists **replace** on merge,
  dicts merge (two levels deep). `protocol.py:481` maps MAV severities to the
  dashboard's five log levels. Lists the five statuses in the order
  `status.py` uses them.
* `ligmax_gui/state.py` — the merge itself. The reason `scans` and `paths` must
  always be published complete rather than incrementally.
* `ligmax_gui/server.py:48` — rejects a body over 4 MB.
* `web/js/telemetry.js` — the widgets. Expects `soc` as a **fraction**, volts,
  amps, watts, °C; has `goodValues`/`warnValues` for the GNSS fix names.
* `web/js/map.js` — the chart. Draws a **metre grid**, not degrees; the amber
  "ideal route" layer is a `path` with `kind: "reference"`.
* `web/js/geo.js` — grid metres ↔ lat/lon for the cursor readout. Uses
  `METRES_PER_DEGREE_LAT = 111320`. **VERIFIED equal** in all three places:
  here, `nodes/io_manager/navigation.py:99`, and
  `nodes/self_driving/geo.py:38`.
* `ligmax_gui/protocol.py:376-441` — `_normalise_track` is a **whitelist**. A
  track field not on it is dropped silently. Accepted: `track_id` (our `id`
  works — line 393 falls back to it), `position`, `type`, `confidence`,
  `avoid_radius`, `heading`, `velocity`, `age`, `hits`, `misses`, `radius`,
  `speed`, `width_m`, `source`, `label`, `why`, `cardinal`, `no_go`. Anything
  else — `sigma_m`, `established` — never reaches the browser, so the Pi carries
  the uncertainty in `radius`/`avoid_radius`/`age` and says the rest in `why`.
* `web/js/status.js` — the status indicator, driven by the top-level `status`.
* `/led_control` — the light pattern authoring page.
* `.env` — holds `LIGMAX_BOAT_KEY` (per-boat ingest secret) and
  `LIGMAX_NODE_KEY` (the update channel's key).
* `docs/` — `findings.md`, `architecture.md`, `comms.md`, `hosting.md`,
  `hardware.md`, `testing.md`. Cited constantly by this repo's docstrings;
  **these are the files most worth reading first** once it is cloned.

Endpoints this repo uses: `POST /api/ingest` (frame up, queued operator
commands back in the reply). There is also an unauthenticated UDP ingest on
8771 that is never port-forwarded.

## ligmax-edge — what this repo believes about it

The Jetson. Dials **out** to the Pi on **TCP 3401** and streams two message
kinds over one length-prefixed protocol — the wire format is mirrored verbatim
in `nodes/io_manager/edge_protocol.py`, which is the authoritative copy on this
end.

* `rig.json` — hand-measured rig geometry. **The front lidar's rotation is
  corrected here and nowhere else.** If front returns come out rotated, fix it
  there, not in `scan.py`. **VERIFIED, and the news is bad:** `yaw_deg` is still
  `-45`, which the file itself marks STALE — it was measured for the old,
  right-side-up mount, and the unit was remounted upside down on 2026-08-08.
  `angle_dir` was flipped to `-1` at the same time and has never been run against
  hardware. Until both are remeasured the front lidar's bearings — and the
  colouring, which projects through the same geometry — are not trustworthy.
  See `next_step.md` §1; this blocks everything else on the water.
* `cam0` is **PORT**-facing (`yaw -75`) and `cam1` **STARBOARD** (`yaw +75`) —
  back to back, 150° apart, not both looking forward. Corrected in `rig.json` on
  2026-08-08 after a green light shone into cam1 came out on the port half of
  the plot.
* `lidar.py` — the C1 driver `nodes/io_manager/lidar.py` was ported from.
* `fusion.py` — colours lidar returns from the camera frames. Owns the yaw
  convention (positive to starboard, about +y/down) that `scan.py` copies.
* `receiver.py` — had the bug the whole edge link is written around: it read
  `header["cam"]` without checking `kind` first, so every lidar sweep was filed
  as camera 0.
* `run.sh` — **WRONG.** `PORT` defaults to 3401, but the front lidar is **on by
  default**, not opt-in: `run.sh:78` is `if [ "${LIDAR:-1}" = "0" ]`, so only
  `LIDAR=0 ./run.sh` turns it off. Verified 2026-08-09.
* `cloud_camera.py` — pushes preview JPEGs straight to shore over HTTPS. The
  operator's camera image does **not** pass through the Pi.
* `--buoy-diameter` — 0.40 m for Njord marks, used for range-from-apparent-size.

Measured on its C1, settled: 10.0 Hz, ~400 returns/rev at 0.9°, about a third
with distance 0 (dropped at the driver).

---

## Live capture from the Jetson, 2026-08-08

Taken on this Pi with the service stopped, 12 s, boat indoors. Confirms the
protocol and adds fields the docstring does not mention:

* **232 camera frames** (~12 fps/camera, two cameras) and **97 lidar sweeps**
  (9.9 Hz) in 12 s.
* Lidar block keys: `seq n dropped coloured stale in_time t_start t_end hz
  skew_ms x y z dt_ms age_ms q cam det rgb frame`. **`age_ms`, `dropped` and
  `stale` are undocumented in `edge_protocol.py`** — `stale` was 179 of 269
  points in the sample, i.e. most returns were coloured from a frame older than
  the freshness window.
* `y` is identically 0.0 — the C1 is a planar scanner, so the rig frame's
  vertical axis carries no information. 2-D is not an approximation here.
* `rgb` is flat, 3·n long. **WRONG about "sensor-native":** the Jetson runs the
  OV5647 correction matrix (`fusion.py::_correct`) before sending, on top of the
  ISP chroma gain in the frame header's `saturation` field (default 2.0). The
  values arriving are *corrected*. This invalidates the reasoning behind
  `config.MIN_SATURATION = 0.55` — see `next_step.md` §5.
* Detections carry a `lidar` sub-block (`n n_used range_m sigma_m nearest_m
  spread_m mixed bearing_deg cam`) with bearing in the **rig** frame while the
  detection's own `bearing_deg` is in the **camera** frame. Do not mix them.
* Cardinal detections carry `card` ("west") and `card_conf` from a second-stage
  classifier — the only source on the boat for *which* cardinal a mark is.
