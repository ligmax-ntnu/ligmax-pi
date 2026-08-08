# Next steps

Where the autonomy stack stands after 2026-08-08, what has to happen next, and
in what order. Read `njord.md` first for the rules the design answers to, and
`repos.md` before downloading anything.

**Njord 2026 is 10-14 August.** Everything below is ordered by what it costs if
it is missing on the day.

---

## 0. Status

Built and passing `python3 tests/test_autopilot.py` (70+ checks, no hardware):

| | |
|---|---|
| `nodes/self_driving/` | the autonomy node - owns TCP 3401, the world model, the plan, the recording |
| `nodes/io_manager/autopilot_bridge.py` | the io_manager end: state out, control in, MAVLink between |
| `tools/review_trip.py` | read back a recorded run |
| `tests/test_autopilot.py` | full simulation: fake hull, fake lidar, everything else real |

Verified on the Pi, live, 2026-08-08: both nodes start under the supervisor, the
node bus runs at 9.9 Hz, the Jetson connects to `self_driving` on 3401,
io_manager stays off that port and gets the front sweep relayed back, and the
pilot correctly refuses to drive with no GPS fix.

**Not yet done on hardware:** anything involving the Pixhawk. It was unplugged
the whole session - see §1.

---

## 1. Hardware, before anything else

### 1.1 The Pixhawk's USB drops out — **blocking**

`dmesg` on this boot:

```
[    0.995694] usb 3-2: Product: Pixhawk6C
[    3.485420] cdc_acm 3-2:1.0: ttyACM0: USB ACM device
[ 1360.990122] usb 3-2: USB disconnect, device number 2
```

It enumerated at boot and **disconnected 22 minutes later and never came back**.
Nothing in software can work around this: no MAVLink means no position, no mode,
no arming, and `pilot.py` will correctly refuse to drive.

`propulsion.py` already documents the likely cause - the Pixhawk shares the power
rail the E-stop cuts, which is called out there as a wiring fault expected to be
rewired. Whether this drop is that, or a cable, it has to be found and fixed
before any on-water test. **This is the single highest-priority item in this
file.**

### 1.2 Confirm GUIDED actually works on the vehicle

The whole control layer assumes ArduPilot Rover accepts, in GUIDED:

* `SET_POSITION_TARGET_GLOBAL_INT` with the position bits (transit legs), and
* `SET_POSITION_TARGET_LOCAL_NED` in `MAV_FRAME_BODY_NED` with velocity +
  yaw-rate (docking, holding, reversing).

Test on the water, disarmed first and then in a clear area:

```sh
# with the boat armed, in GUIDED, watch it move to a point 10 m north
python3 -c "
from pymavlink import mavutil
import time
m = mavutil.mavlink_connection('/dev/ligmax-pixhawk', baud=115200); m.wait_heartbeat()
m.mav.set_position_target_local_ned_send(
    0, m.target_system, m.target_component, mavutil.mavlink.MAV_FRAME_BODY_NED,
    0b0000011111000111, 0,0,0, 0.3,0.0,0.0, 0,0,0, 0, 0.0)
"
```

If body-frame velocity is **not** honoured, docking and station keeping have to
fall back to `RC_CHANNELS_OVERRIDE` on the throttle and steering channels. The
seam for that is one file: `nodes/self_driving/commander.py`. Nothing else
changes.

### 1.3 The sideways thruster — settled

**Confirmed 2026-08-08: ArduPilot owns it as a lateral motor output.** So
`config.LATERAL_MODE` stays at its default, `mavlink`, and the `vy` term of the
GUIDED body-velocity command drives it. Nothing to configure — but it is worth
**verifying on the water** that a commanded `vy` actually produces sideways
movement, because if it does not the symptom is silent: the autopilot simply
drops the term and the boat creeps forward instead of crabbing.

Quick check, boat armed in GUIDED, clear water:

```sh
python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('/dev/ligmax-pixhawk', baud=115200); m.wait_heartbeat()
m.mav.set_position_target_local_ned_send(
    0, m.target_system, m.target_component, mavutil.mavlink.MAV_FRAME_BODY_NED,
    0b0000011111000111, 0,0,0, 0.0,0.25,0.0, 0,0,0, 0, 0.0)
"   # vy = +0.25 -> it should move to STARBOARD without turning
```

The two fallbacks exist if that check fails:

* `LIGMAX_LATERAL_MODE=rc` **plus** `LIGMAX_LATERAL_RC_CHAN=<n>` — the Pi drives
  the channel directly. It refuses to guess a channel deliberately:
  `pixhalwk.py` already uses 16 for ride height and 15 is the remote's
  contribution to the same sum, so neither is free.
* `LIGMAX_LATERAL_MODE=none` — honest, and it works: parallel docking degrades
  to an angled approach (`dock.Dock._crab`).

### 1.4 Measure and set

| Env var | What | Why it matters |
|---|---|---|
| `LIGMAX_AFT_LIDAR_ANGLE_DIR` | `+1` / `-1` | A flipped sign gives a plausible but **mirrored** world astern - the kind of wrong that survives a casual glance. `scan.py` says this has never been checked on hardware. |
| `LIGMAX_AFT_MASK_HALF_WIDTH_M` | 0.5 | Half-width of the corridor masking the aft unit's view of the boat. Measure the widest thing it can see forwards. |
| `LIGMAX_AP_BUOY_CLEARANCE_M` | 2.0 | Half the beam (amas included!) + buoy radius + fix error. |
| `LIGMAX_AP_DOCK_LATERAL_M` | 0.25 | The 2 m berth minus the boat's beam, halved. **Measure the hull.** |
| `LIGMAX_AMA_*_CH`, `LIGMAX_SLIDER_CH` | - | Still unset; `trim.py` publishes nothing without them. |

---

## 2. The website — what to build in `ligmax-server`

This is the biggest remaining piece and none of it is in this repo.

### 2.1 The command protocol — already implemented on the boat

Send these as ordinary operator commands through the existing
`POST /api/ingest` reply channel. io_manager routes anything in
`AUTOPILOT_COMMANDS` to the autonomy node, which **acks each one itself** with a
human-readable result.

| Command | Args | Does |
|---|---|---|
| `set_plan` | `{plan: {...}}` | Load a route (below). Refuses with a readable reason. Pauses a running plan rather than swapping under it. |
| `clear_plan` | - | Stop and forget the plan. |
| `autopilot_start` | `{label?}` | **Engage.** Requests GUIDED + arm, starts a trip recording. |
| `autopilot_stop` | `{why?}` | Disengage, HOLD, close the recording. |
| `autopilot_pause` / `autopilot_resume` | - | Hold station / carry on. |
| `autopilot_skip` | - | Treat the current waypoint as done. |
| `autopilot_back` | - | Step back one — **NJORD §8.2's "re-enter behind the last passed waypoint"**. |
| `autopilot_goto` | `{index}` | Jump the cursor. |
| `record_start` / `record_stop` | `{label?}` | Record without engaging - useful for a manual run. |
| `forget_world` | - | Clear the tracker between tasks. |

### 2.2 The plan format

```jsonc
{
  "name": "task1",
  "channel_bearing": 0,          // direction of buoyage; Njord lays it north
  "start_at": 0,                 // optional: resume mid-course after a takeover
  "waypoints": [
    {"name": "1",   "lat": 63.43, "lon": 10.39, "role": "transit"},
    {"name": "1.1", "x": 12.0, "y": 30.0,       "role": "transit"},
    {"name": "2",   "lat": 63.44, "lon": 10.40, "role": "buoys", "speed": 0.8},
    {"name": "4",   "lat": 63.44, "lon": 10.41, "role": "hold",  "hold_s": 0},
    {"name": "5",   "lat": 63.44, "lon": 10.42, "role": "avoid"},
    {"name": "7",   "lat": 63.44, "lon": 10.43, "role": "dock",  "hold_s": 10},
    {"name": "8",   "lat": 63.44, "lon": 10.44, "role": "dock_parallel", "hold_s": 5}
  ]
}
```

* **`lat`/`lon` or `x`/`y`** (grid metres) - both work. Grid metres are converted
  against the current origin at upload time; lat/lon is stored canonically.
  **The morning handout will be degrees, so make the paste-a-coordinate path the
  primary one.**
* Roles: `transit` (blind GNSS), `buoys` (+ lateral marks and cardinals),
  `avoid` (+ COLREG), `hold` (arrive and station-keep; `hold_s: 0` = forever),
  `dock` (bow-in, hold, reverse out), `dock_parallel` (alongside, hold, ahead).
* Optional per waypoint: `speed`, `radius`, `hold_s`, `channel_bearing`,
  `berth_width_m`, `notes`.

### 2.3 The autopilot panel

`telemetry.autopilot` is published every 0.5 s and already contains everything.
**NJORD §11.4 scores "decision-making transparency" explicitly**, so this panel
is worth points on its own:

```jsonc
{
  "mode": "RUNNING",                  // IDLE/RUNNING/PAUSED/BLOCKED/FINISHED
  "reason": "red buoy #3 at 12 m must be to port - shifting 1.4 m",
  "behaviour": "buoys",
  "stuck": false,
  "blocked": "...",                   // only when refusing to drive
  "sees": "2x red buoy, 1x cardinal (side unknown)",
  "plan": {"name": "task1", "index": 3, "current": "1.3", "role": "buoys",
           "last_passed": "1.2", "waypoints": 12, "finished": false},
  "distance_to_waypoint": 18.4,
  "bearing_to_waypoint": 47.0,
  "commander": {"engaged": true, "intent": "goto", "speed_cmd": 0.8, ...},
  "recording": {"recording": true, "file": "20260810-091455-task1.jsonl.gz"},
  "perception": {"front_clusters": 14, "aft_clusters": 6, "confirmed": 3,
                 "edge": "connected"}
}
```

Minimum useful UI, in priority order:

1. **A big status line**: `mode` + `reason`. One sentence, large type. A jury
   member should be able to read it from behind the operator.
2. **Start / Stop / Pause** buttons, and **Skip / Back**. Back is the §8.2
   recovery and needs to be reachable in one tap under pressure.
3. **Plan progress**: waypoint N of M, its role, distance to it.
4. **A plan editor**: paste coordinates, pick a role per row, send. It must be
   usable *on a phone, on a dock, at 08:15*, because that is when the course
   arrives. Nothing clever - a textarea that accepts pasted lat/lon pairs and a
   role dropdown per line beats a drag-and-drop map you have to be sitting down
   to use.
5. **`stuck`** shown loudly. It means the twenty-second window in §8.2 is
   running.

### 2.4 The chart

* **`tracks`** - published every 0.5 s, a list of what the boat believes is out
  there. Each has `position` (grid metres), `type` (the `ObstacleType` **number**
  - mirror `nodes/self_driving/obsticales.py`, and note the enum values are a
  wire format), `label`, `confidence`, `width_m`, `why`, and `velocity` for
  things that move. Draw them in their real colours; show `why` on hover.
* **`path`** with `kind: "reference"` - the plan, published on upload. The
  dashboard already draws this amber layer. §11.4 asks for the actual course
  over ground "compared against the ideal route" - that comparison is this layer
  plus the existing track history.
* **`scans`** - unchanged; the front cloud is now relayed by the autonomy node,
  so it keeps working exactly as before.

### 2.5 Present-but-unused telemetry worth surfacing

`telemetry.autopilot_bridge` says whether the node bus is delivering. "The
autonomy node is not running" and "the bus is broken" look identical without it.

---

## 3. Tuning, on the water, in this order

Every one of these is an env var in `/etc/ligmax/node.env`; nothing needs a
commit. `nodes/self_driving/config.py` documents each with where its default
came from.

1. **Colour thresholds - do this first, in the day's light.**
   Measured on 6879 real returns on 2026-08-08: the Jetson's RGB is
   **uncalibrated** (sensor-native, the colour matrix runs at the receiver), and
   an indoor scene averaged (80, 48, 44) - a strong warm cast that made 46 of 49
   clusters classify as RED. `MIN_SATURATION` was raised to 0.55 as a result,
   which turns that scene into honest UNKNOWNs.
   * Put a real red and a real green buoy in front of the boat, on the water,
     and check `telemetry.autopilot.sees`.
   * If a cast persists, try `LIGMAX_AP_WHITE_BALANCE=1` (grey-world, per sweep,
     off by default because it fails when one colour fills the view).
   * **The proper fix is white balance on the Jetson**, where the sensor is. See
     §5.
2. **`LIGMAX_AP_BUOY_CLEARANCE_M`** - watch a pass and see if it looks tight.
3. **`LIGMAX_AP_CRUISE_SPEED_MS` / `CAUTION_SPEED_MS`** - the time multiplier is
   worth at most 9 %; a broken autonomy run costs far more. Start slow.
4. **`LIGMAX_AP_LOOKAHEAD_M`** - raise if the boat weaves, lower if it cuts
   corners.
5. **Docking**: `DOCK_SPEED_MS`, `DOCK_STANDOFF_M`, `DOCK_ALIGN_DEG`. The 2 m
   berth is the tightest thing on the course.
6. **`LIGMAX_AP_CARDINAL_VOTES`** (default 4) - raise if the detector flaps,
   lower if it never commits and the boat keeps taking the "side unknown" path.

---

## 4. Known gaps in the autonomy itself

Ordered by what they cost.

1. **Cardinal marks depend entirely on the camera.** No lidar can see a topmark.
   `CardinalVote` requires 4 consistent votes with a margin of 2 before
   committing, and falls back to the planned line when it never does. If the
   YOLO cannot classify cardinals at all on the day, Task 1 part 2 has to be
   flown by laying the intermediate waypoints so the *planned route already
   passes on the correct side* - which is legal, and is the fallback to rehearse.
2. **No AR-tag pipeline.** Deliberate: NJORD §10.4 makes them optional and gives
   **bonus points for docking without them**, and the berth is found from
   geometry instead. If lidar berth-finding fails on the day, a tag detector on
   the Jetson is the backup, and the berth centre would enter through the same
   `Dock._find_berth` seam.
3. **Deconfliction is a steering nudge, not a planner.** It cannot escape a
   concave trap. `pilot.py` detects no-progress and raises `stuck` rather than
   pretending otherwise. A proper planner is next year's job.
4. **The Otter's velocity is a finite difference of a smoothed track.** Good
   enough for a CPA at 20 m; it will lag a sharp turn by a second or so.
5. **`world.py` has no map memory.** Marks are dropped after
   `TRACK_DROP_AFTER_S` (6 s) unseen. Correct on a 3D fix; with **RTK fixed** it
   would be worth remembering them for a whole leg, which would let the boat
   plan a gate it can no longer see. That is a real upgrade and it is cheap.
6. **Nothing uses the gate structure of Task 2.** `perception.split_by_gap`
   already finds 5 m pairs (§9.2) - the same function the berth finder uses - but
   `buoys.py` treats each mark individually. Steering explicitly at gate
   midpoints would be tidier and more robust than obeying two lateral rules.
7. **`STATIONARY_SPEED_MS` is not verified against the tide.** "Stay stationary"
   is scored twice.

---

## 5. For the other repos

### `ligmax-edge` (the Jetson)

1. **White-balance the RGB before it goes on the wire**, or send the gains
   alongside it. This is the single change that would most improve buoy
   classification, and it belongs there - that is where the sensor and its
   colour matrix are. See §3.1 for the measurement.
2. **Mask the front lidar's view of the boat** on that side, where `rig.json`
   lives. (Already planned per the 2026-08-08 conversation. `masks.mask_front`
   exists on the Pi as a backstop but defaults to `none` - two places correcting
   one occlusion is how a rig gets corrected twice.)
3. `run.sh` still needs **`LIDAR=1`** by hand. Make it the default: nothing on
   the boat works without it now.
4. The lidar block sends `age_ms`, `dropped` and `stale` that
   `edge_protocol.py` does not document. In the 2026-08-08 capture **179 of 269
   points were `stale`** - coloured from a frame outside the freshness window.
   Worth understanding; it may be costing colour accuracy.

### `ligmax-server`

Everything in §2. Also: `tracks` and `path` are already being published, so the
chart work can start before any of the UI.

---

## 6. Competition-day runbook

**08:00** - coordinates handed out.

1. Power up. `journalctl -u ligmax-pi -f` and confirm the autonomy node's
   heartbeat line: `IDLE | at (x, y) hdg ... | front N + aft M clusters -> K tracks`.
2. Check `telemetry.gps.fix` is **RTK_FIXED**, not 3D. Everything below assumes
   centimetres.
3. Type the course into the dashboard, one plan per task. Send `set_plan`, check
   the ack, check the amber route on the chart is the right shape.
4. Drive to the start under remote control (the rules require this).
5. **`autopilot_start`.** The timer starts when the boat goes autonomous.
6. Watch `mode` and `reason`. If `stuck` goes true, you have ~20 s before §8.2
   says take over.
7. After the run: `autopilot_stop`, then

   ```sh
   python3 tools/review_trip.py                  # summary of the last run
   python3 tools/review_trip.py --html /tmp/t.html
   ```

   Read the waypoint table and the `tracked` list **before** attempt two. That is
   what the recorder is for.
8. Between tasks: `forget_world`, then `set_plan` for the next one.

**If autonomy will not engage**, the reason is in `telemetry.autopilot.blocked`
and it is one of seven things, in this order: not engaged / no state / comms lost
/ E-stop / a pilot has it (MANUAL) / disarmed / no fix / no plan.

---

## 7. Repo housekeeping

* `nodes/self_driving/{pixhalwk,path_finding,tracking}.py` were **deleted** -
  broken stubs superseded by `perception/` and `commander.py`. `obsticales.py`
  was kept and extended because the frontend mirrors its enum values.
* `nodes/logging/main.py` and `nodes/balancing/main.py` are still empty stubs and
  still `"on": False` in the supervisor.
* `nodes/io_manager/pixhalwk.py` (ride height) is still not wired into
  `main.py`. The autonomy node does not touch ride height at all.
* There is no `CLAUDE.md`; the docstrings are the documentation, which works
  because they are unusually thorough. Keep it that way.
