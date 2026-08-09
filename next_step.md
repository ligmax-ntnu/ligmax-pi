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

### Added 2026-08-09

`ligmax-server` and `ligmax-edge` were **cloned for the first time** (DNS works
from this Pi now) and this repo's assumptions were checked against them. See
`repos.md` for what turned out to be right and what did not. Two findings are
serious enough to have their own entries below: **§5.0** (the front lidar's
mounting angle is known-wrong) and **§5.1** (the Jetson now colour-corrects, so
`MIN_SATURATION` is calibrated against a distribution the boat no longer sends).

Also landed on the Pi, in this repo:

* **The world model remembers.** §4.5 below said it did not and that fixing it
  was "a real upgrade and it is cheap" — it is now done. Established static
  marks are kept indefinitely with a position uncertainty that grows to a 6 m
  ceiling while unseen and collapses on the next sighting; vessels are never
  remembered; and the map persists to disk in lat/lon so **attempt two starts
  with attempt one's survey**.
* **Clear-all and delete-one-object**, on the boat side (§2.1, §2.6).
* **Trip recordings now carry everything** needed for a post-mortem, and are
  bounded so they cannot fill the card (§2.7).
* **A real bug fixed:** `recorder.start(label, plan=…)` raised `TypeError` on
  **every** press of "Engage autonomy". The tick caught it, logged it and
  disengaged — so the boat refused to go autonomous and the journal blamed the
  tick. Also fixed a `NameError` on the origin-moved path in `world.py` and a
  rate gate that silently recorded a 10 Hz loop at 6 Hz.

### Added 2026-08-10

This file was audited section by section against all three repos, because most
of §2 and §5 had been built and the document still asked for it. **Everything
struck through below was found already done.** What actually changed:

* **The recordings come off the boat by themselves** —
  `nodes/io_manager/trip_upload.py`, the Pi end of §2.7. The server route had
  landed and `ligmax-server/ligmax_gui/trips.py:283` already referred to this
  file by name. Chunked, resumable, and gated on the vessel being idle. See
  §2.7, which is now a checklist rather than a proposal.
* **A plan asking for 2.8 m/s no longer looks accepted.** The vessel's ceiling
  went to 5 kn (2.5722 m/s) on 2026-08-09; `ligmax-server`'s mirror and the
  browser's `<input max>` both still said 3.0, so the editor accepted it, the
  server returned 200 and the **boat refused the whole plan** a second later.
  That is the exact failure `ligmax_gui/plan.py`'s docstring says the mirror
  exists to prevent. Fixed, and the browser now takes its bounds from
  `/api/session` (`waypoint_limits`) so there is no longer a third copy.
* **`masks.py` stopped claiming the Jetson masks the front lidar.** It does not
  — checked against `ligmax-edge`, nothing there removes the front unit's view
  of the boat. The claim was in a comment, a docstring, **and in `describe()`,
  which goes into telemetry and into every trip header.** See §5.2.

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
  `pixhalwk.py` uses **14** for ride height (moved off 16 on 2026-08-09) and the
  remote's contribution to the same sum is now on **3**. Per `amas.lua`'s table
  the only channel above 8 still free is **13** — 9..12 are the inhibit switch,
  the two trim knobs and the side thruster, and **15/16 carry the radio link's
  own telemetry**, which is what forced the move: 16 sat at a steady ~2006 µs
  with nothing mapped to it, and `amas.lua` reads that as full-speed creep.
  Do not park anything on 15 or 16.
* `LIGMAX_LATERAL_MODE=none` — honest, and it works: parallel docking degrades
  to an angled approach (`dock.Dock._crab`).

### 1.5 The 5 knot speed limit — one gap is on the Pixhawk, not here

**Done in software, 2026-08-09:** autonomous mode is held to 5 knots
(2.5722 m/s), enforced in five independent places — see the block at the top of
`nodes/self_driving/config.py`. The number is not overridable from the
environment; every speed env var is clamped to it as it is read; a plan asking
for more is refused at upload; and `io_manager/autopilot_bridge.py` clamps again
in the last function before the value becomes a MAVLink message, so the limit
does not depend on the autonomy node being correct.

Verified by setting every speed variable to 99 and attacking the loopback
control bus directly: nothing gets out above 5 kn, forward, astern, or as a
forward+lateral resultant.

**The gap this cannot close.** A MAVLink **MISSION run in AUTO** never routes a
speed through this software at all — ArduPilot flies it from its own parameters.
`mission.py` can upload one and `set_mode` can select AUTO, so if the boat is
ever run that way the Pi-side cap is simply not in the loop.

So set the limit on the flight controller too, once, and leave it:

| Param | Set to | Why |
|---|---|---|
| `WP_SPEED` | ≤ 2.57 | Speed for AUTO/GUIDED waypoint navigation. |
| `CRUISE_SPEED` | ≤ 2.57 | The default the throttle controller trims around. |
| `MOT_THR_MAX` | as needed | Blunt backstop if the two above are not honoured on this frame. |

Both are settable from the dashboard's existing `set_param` command, so this
needs no code. Confirm with `CRUISE_SPEED` at 2.5 and a full-throttle GUIDED leg
that the boat actually tops out where it should — an ArduPilot speed parameter
is a controller target, not a governor, so the boat can overshoot it transiently
on a following sea.

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

**Mostly built, as of 2026-08-10.** This was the biggest remaining piece and it
is now the smallest: the command protocol, the plan editor, the autopilot panel,
the chart layers, the map's clear-all and delete-one, and the trip endpoint all
exist and were verified against source. What is left is listed under each
heading and is small. Read this section for what the boat sends and what the
server does with it; do not read it as a build list any more.

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
| `forget_world` | - | Clear **everything** the boat has seen, the stored survey included. Between tasks. |
| `forget_object` | `{id}` | **NEW.** Delete one tracked object by the `track_id` shown on the chart. Also removes it from the survey and refuses that spot for 30 s, so it does not come straight back. |
| `careful_on` / `careful_off` | - | **NEW.** Careful mode: hold the boat to **1 knot**. Takes effect on the next tick and does not interrupt the run. |

`careful_on` / `careful_off` want a **toggle in the autopilot panel**, not two
buttons — it is a state, and the state is already in the telemetry
(`telemetry.autopilot.commander.careful`, plus `speed_ceiling_kn` for what is
actually in force). Server side that is one COMMANDS entry each, no args:

```python
"careful_on":  {"label": "Careful mode (1 kn)", "args": {}},
"careful_off": {"label": "Normal speed",        "args": {}},
```

Worth showing the ceiling next to the speed readout whenever careful mode is on,
because a boat that is inexplicably slow is the second most common thing a crew
misdiagnoses under time pressure.

~~`forget_object` is the only one of these the server does not offer yet.~~
**Every command in the table above is offered by the server and reaches the boat**
(verified 2026-08-10). `forget_world` clears the persisted survey too, which is
what an operator pressing "clear everything" means and what stops it all
reappearing after the next restart.

Three small gaps remain, none of them blocking:

* **`autopilot_goto` has no UI at all.** The server offers it and the boat
  handles it, but nothing in the frontend sends it — the cursor can only be
  moved one waypoint at a time with Skip/Back. A number input beside those two
  buttons in `web/js/autopilot.js` is the whole job.
* **The optional `label` and `why` args are structurally unsendable.** The boat
  reads `autopilot_start {label}`, `record_start {label}` and
  `autopilot_stop {why}`; the server's validator drops undeclared args *and*
  makes every declared arg mandatory (`ligmax_gui/server.py`), so adding them
  naively would break the existing zero-argument buttons. The boat falls back to
  the plan name and to "operator stopped autonomy", which is fine — treat the
  `{label?}`/`{why?}` in the table above as aspirational until the validator
  learns about optional arguments.
* **`start_at`** (§2.2) has no editor field. The §8.2 re-entry is done with
  `autopilot_back`, which does have a button, so this is a nicety.

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
  `berth_width_m`, `notes`. The editor offers `speed` and `hold_s`; the other
  four have to go in as JSON. **`berth_width_m` is the one worth a field** — it
  is the 2 m berth of Task 3 and it is currently unreachable from the dashboard.

**`speed` is bounded by the vessel's 5 kn limit, and the boat refuses the whole
plan rather than clamping.** That is deliberate — an operator finds out on the
dock with the ack in front of them instead of wondering later why the boat would
not hold the speed they typed. It only works if the dashboard agrees about the
number, and until 2026-08-10 it did not: the server's mirror and the editor's
`<input max>` both still said the old 3.0 m/s. The browser now reads the bounds
off `/api/session` (`waypoint_limits`), so the vessel's limit is the only copy
that decides. **If the vessel's limit ever changes, change it in
`ligmax-pi/config.py` and in `ligmax-server/ligmax_gui/plan.py`, and nowhere
else.**

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

  **New since 2026-08-09, and no server change is needed for any of it** — every
  one of these is already on `protocol.py`'s whitelist:

  | field | meaning |
  |---|---|
  | `avoid_radius` | clearance **+ position uncertainty**, metres. The water the boat will actually give it. |
  | `radius` | the position uncertainty alone, metres. |
  | `age` | seconds since the object was last really measured. |
  | `misses` | ageing steps with no measurement — "occluded" vs "never really there". |
  | `why` | now ends with "remembered from N s ago, position good to about X m" when the boat is remembering rather than seeing. |

  `avoid_radius` had **never been sent**: the Pi omitted it, so
  `ligmax_gui/protocol.py` defaulted it to `0.0` and the map never drew a no-go
  disc. It is now sent and the disc draws. **Check it renders sanely** — a
  remembered mark legitimately claims about 8 m (2 m clearance + 6 m
  uncertainty), which is correct but will look large the first time. Note the
  disc needs **both** the "No-go zones" and the radii layer chips on.

  Two corrections to what this section used to promise (2026-08-10):

  * `web/js/nogo.js::pointBlocked()` will **not** "simply start working" — it is
    dead code, nothing calls it. Wiring it into the chart's cursor readout so
    hovering a point says which track blocks it is a small, genuinely useful job.
  * `web/js/telemetry.js`'s "N with avoid radius" counter will jump from 0 to
    *every* track, because every track now has a non-zero `avoid_radius`. It has
    stopped discriminating; counting `avoid_radius > clearance` — i.e. the
    remembered ones — is what that figure was actually for.
* **`path`** with `kind: "reference"` - the plan, published on upload. The
  dashboard already draws this amber layer. §11.4 asks for the actual course
  over ground "compared against the ideal route" - that comparison is this layer
  plus the existing track history.
* **`scans`** - unchanged; the front cloud is now relayed by the autonomy node,
  so it keeps working exactly as before.

### 2.5 Present-but-unused telemetry worth surfacing — **surfaced**

All of it is rendered, in the `/control` autopilot panel. Three things the audit
of 2026-08-10 found still wrong, in descending order of how much they matter:

1. **`telemetry.autopilot_bridge` is read from the wrong place**, so the panel's
   "Bridge" row never renders against the real vessel and the "No autonomy node"
   message never gets its diagnostic half — which is the entire reason this
   field exists. It is a top-level `telemetry` key, not a field of
   `telemetry.autopilot`. Two lines in `web/js/autopilot.js`.
2. **`recording.free_mb` has no warning indicator** — it is plain text in a `dd`
   with no threshold test, so a card at 20 MB free reads exactly like one at
   20 GB. "The card filled up" is something the crew can act on from the dock
   and cannot otherwise see; it wants to go red below `RECORD_MIN_FREE_MB`.
3. **`survey.marks` does not mean what §6 of this file says.** It is
   `Survey.entries_written`, set by a successful `write()` and **never by the
   restore** — so "check `survey.marks` is non-zero at the start of attempt two"
   is checking the wrong number. Use
   `telemetry.autopilot.perception.restored`, which is set by the restore and is
   what the journal line quotes. `survey.file` is published and displayed
   nowhere, so a survey read from an unexpected path is invisible.

### 2.6 Clear-all and delete-one on the map — ~~the remaining UI work~~ **DONE**

Built in `ligmax-server`, verified 2026-08-10. Nothing outstanding.

**Clear all** is a button in the map's own control cluster, with a confirm that
reads the survey figure and warns you are deleting that too.

**Delete one object** is `forget_object` in the COMMANDS table with `{id: float}`
narrowed to an integer server-side; tracks are hit-testable on the chart by
right-click *and* by press-and-hold (so it cannot fire during a pan), and there
is a hint in the tooltip for admins. Covered by `ligmax-server/tests/test_server.py`.

Both confirm before acting, because a mis-click removes a real mark. An undo is
not needed: if the object is real the lidar puts it back, and the Pi's
suppression window is exactly 30 s (`FORGET_SUPPRESS_S`).

### 2.7 Taking trip recordings off the boat — **BUILT, UNTESTED ON HARDWARE**

The server has ~200 GB free; the Pi has a 32 GB card with the OS on it. A
15-minute attempt is **60 MB** worst case (measured 2026-08-09, synthetic
incompressible data — real sweeps compress far better). The card is safe; the
problem was only ever that the recordings were stuck on it, and the run worth
reviewing is often the one where the boat had to be carried back.

Both ends now exist.

**Server** (`ligmax-server/ligmax_gui/trips.py` + four routes): `POST
/api/trip/<name>` with the boat key, `Content-Type: application/gzip`, and
`Content-Range` for resume; `GET /api/trip` listing what is held **and what is
half-uploaded**; `GET /api/trip/<boat>/<name>` to download in the tent; admin-only
`DELETE`. 256 MB per recording, 8 MB per ranged request, refuses once the ground
station is under 2 GB free. Stored as `trips/<boat>/<name>.jsonl.gz`, `<boat>`
from a `?boat=` query arg defaulting to `ligmax`.

**Pi** (`nodes/io_manager/trip_upload.py`, new 2026-08-10). One daemon thread in
io_manager, because io_manager owns every link to shore. What is worth knowing:

* **It never uploads while the vessel is armed, or while a recording is open.**
  The same 4G carries the command channel and ~95 kB/s of lidar plot; a
  recording that arrives an hour late costs nothing. `LIGMAX_TRIP_UPLOAD_WHILE_ARMED=1`
  lifts the first gate for a bench test.
* The "a recording is open" gate **expires** rather than latching, because an
  autonomy node that died mid-run would otherwise pin its own recording to the
  card forever — and that is the recording you want.
* Newest first: attempt two is decided from attempt one.
* **It never deletes anything.** The recorder's prune owns the card's budget; a
  second thing removing files behind its back is how the recording somebody
  asked for disappears between the asking and the fetching.
* Chunks are 2 MB, not the server's permitted 8 MB, because on a link measured
  in hundreds of kB/s an 8 MB piece is half a minute of exposure to a handover.
* It reads `RECORD_DIR` by importing `nodes/self_driving/config.py`, so
  `LIGMAX_AP_RECORD_DIR` moves the writer and the reader together.

Knobs: `LIGMAX_TRIP_UPLOAD=0` (off), `LIGMAX_TRIP_UPLOAD_CHUNK_MB`,
`LIGMAX_TRIP_UPLOAD_PERIOD_S`, `LIGMAX_TRIP_UPLOAD_QUIET_S`, `LIGMAX_BOAT_NAME`.
Progress is in `telemetry.trips`.

**One mismatch worth knowing, and it is not a bug yet.** The recorder will let a
single trip reach `RECORD_MAX_TRIP_MB` = **512 MB** on disk, and the server
refuses anything over **256 MB**. A file between the two can never be uploaded.
It cannot happen at Njord — a 15-minute attempt is 60 MB and the rules cap the
attempt — so nothing has been changed. The uploader detects it locally and says
so loudly rather than discovering it over 4G, and the file is still on the card
to be fetched by hand. If long bench recordings ever become a habit, set
`LIGMAX_AP_RECORD_MAX_TRIP_MB=256` and the two agree.

#### What to run on the boat — none of this has touched hardware

The checklist is **`docs/testing.md` §7h** in the umbrella checkout, where the
standing hardware checklists live. Five minutes on the dock, and it cannot move
the boat.

The two steps that are not box-ticking: **step 3**, resume after a dropped link,
which is the only part that cannot be checked any other way; and `sha256sum` on
both copies of every file, which is the real pass criterion — a recording that
looks complete and is not is worse than none at all.

By hand, as before, and still the fallback:

```sh
scp admin@ligmax-pi3.local:/home/admin/ligmax-trips/*.jsonl.gz .
python3 tools/review_trip.py <file>
python3 tools/review_trip.py <file> --html out.html
```

---

## 3. Tuning, on the water, in this order

Every one of these is an env var in `/etc/ligmax/node.env`; nothing needs a
commit. `nodes/self_driving/config.py` documents each with where its default
came from.

1. **Colour thresholds - do this first, in the day's light.**
   Measured on 6879 real returns on 2026-08-08: an indoor scene averaged
   (80, 48, 44) - a strong warm cast that made 46 of 49 clusters classify as
   RED. `MIN_SATURATION` was raised to 0.55 as a result, which turns that scene
   into honest UNKNOWNs.

   ~~That reasoning is now in doubt.~~ **Settled 2026-08-09, see §5.1:** the
   capture does *not* predate the Jetson's colour correction, so 0.55 was
   calibrated against roughly the distribution the boat still sends. This is an
   ordinary confirmation in the day's light, **not** the highest-value tuning
   action — that is §5.0, the front lidar's mounting angle.
   * Put a real red and a real green buoy in front of the boat, on the water,
     and check `telemetry.autopilot.sees`.
   * `LIGMAX_AP_WHITE_BALANCE=1` (grey-world, per sweep) is now **less likely to
     be the right answer**, not more: the Jetson already corrects, and grey-world
     on top of a correction is correcting twice. Try it only if a cast persists
     after the threshold has been re-derived.
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
5. ~~**`world.py` has no map memory.**~~ **DONE, 2026-08-09.** Marks the boat has
   properly studied are now kept indefinitely and survive a restart. What to
   know about it:
   * Promotion to permanent memory needs `TRACK_ESTABLISH_HITS` (12) sightings
     spread over `TRACK_ESTABLISH_SPAN_S` (2 s) at `TRACK_ESTABLISH_CONF` (0.80),
     and the type must be static. **The span is the test that matters**: a burst
     off one wave crest can reach twelve hits in 300 ms and cannot reach them
     across two seconds. A one-off stray return still dies after 6 s as before.
   * A remembered mark's position uncertainty grows at
     `TRACK_SIGMA_GROWTH_M_S` (0.05 m/s) to a `TRACK_SIGMA_MAX_M` (6 m) ceiling,
     and collapses to 0.35 m the instant it is measured again. Every clearance
     the boat uses adds it, so it automatically gives a half-remembered mark a
     wide berth and a visible one a tight one.
   * The survey persists to `/home/admin/.ligmax/survey.json` **in lat/lon**,
     not grid metres — the grid origin is cached in `/run`, which is tmpfs, so
     a reboot re-zeroes it and metres would silently describe somewhere else. A
     survey older than `SURVEY_MAX_AGE_S` (2 days) is discarded, because marks
     get re-laid between practice days.
   * Vessels are never remembered. The Otter is the one object guaranteed to
     have moved.
   * `emergency_stop_needed` and docking's berth-end check now ignore tracks
     older than 1 s, so a memory can never slam the brakes on for something that
     is not there.

   **Tune on the water:** `LIGMAX_AP_TRACK_SIGMA_MAX_M` is the honest guess at
   how far a Njord mark drifts on its mooring, and nobody has measured it. If
   the boat gives remembered marks a comically wide berth, that is the number.
6. **Nothing uses the gate structure of Task 2.** `perception.split_by_gap`
   already finds 5 m pairs (§9.2) - the same function the berth finder uses - but
   `buoys.py` treats each mark individually. Steering explicitly at gate
   midpoints would be tidier and more robust than obeying two lateral rules.
7. **`STATIONARY_SPEED_MS` is not verified against the tide.** "Stay stationary"
   is scored twice.

---

## 5. For the other repos

### `ligmax-edge` (the Jetson)

#### 5.0 The front lidar's mounting angle is known-wrong — **BLOCKER**

**`rig.json`, `lidar.yaw_deg`.** The file says so itself, at length:

> "STALE since the unit was remounted upside down (2026-08-08) … This -45 was
> measured for the OLD, right-side-up mount … **do not trust the front lidar's
> bearing (or its colouring, which projects through this same geometry) until it
> is remeasured.**"

`angle_dir` was flipped to `-1` on the same date and the file states that has
**not been run against hardware either**.

Nothing on the Pi can compensate. Everything downstream is affected: cluster
positions, the world model, the buoy rule's port/starboard decision, and now the
survey carried into attempt two. A mirrored world is the failure most likely to
survive a casual glance — the plot looks plausible and every mark is on the
wrong side.

**And it is worse than that, which nobody had written down.** `fusion.py` drops
every return no camera could see (`keep = cam_of >= 0`), and which camera a
return belongs to is decided by projecting it through the rig — *through this
same `yaw_deg`*. So a wrong yaw does not merely rotate the picture: **the ~34°
wedge that gets discarded is the wrong physical wedge**, and real obstacles dead
ahead can be thrown away at the sender and never reach the Pi at all. The Pi
cannot detect this — the sweep arrives looking complete, and `n + dropped` still
adds up. Until the yaw is remeasured, run the Jetson with **`--lidar-keep-unseen`**:
it keeps the uncoloured returns, so a mis-aimed rig costs colour rather than
obstacles.

`rig.json` gives the procedure:

```
put a target dead ahead, read its RAW reported_deg in
test/test_lidar_overlay.py (angle_zero_deg is 0, so it needs no correction),
and with angle_dir now -1,
    yaw_deg = reported_deg_dead_ahead
```

Then check the sign: dead ahead should read near 0° in the overlay tool and near
the top of the receiver's top-down plot.

While in there: `cam0` is **port**-facing (`yaw -75`) and `cam1` **starboard**
(`yaw +75`) — back to back, 150° apart, not both forward. The note says those
two yaws are a hand-measured figure split evenly and asks for verification with
`test/test_lidar_overlay.py --cam 0 --yaw N`. Same session, same rig.

**Do this before any scored run.** It is the on-water equivalent of §1.1.

#### 5.1 The Jetson already colour-corrects — the Pi assumed it did not

~~White-balance the RGB before it goes on the wire.~~ **Already done, and the
old advice was backwards.** `fusion.py::_correct` applies the OV5647 matrix
before sending, on top of the ISP chroma gain reported in the frame header's
`saturation` field (default 2.0, because JetPack ships no ISP tuning for this
sensor and its untouched output is about a third of normal chroma). The edge's
own docstring is explicit:

> "Display them as they arrive … a consumer that boosts saturation to compensate
> for raw sensor values is now correcting twice."

This repo believed the opposite, and `config.MIN_SATURATION = 0.55` was chosen
on that belief. Both `nodes/self_driving/config.py` and
`perception/classify.py` now say so at the value itself. It has deliberately
**not** been guessed downward: too high fails safe (everything reads UNKNOWN and
is avoided on both sides) while too low is a confident wrong-side pass, which is
the failure being scored.

~~**Someone should run `git log -- fusion.py` in `ligmax-edge`**~~ **Answered,
2026-08-09.** It did **not** predate the capture: `age_ms` first appears in
`ligmax-edge` at `1b70dc4` (2026-08-08 18:04) and the capture contains it. The
reasoning is written into `nodes/self_driving/config.py` and
`perception/classify.py` at the value itself. Nobody needs to redo this.

So 0.55 is calibrated against roughly the right distribution after all. **Do not
treat re-deriving it as the highest-value tuning action** — it is an ordinary
confirmation in the day's light: put a real red and a real green mark in front
of the boat and read `telemetry.autopilot.sees`.

#### 5.2 The rest

1. **Mask the front lidar's view of the boat — and note that nothing does it
   today.** This file used to say the Jetson filtered it and the Pi's
   `masks.mask_front` was a backstop defaulting to `none`. Checked against
   `ligmax-edge` on 2026-08-10: **neither side masks it.** Nothing in
   `fusion.py` or `sender.py` removes the front unit's view of the hull and
   `rig.json` carries no mask.

   The Pi asserted otherwise in three places, one of them inside `describe()`,
   which goes into telemetry **and into every trip header** — a false statement
   about which returns were filtered, sitting in the file somebody reads during
   a post-mortem. All three are corrected.

   The design intent stands: exactly one side should own this, because two
   places correcting one occlusion is how a rig gets corrected twice. Either
   implement it on the edge where the geometry lives, or set
   `LIGMAX_FRONT_MASK_MODE=box` on the Pi as the interim — but not both, and
   note that a mask set on the Pi is measured against a bearing §5.0 says is
   not trustworthy yet.
2. ~~`run.sh` still needs **`LIDAR=1`** by hand.~~ **Already the default** —
   `run.sh:78` is `if [ "${LIDAR:-1}" = "0" ]`, so only `LIDAR=0` turns it off.
   `repos.md` has been corrected.
3. ~~The lidar block sends `age_ms`, `dropped` and `stale` that the Pi's
   `edge_protocol.py` **still does not document**.~~ **It documents all three,
   plus `skew_ms` and `cam`.** The code is byte-identical to the edge's
   `protocol.py` — verified by diff, and worth keeping in the checklist, since
   it is the cheapest possible guard on the wire format. In the 2026-08-08
   capture **179 of 269 points were `stale`**, coloured from a frame outside the
   freshness window.

   ~~The useful follow-up is on the Pi: weight a cluster's colour vote by each
   point's `age_ms`.~~ **Done** — `perception/classify.py::age_weights`, with
   its constants pinned to the edge's own CLI defaults, and covered by
   `tests/test_autopilot.py`. Tuning it on the water is optional.

   Still true: `skew_ms` is a whole-sweep summary and no longer decides any
   individual point's colour, and `cam = -1` only appears with
   `--lidar-keep-unseen` — by default those returns are dropped, so `n + dropped`
   is the rotation size and `n` alone is not. The Pi is safe against the `cam`
   change (`scan.py` maps `cam < 0` to the uncoloured sentinel either way).
   **See §5.0 for why `--lidar-keep-unseen` is worth turning on right now.**

### `ligmax-server`

**Almost nothing.** §2.6 and §2.7 both landed, and §2.1–§2.5 were verified
against source on 2026-08-10. What is left is four small jobs, in order of
value:

1. Read `telemetry.autopilot_bridge` from the top level of `telemetry`, not from
   `telemetry.autopilot` — §2.5 item 1. It is the field that tells a dead
   autonomy node from a dead bus, and it currently renders never.
2. A warning state on `recording.free_mb` — §2.5 item 2.
3. `autopilot_goto` has no control anywhere — §2.1.
4. A `berth_width_m` field in the editor — §2.2. The 2 m berth is the tightest
   thing on the course and it is unreachable from the dashboard.

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

   For a **first pass down an unfamiliar course**, or the first run after
   changing a threshold, send `careful_on` first: the boat is then held to
   **1 knot** and can be walked alongside. `careful_off` releases it without
   interrupting the run. The panel shows `speed_ceiling_kn` so there is never a
   question about which is in force — and the boat is never above 5 kn either
   way.
6. Watch `mode` and `reason`. If `stuck` goes true, you have ~20 s before §8.2
   says take over.
7. After the run: `autopilot_stop`, then

   ```sh
   python3 tools/review_trip.py                  # summary of the last run
   python3 tools/review_trip.py --html /tmp/t.html
   ```

   Read the waypoint table and the `tracked` list **before** attempt two. That is
   what the recorder is for. Tracks marked `E` were established and are in the
   survey; `R` means they were restored from a previous run.

### Between the two attempts at the SAME task — new, and worth points

The boat now keeps the marks it surveyed. **Do not press `forget_world` between
attempt one and attempt two of the same task** — that is exactly the map you
want to keep. Attempt two starts knowing roughly where every gate is, at ±6 m,
tightening to ±0.35 m the moment the lidar sees each one again.

Check it loaded: **`telemetry.autopilot.perception.restored`** should be
non-zero at the start of attempt two. ~~`telemetry.autopilot.survey.marks`~~ is
**not** the field to look at — corrected 2026-08-10: it counts entries the boat
has *written*, so it legitimately reads 0 at the start of a run that restored
seven marks, and reading it as "nothing loaded" would send somebody hunting a
fault that is not there. The journal says it loudly and is the other check:

```
restored 7 surveyed mark(s) from /home/admin/.ligmax/survey.json -
each is good to about 6.0 m until the lidar sees it again
```

If a mark on the chart is wrong — a phantom off a wave, or something that has
since been moved — delete that one (§2.6) rather than clearing everything.

8. Between **different tasks**, on a different part of the course:
   `forget_world` (which now clears the survey too), then `set_plan` for the
   next one.

**The one thing that invalidates the survey** is the grid origin moving: a
reboot empties `/run/ligmax/grid-origin.json`, and the dashboard's
`recentre_origin` drops it deliberately. The survey is stored in lat/lon so it
survives both, and `world.py` rebuilds from it automatically — but if you press
`recentre_origin` mid-task you will see the live tracks vanish and the surveyed
ones reappear at their absolute positions. That is correct, not a fault.

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
* `nodes/io_manager/trip_upload.py` is new (2026-08-10) and is wired into
  `main.py`: one daemon thread, fed the armed/recording gates on every 1 Hz
  publish tick, reporting `telemetry.trips`, closed in the `finally`. It imports
  `RECORD_DIR` from `nodes/self_driving/config.py` — the one cross-node import in
  io_manager, and deliberate: a second copy of that path is how an uploader ends
  up watching an empty directory and truthfully reporting nothing to send.
* `nodes/io_manager/pixhalwk.py` (ride height) **is now wired into `main.py`**
  (committed 2026-08-09): a `RideHeight` object refreshed every loop
  tick, driven by the `set_ride_height` command, on **RC 14**. It is inert until
  an operator asks — until the first command the node does not write the channel
  at all. `stop()` holds 1500 rather than releasing, because releasing hands the
  channel back to a transmitter that may have it parked off centre, and
  `amas.lua` reads it as a velocity. The autonomy node still does not touch ride
  height at all. **Untested on hardware.**
* There is no `CLAUDE.md`; the docstrings are the documentation, which works
  because they are unusually thorough. Keep it that way.
