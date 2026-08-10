# Task 1 run day — two attempts, two different runs

The strategy this is built around: **attempt one is slow and surveys the course,
attempt two is fast and is driven off attempt one's map.** NJORD gives two tries
at each subtask (§8.2) and the marks do not move between them, so running the
same pass twice throws one of them away.

The course is in `plans/task1.json` and measured in `plans/README.md`. Read that
table before the first run — it is a slalom, and the three corners over 100
degrees are what the whole pacing story below exists for.

---

## 0. Before the boat goes in — the one measurement that matters

Everything the fast attempt does about corners rests on one number that has
never been measured on this hull:

```
LIGMAX_AP_TURN_LATERAL_ACCEL      default 0.8 m/s²
```

How hard the boat can turn. Measure it, do not trust the default:

1. Open water, GUIDED off, full lock, hold a steady turn at roughly the fast
   profile's speed (~2 m/s).
2. Time one full revolution. `A = speed × 2π / period`.
   A 2 m/s boat circling in 20 s is `2 × 6.28 / 20` = **0.63 m/s²**.
3. Put the answer in `/etc/ligmax/node.env` and restart the node.

Guessing high is the expensive direction — the boat plans a corner it cannot
make and sweeps wide of the waypoint it is scored on passing. In simulation, a
hull 25 % weaker than the configured figure still finished the course when the
boat paced itself and **did not finish at all** when it did not.

If there is no time to measure it, set it to **0.5** and accept a slower fast
attempt. A slow finish beats a missed waypoint: the task-time multiplier is worth
at most 9 %, a missed waypoint is worth the completion points.

Also worth a minute each:

* `LIGMAX_AP_MIN_SATURATION` (0.55) — the colour threshold. Point the boat at a
  red and a green mark in the day's light and read `telemetry.autopilot.sees`. If
  marks come back UNKNOWN, lower it; if ordinary warm surfaces come back RED,
  raise it. Too high fails safe, too low does not.
* Nothing about the direction of buoyage. It is hardcoded to true north
  (`plan.BUOYAGE_BEARING_DEG`, the venue's own definition of its entrance) and no
  field in a plan or on the dashboard can move it — so leave *Direction of
  buoyage* wherever it sits. This used to be the most expensive number on the
  morning, and it is now not a number. See `plans/README.md` for what changed and
  why the inversion window is 135° rather than 90°. **Check the boat is running
  that code**: on an un-updated boat the field is live again, and the old default
  of 0 flips red and green on two of the five buoy legs.

---

## 1. Attempt one — survey

On the dashboard: load the course, press **Survey (1 kn)**, press Start. The
equivalent commands, which is what the chips send:

```
set_plan        {"plan": <contents of plans/task1.json>}
run_profile     {"profile": "survey"}
autopilot_start {"label": "task1-attempt1"}
```

The three profile chips and the alternation toggle are new
(`ligmax-server/web/js/autopilot.js`), so **the ground station needs updating
before the first run** or the only speed control on the panel is the old Careful
mode toggle — which still works and is the same state as `survey`, but there is
then no way to select `fast`.

1 knot, everything watched. Expect roughly **3½–4 minutes** to GPS 4 (220 s in
simulation, and the real thing will be slower).

**What to watch, in this order:**

| watch | where | what is wrong if it is not there |
|---|---|---|
| `profile: survey`, `ceiling_kn: 1.0` | commander block | the wrong attempt is being run |
| marks appearing with `source: front_lidar` | obstacle layer | the lidar or the colour thresholds |
| cardinals going from `cardinal (side unknown)` to `east cardinal (4 votes)` | mark label | the camera never commits — see §3 |
| `survey` block's `marks` count climbing | telemetry | nothing is being established, so attempt two gains nothing |

**The survey is the product of this run**, not the score. A mark has to be seen
on 12 sweeps spanning 2 s to be established, which is easy at 1 kn and not at 5.
If a mark never establishes, attempt two is blind to it.

Then, and this matters:

```
autopilot_stop  {"why": "attempt one done"}
```

`autopilot_stop` writes the survey to `/home/admin/.ligmax/survey.json`. Pulling
the battery instead loses up to 10 seconds of it (`SURVEY_SAVE_PERIOD_S`).

## 2. Between the attempts — 60 seconds of checks

```bash
python3 -c "import json;d=json.load(open('/home/admin/.ligmax/survey.json'));\
print(len(d['marks']));[print(m['kind'], m['cardinal_committed'], round(m['confidence'],2)) for m in d['marks']]"
```

* **Every mark on the course present?** Four expected on the buoy legs. A missing
  one means attempt two will not know about it until the lidar picks it up, which
  at speed it may not.
* **`cardinal_committed` set on the cardinals?** If it is `null`, the camera never
  made up its mind, and attempt two faces the same coin flip. That is when §3
  becomes relevant.
* **Anything present that is not a mark?** A phantom in the file is steered around
  for the whole of attempt two. Delete it from the chart (`forget_object`) or
  clear the file and go in blind — a confident detour round nothing is worse than
  no memory.

A restored mark carries `SURVEY_SIGMA_M` (1.2 m) of position uncertainty rather
than the 6 m a mark that merely drifted out of view gets, because it was measured
deliberately and has been sitting on its mooring since. Every clearance adds that
figure, so a remembered mark claims about 3.2 m of water and two of them still
leave a gap on a 10 m leg.

## 3. The alternation prior — only if the camera failed

**Off by default and it should stay off if the cardinals committed.** Switch it on
only when the survey shows `cardinal_committed: null`:

```
alternation {"on": true}
```

What it then does: an unresolved cardinal is passed on the **opposite side to the
mark before it in the run**, because consecutive marks in a channel alternate the
side you pass them — a mark that pushes you the same way as the last one
constrains nothing, so nobody lays one there. It reads the sides off marks the
boat established for itself, it names no course and no task, and it never
overrides a committed camera vote: where the two disagree the camera wins and the
panel says so.

It will refuse to answer when the geometry does not allow one — on a leg running
north, east and west are unambiguous and north and south say nothing. Watch for
`alternation:` in the behaviour block; it always says either what it concluded or
why it declined.

Without it, an unresolved cardinal makes the boat hold the planned line at
0.6 m/s and say so loudly, which is the NJORD §8.2 twenty-second window arriving
early enough to use.

## 4. Attempt two — fast

Press **Fast (up to 5 kn)**, rewind to the start, press Start:

```
run_profile     {"profile": "fast"}
autopilot_goto  {"index": 0}
autopilot_start {"label": "task1-attempt2"}
```

Up to the 5 knot limit, paced by the geometry. Expect roughly **70–110 seconds**
to GPS 4 against the survey run's 220.

**It will not hold 5 knots, and that is correct.** A flat 5 kn over 126 m would be
49 s, but three of the corners on this course are over 100 degrees on legs of
10–17 m, and a turn radius is set by speed: the straights get the knots and the
corners do not. Two limits do that, both of them narrated on the panel:

* `123 deg turn at 3.2 in 4 m - easing to 1.5 m/s` — the corner ahead, read off
  the plan. Full pace down the leg, brake late.
* `115 deg off the aim - holding 1.5 m/s while it turns onto it` — the boat is
  already crosswise, which is what coming out of a hard corner looks like. This
  is the one that saves the waypoint *after* a tight turn.

Marks also get more room at speed: about 2.6 m on top of the static 2 m at full
pace, because clearance is a time budget wearing metres and speed spends it. This
applies **only** in the fast profile — Task 2's gates are 5 m wide and a speed
term there would make the boat refuse a gate it is meant to drive through.

**Abort criteria — drop to survey mid-run rather than lose the attempt:**

```
run_profile {"profile": "survey"}
```

Takes effect on the next tick and does not interrupt the run. Do it if:

* the trace is visibly wide of a corner — you will see it before the boat does;
* `stuck: true` appears, i.e. no progress for 12 s. There are about 8 s left of
  §8.2's twenty before the crew must take over;
* a mark is drawn somewhere the eye says it is not.

**If part one goes wrong**, the rules allow a restart from GPS 3 (NJORD §9.1),
which is waypoint index 7:

```
autopilot_goto {"index": 7}
```

## 5. At GPS 4

Waypoint `4` has role `hold` with `hold_s: 0` — arrive and stay stationary until
told, which is what §9.1 scores. The plan deliberately never reports "finished";
the boat holds station on the point. Stop it by hand when the jury has seen it:

```
autopilot_stop {"why": "GPS 4 held"}
```

---

## Command reference

| command | args | what it does |
|---|---|---|
| `set_plan` | `{"plan": {...}}` | upload a course. Refused with a readable reason rather than partly accepted |
| `run_profile` | `{"profile": "survey"\|"normal"\|"fast"}` | which attempt this is. Mid-run is fine |
| `careful_on` / `careful_off` | — | the older name for `survey` / `normal`. Same mechanism, not a rival to it |
| `alternation` | `{"on": true\|false}` | the cardinal prior. Off by default |
| `autopilot_start` | `{"label": "..."}` | engage, and start recording under that label |
| `autopilot_stop` | `{"why": "..."}` | stand down, close the recording, **write the survey** |
| `autopilot_pause` / `autopilot_resume` | — | hold station without losing the plan cursor |
| `autopilot_goto` | `{"index": n}` | jump to a waypoint. `7` is GPS 3, the part-two restart |
| `autopilot_back` | — | one waypoint back. §8.2's re-entry point |
| `autopilot_skip` | — | "that one is done" |
| `forget_object` | `{"id": n}` | delete one phantom. The spot is refused new tracks for 30 s |
| `forget_world` | — | clear the live tracks **and** the stored survey |

## The three profiles

| | ceiling | cruise | around marks | extra room per m/s |
|---|---|---|---|---|
| `survey` | 1.0 kn | 0.51 m/s | 0.51 m/s | — |
| `normal` | 3.1 kn | 1.20 m/s | 0.80 m/s | — |
| `fast` | **5.0 kn** | 2.20 m/s | 1.60 m/s | 1.0 m, capped at 3.0 m |

The 5 knot limit is enforced in five independent places and none of them is
overridable from the environment — see the block in `nodes/self_driving/config.py`.
A profile spends the boat's own margin up to that limit; nothing spends past it.

## After the day

`tools/review_trip.py` over the files in `/home/admin/ligmax-trips`. The two
questions worth answering while the memory is fresh:

* **how far had the marks moved between the attempts?** That is what
  `SURVEY_SIGMA_M` should really be.
* **what was the boat's actual lateral acceleration in the corners?** `speed` and
  the rate of change of `heading` are both in the file, and their product is the
  number §0 asked you to guess.
