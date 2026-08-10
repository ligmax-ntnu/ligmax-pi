# The morning's courses, as the boat eats them

The handout arrives at 08:00 and the first run is at 09:00, so nothing in here is
compiled into the boat — these are `plan.py` payloads, uploaded over the command
channel with `set_plan` and persisted to `/home/admin/.ligmax/plan.json`.

| file | what it is |
|---|---|
| `GPS points for tasks.xlsx` | the handout itself, kept for provenance. Two sheets: **Monday docking** (GPS 7–12) and **Monday Maneuvering and path fin** (GPS 1–4 plus the intermediates). |
| `task1.json` | Task 1, GPS 1 → 4. Used for **both** attempts — the slow one and the fast one differ by run profile, not by file. |
| `task3-docking.json` | Task 3, GPS 7 → 9. The roles are a **guess** and must be confirmed before it is run. |

**Read the spreadsheet's first column with suspicion.** Excel turned the
intermediate waypoint labels into dates: `1.1` is stored as `46023`, `3.2` as
`46084`. The order is intact and the coordinates are untouched, but the names in
the sheet are wrong and the JSON is the corrected copy.

## Task 1, measured

126 m of course in 13 waypoints, laid inside a box 33 m east-west by 43 m
north-south. Leg lengths and the turn at the far end of each:

| leg | length | bearing | turn onto the next leg |
|---|---|---|---|
| 1 → 1.1 | 5.0 m | 266° | 104° |
| 1.1 → 1.2 | 7.2 m | 162° | 13° |
| 1.2 → 1.3 | 4.4 m | 175° | 63° |
| 1.3 → 1.4 | 5.7 m | 237° | 33° |
| 1.4 → 1.5 | 10.5 m | 270° | 87° |
| 1.5 → 2 | 17.6 m | 177° | 49° |
| 2 → 3 | 17.6 m | 128° | **102°** |
| 3 → 3.1 | 9.7 m | 26° | 54° |
| 3.1 → 3.2 | 11.2 m | 332° | **123°** |
| 3.2 → 3.3 | 16.1 m | 96° | **115°** |
| 3.3 → 3.4 | 10.5 m | 341° | 57° |
| 3.4 → 4 | 10.6 m | 38° | — (stop) |

Three things fall out of that table and they shape how the boat is driven:

**It is a slalom, not a course.** Five of the twelve corners are over 85° and
three are over 100°, and they come on legs of 10–17 m. A turn radius is
`speed / yaw rate`; at 5 knots that is metres, and a 120° turn at 5 knots eats
most of a 10 m leg and cuts the corner by more than the 3 m acceptance radius —
i.e. the boat would miss the waypoint it is being scored on passing. This is why
`behaviours/base.py` limits speed by the geometry ahead rather than by a number
in the plan: the straights get the knots, the corners do not.

**`channel_bearing` is 27 on the buoy legs, not 0.** Direction of buoyage is
seaward = north (NJORD §10.2), which is the plan-level default here. But leg
3.2 → 3.3 runs at 96°, and `_with_the_buoyage` counts anything within 90° of the
channel as running with it — so at the default that one leg alone would flip
red and green and the boat would pass every mark on it on the wrong side, having
been right about all the others. The boat's actual progress from GPS 3 to GPS 4
is 40 m on a bearing of 27°, so 27 is what the buoy legs carry. It is the case
`plan.py`'s per-waypoint override was written for, and it is the single most
expensive field in this file to get wrong.

**Part 2 starts at GPS 3.** The rules let a team that cannot finish part 1
restart from there (NJORD §9.1), which is why waypoint `3` is the first one with
role `buoys`. `autopilot_goto {"index": 7}` jumps straight to it.

## Uploading one

```bash
# from the dashboard, or straight onto the command channel:
set_plan  {"plan": <the contents of task1.json>}
run_profile {"profile": "survey"}     # attempt 1: 1 kn, look at everything
autopilot_start {"label": "task1-attempt1"}
```

Then for the second attempt, with attempt one's map already on disk:

```bash
run_profile {"profile": "fast"}       # up to 5 kn where the geometry allows
autopilot_goto {"index": 0}
autopilot_start {"label": "task1-attempt2"}
```

See `../RUNBOOK.md` for what to check between the two.
