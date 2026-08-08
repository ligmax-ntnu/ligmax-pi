# Njord 2026 — what the boat is actually judged on

Everything here is from the official handbook at <https://njord.gitbook.io/2026>
(every page also serves plain Markdown by appending `.md` to its URL, which is
how this file was compiled). Read this before touching `nodes/self_driving/` —
most of the odd-looking decisions in there are a rule, not a preference.

**Dates: 10–14 August 2026, Trondheim.** GPS points and the day's schedule are
handed out **at 08:00 (GMT+2) on the competition day itself**, not before. No
team testing on the water from 08:30 until the end of the day. First run 09:00,
teams present from 08:30.

That deadline shapes the whole design: **the course coordinates arrive ~1 hour
before the first run**, so the route has to be something an operator types or
clicks into the dashboard on the morning, not something compiled into the boat.
That is why the plan is data (`plan.py`) and not code.

---

## 1. The runs

Four scored categories: **Maneuvering**, **Path Finding**, **Collision
Avoidance**, **Docking**. They are laid out on one course over GPS points 1–9.

Per task: **15 minutes, two attempts per subtask.** Then 5 min to explain to the
jury and 10 min for them to deliberate — a 30 minute slot.

The timer **starts when the ASV goes into autonomous mode** and stops when the
challenge is done. Task 3's timer specifically stops at GPS point 9.

### Task 1 — Maneuvering and Path Finding (GPS 1 → 4)

Two parts, run as one attempt.

**Part 1, blind GNSS following.** The boat is driven to GPS point 1 *under
remote control*, then switched to autonomous. The course is GPS points 1, 2, 3,
4 plus **8–15 intermediate waypoints** (labelled 1.1–1.10 and 3.1–3.3). The boat
must pass all of them. The attempt starts when the ASV passes the first
waypoint. If part 1 cannot be finished, the team may restart from GPS point 3.

**Part 2, vision.** "Halfway through, combine camera vision and GNSS
navigation." Cardinal marks appear and determine which side to pass:

* **Cardinal East → pass on the East side of the mark.**
* **Cardinal West → pass on the West side of the mark.**
* (and by the same rule North → pass north of it, South → pass south of it.)

There are **no red/green gate pairs in this task** — the buoys are standalone
and just have to be avoided. Contact with buoys must be avoided.

The attempt finishes when the final cardinal mark is passed. **The boat must
then stop at GPS point 4 and stay stationary there.** Stationary is scored, so
station-keeping is a real behaviour, not an afterthought.

### Task 2 — Collision Avoidance (GPS 5 → 6)

Autonomous from GPS point 5, through a series of **gates** (a red/green buoy
pair, **5 m apart**, with **20–80 m between gates**) to GPS point 6, while the
**Njord Otter** marker vessel tries to get in the way.

* Set speed for the task: **2 knots** (≈1.03 m/s). The boat must **immediately
  accelerate to the set speed** at the start of an attempt.
* The Otter approaches at **2.5 knots** (≈1.29 m/s) on a bearing between
  **−100° and +100°** relative to the boat, either head-on or crossing.
* **2 attempts per collision situation.**
* **COLREG compliance is explicitly evaluated.**

The handbook warns that "the form and/or colour of the Otter could vary and the
ASV should still be able to detect the Otter" — so it must not be recognised by
colour alone. Its stated hull size is 2.0 × 1.08 × 1.0 m, which is far larger
than any buoy and is the cue the lidar can be trusted on.

The COLREG cases that can actually arise here, and what we do:

| Situation | Rule | Our action |
|---|---|---|
| Head-on (Otter within ±15° of dead ahead, reciprocal course) | Rule 14 | Both turn **starboard**, pass port-to-port |
| Otter crossing from our **starboard** | Rule 15 — we give way | Turn **starboard**, pass **astern** of it |
| Otter crossing from our **port** | Rule 17 — we stand on | Hold course and speed, but keep the emergency override armed |
| Anything too close | Rule 8 | Substantial, early, obvious alteration — never a series of small ones |

Rule 8's "readily apparent to another vessel observing visually" is the reason
avoidance turns are one large committed alteration rather than a smooth spline:
a jury watching from the dock has to *see* the decision.

### Task 3 — Docking (GPS 7 → 8 → 9)

Starts about **10 m from the dock** at GPS point 7. **Two attempts for normal
docking and two for parallel docking**; moving on to the next part forfeits any
remaining tries on the previous one. Timer stops at GPS point 9.

* **3.1 Normal (bow-in) docking** — berth is **2 m long × 2 m wide**. Enter,
  **hold for 10 seconds**, then **reverse out**.
* **3.2 Parallel docking** — berth is **2 m long × 4 m wide**. Come alongside,
  **stay stationary parallel to the dock for 5 seconds**, then move forward
  toward the next GPS point.

**Three AR tags** per task mark the assigned berth, **18 × 18 cm**. They are
**optional — a team that docks without them gets bonus points.** The handbook
never states the tag family or the IDs, so a run must not depend on them. We
find the berth from the lidar instead: a 2 m gap between two returns-walls is a
geometric fact the C1 measures to ±3 cm, and it is the same measurement whether
a tag is visible, wet, or facing the wrong way.

A 2 m berth for a boat this size means the useful margin is tens of
centimetres — this is the one task where the lidar's accuracy, not the GNSS's,
is what decides the outcome.

---

## 2. Course furniture, with the numbers

### Buoys

| | Colour | Size |
|---|---|---|
| Navigation red | RAL 3001 signal red | 40 cm sphere, 40 cm dia, 20 cm above water; 14 cm × 40 cm cylinder topmark; **60 cm total above water** |
| Navigation green | Neon green | same |

Direction of buoyage is **seaward = North**. So, sailing north:

* **Red → keep on the boat's PORT side.**
* **Green → keep on the boat's STARBOARD side.**

Sailing *south* (returning) the sense inverts. This is why the plan format
carries an explicit `channel_bearing` per leg instead of assuming north — see
`plan.py`. Getting it backwards is the single most expensive mistake available
in Task 2 and it is invisible until you are already through the gate.

### Cardinal marks

Black & yellow (RAL 9005 / RAL 1003), 40 cm buoy 20 cm above water + 14 cm
cylinder, 60 cm total. Stated heights differ per type (N/S 30 cm, E/W 20 cm
from waterline).

**Rule: pass on the named side of the mark.** North cardinal → you go north of
it. East cardinal → you go east of it.

The lidar can see that a cardinal mark exists and it will read yellow-and-black,
but **no lidar can tell a north cardinal from a south one** — that is the
topmark's two black cones, and it is a camera job. The Jetson runs a second
stage classifier and reports `card` ("north"/"south"/"east"/"west") with
`card_conf`. Since that model is not trusted, `perception/classify.py`
accumulates votes across many frames per track and refuses to commit until the
evidence agrees; `behaviours/buoys.py` falls back to the planned route's own
side preference when it never does.

### AR tags

18 × 18 cm, placed around the berth. Optional, and **not using them scores
bonus points**. Family/IDs unpublished.

### The Otter

2.0 × 1.08 × 1.0 m marker vessel. Colour and form may vary between runs. Moves
at 2.5 kn. It is an obstacle in more tasks than just Task 2.

---

## 3. What the GUI must show (scored separately)

From §11.4. A jury member must understand the display **without explanation**.

Required:

* Camera feed and/or lidar
* Latitude and longitude
* Heading
* **Course over ground, with a trail, compared against the ideal GNSS route**
* Speed over ground
* Battery percentage
* Status indicator: autonomous / remote control / standby / out of control

Nice to have: distance to next waypoint, battery Wh remaining.

And the part that matters for this repo: the GUI must show **decision-making
transparency** — how a detected object changed the plan, why the heading or
speed changed, and what the boat is searching for (e.g. an AR tag). That is a
hard requirement on the autonomy node, not on the web front end: the boat has
to *publish its reasoning*, continuously, in words a non-engineer reads. Every
behaviour in `nodes/self_driving/behaviours/` therefore returns a plain-English
`reason` on every single tick, and it is uploaded as
`telemetry.autopilot.reason`.

`ligmax-pi` already covers most of the required list through
`nodes/io_manager/navigation.py` and `status.py`.

---

## 4. Safety requirements that constrain the software

From §7.3:

* A **physical kill switch** on board that disengages all motorised parts, plus
  a **remotely controllable kill switch with the same function**. → covered by
  `nodes/io_manager/emergency_stop.py` (relay, fail-safe active-high) and the
  dashboard's `estop` command.
* **The vessel must automatically stop autonomous movement after 60 seconds
  without radio contact.** → this is a hard requirement on the autonomy node.
  See `LOSS_OF_COMMS_STOP_S` in `nodes/self_driving/config.py`; the boat holds
  station and disarms rather than continuing a plan it can no longer be
  recalled from. **We use 10 s, not 60 s**, because 60 s at 2 kn is 10 m of
  uncommanded travel and the rule is a ceiling, not a target.
* Firebox rated 4 minutes, waterproofing, sharp parts covered — hardware.

From §8.2, the rule that shapes the recovery path:

> The ASV gets a **20 second autonomous search window** if it has a problem.
> After 20 seconds the team takes control by remote and returns the vessel
> **behind the last successfully passed buoy pair / waypoint**, then re-enters.

So a lost behaviour must (a) *say* it is lost, immediately and visibly, and (b)
be interruptible at any instant by the operator taking manual control, and (c)
remember the last waypoint it legitimately passed so the re-entry point is
known. `plan.py` keeps `last_passed_index` for exactly this and it survives a
node restart.

---

## 5. Scoring mechanics worth knowing

* Categories per task: safety, movement, communication, the task-specific skill,
  completion — **minus deductions for breaking autonomy and for collisions.**
* Bonus points for a particularly impressive/unique design, and for an elegant
  successful **first** attempt.
* Docking: bonus for **not** using AR tags.
* A **time-slot multiplier**, 1.00× for slot 1 down to 0.91× for slot 10,
  compensating later slots.
* A **task-time multiplier**, 1.00× for the fastest team down to 0.91× for the
  tenth.

The two multipliers are each worth at most 9 %, while a broken autonomy run
costs a deduction plus the completion points. **Finishing slowly beats being
taken over.** Every speed limit in `config.py` is set from that observation.

---

## 6. Consequences for this repo

1. **The route is data delivered on the morning.** No coordinate is ever
   compiled in. `plan.py` accepts a plan over the existing command channel and
   persists it, so a node restart at 08:55 does not lose the course.
2. **Waypoints carry roles**, because the same GPS point means different things
   in different tasks — blind transit, buoy-rules transit, watch-for-a-boat
   transit, dock here, stop here.
3. **The lidar is the trusted sensor.** The front unit is coloured by the
   Jetson's cameras, so a cluster of returns carries both a measured geometry
   (±3 cm) and a colour. The colour classification is the *only* thing the
   camera is trusted for, and even then by vote, not by one frame.
4. **The boat must narrate.** §11.4 makes the reasoning part of the score.
5. **Station-keeping is a scored behaviour**, twice (stop at GPS 4, hold in the
   berth for 10 s / alongside for 5 s).
6. **Reverse is a scored behaviour** — normal docking requires reversing out.
7. **Every run must be recorded**, because there are two attempts and 15
   minutes, and the only way to fix attempt 2 is to know exactly what attempt 1
   did. See `recorder.py` and `tools/review_trip.py`.

## Sources

* [Njord 2026 handbook](https://njord.gitbook.io/2026)
* [2.1 The Physical Challenge](https://njord.gitbook.io/2026/2-njord-challenge-2026/2.1-the-physical-challenge)
* [8.2 General Task Rules](https://njord.gitbook.io/2026/8-general-task-info/8.2-general-task-rules)
* [8.3 Competition Setup](https://njord.gitbook.io/2026/8-general-task-info/8.3-competition-setup)
* [9.1 Maneuvering and Path Finding](https://njord.gitbook.io/2026/9-task-descriptions/9.1-maneuvering-and-path-finding)
* [9.2 Collision Avoidance](https://njord.gitbook.io/2026/9-task-descriptions/9.2-collision-avoidance)
* [9.3 Docking](https://njord.gitbook.io/2026/9-task-descriptions/9.3-docking)
* [10.2 Buoys](https://njord.gitbook.io/2026/10-course-components/10.2-buoys)
* [10.3 Cardinal Marks](https://njord.gitbook.io/2026/10-course-components/10.3-cardinal-marks)
* [10.4 AR-tags](https://njord.gitbook.io/2026/10-course-components/10.4-ar-tags)
* [10.5 Otter](https://njord.gitbook.io/2026/10-course-components/10.5-otter)
* [7.3 Safety Requirements](https://njord.gitbook.io/2026/7-technical-requirements/7.3-safety-requirements)
* [11.4 Providing of Data/UI](https://njord.gitbook.io/2026/11-evaluation/11.4-providing-of-gui)
* [11.6 General Task Evaluation](https://njord.gitbook.io/2026/11-evaluation/11.6-general-task-evaluation)
