#!/usr/bin/env python3
"""Read back a recorded run. `python3 tools/review_trip.py [file] [--html out.html]`

Njord gives two attempts and fifteen minutes per subtask (§8.2). The only way to
make attempt two better than attempt one is to know what attempt one actually
did - which mark the boat saw, which side it decided to pass, what the cardinal
vote was at the moment it committed, how far off the ideal line it drifted. On a
cold dock, between attempts, nobody remembers, and the dashboard's live view has
long since scrolled past.

So this reads a `.jsonl.gz` from `recorder.py` and answers those questions.

    python3 tools/review_trip.py                     # the newest run
    python3 tools/review_trip.py <file>              # a specific one
    python3 tools/review_trip.py --list              # what is on the card
    python3 tools/review_trip.py --html trip.html    # a self-contained page
    python3 tools/review_trip.py --events            # just the decisions

The text summary is the one to reach for on the dock: it fits on a phone screen
over SSH and it needs nothing installed. `--html` writes a single file with the
track, the ideal route and the obstacles plotted, for looking at afterwards on a
real screen - inline SVG, no libraries, so it opens anywhere.
"""

from __future__ import annotations

import argparse
import glob
import gzip
import json
import math
import os
import sys

DEFAULT_DIR = os.environ.get("LIGMAX_AP_RECORD_DIR", "/home/admin/ligmax-trips")


# ------------------------------------------------------------------ loading

def newest(directory=DEFAULT_DIR):
    files = sorted(glob.glob(os.path.join(directory, "*.jsonl.gz")))
    return files[-1] if files else None


def load(path):
    """`(header, rows, footer)`. Tolerates a file that was cut off mid-write.

    That tolerance is the point of the format: the recording most worth reading
    is disproportionately likely to be the one whose run ended badly, and a
    half-written last line must not cost the other nine thousand.
    """
    header, footer, rows = {}, {}, []
    opener = gzip.open if path.endswith(".gz") else open
    with opener(path, "rt", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except ValueError:
                continue  # truncated final line
            kind = row.get("kind")
            if kind == "header":
                header = row
            elif kind == "footer":
                footer = row
            else:
                rows.append(row)
    return header, rows, footer


# ------------------------------------------------------------------ summary

def summarise(header, rows, footer):
    out = []
    add = out.append

    add(f"trip      {header.get('label', '?')}  {header.get('local_time', '')}")
    add(f"build     {header.get('git') or 'unknown commit'}")
    samples = [r for r in rows if "boat" in r]
    events = [r for r in rows if r.get("kind") == "event"]
    if not samples:
        add("no samples - the run never got a state frame")
        return "\n".join(out)

    duration = samples[-1]["t"] - samples[0]["t"]
    add(f"duration  {duration:.0f} s over {len(samples)} samples, {len(events)} events")

    # Distance run, from the grid positions.
    track = [tuple(r["boat"]["position"]) for r in samples if r["boat"].get("position")]
    if len(track) > 1:
        run = sum(
            math.dist(track[i], track[i + 1]) for i in range(len(track) - 1)
        )
        add(f"distance  {run:.0f} m")
    speeds = [r["boat"].get("sog") for r in samples if r["boat"].get("sog") is not None]
    if speeds:
        add(f"speed     mean {sum(speeds) / len(speeds):.2f} m/s, max {max(speeds):.2f} m/s")

    fixes = _tally(r["boat"].get("fix") for r in samples)
    if fixes:
        add("fix       " + ", ".join(f"{k} {v}" for k, v in fixes))
    modes = _tally(r["boat"].get("mode") for r in samples)
    if modes:
        add("ap mode   " + ", ".join(f"{k} {v}" for k, v in modes))

    # What the autonomy did, waypoint by waypoint. This is the part worth
    # reading before attempt two.
    add("")
    add("waypoints")
    last = None
    for row in samples:
        autopilot = row.get("autopilot") or {}
        plan = autopilot.get("plan") or {}
        key = (plan.get("index"), plan.get("current"), autopilot.get("behaviour"))
        if key == last or plan.get("current") is None:
            continue
        last = key
        add(
            f"  +{row['t'] - samples[0]['t']:6.0f}s  {plan.get('current'):<10} "
            f"{str(plan.get('role')):<14} {str(autopilot.get('behaviour') or ''):<14} "
            f"{autopilot.get('reason', '')[:70]}"
        )

    blocked = _tally(
        (r.get("autopilot") or {}).get("blocked") for r in samples
    )
    if blocked:
        add("")
        add("blocked   (ticks the boat refused to drive)")
        for reason, count in blocked:
            add(f"  {count:5d}x  {reason[:80]}")

    stuck = [r for r in samples if (r.get("autopilot") or {}).get("stuck")]
    if stuck:
        add("")
        add(
            f"STUCK for {len(stuck)} sample(s), first at "
            f"+{stuck[0]['t'] - samples[0]['t']:.0f}s - NJORD 8.2's 20 s window"
        )

    # Everything the boat ever tracked, so a mark it never saw is visible as an
    # absence rather than having to be inferred.
    seen = {}
    for row in samples:
        for track_row in row.get("tracks") or []:
            seen.setdefault(track_row["id"], track_row)
    if seen:
        add("")
        add(f"tracked   {len(seen)} object(s)")
        for track_row in sorted(seen.values(), key=lambda t: t["id"])[:25]:
            position = track_row.get("position") or [0, 0]
            add(
                f"  #{track_row['id']:<4} {track_row.get('label', '?'):<20} "
                f"({position[0]:7.1f}, {position[1]:7.1f}) m  "
                f"conf {track_row.get('confidence', 0):.2f}  "
                f"{str(track_row.get('why') or '')[:50]}"
            )

    if events:
        add("")
        add("events")
        for row in events[:60]:
            detail = {k: v for k, v in row.items() if k not in ("kind", "t", "event")}
            add(
                f"  +{row['t'] - samples[0]['t']:6.0f}s  {row['event']:<16} "
                f"{json.dumps(detail)[:90]}"
            )

    if footer:
        add("")
        add(f"ended     {footer.get('why')} after {footer.get('duration_s')} s")
    else:
        add("")
        add("ended     NO FOOTER - the recording was cut off (crash, or power loss)")
    return "\n".join(out)


def _tally(values):
    counts = {}
    for value in values:
        if value is None:
            continue
        counts[str(value)] = counts.get(str(value), 0) + 1
    return sorted(counts.items(), key=lambda item: item[1], reverse=True)


# --------------------------------------------------------------------- HTML

def to_html(header, rows, footer):
    """A single self-contained page: the track, the plan, and the obstacles.

    Inline SVG and no libraries, because this gets opened on whatever laptop is
    in the tent and a page that needs a CDN is a page that does not load there.
    """
    samples = [r for r in rows if (r.get("boat") or {}).get("position")]
    track = [tuple(r["boat"]["position"]) for r in samples]
    plan_points = []
    for row in rows:
        plan = ((row.get("autopilot") or {}).get("plan")) or {}
        if plan:
            break
    stored = (header.get("plan") or {}).get("waypoints") or []

    marks = {}
    for row in rows:
        for entry in row.get("tracks") or []:
            marks[entry["id"]] = entry

    points = list(track) + [tuple(m["position"]) for m in marks.values() if m.get("position")]
    if not points:
        return "<p>nothing to plot - the run never got a position</p>"

    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    pad = 8.0
    x0, x1 = min(xs) - pad, max(xs) + pad
    y0, y1 = min(ys) - pad, max(ys) + pad
    width, height = 900.0, 900.0 * max(0.4, (y1 - y0) / max(1e-6, x1 - x0))

    def sx(x):
        return (x - x0) / max(1e-6, x1 - x0) * width

    def sy(y):
        return height - (y - y0) / max(1e-6, y1 - y0) * height

    parts = [
        "<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 "
        f"{width:.0f} {height:.0f}' style='width:100%;height:auto;"
        "background:#0b1015'>"
    ]
    if track:
        path = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in track)
        parts.append(
            f"<polyline points='{path}' fill='none' stroke='#4ea8ff' "
            "stroke-width='2.5'/>"
        )
    colours = {
        1: "#e4443a", 2: "#3ad46b", 3: "#f0c419", 4: "#f0c419",
        5: "#f0c419", 6: "#f0c419", 10: "#f0c419", 7: "#ff8c1a",
        8: "#9aa5b1", 0: "#6b7280", 9: "#c471ed",
    }
    for entry in marks.values():
        position = entry.get("position")
        if not position:
            continue
        colour = colours.get(entry.get("type", 0), "#6b7280")
        parts.append(
            f"<circle cx='{sx(position[0]):.1f}' cy='{sy(position[1]):.1f}' r='6' "
            f"fill='{colour}' opacity='0.85'><title>#{entry['id']} "
            f"{entry.get('label')} conf {entry.get('confidence')}</title></circle>"
        )
    parts.append("</svg>")

    summary = summarise(header, rows, footer)
    return (
        "<title>ligmax trip "
        f"{header.get('label', '')} {header.get('local_time', '')}</title>"
        "<style>body{background:#0b1015;color:#dbe4ee;font:14px/1.5 ui-monospace,"
        "monospace;margin:0;padding:24px;max-width:1000px}"
        "pre{white-space:pre-wrap;background:#131a22;padding:16px;border-radius:8px;"
        "overflow-x:auto}h1{font-size:18px}</style>"
        f"<h1>ligmax trip: {header.get('label','?')} "
        f"&mdash; {header.get('local_time','')}</h1>"
        + "".join(parts)
        + "<pre>"
        + summary.replace("&", "&amp;").replace("<", "&lt;")
        + "</pre>"
    )


# --------------------------------------------------------------------- main

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("file", nargs="?", help="a .jsonl.gz trip; default is the newest")
    parser.add_argument("--dir", default=DEFAULT_DIR)
    parser.add_argument("--list", action="store_true", help="list the recordings")
    parser.add_argument("--events", action="store_true", help="only the event lines")
    parser.add_argument("--html", metavar="OUT", help="write a self-contained page")
    args = parser.parse_args()

    if args.list:
        files = sorted(glob.glob(os.path.join(args.dir, "*.jsonl.gz")))
        if not files:
            print(f"no recordings in {args.dir}")
            return 1
        for path in files:
            print(f"{os.path.getsize(path) / 1024:8.0f} kB  {path}")
        return 0

    path = args.file or newest(args.dir)
    if not path or not os.path.exists(path):
        print(f"no trip recording found (looked in {args.dir})", file=sys.stderr)
        return 1

    header, rows, footer = load(path)
    if args.events:
        for row in rows:
            if row.get("kind") == "event":
                print(json.dumps(row))
        return 0

    if args.html:
        with open(args.html, "w", encoding="utf-8") as handle:
            handle.write(to_html(header, rows, footer))
        print(f"wrote {args.html}")
        return 0

    print(f"# {path}")
    print(summarise(header, rows, footer))
    return 0


if __name__ == "__main__":
    sys.exit(main())
