#!/usr/bin/env python3
"""The operator's go-to and its speed cap, against a fake flight controller.

    python3 tests/test_guided.py            # quiet
    python3 tests/test_guided.py -v         # every check

Same shape as `tests/test_preflight.py`: `FakeMaster` records what would have
gone on the wire, `FakeNavigation` stands in for the grid origin, and nothing
here opens a port or needs a boat.

What is worth testing is not the MAVLink encoding - pymavlink owns that - but the
two promises `guided.py` makes:

  * the cap the dashboard shows is the cap the autopilot was given, or the
    operator is told it is not. A value above the 5 kn vessel limit is refused
    rather than clamped, and a refused value never becomes the stored one.
  * a go-to that could not be sent did not half-happen. The speed goes out first
    and the target only after it, so there is no case where the boat has a new
    destination and an old speed.
"""

from __future__ import annotations

import logging
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

logging.basicConfig(
    level=logging.DEBUG if "-v" in sys.argv else logging.CRITICAL,
    format="  log   %(message)s",
)

from pymavlink import mavutil  # noqa: E402

from config import VESSEL_SPEED_LIMIT_MS  # noqa: E402

from nodes.io_manager.guided import (  # noqa: E402
    MAX_LIMIT_MS,
    MAX_RANGE_M,
    MIN_LIMIT_MS,
    POSITION_ONLY,
    Guided,
)

VERBOSE = "-v" in sys.argv
FAILURES: list[str] = []

SPEED_CMD = mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED
# Trondheim harbour, near enough. Only used as a grid origin.
ORIGIN = {"lat": 63.43049, "lon": 10.39506}


def check(condition: object, message: str) -> None:
    if condition:
        if VERBOSE:
            print(f"  ok    {message}")
    else:
        print(f"  FAIL  {message}")
        FAILURES.append(message)


def section(title: str) -> None:
    print(f"\n=== {title}")


class _FakeMav:
    def __init__(self, sent: list) -> None:
        self.sent = sent
        self.fail = False  # make the next send raise, as a dropped link does

    def command_long_send(self, *args) -> None:
        if self.fail:
            raise OSError("link down")
        self.sent.append(("command_long", *args))

    def set_position_target_global_int_send(self, *args) -> None:
        if self.fail:
            raise OSError("link down")
        self.sent.append(("position_target", *args))


class FakeMaster:
    target_system = 1
    target_component = 1

    def __init__(self) -> None:
        self.sent: list = []
        self.mav = _FakeMav(self.sent)


class FakeNavigation:
    """`to_global()` only - the one method a go-to uses."""

    def __init__(self, origin=ORIGIN) -> None:
        self.origin = origin

    def to_global(self, x, y):
        if self.origin is None:
            return None
        # Not the same arithmetic as navigation.py on purpose: this only has to
        # be a deterministic mapping, and reimplementing the flat-earth formula
        # here would test it against itself.
        return self.origin["lat"] + y * 1e-5, self.origin["lon"] + x * 1e-5


class Ack:
    def __init__(self, command: int, result: int) -> None:
        self.command = command
        self.result = result

    def get_type(self) -> str:
        return "COMMAND_ACK"


def speeds(master: FakeMaster) -> list[float]:
    """Every ground speed DO_CHANGE_SPEED was asked for, in order.

    Recorded args are `(kind, target_system, target_component, command,
    confirmation, param1, param2, ...)`, and DO_CHANGE_SPEED carries the speed
    type in param1 and the speed itself in param2 - so message[6], and the check
    below that param1 is 1 is what stops those two being swapped unnoticed.
    """
    out = []
    for message in master.sent:
        if message[0] != "command_long" or message[3] != SPEED_CMD:
            continue
        check(message[5] == 1, "the speed is sent as a GROUND speed (param1 = 1)")
        out.append(message[6])
    return out


# ------------------------------------------------------------------- the cap


def test_the_cap_is_refused_not_clamped() -> None:
    section("the speed cap")
    master, guided = FakeMaster(), Guided()

    ok, why = guided.set_limit(master, VESSEL_SPEED_LIMIT_MS + 1.0)
    check(not ok, "a value above the vessel limit is refused")
    check("vessel limit" in why, f"and says why: {why!r}")
    check(guided.limit == MAX_LIMIT_MS, "the refused value was not stored")
    check(not speeds(master), "and nothing went on the wire")

    ok, _ = guided.set_limit(master, MIN_LIMIT_MS / 2)
    check(not ok, "a value below the floor is refused too")

    for value in ("fast", None, float("nan"), float("inf")):
        ok, _ = guided.set_limit(master, value)
        check(not ok, f"{value!r} is not a speed")
    check(not speeds(master), "none of the bad values reached the autopilot")

    ok, why = guided.set_limit(master, 1.0)
    check(ok, f"1.0 m/s is accepted: {why!r}")
    check(guided.limit == 1.0, "and stored")
    check(speeds(master) == [1.0], "and sent once, as a ground speed")
    check(
        guided.telemetry()["speed_limit_sent"] is True,
        "telemetry says the autopilot has been told",
    )


def test_a_dropped_link_does_not_move_the_cap() -> None:
    section("the cap when the link is gone")
    master, guided = FakeMaster(), Guided()
    guided.set_limit(master, 1.5)
    master.mav.fail = True

    ok, why = guided.set_limit(master, 0.5)
    check(not ok, f"a send that raised is a failure, not a silent one: {why!r}")
    check(
        guided.limit == 1.5,
        "and the cap the dashboard shows is still the one the autopilot has",
    )


def test_an_unsupported_cap_is_not_silent() -> None:
    section("a firmware that refuses DO_CHANGE_SPEED")
    master, guided = FakeMaster(), Guided()
    guided.set_limit(master, 1.0)

    guided.note_ack(Ack(SPEED_CMD, mavutil.mavlink.MAV_RESULT_UNSUPPORTED))
    check(
        "unsupported" in (guided.telemetry().get("speed_limit_refused") or ""),
        "the refusal rides up beside the figure",
    )

    guided.note_ack(Ack(SPEED_CMD, mavutil.mavlink.MAV_RESULT_ACCEPTED))
    check(
        "speed_limit_refused" not in guided.telemetry(),
        "and clears when a later cap is accepted",
    )

    # Somebody else's ack, and the arm/mode acks that arrive constantly.
    guided.note_ack(Ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                        mavutil.mavlink.MAV_RESULT_DENIED))
    check(
        "speed_limit_refused" not in guided.telemetry(),
        "another command's refusal is not read as ours",
    )


# ------------------------------------------------------------------ the go-to


def test_a_goto_sends_the_speed_then_the_target() -> None:
    section("the go-to")
    master, guided = FakeMaster(), Guided()
    guided.set_limit(master, 1.2)
    master.sent.clear()

    ok, why = guided.goto(master, FakeNavigation(), 20.0, -35.0)
    check(ok, f"a point inside the grid is accepted: {why!r}")
    kinds = [message[0] for message in master.sent]
    check(
        kinds == ["command_long", "position_target"],
        f"the speed goes out before the target, and nothing else does: {kinds}",
    )

    target = next(m for m in master.sent if m[0] == "position_target")
    check(speeds(master) == [1.2], "the speed sent is the standing cap")
    check(
        target[4] == mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        "the frame is a global one",
    )
    check(target[5] == POSITION_ONLY, "velocity, acceleration and yaw are ignored")
    expected_lat = int(round((ORIGIN["lat"] - 35.0 * 1e-5) * 1e7))
    expected_lon = int(round((ORIGIN["lon"] + 20.0 * 1e-5) * 1e7))
    check(target[6] == expected_lat, "north metres became latitude, in 1e7 degrees")
    check(target[7] == expected_lon, "east metres became longitude")
    check(
        guided.telemetry().get("goto_target") == "20.0, -35.0 m",
        "and the target reads back in grid metres",
    )


def test_a_goto_that_cannot_work_is_refused() -> None:
    section("go-to refusals")
    master, guided = FakeMaster(), Guided()

    ok, why = guided.goto(master, FakeNavigation(origin=None), 10.0, 10.0)
    check(not ok, f"no grid origin, no go-to: {why!r}")
    check(not master.sent, "and nothing was sent, not even the speed")

    ok, why = guided.goto(master, FakeNavigation(), MAX_RANGE_M + 1.0, 0.0)
    check(not ok, f"a point kilometres away is refused: {why!r}")

    for x, y in ((float("nan"), 0.0), (0.0, float("inf")), ("here", 0.0), (None, None)):
        ok, _ = guided.goto(master, FakeNavigation(), x, y)
        check(not ok, f"({x!r}, {y!r}) is not a point")
    check(not master.sent, "none of the refusals touched the wire")
    check(
        "goto_target" not in guided.telemetry(),
        "and none of them left a target in the telemetry",
    )

    master.mav.fail = True
    ok, why = guided.goto(master, FakeNavigation(), 5.0, 5.0)
    check(not ok, f"a link that drops mid-command is a failure: {why!r}")


TESTS = (
    test_the_cap_is_refused_not_clamped,
    test_a_dropped_link_does_not_move_the_cap,
    test_an_unsupported_cap_is_not_silent,
    test_a_goto_sends_the_speed_then_the_target,
    test_a_goto_that_cannot_work_is_refused,
)


def main() -> int:
    for test in TESTS:
        test()
    print("\n" + "=" * 60)
    if FAILURES:
        print(f"{len(FAILURES)} check(s) failed:")
        for failure in FAILURES:
            print(f"  - {failure}")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
