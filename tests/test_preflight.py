#!/usr/bin/env python3
"""The safety switch and the compass swing, against a fake flight controller.

    python3 tests/test_preflight.py            # quiet
    python3 tests/test_preflight.py -v         # every check

No hardware and no pymavlink connection: `FakeMaster` records what would have
gone on the wire and the tests hand back the COMMAND_ACK the autopilot would
have sent. Same shape as `tests/test_autopilot.py` - plain asserts in one
runnable file, because the person running this is on a laptop in a tent.

What is worth testing here is not the MAVLink encoding, which pymavlink owns.
It is the promise `preflight.py` makes to the operator: that the word on the
dashboard beside "safety switch" is only ever there because the autopilot said
so. Every test below is some way that promise could be broken - a refusal read
as a success, an ack that never came, a link that dropped underneath one.
"""

from __future__ import annotations

import logging
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# The module logs every state change at WARNING, which is right on the boat and
# noise here - the tests assert on the outcomes, not on the log. `-v` puts it
# back, because a failing test is exactly when those lines are worth reading.
logging.basicConfig(
    level=logging.DEBUG if "-v" in sys.argv else logging.CRITICAL,
    format="  log   %(message)s",
)

from pymavlink import mavutil  # noqa: E402

from nodes.io_manager.preflight import (  # noqa: E402
    SAFETY_OFF,
    SAFETY_ON,
    SAFETY_UNKNOWN,
    Preflight,
)

VERBOSE = "-v" in sys.argv
FAILURES: list[str] = []

SAFETY_CMD = mavutil.mavlink.MAV_CMD_DO_SET_SAFETY_SWITCH_STATE
MAG_CMD = mavutil.mavlink.MAV_CMD_FIXED_MAG_CAL_YAW
TRONDHEIM = (63.43049, 10.39506)


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

    def command_long_send(self, *args) -> None:
        self.sent.append(("long", *args))

    def command_int_send(self, *args) -> None:
        self.sent.append(("int", *args))


class FakeMaster:
    """Enough of a pymavlink connection to record what was sent."""

    target_system = 1
    target_component = 1

    def __init__(self) -> None:
        self.sent: list = []
        self.mav = _FakeMav(self.sent)


class Ack:
    """A COMMAND_ACK as the pump would hand it over."""

    def __init__(self, command: int, result: int) -> None:
        self.command = command
        self.result = result

    def get_type(self) -> str:
        return "COMMAND_ACK"


class Clock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


# ------------------------------------------------------------ safety switch


def test_safety_needs_the_autopilots_word():
    section("the safety switch is never claimed on our say-so")
    master = FakeMaster()
    pre = Preflight()

    check(
        pre.telemetry()["safety_switch"] == SAFETY_UNKNOWN,
        "a fresh node does not know where the switch is",
    )
    check(
        pre.telemetry()["safety_switch_seen"] is False,
        "and says so - the word is a default, not an observation",
    )

    started, why = pre.set_safety(master, "c-1", safe=False)
    check(started, f"safety off is sent ({why})")
    check(master.sent[-1][3] == SAFETY_CMD, "as MAV_CMD_DO_SET_SAFETY_SWITCH_STATE")
    check(
        master.sent[-1][5] == float(mavutil.mavlink.SAFETY_SWITCH_STATE_DANGEROUS),
        "with param1 = DANGEROUS, which is the one that makes outputs live",
    )

    # THE test. Between the send and the ack the operator must not be told the
    # outputs are live, because they may not be.
    check(
        pre.telemetry()["safety_switch"] == SAFETY_UNKNOWN,
        "sent is not done: still unknown while the ack is outstanding",
    )
    check(pre.take() is None, "and nothing is acked to the dashboard yet")
    check(pre.telemetry()["pending"], "the dashboard is told what is outstanding")

    pre.handle(master, Ack(SAFETY_CMD, mavutil.mavlink.MAV_RESULT_ACCEPTED))
    command_id, ok, message = pre.take()
    check((command_id, ok) == ("c-1", True), f"accepted -> acked ok ({message})")
    check(pre.telemetry()["safety_switch"] == SAFETY_OFF, "and only now is it 'off'")
    check(pre.telemetry()["safety_switch_seen"] is True, "observed, not defaulted")


def test_a_refusal_is_not_a_success():
    section("a board that refuses is reported as refusing")
    master = FakeMaster()
    pre = Preflight()

    # Get it into a known state first, so the failure below has something it
    # could wrongly overwrite.
    pre.set_safety(master, "c-1", safe=False)
    pre.handle(master, Ack(SAFETY_CMD, mavutil.mavlink.MAV_RESULT_ACCEPTED))
    pre.take()

    for result, word in (
        (mavutil.mavlink.MAV_RESULT_FAILED, "failed"),
        (mavutil.mavlink.MAV_RESULT_UNSUPPORTED, "unsupported"),
        (mavutil.mavlink.MAV_RESULT_DENIED, "denied"),
    ):
        pre.set_safety(master, "c-x", safe=True)
        pre.handle(master, Ack(SAFETY_CMD, result))
        _, ok, message = pre.take()
        check(not ok, f"MAV_RESULT {result} is acked as a failure")
        check(word in message, f"and says {word!r} in the operator's words: {message}")
        check(
            pre.telemetry()["safety_switch"] == SAFETY_OFF,
            "a refused 'safety on' leaves the state where it was - OFF, live, "
            "which is the reading that keeps hands out of the water",
        )


def test_one_at_a_time():
    section("one command in flight, so one ack cannot answer two")
    master = FakeMaster()
    pre = Preflight()
    check(pre.set_safety(master, "c-1", safe=True)[0], "the first goes")
    started, why = pre.set_safety(master, "c-2", safe=False)
    check(not started, f"the second is refused while the first is outstanding ({why})")
    check(pre.take() is None, "the refusal is the caller's to ack, not an outcome here")
    pre.handle(master, Ack(SAFETY_CMD, mavutil.mavlink.MAV_RESULT_ACCEPTED))
    check(pre.set_safety(master, "c-3", safe=False)[0], "and it frees up once acked")


def test_silence_and_a_dropped_link():
    section("no answer, and no link")
    master = FakeMaster()
    clock = Clock()
    pre = Preflight(clock=clock)

    pre.set_safety(master, "c-1", safe=True)
    clock.t = 1.0
    pre.check_timeout()
    check(pre.take() is None, "a command still inside its window is left alone")
    clock.t = 60.0
    pre.check_timeout()
    _, ok, message = pre.take()
    check(not ok, "a command the autopilot never answered fails rather than hangs")
    check("never answered" in message, f"and says why: {message}")
    check(
        pre.telemetry()["safety_switch"] == SAFETY_UNKNOWN,
        "silence teaches us nothing about the switch",
    )

    # A link that drops must not leave the operator's row at "sent" either, and
    # must forget a state that belongs to a flight controller we can no longer
    # see - it may have rebooted, which puts the switch back where BRD_SAFETY_DEFLT
    # says rather than where we last put it.
    pre.set_safety(master, "c-2", safe=False)
    pre.handle(master, Ack(SAFETY_CMD, mavutil.mavlink.MAV_RESULT_ACCEPTED))
    pre.take()
    check(pre.telemetry()["safety_switch"] == SAFETY_OFF, "state observed before the drop")
    pre.set_safety(master, "c-3", safe=True)
    pre.link_down()
    _, ok, message = pre.take()
    check(not ok, "the in-flight command is failed on a dropped link")
    check(
        pre.telemetry()["safety_switch"] == SAFETY_UNKNOWN,
        "and the remembered state goes with it",
    )


def test_no_link_at_all():
    section("no autopilot")
    pre = Preflight()
    started, why = pre.set_safety(None, "c-1", safe=True)
    check(not started and "no autopilot link" in why, f"refused before anything ({why})")
    started, why = pre.compass_cal(None, "c-2", 90.0, position=TRONDHEIM)
    check(not started and "no autopilot link" in why, f"same for the compass ({why})")


# ------------------------------------------------------------------ compass


def test_compass_arguments():
    section("what the compass swing refuses before it sends anything")
    master = FakeMaster()
    pre = Preflight()

    started, why = pre.compass_cal(master, "c-1", 90.0)
    check(not started, "no position, no calibration - the magnetic model needs one")
    check("GNSS" in why, f"and says so: {why}")

    for bad in ("north", None, float("nan"), float("inf")):
        started, why = pre.compass_cal(master, "c-1", bad, position=TRONDHEIM)
        check(not started, f"heading={bad!r} is refused ({why})")

    check(not master.sent, "none of those put anything on the wire")

    started, why = pre.compass_cal(master, "c-1", 451.5, position=TRONDHEIM)
    check(started, f"a heading past 360 wraps rather than failing ({why})")
    check(master.sent[-1][3] == MAG_CMD, "sent as MAV_CMD_FIXED_MAG_CAL_YAW")
    check(abs(master.sent[-1][5] - 91.5) < 1e-6, "wrapped to 91.5 deg")
    check(
        master.sent[-1][6] == 0 and master.sent[-1][7] == 0 and master.sent[-1][8] == 0,
        "all compasses (mask 0), and lat/lon 0 so the autopilot uses its own fix "
        "rather than a copy of it that has been through a float32",
    )

    pre.handle(master, Ack(MAG_CMD, mavutil.mavlink.MAV_RESULT_ACCEPTED))
    _, ok, _ = pre.take()
    swing = pre.telemetry()["compass_cal"]
    check(ok and swing["heading_deg"] == 91.5, "the swing is recorded once accepted")
    check(swing["lat"] == round(TRONDHEIM[0], 6), "with where it was done")


def test_command_int_only_firmware():
    section("firmware that has moved the command to COMMAND_INT")
    master = FakeMaster()
    pre = Preflight()
    pre.compass_cal(master, "c-1", 137.0, position=TRONDHEIM)
    check(master.sent[-1][0] == "long", "tried as COMMAND_LONG first")

    pre.handle(master, Ack(MAG_CMD, mavutil.mavlink.MAV_RESULT_COMMAND_INT_ONLY))
    check(master.sent[-1][0] == "int", "COMMAND_INT_ONLY is retried in the other shape")
    check(pre.busy, "still one command, still waiting - not acked as a failure")
    check(pre.take() is None, "and the operator sees nothing about the retry")

    # Retried once, not forever: a firmware that answers INT_ONLY to both shapes
    # must still reach the operator rather than ping-pong.
    pre.handle(master, Ack(MAG_CMD, mavutil.mavlink.MAV_RESULT_COMMAND_INT_ONLY))
    _, ok, message = pre.take()
    check(not ok, f"the second refusal is reported ({message})")

    pre.compass_cal(master, "c-2", 137.0, position=TRONDHEIM)
    pre.handle(master, Ack(MAG_CMD, mavutil.mavlink.MAV_RESULT_ACCEPTED))
    _, ok, _ = pre.take()
    check(ok, "and an ordinary accept still works afterwards")


def test_in_progress_does_not_time_out():
    section("a calibration the autopilot says it is working on")
    master = FakeMaster()
    clock = Clock()
    pre = Preflight(clock=clock)
    pre.compass_cal(master, "c-1", 20.0, position=TRONDHEIM)

    clock.t = 2.0
    pre.handle(master, Ack(MAG_CMD, mavutil.mavlink.MAV_RESULT_IN_PROGRESS))
    clock.t = 4.0
    pre.check_timeout()
    check(pre.busy, "IN_PROGRESS restarts the clock instead of timing out under it")
    check(pre.take() is None, "and nothing is acked on the strength of it")
    pre.handle(master, Ack(MAG_CMD, mavutil.mavlink.MAV_RESULT_ACCEPTED))
    check(pre.take()[1] is True, "the real answer still lands")


def test_other_traffic_is_left_alone():
    section("acks that are not ours")
    master = FakeMaster()
    pre = Preflight()
    pre.set_safety(master, "c-1", safe=True)

    class Heartbeat:
        def get_type(self) -> str:
            return "HEARTBEAT"

    check(not pre.handle(master, Heartbeat()), "a HEARTBEAT is not consumed")
    check(
        not pre.handle(
            master, Ack(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
        ),
        "nor is the ack for an arm command - a different command number",
    )
    check(pre.busy, "and ours is still outstanding")


def main() -> int:
    start = time.time()
    for test in (
        test_safety_needs_the_autopilots_word,
        test_a_refusal_is_not_a_success,
        test_one_at_a_time,
        test_silence_and_a_dropped_link,
        test_no_link_at_all,
        test_compass_arguments,
        test_command_int_only_firmware,
        test_in_progress_does_not_time_out,
        test_other_traffic_is_left_alone,
    ):
        try:
            test()
        except Exception as exc:  # noqa: BLE001 - report and carry on
            import traceback

            print(f"  ERROR in {test.__name__}: {exc}")
            traceback.print_exc()
            FAILURES.append(f"{test.__name__} raised {exc!r}")

    print(f"\n{'=' * 60}")
    if FAILURES:
        print(f"{len(FAILURES)} FAILURE(S) in {time.time() - start:.1f}s:")
        for failure in FAILURES:
            print(f"  - {failure}")
        return 1
    print(f"all checks passed in {time.time() - start:.1f}s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
