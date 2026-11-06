"""Connection diagnostics: ``python -m cobot.diagnose [ip] [port]``.

Walks the connect sequence one stage at a time and reports where it stops, so a
failure can be pinned on the network, the robot-side controller, or the framing.

IMPORTANT: this opens exactly ONE TCP connection for the whole run.
``SocketServer::initConnect`` does a single ``accept()`` per session and
``CONNECTION_QUEUE`` is 1, so a throwaway "is the port reachable" probe would burn
the robot's only accept slot and get the real connection reset. Never add one.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from .config import LinkConfig
from .enums import API, RobotAction, RobotState, SafetyMode, describe_robot_error
from .errors import ConnectFailed, ConnectionLost
from .messages import ControllerConfig
from .session import RobotSession

DEPLOYED_IP_CSV = Path("/opt/addverb/cobot_backend/config/ip.csv")

OK = "  ok  "
FAIL = " FAIL "
INFO = " info "

RESTART_HINT = (
    "The robot accepts ONE connection per session (CONNECTION_QUEUE = 1). If an "
    "earlier run died mid-session it may never have gone back to accept(). "
    "Restart the robot-side controller, then retry this immediately."
)


def _say(tag: str, msg: str) -> None:
    print(f"[{tag}] {msg}", flush=True)


def _read_deployed_config() -> tuple[str, int] | None:
    """The C++ read line 0 = IP, line 1 = port from this file."""
    try:
        lines = DEPLOYED_IP_CSV.read_text().split()
    except OSError:
        return None
    if len(lines) < 2 or not lines[1].isdigit():
        return None
    return lines[0], int(lines[1])


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("ip", nargs="?", default=None)
    ap.add_argument("port", nargs="?", type=int, default=None)
    ap.add_argument(
        "--handshake-timeout",
        type=float,
        default=60.0,
        help="total seconds to wait for the robot to reach accept() and answer",
    )
    ap.add_argument(
        "--handshake-interval",
        type=float,
        default=0.5,
        help="seconds between init-packet resends",
    )
    ap.add_argument(
        "--power-on",
        action="store_true",
        help="after the link checks, send GO_TO_MOTION and report what the robot does",
    )
    ap.add_argument("--seconds", type=float, default=30.0, help="how long to watch --power-on")
    args = ap.parse_args(argv)

    cfg = LinkConfig.from_env()
    if args.ip:
        cfg.ip = args.ip
    if args.port:
        cfg.port = args.port
    cfg.handshake_timeout = args.handshake_timeout
    cfg.handshake_poll_interval = args.handshake_interval

    deployed = _read_deployed_config()
    if deployed is None:
        _say(INFO, f"no readable 2-line config at {DEPLOYED_IP_CSV}")
    else:
        _say(INFO, f"deployed C++ config says {deployed[0]}:{deployed[1]}")
        if (cfg.ip, cfg.port) != deployed:
            _say(INFO, f"NOTE: you are targeting {cfg.ip}:{cfg.port}, which differs")

    _say(INFO, f"target: {cfg.ip}:{cfg.port}")
    _say(INFO, "opening a single connection for every stage below")

    session = RobotSession(cfg)

    # 1. connect + handshake, in one shot -- no throwaway probe.
    t0 = time.monotonic()
    try:
        session.connect()
    except ConnectFailed as exc:
        elapsed = time.monotonic() - t0
        _say(FAIL, f"connect failed after {elapsed:.1f}s: {exc}")
        if "could not reach" in str(exc):
            _say(
                INFO,
                "No TCP connection at all: wrong IP, routing/firewall, or the "
                "robot-side controller is not listening yet.",
            )
        else:
            _say(INFO, "TCP came up but the robot never sent a framed packet.")
            _say(INFO, RESTART_HINT)
        return 1
    _say(OK, f"connect + handshake completed in {time.monotonic() - t0:.2f}s")

    try:
        # 2. first real packet
        try:
            rx = session.read_fresh()
        except ConnectionLost as exc:
            _say(FAIL, f"no framed packet: {exc}")
            _say(
                INFO,
                "The handshake saw bytes but the first read failed. A reset here "
                "usually means the robot closed the session on us.",
            )
            _say(INFO, RESTART_HINT)
            return 1

        raw = session.last_raw
        _say(OK, f"received {len(raw)} byte packet")
        print(f"       raw: {raw[:300]}{'...' if len(raw) > 300 else ''}")
        _say(
            OK,
            f"decoded: state={session.state.name} "
            f"error={describe_robot_error(session.robot_error)} "
            f"feedback={'yes' if rx.feedback else 'no'}",
        )
        if rx.feedback:
            _say(INFO, f"joint_position: {[round(v, 4) for v in rx.feedback.joint_position]}")
        if session.robot_error:
            _say(INFO, f"robot error: {describe_robot_error(session.robot_error)}")
            if session.state == RobotState.ERROR:
                _say(
                    INFO,
                    "The robot is in ERROR. Call clear_error() (or recover() to also "
                    "home the arm) before power_on() -- it will not enter MOTION from here.",
                )

        if not args.power_on:
            _say(OK, "link is healthy")
            return 0

        # 3. power-on probe, on the SAME session
        return _probe_power_on(session, args.seconds)
    finally:
        session.shutdown()


def _probe_power_on(session: RobotSession, seconds: float) -> int:
    """Drive the power-on handshake by hand and report every state change."""
    _say(INFO, f"--- power-on probe ({seconds:.0f}s) ---")
    safety = int(SafetyMode.NO_HUMAN_SAFETY)
    _say(INFO, f"staging controller=EXTERNAL_VELOCITY(18) safety_mode={safety}, no payload")

    session.append_controller(
        ControllerConfig(controller=int(API.EXTERNAL_VELOCITY), safety_mode=safety)
    )
    session.commit_config()

    seen: set[tuple[int, int]] = set()
    deadline = time.monotonic() + seconds
    packets = 0
    quiet = 0

    while time.monotonic() < deadline:
        session.set_action(RobotAction.GO_TO_MOTION)
        session.write()

        try:
            session.read_fresh(timeout=2.0)
        except ConnectionLost as exc:
            if not session.is_connected:
                _say(FAIL, f"link dropped mid power-on: {exc}")
                _say(INFO, RESTART_HINT)
                return 1
            quiet += 1
            _say(INFO, f"robot quiet ({quiet}) -- still connected, retrying")
            continue

        packets += 1
        key = (session.state.value, session.robot_error)
        if key not in seen:
            seen.add(key)
            _say(
                INFO,
                f"robot reports state={session.state.name} "
                f"error={describe_robot_error(session.robot_error)}",
            )

        if session.state != RobotState.BASE:
            _say(OK, f"left BASE after {packets} packets -> {session.state.name}")
            if session.state == RobotState.ERROR:
                _say(FAIL, f"robot went to ERROR: {describe_robot_error(session.robot_error)}")
                return 1
            return 0

    _say(FAIL, f"still in BASE after {packets} packets / {seconds:.0f}s ({quiet} quiet gaps)")
    _say(
        INFO,
        "The robot is receiving us (it keeps replying) but will not leave BASE. "
        "Check the robot-side log for why it refuses the transition; if it shows "
        "'api recd from the other end : 18' and 'Updated Robot configuration' "
        "then the link and protocol are fine and the refusal is robot-side.",
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
