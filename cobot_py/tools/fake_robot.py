#!/usr/bin/env python3
"""A stand-in robot controller, for developing and testing without hardware.

Speaks the same ``#<json>#`` protocol: accepts a SocketTx, answers with a SocketRx.
It models just enough of the robot to exercise the client — the BASE/MOTION/ERROR
state machine, event acknowledgement, and joint positions integrated from the
velocity stream.

    python tools/fake_robot.py --port 15263
    COBOT_IP=127.0.0.1 python examples/03_velocity_stream.py
"""

from __future__ import annotations

import argparse
import json
import logging
import socket
import time

logging.basicConfig(level=logging.INFO, format="%(levelname)s fake_robot: %(message)s")
log = logging.getLogger(__name__)

N_DOF = 6
START = b"#"
END = b"#"

# mirrors of the enums the client uses
STATE_BASE, STATE_MOTION, STATE_ERROR = 1, 2, 3
ACTION_GO_TO_BASE, ACTION_GO_TO_MOTION = 1, 2
EVENT_EXECUTING, EVENT_SUCCEEDED = 1, 3

#: SafetyContextEnums 0..2; 3 is eMaxRegime, a count the real server rejects
VALID_SAFETY_MODES = (0, 1, 2)
ERR_INVALID_SAFETY_TYPE = 2514

#: cycles an event stays EXECUTING before it reports SUCCEEDED
EVENT_DURATION_CYCLES = 5


class FakeRobot:
    def __init__(self, cycle_time: float = 0.004):
        self.cycle_time = cycle_time
        self.state = STATE_BASE
        self.error = 0
        self.jpos = [0.0] * N_DOF
        self.jvel = [0.0] * N_DOF
        self.t0 = time.monotonic()

        self.event = 0
        self.event_status = 0
        self.event_cycles = 0

    # -- protocol -------------------------------------------------------- #

    def serve(self, host: str, port: int) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((host, port))
        srv.listen(1)
        log.info("listening on %s:%s", host, port)

        while True:
            conn, addr = srv.accept()
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            log.info("client connected: %s", addr)
            try:
                self._session(conn)
            except (OSError, ConnectionError) as exc:
                log.info("client gone: %s", exc)
            finally:
                conn.close()
                self.__init__(self.cycle_time)  # reset for the next run

    def _session(self, conn: socket.socket) -> None:
        buf = bytearray()
        while True:
            packet = self._take(buf)
            if packet is None:
                chunk = conn.recv(4096)
                if not chunk:
                    return
                buf.extend(chunk)
                continue

            try:
                tx = json.loads(packet)
            except ValueError:
                tx = {}

            self._step(tx)
            conn.sendall(START + json.dumps(self._rx()).encode() + END)
            time.sleep(self.cycle_time)

    @staticmethod
    def _take(buf: bytearray) -> str | None:
        begin = buf.find(START)
        if begin < 0:
            buf.clear()
            return None
        end = buf.find(END, begin + 1)
        if end < 0:
            del buf[:begin]
            return None
        body = bytes(buf[begin + 1 : end])
        del buf[: end + 1]
        return body.decode(errors="replace")

    # -- behaviour ------------------------------------------------------- #

    def _step(self, tx: dict) -> None:
        interrupt = tx.get("interrupt") or {}
        action = int(interrupt.get("action") or 0)

        controller = ((tx.get("config") or {}).get("controller_config")) or {}
        safety = controller.get("safety_mode")
        if safety is not None and int(safety) not in VALID_SAFETY_MODES:
            if self.state != STATE_ERROR:
                log.info("invalid safety type %s -> ERROR", safety)
            self.state = STATE_ERROR
            self.error = ERR_INVALID_SAFETY_TYPE
            return

        if action == ACTION_GO_TO_MOTION and self.state == STATE_BASE:
            self.state = STATE_MOTION
            log.info("-> MOTION")
        elif action == ACTION_GO_TO_BASE and self.state != STATE_ERROR:
            if self.state != STATE_BASE:
                log.info("-> BASE")
            self.state = STATE_BASE
            self.jvel = [0.0] * N_DOF

        event_config = tx.get("event_config")
        requested = int((event_config or {}).get("event") or 0)

        if requested != 0:
            if self.event == 0:
                self.event = requested
                self.event_status = EVENT_EXECUTING
                self.event_cycles = 0
                log.info("event %s -> EXECUTING", self.event)
        elif self.event_status == EVENT_SUCCEEDED:
            # the client withdrew the event config and has seen the result;
            # clear it so the next event can arm.
            self.event = 0
            self.event_status = 0

        if self.event_status == EVENT_EXECUTING:
            self.event_cycles += 1
            if self.event_cycles >= EVENT_DURATION_CYCLES:
                self.event_status = EVENT_SUCCEEDED
                log.info("event %s -> SUCCEEDED", self.event)

        control = tx.get("control_int")
        if control and self.state == STATE_MOTION:
            cmd = control.get("external_control") or control.get("jog_joint") or []
            if len(cmd) == N_DOF:
                self.jvel = [float(v) for v in cmd]

        if self.state == STATE_MOTION:
            self.jpos = [p + v * self.cycle_time for p, v in zip(self.jpos, self.jvel)]

    def _rx(self) -> dict:
        return {
            "interrupt": {"action": 0, "state": self.state, "error": self.error},
            "event_status": {"event": self.event, "status": self.event_status},
            "feedback": {
                "joint_position": self.jpos,
                "joint_velocity": self.jvel,
                "joint_acceleration": [0.0] * N_DOF,
                "joint_torque": [0.0] * N_DOF,
                "joint_error_code": [0.0] * N_DOF,
                "joint_status": [0.0] * N_DOF,
                "end_effector_position": [0.0] * 6,
                "force_magnitude": [0.0],
                "force_direction": [0.0, 0.0, 0.0],
                "time_stamp": time.monotonic() - self.t0,
            },
        }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=15263)
    ap.add_argument("--cycle-time", type=float, default=0.004)
    args = ap.parse_args()

    FakeRobot(args.cycle_time).serve(args.host, args.port)


if __name__ == "__main__":
    main()
