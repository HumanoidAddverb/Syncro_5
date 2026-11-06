#!/usr/bin/env python3
"""Record waypoints by hand-guiding the arm, then play them back.

    python examples/05_teach_and_replay.py 192.168.0.12
"""

import logging
import sys
import time

import python_examples.cobot_py.examples._path as _path  # noqa: F401  - allows running without `pip install -e .`
from cobot import API, Cobot, RecordedDataEntry

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")

SECONDS_BETWEEN_POINTS = 4.0


def main() -> None:
    ip = sys.argv[1] if len(sys.argv) > 1 else None

    with Cobot(ip) as bot:
        bot.power_on()

        # free drive: the arm is compliant, push it where you want it
        bot.switch_controller(API.FREE_DRIVE)

        recorded = []
        for i in range(4):
            input(f"position the arm for waypoint {i + 1}, then press Enter...")
            recorded.append(
                RecordedDataEntry(
                    desired_pos=bot.joint_positions,
                    desired_time=SECONDS_BETWEEN_POINTS * (i + 1),
                )
            )
            print("  recorded:", [f"{v:+.3f}" for v in recorded[-1].desired_pos])

        input("press Enter to replay...")
        bot.replay(recorded, label="taught_path", iterations=2)

        # replay runs on the robot; poll until it settles back out of motion
        time.sleep(SECONDS_BETWEEN_POINTS * len(recorded) * 2)

        bot.power_off()


if __name__ == "__main__":
    main()
