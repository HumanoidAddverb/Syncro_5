#!/usr/bin/env python3
"""Joint jogging plus gripper open/close.

    python examples/04_jog_and_gripper.py 192.168.0.12
"""

import logging
import sys

import python_examples.cobot_py.examples._path as _path  # noqa: F401  - allows running without `pip install -e .`
from cobot import API, Cobot, GripperType

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")


def main() -> None:
    ip = sys.argv[1] if len(sys.argv) > 1 else None

    with Cobot(ip) as bot:
        bot.set_gripper_type(GripperType.SUCTION)
        bot.set_payload(mass=1.0, com=[0.0, 0.0, 0.05], moi=[0.0] * 6)
        bot.power_on()

        with bot.control_stream(API.JOG_JOINT, rate_hz=100) as stream:
            for _ in stream.ticks(seconds=2.0):
                bot.jog_joint([0.0, 0.2, 0.0, 0.0, 0.0, 0.0])

        bot.close_gripper(force=40.0)
        bot.open_gripper()

        bot.power_off()


if __name__ == "__main__":
    main()
