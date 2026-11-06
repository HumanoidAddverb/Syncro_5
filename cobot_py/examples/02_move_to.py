#!/usr/bin/env python3
"""Point-to-point joint motion, then a multi-waypoint motion.

    python examples/02_move_to.py 192.168.0.12
"""

import logging
import sys

import python_examples.cobot_py.examples._path as _path  # noqa: F401  - allows running without `pip install -e .`
# from src.cobot import API, Cobot
from cobot import API, Cobot
logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")


def main() -> None:
    ip = sys.argv[1] if len(sys.argv) > 1 else None

    with Cobot(ip) as bot:
        bot.set_payload(mass=1.0, com=[0.0, 0.0, 0.05], moi=[0.0] * 6)
        print("reached here")
        bot.power_on()
        print("home reached:", bot.joint_positions)

        # single goal: all joints to zero over 10 s
        bot.move_to([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], duration=10)
        print("home reached:", bot.joint_positions)

        # waypoint sequence; times are cumulative from the start of the motion
        bot.move_through(
            positions=[
                [0.5, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.5, 0.0, 0.3, 0.0, 0.0, 0.0],
                [-0.5, 0.0, 0.3, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            ],
            times=[5, 10, 16, 22],
            controller=API.MULTI_POINT,
        )
        print("sequence done:", bot.joint_positions)

        bot.power_off()


if __name__ == "__main__":
    main()
