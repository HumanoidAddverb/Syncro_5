#!/usr/bin/env python3
"""Connect and print live robot feedback. Does not move the robot.

    python examples/01_read_state.py 192.168.0.12
"""

import logging
import sys

import python_examples.cobot_py.examples._path as _path  # noqa: F401  - allows running without `pip install -e .`
from cobot import Cobot

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")


def main() -> None:
    ip = sys.argv[1] if len(sys.argv) > 1 else None

    with Cobot(ip) as bot:
        print("state:", bot.state.name)

        for _ in range(20):
            fb = bot.feedback()
            pos = " ".join(f"{v:+.4f}" for v in fb.joint_position)
            tor = " ".join(f"{v:+.2f}" for v in fb.joint_torque)
            print(f"t={fb.time_stamp:.3f}  q=[{pos}]  tau=[{tor}]")


if __name__ == "__main__":
    main()
