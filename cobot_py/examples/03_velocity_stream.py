#!/usr/bin/env python3
"""Stream joint velocities at 100 Hz for 5 seconds.

Continuous controllers need a command every cycle — the stream stops the arm
automatically when the block exits.

    python examples/03_velocity_stream.py 192.168.0.12
"""

import logging
import math
import sys

import python_examples.cobot_py.examples._path as _path  # noqa: F401  - allows running without `pip install -e .`
from cobot import API, Cobot

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")


def main() -> None:
    ip = sys.argv[1] if len(sys.argv) > 1 else None

    with Cobot(ip) as bot:
        bot.power_on()

        with bot.control_stream(API.EXTERNAL_VELOCITY, rate_hz=100) as stream:
            for t in stream.ticks(seconds=5.0):
                # gentle sinusoid on joint 2, everything else held still
                bot.set_velocity([0.0, 0.1 * math.sin(2 * math.pi * 0.2 * t), 0.0, 0.0, 0.0, 0.0])

        print("final:", bot.joint_positions)
        bot.power_off()


if __name__ == "__main__":
    main()
