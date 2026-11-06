"""Connection + robot configuration.

Replaces: socket_config.h, network_config.h (CommConfig), robot_config.h,
robot_config_info.h and backend/config/ip.csv.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path

#: degrees of freedom (robot_config.h N_DOF)
N_DOF = 6


@dataclass
class LinkConfig:
    """Everything the TCP link needs. Defaults match the C++ backend."""

    ip: str = "192.168.0.12"  # ETH_IP / backend/config/ip.csv
    port: int = 15263  # DEFAULT_SERVER_PORT

    start_delim: str = "#"  # CommConfig::start_delim
    end_delim: str = "#"  # CommConfig::end_delim

    read_buffer_size: int = 4096  # READ_BUFFER_SIZE
    quick_transfer_limit: int = 4096  # CommConfig::quick_transfer_limit

    data_transfer_timeout: float = 5.0  # CommConfig::data_transfer_timeout (s)
    file_transfer_timeout: float = 300.0  # CommConfig::file_transfer_timeout (s)

    #: Total budget for the handshake. Both ends run establishDataLine_() and each
    #: waits for the other, so this is a rendezvous. The robot only reaches its
    #: accept() when DataManager::connect() decides to, which can be well after we
    #: connect -- our TCP connect succeeds off the listen backlog long before then.
    #: Quit early and the robot accepts an already-closed socket, which reads as
    #: readable-with-0-bytes and logs "no data to read" forever. Be patient.
    handshake_timeout: float = 10.0

    #: Retry the whole connect (fresh socket, fresh handshake) this many times.
    #: CobotHWInterface::connect_() loops up to max_reattempt_connection_count
    #: (100) with a 2 s sleep, which is how the ROS 2 stack rides out the robot
    #: not being at accept() yet. A failed attempt also lets the robot's server
    #: fall out of its dead session and go back to accept(), so retrying is what
    #: actually clears a half-open rendezvous.
    connect_attempts: int = 30
    connect_retry_delay: float = 2.0

    #: How long to wait for a reply between init-packet resends. Must stay well
    #: under the robot's own 5 s read poll so that whenever it does poll, one of
    #: our packets is already queued.
    handshake_poll_interval: float = 0.5

    #: TCP-level connect() timeout. The C++ inherits the system default (~2 min).
    connect_timeout: float = 10.0

    #: How long the high level waits for the robot to acknowledge a transition
    #: before raising. The C++ loops are unbounded; this is a deliberate change.
    command_timeout: float = 30.0

    #: Log every packet in both directions at INFO. Set with COBOT_TRACE=1.
    trace: bool = False

    @classmethod
    def from_env(cls) -> "LinkConfig":
        """Build from ``COBOT_IP`` / ``COBOT_PORT`` / ``COBOT_TRACE`` env vars."""
        return cls(
            ip=os.environ.get("COBOT_IP", cls.ip),
            port=int(os.environ.get("COBOT_PORT", cls.port)),
            trace=os.environ.get("COBOT_TRACE", "").lower() in ("1", "true", "yes"),
        )

    @classmethod
    def from_ip_file(cls, path: str | Path) -> "LinkConfig":
        """Read a bare IP from a file, the way SocketClient::parseNetworkConfig_ did."""
        return cls(ip=Path(path).read_text().strip())


@dataclass(frozen=True)
class JointLimits:
    """Per-joint software limits (robot_config_info.h::joint)."""

    vel_max: float = 1.0
    vel_min: float = -1.0
    effort_max: float = 60.0
    effort_min: float = -60.0
    jog_max: float = 3.0
    jog_min: float = -3.0
    position: tuple[tuple[float, float], ...] = (
        (-3.1, 3.1),
        (-1.5, 1.7),
        (-1.5, 3.3),
        (-2.9, 2.9),
        (-1.5, 1.5),
        (-6.26, 6.26),
    )


@dataclass(frozen=True)
class GripperLimits:
    force_min: float = 0.0
    force_max: float = 150.0


@dataclass(frozen=True)
class RobotLimits:
    joint: JointLimits = field(default_factory=JointLimits)
    gripper: GripperLimits = field(default_factory=GripperLimits)


DEFAULT_LIMITS = RobotLimits()
