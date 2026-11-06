"""Pure-Python driver for the Addverb cobot.

Talks the same framed-JSON TCP protocol the C++ backend spoke, with no ROS 2 and
no compiled dependencies. Standard library only.

    from cobot import Cobot, API

    with Cobot("192.168.0.12") as bot:
        bot.power_on()
        bot.move_to([0, 0, 0, 0, 0, 0], duration=10)

Layering, outermost first:

    robot.Cobot        high-level API          <- cobot_comm_wrapper.cpp
    session.RobotSession  TX/RX struct state   <- data_handler.cpp
    link.RobotLink     background pump thread  <- network_manager.h
    transport.FramedSocket  '#...#' TCP frames <- socket_client / socket_comm_wrapper
"""

from .config import DEFAULT_LIMITS, N_DOF, GripperLimits, JointLimits, LinkConfig, RobotLimits
from .enums import (
    CONTINUOUS_APIS,
    CRITICAL_APIS,
    API,
    ErrorCode,
    Event,
    EventExecutionStatus,
    FTType,
    GripperType,
    HWStatus,
    HealError,
    RecordingMode,
    RobotAction,
    RobotState,
    SafetyMode,
    describe_robot_error,
)
from .errors import (
    CobotError,
    CommandTimeout,
    ConnectFailed,
    ConnectionLost,
    InvalidState,
    PowerOnFailed,
    RobotInError,
    ValidationError,
)
from .messages import (
    AdvancedControllerConfig,
    ControlInterrupt,
    ControllerConfig,
    EventConfig,
    EventStatus,
    FTConfig,
    GripperConfig,
    PayloadConfig,
    RecordedDataEntry,
    RecordedDataSet,
    RobotFeedback,
    ScriptConfig,
    SocketRx,
    SocketTx,
    UserConfig,
)
from .robot import ControlStream, Cobot
from .session import RobotSession

__version__ = "0.1.0"

__all__ = [
    # entry points
    "Cobot",
    "ControlStream",
    "RobotSession",
    # config
    "LinkConfig",
    "RobotLimits",
    "JointLimits",
    "GripperLimits",
    "DEFAULT_LIMITS",
    "N_DOF",
    # enums
    "API",
    "CONTINUOUS_APIS",
    "CRITICAL_APIS",
    "RobotState",
    "RobotAction",
    "Event",
    "EventExecutionStatus",
    "HWStatus",
    "HealError",
    "describe_robot_error",
    "RecordingMode",
    "SafetyMode",
    "GripperType",
    "FTType",
    "ErrorCode",
    # messages
    "ControllerConfig",
    "ControlInterrupt",
    "AdvancedControllerConfig",
    "EventConfig",
    "EventStatus",
    "PayloadConfig",
    "GripperConfig",
    "FTConfig",
    "ScriptConfig",
    "RecordedDataEntry",
    "RecordedDataSet",
    "RobotFeedback",
    "UserConfig",
    "SocketTx",
    "SocketRx",
    # errors
    "CobotError",
    "ConnectFailed",
    "ConnectionLost",
    "CommandTimeout",
    "RobotInError",
    "InvalidState",
    "PowerOnFailed",
    "ValidationError",
]
