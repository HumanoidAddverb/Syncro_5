"""Enumerations mirrored from the C++ backend.

Sources:
  api_types.h            -> API
  robot_fsm_enums.h      -> RobotState, RobotAction
  event_types.h          -> Event, EventExecutionStatus
  ui_data_types.h        -> HWStatus, RecordingMode
  robot_config_info.h    -> SafetyMode, GripperType, FTType
  ros_wrapper_error_codes.h -> ErrorCode

These integers go on the wire. Do not renumber them.
"""

from enum import IntEnum


class API(IntEnum):
    """Controller / API id sent as ``controller_config.controller``."""

    NONE = 0
    JOG_AXIS = 1
    FLEX_JOG_AXIS = 2
    JOG_RPY = 3
    FLEX_JOG_RPY = 4
    JOG_JOINT = 5
    FLEX_JOG_JOINT = 6
    LINEAR_VELOCITY = 7
    FLEX_LINEAR_VELOCITY = 8
    FREE_DRIVE = 9
    MULTI_POINT = 10
    CUSTOM_1 = 11
    CUSTOM_2 = 12
    CUSTOM_3 = 13
    CUSTOM_4 = 14
    CUSTOM_5 = 15
    CUSTOM_6 = 16
    PLAY_REC = 17
    EXTERNAL_VELOCITY = 18
    EXTERNAL_TORQUE = 19
    JOINT_IMPEDANCE = 20
    MULTI_JOINT_IMPEDANCE = 21
    CARTESIAN_IMPEDANCE = 22
    TCP_MULTI_POINT = 23
    GCOMP_EXTERNAL_TORQUE = 24


#: Controllers driven by a per-cycle ``control_int`` stream rather than a one-shot goal.
#: Mirrors CobotCommWrapper::continous_ctrl_api_.
CONTINUOUS_APIS = frozenset(
    {
        API.EXTERNAL_TORQUE,
        API.EXTERNAL_VELOCITY,
        API.GCOMP_EXTERNAL_TORQUE,
        API.JOG_JOINT,
        API.JOG_RPY,
        API.FREE_DRIVE,
    }
)

#: Controllers that must be handed a valid first command *while* switching,
#: otherwise the robot free-falls. Mirrors CobotCommWrapper::critical_controllers_api_.
CRITICAL_APIS = frozenset({API.EXTERNAL_TORQUE})


class RobotState(IntEnum):
    NONE = 0
    BASE = 1
    MOTION = 2
    ERROR = 3


class RobotAction(IntEnum):
    NONE = 0
    GO_TO_BASE = 1
    GO_TO_MOTION = 2
    GO_TO_ERROR = 3


class Event(IntEnum):
    NONE = 0
    RESET_CONFIG = 1
    OPEN_GRIPPER = 2
    CLOSE_GRIPPER = 3
    DISABLE_ROBOT = 4


class EventExecutionStatus(IntEnum):
    NONE = 0
    EXECUTING = 1
    FAILED = 2
    SUCCEEDED = 3


class HWStatus(IntEnum):
    NONE = 0
    DISCONNECTED = 1
    CONNECTED = 2
    CONFIGURED = 3
    MOVING = 4
    WAITING = 5
    ERROR = 6
    RECORDING = 7


class RecordingMode(IntEnum):
    NONE = 0
    CONTINUOUS = 1
    INTERMEDIATE = 2


class SafetyMode(IntEnum):
    NOTHING = 0
    NO_HUMAN_SAFETY = 1
    NO_POS_SAFETY = 2  # also used as the error-recovery safety mode
    # 3 is eMaxRegime in the C++, an end-of-enum count; the robot rejects it.


class GripperType(IntEnum):
    NONE = 0
    DYNAMIXEL = 1
    ROBOTIQ = 2
    DH = 3
    SUCTION = 4
    FEETECH = 5
    INSPIRE = 6


class FTType(IntEnum):
    NONE = 0
    ROBOTIQ = 1
    ROBOTOUS = 2


#: Base values from error_codes.h. The enum below is contiguous from each base,
#: so the arithmetic is spelled out rather than hard-coded.
_ROBOT_BASE = 1
_HARDWARE_BASE = 1000 + _ROBOT_BASE  # 1001
_SOCKET_BASE = 1000 + _HARDWARE_BASE  # 2001
_ROBOT_FSM_BASE = 500 + _SOCKET_BASE  # 2501


class HealError(IntEnum):
    """Errors reported *by the robot* in ``interrupt.error`` (HealErrors::Code).

    Distinct from :class:`ErrorCode`, which is this client's own return codes.
    """

    NONE = 0

    # ---- TCP/IP socket errors (2001+) ----
    CONNECTION_LOST = _SOCKET_BASE
    DATA_LOSS = _SOCKET_BASE + 1
    FAILED_TO_CONNECT = _SOCKET_BASE + 2
    FAILED_TO_DISCONNECT = _SOCKET_BASE + 3

    # ---- Robot FSM errors (2501+) ----
    DISCONNECTED = _ROBOT_FSM_BASE
    FAILED_TO_SETUP_ROBOT = _ROBOT_FSM_BASE + 1
    FAILED_TO_RUN_ROBOT = _ROBOT_FSM_BASE + 2
    FAILED_TO_SETUP_STATE = _ROBOT_FSM_BASE + 3
    FAILED_TO_CLEAR_RESOURCES = _ROBOT_FSM_BASE + 4
    INVALID_CONFIGURATION_DATA = _ROBOT_FSM_BASE + 5
    INVALID_CONTROL_INPUT = _ROBOT_FSM_BASE + 6
    UNSUPPORTED_API = _ROBOT_FSM_BASE + 7
    BAD_JSON = _ROBOT_FSM_BASE + 8
    FAILED_TO_STOP_ROBOT = _ROBOT_FSM_BASE + 9
    FAILED_TO_POWER_OFF_ROBOT = _ROBOT_FSM_BASE + 10
    INVALID_ACTION = _ROBOT_FSM_BASE + 11
    INVALID_EVENT = _ROBOT_FSM_BASE + 12
    INVALID_SAFETY_TYPE = _ROBOT_FSM_BASE + 13
    INVALID_GRIPPER_TYPE = _ROBOT_FSM_BASE + 14
    INVALID_FT_TYPE = _ROBOT_FSM_BASE + 15
    INVALID_PAYLOAD_MASS = _ROBOT_FSM_BASE + 16
    INVALID_PAYLOAD_COM = _ROBOT_FSM_BASE + 17
    INVALID_PAYLOAD_MOI = _ROBOT_FSM_BASE + 18

    # ---- Data manager errors ----
    FAILED_TO_GET_ACTION = _ROBOT_FSM_BASE + 19
    FAILED_TO_GET_EVENT = _ROBOT_FSM_BASE + 20
    FAILED_TO_SET_EVENT_STATUS = _ROBOT_FSM_BASE + 21
    FAILED_TO_READ_DATA = _ROBOT_FSM_BASE + 22
    FAILED_TO_WRITE_DATA = _ROBOT_FSM_BASE + 23
    FAILED_TO_GET_ROBOT_DATA = _ROBOT_FSM_BASE + 24
    FAILED_TO_GET_CLAMP_FORCE = _ROBOT_FSM_BASE + 25
    FAILED_TO_SET_ROBOT_DATA = _ROBOT_FSM_BASE + 26
    FAILED_TO_SET_USER_DATA = _ROBOT_FSM_BASE + 27
    FAILED_TO_GET_API = _ROBOT_FSM_BASE + 28
    FAILED_TO_SETUP_NETWORK = _ROBOT_FSM_BASE + 29
    FAILED_TO_SHUTDOWN_NETWORK = _ROBOT_FSM_BASE + 30

    # ---- Data handler / store ----
    EMPTY_INPUT = _ROBOT_FSM_BASE + 31
    NON_UNIQUE_ID = _ROBOT_FSM_BASE + 32

    # ---- Event handler ----
    FAILED_TO_RESET_CONFIG = _ROBOT_FSM_BASE + 33
    FAILED_TO_OPEN_GRIPPER = _ROBOT_FSM_BASE + 34
    FAILED_TO_CLOSE_GRIPPER = _ROBOT_FSM_BASE + 35


#: Extra context for the codes whose name alone does not say what to do.
_HEAL_HINTS = {
    HealError.FAILED_TO_SETUP_STATE: (
        "the robot could not bring up the requested state (e.g. motion). Its own "
        "log shows how far it got; clear the error before retrying"
    ),
    HealError.INVALID_CONFIGURATION_DATA: "the robot rejected the config we sent",
    HealError.INVALID_CONTROL_INPUT: "a per-cycle command was out of range or wrong length",
    HealError.UNSUPPORTED_API: "the robot does not implement the requested controller id",
    HealError.BAD_JSON: "the robot could not parse our packet",
    HealError.INVALID_SAFETY_TYPE: "safety_mode is not one of SafetyMode",
    HealError.INVALID_PAYLOAD_MASS: "payload mass rejected (must be > 0)",
    HealError.INVALID_PAYLOAD_COM: "payload centre of mass rejected (needs 3 values)",
    HealError.INVALID_PAYLOAD_MOI: "payload inertia rejected (needs 6 non-negative values)",
    HealError.CONNECTION_LOST: "the robot considers the link dead",
}


def describe_robot_error(code: int) -> str:
    """Render a robot-reported ``interrupt.error`` as something readable."""
    if not code:
        return "none"
    try:
        err = HealError(code)
    except ValueError:
        return f"unknown robot error {code}"
    hint = _HEAL_HINTS.get(err)
    return f"{err.name} ({code}){f' - {hint}' if hint else ''}"


class ErrorCode(IntEnum):
    """Return codes from the high-level wrapper (ros_wrapper_error_codes.h)."""

    NO_ERROR = 0
    INVALID_REQUEST = 1
    DATA_NOT_PRESENT = 2
    ROBOT_FAILED_TO_SETUP = 3
    ROBOT_FAILED_TO_POWER_ON = 4
    CONNECTION_BROKEN = 5
    IN_ERROR_STATE = 6
    FAILED_TO_CONNECT = 7
    FAILED_TO_DISCONNECT = 8
    FAILED_TO_WRITE_DATA = 9
    FAILED_TO_CLEAR_ERROR = 10
    FAILED_TO_READ_DATA = 11
    INVALID_SAFETY_TYPE = 12
    INVALID_GRIPPER_TYPE = 13
    INVALID_FT_TYPE = 14
    INVALID_FT_ROTATION_MATRIX = 15
    REPLAY_ERROR = 16
    INVALID_VELOCITY = 17
    INVALID_EFFORT = 18
    INVALID_GRIPPER_CMD = 19
    INVALID_STATE = 20
