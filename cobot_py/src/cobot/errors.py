"""Exceptions raised by the Python cobot stack.

The C++ layers returned ``error_codes``; here failures raise, so raw scripts do not
have to check a return value after every call. ``ErrorCode`` is still carried on the
exception for parity with the C++ API.
"""

from __future__ import annotations

from typing import Optional

from .enums import ErrorCode, RobotState


class CobotError(Exception):
    """Base class for every failure in this package."""

    code: ErrorCode = ErrorCode.INVALID_REQUEST

    def __init__(self, message: str, code: Optional[ErrorCode] = None):
        super().__init__(message)
        if code is not None:
            self.code = code


class ConnectionLost(CobotError):
    code = ErrorCode.CONNECTION_BROKEN


class ConnectFailed(CobotError):
    code = ErrorCode.FAILED_TO_CONNECT


class CommandTimeout(CobotError):
    """The robot did not reach the expected state within ``LinkConfig.command_timeout``."""

    code = ErrorCode.FAILED_TO_READ_DATA


class RobotInError(CobotError):
    code = ErrorCode.IN_ERROR_STATE

    def __init__(self, message: str, robot_error: int = 0):
        super().__init__(message)
        self.robot_error = robot_error


class InvalidState(CobotError):
    code = ErrorCode.INVALID_STATE

    def __init__(self, expected: RobotState, actual: RobotState):
        super().__init__(f"robot is in {actual.name}, expected {expected.name}")
        self.expected = expected
        self.actual = actual


class PowerOnFailed(CobotError):
    code = ErrorCode.ROBOT_FAILED_TO_POWER_ON


class ValidationError(CobotError):
    """A command was rejected locally, before it reached the robot."""

    code = ErrorCode.INVALID_REQUEST
