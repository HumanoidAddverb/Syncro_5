"""Local command validation — the port of utility/src/data_validator.cpp.

Runs before anything hits the wire, so a bad script fails in Python rather than
putting the robot into an error state.
"""

from __future__ import annotations

from collections.abc import Sequence

from .config import DEFAULT_LIMITS, N_DOF, RobotLimits
from .enums import FTType, GripperType, SafetyMode
from .errors import ValidationError


def _check_len(values: Sequence[float], n: int, what: str) -> list[float]:
    vals = [float(v) for v in values]
    if len(vals) != n:
        raise ValidationError(f"{what} must have {n} elements, got {len(vals)}")
    return vals


def check_velocity(cmd: Sequence[float], limits: RobotLimits = DEFAULT_LIMITS) -> list[float]:
    vals = _check_len(cmd, N_DOF, "joint velocity")
    lo, hi = limits.joint.vel_min, limits.joint.vel_max
    for i, v in enumerate(vals):
        if not lo <= v <= hi:
            raise ValidationError(
                f"joint {i + 1} velocity {v} outside [{lo}, {hi}] rad/s"
            )
    return vals


def check_effort(cmd: Sequence[float], limits: RobotLimits = DEFAULT_LIMITS) -> list[float]:
    vals = _check_len(cmd, N_DOF, "joint effort")
    lo, hi = limits.joint.effort_min, limits.joint.effort_max
    for i, v in enumerate(vals):
        if not lo <= v <= hi:
            raise ValidationError(f"joint {i + 1} effort {v} outside [{lo}, {hi}] Nm")
    if all(abs(v) <= 1e-4 for v in vals):
        # DataValidator flagged this: a fully zero torque command drops the arm.
        raise ValidationError(
            "commanded effort is ~0 on every joint; the robot would fall. "
            "Send gravity-compensated torques instead."
        )
    return vals


def check_jog(cmd: Sequence[float], limits: RobotLimits = DEFAULT_LIMITS) -> list[float]:
    vals = _check_len(cmd, N_DOF, "jog command")
    lo, hi = limits.joint.jog_min, limits.joint.jog_max
    for i, v in enumerate(vals):
        if not lo <= v <= hi:
            raise ValidationError(f"jog element {i + 1} value {v} outside [{lo}, {hi}]")
    return vals


def check_joint_positions(
    pos: Sequence[float], limits: RobotLimits = DEFAULT_LIMITS
) -> list[float]:
    vals = _check_len(pos, N_DOF, "target position")
    for i, (v, (lo, hi)) in enumerate(zip(vals, limits.joint.position)):
        if not lo <= v <= hi:
            raise ValidationError(
                f"joint {i + 1} target {v} outside limits [{lo}, {hi}] rad"
            )
    return vals


def check_duration(seconds: float) -> float:
    if seconds <= 0:
        raise ValidationError(f"duration must be > 0, got {seconds}")
    return float(seconds)


def check_payload(
    mass: float, com: Sequence[float], moi: Sequence[float]
) -> tuple[float, list[float], list[float]]:
    if mass <= 0:
        raise ValidationError(f"payload mass must be > 0, got {mass}")
    com_v = _check_len(com, 3, "payload centre of mass")
    moi_v = _check_len(moi, 6, "payload moment of inertia")
    if any(v < 0 for v in moi_v):
        raise ValidationError("payload moment of inertia values must be >= 0")
    return float(mass), com_v, moi_v


def check_grasp_force(force: float, limits: RobotLimits = DEFAULT_LIMITS) -> float:
    lo, hi = limits.gripper.force_min, limits.gripper.force_max
    if not lo <= force <= hi:
        raise ValidationError(f"grasp force {force} outside [{lo}, {hi}] N")
    return float(force)


def check_ft_rotation(matrix: Sequence[float]) -> list[float]:
    return _check_len(matrix, 9, "FT rotation matrix")


def check_enum(value: int, enum_cls, what: str) -> int:
    try:
        return int(enum_cls(value))
    except ValueError as exc:
        allowed = ", ".join(m.name for m in enum_cls)
        raise ValidationError(f"invalid {what}: {value}. Allowed: {allowed}") from exc


def check_safety(mode: int) -> int:
    return check_enum(mode, SafetyMode, "safety mode")


def check_gripper_type(gripper: int) -> int:
    return check_enum(gripper, GripperType, "gripper type")


def check_ft_type(ft: int) -> int:
    return check_enum(ft, FTType, "FT sensor type")
