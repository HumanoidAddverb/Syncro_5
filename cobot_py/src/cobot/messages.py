"""Wire structures — a 1:1 port of ui_data_types.h.

The C++ side serialises with struct_mapping's ``map_struct_to_json(s, ss, " ", false)``.
``hide_null = false`` means **unset optionals are emitted as explicit ``null``**, not
omitted. We reproduce that: ``to_dict()`` always emits every key.

JSON key names are authoritative and must match the C++ ``struct_mapping::reg`` names
exactly — they are written out in ``_KEYS`` next to each field.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional

from .config import N_DOF

Vec = list[float]
Mat = list[list[float]]


def _zeros(n: int = N_DOF) -> Vec:
    return [0.0] * n


def _opt(obj: Any) -> Any:
    """Serialise an optional sub-struct, preserving the C++ ``null`` for unset."""
    return None if obj is None else obj.to_dict()


# --------------------------------------------------------------------------- #
# Backend -> robot   (SocketTx and its children)
# --------------------------------------------------------------------------- #


@dataclass
class UIInterrupt:
    """SocketTx.interrupt — the mandatory action/error pair."""

    action: int = 0
    error: int = 0

    def to_dict(self) -> dict:
        return {"action": int(self.action), "error": int(self.error)}


@dataclass
class FTConfig:
    ft_type: int = 0
    ft_rotation_matrix: Vec = field(default_factory=list)  # 9 elements, row-major

    def to_dict(self) -> dict:
        return {
            "ft_type": int(self.ft_type),
            "ft_rotation_matrix": list(self.ft_rotation_matrix),
        }


@dataclass
class GripperConfig:
    gripper_type: int = 0

    def to_dict(self) -> dict:
        return {"gripper_type": int(self.gripper_type)}


@dataclass
class PayloadConfig:
    mass: float = 0.0
    com: Vec = field(default_factory=lambda: _zeros(3))
    moi: Vec = field(default_factory=lambda: _zeros(6))
    ft_config: Optional[FTConfig] = None
    gripper_config: Optional[GripperConfig] = None

    def to_dict(self) -> dict:
        return {
            "mass": float(self.mass),
            "com": list(self.com),
            "moi": list(self.moi),
            "ft_config": _opt(self.ft_config),
            "gripper_config": _opt(self.gripper_config),
        }


@dataclass
class ScriptConfig:
    label: str = ""
    iterations: int = 0
    mode: int = 0  # RecordingMode

    def to_dict(self) -> dict:
        return {
            "label": self.label,
            "iterations": int(self.iterations),
            "mode": int(self.mode),
        }


@dataclass
class ControllerConfig:
    """A one-shot goal for the robot. JSON key for ``target_pos`` is ``target_position``."""

    controller: int = 0  # API
    target_pos: Vec = field(default_factory=list)  # "target_position"
    target_time: float = 0.0
    flex_factor: float = 0.0  # "flexibility_factor"
    safety_mode: int = 0
    target_pos_seq: Mat = field(default_factory=list)  # "target_position_seq"
    target_time_seq: Vec = field(default_factory=list)
    script_config: Optional[ScriptConfig] = None

    def to_dict(self) -> dict:
        return {
            "controller": int(self.controller),
            "target_position": list(self.target_pos),
            "target_time": float(self.target_time),
            "flexibility_factor": float(self.flex_factor),
            "safety_mode": int(self.safety_mode),
            "target_position_seq": [list(p) for p in self.target_pos_seq],
            "target_time_seq": list(self.target_time_seq),
            "script_config": _opt(self.script_config),
        }


@dataclass
class RecordedDataEntry:
    desired_pos: Vec = field(default_factory=list)
    desired_time: float = 0.0

    def to_dict(self) -> dict:
        return {
            "desired_pos": list(self.desired_pos),
            "desired_time": float(self.desired_time),
        }


@dataclass
class RecordedDataSet:
    dataset: list[RecordedDataEntry] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {"dataset": [e.to_dict() for e in self.dataset]}


@dataclass
class AdvancedControllerConfig:
    dataset: Optional[RecordedDataSet] = None

    def to_dict(self) -> dict:
        return {"dataset": _opt(self.dataset)}


@dataclass
class ControlInterrupt:
    """Per-cycle command for the continuous controllers (velocity, torque, jogging).

    Which field the robot reads depends on the active controller:
      EXTERNAL_VELOCITY / EXTERNAL_TORQUE / GCOMP_EXTERNAL_TORQUE -> ``ext_ctrl``
      JOG_JOINT                                                   -> ``joint_jog``
      JOG_RPY / JOG_AXIS                                          -> ``cart_jog``
      JOINT_IMPEDANCE / CARTESIAN_IMPEDANCE                       -> the matrices
    """

    cart_jog: Vec = field(default_factory=list)  # "jog_cartesian"
    joint_jog: Vec = field(default_factory=list)  # "jog_joint"
    ext_ctrl: Vec = field(default_factory=list)  # "external_control"
    stiffness_matrix: Mat = field(default_factory=list)
    damping_matrix: Mat = field(default_factory=list)
    mass_matrix: Mat = field(default_factory=list)
    ft_force: Vec = field(default_factory=list)
    target_force: Vec = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "jog_cartesian": list(self.cart_jog),
            "jog_joint": list(self.joint_jog),
            "external_control": list(self.ext_ctrl),
            "stiffness_matrix": [list(r) for r in self.stiffness_matrix],
            "damping_matrix": [list(r) for r in self.damping_matrix],
            "mass_matrix": [list(r) for r in self.mass_matrix],
            "ft_force": list(self.ft_force),
            "target_force": list(self.target_force),
        }


@dataclass
class EventConfig:
    event: int = 0  # Event
    clamp_force: float = 0.0

    def to_dict(self) -> dict:
        return {"event": int(self.event), "clamp_force": float(self.clamp_force)}


@dataclass
class UserConfig:
    """SocketTx.config — slow-moving configuration, persists until cleared."""

    payload: Optional[PayloadConfig] = None
    controller_config: Optional[ControllerConfig] = None
    advanced_controller_config: Optional[AdvancedControllerConfig] = None

    def to_dict(self) -> dict:
        return {
            "payload": _opt(self.payload),
            "controller_config": _opt(self.controller_config),
            "advanced_controller_config": _opt(self.advanced_controller_config),
        }

    def clear(self) -> None:
        self.payload = None
        self.controller_config = None
        self.advanced_controller_config = None


@dataclass
class SocketTx:
    """The complete packet backend -> robot."""

    interrupt: UIInterrupt = field(default_factory=UIInterrupt)
    event_config: Optional[EventConfig] = None
    config: Optional[UserConfig] = None
    control: Optional[ControlInterrupt] = None  # JSON key "control_int"

    def to_dict(self) -> dict:
        return {
            "interrupt": self.interrupt.to_dict(),
            "event_config": _opt(self.event_config),
            "config": _opt(self.config),
            "control_int": _opt(self.control),
        }


# --------------------------------------------------------------------------- #
# Robot -> backend   (SocketRx and its children)
# --------------------------------------------------------------------------- #


@dataclass
class RobotInterrupt:
    action: int = 0
    state: int = 0  # RobotState
    error: int = 0

    @classmethod
    def from_dict(cls, d: dict | None) -> "RobotInterrupt":
        d = d or {}
        return cls(
            action=int(d.get("action", 0) or 0),
            state=int(d.get("state", 0) or 0),
            error=int(d.get("error", 0) or 0),
        )


@dataclass
class EventStatus:
    event: int = 0
    status: int = 0  # EventExecutionStatus

    @classmethod
    def from_dict(cls, d: dict | None) -> Optional["EventStatus"]:
        if not d:
            return None
        return cls(
            event=int(d.get("event", 0) or 0),
            status=int(d.get("status", 0) or 0),
        )


@dataclass
class RobotFeedback:
    """Live robot state. All vectors are N_DOF long except the end-effector pose (6)."""

    joint_position: Vec = field(default_factory=_zeros)
    joint_velocity: Vec = field(default_factory=_zeros)
    joint_acceleration: Vec = field(default_factory=_zeros)
    joint_torque: Vec = field(default_factory=_zeros)
    joint_error_code: Vec = field(default_factory=_zeros)
    joint_status: Vec = field(default_factory=_zeros)
    end_effector_position: Vec = field(default_factory=lambda: _zeros(6))
    force_magnitude: Vec = field(default_factory=list)
    force_direction: Vec = field(default_factory=list)
    time_stamp: float = 0.0

    _KEYS = {
        "joint_position": "joint_position",
        "joint_velocity": "joint_velocity",
        "joint_acceleration": "joint_acceleration",
        "joint_torque": "joint_torque",
        "joint_error_code": "joint_error_code",
        "joint_status": "joint_status",
        "end_effector_position": "end_effector_position",
        "force_magnitude": "force_magnitude",
        "force_direction": "force_direction",
    }

    @classmethod
    def from_dict(cls, d: dict | None) -> Optional["RobotFeedback"]:
        if not d:
            return None
        out = cls()
        for attr, key in cls._KEYS.items():
            val = d.get(key)
            if val is not None:
                setattr(out, attr, [float(x) for x in val])
        out.time_stamp = float(d.get("time_stamp", 0.0) or 0.0)
        return out


@dataclass
class SocketRx:
    """The complete packet robot -> backend."""

    interrupt: RobotInterrupt = field(default_factory=RobotInterrupt)
    event_status: Optional[EventStatus] = None
    feedback: Optional[RobotFeedback] = None

    @classmethod
    def from_dict(cls, d: dict | None) -> "SocketRx":
        d = d or {}
        return cls(
            interrupt=RobotInterrupt.from_dict(d.get("interrupt")),
            event_status=EventStatus.from_dict(d.get("event_status")),
            feedback=RobotFeedback.from_dict(d.get("feedback")),
        )
