"""The public API — the port of utility/src/cobot_comm_wrapper.cpp.

Everything a raw Python script needs:

    from cobot import Cobot, API

    with Cobot("192.168.0.12") as bot:
        bot.power_on()
        bot.move_to([0, 0, 0, 0, 0, 0], duration=10)
        print(bot.feedback().joint_position)

Differences from the C++ original, all deliberate:

* every wait is bounded by ``LinkConfig.command_timeout`` (the C++ loops were
  unbounded and would hang forever on a wedged robot);
* waits block on a *fresh* packet instead of busy-spinning on the last one;
* failures raise instead of returning an ``error_codes`` the caller may ignore.
"""

from __future__ import annotations

import logging
import time
from collections.abc import Sequence
from contextlib import contextmanager
from typing import Callable, Iterator, Optional

from . import limits as check
from .config import DEFAULT_LIMITS, N_DOF, LinkConfig, RobotLimits
from .enums import (
    CONTINUOUS_APIS,
    CRITICAL_APIS,
    API,
    ErrorCode,
    Event,
    EventExecutionStatus,
    FTType,
    GripperType,
    RobotAction,
    RobotState,
    SafetyMode,
    describe_robot_error,
)
from .errors import (
    CommandTimeout,
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
    FTConfig,
    GripperConfig,
    PayloadConfig,
    RecordedDataEntry,
    RecordedDataSet,
    RobotFeedback,
    ScriptConfig,
)
from .session import RobotSession

log = logging.getLogger(__name__)

#: Safety mode the robot is powered on with during error recovery
#: (SAFETY_MODE_ERR_RECOVERY in cobot_comm_wrapper.h).
SAFETY_MODE_ERR_RECOVERY = int(SafetyMode.NO_POS_SAFETY)


class Cobot:
    """A connected Addverb cobot."""

    def __init__(
        self,
        ip: Optional[str] = None,
        port: Optional[int] = None,
        *,
        config: Optional[LinkConfig] = None,
        limits: RobotLimits = DEFAULT_LIMITS,
        connect: bool = True,
    ):
        cfg = config or LinkConfig.from_env()
        if ip is not None:
            cfg.ip = ip
        if port is not None:
            cfg.port = port

        self.cfg = cfg
        self.limits = limits
        self._s = RobotSession(cfg)

        self._controller: API = API.NONE
        self._safety: int = int(SafetyMode.NO_HUMAN_SAFETY)  # as payload_config.yaml
        self._payload: Optional[PayloadConfig] = None
        self._gripper: Optional[GripperConfig] = None
        self._ft: Optional[FTConfig] = None
        self._last_control = ControlInterrupt()

        if connect:
            self.connect()

    # ------------------------------------------------------------------ #
    # connection
    # ------------------------------------------------------------------ #

    def connect(self) -> None:
        self._s.connect()
        self._s.read_fresh()
        log.info("connected to cobot at %s:%s", self.cfg.ip, self.cfg.port)

    def disconnect(self) -> None:
        self._s.shutdown()

    @property
    def is_connected(self) -> bool:
        return self._s.is_connected

    def __enter__(self) -> "Cobot":
        if not self.is_connected:
            self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        # Best effort: bring the arm to a stop before dropping the link, even if
        # the body of the with-block raised.
        try:
            if self.is_connected and self.state == RobotState.MOTION:
                self.power_off()
        except Exception:  # noqa: BLE001 - never mask the original exception
            log.exception("power off during shutdown failed")
        finally:
            self.disconnect()

    # ------------------------------------------------------------------ #
    # state / feedback
    # ------------------------------------------------------------------ #

    @property
    def state(self) -> RobotState:
        self._s.read()
        return self._s.state

    def feedback(self, fresh: bool = True) -> RobotFeedback:
        """Latest joint positions, velocities, torques and end-effector pose."""
        if fresh:
            self._s.read_fresh()
        else:
            self._s.read()
        return self._s.feedback

    @property
    def joint_positions(self) -> list[float]:
        return self.feedback().joint_position

    # ------------------------------------------------------------------ #
    # setup (setters — take effect at the next power_on)
    # ------------------------------------------------------------------ #

    def set_safety(self, mode: SafetyMode | int) -> None:
        self._safety = check.check_safety(int(mode))

    def set_gripper_type(self, gripper: GripperType | int) -> None:
        self._gripper = GripperConfig(gripper_type=check.check_gripper_type(int(gripper)))
        if self._payload is not None:
            self._payload.gripper_config = self._gripper

    def set_ft_sensor(
        self, ft_type: FTType | int, rotation_matrix: Optional[Sequence[float]] = None
    ) -> None:
        self._ft = FTConfig(
            ft_type=check.check_ft_type(int(ft_type)),
            ft_rotation_matrix=check.check_ft_rotation(rotation_matrix)
            if rotation_matrix is not None
            else [1, 0, 0, 0, 1, 0, 0, 0, 1],
        )
        if self._payload is not None:
            self._payload.ft_config = self._ft

    def set_payload(
        self, mass: float, com: Sequence[float], moi: Sequence[float]
    ) -> None:
        """Declare the end-effector load so gravity compensation stays correct."""
        mass, com_v, moi_v = check.check_payload(mass, com, moi)
        self._payload = PayloadConfig(mass=mass, com=com_v, moi=moi_v)
        # setPayload() in the C++ folds the already-known gripper/FT config in.
        if self._gripper is not None:
            self._payload.gripper_config = self._gripper
        if self._ft is not None:
            self._payload.ft_config = self._ft

    # ------------------------------------------------------------------ #
    # power
    # ------------------------------------------------------------------ #

    def power_on(self) -> None:
        """Take the robot from BASE to MOTION. Port of ``powerOnRobot``.

        Requires the robot to start in BASE. If it is already in ERROR, clear it
        first (:meth:`clear_error`, or :meth:`recover` to also home the arm) --
        otherwise the wait below would fall straight through and report the
        pre-existing fault as if we had caused it.
        """
        entry_state = self.state
        if entry_state == RobotState.ERROR:
            raise RobotInError(
                f"robot is already in ERROR before power on: "
                f"{describe_robot_error(self._s.robot_error)}. "
                f"Call clear_error() (or recover()) first.",
                self._s.robot_error,
            )
        if entry_state == RobotState.MOTION:
            log.info("already in MOTION; nothing to power on")
            return

        cfg = ControllerConfig(
            controller=int(API.EXTERNAL_VELOCITY),
            safety_mode=self._safety,
        )
        if self._payload is not None:
            self._s.append_payload(self._payload)

        self._s.append_controller(cfg)
        self._s.commit_config()

        def rearm() -> None:
            self._s.set_action(RobotAction.GO_TO_MOTION)

        self._pump_until(
            lambda: self._s.state != RobotState.BASE,
            rearm=rearm,
            what="power on",
        )

        if self._s.state == RobotState.ERROR:
            raise PowerOnFailed(
                f"robot went to ERROR while powering on: "
                f"{describe_robot_error(self._s.robot_error)}"
            )
        self._s.clear_config()
        self._s.write()
        log.info("powered on (state=%s)", self._s.state.name)

    def power_off(self) -> None:
        """Take the robot back to BASE and drop the link. Port of ``powerOffRobot``."""
        try:
            self._pump_until(
                lambda: self._s.state != RobotState.MOTION,
                rearm=lambda: self._s.set_action(RobotAction.GO_TO_BASE),
                what="power off",
            )
            state = self._s.state
        finally:
            self._s.shutdown()

        if state == RobotState.ERROR:
            raise RobotInError(
                f"robot is in ERROR after power off: "
                f"{describe_robot_error(self._s.robot_error)}",
                self._s.robot_error,
            )
        log.info("powered off")

    # ------------------------------------------------------------------ #
    # error handling
    # ------------------------------------------------------------------ #

    def clear_error(self) -> None:
        """Drive the robot out of ERROR back to BASE. Port of ``clearErrorState``."""
        self._pump_until(
            lambda: self._s.state != RobotState.ERROR,
            rearm=lambda: self._s.set_action(RobotAction.GO_TO_BASE),
            what="clear error",
        )

        if self._s.state == RobotState.MOTION:
            raise RobotInError("failed to clear error: robot is still in MOTION")

        self._s.clear_config()
        self._s.write()
        log.info("error cleared (state=%s)", self._s.state.name)

    def recover(self) -> None:
        """Full recovery: clear the error, power on in recovery safety, drive home, stop.

        Port of ``powerOnRobotErrorRecovery`` + ``executeErrorRecovery``.
        """
        if self.state == RobotState.ERROR:
            self.clear_error()

        saved_safety = self._safety
        self._safety = SAFETY_MODE_ERR_RECOVERY
        try:
            self.power_on()
        finally:
            self._safety = saved_safety

        home = ControllerConfig(
            controller=int(API.LINEAR_VELOCITY),
            target_pos=[0.0] * N_DOF,
            target_time=15.0,
        )
        self._run_goal(home)
        self._await_event_done(what="error recovery homing")

        self._pump_until(
            lambda: self._s.state != RobotState.MOTION,
            rearm=lambda: self._s.set_action(RobotAction.GO_TO_BASE),
            what="error recovery stop",
        )
        if self._s.state == RobotState.ERROR:
            raise RobotInError(
                f"error recovery ended in ERROR: "
                f"{describe_robot_error(self._s.robot_error)}",
                self._s.robot_error,
            )
        log.info("error recovery complete")

    # ------------------------------------------------------------------ #
    # controllers
    # ------------------------------------------------------------------ #

    @property
    def controller(self) -> API:
        return self._controller

    def switch_controller(
        self, controller: API | int, first_command: Optional[ControlInterrupt] = None
    ) -> None:
        """Activate a controller. Port of ``switchController``.

        Continuous controllers need a handshake: the robot acknowledges with
        EXECUTING and then SUCCEEDED before it will accept a command stream.
        For the critical ones (external torque) a valid ``first_command`` must be
        streamed *during* the switch or the arm drops.
        """
        api = API(int(controller))
        self._controller = api

        if api not in CONTINUOUS_APIS:
            # One-shot controllers carry their config with the goal itself.
            return

        if api in CRITICAL_APIS and first_command is None:
            raise ValidationError(
                f"{api.name} is a critical controller: pass first_command with a "
                "gravity-compensated torque, otherwise the robot will fall on switch"
            )

        def rearm() -> None:
            if first_command is not None:
                self._s.set_control(first_command)

        self._s.append_controller(ControllerConfig(controller=int(api)))
        self._s.commit_config()
        self._s.set_event(EventConfig(event=int(Event.RESET_CONFIG)))
        rearm()

        self._pump_until(
            lambda: self._s.event_status.status == int(EventExecutionStatus.EXECUTING),
            rearm=rearm,
            what=f"switch to {api.name} (executing)",
        )

        self._s.clear_event()
        self._s.clear_config()

        self._pump_until(
            lambda: self._s.event_status.status == int(EventExecutionStatus.SUCCEEDED),
            rearm=rearm,
            what=f"switch to {api.name} (succeeded)",
        )
        log.info("controller active: %s", api.name)

    # -- one-shot goals ------------------------------------------------- #

    def move_to(
        self,
        position: Sequence[float],
        duration: float,
        *,
        controller: API = API.LINEAR_VELOCITY,
        wait: bool = True,
    ) -> None:
        """Move all joints to ``position`` (rad) over ``duration`` seconds."""
        goal = ControllerConfig(
            controller=int(controller),
            target_pos=check.check_joint_positions(position, self.limits),
            target_time=check.check_duration(duration),
        )
        self._controller = API(int(controller))
        self._run_goal(goal)
        if wait:
            self._await_event_done(what="move_to", timeout=duration + self.cfg.command_timeout)

    def move_through(
        self,
        positions: Sequence[Sequence[float]],
        times: Sequence[float],
        *,
        controller: API = API.MULTI_POINT,
        wait: bool = True,
    ) -> None:
        """Move through a sequence of waypoints.

        ``times`` are cumulative timestamps from the start of the motion, one per
        waypoint — the same convention as ``target_time_seq`` on the C++ side.
        """
        if len(positions) != len(times):
            raise ValidationError(
                f"got {len(positions)} waypoints but {len(times)} times; they must match"
            )
        if not positions:
            raise ValidationError("need at least one waypoint")
        if any(b <= a for a, b in zip(times, times[1:])):
            raise ValidationError("times must be strictly increasing")

        goal = ControllerConfig(
            controller=int(controller),
            target_pos_seq=[check.check_joint_positions(p, self.limits) for p in positions],
            target_time_seq=[check.check_duration(t) for t in times],
            target_time=float(times[-1]),
        )
        self._controller = API(int(controller))
        self._run_goal(goal)
        if wait:
            self._await_event_done(
                what="move_through", timeout=float(times[-1]) + self.cfg.command_timeout
            )

    def replay(self, dataset: Sequence[RecordedDataEntry], label: str, iterations: int = 1) -> None:
        """Play back a recorded motion. Port of ``setCmd(controller, advanced)``."""
        script = ScriptConfig(label=label, iterations=int(iterations))
        goal = ControllerConfig(controller=int(API.PLAY_REC), script_config=script)

        self._s.append_advanced(AdvancedControllerConfig(dataset=RecordedDataSet(list(dataset))))
        self._s.append_script(script)
        self._s.append_controller(goal)
        self._s.commit_config()
        self._s.set_event(EventConfig(event=int(Event.RESET_CONFIG)))
        self._s.write()

        self._pump_until(
            lambda: self._s.event_status.status == int(EventExecutionStatus.EXECUTING),
            what="replay start",
        )
        self._s.clear_event()
        self._s.clear_config()
        self._s.write()

    # -- continuous command streams -------------------------------------- #

    def send_control(self, control: ControlInterrupt) -> None:
        """Push one cycle of a continuous command. Port of ``setCmd(ControlInterrupt)``."""
        self._last_control = control
        self._s.set_control(control)
        self._s.write()

    def set_velocity(self, velocity: Sequence[float]) -> None:
        """One cycle of joint velocity (rad/s) for API.EXTERNAL_VELOCITY."""
        self.send_control(
            ControlInterrupt(ext_ctrl=check.check_velocity(velocity, self.limits))
        )

    def set_torque(self, torque: Sequence[float]) -> None:
        """One cycle of joint torque (Nm) for API.EXTERNAL_TORQUE / GCOMP_EXTERNAL_TORQUE."""
        self.send_control(ControlInterrupt(ext_ctrl=check.check_effort(torque, self.limits)))

    def jog_joint(self, jog: Sequence[float]) -> None:
        """One cycle of joint-space jogging for API.JOG_JOINT."""
        self.send_control(ControlInterrupt(joint_jog=check.check_jog(jog, self.limits)))

    def jog_cartesian(self, jog: Sequence[float]) -> None:
        """One cycle of cartesian jogging for API.JOG_RPY / API.JOG_AXIS."""
        self.send_control(ControlInterrupt(cart_jog=check.check_jog(jog, self.limits)))

    def set_impedance(
        self,
        stiffness: Sequence[Sequence[float]],
        damping: Sequence[Sequence[float]],
        mass: Optional[Sequence[Sequence[float]]] = None,
        target_force: Optional[Sequence[float]] = None,
    ) -> None:
        """One cycle of impedance gains for the joint/cartesian impedance controllers."""
        self.send_control(
            ControlInterrupt(
                stiffness_matrix=[list(r) for r in stiffness],
                damping_matrix=[list(r) for r in damping],
                mass_matrix=[list(r) for r in (mass or [])],
                target_force=list(target_force or []),
            )
        )

    @contextmanager
    def control_stream(
        self, controller: API | int, rate_hz: float = 100.0, first_command: Optional[ControlInterrupt] = None
    ) -> Iterator["ControlStream"]:
        """Switch to a continuous controller and hand back a rate-limited streamer.

            with bot.control_stream(API.EXTERNAL_VELOCITY) as stream:
                for _ in stream.ticks(seconds=5):
                    bot.set_velocity([0, 0.1, 0, 0, 0, 0])
        """
        self.switch_controller(controller, first_command=first_command)
        stream = ControlStream(self, rate_hz)
        try:
            yield stream
        finally:
            # Leave the arm commanded to a stop rather than at the last velocity.
            try:
                self.send_control(ControlInterrupt(ext_ctrl=[0.0] * N_DOF))
            except ConnectionLost:
                pass

    # -- events (gripper etc.) ------------------------------------------- #

    def send_event(self, event: EventConfig) -> None:
        """Fire an event and wait for the robot to pick it up. Port of ``setCmd(EventConfig)``."""
        self._s.set_event(event)
        self._pump_until(
            lambda: self._s.event_status.status == int(EventExecutionStatus.EXECUTING),
            what=f"event {event.event}",
        )
        self._s.clear_event()
        self._s.clear_config()
        self._s.write()

    def open_gripper(self) -> None:
        self.send_event(EventConfig(event=int(Event.OPEN_GRIPPER)))

    def close_gripper(self, force: float = 50.0) -> None:
        self.send_event(
            EventConfig(
                event=int(Event.CLOSE_GRIPPER),
                clamp_force=check.check_grasp_force(force, self.limits),
            )
        )

    # ------------------------------------------------------------------ #
    # internals
    # ------------------------------------------------------------------ #

    def _run_goal(self, goal: ControllerConfig) -> None:
        """Stage a one-shot goal and wait for the robot to accept it.

        Mirrors ``setCmd(ControllerConfig)``: send config + RESET_CONFIG event, wait
        for EXECUTING, then withdraw both so they are not replayed on the next cycle.
        """
        self._s.append_controller(goal)
        self._s.commit_config()
        self._s.set_event(EventConfig(event=int(Event.RESET_CONFIG)))

        self._pump_until(
            lambda: self._s.event_status.status == int(EventExecutionStatus.EXECUTING)
            or self._s.state != RobotState.MOTION,
            what="goal accept",
        )

        state = self._s.state
        self._s.clear_config()
        self._s.clear_event()
        self._s.write()

        if state == RobotState.ERROR:
            raise RobotInError(
                f"robot entered ERROR while accepting the goal: "
                f"{describe_robot_error(self._s.robot_error)}",
                self._s.robot_error,
            )
        if state == RobotState.BASE:
            raise InvalidState(RobotState.MOTION, state)

    def _await_event_done(self, what: str, timeout: Optional[float] = None) -> None:
        """Block until the running event reports SUCCEEDED (or fails)."""
        self._pump_until(
            lambda: self._s.event_status.status
            in (int(EventExecutionStatus.SUCCEEDED), int(EventExecutionStatus.FAILED))
            or self._s.state != RobotState.MOTION,
            what=what,
            timeout=timeout,
        )
        if self._s.state == RobotState.ERROR:
            raise RobotInError(
                f"{what}: robot entered ERROR: "
                f"{describe_robot_error(self._s.robot_error)}",
                self._s.robot_error,
            )
        if self._s.event_status.status == int(EventExecutionStatus.FAILED):
            raise RobotInError(f"{what}: robot reported the motion FAILED")

    def _pump_until(
        self,
        done: Callable[[], bool],
        what: str,
        rearm: Optional[Callable[[], None]] = None,
        timeout: Optional[float] = None,
    ) -> None:
        """Write, wait for a fresh packet, re-check — until ``done()`` or timeout.

        ``rearm`` re-sets the volatile fields (action, control) that
        :meth:`RobotSession.write` clears after each send.
        """
        budget = self.cfg.command_timeout if timeout is None else timeout
        deadline = time.monotonic() + budget

        def expired() -> CommandTimeout:
            return CommandTimeout(
                f"{what}: robot did not respond within {budget:.1f}s. "
                f"Last packet from robot: state={self._s.state.name} "
                f"error={describe_robot_error(self._s.robot_error)} "
                f"event={self._s.event_status.event}/"
                f"status={self._s.event_status.status}. "
                f"Check the robot-side log, and run COBOT_TRACE=1 to dump the "
                f"packets on the wire."
            )

        while True:
            if rearm is not None:
                rearm()
            self._s.write()

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise expired()

            try:
                # Floor the wait: a sliver of remaining budget would otherwise ask
                # for a ~0 s read and surface as a bogus "link is down".
                self._s.read_fresh(
                    timeout=max(0.05, min(remaining, self.cfg.data_transfer_timeout))
                )
            except ConnectionLost:
                # Our wait can expire while the link is still up (read_fresh uses a
                # shorter budget than the pump's own read). That is a stall, not a
                # dead link, so keep pumping until the command budget runs out.
                if not self._s.is_connected:
                    raise
                if time.monotonic() >= deadline:
                    raise expired() from None
                continue

            if done():
                return
            if not self._s.is_connected:
                raise ConnectionLost(f"{what}: link went down")


class ControlStream:
    """Rate-limited tick source for continuous control loops."""

    def __init__(self, robot: Cobot, rate_hz: float):
        if rate_hz <= 0:
            raise ValidationError(f"rate_hz must be > 0, got {rate_hz}")
        self.robot = robot
        self.period = 1.0 / rate_hz

    def ticks(self, seconds: Optional[float] = None, count: Optional[int] = None) -> Iterator[float]:
        """Yield elapsed seconds at the configured rate.

        Bounded by ``seconds`` or ``count``; unbounded if neither is given.
        """
        start = time.monotonic()
        i = 0
        while True:
            if count is not None and i >= count:
                return
            elapsed = time.monotonic() - start
            if seconds is not None and elapsed >= seconds:
                return

            yield elapsed

            i += 1
            slack = (start + i * self.period) - time.monotonic()
            if slack > 0:
                time.sleep(slack)
