"""Struct-level view of the link — the port of data_handler.h / data_handler.cpp.

Owns the outgoing :class:`SocketTx` and the last decoded :class:`SocketRx`, and
reproduces the C++ clearing policy, which is easy to get wrong:

* ``interrupt.action``, ``interrupt.error`` and ``control_int`` are **volatile** —
  cleared after every :meth:`write` (``DataHandler::clearSendData_``).
* ``config`` and ``event_config`` are **sticky** — they keep being resent until
  :meth:`clear_config` / :meth:`clear_event` (``DataHandler::clearData``).

That is why the higher layer sets a controller config once and then only pumps.
"""

from __future__ import annotations

import json
import logging
import time

from .config import LinkConfig
from .enums import RobotAction, RobotState
from .errors import ConnectFailed
from .link import RobotLink
from .messages import (
    AdvancedControllerConfig,
    ControlInterrupt,
    ControllerConfig,
    EventConfig,
    EventStatus,
    PayloadConfig,
    RobotFeedback,
    ScriptConfig,
    SocketRx,
    SocketTx,
    UserConfig,
)

log = logging.getLogger(__name__)


class RobotSession:
    def __init__(self, cfg: LinkConfig):
        self.cfg = cfg
        self._link = RobotLink(cfg)

        self._tx = SocketTx()
        self._rx = SocketRx()
        #: must match RobotLink's initial sequence, so that the first read_fresh()
        #: waits for a real packet instead of returning the empty initial buffer.
        self._rx_seq = 0

        #: staged configuration, assembled across several append_* calls before
        #: commit_config() attaches it to the outgoing packet (UserConfig config_).
        self._config = UserConfig()
        self._script_config: ScriptConfig | None = None

    # -- lifecycle ---------------------------------------------------------- #

    @property
    def is_connected(self) -> bool:
        return self._link.is_connected

    def connect(self, attempts: int | None = None) -> None:
        """Mirror of ``DataHandler::connect`` — seed the line, then handshake.

        Retried like ``CobotHWInterface::connect_()``: a fresh socket and a fresh
        handshake each time, with ``connect_retry_delay`` between tries. Retrying
        matters because a failed attempt also lets the robot's server drop its
        half-open session and return to ``accept()``.
        """
        tries = self.cfg.connect_attempts if attempts is None else attempts
        tries = max(1, tries)

        for attempt in range(1, tries + 1):
            self._tx = SocketTx()
            self._config = UserConfig()
            self._rx = SocketRx()
            self._rx_seq = 0
            try:
                self._link.connect(self._encode())
                if attempt > 1:
                    log.info("connected on attempt %d/%d", attempt, tries)
                return
            except ConnectFailed as exc:
                if attempt >= tries:
                    raise
                log.warning(
                    "connect attempt %d/%d failed (%s); retrying in %.1fs",
                    attempt,
                    tries,
                    exc,
                    self.cfg.connect_retry_delay,
                )
                time.sleep(self.cfg.connect_retry_delay)

    def shutdown(self) -> None:
        self._link.shutdown()

    # -- outgoing ----------------------------------------------------------- #

    def set_action(self, action: RobotAction) -> None:
        self._tx.interrupt.action = int(action)

    def set_error(self, error: int) -> None:
        self._tx.interrupt.error = int(error)

    def set_control(self, control: ControlInterrupt) -> None:
        self._tx.control = control

    def set_event(self, event: EventConfig) -> None:
        self._tx.event_config = event

    def append_payload(self, payload: PayloadConfig) -> None:
        self._config.payload = payload

    def append_controller(self, controller: ControllerConfig) -> None:
        if self._script_config is not None and controller.script_config is None:
            controller.script_config = self._script_config
        self._config.controller_config = controller

    def append_advanced(self, advanced: AdvancedControllerConfig) -> None:
        self._config.advanced_controller_config = advanced

    def append_script(self, script: ScriptConfig) -> None:
        self._script_config = script

    def commit_config(self) -> None:
        """Attach the staged config to the outgoing packet (``setUserConfig``)."""
        self._tx.config = self._config

    def clear_config(self) -> None:
        self._config = UserConfig()
        self._tx.config = None

    def clear_event(self) -> None:
        self._tx.event_config = None

    def clear_control(self) -> None:
        self._tx.control = None

    def write(self) -> None:
        """Publish the current TX, then clear the volatile fields."""
        self._link.set_tx(self._encode())
        self._tx.interrupt.action = int(RobotAction.NONE)
        self._tx.interrupt.error = 0
        self._tx.control = None

    # -- incoming ----------------------------------------------------------- #

    def read(self) -> SocketRx:
        """Decode the newest packet the pump has seen (may repeat)."""
        raw, seq = self._link.get_rx()
        if seq != self._rx_seq:
            self._rx = self._decode(raw)
            self._rx_seq = seq
        return self._rx

    def read_fresh(self, timeout: float | None = None) -> SocketRx:
        """Block for a packet we have not decoded yet."""
        raw, seq = self._link.wait_for_rx(
            after_seq=self._rx_seq,
            timeout=self.cfg.data_transfer_timeout if timeout is None else timeout,
        )
        self._rx = self._decode(raw)
        self._rx_seq = seq
        return self._rx

    @property
    def last_raw(self) -> str:
        """The most recent packet exactly as it came off the wire."""
        return self._link.get_rx()[0]

    @property
    def state(self) -> RobotState:
        try:
            return RobotState(self._rx.interrupt.state)
        except ValueError:
            return RobotState.NONE

    @property
    def robot_error(self) -> int:
        return self._rx.interrupt.error

    @property
    def event_status(self) -> EventStatus:
        return self._rx.event_status or EventStatus()

    @property
    def feedback(self) -> RobotFeedback:
        return self._rx.feedback or RobotFeedback()

    # -- codec -------------------------------------------------------------- #

    def _encode(self) -> str:
        # separators keep the packet small; struct_mapping pretty-printed, but JSON
        # whitespace is insignificant. Nulls *are* emitted (hide_null = false).
        out = json.dumps(self._tx.to_dict(), separators=(",", ":"))
        if self.cfg.trace:
            log.info("TX %s", out)
        return out

    def _decode(self, raw: str) -> SocketRx:
        if self.cfg.trace:
            log.info("RX %s", raw if len(raw) <= 400 else raw[:400] + "...")
        if not raw:
            return SocketRx()
        try:
            return SocketRx.from_dict(json.loads(raw))
        except (ValueError, TypeError) as exc:
            # DataAdapter::stringToStruct swallows parse errors and yields a default
            # struct; a truncated packet must not take the process down.
            log.debug("dropping malformed packet (%s)", exc)
            return SocketRx()
