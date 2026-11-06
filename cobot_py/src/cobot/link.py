"""Background read/write pump — the port of network_manager.h.

The robot expects to be fed continuously: it drives the cadence by streaming state,
and every packet it sends must be answered with the latest command. That loop runs
on its own thread here exactly as it did in ``NetworkManager::clientComm_``:

    read()  ->  send(latest_tx)

Application code never touches the socket. It publishes the next TX with
:meth:`set_tx` and picks up the newest RX with :meth:`get_rx` / :meth:`wait_for_rx`.
Because the read blocks until the robot speaks, the loop is paced by the robot and
does not spin.
"""

from __future__ import annotations

import logging
import threading

from .config import LinkConfig
from .errors import ConnectionLost
from .transport import FramedSocket

log = logging.getLogger(__name__)


class RobotLink:
    def __init__(self, cfg: LinkConfig):
        self.cfg = cfg
        self._sock = FramedSocket(cfg)

        self._lock = threading.Lock()
        self._new_rx = threading.Condition(self._lock)

        self._tx: str = ""
        self._rx: str = ""
        self._rx_seq: int = 0  # bumped on every packet, so callers can wait for a *fresh* one
        self._connected = False
        self._failure: BaseException | None = None

        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    # -- lifecycle ---------------------------------------------------------- #

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._connected

    def connect(self, init_packet: str) -> None:
        """Handshake, then start the pump thread."""
        with self._lock:
            self._tx = init_packet
            self._failure = None

        self._sock.connect(init_packet)

        with self._lock:
            self._connected = True

        self._stop.clear()
        self._thread = threading.Thread(target=self._pump, name="cobot-link", daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._stop.set()
        if self._thread is not None and self._thread is not threading.current_thread():
            self._thread.join(timeout=self.cfg.data_transfer_timeout + 1.0)
        self._thread = None
        self._sock.close()
        with self._lock:
            self._connected = False
            self._new_rx.notify_all()

    # -- data exchange ------------------------------------------------------ #

    def set_tx(self, packet: str) -> None:
        """Publish the packet the pump should send from now on."""
        with self._lock:
            self._tx = packet

    def get_rx(self) -> tuple[str, int]:
        """Return the most recent packet and its sequence number (may be stale)."""
        with self._lock:
            self._raise_failure()
            return self._rx, self._rx_seq

    def wait_for_rx(self, after_seq: int, timeout: float) -> tuple[str, int]:
        """Block until a packet newer than ``after_seq`` arrives.

        This is what the C++ busy-wait loops were approximating; waiting on a
        sequence number instead means we never re-read the same packet twice.
        """
        with self._new_rx:
            ok = self._new_rx.wait_for(
                lambda: self._rx_seq > after_seq or self._failure is not None,
                timeout=timeout,
            )
            self._raise_failure()
            if not ok:
                raise ConnectionLost(f"no update from robot within {timeout:.1f}s")
            return self._rx, self._rx_seq

    def _raise_failure(self) -> None:
        if self._failure is not None:
            raise self._failure

    # -- the pump ----------------------------------------------------------- #

    def _pump(self) -> None:
        try:
            while not self._stop.is_set():
                packet = self._sock.recv()

                with self._new_rx:
                    self._rx = packet
                    self._rx_seq += 1
                    tx = self._tx
                    self._new_rx.notify_all()

                self._sock.send(tx)
        except BaseException as exc:  # noqa: BLE001 - surfaced to the caller thread
            if not self._stop.is_set():
                log.error("link down: %s", exc)
            with self._new_rx:
                self._connected = False
                self._failure = exc if isinstance(exc, ConnectionLost) else ConnectionLost(str(exc))
                self._new_rx.notify_all()
        finally:
            with self._lock:
                self._connected = False
