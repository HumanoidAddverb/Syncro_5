"""Framed TCP client to the robot.

Collapses four C++ classes into one:
  socket_client.h              -> socket creation / connect / close
  socket_communication_json.h  -> poll-based read with timeout, send
  socket_comm_wrapper.h        -> '#' start/end delimiter framing + leftover buffering
  socket_client_comm_wrapper.h -> the data-line handshake

Wire format: ``#<json>#``. Anything before the first ``#`` is discarded, exactly as
``SocketCommWrapper::read`` does.
"""

from __future__ import annotations

import fcntl
import logging
import select
import socket
import struct
import termios
import time

from .config import LinkConfig
from .errors import ConnectFailed, ConnectionLost

log = logging.getLogger(__name__)


class FramedSocket:
    def __init__(self, cfg: LinkConfig):
        self.cfg = cfg
        self._sock: socket.socket | None = None
        self._buf = bytearray()
        self._read_timeout = cfg.data_transfer_timeout
        #: set when the last packet we sent exceeded quick_transfer_limit, so the
        #: reply may be a slow bulk transfer (file_transfer_alert_ in C++).
        self._file_transfer = False

    # -- lifecycle ---------------------------------------------------------- #

    @property
    def is_open(self) -> bool:
        return self._sock is not None

    def connect(self, init_packet: str) -> None:
        """Open the socket and establish the data line.

        Both ends run ``establishDataLine_()`` and each waits for the other, so
        this is a rendezvous rather than a one-shot. We resend ``init_packet``
        every ``handshake_poll_interval`` for up to ``handshake_timeout``, which
        keeps a packet queued for whenever the robot reaches its ``accept()`` and
        polls -- and keeps our socket open long enough for it to get there.
        """
        self.close()

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.settimeout(self.cfg.connect_timeout)
        try:
            sock.connect((self.cfg.ip, self.cfg.port))
        except OSError as exc:
            sock.close()
            raise ConnectFailed(
                f"could not reach robot at {self.cfg.ip}:{self.cfg.port}: {exc}"
            ) from exc

        sock.settimeout(None)  # blocking; we gate every read with select()
        self._sock = sock
        self._buf.clear()

        started = time.monotonic()
        deadline = started + self.cfg.handshake_timeout
        attempt = 0

        while True:
            attempt += 1
            # Send first: the robot is waiting on us, so polling before we have
            # spoken just burns the interval.
            self.send(init_packet)

            if self._readable(self.cfg.handshake_poll_interval):
                if self._bytes_pending() > 0:
                    log.info(
                        "data line established with %s:%s after %d init packet(s), %.1fs",
                        self.cfg.ip,
                        self.cfg.port,
                        attempt,
                        time.monotonic() - started,
                    )
                    return

                # Readable with nothing queued means the peer hung up. Fail fast so
                # the caller reconnects rather than "succeeding" and dying on the
                # first read.
                self.close()
                raise ConnectFailed(
                    f"{self.cfg.ip}:{self.cfg.port} accepted the connection and then "
                    f"closed it without sending data (readable, 0 bytes pending). "
                    f"The robot dropped the session; a reconnect should get a fresh "
                    f"accept()."
                )

            if time.monotonic() >= deadline:
                break

            # Keep the operator informed without one line per half second.
            if attempt % 10 == 0:
                log.warning(
                    "%s:%s has not answered %d init packets (%.0fs of %.0fs); "
                    "waiting for the robot to reach accept()",
                    self.cfg.ip,
                    self.cfg.port,
                    attempt,
                    time.monotonic() - started,
                    self.cfg.handshake_timeout,
                )

        elapsed = time.monotonic() - started
        self.close()
        raise ConnectFailed(
            f"robot at {self.cfg.ip}:{self.cfg.port} accepted the TCP connection but "
            f"sent nothing back to {attempt} init packets over {elapsed:.0f}s. "
            f"If the robot logs 'accepted' then 'no data to read' / 'attempted to "
            f"establish data line', it accepted a socket we had already closed: "
            f"restart the robot-side controller so it returns to accept(), then "
            f"retry. Also check nothing else holds the connection "
            f"(CONNECTION_QUEUE is 1)."
        )

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        self._buf.clear()

    # -- io ----------------------------------------------------------------- #

    def send(self, payload: str) -> None:
        """Frame and send one packet."""
        if self._sock is None:
            raise ConnectionLost("socket is not open")

        data = (self.cfg.start_delim + payload + self.cfg.end_delim).encode()

        # A large outgoing packet (e.g. a replay dataset) means the robot needs
        # longer to chew on it before replying.
        self._file_transfer = len(data) > self.cfg.quick_transfer_limit

        try:
            self._sock.sendall(data)
        except OSError as exc:
            raise ConnectionLost(f"send failed: {exc}") from exc

    def recv(self) -> str:
        """Block until one complete ``#...#`` packet arrives and return its body."""
        budget = (
            self.cfg.file_transfer_timeout
            if self._file_transfer
            else self.cfg.data_transfer_timeout
        )
        deadline = time.monotonic() + budget

        while True:
            packet = self._take_packet()
            if packet is not None:
                self._file_transfer = False
                return packet

            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise ConnectionLost(f"no packet from robot within {budget:.1f}s")
            self._fill(remaining)

    # -- internals ---------------------------------------------------------- #

    def _fill(self, timeout: float) -> None:
        if self._sock is None:
            raise ConnectionLost("socket is not open")

        # Non-strict here on purpose: an EOF/RST must reach recv() so it can be
        # reported as a lost link rather than silently spun on.
        if not self._readable(timeout):
            return

        try:
            chunk = self._sock.recv(self.cfg.read_buffer_size)
        except OSError as exc:
            raise ConnectionLost(f"read failed: {exc}") from exc

        if not chunk:
            raise ConnectionLost("robot closed the connection")

        self._buf.extend(chunk)

    def _readable(self, timeout: float) -> bool:
        """True if select() says something is there -- including EOF or RST."""
        if self._sock is None:
            return False
        readable, _, _ = select.select([self._sock], [], [], timeout)
        return bool(readable)

    def _bytes_pending(self) -> int:
        """Bytes actually queued, via ioctl(FIONREAD).

        ``select()`` alone is not enough: a closed or reset peer reads as
        *readable with zero bytes*, which would look like a successful handshake
        and then blow up on the first recv. ``SocketCommunicationJSON::
        isDataAvailable_`` guards against exactly this by requiring
        ``expected_data_size > 0`` from ``ioctl(FIONREAD)``.
        """
        if self._sock is None:
            return 0
        try:
            packed = fcntl.ioctl(self._sock, termios.FIONREAD, b"\0\0\0\0")
        except OSError:
            return 0
        return struct.unpack("I", packed)[0]

    def _take_packet(self) -> str | None:
        """Pop the first complete frame out of the buffer, or return None."""
        start_b = self.cfg.start_delim.encode()
        end_b = self.cfg.end_delim.encode()

        begin = self._buf.find(start_b)
        if begin < 0:
            # No frame start yet. Nothing before it is usable, so drop it.
            self._buf.clear()
            return None

        end = self._buf.find(end_b, begin + len(start_b))
        if end < 0:
            del self._buf[:begin]  # keep the partial frame, drop the junk before it
            return None

        body = bytes(self._buf[begin + len(start_b) : end])
        del self._buf[: end + len(end_b)]
        return body.decode(errors="replace")
