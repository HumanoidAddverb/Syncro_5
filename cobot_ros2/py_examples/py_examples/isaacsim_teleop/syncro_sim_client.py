import json
import math
import socket
import struct
import threading
import time
import base64
import hashlib
import os
from typing import Dict, Optional


_REAL_KEYS = [
    "shoulder_pan.pos",
    "shoulder_lift.pos",
    "elbow_flex.pos",
    "wrist_flex.pos",
    "wrist_roll.pos",
    "gripper.pos",
]

SIM_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

REAL_TO_SIM: Dict[str, str] = dict(zip(_REAL_KEYS, SIM_JOINT_NAMES))


class SyncroSimClient:
    def __init__(
        self,
        robot: str = "syncro5",
        host: str = "localhost",
        port: int = 8765,
    ):
        if robot not in ("syncro5", "syncro10"):
            raise ValueError(f"robot must be 'syncro5' or 'syncro10', got {robot!r}")
        self._host = host
        self._port = port
        self._sock: Optional[socket.socket] = None
        self._lock = threading.Lock()   # serialises all socket I/O
        self._joint_names = SIM_JOINT_NAMES
        self._real_to_sim = REAL_TO_SIM

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self, timeout: float = 10.0) -> None:
        """Connect to the Isaac Sim WebSocket server, retrying until *timeout*."""
        deadline = time.monotonic() + timeout
        while True:
            try:
                sock = socket.create_connection((self._host, self._port), timeout=2.0)
                break
            except (ConnectionRefusedError, OSError):
                if time.monotonic() > deadline:
                    raise TimeoutError(
                        f"Could not connect to sim server at {self._host}:{self._port} "
                        f"within {timeout}s. Is syncro_sim.py running?"
                    )
                time.sleep(0.25)

        self._ws_handshake(sock)
        self._sock = sock
        self._sock.settimeout(None)   # switch to blocking mode

    def _ws_handshake(self, sock: socket.socket) -> None:
        """Perform the HTTP → WebSocket upgrade handshake."""
        key = base64.b64encode(os.urandom(16)).decode()
        request = (
            f"GET / HTTP/1.1\r\n"
            f"Host: {self._host}:{self._port}\r\n"
            f"Upgrade: websocket\r\n"
            f"Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            f"Sec-WebSocket-Version: 13\r\n"
            f"\r\n"
        )
        sock.sendall(request.encode())

        # Read until the blank line that ends the HTTP headers.
        buf = b""
        while b"\r\n\r\n" not in buf:
            chunk = sock.recv(4096)
            if not chunk:
                raise ConnectionError("Server closed connection during WebSocket handshake")
            buf += chunk

        if b"101" not in buf:
            raise ConnectionError(f"WebSocket upgrade failed:\n{buf[:300].decode(errors='replace')}")

        # Verify the server's accept key (optional but correct).
        accept_key = base64.b64encode(
            hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()
        ).decode()
        if accept_key.encode() not in buf:
            raise ConnectionError("WebSocket accept key mismatch — unexpected server.")

    def disconnect(self) -> None:
        """Close the WebSocket connection."""
        with self._lock:
            if self._sock:
                try:
                    # Send a close frame (opcode 0x8, no payload).
                    self._sock.sendall(b"\x88\x80" + bytes(4))
                except OSError:
                    pass
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None

    # ── Low-level WebSocket framing ───────────────────────────────────────────

    @staticmethod
    def _encode_frame(payload: str) -> bytes:
        """Encode a UTF-8 text payload as a masked WebSocket frame (client → server)."""
        data   = payload.encode("utf-8")
        length = len(data)
        mask   = os.urandom(4)

        header = bytearray()
        header.append(0x81)                     # FIN=1, opcode=text
        if length < 126:
            header.append(0x80 | length)        # MASK=1
        elif length < 65536:
            header.append(0x80 | 126)
            header += struct.pack(">H", length)
        else:
            header.append(0x80 | 127)
            header += struct.pack(">Q", length)
        header += mask

        masked = bytearray(b ^ mask[i % 4] for i, b in enumerate(data))
        return bytes(header) + bytes(masked)

    def _decode_frame(self) -> Optional[str]:
        """Read a full WebSocket message (handles continuation frames)."""

        def recv_exact(n: int) -> bytes:
            buf = b""
            while len(buf) < n:
                chunk = self._sock.recv(n - len(buf))
                if not chunk:
                    raise ConnectionError("Sim server closed the connection")
                buf += chunk
            return buf

        message = bytearray()

        while True:
            header = recv_exact(2)
            fin    = (header[0] & 0x80) != 0
            opcode = header[0] & 0x0F
            is_mask = (header[1] & 0x80) != 0
            length  = header[1] & 0x7F

            # Extended length handling
            if length == 126:
                length = struct.unpack(">H", recv_exact(2))[0]
            elif length == 127:
                length = struct.unpack(">Q", recv_exact(8))[0]

            mask_key = recv_exact(4) if is_mask else b""
            payload  = bytearray(recv_exact(length))

            if is_mask:
                for i in range(length):
                    payload[i] ^= mask_key[i % 4]

            # ---- Control frames ----
            if opcode == 0x8:  # close
                self._sock.sendall(b"\x88\x00")  # reply close
                return None

            if opcode == 0x9:  # ping
                # MUST mirror payload
                pong = b"\x8A" + bytes([len(payload)]) + payload
                self._sock.sendall(pong)
                continue

            if opcode == 0xA:  # pong
                continue

            # ---- Data frames ----
            if opcode == 0x1:  # text frame (start)
                message.extend(payload)

            elif opcode == 0x0:  # continuation
                message.extend(payload)

            elif opcode == 0x2:  # binary (optional: ignore or handle)
                # If you don't expect binary, skip it
                continue

            else:
                # Unknown opcode — ignore safely
                continue

            # If final frame, decode full message
            if fin:
                try:
                    return message.decode("utf-8")
                except UnicodeDecodeError as e:
                    raise RuntimeError(f"Invalid UTF-8 message: {message}") from e
        
        # ── Public API ────────────────────────────────────────────────────────────

    def set_joint_positions(self, positions: Dict[str, float]) -> None:
        """
        Stream joint position targets (radians) to the simulated robot.

        Only the joints listed in *positions* are updated; unspecified joints
        keep their current targets.

        Valid joint names: Rotation | Pitch | Elbow | Wrist_Pitch | Wrist_Roll | Jaw
        """
        frame = self._encode_frame(json.dumps({"cmd": "set_joints", "positions": positions}))
        with self._lock:
            self._sock.sendall(frame)

    def get_state(self) -> dict:
        """
        Request the current simulated joint state.

        Returns:
            {
              "joint_positions":  {"Rotation": <rad>, ...},
              "joint_velocities": {"Rotation": <rad/s>, ...},
            }
        """
        frame = self._encode_frame(json.dumps({"cmd": "get_state"}))
        with self._lock:
            self._sock.sendall(frame)
            raw = self._decode_frame()
        if raw is None:
            raise ConnectionError("Sim server closed the connection")
        return json.loads(raw)

    def mirror_real_robot(self, obs: Dict[str, float], degrees: bool = True) -> None:
        """
        Translate a real-robot observation dict into a sim joint command.

        Parameters
        ----------
        obs:
            LeRobot-style observation, e.g.::

                {
                    "shoulder_pan.pos":  -6.36,
                    "shoulder_lift.pos": -51.95,
                    "elbow_flex.pos":     16.73,
                    "wrist_flex.pos":     89.25,
                    "wrist_roll.pos":    -51.85,
                    "gripper.pos":         0.0,
                    # camera / other keys are silently ignored
                }

        degrees:
            True (default) — values are in degrees and will be converted to radians.
            False — values are already in radians.
        """
        positions: Dict[str, float] = {}
        for real_key, sim_key in self._real_to_sim.items():
            if real_key in obs:
                val = obs[real_key]
                positions[sim_key] = math.radians(val) if degrees else float(val)
        if positions:
            self.set_joint_positions(positions)

    def set_joint_positions_degrees(self, positions_deg: Dict[str, float]) -> None:
        """Convenience wrapper: accepts joint positions in degrees, converts to radians."""
        self.set_joint_positions({k: math.radians(v) for k, v in positions_deg.items()})

    def set_all_joints(self, values: list, degrees: bool = False) -> None:
        """
        Set all six joints by ordered list.

        values: [joint1/J1, joint2/J2, joint3/J3, joint4/J4, joint5/J5, joint6/J6]
                 (base-rotation, shoulder, elbow, wrist-pitch, wrist-roll, gripper)
        """
        if len(values) != len(self._joint_names):
            raise ValueError(f"Expected {len(self._joint_names)} values, got {len(values)}")
        pos = {name: (math.radians(v) if degrees else float(v))
               for name, v in zip(self._joint_names, values)}
        self.set_joint_positions(pos)

    # ── Context manager ───────────────────────────────────────────────────────

    def __enter__(self) -> "SyncroSimClient":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.disconnect()


# ── CLI smoke-test ────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import sys

    robot = sys.argv[1] if len(sys.argv) > 1 else "syncro5"
    host  = sys.argv[2] if len(sys.argv) > 2 else "localhost"
    port  = int(sys.argv[3]) if len(sys.argv) > 3 else 8765

    print(f"Connecting to ws://{host}:{port} ({robot}) …")
    with SyncroSimClient(robot=robot, host=host, port=port) as client:
        print("Connected.")

        # Sweep base-rotation joint from -30° to +30° over 3 seconds.
        base_joint = client._joint_names[0]
        steps = 60
        for i in range(steps + 1):
            angle_deg = -30 + 60 * (i / steps)
            client.set_joint_positions_degrees({base_joint: angle_deg})
            time.sleep(3.0 / steps)

        state = client.get_state()
        print("Final sim state:")
        for name, rad in state["joint_positions"].items():
            print(f"  {name:15s} {math.degrees(rad):+.2f}°  ({rad:+.4f} rad)")
