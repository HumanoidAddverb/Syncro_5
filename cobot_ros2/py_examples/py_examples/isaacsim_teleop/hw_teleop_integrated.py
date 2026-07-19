"""
hw_teleop_integrated.py
=======================
Integrated keyboard teleop + multi-episode hardware teleoperation data logger.

Merges keyboard_teleop_only_hw.py and hw_with_cam.py into one process — no
second terminal needed.

Episode lifecycle
-----------------
  1. Press any key in terminal → start recording
  2. Use keyboard in terminal to teleoperate:
       q w e r t y → joint 1-6  +velocity
       a s d f g h → joint 1-6  -velocity
       u → open gripper   j → close gripper
  3. Press SPACE in camera window → end episode
  4. Press  y = save  /  n = discard  in terminal
  5. Robot homes automatically
  6. Put the object back, press any key → next episode
  7. q in camera window or Ctrl-C → quit session

Key architectural difference from the two-process design
---------------------------------------------------------
A TeleopState machine (IDLE / RECORDING / SAVE_PROMPT) governs which handler
consumes each stdin keystroke.  In RECORDING the keys drive joint velocities and
the gripper; in IDLE/SAVE_PROMPT they drive the episode lifecycle.  A single
ROS2 node publishes velocity commands *and* subscribes to them for logging via
a natural loopback on /velocity_controller/commands.

Data sources
------------
  /joint_states                   → observation.state.joint_positions
  /velocity_controller/commands   → action.robot (arm velocities)
  /gripper_action_command         → action.robot[6] (gripper)
  RealSense D435 × 2              → observation.images.ego / .external
"""

import enum
import math
import os
import queue
import subprocess
import sys
import termios
import threading
import time
import tty
import signal
import atexit

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float32
from addverb_cobot_msgs.srv import Gripper

# ── Canonical sim logger (shared with syncro_isaac_sim) ───────────────────────
# The logging engine + dataset config MUST match the syncro_isaac_sim pipeline so
# hardware datasets carry the exact same LeRobot/GROOT schema as the sim datasets
# and can be trained together. Prefer a sibling checkout of that repo if present;
# otherwise fall back to the copy vendored alongside this script
# (sim_data_logger.py / metadata_utils_parquet.py / data_logger_config.yaml).
_HERE = os.path.dirname(os.path.abspath(__file__))
_SIBLING_SIM_ROOT = os.path.abspath(
    os.path.join(_HERE, "..", "..", "..", "..", "syncro_isaac_sim")
)
_SIM_LOGGER_ROOT = _SIBLING_SIM_ROOT if os.path.isdir(_SIBLING_SIM_ROOT) else _HERE
if _SIM_LOGGER_ROOT not in sys.path:
    sys.path.insert(0, _SIM_LOGGER_ROOT)

from sim_data_logger import SimDataLogger  # noqa: E402

# ── Constants ─────────────────────────────────────────────────────────────────

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
DEFAULT_CONFIG_PATH = os.path.join(_SIM_LOGGER_ROOT, "data_logger_config.yaml")
_HOMING_PY = os.path.abspath(os.path.join(_HERE, "..", "homing.py"))
TOPIC_TIMEOUT_S: float = 2.0
VELOCITY_MAGNITUDE = 0.05
MAX_ACCEL = 0.4
CONTROL_DT = 0.02  # 50 Hz ramp loop

# ── Terminal management ───────────────────────────────────────────────────────

STDIN_FD = sys.stdin.fileno()
_ORIG_TERM = termios.tcgetattr(STDIN_FD)


def _restore_terminal():
    try:
        termios.tcsetattr(STDIN_FD, termios.TCSADRAIN, _ORIG_TERM)
    except Exception:
        pass


atexit.register(_restore_terminal)


def _getch_raw() -> str:
    """Read one character from stdin in cbreak mode. Returns '' on error."""
    try:
        tty.setcbreak(STDIN_FD)
        return sys.stdin.read(1)
    except Exception:
        return ""


# ── Teleop state machine ──────────────────────────────────────────────────────

class TeleopState(enum.Enum):
    IDLE = "idle"            # waiting to start an episode
    RECORDING = "recording"  # robot teleop active
    SAVE_PROMPT = "save_prompt"  # episode ended; waiting for y / n


_teleop_state = TeleopState.IDLE
_state_lock = threading.Lock()

_last_teleop_key: str | None = None
_key_lock = threading.Lock()

_shared_joint_velocities = [0.0] * len(JOINT_NAMES)
_velocity_lock = threading.Lock()

# Keys for the episode lifecycle (IDLE + SAVE_PROMPT states)
_episode_key_queue: queue.Queue = queue.Queue()

_shutdown_event = threading.Event()

# ── Custom exception ──────────────────────────────────────────────────────────


class TopicTimeoutError(RuntimeError):
    def __init__(self, topic: str, elapsed: float):
        self.topic = topic
        self.elapsed = elapsed
        super().__init__(
            f"No data on '{topic}' for {elapsed:.1f} s "
            f"(timeout={TOPIC_TIMEOUT_S} s) — stopping logger."
        )


# ── Integrated ROS2 node ──────────────────────────────────────────────────────


class _IntegratedNode(Node):
    """
    Single ROS2 node: publishes velocity/gripper commands for teleop AND
    subscribes to the same topics for data logging (natural loopback).
    """

    def __init__(self):
        super().__init__("hw_teleop_integrated")

        self._obs_pos: dict[str, float] = {}
        self._obs_lock = threading.Lock()

        self._arm_action: list[float] = [0.0] * len(JOINT_NAMES)
        self._gripper_action: float = 0.0
        self._action_lock = threading.Lock()

        self._last_joint_state_t: float | None = None
        self._last_action_t: float | None = None
        self._watchdog_lock = threading.Lock()

        # Subscribers (for logging)
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        self.create_subscription(
            Float64MultiArray, "/velocity_controller/commands",
            self._velocity_commands_cb, 10,
        )
        self.create_subscription(Float32, "/gripper_action_command", self._gripper_action_cb, 10)

        # Publishers (for teleop)
        self._vel_pub = self.create_publisher(Float64MultiArray, "/velocity_controller/commands", 10)
        self._gripper_pub = self.create_publisher(Float32, "/gripper_action_command", 10)

        # Gripper service client
        self._gripper_client = self.create_client(Gripper, "/gripper_controller/command")

        # 50 Hz velocity publish timer
        self.create_timer(1.0 / 50.0, self._velocity_timer_cb)

        self.get_logger().info("_IntegratedNode initialized.")

    # ── Subscriber callbacks ──────────────────────────────────────────────────

    def _joint_state_cb(self, msg: JointState):
        local: dict[str, float] = {}
        for i, name in enumerate(msg.name):
            if name not in JOINT_NAMES:
                continue
            if i < len(msg.position):
                val = float(msg.position[i])
                local[name] = 0.0 if math.isnan(val) else val
        with self._obs_lock:
            self._obs_pos = local
        with self._watchdog_lock:
            self._last_joint_state_t = time.monotonic()

    def _velocity_commands_cb(self, msg: Float64MultiArray):
        data = [float(v) if not math.isnan(float(v)) else 0.0 for v in msg.data]
        data = (data + [0.0] * len(JOINT_NAMES))[: len(JOINT_NAMES)]
        with self._action_lock:
            self._arm_action = data
        with self._watchdog_lock:
            self._last_action_t = time.monotonic()

    def _gripper_action_cb(self, msg: Float32):
        with self._action_lock:
            self._gripper_action = float(msg.data)

    # ── Velocity publish timer (teleop) ───────────────────────────────────────

    def _velocity_timer_cb(self):
        with _velocity_lock:
            velocities = _shared_joint_velocities[:]
        msg = Float64MultiArray()
        msg.data = velocities
        self._vel_pub.publish(msg)

    # ── Data access for logging ───────────────────────────────────────────────

    def get_obs_joints(self) -> np.ndarray:
        """(7,) float32: [j1..j6, gripper]"""
        with self._obs_lock:
            pos = self._obs_pos.copy()
        with self._action_lock:
            grip = self._gripper_action
        return np.array([pos.get(n, 0.0) for n in JOINT_NAMES] + [grip], dtype=np.float32)

    def get_action_joints(self) -> np.ndarray:
        """(7,) float32: [vel_j1..vel_j6, gripper]"""
        with self._action_lock:
            arm = self._arm_action.copy()
            grip = self._gripper_action
        return np.array(arm + [grip], dtype=np.float32)

    def check_topic_health(self):
        now = time.monotonic()
        with self._watchdog_lock:
            js_t = self._last_joint_state_t
            act_t = self._last_action_t
        if js_t is not None and (now - js_t) > TOPIC_TIMEOUT_S:
            raise TopicTimeoutError("/joint_states", now - js_t)
        if act_t is not None and (now - act_t) > TOPIC_TIMEOUT_S:
            raise TopicTimeoutError("/velocity_controller/commands", now - act_t)

    # ── Gripper ───────────────────────────────────────────────────────────────

    def call_gripper(self, position: float, force: float = 100.0):
        req = Gripper.Request()
        req.position = float(position)
        req.grasp_force = float(force)
        future = self._gripper_client.call_async(req)

        def _done(fut):
            try:
                result = fut.result()
                if result is None:
                    print("[Gripper] Service returned no result")
                    return
                if result.success:
                    print(f"[Gripper] SUCCESS | position={position}, force={force}")
                    msg = Float32()
                    # closed=1.0 → log as 1.0; open=0.0 → log as 0.0 (reversed convention)
                    msg.data = 0.0 if position == 1.0 else 1.0
                    self._gripper_pub.publish(msg)
                else:
                    print(f"[Gripper] FAILED | message={result.message}")
            except Exception as exc:
                print(f"[Gripper] Exception: {exc}")

        future.add_done_callback(_done)


# ── Keyboard thread ───────────────────────────────────────────────────────────


def _keyboard_thread_fn(node: _IntegratedNode):
    """
    Single stdin reader. Routes each key by TeleopState:
      RECORDING     → joint velocity target or gripper command
      IDLE / SAVE_PROMPT → episode lifecycle queue
    """
    global _last_teleop_key

    while not _shutdown_event.is_set():
        ch = _getch_raw()
        if not ch:
            continue

        if ch == "\x03":  # Ctrl-C
            _shutdown_event.set()
            _episode_key_queue.put("\x03")
            break

        with _state_lock:
            state = _teleop_state

        if state == TeleopState.RECORDING:
            if ch == "u":
                node.call_gripper(0.0)   # open
            elif ch == "j":
                node.call_gripper(1.0)   # close
            else:
                with _key_lock:
                    _last_teleop_key = ch
        else:
            _episode_key_queue.put(ch)


# ── Velocity ramp thread ──────────────────────────────────────────────────────


def _teleop_ramp_thread_fn():
    """
    50 Hz loop: ramps _shared_joint_velocities toward the keyboard target.
    Only applies joint key input in RECORDING state; otherwise ramps to zero.
    """
    global _last_teleop_key, _shared_joint_velocities

    target_velocities = [0.0] * len(JOINT_NAMES)
    current_velocities = [0.0] * len(JOINT_NAMES)
    max_step = MAX_ACCEL * CONTROL_DT

    while not _shutdown_event.is_set():
        with _state_lock:
            state = _teleop_state

        target_velocities = [0.0] * len(JOINT_NAMES)

        if state == TeleopState.RECORDING:
            with _key_lock:
                ch = _last_teleop_key
                _last_teleop_key = None

            if ch:
                if ch in "qwerty":
                    idx = "qwerty".index(ch)
                    if idx < len(JOINT_NAMES):
                        target_velocities[idx] = VELOCITY_MAGNITUDE
                elif ch in "asdfgh":
                    idx = "asdfgh".index(ch)
                    if idx < len(JOINT_NAMES):
                        target_velocities[idx] = -VELOCITY_MAGNITUDE

        for i in range(len(JOINT_NAMES)):
            error = target_velocities[i] - current_velocities[i]
            if abs(error) <= max_step:
                current_velocities[i] = target_velocities[i]
            else:
                current_velocities[i] += max_step * math.copysign(1.0, error)

        with _velocity_lock:
            _shared_joint_velocities = current_velocities[:]

        time.sleep(CONTROL_DT)


# ── Episode key helper ────────────────────────────────────────────────────────


def _wait_episode_key() -> str:
    """Block until a lifecycle key arrives, polling every 100 ms for shutdown."""
    while not _shutdown_event.is_set():
        try:
            return _episode_key_queue.get(timeout=0.1)
        except queue.Empty:
            continue
    return "\x03"


# ── Hardware data logger session ──────────────────────────────────────────────


class HardwareDataLogger:
    """
    Manages one multi-episode data-collection session.

    The ROS2 node, spin thread, keyboard thread, and ramp thread are created
    outside this class (in main) and passed in.  This class owns only the
    cameras, SimDataLogger instances, and the per-episode capture loop.
    """

    def __init__(
        self,
        node: _IntegratedNode,
        dataset_root: str,
        config_path: str = DEFAULT_CONFIG_PATH,
        fps: float = 30.0,
    ):
        self._node = node
        self._dataset_root = dataset_root
        self._config_path = config_path
        self._fps = fps

        self._session_stop = threading.Event()
        self._episode_stop = threading.Event()

        self._pipelines: dict[str, rs.pipeline] = {}
        self._capture_thread: threading.Thread | None = None
        self._sim_logger: SimDataLogger | None = None

    # ── Session lifecycle ─────────────────────────────────────────────────────

    def open(self):
        """Open RealSense cameras. Call once per session after ROS2 is up."""
        ctx = rs.context()
        cam_names = ["external", "ego"]
        for i, dev in enumerate(ctx.query_devices()):
            if i >= len(cam_names):
                break
            dev_sn = dev.get_info(rs.camera_info.serial_number)
            name = cam_names[i]
            pipeline = rs.pipeline(ctx)
            cfg = rs.config()
            cfg.enable_device(dev_sn)
            cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            pipeline.start(cfg)
            self._pipelines[name] = pipeline

        if len(self._pipelines) < 2:
            print(f"[HW] WARNING: expected 2 cameras, found {len(self._pipelines)}")

        print(f"[HW] Session opened → {self._dataset_root}")

    def close(self):
        """Stop cameras, finalize any running episode."""
        self._session_stop.set()
        self._episode_stop.set()

        if self._capture_thread and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=5.0)

        if self._sim_logger:
            try:
                self._sim_logger.finalize()
            except Exception:
                pass
            self._sim_logger = None

        for pipeline in self._pipelines.values():
            try:
                pipeline.stop()
            except Exception:
                pass
        self._pipelines.clear()

        cv2.destroyAllWindows()
        print("[HW] Session closed.")

    # ── Episode lifecycle ─────────────────────────────────────────────────────

    def start_episode(self, episode_num: int) -> SimDataLogger:
        self._episode_stop = threading.Event()

        self._sim_logger = SimDataLogger(
            dataset_root=self._dataset_root,
            episode_num=episode_num,
            cameras={name: {"width": 640, "height": 480} for name in self._pipelines},
            fps=self._fps,
            dataset_config_path=self._config_path,
        )
        self._sim_logger.init_episode()

        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            daemon=True,
            name=f"HW-Capture-ep{episode_num}",
        )
        self._capture_thread.start()

        print(f"[HW] Episode {episode_num} started — SPACE in camera window to end.")
        return self._sim_logger

    def end_episode(self) -> SimDataLogger:
        self._episode_stop.set()
        if self._capture_thread and self._capture_thread.is_alive():
            self._capture_thread.join(timeout=5.0)
        self._capture_thread = None

        logger = self._sim_logger
        self._sim_logger = None
        return logger

    # ── Homing ───────────────────────────────────────────────────────────────

    def home(self):
        """
        Home the robot via homing.py subprocess so its rclpy lifecycle is
        isolated from this process's ROS2 context.
        """
        if not os.path.isfile(_HOMING_PY):
            print(f"[HW] homing.py not found at {_HOMING_PY} — skipping.")
            return
        print("[HW] Homing robot to zero position...")
        try:
            result = subprocess.run([sys.executable, _HOMING_PY], timeout=15.0)
            if result.returncode != 0:
                print(f"[HW] Homing script exited with code {result.returncode}")
        except subprocess.TimeoutExpired:
            print("[HW] Homing timed out after 15 s — continuing anyway.")
        except Exception as exc:
            print(f"[HW] Homing error: {exc!r}")
        print("[HW] Homing done.")

    # ── Capture loop ─────────────────────────────────────────────────────────

    def _capture_loop(self):
        """30 Hz: read cameras + joints → push to SimDataLogger."""
        TARGET_DT = 1.0 / 30.0
        last_frame_numbers = {name: -1 for name in self._pipelines}
        next_tick = time.perf_counter()
        valid = True

        try:
            while not self._episode_stop.is_set() and not self._session_stop.is_set():
                loop_start = time.perf_counter()

                try:
                    self._node.check_topic_health()

                    visual_obs: dict = {}
                    current_frame_numbers: dict = {}

                    if valid:
                        for name, pipeline in self._pipelines.items():
                            frames = pipeline.wait_for_frames(timeout_ms=200)
                            color_frame = frames.get_color_frame()
                            if color_frame is None:
                                valid = False
                                break
                            fn = color_frame.get_frame_number()
                            if fn == last_frame_numbers[name]:
                                valid = False
                                break
                            current_frame_numbers[name] = fn
                            image_bgr = np.asanyarray(color_frame.get_data())
                            if (
                                image_bgr is None
                                or image_bgr.size == 0
                                or image_bgr.ndim != 3
                                or image_bgr.shape[2] != 3
                            ):
                                valid = False
                                break
                            image_rgb = image_bgr[:, :, ::-1].copy()
                            if not np.isfinite(image_rgb).all():
                                valid = False
                                break
                            visual_obs[f"rgb_{name}"] = image_rgb
                            cv2.imshow(f"Camera: {name}", image_bgr)

                    key = cv2.waitKey(1) & 0xFF
                    if key == ord(" "):
                        print("[HW] SPACE → ending episode.")
                        self._episode_stop.set()
                        break
                    elif key == ord("q"):
                        print("[HW] q → stopping session.")
                        self._session_stop.set()
                        self._episode_stop.set()
                        break

                    if valid and len(visual_obs) == len(self._pipelines):
                        for name, fn in current_frame_numbers.items():
                            last_frame_numbers[name] = fn
                        obs_joints = self._node.get_obs_joints()
                        action_joints = self._node.get_action_joints()
                        self._sim_logger.push(action_joints, obs_joints, visual_obs, loop_start)

                except TopicTimeoutError:
                    raise
                except RuntimeError:
                    pass
                except Exception as exc:
                    print(f"[HW] capture error: {exc!r}")

                next_tick += TARGET_DT
                now = time.perf_counter()
                sleep_time = next_tick - now
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    next_tick = now

        except TopicTimeoutError as exc:
            print(f"[HW] {exc}")
            self._episode_stop.set()
        except Exception as exc:
            print(f"[HW] capture loop fatal: {exc!r}")
            self._episode_stop.set()


# ── Entry point ───────────────────────────────────────────────────────────────


def main():
    global _teleop_state

    rclpy.init()
    node = _IntegratedNode()

    while not node._gripper_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Waiting for gripper service...")

    threading.Thread(target=rclpy.spin, args=(node,), daemon=True, name="ROS2-Spin").start()
    threading.Thread(target=_keyboard_thread_fn, args=(node,), daemon=True, name="KeyboardInput").start()
    threading.Thread(target=_teleop_ramp_thread_fn, daemon=True, name="VelocityRamp").start()

    dataset_root = f"/vla_dataset/syncro5/teleop_session_{int(time.time())}"

    session = HardwareDataLogger(
        node=node,
        dataset_root=dataset_root,
        config_path=DEFAULT_CONFIG_PATH,
        fps=30.0,
    )

    def _shutdown(sig, frame):
        print("\n[HW] Shutting down session...")
        _shutdown_event.set()
        session.close()
        _restore_terminal()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    session.open()

    print("\n" + "=" * 60)
    print("  Hardware Teleop Session (Integrated)")
    print(f"  Dataset → {dataset_root}")
    print("=" * 60)
    print("  q w e r t y → joint 1-6  +velocity")
    print("  a s d f g h → joint 1-6  -velocity")
    print("  u → open gripper   j → close gripper")
    print("  SPACE (camera window) → end episode")
    print("  q    (camera window) → quit session")
    print("  Ctrl-C               → quit session")
    print("=" * 60 + "\n")

    episode_num = 0

    try:
        while not session._session_stop.is_set() and not _shutdown_event.is_set():

            # ── Wait to start episode ─────────────────────────────────────────
            print(
                f"[Episode {episode_num + 1}] Press any key to start recording"
                "  (Ctrl-C to quit)..."
            )
            with _state_lock:
                _teleop_state = TeleopState.IDLE

            ch = _wait_episode_key()
            if ch == "\x03" or session._session_stop.is_set() or _shutdown_event.is_set():
                break

            episode_num += 1
            print(
                f"[Episode {episode_num}] Recording started."
                "  Press SPACE in the camera window to end."
            )

            # ── Record ───────────────────────────────────────────────────────
            with _state_lock:
                _teleop_state = TeleopState.RECORDING

            logger = session.start_episode(episode_num)

            while not session._episode_stop.is_set() and not _shutdown_event.is_set():
                time.sleep(0.05)

            # Transition state before end_episode so ramp thread zeroes out
            with _state_lock:
                _teleop_state = TeleopState.IDLE

            logger = session.end_episode()
            frame_count = logger._frame_idx if logger else 0
            print(f"[Episode {episode_num}] Stopped — {frame_count} frames captured.")

            if session._session_stop.is_set() or _shutdown_event.is_set():
                if logger:
                    print(f"[Episode {episode_num}] Session quit — saving episode...")
                    logger.finalize()
                break

            # ── Save / discard ────────────────────────────────────────────────
            print(f"[Episode {episode_num}] Save this episode?  y = save   n = discard")
            with _state_lock:
                _teleop_state = TeleopState.SAVE_PROMPT

            while True:
                ch = _wait_episode_key()
                if ch == "y":
                    logger.finalize()
                    print(f"[Episode {episode_num}] Saved  ({frame_count} frames).")
                    break
                elif ch == "n":
                    logger.discard()
                    episode_num -= 1
                    print(f"[Episode {episode_num + 1}] Discarded.")
                    break
                elif ch == "\x03":
                    logger.finalize()
                    print(f"[Episode {episode_num}] Ctrl-C — saving and quitting.")
                    session._session_stop.set()
                    break

            if session._session_stop.is_set() or _shutdown_event.is_set():
                break

            # ── Home robot ────────────────────────────────────────────────────
            session.home()

            # ── Scene reset prompt ────────────────────────────────────────────
            print()
            print("  ┌─────────────────────────────────────────────────────┐")
            print("  │  ACTION REQUIRED: Put the object back to its        │")
            print("  │  starting position and reset the scene.             │")
            print("  │                                                     │")
            print("  │  Press any key when ready for the next episode...   │")
            print("  └─────────────────────────────────────────────────────┘")
            print()
            with _state_lock:
                _teleop_state = TeleopState.IDLE

            ch = _wait_episode_key()
            if ch == "\x03":
                break

    except KeyboardInterrupt:
        pass
    finally:
        _shutdown_event.set()
        session.close()
        _restore_terminal()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass
        print(f"\n[HW] Session complete — {episode_num} episode(s) saved to {dataset_root}")


if __name__ == "__main__":
    main()
