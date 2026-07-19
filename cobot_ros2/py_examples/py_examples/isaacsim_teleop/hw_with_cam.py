"""
hw_with_cam.py
==============
Multi-episode hardware teleoperation data logger.

Each run is one *session* (one dataset root). Within a session the operator
records as many episodes as needed. After each episode the robot is
automatically homed and the operator is prompted to reset the scene before
starting the next episode — mirroring the Isaac Sim teleop workflow.

Episode lifecycle
-----------------
  1. Press any key → start recording
  2. Operate the robot (keyboard_teleop_only_hw.py in a separate terminal)
  3. Press SPACE in the camera window → end episode
  4. Press  y = save  /  n = discard
  5. Robot homes automatically
  6. Put the object back, press any key → next episode
  7. Ctrl-C or q in camera window → quit session

Data sources
------------
  /joint_states                   → observation.state.joint_positions
  /velocity_controller/commands   → action.robot (arm velocities)
  /gripper_action_command         → action.robot[6] (gripper)
  RealSense D435 × 2              → observation.images.ego / .external

Output format
-------------
Uses the canonical SimDataLogger from syncro_isaac_sim — identical Parquet
schema to sim-collected data, so both can be mixed for Groot N1.7 training.
"""

import math
import os
import subprocess
import sys
import time
import threading
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Float32

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

# Path to the standalone homing script.
_HOMING_PY = os.path.abspath(os.path.join(_HERE, "..", "homing.py"))

# Seconds of silence on a topic before TopicTimeoutError is raised.
TOPIC_TIMEOUT_S: float = 2.0


# ── Custom exception ──────────────────────────────────────────────────────────

class TopicTimeoutError(RuntimeError):
    """Raised when a required ROS2 topic stops publishing for TOPIC_TIMEOUT_S."""
    def __init__(self, topic: str, elapsed: float):
        self.topic = topic
        self.elapsed = elapsed
        super().__init__(
            f"No data on '{topic}' for {elapsed:.1f} s "
            f"(timeout={TOPIC_TIMEOUT_S} s) — stopping logger."
        )


# ── ROS2 subscriber node ───────────────────────────────────────────────────────

class _DataSubscriberNode(Node):
    """Minimal ROS2 node: joint states + velocity commands + gripper."""

    def __init__(self):
        super().__init__("hardware_data_logger_node")

        self._obs_pos: dict[str, float] = {}
        self._obs_lock = threading.Lock()

        self._arm_action: list[float] = [0.0] * len(JOINT_NAMES)
        self._gripper_action: float = 0.0
        self._action_lock = threading.Lock()

        self._last_joint_state_t: float | None = None
        self._last_action_t: float | None = None
        self._watchdog_lock = threading.Lock()

        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)
        self.create_subscription(
            Float64MultiArray, "/velocity_controller/commands",
            self._velocity_commands_cb, 10
        )
        self.create_subscription(Float32, "/gripper_action_command", self._gripper_action_cb, 10)
        self.get_logger().info("_DataSubscriberNode: subscribed to joint_states, velocity_controller/commands, gripper_action_command")

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


# ── Multi-episode hardware logger ─────────────────────────────────────────────

class HardwareDataLogger:
    """
    Manages one collection session across multiple episodes.

    Usage
    -----
        session = HardwareDataLogger(dataset_root="/vla_dataset/...")
        session.open()                    # start cameras + ROS (once per session)

        logger = session.start_episode(0) # begin recording
        ...                               # operator tele-ops the robot
        logger = session.end_episode()    # stop capture; returns SimDataLogger
        logger.finalize()                 # or logger.discard()

        session.home()                    # send robot to zero (blocks ~10 s)

        session.close()                   # shut down cameras + ROS
    """

    def __init__(
        self,
        dataset_root: str,
        config_path: str = DEFAULT_CONFIG_PATH,
        fps: float = 30.0,
    ):
        self._dataset_root = dataset_root
        self._config_path = config_path
        self._fps = fps

        self._session_stop = threading.Event()
        self._episode_stop = threading.Event()

        self._node: _DataSubscriberNode | None = None
        self._ros_spin_thread: threading.Thread | None = None
        self._pipelines: dict[str, rs.pipeline] = {}
        self._capture_thread: threading.Thread | None = None
        self._sim_logger: SimDataLogger | None = None

    # ── Session lifecycle ─────────────────────────────────────────────────────

    def open(self):
        """Start RealSense cameras and ROS2 subscriber node. Call once per session."""
        if not rclpy.ok():
            rclpy.init()

        self._node = _DataSubscriberNode()
        self._ros_spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True, name="HW-ROS2Spin"
        )
        self._ros_spin_thread.start()

        # Open RealSense cameras; assign names in detection order.
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
        """Stop cameras, finalize any running episode, and shut down ROS2 node."""
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

        if self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None

        cv2.destroyAllWindows()
        print("[HW] Session closed.")

    # ── Episode lifecycle ─────────────────────────────────────────────────────

    def start_episode(self, episode_num: int) -> "SimDataLogger":
        """
        Create a new SimDataLogger for *episode_num* and start the capture loop.
        Returns the SimDataLogger so the caller can call finalize/discard later.
        """
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

    def end_episode(self) -> "SimDataLogger":
        """
        Stop the capture loop and return the SimDataLogger.
        Caller must call either logger.finalize() or logger.discard().
        """
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
        Home the robot to zero position using homing.py.

        Runs homing.py as a subprocess so its rclpy.init/shutdown lifecycle
        is isolated from this process's ROS2 context.  Blocks until homing
        completes (≤ 30 s) or times out.
        """
        if not os.path.isfile(_HOMING_PY):
            print(f"[HW] homing.py not found at {_HOMING_PY} — skipping.")
            return

        print("[HW] Homing robot to zero position...")
        try:
            result = subprocess.run(
                [sys.executable, _HOMING_PY],
                timeout=15.0,
            )
            if result.returncode != 0:
                print(f"[HW] Homing script exited with code {result.returncode}")
        except subprocess.TimeoutExpired:
            print("[HW] Homing timed out after 15 s — continuing anyway.")
        except Exception as exc:
            print(f"[HW] Homing error: {exc!r}")

        print("[HW] Homing done.")

    # ── Internal capture loop ─────────────────────────────────────────────────

    def _capture_loop(self):
        """30 Hz capture: read cameras + joints → push to SimDataLogger."""
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


# ── Standalone entry-point — multi-episode loop ───────────────────────────────

if __name__ == "__main__":
    import signal
    import tty
    import termios

    # ── Terminal helpers ──────────────────────────────────────────────────────
    STDIN_FD = sys.stdin.fileno()
    _ORIG_TERM = termios.tcgetattr(STDIN_FD)

    def _restore_terminal():
        try:
            termios.tcsetattr(STDIN_FD, termios.TCSADRAIN, _ORIG_TERM)
        except Exception:
            pass

    def _getch() -> str:
        """Read one character without requiring Enter. Returns empty string on error."""
        try:
            tty.setcbreak(STDIN_FD)
            ch = sys.stdin.read(1)
            _restore_terminal()
            return ch
        except Exception:
            _restore_terminal()
            return ""

    # ── Session setup ─────────────────────────────────────────────────────────
    dataset_root = f"/vla_dataset/syncro5/teleop_session_{int(time.time())}"

    session = HardwareDataLogger(
        dataset_root=dataset_root,
        config_path=DEFAULT_CONFIG_PATH,
        fps=30.0,
    )

    def _shutdown(sig, frame):
        print("\n[HW] Shutting down session...")
        session.close()
        _restore_terminal()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # ── Open session ──────────────────────────────────────────────────────────
    session.open()

    print("\n" + "=" * 60)
    print("  Hardware Teleop Session")
    print(f"  Dataset → {dataset_root}")
    print("=" * 60)
    print("  Run keyboard_teleop_only_hw.py in a SEPARATE terminal.")
    print("  SPACE (camera window) → end episode")
    print("  q     (camera window) → quit session")
    print("  Ctrl-C                → quit session")
    print("=" * 60 + "\n")

    episode_num = 0

    try:
        while not session._session_stop.is_set():

            # ── Wait for operator to start the episode ────────────────────────
            print(f"[Episode {episode_num + 1}] Press any key to start recording"
                  "  (Ctrl-C to quit)...")
            ch = _getch()
            if ch == "\x03" or session._session_stop.is_set():
                break

            episode_num += 1
            print(f"[Episode {episode_num}] Recording started."
                  "  Press SPACE in the camera window to end this episode.")

            # ── Record ───────────────────────────────────────────────────────
            logger = session.start_episode(episode_num)

            # Block until episode ends (SPACE / q in camera window, or Ctrl-C)
            while not session._episode_stop.is_set():
                time.sleep(0.05)

            # Pause capture — get the logger back for finalize/discard decision
            logger = session.end_episode()
            frame_count = logger._frame_idx if logger else 0
            print(f"[Episode {episode_num}] Stopped — {frame_count} frames captured.")

            # If session quit was requested (q or signal), save and exit
            if session._session_stop.is_set():
                if logger:
                    print(f"[Episode {episode_num}] Session quit — saving episode...")
                    logger.finalize()
                break

            # ── Save / discard prompt ─────────────────────────────────────────
            print(f"[Episode {episode_num}] Save this episode?  y = save   n = discard")
            saved = False
            while True:
                ch = _getch()
                if ch == "y":
                    logger.finalize()
                    saved = True
                    print(f"[Episode {episode_num}] Saved  ({frame_count} frames).")
                    break
                elif ch == "n":
                    logger.discard()
                    episode_num -= 1  # don't count discarded episodes
                    print(f"[Episode {episode_num + 1}] Discarded.")
                    break
                elif ch == "\x03":
                    logger.finalize()
                    saved = True
                    print(f"[Episode {episode_num}] Ctrl-C — saving and quitting.")
                    session._session_stop.set()
                    break

            if session._session_stop.is_set():
                break

            # ── Home the robot ────────────────────────────────────────────────
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
            ch = _getch()
            if ch == "\x03":
                break

    except KeyboardInterrupt:
        pass
    finally:
        session.close()
        _restore_terminal()
        print(f"\n[HW] Session complete — {episode_num} episode(s) saved to {dataset_root}")
        sys.exit(0)
