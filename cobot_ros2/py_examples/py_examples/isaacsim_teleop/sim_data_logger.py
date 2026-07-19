"""
sim_data_logger.py
==================
Background 30-Hz logging thread that writes teleoperation data from
Isaac Sim into the same Parquet + JPEG format used by
addverb_vla_datacollection/data-collect.py.

Schema written:
    observation.state.joint_positions  float32  [7]  — sim arm (6) + gripper (1) joint positions
    action.robot                       float32  [7]  — commanded joint targets (same order)
    state.base                         float32  [2]  — zeros (no mobile base)
    action.base                        float32  [2]  — zeros (no mobile base)
    base.timestamp_us                  int64    [1]  — leader hardware timestamp µs
    observation.images.<cam>           JPEG image    — per camera

Output directory tree:
    <dataset_root>/
        meta/
            info.json
            tasks.parquet
            episodes/chunk-000/file-000.parquet
        data/chunk-000/file-000.parquet
        images/observation.images.<cam>/episode_XXX/<ts>.jpg
"""

import os
import sys
import time
import threading
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import torch
import yaml

# ── Locate and import addverb_vla_datacollection helpers ─────────────────────
# Prefer the local copy bundled in this directory (works inside the Docker
# container where addverb_arms/ is NOT mounted). Fall back to the canonical
# file in addverb_arms/addverb_vla_datacollection/ on the host.
_HERE    = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)
_DC_ROOT = os.path.abspath(os.path.join(_HERE, "..", "..", "addverb_arms", "addverb_vla_datacollection"))
if os.path.isdir(_DC_ROOT) and _DC_ROOT not in sys.path:
    sys.path.append(_DC_ROOT)

DEFAULT_CONFIG_PATH = "data_logger_config.yaml"


from metadata_utils_parquet import (   # noqa: E402
    append_to_data_parquet,
    append_to_episodes_parquet,
    update_info_json,
    update_tasks_parquet,
    get_session_stats,
    set_config,
)


# ── Config loader (inline — avoids circular imports) ─────────────────────────

def _load_cfg(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


class SimDataLogger(threading.Thread):
    """
    Background thread that drains a frame queue at the configured logging
    rate (default 30 Hz) and writes Parquet rows + JPEG images.

    Usage
    -----
        logger = SimDataLogger(dataset_root="/data/ep0", episode_num=0, ...)
        logger.init_episode()   # creates dirs, starts thread
        logger.push(action_joints, obs_joints, visual_obs, leader_ts)
        ...
        logger.finalize()       # flushes rows, writes Parquet, updates meta/info.json
    """

    def __init__(
        self,
        dataset_root: str,
        episode_num: int,
        cameras: dict,       # {"ego": {"width": 640, "height": 480}, ...}
        fps: float = 30.0,
        dataset_config_path: str = DEFAULT_CONFIG_PATH,
    ):
        super().__init__(daemon=True, name="SimDataLogger")

        # ── Config ───────────────────────────────────────────────────────────
        if dataset_config_path is None:
            dataset_config_path = os.path.join(
                _HERE, "..", "config", "dataset_config.yaml"
            )
        self._cfg = _load_cfg(dataset_config_path)
        dc = self._cfg["dataset"]
        dof = self._cfg["dof"]
        img_cfg = self._cfg["images"]
        log_cfg = self._cfg["logging"]
        task_ranges = self._cfg.get("task_ranges", [[0, 9999, "Manipulation", "Manipulate object."]])

        self.out_root      = Path(dataset_root)
        self.episode_num   = episode_num
        self.cameras       = cameras                      # dict of cam names
        self.fps           = fps
        self._dt           = 1.0 / fps

        self._joint_dim    = dof["joint_dim"]             # 7
        self._action_dim   = dof["action_dim"]            # 7
        self._base_obs_dim = dof["base_obs_dim"]          # 0
        self._base_act_dim = dof["base_act_dim"]          # 0

        self._log_w        = int(img_cfg["log_width"])
        self._log_h        = int(img_cfg["log_height"])
        self._jpeg_q       = int(img_cfg["jpeg_quality"])
        self._cam_names    = list(img_cfg["cameras"])

        # Validate the loaded config loud-and-clear. cv2.resize dies with an
        # opaque `!dsize.empty()` error if these silently come back as 0, so
        # surface the problem HERE with an actionable message.
        if self._log_w <= 0 or self._log_h <= 0:
            raise ValueError(
                f"[SimDataLogger] bad image dims in {dataset_config_path}: "
                f"log_width={self._log_w}, log_height={self._log_h} (need > 0)"
            )
        print(f"[SimDataLogger] config       : {dataset_config_path}")
        print(f"[SimDataLogger]   log image  : {self._log_w}x{self._log_h}  "
              f"jpeg_q={self._jpeg_q}")
        print(f"[SimDataLogger]   cameras    : {self._cam_names}")
        print(f"[SimDataLogger]   joint/act  : {self._joint_dim}/{self._action_dim}")
        print(f"[SimDataLogger]   rate       : {self.fps:.1f} Hz")

        self._task_ranges  = task_ranges
        self._task_prompt  = self._get_task_prompt(episode_num)
        self._debug_interval = max(1, int(fps / log_cfg.get("debug_freq_hz", 1.0)))

        # ── Internal state ────────────────────────────────────────────────────
        self._queue: deque   = deque(maxlen=int(log_cfg.get("max_queue_depth", 500)))
        self._queue_lock     = threading.Lock()
        self._rows: list     = []
        self._running        = True
        self._frame_idx      = 0
        self._dataset_start_index = 0
        self._task_index = None

        # Inject the sim-side constants into metadata_utils_parquet so the
        # meta/info.json that it writes reflects OUR config (joint / image
        # shape, fps, task list, codebase + robot type) instead of the
        # addverb_vla_datacollection defaults baked into config.py.
        set_config({
            "CODEBASE_VERSION":      dc.get("codebase_version", "v3.0"),
            "ROBOT_TYPE":            dc.get("robot_type", "addverb_sim_6dof"),
            "RATE_LOGGING_METADATA": self.fps,
            "LOG_WIDTH":             self._log_w,
            "LOG_HEIGHT":            self._log_h,
            "JOINT_DIM":             self._joint_dim,
            "ACTION_DIM":            self._action_dim,
            "BASE_STATE_DIM":        self._base_obs_dim,
            "BASE_ACTION_DIM":       self._base_act_dim,
            "EPISODE_TASK_RANGES":   task_ranges,
        })

    # ── Public API ────────────────────────────────────────────────────────────

    def init_episode(self):
        """Create output directories and start the background logging thread."""
        self.out_root.mkdir(parents=True, exist_ok=True)
        for cam in self._cam_names:
            img_dir = self._image_dir(cam)
            img_dir.mkdir(parents=True, exist_ok=True)
        update_tasks_parquet(self.out_root)
        # Cache the starting global row index for this episode so every row gets
        # a monotonic dataset-wide `index` even before we write the parquet file.
        _total_eps, frames_before = self._session_stats()
        self._dataset_start_index = int(frames_before)
        # Infer task_index for this episode using the configured episode ranges,
        # validating against meta/tasks.parquet (task_index -> task name list).
        self._task_index = self._infer_task_index(self.episode_num)
        self.start()
        print(f"[SimDataLogger] Episode {self.episode_num} started → {self.out_root}")

    def push(
        self,
        action_joints: np.ndarray,    # (7,) commanded joint targets; indices 0-5 arm, 6 gripper
        obs_joints:    np.ndarray,    # (7,) sim observed joint positions
        visual_obs:    dict,          # {"rgb_ego": Tensor(H,W,3), "rgb_external": Tensor(H,W,3), ...}
        leader_ts:     float,         # float — hardware leader timestamp (seconds)
    ):
        """Non-blocking — drops oldest frame if queue is full."""
        frame = {
            "action_joints": action_joints.copy(),
            "obs_joints":    obs_joints.copy(),
            # Copy image data so the sim loop can proceed; only for registered cameras
            "images":        {
                cam: visual_obs[f"rgb_{cam}"].clone()
                     if isinstance(visual_obs.get(f"rgb_{cam}"), torch.Tensor)
                     else np.array(visual_obs.get(f"rgb_{cam}", []))
                for cam in self._cam_names
                if f"rgb_{cam}" in visual_obs
            },
            "leader_ts": leader_ts,
        }

        self._write_row(frame)

        # with self._queue_lock:
        #     self._queue.append(frame)

    def discard(self):
        """Abandon the current episode without writing Parquet.

        Stops the background thread, drops in-memory rows, and removes the
        per-camera per-episode JPEG directories that were already written
        to disk. The dataset's meta/ files are NOT touched (they only get
        updated by finalize()).
        """
        import shutil
        self._running = False
        self.join(timeout=5.0)
        with self._queue_lock:
            self._queue.clear()
        self._rows.clear()
        # Remove per-camera per-episode image directories that this run wrote.
        for cam in self._cam_names:
            ep_dir = self._image_dir(cam)
            if ep_dir.exists():
                shutil.rmtree(ep_dir, ignore_errors=True)
        print(f"[SimDataLogger] discarded episode {self.episode_num} (no Parquet written)")

    def finalize(self):
        """
        Signal the thread to stop, flush remaining rows to Parquet,
        and update meta/info.json.
        """
        self._running = False
        self.join(timeout=5.0)

        if not self._rows:
            print("[SimDataLogger] No data to save.")
            return

        total_eps, frames_before = self._session_stats()

        append_to_data_parquet(self.out_root, self._rows)
        append_to_episodes_parquet(
            self.out_root,
            episode_index=self.episode_num,
            length=len(self._rows),
            start_index=frames_before,
            task_prompt=self._task_prompt,
        )
        update_info_json(
            self.out_root,
            total_episodes=total_eps + 1,
            total_frames=frames_before + len(self._rows),
            cameras=self._cam_names,
            image_shape=[self._log_h, self._log_w, 3],
        )
        print(
            f"[SimDataLogger] Saved {len(self._rows)} frames "
            f"(episode {self.episode_num}) → {self.out_root}"
        )

    # ── Thread body ───────────────────────────────────────────────────────────

    # def run(self):
    #     next_t = time.time() + self._dt
    #     _err_last_log = 0.0
    #     while self._running:
    #         now = time.time()
    #         if now >= next_t:
    #             with self._queue_lock:
    #                 # Take the most recent frame (drop stale ones silently)
    #                 frame = self._queue[-1] if self._queue else None
    #                 if frame is not None:
    #                     self._queue.clear()

    #             if frame is not None:
    #                 # A per-frame exception must NOT kill the logger thread —
    #                 # losing the logger silently is how we ended up with a
    #                 # stuck-but-not-recording sim in the past.
    #                 try:
    #                     self._write_row(frame)
    #                 except Exception as e:
    #                     if now - _err_last_log > 1.0:
    #                         print(f"[SimDataLogger] frame write error: {e!r}")
    #                         _err_last_log = now

    #             next_t += self._dt

    #         time.sleep(0.0005)   # ~0.5 ms granularity; avoids busy-wait

    # ── Private helpers ───────────────────────────────────────────────────────

    def _write_row(self, frame: dict):
        action  = frame["action_joints"].astype(np.float32)   # (7,)
        obs     = frame["obs_joints"].astype(np.float32)       # (7,)
        ts      = frame["leader_ts"]

        # Zero-pad for base dims (maintains schema compatibility if base_obs_dim=2)
        state_vec  = np.concatenate([obs,    np.zeros(self._base_obs_dim, dtype=np.float32)])
        action_vec = np.concatenate([action, np.zeros(self._base_act_dim, dtype=np.float32)])

        ts_str = f"{ts:.6f}"
        # Save JPEG images and capture the exact on-disk paths we wrote.
        saved_rel_paths: dict[str, str | None] = {cam: None for cam in self._cam_names}
        for cam in self._cam_names:
            img_raw = frame["images"].get(cam)
            if img_raw is None:
                continue
            if isinstance(img_raw, torch.Tensor):
                img_np = img_raw.cpu().numpy()
            else:
                img_np = np.asarray(img_raw)

            # Isaac Lab TiledCamera returns (num_envs, H, W, C). cv2.resize
            # only accepts 2D or 3D arrays; a 4D input crashes with the
            # `!dsize.empty()` assertion because the numpy→Mat conversion
            # can't infer a 2D image. Strip the leading env dim.
            if img_np.ndim == 4 and img_np.shape[0] == 1:
                img_np = img_np[0]

            # Skip frames with no pixels (camera not ready yet on early steps).
            if img_np.size == 0 or img_np.ndim < 2:
                continue

            if img_np.dtype != np.uint8:
                img_np = (np.clip(img_np, 0, 255)).astype(np.uint8)

            # Resize to log resolution
            img_resized = cv2.resize(img_np, (self._log_w, self._log_h))
            img_bgr     = cv2.cvtColor(img_resized, cv2.COLOR_RGB2BGR)
            out_path    = self._image_dir(cam) / f"{ts_str}.jpg"
            cv2.imwrite(str(out_path), img_bgr, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_q])
            try:
                saved_rel_paths[cam] = str(out_path.relative_to(self.out_root))
            except Exception:
                saved_rel_paths[cam] = str(out_path)

        row = {
            # Dataset-required columns only
            "index": int(self._dataset_start_index + self._frame_idx),
            "episode_index": int(self.episode_num),
            "frame_index": int(self._frame_idx),
            "task_index": int(self._task_index if self._task_index is not None else 0),
            "timestamp": float(ts),
            "observation.state.joint_positions": state_vec.tolist(),
            "action.robot": action_vec.tolist(),
            # Parquet-friendly image references: exactly the paths we wrote above
            "observation.images.ego": saved_rel_paths.get("ego"),
            "observation.images.external": saved_rel_paths.get("external"),
        }
        self._rows.append(row)

        if self._frame_idx % self._debug_interval == 0:
            print(
                f"[SimDataLogger] frame={self._frame_idx:5d}  "
                f"arm={np.round(obs[:6], 3)}  gripper={obs[6]:.4f}"
            )

        self._frame_idx += 1

    def _image_dir(self, cam: str) -> Path:
        return (
            self.out_root
            / "images"
            / f"observation.images.{cam}"
            / f"episode_{self.episode_num:03d}"
        )

    def _session_stats(self):
        try:
            return get_session_stats(self.out_root)
        except Exception:
            return 0, 0

    def _get_task_prompt(self, episode_num: int) -> str:
        for entry in self._task_ranges:
            start, end, _name, prompt = entry
            if start <= episode_num <= end:
                return prompt
        return "Manipulation task."

    def _infer_task_index(self, episode_num: int) -> int:
        """
        Infer per-episode task_index using configured episode ranges and the
        meta/tasks.parquet task list.

        Mapping logic:
        - tasks.parquet stores the ordered list of tasks with their numeric task_index.
        - data_logger_config.yaml defines EPISODE_TASK_RANGES as [start_episode, end_episode, task_name, prompt].
        - For a given episode_num, we pick the first matching range and look up its task_name in tasks.parquet.
          If task_name is not present (or tasks.parquet is missing/unreadable), we fall back to the range index.
        """
        import pandas as pd

        # Determine candidate task_index from configured episode ranges.
        candidate_idx = 0
        candidate_name = None
        for idx, (start, end, task_name, _prompt) in enumerate(self._task_ranges):
            if int(start) <= int(episode_num) <= int(end):
                candidate_idx = int(idx)
                candidate_name = str(task_name)
                break

        tasks_path = self.out_root / "meta" / "tasks.parquet"
        try:
            if tasks_path.exists():
                df = pd.read_parquet(tasks_path)
                # Expect: task_index int64, __index_level_0__ string (task_name)
                if "task_index" in df.columns and "__index_level_0__" in df.columns:
                    name_to_idx = {
                        str(name): int(ti)
                        for ti, name in zip(df["task_index"].tolist(), df["__index_level_0__"].tolist())
                    }
                    if candidate_name is not None and candidate_name in name_to_idx:
                        return int(name_to_idx[candidate_name])
                    if candidate_name is not None and candidate_name not in name_to_idx:
                        print(
                            f"[SimDataLogger] WARN: task_name {candidate_name!r} not found in meta/tasks.parquet; "
                            f"falling back to task_index={candidate_idx}"
                        )
        except Exception as e:
            print(f"[SimDataLogger] WARN: failed reading meta/tasks.parquet for task inference: {e!r}")

        return candidate_idx
