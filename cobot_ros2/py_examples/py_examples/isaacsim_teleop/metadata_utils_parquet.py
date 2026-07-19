"""
metadata_utils_parquet.py
=========================
Helper functions for reading and writing the LeRobot-compatible parquet dataset.
Directory layout produced:
    <dataset_root>/
        meta/
            info.json
            tasks.parquet
            episodes/
                chunk-000/
                    file-000.parquet
        data/
            chunk-000/
                file-000.parquet
        images/
            observation.images.<cam>/
                episode_XXX/
                    <timestamp>.jpg
"""

import json
import os
from pathlib import Path

import numpy as np
import pandas as pd

# ── Pull constants from config ──────────────────────────────────────────────
# We import lazily so this module can also be used server-side (where config.py
# may or may not be present). In that case supply a fallback config dict via
# set_config().

_config = None


def set_config(cfg):
    """Inject a config namespace (or dict-like object) from outside."""
    global _config
    _config = cfg


def _get(key, default=None):
    """Safe getter from the injected or imported config."""
    global _config
    if _config is None:
        try:
            import config as _c
            _config = _c
        except ImportError:
            pass
    if _config is None:
        return default
    if hasattr(_config, "get"):
        return _config.get(key, default)
    return getattr(_config, key, default)


# ── Filesystem helpers ───────────────────────────────────────────────────────

def ensure_dir(path: Path):
    path.mkdir(parents=True, exist_ok=True)


def write_json(path: Path, data: dict):
    with open(path, "w") as f:
        json.dump(data, f, indent=4)


# ── Feature schema ───────────────────────────────────────────────────────────

def get_default_features(cameras: list[str]) -> dict:
    """
    Returns the feature schema dict written into meta/info.json.

    Schema:
        observation.state.joint_positions  float32 [JOINT_DIM]   — arm joints
        observation.state.base             float32 [2]           — [lin_vel, omega]
        action.robot                       float32 [ACTION_DIM]  — arm joint targets
        action.base                        float32 [2]           — [lin_vel, omega]
        base.timestamp_us                  int64   [1]           — hw timestamp
        observation.images.<cam>           image   [H, W, 3]
    """
    joint_dim       = _get("JOINT_DIM",       14)
    action_dim      = _get("ACTION_DIM",      14)
    base_state_dim  = _get("BASE_STATE_DIM",   2)
    base_action_dim = _get("BASE_ACTION_DIM",  2)
    log_h           = _get("LOG_HEIGHT",      240)
    log_w           = _get("LOG_WIDTH",       320)

    features = {
        "index":           {"dtype": "int64",   "shape": [1]},
        "episode_index":   {"dtype": "int64",   "shape": [1]},
        "frame_index":     {"dtype": "int64",   "shape": [1]},
        "task_index":      {"dtype": "int64",   "shape": [1]},
        "timestamp":       {"dtype": "float32", "shape": [1]},
        "observation.state.joint_positions": {
            "dtype": "float32", "shape": [joint_dim + base_state_dim],
        },
        "action.robot": {
            "dtype": "float32", "shape": [action_dim + base_action_dim],
        },
    }

    # if base_state_dim > 0:
    #     features["observation.state.base"] = {
    #         "dtype": "float32", "shape": [base_state_dim],
    #     }
    #     features["action.base"] = {
    #         "dtype": "float32", "shape": [base_action_dim],
    #     }
    #     features["base.timestamp_us"] = {"dtype": "int64", "shape": [1]}
    for cam in cameras:
        features[f"observation.images.{cam}"] = {
            "dtype": "image",
            "shape": [log_h, log_w, 3],
        }
    return features


# ── meta/info.json ───────────────────────────────────────────────────────────

def update_info_json(out_root: Path, total_episodes: int, total_frames: int,
                     cameras: list[str], image_shape=None):
    """Write (overwrite) meta/info.json with current totals."""
    features = get_default_features(cameras)
    if image_shape:
        for cam in cameras:
            features[f"observation.images.{cam}"]["shape"] = list(image_shape)

    info = {
        "codebase_version": _get("CODEBASE_VERSION", "v3.0"),
        "robot_type":       _get("ROBOT_TYPE",       "dual_arm_mobile"),
        "fps":              _get("RATE_LOGGING_METADATA",  30.0),
        "total_episodes":   total_episodes,
        "total_frames":     total_frames,
        "splits": {
            "train": {
                "from_episode": 0,
                "to_episode":   total_episodes,
            }
        },
        "features":    features,
        "data_path":   "data",
        "video_path":  None,
        "chunks_size": total_frames,
    }
    ensure_dir(out_root / "meta")
    write_json(out_root / "meta" / "info.json", info)


# ── meta/tasks.parquet ───────────────────────────────────────────────────────

def update_tasks_parquet(out_root: Path):
    """Write (overwrite) meta/tasks.parquet from EPISODE_TASK_RANGES."""
    task_ranges = _get("EPISODE_TASK_RANGES", [(0, 9999, "task", "task")])
    task_indices = []
    task_names   = []
    for task_idx, (start, end, task_name, _) in enumerate(task_ranges):
        task_indices.append(task_idx)
        task_names.append(task_name)

    df = pd.DataFrame({
        "task_index":         pd.Series(task_indices, dtype="int64"),
        "__index_level_0__":  pd.Series(task_names,   dtype="string"),
    })
    ensure_dir(out_root / "meta")
    df.to_parquet(out_root / "meta" / "tasks.parquet", index=False)


# ── data/chunk-000/file-000.parquet ─────────────────────────────────────────

def append_to_data_parquet(out_root: Path, new_rows: list) -> int:
    """
    Append *new_rows* to the main data parquet file.
    Returns the new total frame count.
    """
    data_dir    = out_root / "data" / "chunk-000"
    ensure_dir(data_dir)
    parquet_path = data_dir / "file-000.parquet"

    new_df = pd.DataFrame(new_rows)

    # Enforce core schema dtypes (keeps downstream readers stable)
    for col in ("index", "episode_index", "frame_index", "task_index"):
        if col in new_df.columns:
            new_df[col] = new_df[col].astype("int64")
    if "timestamp" in new_df.columns:
        # Store as float32 per meta/info.json schema.
        new_df["timestamp"] = new_df["timestamp"].astype("float32")

    # Ensure correct dtypes for numpy array columns
    for col in ("observation.state.joint_positions",
                "action.robot",
                ):
        if col in new_df.columns:
            new_df[col] = new_df[col].apply(
                lambda x: np.array(x, dtype=np.float32) if x is not None else None
            )

    # Image columns are stored as parquet-friendly references (relative JPEG paths).
    for col in ("observation.images.ego", "observation.images.external"):
        if col in new_df.columns:
            new_df[col] = new_df[col].astype("string")

    if parquet_path.exists():
        old_df      = pd.read_parquet(parquet_path)
        combined_df = pd.concat([old_df, new_df], ignore_index=True)
    else:
        combined_df = new_df

    combined_df.to_parquet(parquet_path, engine="pyarrow", index=False)
    return len(combined_df)


# ── meta/episodes/chunk-000/file-000.parquet ────────────────────────────────

def append_to_episodes_parquet(out_root: Path, episode_index: int,
                                length: int, start_index: int,
                                task_prompt: str) -> int:
    """Append one row to the episodes metadata parquet. Returns new row count."""
    episodes_dir  = out_root / "meta" / "episodes" / "chunk-000"
    ensure_dir(episodes_dir)
    parquet_path  = episodes_dir / "file-000.parquet"

    new_row = {
        "episode_index":      episode_index,
        "dataset_from_index": start_index,
        "dataset_to_index":   start_index + length,
        "length":             length,
        "tasks":              task_prompt,
    }
    new_df = pd.DataFrame([new_row])

    if parquet_path.exists():
        old_df      = pd.read_parquet(parquet_path)
        combined_df = pd.concat([old_df, new_df], ignore_index=True)
    else:
        combined_df = new_df

    combined_df.to_parquet(parquet_path, engine="pyarrow", index=False)
    return len(combined_df)


# ── Session-level stats reader ───────────────────────────────────────────────

def get_session_stats(out_root: Path) -> tuple[int, int]:
    """
    Read current total_episodes and total_frames from meta/info.json.
    Returns (0, 0) if not yet initialised.
    """
    info_path = out_root / "meta" / "info.json"
    if info_path.exists():
        with open(info_path) as f:
            info = json.load(f)
        return info["total_episodes"], info["total_frames"]
    return 0, 0
