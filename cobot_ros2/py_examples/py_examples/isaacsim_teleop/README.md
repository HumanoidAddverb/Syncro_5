# isaacsim_teleop — Hardware Data Collection for VLA / RL Training

Collect robot + camera demonstration data on the **Syncro5** hardware in the **LeRobot / GROOT** dataset format (Parquet + JPEG). The output is schema-compatible with the simulation datasets produced by the [`syncro_isaac_sim`](https://github.com/HumanoidAddverb/syncro_isaac_sim.git) repo, so **real and sim episodes can be mixed and trained together**.

**Location:** `~/cobot_ros2_ws/src/Syncro_5/cobot_ros2/py_examples/py_examples/isaacsim_teleop/`

> Full robot bring-up (Docker, Heal Server, control manager) is covered in the workspace docs [Setup.md](../../../Setup.md) and [control.md](../../../control.md). This guide covers **only** the data-collection workflow.

---

## Table of Contents

1. [Scripts](#scripts)
2. [Prerequisites](#prerequisites)
3. [Install dependencies](#install-dependencies)
4. [Collect data](#collect-data)
5. [Output dataset structure](#output-dataset-structure)
6. [Validate & inspect](#validate--inspect)
7. [Dataset compatibility with `syncro_isaac_sim`](#dataset-compatibility-with-syncro_isaac_sim)
8. [Configuration reference](#configuration-reference)

---

## Scripts

```
isaacsim_teleop/
├── hw_with_cam.py               # Data logger (robot state + RealSense cameras)
├── hw_teleop_integrated.py      # Data logger + built-in keyboard teleop (all-in-one)
├── keyboard_teleop_only_hw.py   # Keyboard teleoperation only (no logging)
├── sim_data_logger.py           # Core Parquet/JPEG logging engine  (used internally)
├── metadata_utils_parquet.py    # meta/info.json + tasks.parquet writer (used internally)
├── data_logger_config.yaml      # Dataset schema config
├── validate_data.py             # Post-collection data validation
├── view_parquet.py              # Dump a Parquet file to CSV for inspection
├── extract_state_action.py      # Extract state/action arrays from an episode
└── requirements.txt             # Python dependencies
```

| Script | Purpose |
|---|---|
| `hw_with_cam.py` | **Main logger** — run alongside a separate teleop terminal. |
| `hw_teleop_integrated.py` | Logger **and** keyboard teleop in one process (no second terminal). |
| `keyboard_teleop_only_hw.py` | Teleop only — drive the robot without recording. |
| `validate_data.py` / `view_parquet.py` / `extract_state_action.py` | Post-collection utilities (no robot needed). |

---

## Prerequisites

Before collecting data, bring up the robot and make the **velocity controller** active (see [Setup.md](../../../Setup.md) / [control.md](../../../control.md) for details). At minimum:

```bash
# In every new terminal
cd ~/cobot_ros2_ws && source install/setup.bash
```

Activate `velocity_controller` (teleop drives the robot through it):

```bash
ros2 service call /controller_manager/switch_controller \
controller_manager_msgs/srv/SwitchController "{
  activate_controllers: [\"velocity_controller\"],
  deactivate_controllers: [\"\"],
  strictness: 1
}"

# verify
ros2 control list_controllers      # velocity_controller ... active
```

**Hardware:** two Intel RealSense cameras connected (enumerated as `ego` and `external`), and the gripper service (`/gripper_controller/command`) live.

---

## Install dependencies

First time only:

```bash
cd ~/cobot_ros2_ws/src/Syncro_5/cobot_ros2/py_examples/py_examples/isaacsim_teleop/
pip install -r requirements.txt
```

> ROS 2 packages (`rclpy`, `sensor_msgs`, …) come from the sourced workspace, not `pip`.

---

## Collect data

### Option A — logger + separate teleop (two terminals)

**Terminal 1 — data logger:**

```bash
cd ~/cobot_ros2_ws && source install/setup.bash
cd ~/cobot_ros2_ws/src/Syncro_5/cobot_ros2/py_examples/py_examples/isaacsim_teleop/
python3 hw_with_cam.py
```

Expected:

```
[HardwareDataLogger] started → /vla_dataset/syncro5/teleop_session_<timestamp>
[HardwareDataLogger] logging — press Ctrl+C to stop and save.
```

Two live camera preview windows open (`Camera: ego`, `Camera: external`).

**Terminal 2 — keyboard teleop:**

```bash
cd ~/cobot_ros2_ws && source install/setup.bash
cd ~/cobot_ros2_ws/src/Syncro_5/cobot_ros2/py_examples/py_examples/isaacsim_teleop/
python3 keyboard_teleop_only_hw.py
```

### Option B — all-in-one (single terminal)

```bash
cd ~/cobot_ros2_ws && source install/setup.bash
cd ~/cobot_ros2_ws/src/Syncro_5/cobot_ros2/py_examples/py_examples/isaacsim_teleop/
python3 hw_teleop_integrated.py
```

### Keyboard mapping

| Key | Action |
|---|---|
| `q w e r t y` | Joints 1–6 **+** velocity |
| `a s d f g h` | Joints 1–6 **−** velocity |
| `u` | Open gripper |
| `j` | Close gripper |
| `Ctrl+C` | Stop |

> Velocities ramp smoothly (≈0.4 rad/s²); releasing a key ramps back to zero.

### Stopping / saving

- Press **`q`** in a camera preview window, **or** **`Ctrl+C`** in the logger terminal.
- The Parquet file is finalized, camera pipelines released, and the ROS node destroyed cleanly. The dataset is saved under `dataset_root` (default `/vla_dataset/syncro5/teleop_session_<ts>`).

---

## Output dataset structure

```
/vla_dataset/syncro5/teleop_session_<unix_timestamp>/
├── data/chunk-000/file-000.parquet         # per-frame state, action, timestamps, image refs
├── images/
│   ├── observation.images.ego/episode_001/<ts>.jpg
│   └── observation.images.external/episode_001/<ts>.jpg
└── meta/
    ├── info.json                           # feature schema, robot_type, fps, counts
    ├── tasks.parquet
    └── episodes/chunk-000/file-000.parquet
```

**Parquet schema (per frame):**

| Column | dtype | Notes |
|---|---|---|
| `index`, `episode_index`, `frame_index`, `task_index` | int64 | dataset-wide + per-episode indices |
| `timestamp` | float32 | seconds |
| `observation.state.joint_positions` | float32 `[7]` | `[j1..j6, gripper]` |
| `action.robot` | float32 `[7]` | `[vel_j1..vel_j6, gripper]` |
| `observation.images.ego`, `observation.images.external` | string | relative JPEG paths |

`meta/info.json`: `robot_type: syncro5`, `codebase_version: v3.0`, `fps: 30`.

**ROS topics/services used while logging:**

| Name | Type | Direction |
|---|---|---|
| `/joint_states` | `sensor_msgs/JointState` | subscribed → observation |
| `/velocity_controller/commands` | `std_msgs/Float64MultiArray` | subscribed → action |
| `/gripper_action_command` | `std_msgs/Float32` | subscribed → gripper state |
| `/gripper_controller/command` | `addverb_cobot_msgs/srv/Gripper` | service (teleop) |

---

## Validate & inspect

After a session, sanity-check the episode. Edit the `root` path at the top of `validate_data.py` to point to your session, then:

```bash
python3 validate_data.py
```

Checks: image count vs Parquet rows, corrupt images, NaNs in `observation.state.joint_positions` / `action.robot`, null timestamps, and timestamp deltas (~0.033 s at 30 Hz).

A clean episode looks like:

```
Total rows in parquet: 450
EGO      → total: 450, missing: 0, corrupt: 0
EXTERNAL → total: 450, missing: 0, corrupt: 0
Counts match ✅
Bad observation rows: 0
Bad action rows     : 0
Bad timestamps      : 0
Mean dt: 0.0333
```

**Discard an episode if:** image count ≠ Parquet rows, >1 % of joint rows contain NaN, or max timestamp delta > 0.5 s (dropped frames).

Inspect raw Parquet as CSV (edit its `root` path first):

```bash
python3 view_parquet.py          # writes a CSV of the episode
python3 extract_state_action.py  # extract state/action arrays
```

---

## Dataset compatibility with `syncro_isaac_sim`

Hardware datasets use the **same logging engine and dataset config as the sim pipeline**, so they share the exact LeRobot/GROOT schema and train together with sim data. These files are vendored into this folder (identical to the upstream `syncro_isaac_sim` versions):

```
sim_data_logger.py          metadata_utils_parquet.py          data_logger_config.yaml
```

The loggers run **stand-alone** here — no external checkout needed. If you clone `syncro_isaac_sim` as a sibling of the repo, it is picked up automatically as the single source of truth:

```bash
git clone https://github.com/HumanoidAddverb/syncro_isaac_sim.git \
    ~/cobot_ros2_ws/src/Syncro_5/syncro_isaac_sim
```

> Keep the three vendored files **in sync with `syncro_isaac_sim`**. If the sim dataset schema changes, re-copy them (or clone the sibling repo), otherwise real and sim datasets drift apart.

**For training and evaluation**, follow the instructions at [syncro-sim.addverb.ai](https://syncro-sim.addverb.ai/training.html).

---

## Configuration reference

**`data_logger_config.yaml`** — dataset schema (rate, cameras, DOF, task labels):

```yaml
dataset:
  name: "syncro5_dataset"
  root: "/vla_dataset"        # where episodes are saved
  robot_type: "syncro5"
dof:
  joint_dim: 7                # [j1..j6, gripper] — must match logged vector width
  action_dim: 7
images:
  cameras: ["ego", "external"]
  log_width: 320
  log_height: 320
logging:
  rate_hz: 30.0
task_ranges:
  - [0, 9999, "Manipulation", "Pick up the object and place it in the designated location."]
```

**Script constants** (top of `hw_with_cam.py` / `hw_teleop_integrated.py`):

| Variable | Default | Description |
|---|---|---|
| `dataset_root` | `/vla_dataset/syncro5/teleop_session_<ts>` | Output directory |
| `DEFAULT_CONFIG_PATH` | local `data_logger_config.yaml` | Dataset schema config |
| `fps` | `30.0` | Logging frequency (Hz) |
| `TOPIC_TIMEOUT_S` | `2.0` | Watchdog timeout before an error is raised |

> `dof.joint_dim` / `dof.action_dim` are `7` so `info.json` declares shape `[7]`, matching the actual `[j1..j6, gripper]` data. Change these only alongside the matching change in `syncro_isaac_sim`, or hardware and sim datasets will no longer align.
