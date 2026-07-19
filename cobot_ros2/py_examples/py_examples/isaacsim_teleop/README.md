# Addverb Cobot ROS 2 

> **Prerequisites**: Make sure you have followed the steps in [Setup.md](Setup.md) before running any controllers. The robot must be physically connected and powered before proceeding.

---

## Table of Contents

1. [Repository Overview](#repository-overview)
2. [Quick Start — Robot Bring-Up](#quick-start--robot-bring-up)
3. [VLA / RL Training Data Collection](#vla--rl-training-data-collection)
   - [Step 1 — Start Heal Server & Connect via ROS 2 Stack](#step-1--start-heal-server--connect-via-ros-2-stack)
   - [Step 2 — Start Data Logger (`hw_with_cam.py`)](#step-2--start-data-logger-hw_with_campy)
   - [Step 3 — Control the Robot with Keyboard Teleop (`keyboard_teleop_only_hw.py`)](#step-3--control-the-robot-with-keyboard-teleop-keyboard_teleop_only_hwpy)
4. [Available Controllers](#available-controllers)
5. [Controller Management](#controller-management)
6. [ROS 2 Services Reference](#ros-2-services-reference)
7. [Environment Requirements](#environment-requirements)

---

## Repository Overview

```
medical_robotics-cobot-ros2/
├── addverb_cobot_control/        # Controller launch files & YAML configs
├── addverb_cobot_controllers/    # Custom ROS 2 controllers
├── addverb_cobot_description/    # URDF / xacro robot description
├── addverb_cobot_hardware/       # Hardware interface (EtherCAT, Modbus, FT sensor)
├── addverb_cobot_msgs/           # Custom ROS 2 message / service / action types
├── addverb_moveit_configs/       # MoveIt 2 configuration
├── examples/                     # Pre-built demo executables
├── py_example/
│   └── py_example/
│       └── isaacsim_teleop/      # ← VLA/RL data collection scripts (see below)
├── teleoperation/                # Haptic / leader-follower teleoperation
├── control.md                    # Detailed controller reference
└── Setup.md                      # Initial environment setup guide
```

---

## Quick Start — Robot Bring-Up

### 1 — Run the Docker container

```bash
# Resume an existing container
sudo ./cobot run

# Create a brand-new container
sudo ./cobot create

# Remove a container
sudo ./cobot remove
```

### 2 — Start the Heal Server

The Heal Server is the low-level bridge between the hardware and the ROS 2 SDK.  
Run it in a **dedicated terminal** (keep it open for the entire session):

```bash
./heal_server
```

### 3 — Source the workspace (every new terminal)

```bash
cd ~/cobot_ros2_ws
source install/setup.bash
```

### 4 — Launch the control manager (Terminal 1)

```bash
ros2 launch addverb_cobot_control cobot_control.launch.py
```

---

## VLA / RL Training Data Collection

This section describes the full end-to-end workflow for collecting robot + camera demonstration data that is compatible with the **addverb_vla_datacollection** pipeline (Parquet + JPEG format).

All data-collection scripts live in:

```
py_example/py_example/isaacsim_teleop/
├── hw_with_cam.py               # Data logger  (robot state + RealSense cameras)
├── keyboard_teleop_only_hw.py   # Keyboard teleoperation controller
├── sim_data_logger.py           # Core Parquet/JPEG logging engine (used internally)
├── metadata_utils_parquet.py    # Dataset metadata helpers
├── validate_data.py             # Post-collection data validation
├── view_parquet.py              # Quick viewer for collected Parquet files
├── extract_state_action.py      # Utility to extract state/action arrays
└── requirements.txt             # All Python environment dependencies
```

### Data Format

Each session produces a **Parquet** file plus **JPEG** frames with the following schema:

| Field | Source | Description |
|---|---|---|
| `observation.state.joint_positions` | `/joint_states` | 7-DoF arm + gripper positions (rad) |
| `action.robot` | `/velocity_controller/commands` | 7-DoF velocity commands (rad/s) |
| `observation.images.ego` | RealSense (ego cam) | 640×480 RGB image |
| `observation.images.external` | RealSense (ext cam) | 640×480 RGB image |

---

### Step 1 — Start Heal Server & Connect via ROS 2 Stack

You need **four terminals** (all sourced with the workspace):

```bash
# Run this in every new terminal before any ros2 command
cd ~/cobot_ros2_ws && source install/setup.bash
```

#### Terminal A — Heal Server (hardware bridge)

```bash
./heal_server
```

> Keep this terminal open for the entire data-collection session. The Heal Server must remain running.

#### Terminal B — ROS 2 Control Manager

```bash
ros2 launch addverb_cobot_control cobot_control.launch.py
```

Wait until you see `Controller Manager is ready` before continuing.

#### Terminal C — Activate the Velocity Controller

For teleoperation-based data collection the `velocity_controller` must be active.

**Check which controller is currently active:**

```bash
ros2 control list_controllers
```

**Activate `velocity_controller`** (deactivate any other active command controller first):

```bash
ros2 service call /controller_manager/switch_controller \
controller_manager_msgs/srv/SwitchController "{
  activate_controllers: [\"velocity_controller\"],
  deactivate_controllers: [\"\"],
  start_controllers: [],
  stop_controllers: [],
  strictness: 1,
  start_asap: false,
  activate_asap: false,
  timeout: {sec: 0, nanosec: 0}
}"
```

If another controller (e.g. `ptp_joint_controller`) was already active, pass its name in `deactivate_controllers`:

```bash
ros2 service call /controller_manager/switch_controller \
controller_manager_msgs/srv/SwitchController "{
  activate_controllers: [\"velocity_controller\"],
  deactivate_controllers: [\"ptp_joint_controller\"],
  start_controllers: [],
  stop_controllers: [],
  strictness: 1,
  start_asap: false,
  activate_asap: false,
  timeout: {sec: 0, nanosec: 0}
}"
```

**Verify** the controller is running:

```bash
ros2 control list_controllers
# velocity_controller   active
```

> **Note**: Only one command controller may be active at a time. Always deactivate the current controller before activating a new one.

---

### Step 2 — Start Data Logger (`hw_with_cam.py`)

`hw_with_cam.py` is a self-contained data logger. It:

- Subscribes to `/joint_states` → robot observation
- Subscribes to `/velocity_controller/commands` → action commands
- Subscribes to `/gripper_action_command` → gripper state
- Streams frames from **up to 2 Intel RealSense cameras** (`external`, `ego`)
- Writes a timestamped **Parquet + JPEG** dataset at ~30 Hz

#### Install dependencies (first time only)

```bash
cd py_example/py_example/isaacsim_teleop/
pip install -r requirements.txt
```

#### Run the logger

Open a new terminal (**Terminal D**):

```bash
cd ~/cobot_ros2_ws && source install/setup.bash
cd ~/medical_robotics-cobot-ros2/py_example/py_example/isaacsim_teleop/
python3 hw_with_cam.py
```

**Expected output:**

```
[HardwareDataLogger] started → /vla_dataset/syncro5/teleop_session_<timestamp>
[HardwareDataLogger] logging — press Ctrl+C to stop and save.
```

Two live camera preview windows will open (`Camera: ego`, `Camera: external`).

#### Configuration

Edit the top of `hw_with_cam.py` to change defaults:

| Variable | Default | Description |
|---|---|---|
| `dataset_root` | `/vla_dataset/syncro5/teleop_session_<ts>` | Output directory |
| `DEFAULT_CONFIG_PATH` | path to `data_logger_config.yaml` | Dataset schema config |
| `fps` | `30.0` | Logging frequency (Hz) |
| `TOPIC_TIMEOUT_S` | `2.0` | Seconds before watchdog raises an error |

> `DEFAULT_CONFIG_PATH` and `data_logger_config.yaml` come from the separate **`syncro_isaac_sim`** repo, not this folder — see [External Dependency — `syncro_isaac_sim`](#external-dependency--syncro_isaac_sim-separate-repository) before your first run.

#### Camera notes

- The logger auto-discovers all connected RealSense devices.
- It expects **2 cameras** (named `external` and `ego` in order of enumeration).
- A warning is printed if fewer than 2 cameras are found; logging continues with available cameras.

#### Stopping the logger

- Press **`q`** in one of the camera preview windows, **or**
- Press **`Ctrl+C`** in Terminal D.

Both will trigger a clean shutdown: Parquet is finalized, pipelines are released, and ROS node is destroyed.

---

### Step 3 — Control the Robot with Keyboard Teleop (`keyboard_teleop_only_hw.py`)

`keyboard_teleop_only_hw.py` publishes velocity commands to `/velocity_controller/commands` at **50 Hz**. It also controls the gripper via the `/gripper_controller/command` service.

#### Run the teleop node

Open a new terminal (**Terminal E**):

```bash
cd ~/cobot_ros2_ws && source install/setup.bash
cd ~/medical_robotics-cobot-ros2/py_example/py_example/isaacsim_teleop/
python3 keyboard_teleop_only_hw.py
```

**Expected output:**

```
--- KEYBOARD CONTROL ACTIVE ---
q w e r t y → +0.05 rad/s
a s d f g h → -0.05 rad/s
u → open gripper | j → close gripper
Ctrl+C to exit
-------------------------------
Teleop ready. Use keyboard to command velocities.
```

#### Keyboard Mapping

| Key | Joint | Action |
|---|---|---|
| `q` | Joint 1 | +velocity |
| `w` | Joint 2 | +velocity |
| `e` | Joint 3 | +velocity |
| `r` | Joint 4 | +velocity |
| `t` | Joint 5 | +velocity |
| `y` | Joint 6 | +velocity |
| `a` | Joint 1 | −velocity |
| `s` | Joint 2 | −velocity |
| `d` | Joint 3 | −velocity |
| `f` | Joint 4 | −velocity |
| `g` | Joint 5 | −velocity |
| `h` | Joint 6 | −velocity |
| `u` | Gripper | Open (position = 0.0) |
| `j` | Gripper | Close (position = 1.0) |
| `Ctrl+C` | — | Exit |

> **Velocity ramping**: Velocities ramp smoothly at 0.4 rad/s² to avoid abrupt joint movement. Releasing a key ramps velocity back to zero.

#### Performing a demonstration

1. With the data logger running (**Step 2**) and teleop active (**Step 3**), move the robot through the desired task using the keyboard.
2. Camera frames and joint data are logged automatically at 30 Hz.
3. When the task is complete, stop the teleop with `Ctrl+C`, then stop the logger.
4. The dataset is saved to the `dataset_root` directory configured in `hw_with_cam.py`.

#### ROS 2 Topics used by the teleop node

| Topic | Message Type | Direction |
|---|---|---|
| `/velocity_controller/commands` | `std_msgs/Float64MultiArray` | Published (6 joint velocities) |
| `/gripper_action_command` | `std_msgs/Float32` | Published (gripper state for logging) |
| `/joint_states` | `sensor_msgs/JointState` | Subscribed (robot feedback) |
| `/gripper_controller/command` | `addverb_cobot_msgs/srv/Gripper` | Service call |

---

### Complete Terminal Layout for Data Collection

| Terminal | Command | Purpose |
|---|---|---|
| **A** | `./heal_server` | Hardware bridge (keep running) |
| **B** | `ros2 launch addverb_cobot_control cobot_control.launch.py` | ROS 2 control manager |
| **C** | `ros2 service call ... switch_controller ...` | Activate velocity controller (one-time) |
| **D** | `python3 hw_with_cam.py` | Data logger (robot + cameras) |
| **E** | `python3 keyboard_teleop_only_hw.py` | Keyboard teleoperation |

---

## Available Controllers

| # | Controller Name | Description |
|---|---|---|
| 1 | `velocity_controller` | Joint velocity commands — **used for data collection** |
| 2 | `effort_controller` | Joint torque/effort commands |
| 3 | `gravity_comp_effort_controller` | Gravity-compensated torque control |
| 4 | `free_drive_controller` | Backdrivable gravity compensation (hand-guiding) |
| 5 | `recorder_controller` | Record and replay joint trajectories |
| 6 | `ptp_joint_controller` | Point-to-point joint-space motion |
| 7 | `ptp_tcp_controller` | Point-to-point Cartesian motion |
| 8 | `joint_jogging_controller` | Incremental joint jogging |
| 9 | `cartesian_jogging_controller` | Incremental Cartesian jogging |
| 10 | `joint_impedance_controller` | Joint-space impedance control |
| 11 | `cartesian_impedance_controller` | Cartesian-space impedance control |
| 12 | `gripper_controller` | Gripper open/close |

---

## Controller Management

### List active controllers

```bash
ros2 control list_controllers
```

### Switch controllers (general template)

```bash
ros2 service call /controller_manager/switch_controller \
controller_manager_msgs/srv/SwitchController "{
  activate_controllers: [\"<desired_controller>\"],
  deactivate_controllers: [\"<current_active_controller>\"],
  start_controllers: [],
  stop_controllers: [],
  strictness: 1,
  start_asap: false,
  activate_asap: false,
  timeout: {sec: 0, nanosec: 0}
}"
```

### Example: switch from velocity → ptp_joint

```bash
ros2 service call /controller_manager/switch_controller \
controller_manager_msgs/srv/SwitchController "{
  activate_controllers: [\"ptp_joint_controller\"],
  deactivate_controllers: [\"velocity_controller\"],
  start_controllers: [],
  stop_controllers: [],
  strictness: 1,
  start_asap: false,
  activate_asap: false,
  timeout: {sec: 0, nanosec: 0}
}"
```

---

## ROS 2 Services Reference

### Gripper

```bash
# Open gripper
ros2 service call /gripper_controller/command addverb_cobot_msgs/srv/Gripper \
  "{position: 1.0, grasp_force: 100.0}"

# Close gripper
ros2 service call /gripper_controller/command addverb_cobot_msgs/srv/Gripper \
  "{position: 0.0, grasp_force: 0.0}"
```

### Error Recovery

If the robot becomes unresponsive after a fault:

```bash
ros2 service call /cobot_services/error_recovery_srv std_srvs/srv/Trigger {}
```

### Safe Shutdown

```bash
ros2 service call /cobot_services/shutdown_srv std_srvs/srv/Trigger {}
```

---

## Dataset compatibility with `syncro_isaac_sim` (training)

Hardware datasets collected here are written in the **exact same LeRobot/GROOT schema** as the simulation datasets produced by the [`syncro_isaac_sim`](https://github.com/HumanoidAddverb/syncro_isaac_sim.git) repo, so real + sim episodes can be mixed and trained together.

This is guaranteed by using the **same logging engine and dataset config as the sim**. Those files are now **vendored into this folder** (identical to the upstream `syncro_isaac_sim` versions):

```
sim_data_logger.py          # canonical Parquet/JPEG engine (from syncro_isaac_sim)
metadata_utils_parquet.py   # meta/info.json + tasks.parquet writer (from syncro_isaac_sim)
data_logger_config.yaml     # dataset schema config (from syncro_isaac_sim)
```

`hw_with_cam.py` / `hw_teleop_integrated.py` resolve the engine + config like this:

```python
# Prefer a sibling checkout of syncro_isaac_sim if present, else use the vendored copy:
_SIBLING_SIM_ROOT = <this_folder>/../../../../syncro_isaac_sim
_SIM_LOGGER_ROOT  = _SIBLING_SIM_ROOT if isdir(_SIBLING_SIM_ROOT) else <this_folder>
DEFAULT_CONFIG_PATH = _SIM_LOGGER_ROOT/data_logger_config.yaml
```

So the loggers run **stand-alone** here (no external clone needed). If you *do* clone the sim repo as a sibling, it automatically becomes the single source of truth:

```bash
git clone https://github.com/HumanoidAddverb/syncro_isaac_sim.git \
    ~/cobot_ros2_ws/src/Syncro_5/syncro_isaac_sim
```

> Keep the three vendored files **in sync with `syncro_isaac_sim`**. If the sim pipeline's schema changes, re-copy them (or clone the sibling repo) — otherwise real and sim datasets will drift apart and can no longer be trained together. The pre-existing stale copies were backed up to `.stale_local_backup/`.

### Produced schema (verified)

Every episode writes:

| Parquet column | dtype | Notes |
|---|---|---|
| `index`, `episode_index`, `frame_index`, `task_index` | int64 | dataset-wide + per-episode indices |
| `timestamp` | float32 | seconds |
| `observation.state.joint_positions` | float32 vector | `[j1..j6, gripper]` |
| `action.robot` | float32 vector | `[vel_j1..vel_j6, gripper]` |
| `observation.images.ego`, `observation.images.external` | string | relative JPEG paths |

plus `meta/info.json` (`robot_type: syncro5`, `codebase_version: v3.0`, `fps: 30`), `meta/tasks.parquet`, and `meta/episodes/…`.

> **Schema note:** `dof.joint_dim` / `dof.action_dim` are set to **`7`** so `info.json` declares `observation.state.joint_positions` and `action.robot` as shape **[7]**, matching the actual `[j1..j6, gripper]` data. (Earlier sim configs used `9`, which mismatched the 7-wide data; this has been corrected on both the hardware and `syncro_isaac_sim` sides. Any sim datasets collected before that fix will still declare [9] in their `info.json` — regenerate their metadata if a strict loader rejects them.)

### Which scripts need the logging engine

| Script | Uses engine/config? |
|---|---|
| `hw_with_cam.py`, `hw_teleop_integrated.py` | **Yes** — write datasets |
| `keyboard_teleop_only_hw.py` | No — pure teleop |
| `validate_data.py`, `view_parquet.py`, `extract_state_action.py` | No — post-processing only |

---

## Environment Requirements

All Python dependencies for the data-collection scripts are listed in:

```
py_example/py_example/isaacsim_teleop/requirements.txt
```

Install all dependencies:

```bash
cd py_example/py_example/isaacsim_teleop/
pip install -r requirements.txt
```

> **Note**: ROS 2 packages (e.g. `rclpy`, `sensor-msgs`) are typically installed system-wide via `apt` or the ROS 2 workspace. The `requirements.txt` lists all packages for reference and for isolated environment setup.

---

*For detailed controller examples and ROS 2 CLI usage, refer to [control.md](control.md).*  
*For workspace setup and Docker instructions, refer to [Setup.md](Setup.md).*
