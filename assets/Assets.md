# Assets: index of important files

One page with links to the files you will most often need in the Syncro_5 repo (`main` branch). Links are relative to this page (`assets/`). ROS 2 packages live under [cobot_ros2/](../cobot_ros2/), and the prebuilt backend under [cobot_backend/](../cobot_backend/). The Heal arm has its own `heal` branch; Heal files that remain on `main` are listed under [Legacy Heal files](#legacy-heal-files).

## Docs: start here

| File | What it covers |
|---|---|
| [README.md](../README.md) | Creating the workspace, cloning (`main` for Syncro 5, `heal` for Heal), copying `cobot_backend` to `/opt/addverb/` |
| [Syncro 5 User Manual.pdf](../Syncro%205%20User%20Manual.pdf) | Vendor user manual for the Syncro 5 arm |
| [Setup.md](../cobot_ros2/Setup.md) | Robot-PC Docker image, FT-sensor calibration, user-PC dependencies, building the workspace, Wi-Fi/Ethernet network config |
| [control.md](../cobot_ros2/control.md) | Running the ROS 2 stack, available controllers, switching controllers, demos and CLI commands |
| [haptic/README.md](../cobot_ros2/haptic/README.md) | Haptic-device teleop setup |
| [moveit_pytest/README.md](../cobot_ros2/moveit_pytest/README.md) | MoveIt in RViz and on the real robot (`syncro`, `syncro_gripper`, `syncro_gripper_ft`) |
| [isaacsim_teleop/README.md](../cobot_ros2/py_examples/py_examples/isaacsim_teleop/README.md) | Hardware teleop and data collection for VLA / RL training |

## Docker (repo root)

| File | Purpose |
|---|---|
| [Dockerfile](../Dockerfile) | Ubuntu 22.04 image, workdir `/cobot_ros2_ws` |
| [syncro_ros2](../syncro_ros2) | Bash CLI that builds the image and creates, runs or removes the container (`create` / `run` / `remove`) |

## Robot description (URDF) (`addverb_cobot_description`)

| File | Purpose |
|---|---|
| [syncro_ros2_control.urdf.xacro](../cobot_ros2/addverb_cobot_description/urdf/syncro_ros2_control.urdf.xacro) | Robot + ros2_control. **Used by `cobot_control.launch.py`** |
| [syncro.urdf.xacro](../cobot_ros2/addverb_cobot_description/urdf/syncro.urdf.xacro) | Kinematics/visual model. Adds the FT sensor and gripper only if `ft_type` / `gripper_type` are non-zero in [payload_config.yaml](../cobot_ros2/addverb_cobot_control/config/payload_config.yaml) |
| [syncro.ros2_control.xacro](../cobot_ros2/addverb_cobot_description/ros2_control/syncro.ros2_control.xacro) | Syncro joints and GPIOs. **The order of the `<gpio>` entries matters** (they are bound by position) |
| [display.launch.py](../cobot_ros2/addverb_cobot_description/launch/display.launch.py) · [default.rviz](../cobot_ros2/addverb_cobot_description/rviz/default.rviz) | View `syncro.urdf.xacro` in RViz |

### Meshes ([syncro_meshes/](../cobot_ros2/addverb_cobot_description/syncro_meshes/))
- Arm: [base_link](../cobot_ros2/addverb_cobot_description/syncro_meshes/base_link.STL) · [Link_1](../cobot_ros2/addverb_cobot_description/syncro_meshes/Link_1.STL) · [Link_2](../cobot_ros2/addverb_cobot_description/syncro_meshes/Link_2.STL) · [Link_3](../cobot_ros2/addverb_cobot_description/syncro_meshes/Link_3.STL) · [Link_4](../cobot_ros2/addverb_cobot_description/syncro_meshes/Link_4.STL) · [Link_5](../cobot_ros2/addverb_cobot_description/syncro_meshes/Link_5.STL) · [Link_6](../cobot_ros2/addverb_cobot_description/syncro_meshes/Link_6.STL)
- FT sensor: [ft/FT_sensor.STL](../cobot_ros2/addverb_cobot_description/syncro_meshes/ft/FT_sensor.STL)
- Gripper ([gripper/](../cobot_ros2/addverb_cobot_description/syncro_meshes/gripper/)): `gripper_base_link`, `linear_block`, `connect_left/right`, `left/right_link_1`…`_4`, `left/right_low_pin`

## Launch & configuration (`addverb_cobot_control`)

| File | Purpose |
|---|---|
| [cobot_control.launch.py](../cobot_ros2/addverb_cobot_control/launch/cobot_control.launch.py) | Main launch file. Hard-codes `syncro_ros2_control.urdf.xacro`; also spawns `gripper_controller` and starts `gripper_action_bridge` |
| [ros2_controllers.yaml](../cobot_ros2/addverb_cobot_control/config/ros2_controllers.yaml) | Controller manager and per-controller parameters |
| [payload_config.yaml](../cobot_ros2/addverb_cobot_control/config/payload_config.yaml) | Payload mass/CoM/inertia, `gripper_type`, `ft_type` (also read by the URDF) |

## Hardware interface (`addverb_cobot_hardware`)

| File | Purpose |
|---|---|
| [cobot_hw_interface.cpp](../cobot_ros2/addverb_cobot_hardware/src/cobot_hw_interface.cpp) | `CobotHWInterface`: exporting interfaces, read/write, `perform_command_mode_switch` |
| [cobot_services.cpp](../cobot_ros2/addverb_cobot_hardware/src/cobot_services.cpp) | Error-recovery and shutdown services |
| [cobot_executor.cpp](../cobot_ros2/addverb_cobot_hardware/src/cobot_executor.cpp) · [cobot_auxiliary.cpp](../cobot_ros2/addverb_cobot_hardware/src/cobot_auxiliary.cpp) | Executor and auxiliary node |
| [cobot_hw_interface.xml](../cobot_ros2/addverb_cobot_hardware/cobot_hw_interface.xml) | Pluginlib export |

## Communication layer (`utility`)

| File | Purpose |
|---|---|
| [cobot_comm_wrapper.cpp](../cobot_ros2/utility/src/cobot_comm_wrapper.cpp) · [.h](../cobot_ros2/utility/include/utility/cobot_comm_wrapper.h) | `CobotCommWrapper`, the only code that talks to the backend |
| [robot_config_info.h](../cobot_ros2/utility/include/utility/robot_config_info.h) | Joint limits, **copied by hand from the robot side** |
| [data_validator.cpp](../cobot_ros2/utility/src/data_validator.cpp) | Local limit checks |
| [data_processor.cpp](../cobot_ros2/utility/src/data_processor.cpp) · [data_converter.cpp](../cobot_ros2/utility/src/data_converter.cpp) · [data_communicator.cpp](../cobot_ros2/utility/src/data_communicator.cpp) | Convert ros2_control buffers into wrapper structs |
| [hardware_interface_defs.cpp](../cobot_ros2/utility/src/hardware_interface_defs.cpp) · [controller_defs.h](../cobot_ros2/utility/include/utility/controller_defs.h) | Interface and controller name definitions |
| [ros_wrapper_error_codes.h](../cobot_ros2/utility/include/utility/ros_wrapper_error_codes.h) | Error codes |

## Controllers (`addverb_cobot_controllers`)

Source in [src/](../cobot_ros2/addverb_cobot_controllers/src/), headers in [include/](../cobot_ros2/addverb_cobot_controllers/include/addverb_cobot_controllers/). Registration: [addverb_cobot_controllers_plugin.xml](../cobot_ros2/addverb_cobot_controllers/addverb_cobot_controllers_plugin.xml).

| Controller | Source |
|---|---|
| Velocity | [velocity_controller.cpp](../cobot_ros2/addverb_cobot_controllers/src/velocity_controller.cpp) |
| Effort | [effort_controller.cpp](../cobot_ros2/addverb_cobot_controllers/src/effort_controller.cpp) |
| Gravity-comp effort | [gravity_comp_effort_controller.cpp](../cobot_ros2/addverb_cobot_controllers/src/gravity_comp_effort_controller.cpp) |
| Free drive | [free_drive_controller.cpp](../cobot_ros2/addverb_cobot_controllers/src/free_drive_controller.cpp) |
| Recorder / replay | [recorder_controller.cpp](../cobot_ros2/addverb_cobot_controllers/src/recorder_controller.cpp) |
| PTP joint | [ptp_joint_controller.cpp](../cobot_ros2/addverb_cobot_controllers/src/ptp_joint_controller.cpp) |
| PTP TCP | [ptp_tcp_controller.cpp](../cobot_ros2/addverb_cobot_controllers/src/ptp_tcp_controller.cpp) |
| Joint jogging | [joint_jogging_controller.cpp](../cobot_ros2/addverb_cobot_controllers/src/joint_jogging_controller.cpp) |
| Cartesian jogging | [cartesian_jogging_controller.cpp](../cobot_ros2/addverb_cobot_controllers/src/cartesian_jogging_controller.cpp) |
| Joint impedance | [joint_impedance_controller.cpp](../cobot_ros2/addverb_cobot_controllers/src/joint_impedance_controller.cpp) |
| Cartesian impedance | [cartesian_impedance_controller.cpp](../cobot_ros2/addverb_cobot_controllers/src/cartesian_impedance_controller.cpp) |
| Gripper | [gripper_controller.cpp](../cobot_ros2/addverb_cobot_controllers/src/gripper_controller.cpp) · [gripper_action_bridge.cpp](../cobot_ros2/addverb_cobot_controllers/src/gripper_action_bridge.cpp) |
| Shared helpers | [controller_utils.cpp](../cobot_ros2/addverb_cobot_controllers/src/controller_utils.cpp) · [control_default_params.h](../cobot_ros2/addverb_cobot_controllers/include/addverb_cobot_controllers/control_default_params.h) |

## Messages (`addverb_cobot_msgs`)

- Actions: [FollowCartesianTrajectory](../cobot_ros2/addverb_cobot_msgs/action/FollowCartesianTrajectory.action) · [Replay](../cobot_ros2/addverb_cobot_msgs/action/Replay.action)
- Services: [Gripper](../cobot_ros2/addverb_cobot_msgs/srv/Gripper.srv) · [Record](../cobot_ros2/addverb_cobot_msgs/srv/Record.srv) · [FlexFactor](../cobot_ros2/addverb_cobot_msgs/srv/FlexFactor.srv)
- Messages: [CartesianJoggingVelocity](../cobot_ros2/addverb_cobot_msgs/msg/CartesianJoggingVelocity.msg) · [JointJoggingVelocity](../cobot_ros2/addverb_cobot_msgs/msg/JointJoggingVelocity.msg) · [CartesianPoint](../cobot_ros2/addverb_cobot_msgs/msg/CartesianPoint.msg) · [CartesianTrajectory](../cobot_ros2/addverb_cobot_msgs/msg/CartesianTrajectory.msg) · [CartesianTrajectoryPoint](../cobot_ros2/addverb_cobot_msgs/msg/CartesianTrajectoryPoint.msg)

## Examples

### ROS 2 C++ ([examples/src/](../cobot_ros2/examples/src/))

[demo_velocity](../cobot_ros2/examples/src/demo_velocity.cpp) · [demo_effort](../cobot_ros2/examples/src/demo_effort.cpp) · [demo_gravity_comp_effort](../cobot_ros2/examples/src/demo_gravity_comp_effort.cpp) · [demo_ptp_joint](../cobot_ros2/examples/src/demo_ptp_joint.cpp) · [demo_ptp_tcp](../cobot_ros2/examples/src/demo_ptp_tcp.cpp) · [demo_joint_jogging](../cobot_ros2/examples/src/demo_joint_jogging.cpp) · [demo_cartesian_jogging](../cobot_ros2/examples/src/demo_cartesian_jogging.cpp) · [demo_joint_impedance](../cobot_ros2/examples/src/demo_joint_impedance.cpp) · [demo_cartesian_impedance](../cobot_ros2/examples/src/demo_cartesian_impedance.cpp) · [demo_recorder](../cobot_ros2/examples/src/demo_recorder.cpp) · [demo_gripper](../cobot_ros2/examples/src/demo_gripper.cpp)

Build: [CMakeLists.txt](../cobot_ros2/examples/CMakeLists.txt)

### ROS 2 Python ([py_examples/](../cobot_ros2/py_examples/py_examples/))

[demo_velocity](../cobot_ros2/py_examples/py_examples/demo_velocity.py) · [demo_effort](../cobot_ros2/py_examples/py_examples/demo_effort.py) · [demo_gravity_comp_effort](../cobot_ros2/py_examples/py_examples/demo_gravity_comp_effort.py) · [demo_ptp_joint](../cobot_ros2/py_examples/py_examples/demo_ptp_joint.py) · [demo_ptp_tcp](../cobot_ros2/py_examples/py_examples/demo_ptp_tcp.py) · [demo_joint_jogging](../cobot_ros2/py_examples/py_examples/demo_joint_jogging.py) · [demo_cartesian_jogging](../cobot_ros2/py_examples/py_examples/demo_cartesian_jogging.py) · [demo_joint_impedance](../cobot_ros2/py_examples/py_examples/demo_joint_impedance.py) · [demo_cartesian_impedance](../cobot_ros2/py_examples/py_examples/demo_cartesian_impedance.py) · [demo_gripper](../cobot_ros2/py_examples/py_examples/demo_gripper.py) · [homing](../cobot_ros2/py_examples/py_examples/homing.py)

Entry points: [setup.py](../cobot_ros2/py_examples/setup.py)

### Isaac Sim teleop / data collection ([isaacsim_teleop/](../cobot_ros2/py_examples/py_examples/isaacsim_teleop/))

| File | Purpose |
|---|---|
| [hw_teleop_integrated.py](../cobot_ros2/py_examples/py_examples/isaacsim_teleop/hw_teleop_integrated.py) · [hw_with_cam.py](../cobot_ros2/py_examples/py_examples/isaacsim_teleop/hw_with_cam.py) · [keyboard_teleop_only_hw.py](../cobot_ros2/py_examples/py_examples/isaacsim_teleop/keyboard_teleop_only_hw.py) | Teleop entry points (with/without camera, keyboard-only) |
| [syncro_sim_client.py](../cobot_ros2/py_examples/py_examples/isaacsim_teleop/syncro_sim_client.py) | Isaac Sim client |
| [sim_data_logger.py](../cobot_ros2/py_examples/py_examples/isaacsim_teleop/sim_data_logger.py) · [data_logger_config.yaml](../cobot_ros2/py_examples/py_examples/isaacsim_teleop/data_logger_config.yaml) · [metadata_utils_parquet.py](../cobot_ros2/py_examples/py_examples/isaacsim_teleop/metadata_utils_parquet.py) | Logging to Parquet |
| [extract_state_action.py](../cobot_ros2/py_examples/py_examples/isaacsim_teleop/extract_state_action.py) · [validate_data.py](../cobot_ros2/py_examples/py_examples/isaacsim_teleop/validate_data.py) · [view_parquet.py](../cobot_ros2/py_examples/py_examples/isaacsim_teleop/view_parquet.py) | Post-processing and inspection |
| [requirements.txt](../cobot_ros2/py_examples/py_examples/isaacsim_teleop/requirements.txt) | Python dependencies |

## MoveIt (`addverb_moveit_configs`)

Three config packages, one per tooling option: [syncro/](../cobot_ros2/addverb_moveit_configs/syncro/) (bare arm), [syncro_gripper/](../cobot_ros2/addverb_moveit_configs/syncro_gripper/) (with gripper), [syncro_gripper_ft/](../cobot_ros2/addverb_moveit_configs/syncro_gripper_ft/) (with FT sensor and gripper). Each has the same layout; links below are for `syncro`.

| File | Purpose |
|---|---|
| [demo.launch.py](../cobot_ros2/addverb_moveit_configs/syncro/launch/demo.launch.py) | MoveIt + RViz demo |
| [move_group.launch.py](../cobot_ros2/addverb_moveit_configs/syncro/launch/move_group.launch.py) · [moveit_rviz.launch.py](../cobot_ros2/addverb_moveit_configs/syncro/launch/moveit_rviz.launch.py) | Move group and RViz separately (real robot) |
| [robot.srdf](../cobot_ros2/addverb_moveit_configs/syncro/config/robot.srdf) | Planning groups, named poses, collision pairs |
| [robot.urdf.xacro](../cobot_ros2/addverb_moveit_configs/syncro/config/robot.urdf.xacro) · [robot.ros2_control.xacro](../cobot_ros2/addverb_moveit_configs/syncro/config/robot.ros2_control.xacro) | Robot description used by MoveIt |
| [moveit_controllers.yaml](../cobot_ros2/addverb_moveit_configs/syncro/config/moveit_controllers.yaml) · [ros2_controllers.yaml](../cobot_ros2/addverb_moveit_configs/syncro/config/ros2_controllers.yaml) | Controllers MoveIt executes through |
| [joint_limits.yaml](../cobot_ros2/addverb_moveit_configs/syncro/config/joint_limits.yaml) · [pilz_cartesian_limits.yaml](../cobot_ros2/addverb_moveit_configs/syncro/config/pilz_cartesian_limits.yaml) · [kinematics.yaml](../cobot_ros2/addverb_moveit_configs/syncro/config/kinematics.yaml) | Limits and IK solver |
| [initial_positions.yaml](../cobot_ros2/addverb_moveit_configs/syncro/config/initial_positions.yaml) | Start pose for the fake/mock hardware |

MoveIt test scripts: [plan_and_execute.py](../cobot_ros2/moveit_pytest/moveit_pytest/plan_and_execute.py) · [moveit_test.py](../cobot_ros2/moveit_pytest/moveit_pytest/moveit_test.py)

## Haptic teleop (`haptic`)

[haptic_integration.py](../cobot_ros2/haptic/haptic/haptic_integration.py) · [haptic_config.py](../cobot_ros2/haptic/haptic/haptic_config.py) · [haptic_defs.py](../cobot_ros2/haptic/haptic/haptic_defs.py) · [setup.py](../cobot_ros2/haptic/setup.py)

## Teleoperation headers (`teleoperation`)

[teleoperation_interface.h](../cobot_ros2/teleoperation/include/teleoperation_interface.h) · [teleoperation_datatypes.h](../cobot_ros2/teleoperation/include/teleoperation_datatypes.h) (headers only, no build files)

## Backend (`cobot_backend`, copied to `/opt/addverb/`)

| Path | Purpose |
|---|---|
| [config/ip.csv](../cobot_backend/config/ip.csv) | Robot IP (line 1) and port (line 2). Default `192.168.0.12` / `15263` |
| [include/](../cobot_backend/include/) | Backend headers (`api_types.h`, `error_codes.h`, `network_manager.h`, …) |
| [lib/](../cobot_backend/lib/) | Prebuilt shared libraries (`libbackend_executor.so`, `libnetwork_manager.so`, …) |

## Legacy Heal files

Still on `main` but not used by the Syncro launch path. For a Heal arm, use the `heal` branch.

- URDF: [heal_ros2_control.urdf.xacro](../cobot_ros2/addverb_cobot_description/urdf/heal_ros2_control.urdf.xacro) · [heal.urdf.xacro](../cobot_ros2/addverb_cobot_description/urdf/heal.urdf.xacro) · [heal.urdf](../cobot_ros2/addverb_cobot_description/urdf/heal.urdf)
- ros2_control: [heal.ros2_control.xacro](../cobot_ros2/addverb_cobot_description/ros2_control/heal.ros2_control.xacro) · [heal.config.xacro](../cobot_ros2/addverb_cobot_description/ros2_control/heal.config.xacro)
- Meshes: [heal_meshes/](../cobot_ros2/addverb_cobot_description/heal_meshes/) (arm links plus Robotiq 85 gripper parts in `_coarse` / `_fine` variants)

## Outside this repo (on the machines)

- `/opt/addverb/cobot_backend/config/ip.csv`: the copy actually read at runtime on the user PC. It must match `~/robot_network_config/ip.csv` on the robot PC (see [Setup.md](../cobot_ros2/Setup.md))
- `/opt/addverb/recorded_scripts/bucket_list.txt`: needed by the recorder controller (the directory must be world-writable)
