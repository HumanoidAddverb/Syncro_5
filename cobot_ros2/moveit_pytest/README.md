MoveIt Python Interface – moveit_pytest
======================================

This package provides Python-based testing utilities and example scripts
to interface with MoveIt using ROS 2.

---------------------------------------
CONTROLLER SETUP
---------------------------------------
Before running any MoveIt commands, ensure that the `ptp_joint_controller`
is activated on the robot. Otherwise, motion will be planned but not
executed on the robot.

Use the following command to switch controllers:

    ros2 service call /controller_manager/switch_controller \
    controller_manager_msgs/srv/SwitchController \
    "{ activate_controllers: ['ptp_joint_controller'], \
       deactivate_controllers: ['<YOUR_ACTIVE_CONTROLLER>'], \
       start_controllers: [], \
       stop_controllers: [], \
       strictness: 0, \
       start_asap: false, \
       activate_asap: false, \
       timeout: {sec: 0, nanosec: 0} }"

Replace `<YOUR_ACTIVE_CONTROLLER>` with the currently active controller
(e.g., a trajectory controller or another joint controller).

NOTE:
- Only one controller should be active at a time.
- Ensure that any previously active controllers are properly deactivated.

---------------------------------------
# HOW TO USE
---------------------------------------

    Package name: " heal"


---------------------------------------
# 1. Using MoveIt with RViz (Simulation)
---------------------------------------

To run MoveIt in simulation mode with RViz, launch:

    ros2 launch heal demo_launch.py

This will start:
- MoveIt
- RViz with the robot model
- A simulated joint controller

---------------------------------------
# 2. Using MoveIt with a Real Robot (No Simulation)
---------------------------------------

First, launch MoveIt without simulation:

    ros2 launch heal move_group.launch.py

To visualize the robot and planning in RViz (optional), open a new terminal
and run:

    ros2 launch heal moveit_rviz.launch.py

NOTE:
If the gripper state does not change when toggled:
- Switch to the opposite gripper state
- Plan and execute once
- Then continue with the intended task

This behavior occurs because there is no feedback from the gripper to the
controller, and the initial state is assumed.

---------------------------------------
# Switching Between Arm and Gripper Planning
---------------------------------------

In RViz, select the Move Group corresponding to the component you intend
to control (arm or gripper) before planning and execution.

---------------------------------------
# 3. Testing MoveIt Functionality
---------------------------------------

An example script is provided in the `moveit_pytest` package to test basic
planning and execution:

    ros2 run moveit_pytest plan_and_execute

This script:
- Randomly generates joint-space goal positions
- Plans motion trajectories using MoveIt
- Executes the planned motion on the robot and/or simulation (if active)
