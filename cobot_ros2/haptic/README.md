# Haptic Integration

## Overview

This package provides example scripts demonstrating how to integrate a haptic device with a collaborative robot (Cobot) for intuitive control and force feedback.

## Prerequisites

- Robot must have **only** `cartesian_jogging_controller` as the active controller
- Haptic ROS2 SDK installed

## Setup

### 1. Launch the Haptic Node

Open a terminal and execute the following commands:
```bash
cd ~/haptic_ros2/ # navigate to you corresponding folder
colcon build
source install/setup.bash
ros2 run haptic_pkg haptic_node
```

### 2. Launch the Haptic integration Node

In a **separate terminal**, navigate to the Cobot ROS2 SDK folder:
```bash
cd ~/cobot_ros2_ws/ 
source install/setup.bash
source ~/haptic_ros2/install/setup.bash # navigate to the haptic SDK folder and soure it
ros2 run haptic haptic_integration
```

## Usage Instructions

### Control Features

- **End Effector Jogging**: Use the haptic device to jog the robot's end effector along the robot's axes
- **Enable Button** (touch sensor): Commands are only sent when this button is engaged
- **Grab Button**: 
  - Toggle gripper state (open/close)
  - When pressed simultaneously with the enable button: Rotates the end effector along its axis using haptic commands

### Motion and Feedback Scaling

You can adjust four parameters via terminal input to customize motion behavior and haptic feedback.

**Example Command:**
```bash
0.5 0.6 0.5 2
```

**Parameter Definitions:**

| Parameter | Description |
|-----------|-------------|
| **1st value** | Velocity for haptic command execution |
| **2nd value** | Velocity along Z-axis in force-controlled mode |
| **3rd value** | Velocity along X and Y axes in force-controlled mode |
| **4th value** | Haptic feedback scaling factor |

> **Note:** Force control mode uses forces on the end effector to generate robot motion.
