 # Addverb Cobot ROS 2 Control Guide

> **Note**: Make sure you have followed the steps in [Setup.md](Setup.md) and that the robot is connected to the system before running any controllers. 

## Execution
### Run the docker container
Run the following command to resume the container
```bash
sudo ./cobot.sh run
```
Or run the following command to create a new container
```bash
sudo ./cobot.sh create
```
And if you wish to delete any container, use the following command
```bash
sudo ./cobot.sh remove
```
## Start Heal Server to connect with the ROS2 SDK
```bash
./heal_server
```
------------------------------------------------

Open two terminals (referred to as **Terminal 1** and **Terminal 2**). In each terminal, run the following setup commands:  
```bash
cd ~/cobot_ros2_ws
source install/setup.bash
```
### **Terminal 1**: Launch control manager
  ```bash 
  ros2 launch addverb_cobot_control cobot_control.launch.py
  ```
### **Terminal 2**: Run the Desired Controllers 
## Available Controllers

1. `velocity_controller`  
2. `effort_controller` 
3. `gravity_comp_effort_controller` 
4. `free_drive_controller`  
5. `recorder_controller`  
6. `ptp_joint_controller`  
7. `ptp_tcp_controller`  
8. `joint_jogging_controller`  
9. `cartesian_jogging_controller`  
10. `joint_impedance_controller`  
11. `cartesian_impedance_controller`  
12. `gripper_controller` 

## Checking Active Controllers  
To list all controllers and see which ones are active, run:  

```bash
ros2 control list_controllers
```
## Switching Controllers

*General Template*
```bash
ros2 service call /controller_manager/switch_controller \
controller_manager_msgs/srv/SwitchController "{
  activate_controllers: ["<desired_controller_name>"],
  deactivate_controllers: ["<active_controller_name>"],
  start_controllers: [],
  stop_controllers: [],
  strictness: 1,
  start_asap: false,
  activate_asap: false,
  timeout: {sec: 0, nanosec: 0}
}"
```
* Replace `<desired_controller_name>` with the controller you want to activate. Example: `velocity_controller`

* Replace `<active_controller_name>` with the controller you want to deactivate.

> **Note**: A controller must be activated before it can be used. And all other controllers that are not in use must be deactivated. 

*Examples*
1. Activate `velocity_controller`

    ```bash
    ros2 service call /controller_manager/switch_controller \
    controller_manager_msgs/srv/SwitchController "{
      activate_controllers: ["velocity_controller"],
      deactivate_controllers: [""],
      start_controllers: [],
      stop_controllers: [],
      strictness: 1,
      start_asap: false,
      activate_asap: false,
      timeout: {sec: 0, nanosec: 0}
    }"
    ```
2. Switch from `velocity_controller` → `ptp_joint_controller`

    ```bash
    ros2 service call /controller_manager/switch_controller \
    controller_manager_msgs/srv/SwitchController "{
      activate_controllers: ["ptp_joint_controller"],
      deactivate_controllers: ["velocity_controller"],
      start_controllers: [],
      stop_controllers: [],
      strictness: 1,
      start_asap: false,
      activate_asap: false,
      timeout: {sec: 0, nanosec: 0}
    }"
    ```
## Execution

Open two terminals (referred to as **Terminal 1** and **Terminal 2**). In each terminal, run the following setup commands:  
```bash
cd ~/cobot_ros2_ws
source install/setup.bash
```
### **Terminal 1**: Launch control manager
  ```bash 
  ros2 launch addverb_cobot_control cobot_control.launch.py
  ```
### **Terminal 2**: Run the Desired Controllers 
You can run controllers in two methods :

1. Pre programmed demos
2. User defined inputs
---
#### **Method 1**: Using Pre Programmed Demos

*General template*
```bash
ros2 run examples <desired demo>
```
Activate the desired controller and replace `<desired demo>` with the one you want to run.  
*(The list of available demos can be found in the provided user manual).*

- **`<desired demo>`** → executable name (e.g., `demo_ptp`).  

*Example*
```bash
ros2 run examples demo_ptp
```

#### **Method 2**: Through User Defined Inputs (CLI commands)
##### **Using Action Server**
---

*General template*
```bash
ros2 action send_goal <action server name> <message type> <message content>
```
Replace `<action server name>` with one of the following, and use the command:
1. ptp_joint_controller → `/ptp_joint_controller/follow_joint_trajectory`  
2. cartesian_impedance_controller → `/cartesian_impedance_controller/follow_joint_trajectory`  
3. joint_impedance_controller → `/joint_impedance_controller/follow_joint_trajectory`  
4. ptp_tcp_controller → `/ptp_tcp_controller/follow_cartesian_trajectory`  
5. recorder_controller → `/recorder_controller/replay_mode`  

*You can find the `<message type>` and `<message content>` for each controller from the provided user manual.*

*Example*
```bash
ros2 action send_goal /ptp_joint_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
    points: [
      {
        positions: [0.1, -0.2, 0.3, -0.4, 0.5, -0.6],
        time_from_start: {sec: 3, nanosec: 5}
      },
      {
        positions: [0.2, -0.1, 0.4, -0.3, 0.6, -0.5],
        time_from_start: {sec: 6, nanosec: 15}
      }
    ]
  },
  goal_time_tolerance: {sec: 1, nanosec: 4}
}"
```
##### **Using `ros2 topic pub`**
---

*General template*
```bash
ros2 topic pub /<desired controller>/commands <message type> <message content>
```
*You can find the `<message type>` and `<message content>` for available controllers from the provided user manual.*

*Example 1*
```bash
ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "{
    data: [0.01, 0, 0, 0, 0, 0]
}"
```
*Example 2*
```bash
ros2 topic pub /cartesian_jogging_controller/cartesian_jogging/command geometry_msgs/msg/Twist "{
    linear:  {x: 1.0, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: 0.0}
}"
  ``` 
## ROS2 services
### Recorder Service

***NOTE : Run this command ONLY ONCE to enable the recorder controller.***
```bash
sudo chmod a+rwx /opt/addverb/recorded_scripts/
```

***Start recording***
```bash
ros2 service call /recorder_controller/record_mode addverb_cobot_msgs/srv/Record "{enable: true,label: "test",rate: 50}"
```
***Stop recording***
```bash
ros2 service call /recorder_controller/record_mode addverb_cobot_msgs/srv/Record "{enable: false,label: "test",rate: 50}"
```
***Replay recording***
```bash
ros2 action send_goal /recorder_controller/replay_mode addverb_cobot_msgs/action/Replay "{label : "test", iterations: 3}"
```
### Gripper Service

***Open grippper***
```bash
ros2 service call /gripper_controller/command addverb_cobot_msgs/srv/Gripper"{position: 1.0, grasp_force: 100.0}"
```
***Close grippper***
```bash
ros2 service call /gripper_controller/command addverb_cobot_msgs/srv/Gripper"{position: 0.0, grasp_force: 0.0}"
```
### Error Recovery Service
In case of any error, and the robot becomes irresponsive. Call the error recovery service by using the following command:
```bash
ros2 service call /cobot_services/error_recovery_srv std_srvs/srv/Trigger {}
```
### Shut Down Service
Inorder to shut down the robot and take it to a safe state, use the following command:
```bash
ros2 service call /cobot_services/shutdown_srv std_srvs/srv/Trigger {}
```
