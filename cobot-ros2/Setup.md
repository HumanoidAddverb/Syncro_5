# COBOT DUAL ARM | ROS2 - SETUP

## Robot Software Installation
The software is provided as a docker image, 

***NOTE : currently the docker image is already installed in the cobot***

## RE-LOADING DOCKER IMAGE (If required)

### 1. Download the docker image If not present
The `<docker name>.tar` file is to be downloaded on the cobot's system.

### 2. Load the docker image
Whenever you want to use a new updated docker image .tar, load it with:
```bash
docker load -i <docker name>.tar
```
--------------------------------------------------

### Run the docker container
Run the following command or use the cobot.sh script.
```bash
docker run -it --network host --privileged -v ~/robot_network_config:/robot_network_config --tty --volume /dev:/dev <image_name>:<tag_name>

```
## Start Cobot Server to connect with the ROS2 SDK
```bash
./cobot_server
```
------------------------------------------------
## User System Setup
### Dependencies

1. Ubuntu 22.04 LTS
2. ROS 2 Humble (if not installed already, please install using the instructions provided here : [ROS2 humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html))
3. Install ROS dependencies using the following command :
    ```bash
    sudo apt-get install liborocos-kdl-dev ros-humble-ros2-control ros-humble-ros2-controllers python3-colcon-common-extensions ros-humble-control-msgs ros-humble-hardware-interface ros-humble-controller-manager ros-humble-xacro ros-humble-launch-ros ros-humble-joint-state-broadcaster ros-humble-position-controllers ros-humble-velocity-controllers ros-humble-effort-controllers ros-humble-moveit
    ```
---------------------------------------------------
### ROS2 Package Installation 

1. Follow the Installation.md file to setup and install the workspace.
    ```bash
    source /opt/ros/humble/setup.bash
    ```
    Install the provided cobot ROS2 code.

3. For recording while using recorder controller:
    ```bash
    sudo mkdir -p /opt/addverb/recorded_scripts
    sudo touch /opt/addverb/recorded_scripts/bucket_list.txt
    sudo chmod a+rwx /opt/addverb/recorded_scripts/
    ```
4. Backend for cobot ROS2:
    Paste the dual_arm_backend folder such that it looks like:
    ```bash
    /opt/addverb/
            └── dual_arm_backend/
    ```
4. Build cobot_ros2_ws 
    ```bash
    cd cobot_ros2_ws
    #for dual arm use the dual_arm branch, steps mentioned below.
    colcon build --symlink-install
    source install/setup.bash   

> **Note:** For Very Low RAM System 'colcon build --symlink-install' command may crash. The command below can be used in such cases use the command below

```bash
export MAKEFLAGS="-j1"
colcon build --executor sequential
```

# To switch branch:
```bash
cd ~/cobot_ros2_ws/src/cobot_ros2/
# to check branch
git branch
# to switch branch
git switch dual_arm
```
_____________________________________________
### Switch between Wifi & ethernet connection
_______________________________________________

***CONFIG FILES TO CHANGE***
```
# On cobot PC
~/robot_network_config/ip_left.csv
~/robot_network_config/ip_right.csv


# On user PC
/opt/addverb/dual_arm_backend/ip_left.csv
/opt/addverb/dual_arm_backend/ip_right.csv


```

#### File Content Format

| Line | Meaning    |
|------|------------|
| 1    | IP Address |
| 2    | Port       |

> NOTE: Both files (Cobot PC + User PC) MUST have identical content for the same cobot.

Example:

```
192.168.0.12
15263
```
__________________________________________________
### To control the robot:
 Instructions for operating the robot are provided in the [control.md](control.md) file