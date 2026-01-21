# COBOT | ROS2 - SETUP

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
docker run -it --network host --privileged -v ~/robot_network_config:/robot_network_config --tty --volume /dev:/dev cobot:deploy
```
## calibrate the FT sensor
```bash
./ft_calibration
```
## Start Heal Server to connect with the ROS2 SDK
```bash
./heal_server
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
    Paste the cobot_backend folder such that it looks like:
    ```bash
    /opt/addverb/
            └── cobot_backend/
    ```
4. Build cobot_ros2_ws 
    ```bash
    cd cobot_ros2_ws
    #for syncro_5 use main branch, for heal switch to heal branch, steps mentioned below.
    colcon build --symlink-install
    source install/setup.bash   

> **Note:** For Very Low RAM System 'colcon build --symlink-install' command may crash. The command below can be used in such cases use the command below

```bash
colcon build \
--executor sequential \
--cmake-args -DCMAKE_BUILD_PARALLEL_LEVEL=1 \
--event-handlers console_direct+
```

# To switch branch:
```bash
cd ~/cobot_ros2_ws/src/cobot_ros2/
# to check branch
git branch
# to switch branch
git switch heal
```
_____________________________________________
### Switch between Wifi & ethernet connection
_______________________________________________

***CONFIG FILES TO CHANGE***
```
# On cobot PC
~/robot_network_config/ip.csv

# On user PC
/opt/addverb/cobot_backend/config/ip.csv

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

---

### Steps to connect via Wi-Fi

1. Check available Wi-Fi networks:
    ```bash
    sudo nmcli dev wifi rescan
    sudo nmcli dev wifi
    ```

2. Connect to Wi-Fi:
    ```bash
    sudo nmcli dev wifi connect "NETWORK_NAME" password "NETWORK_PASSWORD"
    ```
3. check the ipv4 gateway using:
    ```bash
    ip a # or ifconfig
    ```
3. Make the IP static for that Wi-Fi network (recommended):
    ```bash
    # Replace <network_name> with nmcli connection name
    sudo nmcli connection modify <network_name> ipv4.addresses 192.168.1.12/24
    sudo nmcli connection modify <network_name> ipv4.gateway 192.168.1.1
    sudo nmcli connection modify <network_name> ipv4.dns 8.8.8.8
    sudo nmcli connection modify <network_name> ipv4.method manual

    sudo nmcli connection down <network_name>
    sudo nmcli connection up <network_name>
    ```
__________________________________________________
### To control the robot:
 Instructions for operating the robot are provided in the [control.md](control.md) file