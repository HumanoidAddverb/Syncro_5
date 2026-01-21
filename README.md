# Cobot Installation Instructions

## Installation Steps

### Step 1: Creat a ROS2 workspace
```bash
mkdir -p ~/cobot_ros2_ws/src
cd ~/cobot_ros2_ws/src/
```

### step 2: clone the repository
```bash
#for syncro_5 use the main branch
git clone https://github.com/HumanoidAddverb/cobot_ros2.git

# if you have heal, switch to heal branch
git switch heal
```

### Step 3: Copy Backend Folders

Move the backend folders to the installation directory:
```bash
sudo mkdir -p /opt/addverb/
sudo cp -r cobot_backend /opt/addverb/
```

> **Note:** `sudo` privileges are required to write to `/opt/addverb/`

### step 4: (OPTIONAL) Delete the cobot_backend folder from the cobot_ros2_ws 
```bash
rm -rf ~/cobot_ros2_ws/src/cobot_ros2/cobot_backend
``` 


## Directory Structure

After installation, your directory structure should look like:
```
/opt/addverb/
└── cobot_backend/

~/cobot_ros2_ws/
└── src/
    └── cobot_ros2/
```
## To set up ROS2 work space
* Refer to the [Setup.md](cobot_ros2/Setup.md) file inside the cobot_ros2 directory to build and configure the ROS2 workspace.

## To control the robot
* Instructions for operating the robot are provided in the [control.md](cobot_ros2/control.md) file located inside the cobot_ros2 directory.