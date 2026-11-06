# COBOT | PYTHON - SETUP

Setup for the pure-Python driver (`cobot_py`). No ROS 2 and no C++ build are needed.
Based on the ROS 2 setup in the `main`/`heal` branches ([cobot_ros2/Setup.md](https://github.com/HumanoidAddverb/Syncro_5/blob/main/cobot_ros2/Setup.md)), with the ROS-only steps removed.

## Robot Software Installation
The software is provided as a docker image.

***NOTE : currently the docker image is already installed in the cobot***

## RE-LOADING DOCKER IMAGE (If required)

### 1. Download the docker image if not present
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
## Calibrate the FT sensor
```bash
./ft_calibration
```
## Start Heal Server to connect with the SDK
```bash
./heal_server
```
------------------------------------------------
## User System Setup
### Dependencies

1. Ubuntu 22.04 LTS
2. Python 3.10 or newer (`python3 --version`)
3. pip:
    ```bash
    sudo apt-get install python3-pip
    ```

`cobot_py` uses only the Python standard library. ROS 2 is **not** required.

---------------------------------------------------
### Cobot Backend Setup

The [cobot_backend/](cobot_backend/) folder in this repo holds the robot network config
(`config/ip.csv`), the backend headers (`include/`) and the prebuilt libraries (`lib/`).
`cobot_py` itself only reads `config/ip.csv`, but the folder is installed the same way as
for the ROS 2 stack so both can share one config.

1. Clone this branch:
    ```bash
    git clone -b python_examples git@github.com:HumanoidAddverb/Syncro_5.git
    cd Syncro_5
    ```

2. Copy the backend folder to the installation directory:
    ```bash
    sudo mkdir -p /opt/addverb/
    sudo cp -r cobot_backend /opt/addverb/
    ```
    > **Note:** `sudo` privileges are required to write to `/opt/addverb/`

    The result should look like:
    ```
    /opt/addverb/
    └── cobot_backend/
        ├── config/ip.csv
        ├── include/
        └── lib/
    ```

3. Set the robot IP and port in `/opt/addverb/cobot_backend/config/ip.csv`
   (see [File Content Format](#file-content-format) below). The default is:
    ```
    192.168.0.12
    15263
    ```

4. Check the backend config is readable:
    ```bash
    cat /opt/addverb/cobot_backend/config/ip.csv
    ```

---------------------------------------------------
### cobot_py Installation

1. Install the package (editable, so edits under `src/cobot/` take effect immediately):
    ```bash
    cd cobot_py
    python3 -m pip install -e ".[dev]"
    ```

2. Check the install:
    ```bash
    python3 -c "import cobot; print(cobot.__version__)"
    ```

3. Check the connection to the robot. With no arguments, `diagnose` reads the IP and port
   from `/opt/addverb/cobot_backend/config/ip.csv`:
    ```bash
    python3 -m cobot.diagnose                    # uses /opt/addverb config
    python3 -m cobot.diagnose 192.168.0.12 15263 # or pass them explicitly
    ```
    It reports which stage failed (config, TCP reach, handshake, first packet, decode).
    See *When the connection will not come up* in the [README](README.md) before retrying.

> **Note:** The robot accepts only **one connection per session**. Make sure no ROS 2 node or
> leftover C++ backend is connected to the robot while using `cobot_py`.

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

You can also override the IP/port for a single run without editing the file:
```bash
COBOT_IP=192.168.1.12 COBOT_PORT=15263 python3 examples/01_read_state.py
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
3. Check the ipv4 gateway using:
    ```bash
    ip a # or ifconfig
    ```
4. Make the IP static for that Wi-Fi network (recommended):
    ```bash
    # Replace <network_name> with nmcli connection name
    sudo nmcli connection modify <network_name> ipv4.addresses 192.168.1.12/24
    sudo nmcli connection modify <network_name> ipv4.gateway 192.168.1.1
    sudo nmcli connection modify <network_name> ipv4.dns 8.8.8.8
    sudo nmcli connection modify <network_name> ipv4.method manual

    sudo nmcli connection down <network_name>
    sudo nmcli connection up <network_name>
    ```
5. Update `ip.csv` on **both** PCs with the new IP.

__________________________________________________
### To control the robot:
Usage, examples and the walkthrough notebook are described in the [README.md](README.md).
