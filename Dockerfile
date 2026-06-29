FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Kolkata

# Locale 
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

#  ROS 2 Humble repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

#  ROS 2 packages 
RUN apt-get update && apt-get install -y \
    # Core ROS 2
    ros-humble-ros-base \
    ros-humble-ament-cmake \
    # Build tools
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-argcomplete \
    build-essential \
    cmake \
    git \
    # ros2_control stack
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-control-msgs \
    ros-humble-hardware-interface \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-position-controllers \
    ros-humble-velocity-controllers \
    ros-humble-effort-controllers \
    # Description / launch helpers
    ros-humble-xacro \
    ros-humble-launch-ros \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    # MoveIt 2
    ros-humble-moveit \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && rosdep update

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /cobot_ros2_ws

CMD ["bash"]