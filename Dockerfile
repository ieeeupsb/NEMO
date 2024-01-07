# Pull a base image
FROM ubuntu:22.04

# Update package lists
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    software-properties-common curl\
    gnupg

# Setting up the non interactivity
ENV DEBIAN_FRONTEND noninteractive

# Setup the ROS sources
RUN add-apt-repository universe && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/nul

# Installing ROS2 image
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-humble-desktop-full

# Installing colcon
RUN apt-get install -y python3 python3-pip && \ 
    pip install -U colcon-common-extensions

# Installing Nav2
RUN apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

# Installing the Ubuntu dependencies
RUN apt-get update && apt-get install -y \
    # Dev dependencies
    build-essential net-tools inetutils-ping htop nano bc wget curl \
    # Nvidia dependencies
    mesa-utils

# Setting up nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Setting up Gazebo models path
ENV IGN_GAZEBO_RESOURCE_PATH="/app/simulator/data/models"