# How to run nemo_simulator in Ubuntu 22.04 with ros2

First you will need to have the Ubuntu OS in your computer or WSL in Windows, then proceed with the installation of ROS 2 Humble.

Ubuntu 22.04 image:
https://releases.ubuntu.com/jammy/

Installation guide for ROS 2 Humble:
https://docs.ros.org/en/humble/Installation.html

For reference, `sudo apt install ros-humble-desktop-full` will install all of ROS packages and other useful applications.

## Launch

```sh
export IGN_GAZEBO_RESOURCE_PATH="$(pwd)/data/models:${IGN_GAZEBO_RESOURCE_PATH}"
source /opt/ros/humble/local_setup.sh
ros2 launch launch/simulation.launch.xml
```
