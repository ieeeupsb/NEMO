# How to run nemo_simulator in Ubuntu 22.04 with ros2

First you will need to have the Ubuntu OS in your computer or WSL in Windows, then proceed with the installation of ros2.

Ubuntu 22.04 image:
https://releases.ubuntu.com/jammy/

Installation guide for ros2 Humble:
https://docs.ros.org/en/humble/Installation.html

For reference:
`sudo apt install ros-humble-desktop-full`
will install all of ros2 and other useful applications.

### Creating a ros2 workspace

```bash
mkdir nemo_ws/
mkdir nemo_ws/src
cd nemo_ws/src
```
You can now clone this package to the workspace with

```bash
git clone git@github.com:ieeeupsb/nemo_simulator.git
```
go to the `nemo_ws` folder with `cd ../` (assuming you are in `nemo_ws/src`) and build the ros2 packages:

```bash
colcon build
```

# How to run nemo_simulator with a Docker Container
In the future this might be a possibility to make the simulator more accessible.
