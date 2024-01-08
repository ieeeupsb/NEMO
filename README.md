# NEMO 🐠

NEMO is a robot designed to compete in [Robot@Factory 4.0](https://www.festivalnacionalrobotica.pt/2023/en/robotfactory-4-0-en/) challenge

## Build Dockerfile

```sh
docker build -t ros2:ubuntu-humble-desktop-full .
```

## Run Dockerfile

```sh
./docker_run.sh
```

## Dependencies

Inside the root directory, run the following commands:

```sh
sudo apt install ros-humble-desktop-full python3 python3-pip
```

## Syntax highlighting for VSCode

Inside the root directory, run the following command:

```sh
colcon --log-base log_ build --build-base build_  --install-base install_  --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

<!-- Also make sure the serial port `/dev/tty1` is available on your system. -->

# Sourcing ROS 2:

Run the following command:

```sh
source /opt/ros/humble/local_setup.bash
```

## Building

Inside the root directory, run the following command:

```sh
colcon build
```

## Sourcing Setup file

Inside the root directory, run the following command:

```sh
source install/setup.bash
```

## Running the program

### Simulation

```sh
ros2 launch nemo simulation.launch.py
```

### Navigation

```sh
ros2 launch nemo navigation.launch.py
```

## Testing

To test the robot's movement, there are a few messages you can publish:

### Altering the robot's velocity

```sh
ros2 topic pub /model/robot/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 3.0, y: 0.0, z: 0.0}}"
```

In this message, x represents the new linear velocity while y represents the new angular velocity of the robot

## Related work

- https://github.com/rfzeg/rtab_dumpster
- https://github.com/plusk01/aruco_localization
- https://medium.com/@geetkal67/how-to-subscribe-to-ignition-gazebo-topics-using-ros2-8bcff7a0242e
- https://medium.com/@junbs95/code-completion-and-debugging-for-ros2-in-vscode-a4ede900d979
- https://answers.ros.org/question/403966/how-to-initialize-image_transport-using-rclcpp/
- https://github.com/ros-perception/image_transport_tutorials
- https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco/tree/main
- https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html
- https://automaticaddison.com/sensor-fusion-using-the-robot-localization-package-ros-2/
- https://automaticaddison.com/navigation-and-slam-using-the-ros-2-navigation-stack/
- https://navigation.ros.org/setup_guides/transformation/setup_transforms.html

```sh
xacro nemo/models/robot/model.xacro > nemo/models/robot/model.sdf
```

```sh
ros2 topic pub /navigate_to_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 5.0, y: -2.0, z: 0.0}, orientation: {w: 1.0}}}"
```

```sh
ros2 run tf2_tools view_frames -o frames
```