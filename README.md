# NEMO 🐠

NEMO is a robot designed to compete in [Robot@Factory 4.0](https://www.festivalnacionalrobotica.pt/2023/en/robotfactory-4-0-en/) challenge

## Dependencies

Inside the root directory, run the following commands:

```sh
sudo apt install ros-humble-desktop-full python3 python3-pip
pip install -r requirements.txt
```

## Syntax highlighting for VSCode

Inside the root directory, run the following command:

```sh
colcon build --build-base build_  --install-base install_ --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
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

### Robot

Inside the root directory, run the following command:

```sh
ros2 launch core core.launch.xml
```

### Simulation

Inside the simulation directory, run the following commands:

```sh
export IGN_GAZEBO_RESOURCE_PATH="$(pwd)/data/models"
ros2 launch launch/simulation.launch.xml
```

## Testing

To test the robot's movement, there are a few messages you can publish:

### Altering the robot's velocity

```
ros2 topic pub velocity_callback geometry_msgs/msg/Point "{x: 0.0, y: 0.0}"
```

In this message, x represents the new linear velocity while y represents the new angular velocity of the robot

### Moving to a tile

```
ros2 topic pub nemo_move_to_tile std_msgs/msg/Int32 "data: id"
```

In this message, id represents the end tile of the robot (from left to right, top to bottom, starting from 0)

## To-do

- [ ] Movement to specified coordinates
- [ ] Movement to specified tile
- [ ] Location
- [ ] Pathfinding
- [ ] Pick-up

## Related work

- https://github.com/rfzeg/rtab_dumpster
- https://github.com/plusk01/aruco_localization
- https://medium.com/@geetkal67/how-to-subscribe-to-ignition-gazebo-topics-using-ros2-8bcff7a0242e
- https://medium.com/@junbs95/code-completion-and-debugging-for-ros2-in-vscode-a4ede900d979
- https://answers.ros.org/question/403966/how-to-initialize-image_transport-using-rclcpp/
- https://github.com/ros-perception/image_transport_tutorials