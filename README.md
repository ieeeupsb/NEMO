# NEMO 🐠

NEMO is a robot designed to compete in [Robot@Factory 4.0](https://www.festivalnacionalrobotica.pt/2023/en/robotfactory-4-0-en/) challenge

## Dependencies

Inside the root directory, run the following commands:

```sh
sudo apt install ros-humble-desktop-full python3 python3-pip
pip install -r requirements.txt
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
export IGN_GAZEBO_RESOURCE_PATH="$(pwd)/data/models:${IGN_GAZEBO_RESOURCE_PATH}"
ros2 launch launch/simulation.launch.xml
```

## Testing

To test movement messages sent to the robot, you can run the following command:
`ros2 topic pub nemo_subscriber geometry_msgs/msg/Point "{x: 0.0, y: 0.0}"` (where x is the new linear velocity and y is the new angular velocity of the robot)
