# NEMO 🐠

<p align="center">
  <img width="300" src="nemo.webp" />
</p>

NEMO is a robot designed to compete in [Robot@Factory 4.0 challenge](https://www.festivalnacionalrobotica.pt/2023/en/robotfactory-4-0-en/).

This project was developed in the context of *Assignment 4* of [Intelligent Robotics](https://sigarra.up.pt/feup/en/ucurr_geral.ficha_uc_view?pv_ocorrencia_id=518841) classes at [FEUP](https://sigarra.up.pt/feup/en/web_page.Inicial).

## Group (A34_Morn_Gr_H)
 
- Eduardo Correia
- Luís Lucas 
- Mário Travassos

## Technologies

- Ubuntu 22.04
- ROS Humble Hawkbill
- Gazebo Fortress
- RViz2
- Navigation2  

## Folder structure

```sh
.
├── Dockerfile                  # Dockerfile with necessary dependencies
├── docker-run.sh               # Script to run the Docker container
├── docs                        # Documentation 
├── nemo                        # Core package
│   ├── CMakeLists.txt  
│   ├── package.xml             
│   ├── config                  # Configuration files 
│   ├── launch                  # Launch files
│   ├── maps                    # Map files
│   ├── models                  # Gazebo models
│   ├── README.md               # This file
│   ├── rviz                    # RViz configuration files
│   ├── src                     # Source code
│   └── worlds                  # Gazebo world files
└── README.md
```

## Setup

To run the project, you can either use Docker or install the dependencies on your system.

### Docker

#### Build Dockerfile

```sh
docker build -t ros2:ubuntu-humble-desktop-full .
```

#### Run Dockerfile

To run the Dockerfile, run the following script:

```sh
./docker_run.sh
```

To connect to an existing container, use:

```sh
docker exec -it <container_id> bash
```

You can get the `<container_id>` by running `docker ps`:

### Local install

Assuming you are using Ubuntu 22.04, you need to install the following dependencies:

```sh
# Installing ROS2 Humble
sudo apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-humble-desktop-full

# Installing colcon
sudo apt-get install -y python3 python3-pip && \ 
    pip install -U colcon-common-extensions

# Installing navigation packages
sudo apt install -y \
    ros-humble-robot-localization \
    ros-humble-joint-state-publisher \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup

# Installing Cyclone DDS
sudo apt install -y \
    ros-humble-rmw-cyclonedds-cpp

export RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"

# Setting up Gazebo models path
export IGN_GAZEBO_RESOURCE_PATH="$(pwd)/nemo/models"

echo "source /opt/ros/humble/local_setup.bash" >> ~/.bashrc
```

## Launching

To launch the project, you first need to build with it `colcon`, from the root directory:

```sh
colcon build
```

Then, you need to source the `setup.bash` file:

```sh
source install/setup.bash
```

Now to run the necessary nodes, you can run the following commands in separate terminals:

### Simulation

```sh   
ros2 launch nemo simulation.launch.py
```

### Navigation

```sh
ros2 launch nemo navigation.launch.py
```
