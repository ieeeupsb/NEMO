from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, PythonExpression

import os


def generate_launch_description():
    # ROS Gazebo Bridge
    ros_gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            "/model/nemo/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/model/nemo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            "/camera@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
        ],
    )

    # Robot Localization
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            "config/ekf.yaml",
            {"use_sim_time": True},
        ],
    )

    # Ignition Gazebo
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_ign_gazebo"),
                    "launch",
                    "ign_gazebo.launch.py",
                ]
            ),
        ),
        launch_arguments={
            "gz_args": "data/worlds/factory.world",
        }.items(),
    )

    return LaunchDescription(
        [
            ros_gz_bridge_node,
            robot_localization_node,
            ign_gazebo,
        ]
    )
