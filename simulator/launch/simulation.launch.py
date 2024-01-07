from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, PythonExpression

import os


def generate_launch_description():
    sdf_path = "data/models/nemo/model.sdf"

    with open(sdf_path, 'r') as f:
        robot_description = f.read()

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        arguments=[sdf_path],
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

    # Rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", "rviz/urdf_config.rviz"],
    )

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
            robot_state_publisher_node,
            joint_state_publisher_node,
            # robot_localization_node,
            rviz_node,
            # ros_gz_bridge_node,
            # ign_gazebo,
        ]
    )
