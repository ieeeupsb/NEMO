from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command


def generate_launch_description():
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": Command(["xacro ", "nemo/models/robot/model.xacro"])}
        ],
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        arguments=["nemo/models/robot/model.sdf"],
        parameters=[
            {"use_sim_time": True},
        ],
    )

    # Robot Localization
    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            "nemo/config/ekf.yaml",
            {"use_sim_time": True},
        ],
        remappings=[
            ("/odometry/filtered", "/odom"),
        ],
    )

    # Rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", "nemo/rviz/urdf_config.rviz"],
    )

    # ROS Gazebo Bridge
    ros_gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
            "/model/robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/model/robot/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
            "/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan",
            "/lidar@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked",
            "/camera@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            "/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V",
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
            "gz_args": "nemo/worlds/factory.world",
        }.items(),
    )

    return LaunchDescription(
        [
            joint_state_publisher_node,
            robot_state_publisher_node,
            robot_localization_node,
            rviz_node,
            ros_gz_bridge_node,
            ign_gazebo,
        ]
    )
