from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    nav2_bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "bringup_launch.py",
                ]
            ),
        ),
        launch_arguments={
            "map": "nemo/maps/factory.yaml",
            "use_sim_time": "True",
            "params_file": "nemo/config/nav2_params.yaml",
        }.items(),
    )

    return LaunchDescription(
        [
            SetParameter("use_sim_time", True),
            nav2_bringup_node,
        ]
    )
