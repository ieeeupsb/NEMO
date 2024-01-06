from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory

import launch_ros
import os


def generate_launch_description():
    namespace = "nemo"
    params_file = "core/config/nav2_params.yaml"
    param_substitutions = {}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    lifecycle_nodes = [
        "map_server",
        "controller_server",
        "planner_server",
        "bt_navigator",
    ]

    return LaunchDescription(
        [
            GroupAction(
                [
                    PushRosNamespace(namespace),
                    LaunchDescription(
                        [
                            SetParameter("use_sim_time", True),
                            # Map server
                            Node(
                                package="nav2_map_server",
                                name="map_server",
                                executable="map_server",
                                output="screen",
                                parameters=[
                                    configured_params,
                                    {
                                        "yaml_filename": os.path.join(
                                            get_package_share_directory("core"),
                                            "launch",
                                            "maps",
                                            "factory.yaml",
                                        )
                                    },
                                ],
                            ),
                            # Controller server
                            Node(
                                package="nav2_controller",
                                name="controller_server",
                                executable="controller_server",
                                output="screen",
                                parameters=[
                                    ParameterFile("core/config/controller.yaml"),
                                    {
                                        "FollowPath": {
                                            "plugin": "dwb_core::DWBLocalPlanner",
                                            "critics": [
                                                "RotateToGoal",
                                                "Oscillation",
                                                "BaseObstacle",
                                                "GoalAlign",
                                                "PathAlign",
                                                "PathDist",
                                                "GoalDist",
                                            ],
                                        }
                                    },
                                ],
                            ),
                            # Planner server
                            Node(
                                package="nav2_planner",
                                name="planner_server",
                                executable="planner_server",
                                output="screen",
                                parameters=[configured_params],
                            ),
                            # BT navigator
                            Node(
                                package="nav2_bt_navigator",
                                name="bt_navigator",
                                executable="bt_navigator",
                                output="screen",
                                parameters=[configured_params],
                            ),
                            # Lifecycle manager
                            Node(
                                package="nav2_lifecycle_manager",
                                name="lifecycle_manager",
                                executable="lifecycle_manager",
                                output="screen",
                                parameters=[
                                    {"autostart": True},
                                    {"use_sim_time": True},
                                    {"node_names": lifecycle_nodes},
                                ],
                            ),
                        ]
                    ),
                ]
            ),
        ]
    )
