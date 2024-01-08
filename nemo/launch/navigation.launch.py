from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    lifecycle_nodes = [
        "map_server",
        "controller_server",
        # "planner_server",
        # "bt_navigator",
    ]

    # Map server
    map_server_node = Node(
        package="nav2_map_server",
        name="map_server",
        executable="map_server",
        output="screen",
        parameters=[
            "nemo/config/map_server.yaml",
        ],
    )

    # Controller server
    controller_server_node = Node(
        package="nav2_controller",
        name="controller_server",
        executable="controller_server",
        output="screen",
        parameters=[
            "nemo/config/controller_server.yaml",
        ],
    )

    # Planner server
    planner_server_node = Node(
        package="nav2_planner",
        name="planner_server",
        executable="planner_server",
        output="screen",
        parameters=[
            "nemo/config/controller_server.yaml",
        ],
    )

    # BT Navigator
    bt_navigator_node = Node(
        package="nav2_bt_navigator",
        name="bt_navigator",
        executable="bt_navigator",
        output="screen",
        parameters=[],
    )

    # Lifecycle manager
    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        name="lifecycle_manager",
        executable="lifecycle_manager",
        output="screen",
        parameters=[
            {"autostart": True},
            {"use_sim_time": True},
            {"node_names": lifecycle_nodes},
        ],
    )

    return LaunchDescription(
        [
            SetParameter("use_sim_time", True),
            map_server_node,
            controller_server_node,
            # planner_server_node,
            # bt_navigator_node,
            lifecycle_manager,
        ]
    )
