import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("sobits_slam")
    default_map = os.path.join(package_share, "map", "map_example.yaml")
    default_rviz = os.path.join(package_share, "rviz", "room_polygon_editor.rviz")
    default_config = os.path.join(package_share, "location", "room_information_example.yaml")

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=default_map,
        description="Full path to the map YAML file.",
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz,
        description="Full path to the RViz config file.",
    )
    config_arg = DeclareLaunchArgument(
        "config_path",
        default_value=default_config,
        description="Full path to the room information YAML file.",
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": LaunchConfiguration("map")}],
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{"autostart": True, "node_names": ["map_server"]}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config")],
    )

    room_polygon_gui = Node(
        package="sobits_slam",
        executable="room_polygon_setting",
        name="room_polygon_setting",
        output="screen",
        parameters=[{"config_path": LaunchConfiguration("config_path")}],
    )

    return LaunchDescription(
        [
            map_arg,
            rviz_arg,
            config_arg,
            map_server,
            lifecycle_manager,
            rviz,
            room_polygon_gui,
        ]
    )
