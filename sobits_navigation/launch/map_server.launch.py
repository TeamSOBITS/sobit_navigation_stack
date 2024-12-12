import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sobits_navigation',
            executable='location_tf_broadcaster',
            name='location_tf_broadcaster',
            parameters=[
                {'location_file_path': os.path.join(get_package_share_directory("sobits_mapping"), 'location', 'location_file_name.yaml')}
            ]
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            parameters=[{'yaml_filename': os.path.join(get_package_share_directory("sobits_mapping"), 'map', 'example.yaml')}],
            name='map_server'
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        ),

        # Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory("sobits_mapping"), 'rviz', 'sobits_mapping.rviz')],
        )
    ])