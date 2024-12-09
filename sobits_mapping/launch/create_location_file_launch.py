import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    return LaunchDescription([

        IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sobits_mapping'), 'launch', 'map_server_launch.py')])
        ),

        # Arguments
        DeclareLaunchArgument(
            'use_robot',
            # ロボットを動かす場合true,動かさない場合false
            default_value='false',
            description='Whether to use the robot'
        ),

        # Create Location File
        Node(
            package='sobits_mapping',
            executable='location_setting',
            name='location_setting',
            output='screen',
            # prefix='xterm -e',
            parameters=[
                # {'save_location_folder_path': LaunchConfiguration('location_folder_path')},
                {'use_robot': LaunchConfiguration('use_robot')}
            ]
        ),

        # Node(
        #     package='nav2_map_server',
        #     executable='map_server',
        #     output='screen',
        #     parameters=[{'yaml_filename': os.path.join(get_package_share_directory("sobits_mapping"), 'map', 'example.yaml')}],
        #     name='map_server'
        # ),

        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager_map',
        #     output='screen',
        #     parameters=[{'use_sim_time': True},
        #                 {'autostart': True},
        #                 {'node_names': ['map_server']}]
        # ),

        Node(
            package='sobits_navigation',
            executable='location_tf_broadcaster',
            name='location_tf_broadcaster'
        ),

        # Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory("sobits_mapping"), 'rviz', 'sobits_mapping.rviz')],
        )
    ])