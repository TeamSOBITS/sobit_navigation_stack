import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            # ロボットを動かす場合true,動かさない場合false
            'use_robot', default_value='false'
        ),
        DeclareLaunchArgument(
            # ロボットの名前を指定
            # 'robot_name', default_value='sobit_pro'
            'robot_name', default_value='sobit_edu'
            # 'robot_name', default_value='sobit_mini'
            # 'robot_name', default_value='sobit_light'
        ),
        DeclareLaunchArgument(
            # mapのファイルパス
            'yaml_filename', default_value=os.path.join(get_package_share_directory("sobits_mapping"), 'map', 'example.yaml')
        ),

        # Create Location File
        Node(
            package='sobits_mapping',
            executable='location_setting',
            name='location_setting',
            output='screen',
            prefix='xterm -font r16 -fg floralwhite -bg darkslateblue -e',
            parameters=[
                {
                    'use_robot': LaunchConfiguration('use_robot'),
                    'robot_name': LaunchConfiguration('robot_name')
                }
            ]
        ),

        Node(
            package='sobits_navigation',
            executable='location_tf_broadcaster',
            name='location_tf_broadcaster',
            parameters=[
                {
                    "initial_x": 0.0,
                    "initial_y": 0.0,
                    "initial_yaw": 0.0,
                    "initial_command": LaunchConfiguration('use_robot'),
                    "create_location_file": True,
                }
            ]
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            output='screen',
            parameters=[{'yaml_filename': LaunchConfiguration('yaml_filename')}],
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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("sobits_navigation"), 'launch', 'nav2.launch.py')),
            launch_arguments={'use_tbc': 'False'}.items(),
            condition=IfCondition(LaunchConfiguration("use_robot"))
        ),

        # Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory("sobits_mapping"), 'rviz', 'sobits_mapping.rviz')],
            condition=UnlessCondition(LaunchConfiguration('use_robot'))
        )
    ])