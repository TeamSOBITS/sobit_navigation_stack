from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_robot',
            # ロボットを動かす場合true,動かさない場合false
            default_value='false',
            description='Whether to use the robot'
        ),
        DeclareLaunchArgument(
            'location_folder_path',
            default_value='/home/sobits/colcon_ws/src/sobit_navigation_stack/location/',
            description='Path to the location folder'
        ),
        
        # Create Location File
        Node(
            package='sobit_navigation_stack',
            executable='create_location_file',
            namespace='/create_location_file',
            name='create_location_file',
            output='screen',
            prefix='xterm -e',
            parameters=[
                {'save_location_folder_path': LaunchConfiguration('location_folder_path')},
                {'use_robot': LaunchConfiguration('use_robot')}
            ]
        ),
        Node(
            package='sobit_navigation_stack',
            executable='display_location_marker',
            namespace='/create_location_file',
            name='display_location_marker'
        ),

        # teleop(ロボットを使う場合のみ)
        Node(
            package='sobit_navigation_stack',
            executable='teleop.py',
            name='teleop',
            output='screen',
            prefix='xterm -e',
            condition=IfCondition(LaunchConfiguration('use_robot'))
        ),
        
        # Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', '/home/sobits/colcon_ws/src/kachaka-api/ros2/demos/kachaka_nav2_bringup/rviz/kachaka-nav.rviz']
        )
    ])