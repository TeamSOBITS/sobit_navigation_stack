import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        ########## Customizable parameters ##########
        DeclareLaunchArgument(
            # ロボットを動かす場合true,動かさない場合false
            'use_robot', default_value='true'
        ),
        DeclareLaunchArgument(
            # ロボットの名前を指定
            # 'robot_name', default_value='sobit_pro'
            # 'robot_name', default_value='sobit_edu'
            # 'robot_name', default_value='sobit_mini'
            # 'robot_name', default_value='sobit_light'
            # 'robot_name', default_value='hsr_sim'
            'robot_name', default_value='hsrb_robot'
        ),
        DeclareLaunchArgument(
            # mapのファイルパス
            'map', default_value=os.path.join(get_package_share_directory("sobits_slam"), 'map', 'map_example.yaml')
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='True'
        ),
        #############################################

        # Create Location File
        Node(
            package='sobits_slam',
            executable='location_setting',
            name='location_setting',
            output='screen',
            parameters=[
                {
                    'use_robot': LaunchConfiguration('use_robot'),
                    'robot_name': LaunchConfiguration('robot_name')
                }
            ]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("sobits_nav"), 'launch', 'nav2.launch.py')),
            launch_arguments={
                'map'          : LaunchConfiguration('map'),
                'robot_name'   : LaunchConfiguration('robot_name'),
                'location_file_path' : "",
                'use_rviz'     : LaunchConfiguration('use_rviz'),
            }.items(),
        ),
    ])