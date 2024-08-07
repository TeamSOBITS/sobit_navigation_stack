from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    location_file_path = '/home/sobits/colcon_ws/src/sobit_navigation_stack/location/location.yaml'

    return LaunchDescription([

        # Node to launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/sobits/colcon_ws/src/sobit_navigation_stack/rviz/create_location.rviz'],
            output='screen'
        ),

        # Node to launch location_file_viewer
        Node(
            package='sobit_navigation_stack',
            executable='location_file_viewer',
            name='location_file_viewer',
            namespace='location_file_viewer',
            # ここのパスを適宜変えてください
            parameters=[{'location_file_path': location_file_path}],
            output='screen',
            prefix= 'xterm -e'
        )
    ])