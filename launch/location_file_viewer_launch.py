from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        # Launch argument for location file
        DeclareLaunchArgument(
            'location_file',
            default_value='/home/sobits/colcon_ws/src/sobit_navigation_stack/location/location.yaml', #ここのパスを適宜変えてください
            description='Path to the location file'
        ),

        # Node to launch RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', '/home/sobits/colcon_ws/src/kachaka-api/ros2/demos/kachaka_nav2_bringup/rviz/kachaka-nav.rviz'],
            output='screen'
        ),

        # Node to launch location_file_viewer
        Node(
            package='sobit_navigation_stack',
            executable='location_file_viewer',
            name='location_file_viewer',
            namespace='location_file_viewer',
            parameters=['location_file'],
            output='screen',
            prefix= 'xterm -e'
        )
    ])