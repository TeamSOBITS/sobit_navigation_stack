import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def declare_param_value(context, *args, **kwargs):
    robot_name_value = LaunchConfiguration('robot_name').perform(context)
    if   ("home" in robot_name_value):
        robot_base_frame = robot_name_value + "/base_footprint"
    elif ("pro" in robot_name_value):
        robot_base_frame = robot_name_value + "/base_footprint"
    elif ("edu" in robot_name_value):
        robot_base_frame = robot_name_value + "/base_footprint"
    elif ("mini" in robot_name_value):
        robot_base_frame = robot_name_value + "/base_footprint"
    elif ("light" in robot_name_value):
        robot_base_frame = robot_name_value + "/base_footprint"
    elif ("hsrb" in robot_name_value):
        robot_base_frame = "base_footprint"
    elif ("hsr" in robot_name_value):
        robot_base_frame = "base_footprint"
    else:
        robot_base_frame = ""  ## CUSTOM FRAME

    return [
        DeclareLaunchArgument('robot_base_frame', default_value=robot_base_frame, description='ROBOT base frame name.'),
    ]


def generate_launch_description():
    # Get the launch directory
    navigation_dir = get_package_share_directory('sobits_nav')
    mapping_dir = get_package_share_directory('sobits_slam')

    explore_config = os.path.join(get_package_share_directory("explore_lite"), "config", "params.yaml")

    robot_name = LaunchConfiguration('robot_name')
    save_map_command = LaunchConfiguration('save_map_command')
    robot_base_frame = LaunchConfiguration('robot_base_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value="sobit_home",
        # default_value="sobit_pro",
        # default_value="sobit_edu",
        # default_value="sobit_mini",
        # default_value="sobit_light",
        # default_value="hsr_sim",
        # default_value="hsrb_robot",
        description='choice your used robot name')

    declare_save_map_command_cmd = DeclareLaunchArgument(
        'save_map_command', default_value='true',
        description='command map saver select')

    use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_dir, "launch", "nav2.launch.py")),
        launch_arguments={
                            'slam': "True",
                            'location_file_path': "",
                            'robot_name': robot_name,
                            'use_rviz': 'False',
                            'use_flex_nav': 'False',
                        }.items())

    mapping_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(mapping_dir, "launch", "gmapping.launch.py")),
        launch_arguments={
                            'robot_name': robot_name,
                            'save_map_command': save_map_command,
                            'rviz_viewer': 'False'
                        }.items())

    explore_node_cmd = Node(
        package="explore_lite",
        name="explore_node",
        executable="explore",
        parameters=[explore_config, 
                    {
                        "robot_base_frame": robot_base_frame,
                        "return_to_init": "False",
                        "use_sim_time": use_sim_time,
                    }],
        output="screen",
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(navigation_dir, 'rviz', 'sobits_nav.rviz')]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_save_map_command_cmd)
    ld.add_action(use_sim_time_cmd)
    ld.add_action(OpaqueFunction(function=declare_param_value))
    ld.add_action(nav2_cmd)
    ld.add_action(mapping_cmd)
    ld.add_action(explore_node_cmd)
    ld.add_action(rviz_cmd)

    return ld