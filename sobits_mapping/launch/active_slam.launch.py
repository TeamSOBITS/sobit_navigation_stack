import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def declare_param_value(context, *args, **kwargs):
    robot_name_value = LaunchConfiguration('robot_name').perform(context)
    if   ("pro" in robot_name_value):
        robot_base_frame = robot_name_value + "/base_footprint"
        vel_topic_name = "/" + robot_name_value + "/cmd_vel"
    elif ("edu" in robot_name_value):
        robot_base_frame = robot_name_value + "/base_footprint"
        vel_topic_name = "/" + robot_name_value + "/commands/velocity"
    elif ("mini" in robot_name_value):
        robot_base_frame = robot_name_value + "/base_footprint"
        vel_topic_name = "/" + robot_name_value + "/commands/velocity"
    elif ("light" in robot_name_value):
        robot_base_frame = robot_name_value + "/base_footprint"
        vel_topic_name = "/" + robot_name_value + "/cmd_vel"
    elif ("hsr" in robot_name_value):
        robot_base_frame = "base_footprint"
        vel_topic_name = "/hsrb/command_velocity"
    else:
        robot_base_frame = ""  ## CUSTOM FRAME
        vel_topic_name = ""    ## CUSTOM TOPIC

    return [
        DeclareLaunchArgument('robot_base_frame', default_value=robot_base_frame, description='ROBOT base frame name.'),
        DeclareLaunchArgument('velocity_topic_name', default_value=vel_topic_name, description='Velocity Topic Name.'),
    ]


def generate_launch_description():
    # Get the launch directory
    navigation_dir = get_package_share_directory('sobits_navigation')
    mapping_dir = get_package_share_directory('sobits_mapping')

    explore_config = os.path.join(get_package_share_directory("explore_lite"), "config", "params.yaml")

    robot_name = LaunchConfiguration('robot_name')
    velocity_topic_name = LaunchConfiguration('velocity_topic_name')
    save_map_command = LaunchConfiguration('save_map_command')
    robot_base_frame = LaunchConfiguration('robot_base_frame')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        # default_value="sobit_pro",
        # default_value="sobit_edu",
        # default_value="sobit_mini",
        # default_value="sobit_light",
        default_value="hsr_sim",
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
                            'velocity_topic_name': velocity_topic_name,
                            'use_rviz': 'False'
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
        arguments=['-d', os.path.join(navigation_dir, 'rviz', 'sobits_navigation.rviz')]
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