# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    OpaqueFunction,
)

from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.actions import PushROSNamespace
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():

    ############################## Customizable parameters ##############################

    # Select of Robot Name
    robot_name_val = "sobit_home"
    # "sobit_home" "sobit_pro" "sobit_edu" "sobit_mini" "sobit_light" "hsrb_robot" "hsr_sim" #

    # Starting Point on the Map
    start_x   = 0.0
    start_y   = 0.0
    start_yaw = 0.0

    # Map File Path
    map_file = os.path.join(get_package_share_directory('sobits_slam'), 'map', 'map_example.yaml')

    # Location File Path
    location_file = os.path.join(get_package_share_directory('sobits_slam'), 'location', 'location_example.yaml')

    # Use Gazebo
    use_gazebo = False

    # Customize of Costmaps
    cost_map = "scan"  # "scan rgbd out_color objects" (space-separated)
    # cost_map = "scan rgbd"    # TODO: "out_color" "objects"

    # Keepout Filter Map Config
    use_keepoutmap = False
    keepout_map_file = os.path.join(get_package_share_directory('sobits_slam'), 'map', 'map_example_keepout_mask.yaml')

    # Pan-Tilt Movement Config
    use_pantilt_move = False

    #####################################################################################



    # Create the launch configuration variables
    robot_name = LaunchConfiguration('robot_name')  # SOBITS Customize
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    keepout_map_yaml_file = LaunchConfiguration('keepout_map')  # SOBITS Customize
    use_keepout_map = LaunchConfiguration('use_keepout_map')  # SOBITS Customize
    location_yaml_file = LaunchConfiguration('location')  # SOBITS Customize
    use_rviz = LaunchConfiguration('use_rviz')  # SOBITS Customize
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    slamtool_param_file = PathJoinSubstitution(
        [FindPackageShare('sobits_slam'), 'param', robot_name, 'slamtool_config.yaml']
    )
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    use_localization = LaunchConfiguration('use_localization')
    use_flex_nav = LaunchConfiguration('use_flex_nav')  # SOBITS Customize
    initial_x = LaunchConfiguration('initial_x')  # SOBITS Customize
    initial_y = LaunchConfiguration('initial_y')  # SOBITS Customize
    initial_yaw = LaunchConfiguration('initial_yaw')  # SOBITS Customize
    velocity_topic_name = LaunchConfiguration('velocity_topic_name')  # SOBITS Customize
    custom_costmap_layer = LaunchConfiguration('custom_costmap_layer')  # SOBITS Customize

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # Only it applys when `use_namespace` is True.
    # '<robot_namespace>' keyword shall be replaced by 'namespace' launch argument
    # in config file 'nav2_multirobot_params.yaml' as a default & example.
    # User defined config file should contain '<robot_namespace>' keyword for the replacements.
    params_file = ReplaceString(
        source_file=params_file,
        replacements={'<robot_namespace>': ('/', namespace)},
        condition=IfCondition(use_namespace),
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={'use_sim_time': use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value=robot_name_val,
        description='TODO: merge of namespace param.....'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('sobits_nav'), 'param', robot_name_val, 'navigation_config.yaml']
        ),
        description='Full path to nav2 params YAML; override from rc_doinglaundry for competition runs',
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack',
    )

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False', description='Whether run a SLAM'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_file,
        description='Full path to map yaml file to load'
    )

    declare_keepout_map_yaml_cmd = DeclareLaunchArgument(
        'keepout_map',
        default_value=keepout_map_file,
        description='Full path to map yaml file to load for keepout filtered map'
    )

    declare_use_keepout_map_cmd = DeclareLaunchArgument(
        'use_keepout_map',
        default_value=str(use_keepoutmap),
        description='Whether to use of keepout map',
    )

    declare_location_yaml_cmd = DeclareLaunchArgument(
        'location',
        default_value=location_file,
        description='Full path to location yaml file to load on the map'
    )

    declare_use_localization_cmd = DeclareLaunchArgument(
        'use_localization', default_value='True',
        description='Whether to enable localization or not'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Use Visualization for ROS2 (Rviz2)',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value=str(use_gazebo),
        description='Use simulation (Gazebo) clock if true',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        description='Whether to use composed bringup',
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    declare_use_flex_nav_cmd = DeclareLaunchArgument(
        'use_flex_nav', default_value=str(use_pantilt_move),
        description='Whether to use pan-tilt movement in the navigate'
    )

    declare_initial_x_cmd = DeclareLaunchArgument(
        'initial_x',
        default_value=str(float(start_x)),
        description='initial point x on the map')

    declare_initial_y_cmd = DeclareLaunchArgument(
        'initial_y',
        default_value=str(float(start_y)),
        description='initial point y on the map')

    declare_initial_yaw_cmd = DeclareLaunchArgument(
        'initial_yaw',
        default_value=str(float(start_yaw)),
        description='initial rotation(yaw) on the map')

    declare_custom_costmap_layer_cmd = DeclareLaunchArgument(
        'custom_costmap_layer',
        default_value=cost_map,
        description='Custom Costmap Layer for Global and Local Costmap')

    # Specify the actions
    bringup_cmd_group = GroupAction(
        [
            PushROSNamespace(condition=IfCondition(use_namespace), namespace=namespace),
            Node(
                condition=IfCondition(use_composition),
                name='nav2_container',
                package='rclcpp_components',
                executable='component_container_isolated',
                parameters=[configured_params, {'autostart': autostart, 'keepout_filter.enabled': use_keepout_map, 'voxel_layer.observation_sources': custom_costmap_layer, 'obstacle_layer.observation_sources': custom_costmap_layer,}],
                # parameters=[configured_params, {'autostart': autostart, 'keepout_filter.enabled': use_keepout_map,}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
                output='screen',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'slam_launch.py')
                ),
                condition=IfCondition(PythonExpression([slam, ' and ', use_localization])),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'use_respawn': use_respawn,
                    'params_file': slamtool_param_file,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('sobits_nav'), 'launch', 'include', 'localization.launch.py')
                ),
                condition=IfCondition(PythonExpression(['not ', slam, ' and ', use_localization])),
                launch_arguments={
                    'namespace': namespace,
                    'map': map_yaml_file,
                    'initial_x': initial_x,
                    'initial_y': initial_y,
                    'initial_yaw': initial_yaw,
                    'keepout_map': keepout_map_yaml_file,  # SOBITS Customize
                    'use_keepout_map': use_keepout_map,  # SOBITS Customize
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': params_file,
                    'use_composition': use_composition,
                    'use_respawn': use_respawn,
                    'container_name': 'nav2_container',
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('sobits_nav'), 'launch', 'include', 'navigation.launch.py')
                ),
                launch_arguments={
                    'namespace': namespace,
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'params_file': params_file,
                    'use_composition': use_composition,
                    'use_respawn': use_respawn,
                    'use_keepout_map': use_keepout_map,  # SOBITS Customize
                    'velocity_topic_name': velocity_topic_name,  # SOBITS Customize
                    'container_name': 'nav2_container',
                }.items(),
            ),
        ]
    )


    tf_broadcaster_cmd = Node(
        package='sobits_nav',
        executable='location_tf_broadcaster',
        name='location_tf_broadcaster',
        parameters=[{"location_file_path": location_yaml_file,}],
    )

    flex_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('flex_nav'), 'launch', 'flex_nav.launch.py')
        ),
        condition=IfCondition(use_flex_nav),
        launch_arguments={'robot_name': robot_name, 'use_sim_time': use_sim_time,}.items(),
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('sobits_nav'), 'rviz', 'sobits_nav.rviz')],
        condition=IfCondition(use_rviz)
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_robot_name_cmd)  # SOBITS Customize
    ld.add_action(declare_params_file_cmd)  # SOBITS Customize
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_keepout_map_yaml_cmd)  # SOBITS Customize
    ld.add_action(declare_use_keepout_map_cmd)  # SOBITS Customize
    ld.add_action(declare_location_yaml_cmd)  # SOBITS Customize
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(OpaqueFunction(function=declare_param_file))  # SOBITS Customize
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_use_localization_cmd)
    ld.add_action(declare_use_flex_nav_cmd)  # SOBITS Customize
    ld.add_action(declare_initial_x_cmd)  # SOBITS Customize
    ld.add_action(declare_initial_y_cmd)  # SOBITS Customize
    ld.add_action(declare_initial_yaw_cmd)  # SOBITS Customize
    ld.add_action(declare_custom_costmap_layer_cmd)  # SOBITS Customize


    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)
    ld.add_action(tf_broadcaster_cmd)  # SOBITS Customize
    ld.add_action(flex_nav_cmd)  # SOBITS Customize
    ld.add_action(rviz_cmd)  # SOBITS Customize

    return ld



def declare_param_file(context, *args, **kwargs):
    robot_name_value = LaunchConfiguration('robot_name').perform(context)

    if   ("home"    in robot_name_value): velocity_topic_name = "/" + robot_name_value + "/cmd_vel"
    elif ("pro"     in robot_name_value): velocity_topic_name = "/" + robot_name_value + "/cmd_vel"
    elif ("edu"     in robot_name_value): velocity_topic_name = "/" + robot_name_value + "/commands/velocity"
    elif ("mini"    in robot_name_value): velocity_topic_name = "/" + robot_name_value + "/commands/velocity"
    elif ("light"   in robot_name_value): velocity_topic_name = "/" + robot_name_value + "/manual_control/cmd_vel"
    elif ("hsrb"    in robot_name_value): velocity_topic_name = "/omni_base_controller/cmd_vel"
    elif ("hsr_sim" in robot_name_value): velocity_topic_name = "/hsrb/command_velocity"

    return [
            SetLaunchConfiguration('velocity_topic_name', velocity_topic_name),
        ]
