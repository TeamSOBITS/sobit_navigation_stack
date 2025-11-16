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
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml



def generate_launch_description():
    ########## Customizable parameters ##########
    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        # default_value="sobit_pro",
        # default_value="sobit_edu",
        # default_value="sobit_mini",
        # default_value="sobit_light",
        default_value="hsr_sim",
        # default_value="hsrb_robot",
        description='choice your used robot name')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(get_package_share_directory('sobits_slam'), 'map', 'map_example.yaml'),
        description='Full path to map yaml file to load')

    declare_location_yaml_cmd = DeclareLaunchArgument(
        'location_file_path',
        default_value=os.path.join(
            get_package_share_directory('sobits_slam'), 'location', 'location_example.yaml'),
        description='Full path to location file to load')
    
    keepout_mask_yaml_file = os.path.join(
        get_package_share_directory('sobits_slam'), 'map', 'map_example_keepout_mask.yaml')
    
    declare_use_keepout_filter_cmd = DeclareLaunchArgument(
        'use_keepout_filter',
        default_value='False',
        description='Whether to use keepout filter')
    
    declare_flex_nav_cmd = DeclareLaunchArgument(
        'use_flex_nav',
        default_value="False",
        description="Whether to activate Flex nav")

    declare_initial_x_cmd = DeclareLaunchArgument(
        'initial_x',
        default_value="0.0",
        description='initial_point x')

    declare_initial_y_cmd = DeclareLaunchArgument(
        'initial_y',
        default_value="0.0",
        description='initial_point y')

    declare_initial_yaw_cmd = DeclareLaunchArgument(
        'initial_yaw',
        default_value="0.0",
        description='initial_rotation yaw')
    #############################################


    # Get the launch directory
    bringup_dir = get_package_share_directory('sobits_nav')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    slam = LaunchConfiguration('slam')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    map_yaml_file = LaunchConfiguration('map')
    robot_name = LaunchConfiguration('robot_name')
    params_file = LaunchConfiguration('params_file')
    slamtool_param_file = LaunchConfiguration('slamtool_param_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # use_location = LaunchConfiguration('use_location')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')
    location_file_path = LaunchConfiguration('location_file_path')
    use_flex_nav = LaunchConfiguration('use_flex_nav')
    velocity_topic_name = LaunchConfiguration('velocity_topic_name')
    use_keepout_filter = LaunchConfiguration('use_keepout_filter')

    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother',
                       ]

    lifecycle_nodes_with_filter = lifecycle_nodes + [
                       'keepout_filter_mask_server',
                       'costmap_filter_info_server',
                       ]

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'keepout_filter.enabled': use_keepout_filter
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    
    declare_slam_cmd = DeclareLaunchArgument(
        'slam',
        default_value='False',
        description='Whether run a SLAM')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')


    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings +
                        [('cmd_vel_smoothed', velocity_topic_name)]),
            Node(
                condition=IfCondition(use_keepout_filter),
                package='nav2_map_server',
                executable='map_server',
                name='keepout_filter_mask_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params,
                            {'use_sim_time': use_sim_time},
                            {'yaml_filename': keepout_mask_yaml_file}],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                condition=IfCondition(use_keepout_filter),
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='costmap_filter_info_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
            ),
            Node(
                condition=IfCondition(use_keepout_filter),
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes_with_filter}]),
            Node(
                condition=UnlessCondition(use_keepout_filter),
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ]
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings +
                           [('cmd_vel_smoothed', velocity_topic_name)]),
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='keepout_filter_mask_server',
                parameters=[configured_params,
                            {'use_sim_time': use_sim_time},
                            {'yaml_filename': keepout_mask_yaml_file}]
            ),
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::CostmapFilterInfoServer',
                name='costmap_filter_info_server',
                parameters=[configured_params,
                            {'use_sim_time': use_sim_time},]
            ),
            ComposableNode(
                condition=IfCondition(use_keepout_filter),
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes_with_filter}]),
            ComposableNode(
                condition=UnlessCondition(use_keepout_filter),
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
        ],
    )

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            condition=IfCondition(use_composition),
            name='nav2_container',
            package='rclcpp_components',
            executable='component_container_isolated',
            parameters=[configured_params, {'autostart': autostart}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_bringup'),
                                                        "launch",
                                                        "slam_launch.py")),
            condition=IfCondition(slam),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'use_respawn': use_respawn,
                              'params_file': slamtool_param_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir,
                                                       "launch",
                                                       "include",
                                                       "localization.launch.py")),
            condition=IfCondition(PythonExpression(['not ', slam])),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'use_composition': use_composition,
                              'use_respawn': use_respawn,
                              'container_name': 'nav2_container',
                              'initial_x': initial_x,
                              'initial_y': initial_y,
                              'initial_yaw': initial_yaw,}.items()),
    ])

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(bringup_dir, 'rviz', 'sobits_nav.rviz')],
        condition=IfCondition(use_rviz)
    )

    tf_broadcaster_cmd = Node(
        package='sobits_nav',
        executable='location_tf_broadcaster',
        name='location_tf_broadcaster',
        parameters=[{"location_file_path": location_file_path,}],
    )

    flex_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('flex_nav'), 
                                                   'launch', 
                                                   'flex_nav.launch.py')),
        launch_arguments={'robot_name': robot_name}.items(),
        condition=IfCondition(use_flex_nav),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(OpaqueFunction(function=declare_param_file))
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_initial_x_cmd)
    ld.add_action(declare_initial_y_cmd)
    ld.add_action(declare_use_keepout_filter_cmd)
    ld.add_action(declare_initial_yaw_cmd)
    ld.add_action(declare_location_yaml_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)
    ld.add_action(bringup_cmd_group)
    ld.add_action(tf_broadcaster_cmd)

    ld.add_action(declare_flex_nav_cmd)
    ld.add_action(flex_nav_launch)

    return ld



def declare_param_file(context, *args, **kwargs):
    robot_name_value = LaunchConfiguration('robot_name').perform(context)
    param_file_path = os.path.join(get_package_share_directory('sobits_nav'), 'param', robot_name_value, 'navigation_config.yaml')
    slamtool_param_file_path = os.path.join(get_package_share_directory('sobits_slam'), 'param', robot_name_value, 'slamtool_config.yaml')

    if   ("pro" in robot_name_value):
        vel_topic_name = "/" + robot_name_value + "/cmd_vel"
    elif ("edu" in robot_name_value):
        vel_topic_name = "/" + robot_name_value + "/commands/velocity"
    elif ("mini" in robot_name_value):
        vel_topic_name = "/" + robot_name_value + "/commands/velocity"
    elif ("light" in robot_name_value):
        vel_topic_name = "/" + robot_name_value + "/cmd_vel"
    elif ("hsrb" in robot_name_value):
        vel_topic_name = "/omni_base_controller/cmd_vel"
    elif ("hsr" in robot_name_value):
        vel_topic_name = "/hsrb/command_velocity"
    else:
        vel_topic_name = ""  ## CUSTOM TOPIC

    return [
            DeclareLaunchArgument('params_file', default_value=param_file_path, description='Full path to the parameters file.'),
            DeclareLaunchArgument('slamtool_param_file', default_value=slamtool_param_file_path, description='slam toolbox parameters file.'),
            DeclareLaunchArgument('velocity_topic_name', default_value=vel_topic_name, description='Velocity Topic Name.'),
        ]