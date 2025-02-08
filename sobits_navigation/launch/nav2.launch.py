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

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    # bringup_dir = get_package_share_directory('nav2_bringup')
    # launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    slam = LaunchConfiguration('slam')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    # rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_simulator = LaunchConfiguration('use_simulator')
    use_tbc = LaunchConfiguration('use_tbc')
    use_rviz = LaunchConfiguration('use_rviz')

    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_yaw = LaunchConfiguration('initial_yaw')
    location_file_path = LaunchConfiguration('location_file_path')


    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Declare the launch arguments
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

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            get_package_share_directory('sobits_mapping'), 'map', 'example.yaml'),
        description='Full path to map file to load')

    declare_location_yaml_cmd = DeclareLaunchArgument(
        'location_file_path',
        default_value=os.path.join(
            get_package_share_directory('sobits_mapping'), 'location', 'location_example.yaml'),
        description='Full path to location file to load')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(get_package_share_directory('sobits_navigation'), 'param', 'navigation_config.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Whether to use composed bringup')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    # declare_rviz_config_file_cmd = DeclareLaunchArgument(
    #     'rviz_config_file',
    #     default_value=os.path.join(
    #         bringup_dir, 'rviz', 'nav2_default_view.rviz'),
    #     description='Full path to the RVIZ config file to use')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='False',
        description='Whether to start the simulator')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_use_tbc_cmd = DeclareLaunchArgument(
        'use_tbc',
        default_value='True',
        description='')

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

    # rviz_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(launch_dir, 'rviz_launch.py')),
    #     condition=IfCondition(use_rviz),
    #     launch_arguments={'namespace': namespace,
    #                       'use_namespace': use_namespace,
    #                       'rviz_config': rviz_config_file}.items())

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory("sobits_mapping"), 'rviz', 'sobits_mapping.rviz')],
        condition=IfCondition(use_rviz)
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sobits_navigation'), 'launch', 'bringup.launch.py')),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': params_file,
                          'autostart': autostart,
                          'use_composition': use_composition,
                          'use_respawn': use_respawn}.items())

    tf_broadcaster_cmd = Node(
        package='sobits_navigation',
        executable='location_tf_broadcaster',
        name='location_tf_broadcaster',
        parameters=[
            {
                "initial_x": initial_x,
                "initial_y": initial_y,
                "initial_yaw": initial_yaw,
                "location_file_path": location_file_path,
                "initial_command": True,
                "create_location_file": False,
            }
        ],
        condition=IfCondition(use_tbc)
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_location_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)

    # ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_tbc_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_initial_x_cmd)
    ld.add_action(declare_initial_y_cmd)
    ld.add_action(declare_initial_yaw_cmd)

    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(tf_broadcaster_cmd)

    return ld