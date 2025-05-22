import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            RegisterEventHandler, OpaqueFunction)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import (AndSubstitution, LaunchConfiguration,
                                  NotSubstitution)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def declare_param_file(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('sobits_mapping')
    robot_name_value = LaunchConfiguration('robot_name').perform(context)
    param_file_path = os.path.join(bringup_dir, 'param', robot_name_value, 'gmapping_config.yaml')

    return [DeclareLaunchArgument(
        'slam_params_file',
        default_value=param_file_path,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )]


def generate_launch_description():
    save_map_command = LaunchConfiguration('save_map_command')
    declare_save_map_command_cmd = DeclareLaunchArgument(
        'save_map_command', default_value='true',
        description='command map saver select')

    rviz_viewer = LaunchConfiguration('rviz_viewer')
    declare_rviz_viewer_cmd = DeclareLaunchArgument(
        'rviz_viewer', default_value='true',
        description='use rviz')

    robot_name = LaunchConfiguration('robot_name')
    declare_robot_name_cmd = DeclareLaunchArgument(
        # 'robot_name', default_value='sobit_pro',
        'robot_name', default_value='sobit_edu',
        # 'robot_name', default_value='sobit_mini',
        # 'robot_name', default_value='sobit_light',
        # 'robot_name', default_value='hsr_sim',
        description='choice your used robot name')

    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the slamtoolbox. '
                    'Ignored when use_lifecycle_manager is true.')
    declare_use_lifecycle_manager = DeclareLaunchArgument(
        'use_lifecycle_manager', default_value='false',
        description='Enable bond connection during node activation')
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')
    # declare_slam_params_file_cmd = DeclareLaunchArgument(
    #     'slam_params_file',
    #     default_value=os.path.join(get_package_share_directory("sobits_mapping"), 'param', robot_name + '_gmapping_config.yaml'),
    #     description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[
          slam_params_file,
          {
            'use_lifecycle_manager': use_lifecycle_manager,
            'use_sim_time': use_sim_time
          }
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=''
    )

    configure_event = EmitEvent(
        event=ChangeState(
          lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
          transition_id=Transition.TRANSITION_CONFIGURE
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] Slamtoolbox node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                    transition_id=Transition.TRANSITION_ACTIVATE
                ))
            ]
        ),
        condition=IfCondition(AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager)))
    )


    # if (save_map_command):
    sobits_mapping = Node(
        package='sobits_mapping',
        executable='sobits_map_saver',
        output='log',
        name='sobits_map_saver',
        condition=IfCondition(save_map_command),
    )

    # if (rviz_viewer):
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', os.path.join(get_package_share_directory("sobits_navigation"), 'rviz', 'sobits_navigation.rviz')],
        condition=IfCondition(rviz_viewer),
    )

    ld = LaunchDescription()

    ld.add_action(declare_save_map_command_cmd)
    ld.add_action(declare_rviz_viewer_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_lifecycle_manager)
    ld.add_action(declare_use_sim_time_argument)
    # ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(OpaqueFunction(function=declare_param_file))
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(configure_event)
    ld.add_action(activate_event)
    # if (save_map_command):
    ld.add_action(sobits_mapping)
    # if (rviz_viewer):
    ld.add_action(rviz_node)

    return ld
