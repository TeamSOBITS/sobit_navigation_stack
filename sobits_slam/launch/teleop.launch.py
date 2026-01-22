from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ########## Customizable parameters ##########
    velocity_topic_name_arg = DeclareLaunchArgument(
        # 'velocity_topic_name', default_value='/sobit_pro/cmd_vel', description='Velocity topic name',
        # 'velocity_topic_name', default_value='/sobit_edu/commands/velocity', description='Velocity topic name',
        # 'velocity_topic_name', default_value='/sobit_mini/commands/velocity', description='Velocity topic name',
        # 'velocity_topic_name', default_value="/sobit_light/manual_control/cmd_vel", description='Velocity topic name',
        # 'velocity_topic_name', default_value='/hsrb/command_velocity', description='Velocity topic name',
        'velocity_topic_name', default_value='/omni_base_controller/cmd_vel', description='Velocity topic name',
    )
    timeout_sec_arg = DeclareLaunchArgument(
        'timeout_sec', default_value='0.3', description='Timeout in seconds'
    )
    max_linear_arg = DeclareLaunchArgument(
        'max_linear', default_value='0.2', description='Maximum linear velocity'
    )
    max_angular_arg = DeclareLaunchArgument(
        'max_angular', default_value='0.7', description='Maximum angular velocity'
    )
    accel_linear_arg = DeclareLaunchArgument(
        'accel_linear', default_value='0.3', description='Linear acceleration'
    )
    accel_angular_arg = DeclareLaunchArgument(
        'accel_angular', default_value='0.7', description='Angular acceleration'
    )
    #############################################

    # Node configuration
    keyboard_teleop_node = Node(
        package='sobits_nav',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        prefix='xterm -font r16 -fg floralwhite -bg darkslateblue -e',
        parameters=[{
            'timeout_sec': LaunchConfiguration('timeout_sec'),
            'velocity_topic_name': LaunchConfiguration('velocity_topic_name'),
            'max_linear': LaunchConfiguration('max_linear'),
            'max_angular': LaunchConfiguration('max_angular'),
            'accel_linear': LaunchConfiguration('accel_linear'),
            'accel_angular': LaunchConfiguration('accel_angular'),
        }]
    )

    # Return LaunchDescription object
    return LaunchDescription([
        timeout_sec_arg,
        velocity_topic_name_arg,
        max_linear_arg,
        max_angular_arg,
        accel_linear_arg,
        accel_angular_arg,
        keyboard_teleop_node
    ])
