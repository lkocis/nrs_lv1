"""Launch the 3F Modbus backend and its backend-neutral controller."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build a standalone, monitor-only-by-default gripper launch."""
    driver_share = get_package_share_directory('robotiq_3f_driver')
    state_publisher_share = get_package_share_directory(
        'robotiq_3f_state_publisher'
    )
    arguments = [
        DeclareLaunchArgument(
            'gripper_ip',
            description='IP address of the Robotiq 3F Modbus TCP device.',
        ),
        DeclareLaunchArgument('port', default_value='502'),
        DeclareLaunchArgument('unit_id', default_value='0'),
        DeclareLaunchArgument('poll_rate_hz', default_value='10.0'),
        DeclareLaunchArgument(
            'allow_automatic_release', default_value='false',
        ),
        DeclareLaunchArgument(
            'gripper_command_closed_position_m', default_value='0.0',
        ),
        DeclareLaunchArgument(
            'gripper_command_max_opening_m', default_value='0.155',
        ),
        DeclareLaunchArgument(
            'gripper_command_closed_raw', default_value='112',
        ),
        DeclareLaunchArgument(
            'gripper_command_min_effort_n', default_value='15.0',
        ),
        DeclareLaunchArgument(
            'gripper_command_max_effort_n', default_value='60.0',
        ),
        DeclareLaunchArgument(
            'gripper_command_default_effort_n', default_value='30.0',
        ),
        DeclareLaunchArgument(
            'gripper_command_speed', default_value='20',
        ),
        DeclareLaunchArgument(
            'launch_joint_state_publisher', default_value='true',
            description='Map status feedback to articulated gripper joints.',
        ),
        DeclareLaunchArgument('tf_prefix', default_value=''),
    ]

    driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(driver_share + '/launch/driver.launch.py')
        ),
        launch_arguments={
            'gripper_ip': LaunchConfiguration('gripper_ip'),
            'port': LaunchConfiguration('port'),
            'unit_id': LaunchConfiguration('unit_id'),
            'poll_rate_hz': LaunchConfiguration('poll_rate_hz'),
            'allow_automatic_release': LaunchConfiguration(
                'allow_automatic_release'
            ),
        }.items(),
    )
    controller = Node(
        package='robotiq_3f_controller',
        executable='robotiq_3f_controller_node',
        namespace='robotiq_3f',
        name='controller',
        output='screen',
        parameters=[{
            'gripper_command_closed_position_m': LaunchConfiguration(
                'gripper_command_closed_position_m'
            ),
            'gripper_command_max_opening_m': LaunchConfiguration(
                'gripper_command_max_opening_m'
            ),
            'gripper_command_closed_raw': LaunchConfiguration(
                'gripper_command_closed_raw'
            ),
            'gripper_command_min_effort_n': LaunchConfiguration(
                'gripper_command_min_effort_n'
            ),
            'gripper_command_max_effort_n': LaunchConfiguration(
                'gripper_command_max_effort_n'
            ),
            'gripper_command_default_effort_n': LaunchConfiguration(
                'gripper_command_default_effort_n'
            ),
            'gripper_command_speed': LaunchConfiguration(
                'gripper_command_speed'
            ),
        }],
    )
    joint_states = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(state_publisher_share + '/launch/joint_states.launch.py')
        ),
        condition=IfCondition(
            LaunchConfiguration('launch_joint_state_publisher')
        ),
        launch_arguments={
            'tf_prefix': LaunchConfiguration('tf_prefix'),
        }.items(),
    )
    return LaunchDescription(arguments + [driver, controller, joint_states])
