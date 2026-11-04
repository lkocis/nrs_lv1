"""Launch the Robotiq 3F Modbus TCP backend without implicit motion."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Create a safe-by-default standalone hardware-backend launch."""
    arguments = [
        DeclareLaunchArgument(
            'gripper_ip',
            description='IP address of the Robotiq 3F Modbus TCP device.',
        ),
        DeclareLaunchArgument('port', default_value='502'),
        DeclareLaunchArgument('unit_id', default_value='0'),
        DeclareLaunchArgument('poll_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('connect_timeout_ms', default_value='1000'),
        DeclareLaunchArgument('response_timeout_ms', default_value='500'),
        DeclareLaunchArgument('reconnect_initial_ms', default_value='500'),
        DeclareLaunchArgument('reconnect_max_ms', default_value='5000'),
        DeclareLaunchArgument(
            'allow_automatic_release', default_value='false',
            description='Allow dangerous automatic-release register writes.',
        ),
    ]

    parameters = {
        name: LaunchConfiguration(name)
        for name in [
            'gripper_ip', 'port', 'unit_id', 'poll_rate_hz',
            'connect_timeout_ms', 'response_timeout_ms',
            'reconnect_initial_ms', 'reconnect_max_ms',
            'allow_automatic_release',
        ]
    }
    node = Node(
        package='robotiq_3f_driver',
        executable='robotiq_3f_driver_node',
        namespace='robotiq_3f',
        name='driver',
        output='screen',
        parameters=[parameters],
    )
    return LaunchDescription(arguments + [node])
