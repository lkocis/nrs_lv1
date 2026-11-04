"""Publish articulated joint states from Robotiq 3F register feedback."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Build the status-to-joint-state mapper launch description."""
    package_share = Path(
        get_package_share_directory('robotiq_3f_state_publisher')
    )
    config_file = package_share / 'config' / 'joint_mapping.yaml'

    arguments = [
        DeclareLaunchArgument('tf_prefix', default_value=''),
        DeclareLaunchArgument(
            'status_topic', default_value='/robotiq_3f/status'
        ),
        DeclareLaunchArgument(
            'joint_states_topic', default_value='/joint_states'
        ),
        DeclareLaunchArgument(
            'publish_reference_without_status', default_value='true'
        ),
    ]
    node = Node(
        package='robotiq_3f_state_publisher',
        executable='robotiq_3f_joint_state_publisher_node',
        namespace='robotiq_3f',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            str(config_file),
            {
                'tf_prefix': LaunchConfiguration('tf_prefix'),
                'status_topic': LaunchConfiguration('status_topic'),
                'joint_states_topic': LaunchConfiguration(
                    'joint_states_topic'
                ),
                'publish_reference_without_status': LaunchConfiguration(
                    'publish_reference_without_status'
                ),
            },
        ],
    )
    return LaunchDescription(arguments + [node])
