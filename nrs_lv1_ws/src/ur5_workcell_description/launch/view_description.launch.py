from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory('ur5_workcell_description'))
    xacro_file = package_share / 'urdf' / 'workcell.urdf.xacro'
    rviz_config = package_share / 'rviz' / 'view_workcell.rviz'

    tf_prefix = LaunchConfiguration('tf_prefix')
    include_cable_plugs = LaunchConfiguration('include_cable_plugs')
    articulated_gripper = LaunchConfiguration('articulated_gripper')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ', str(xacro_file),
            ' tf_prefix:=', tf_prefix,
            ' include_cable_plugs:=', include_cable_plugs,
            ' articulated_gripper:=', articulated_gripper,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument('tf_prefix', default_value=''),
        DeclareLaunchArgument('include_cable_plugs', default_value='true'),
        DeclareLaunchArgument(
            'articulated_gripper',
            default_value='true',
            description='Use movable AGS finger joints; false restores the fixed baseline.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            parameters=[{'robot_description': robot_description}],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(rviz_config)],
            output='screen',
        ),
    ])
