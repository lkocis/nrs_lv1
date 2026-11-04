"""Start MoveIt for an already-running authoritative workcell bringup."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Build a planning-first MoveIt launch description."""
    package_share = Path(
        get_package_share_directory('ur5_workcell_moveit_config')
    )
    launch_rviz = LaunchConfiguration('launch_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    articulated_gripper = LaunchConfiguration('articulated_gripper')

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name='ur5_workcell',
            package_name='ur5_workcell_moveit_config',
        )
        .robot_description(mappings={
            'safety_limits': 'true',
            'articulated_gripper': articulated_gripper,
        })
        .robot_description_semantic(
            file_path='config/ur5_workcell.srdf'
        )
        .robot_description_kinematics(file_path='config/kinematics.yaml')
        .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(file_path='config/moveit_controllers.yaml')
        .planning_pipelines(
            default_planning_pipeline='ompl',
            pipelines=['ompl'],
        )
        .to_moveit_configs()
    )

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {
                'publish_robot_description': False,
                'use_sim_time': use_sim_time,
                'publish_robot_description_semantic': True,
                'publish_planning_scene': True,
                'publish_geometry_updates': True,
                'publish_state_updates': True,
                'publish_transforms_updates': True,
            },
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='moveit_rviz',
        output='screen',
        condition=IfCondition(launch_rviz),
        arguments=['-d', str(package_share / 'config' / 'moveit.rviz')],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz with the MoveIt planning panel.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the Gazebo simulation clock.',
        ),
        DeclareLaunchArgument(
            'articulated_gripper',
            default_value='true',
            description='Use the articulated Robotiq model in MoveIt.',
        ),
        move_group,
        rviz,
    ])
