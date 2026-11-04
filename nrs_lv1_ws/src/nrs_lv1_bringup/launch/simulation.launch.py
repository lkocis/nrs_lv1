from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def spawn_cabinet(context):
    parameters = [LaunchConfiguration('cabinet_config').perform(context),
                  {'use_sim_time': True}]
    seed = LaunchConfiguration('seed').perform(context)
    if seed:
        parameters.append({'seed': int(seed)})
    return [Node(package='cabinet_generator', executable='spawn_cabinet',
                 name='cabinet_spawner', output='screen',
                 condition=IfCondition(LaunchConfiguration('spawn_cabinet')),
                 parameters=parameters)]


def generate_launch_description():
    lab = get_package_share_directory('nrs_lv1_bringup')
    sim = get_package_share_directory('ur5_workcell_sim')
    return LaunchDescription([
        DeclareLaunchArgument('gazebo_gui', default_value='true'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('render_engine', default_value=EnvironmentVariable(
            'NRS_RENDER_ENGINE', default_value='ogre2'), choices=['ogre', 'ogre2']),
        DeclareLaunchArgument('spawn_cabinet', default_value='true'),
        DeclareLaunchArgument('cabinet_config', default_value=lab + '/config/cabinet.yaml'),
        DeclareLaunchArgument('seed', default_value='',
                              description='Override the seed in cabinet_config.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sim + '/launch/sim.launch.py'),
            launch_arguments={
                'gazebo_gui': LaunchConfiguration('gazebo_gui'),
                'launch_rviz': LaunchConfiguration('launch_rviz'),
                'render_engine': LaunchConfiguration('render_engine'),
            }.items()),
        OpaqueFunction(function=spawn_cabinet),
    ])
