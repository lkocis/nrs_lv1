from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    def include(package, filename, arguments, condition=None):
        options = {} if condition is None else {'condition': IfCondition(condition)}
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory(package) + '/launch/' + filename),
            launch_arguments=arguments.items(), **options)

    desc = get_package_share_directory('ur5_workcell_description')
    defaults = {
        'robot_ip': '192.168.88.245', 'gripper_ip': '192.168.88.237',
        'reverse_ip': '0.0.0.0', 'use_mock_hardware': 'false',
        'launch_gripper': 'true', 'launch_ft_sensor': 'false',
        'launch_moveit': 'true', 'launch_rviz': 'true',
        'kinematics_params_file': desc + '/config/ur5_calibration.yaml',
    }
    args = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    lc = LaunchConfiguration
    return LaunchDescription(args + [
        include('ur_robot_driver', 'ur_control.launch.py', {
            'ur_type': 'ur5', 'robot_ip': lc('robot_ip'),
            'reverse_ip': lc('reverse_ip'),
            'use_mock_hardware': lc('use_mock_hardware'),
            'kinematics_params_file': lc('kinematics_params_file'),
            'description_file': desc + '/urdf/workcell_control.urdf.xacro',
            'initial_joint_controller': 'scaled_joint_trajectory_controller',
            'launch_rviz': 'false', 'launch_dashboard_client': 'false',
        }),
        include('robotiq_3f_controller', 'hardware.launch.py', {
            'gripper_ip': lc('gripper_ip'),
        }, lc('launch_gripper')),
        include('robotiq_ft_streamer', 'ft_streamer.launch.py', {
            'robot_ip': lc('robot_ip'), 'startup_zero': 'false',
        }, lc('launch_ft_sensor')),
        include('ur5_workcell_moveit_config', 'moveit.launch.py', {
            'launch_rviz': lc('launch_rviz'), 'use_sim_time': 'false',
        }, lc('launch_moveit')),
    ])
