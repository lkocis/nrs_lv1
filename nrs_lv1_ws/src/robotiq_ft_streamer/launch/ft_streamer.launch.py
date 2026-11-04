"""Launch the Robotiq FT TCP streamer independently."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _argument(name, default, description, required=False):
    options = {'description': description}
    if not required:
        options['default_value'] = str(default)
    return DeclareLaunchArgument(name, **options)


def generate_launch_description():
    """Build the standalone FT streamer launch description."""
    package_share = Path(
        get_package_share_directory('robotiq_ft_streamer')
    )
    arguments = [
        _argument(
            'robot_ip',
            '',
            'IP address of the UR controller exposing TCP port 63351.',
            required=True,
        ),
        _argument('sensor_port', 63351, 'Robotiq FT stream TCP port.'),
        _argument(
            'frame_id',
            'ft_measurement_frame',
            'TF frame assigned to wrench samples.',
        ),
        _argument(
            'wrench_topic',
            '/robotiq_ft_sensor/wrench',
            'Published WrenchStamped topic.',
        ),
        _argument(
            'publish_rate_hz',
            100.0,
            'Maximum publish rate; zero publishes every received sample.',
        ),
        _argument(
            'startup_zero',
            'false',
            'Average initial samples before publishing.',
        ),
        _argument(
            'zero_sample_count',
            100,
            'Number of valid samples used for software zeroing.',
        ),
    ]

    node = Node(
        package='robotiq_ft_streamer',
        executable='robotiq_ft_streamer',
        name='robotiq_ft_streamer',
        output='screen',
        parameters=[
            str(package_share / 'config' / 'ft_streamer.yaml'),
            {
                'robot_ip': LaunchConfiguration('robot_ip'),
                'sensor_port': ParameterValue(
                    LaunchConfiguration('sensor_port'), value_type=int
                ),
                'frame_id': LaunchConfiguration('frame_id'),
                'wrench_topic': LaunchConfiguration('wrench_topic'),
                'publish_rate_hz': ParameterValue(
                    LaunchConfiguration('publish_rate_hz'), value_type=float
                ),
                'startup_zero': ParameterValue(
                    LaunchConfiguration('startup_zero'), value_type=bool
                ),
                'zero_sample_count': ParameterValue(
                    LaunchConfiguration('zero_sample_count'), value_type=int
                ),
            },
        ],
    )
    return LaunchDescription(arguments + [node])
