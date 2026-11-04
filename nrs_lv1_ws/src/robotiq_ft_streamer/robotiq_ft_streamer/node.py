"""ROS 2 node for the UR-controller Robotiq FT TCP stream."""

import threading
import time

from geometry_msgs.msg import WrenchStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robotiq_ft_streamer.client import FTStreamClient
from robotiq_ft_streamer.zeroing import ZeroingManager
from std_srvs.srv import Trigger


class RobotiqFTStreamerNode(Node):
    """Publish a software-zeroed external Robotiq FT wrench stream."""

    def __init__(self, parameter_overrides=None):
        super().__init__(
            'robotiq_ft_streamer',
            parameter_overrides=parameter_overrides,
        )

        self.declare_parameter('robot_ip', '')
        self.declare_parameter('sensor_port', 63351)
        self.declare_parameter('frame_id', 'ft_measurement_frame')
        self.declare_parameter('wrench_topic', '/robotiq_ft_sensor/wrench')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('startup_zero', False)
        self.declare_parameter('zero_sample_count', 100)
        self.declare_parameter('zero_timeout_sec', 5.0)
        self.declare_parameter('connect_timeout_sec', 3.0)
        self.declare_parameter('read_timeout_sec', 1.0)
        self.declare_parameter('stale_timeout_sec', 5.0)
        self.declare_parameter('reconnect_delay_sec', 2.0)
        self.declare_parameter('max_buffer_bytes', 65536)

        robot_ip = self._string_parameter('robot_ip')
        sensor_port = self._positive_integer_parameter('sensor_port')
        self._frame_id = self._string_parameter('frame_id')
        wrench_topic = self._string_parameter('wrench_topic')
        self._publish_rate_hz = self._nonnegative_float_parameter(
            'publish_rate_hz'
        )
        self._zero_sample_count = self._positive_integer_parameter(
            'zero_sample_count'
        )
        self._zero_timeout_sec = self._positive_float_parameter(
            'zero_timeout_sec'
        )
        connect_timeout_sec = self._positive_float_parameter(
            'connect_timeout_sec'
        )
        read_timeout_sec = self._positive_float_parameter(
            'read_timeout_sec'
        )
        stale_timeout_sec = self._positive_float_parameter(
            'stale_timeout_sec'
        )
        reconnect_delay_sec = self._positive_float_parameter(
            'reconnect_delay_sec'
        )
        max_buffer_bytes = self._positive_integer_parameter(
            'max_buffer_bytes'
        )
        if stale_timeout_sec < read_timeout_sec:
            raise ValueError(
                'stale_timeout_sec must be greater than or equal to '
                'read_timeout_sec'
            )

        self._publisher = self.create_publisher(
            WrenchStamped,
            wrench_topic,
            qos_profile_sensor_data,
        )
        self._zero_service = self.create_service(
            Trigger,
            '~/zero',
            self._zero_service_callback,
        )
        self._zeroing = ZeroingManager()
        self._stop_event = threading.Event()
        self._worker_join_timeout = max(
            connect_timeout_sec,
            read_timeout_sec,
        ) + 0.5
        self._next_publish_time = 0.0
        self._connected = False

        if self.get_parameter('startup_zero').value:
            self._zeroing.request(self._zero_sample_count)
            self.get_logger().info(
                'Startup zero requested; keep the tool unloaded and still '
                f'for {self._zero_sample_count} samples'
            )

        self._client = FTStreamClient(
            host=robot_ip,
            port=sensor_port,
            sample_callback=self._sample_callback,
            connection_callback=self._connection_callback,
            connect_timeout_sec=connect_timeout_sec,
            read_timeout_sec=read_timeout_sec,
            stale_timeout_sec=stale_timeout_sec,
            reconnect_delay_sec=reconnect_delay_sec,
            max_buffer_bytes=max_buffer_bytes,
        )
        self._worker = threading.Thread(
            target=self._client.run,
            args=(self._stop_event,),
            name='robotiq_ft_tcp',
            daemon=True,
        )
        self._worker.start()
        self.get_logger().info(
            f'FT streamer configured for {robot_ip}:{sensor_port}; publishing '
            f'{wrench_topic} in frame {self._frame_id}'
        )

    def _sample_callback(self, sample):
        corrected, zero_result = self._zeroing.process(sample)
        if zero_result is not None:
            formatted_offset = ', '.join(
                f'{value:.6g}' for value in zero_result.offset
            )
            self.get_logger().info(
                f'Software zero complete; offset=[{formatted_offset}]'
            )
        if corrected is None or not self._publish_due():
            return

        message = WrenchStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._frame_id
        message.wrench.force.x = corrected[0]
        message.wrench.force.y = corrected[1]
        message.wrench.force.z = corrected[2]
        message.wrench.torque.x = corrected[3]
        message.wrench.torque.y = corrected[4]
        message.wrench.torque.z = corrected[5]
        self._publisher.publish(message)

    def _publish_due(self):
        if self._publish_rate_hz == 0.0:
            return True
        now = time.monotonic()
        if now < self._next_publish_time:
            return False
        period = 1.0 / self._publish_rate_hz
        self._next_publish_time = max(
            self._next_publish_time + period,
            now,
        )
        return True

    def _connection_callback(self, connected, message):
        if connected:
            self._zeroing.reset_progress()
            self._connected = True
            self.get_logger().info('Connected to the Robotiq FT TCP stream')
            return

        if self._connected:
            self.get_logger().warning(
                f'Robotiq FT stream disconnected: {message}; reconnecting'
            )
        else:
            self.get_logger().warning(
                f'Could not connect to the Robotiq FT stream: {message}; '
                'retrying'
            )
        self._connected = False

    def _zero_service_callback(self, request, response):
        del request
        generation = self._zeroing.request(self._zero_sample_count)
        self.get_logger().info(
            f'Zero service collecting {self._zero_sample_count} samples'
        )
        result = self._zeroing.wait(generation, self._zero_timeout_sec)
        if result is None:
            self._zeroing.cancel(generation)
            response.success = False
            response.message = (
                'Software zero timed out or was superseded; previous offset '
                'was retained'
            )
            return response

        response.success = True
        response.message = 'Software zero completed'
        return response

    def destroy_node(self):
        """Stop the worker before destroying ROS entities."""
        self._stop_event.set()
        self._zeroing.close()
        worker = getattr(self, '_worker', None)
        if worker is not None and worker.is_alive():
            worker.join(timeout=self._worker_join_timeout)
        return super().destroy_node()

    def _string_parameter(self, name):
        value = str(self.get_parameter(name).value).strip()
        if not value:
            raise ValueError(f'{name} must not be empty')
        return value

    def _positive_integer_parameter(self, name):
        value = self.get_parameter(name).value
        if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
            raise ValueError(f'{name} must be a positive integer')
        return value

    def _positive_float_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if value <= 0.0:
            raise ValueError(f'{name} must be greater than zero')
        return value

    def _nonnegative_float_parameter(self, name):
        value = float(self.get_parameter(name).value)
        if value < 0.0:
            raise ValueError(f'{name} must be greater than or equal to zero')
        return value


def main(args=None):
    """Run the Robotiq FT streamer node."""
    rclpy.init(args=args)
    node = None
    try:
        node = RobotiqFTStreamerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
