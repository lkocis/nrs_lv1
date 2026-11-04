"""End-to-end loopback test through the ROS 2 FT streamer node."""

import socket
import threading
import time

from geometry_msgs.msg import WrenchStamped
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from robotiq_ft_streamer.node import RobotiqFTStreamerNode


def test_node_zeroes_then_publishes_measurement_frame():
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(('127.0.0.1', 0))
    listener.listen(1)
    port = listener.getsockname()[1]
    server_error = []

    def serve():
        try:
            connection, _ = listener.accept()
            with connection:
                time.sleep(0.3)
                connection.sendall(
                    b'(1,1,1,1,1,1)(3,3,3,3,3,3)'
                    b'(5,6,7,8,9,10)'
                )
                time.sleep(0.2)
        except Exception as error:  # pragma: no cover - surfaced below
            server_error.append(error)
        finally:
            listener.close()

    server = threading.Thread(target=serve, daemon=True)
    server.start()
    rclpy.init()
    streamer = RobotiqFTStreamerNode(
        parameter_overrides=[
            Parameter('robot_ip', value='127.0.0.1'),
            Parameter('sensor_port', value=port),
            Parameter('publish_rate_hz', value=0.0),
            Parameter('startup_zero', value=True),
            Parameter('zero_sample_count', value=2),
            Parameter('connect_timeout_sec', value=0.2),
            Parameter('read_timeout_sec', value=0.1),
            Parameter('stale_timeout_sec', value=0.5),
            Parameter('reconnect_delay_sec', value=0.05),
        ]
    )
    observer = Node('ft_streamer_test_observer')
    messages = []
    observer.create_subscription(
        WrenchStamped,
        '/robotiq_ft_sensor/wrench',
        messages.append,
        qos_profile_sensor_data,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(streamer)
    executor.add_node(observer)

    deadline = time.monotonic() + 3.0
    while not messages and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)

    executor.remove_node(streamer)
    executor.remove_node(observer)
    streamer.destroy_node()
    observer.destroy_node()
    executor.shutdown()
    rclpy.shutdown()
    server.join(timeout=1.0)

    assert not server_error
    assert messages
    message = messages[0]
    assert message.header.frame_id == 'ft_measurement_frame'
    assert message.wrench.force.x == 3.0
    assert message.wrench.force.y == 4.0
    assert message.wrench.force.z == 5.0
    assert message.wrench.torque.x == 6.0
    assert message.wrench.torque.y == 7.0
    assert message.wrench.torque.z == 8.0
