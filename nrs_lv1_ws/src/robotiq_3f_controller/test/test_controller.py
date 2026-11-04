"""Offline integration tests for the backend-neutral 3F controller."""

import time
import unittest

from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from rclpy.action import ActionClient
from robotiq_3f_interfaces.action import Move
from robotiq_3f_interfaces.msg import Robotiq3FCommand, Robotiq3FStatus
from std_srvs.srv import Trigger


@pytest.mark.launch_test
def generate_test_description():
    """Start a command-enabled controller against a test-owned fake backend."""
    controller = Node(
        package='robotiq_3f_controller',
        executable='robotiq_3f_controller_node',
        namespace='test_robotiq_3f',
        name='controller',
        parameters=[{
            'activation_timeout_s': 3.0,
            'status_timeout_s': 1.0,
        }],
        output='screen',
    )
    return LaunchDescription([
        controller,
        launch_testing.actions.ReadyToTest(),
    ])


class TestControllerActivation(unittest.TestCase):
    """Exercise the explicit reset-then-activate state machine."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('robotiq_3f_controller_test')
        self.commands = []
        self.status_publisher = self.node.create_publisher(
            Robotiq3FStatus,
            '/test_robotiq_3f/backend/status',
            10,
        )
        self.command_subscription = self.node.create_subscription(
            Robotiq3FCommand,
            '/test_robotiq_3f/backend/command',
            self.commands.append,
            10,
        )
        self.activate_client = self.node.create_client(
            Trigger,
            '/test_robotiq_3f/activate',
        )
        self.move_client = ActionClient(
            self.node,
            Move,
            '/test_robotiq_3f/move',
        )
        self.gripper_command_client = ActionClient(
            self.node,
            GripperCommand,
            '/test_robotiq_3f/gripper_command',
        )

    def tearDown(self):
        self.gripper_command_client.destroy()
        self.move_client.destroy()
        self.node.destroy_node()

    def _publish_status(self, **values):
        status = Robotiq3FStatus()
        status.connected = True
        status.connection_id = 42
        status.activated = True
        status.mode = Robotiq3FStatus.MODE_BASIC
        status.initialization_status = Robotiq3FStatus.INITIALIZATION_READY
        for name, value in values.items():
            setattr(status, name, value)
        self.status_publisher.publish(status)

    def test_activation_waits_for_each_written_command_id(self):
        """Activation cannot skip reset due to an already matching status."""
        deadline = time.monotonic() + 5.0
        while not self.activate_client.wait_for_service(timeout_sec=0.05):
            self._publish_status()
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self.assertLess(time.monotonic(), deadline)

        for _ in range(5):
            self._publish_status()
            rclpy.spin_once(self.node, timeout_sec=0.05)

        future = self.activate_client.call_async(Trigger.Request())
        reset_acknowledged = False
        activate_acknowledged = False
        reset_command_id = 0
        activate_command_id = 0
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.commands and not reset_acknowledged:
                reset = self.commands[0]
                self.assertFalse(reset.activate)
                self.assertNotEqual(reset.command_id, 0)
                reset_command_id = reset.command_id
                reset_acknowledged = True
            elif len(self.commands) >= 2 and not activate_acknowledged:
                activate = self.commands[1]
                self.assertTrue(activate.activate)
                self.assertGreater(activate.command_id, self.commands[0].command_id)
                activate_command_id = activate.command_id
                activate_acknowledged = True

            if activate_acknowledged:
                self._publish_status(last_command_id=activate_command_id)
            elif reset_acknowledged:
                self._publish_status(
                    activated=False,
                    initialization_status=Robotiq3FStatus.INITIALIZATION_RESET,
                    last_command_id=reset_command_id,
                )
            else:
                self._publish_status()

        self.assertTrue(future.done())
        self.assertTrue(future.result().success)
        self.assertEqual(len(self.commands), 2)

    def test_move_timeout_sends_stop_command(self):
        """A connected backend receives go-to false when motion times out."""
        deadline = time.monotonic() + 5.0
        while not self.move_client.wait_for_server(timeout_sec=0.05):
            self._publish_status()
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self.assertLess(time.monotonic(), deadline)

        for _ in range(3):
            self._publish_status()
            rclpy.spin_once(self.node, timeout_sec=0.05)

        goal = Move.Goal()
        goal.mode = Robotiq3FCommand.MODE_BASIC
        goal.position_a = 25
        goal.speed_a = 10
        goal.force_a = 10
        goal.timeout.nanosec = 300_000_000
        send_future = self.move_client.send_goal_async(goal)
        while not send_future.done() and time.monotonic() < deadline:
            self._publish_status()
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertTrue(send_future.done())
        goal_handle = send_future.result()
        self.assertTrue(goal_handle.accepted)
        result_future = goal_handle.get_result_async()
        while not result_future.done() and time.monotonic() < deadline:
            if self.commands:
                move = self.commands[0]
                self._publish_status(
                    mode=Robotiq3FStatus.MODE_BASIC,
                    go_to=True,
                    motion_status=Robotiq3FStatus.MOTION_MOVING,
                    requested_position_a=move.position_a,
                    last_command_id=move.command_id,
                )
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertTrue(result_future.done())
        wrapped_result = result_future.result()
        while len(self.commands) < 2 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.assertEqual(wrapped_result.status, GoalStatus.STATUS_ABORTED)
        self.assertIn('Stop sent', wrapped_result.result.message)
        self.assertEqual(len(self.commands), 2)
        self.assertTrue(self.commands[0].go_to)
        self.assertFalse(self.commands[1].go_to)

    def test_standard_gripper_command_maps_si_units_to_pinch_mode(self):
        """The compatibility action converts SI units and selects Pinch."""
        deadline = time.monotonic() + 5.0
        while not self.gripper_command_client.wait_for_server(
                timeout_sec=0.05):
            self._publish_status()
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self.assertLess(time.monotonic(), deadline)

        for _ in range(3):
            self._publish_status()
            rclpy.spin_once(self.node, timeout_sec=0.05)

        goal = GripperCommand.Goal()
        goal.command.position = 0.0775
        goal.command.max_effort = 30.0
        send_future = self.gripper_command_client.send_goal_async(goal)
        while not send_future.done() and time.monotonic() < deadline:
            self._publish_status()
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertTrue(send_future.done())
        goal_handle = send_future.result()
        self.assertTrue(goal_handle.accepted)
        result_future = goal_handle.get_result_async()
        while not result_future.done() and time.monotonic() < deadline:
            if self.commands:
                command = self.commands[0]
                self._publish_status(
                    mode=Robotiq3FStatus.MODE_PINCH,
                    go_to=True,
                    motion_status=Robotiq3FStatus.MOTION_REACHED,
                    requested_position_a=command.position_a,
                    actual_position_a=command.position_a,
                    last_command_id=command.command_id,
                )
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertTrue(result_future.done())
        wrapped_result = result_future.result()
        self.assertEqual(wrapped_result.status, GoalStatus.STATUS_SUCCEEDED)
        self.assertEqual(len(self.commands), 1)
        command = self.commands[0]
        self.assertEqual(command.mode, Robotiq3FCommand.MODE_PINCH)
        self.assertFalse(command.individual_finger_control)
        self.assertFalse(command.individual_scissor_control)
        self.assertEqual(command.position_a, 56)
        self.assertEqual(command.position_b, 56)
        self.assertEqual(command.position_c, 56)
        self.assertEqual(command.speed_a, 20)
        self.assertEqual(command.force_a, 85)
        self.assertTrue(wrapped_result.result.reached_goal)
        self.assertFalse(wrapped_result.result.stalled)
        self.assertAlmostEqual(
            wrapped_result.result.position,
            0.0775,
        )
        self.assertEqual(wrapped_result.result.effort, 0.0)

    def test_standard_gripper_command_rejects_out_of_range_position(self):
        """A standard goal outside the configured physical gap is rejected."""
        deadline = time.monotonic() + 5.0
        while not self.gripper_command_client.wait_for_server(
                timeout_sec=0.05):
            rclpy.spin_once(self.node, timeout_sec=0.05)
            self.assertLess(time.monotonic(), deadline)

        goal = GripperCommand.Goal()
        goal.command.position = 0.2
        goal.command.max_effort = 30.0
        future = self.gripper_command_client.send_goal_async(goal)
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertTrue(future.done())
        self.assertFalse(future.result().accepted)
        self.assertEqual(self.commands, [])
