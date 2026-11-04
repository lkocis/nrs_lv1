#!/usr/bin/env python3
"""Register-compatible Robotiq 3F backend for Gazebo Sim."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from robotiq_3f_interfaces.msg import Robotiq3FCommand, Robotiq3FStatus
from std_msgs.msg import Float64MultiArray


def interpolate(inputs, outputs, value):
    """Evaluate a clamped piecewise-linear curve."""
    if value <= inputs[0]:
        return outputs[0]
    if value >= inputs[-1]:
        return outputs[-1]
    for index in range(1, len(inputs)):
        if value <= inputs[index]:
            fraction = ((value - inputs[index - 1]) /
                        (inputs[index] - inputs[index - 1]))
            return outputs[index - 1] + fraction * (
                outputs[index] - outputs[index - 1])
    raise RuntimeError('unreachable interpolation interval')


class Robotiq3FSimBackend(Node):
    """Emulate the native process image and command Gazebo finger joints."""

    def __init__(self):
        super().__init__('sim_backend')
        self._declare_mapping_parameters()
        self._read_mapping_parameters()

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self._status_publisher = self.create_publisher(
            Robotiq3FStatus, 'backend/status', qos)
        self._position_publisher = self.create_publisher(
            Float64MultiArray,
            '/robotiq_3f_position_controller/commands',
            qos,
        )
        self._command_subscription = self.create_subscription(
            Robotiq3FCommand,
            'backend/command',
            self._receive_command,
            qos,
        )

        self._status = Robotiq3FStatus()
        self._status.connected = True
        self._status.connection_id = 1
        self._status.diagnostic = 'Gazebo simulated Robotiq 3F'
        self._status.mode = Robotiq3FStatus.MODE_PINCH
        self._status.initialization_status = (
            Robotiq3FStatus.INITIALIZATION_RESET)
        self._status.motion_status = Robotiq3FStatus.MOTION_REACHED
        self._actual = [6.0, 6.0, 6.0, 220.0]
        self._target = list(self._actual)
        self._speeds = [255, 255, 255, 255]
        self._last_update = self.get_clock().now()
        self.create_timer(1.0 / self._update_rate, self._update)
        self.get_logger().info(
            'Simulated Robotiq 3F backend supports Basic, Pinch, Wide, '
            'Scissor, individual fingers, and individual scissor control')

    def _declare_mapping_parameters(self):
        self.declare_parameter('update_rate_hz', 50.0)
        self.declare_parameter('minimum_raw_speed_per_s', 20.0)
        self.declare_parameter('maximum_raw_speed_per_s', 255.0)
        self.declare_parameter('mode_scissor.basic', 137)
        self.declare_parameter('mode_scissor.pinch', 220)
        self.declare_parameter('mode_scissor.wide', 24)
        self.declare_parameter('finger.breakpoints',
                               [0, 6, 32, 60, 96, 128, 148, 164,
                                192, 225, 240, 255])
        self.declare_parameter('finger.proximal_rad',
                               [0.0, 0.0, 0.20943951023931956,
                                0.4886921905584123, 0.7853981633974483,
                                1.0777408131064985, 1.0777408131064985,
                                1.0777408131064985, 1.0777408131064985,
                                1.0777408131064985, 1.0777408131064985,
                                1.0777408131064985])
        self.declare_parameter('finger.middle_rad',
                               [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.08726646259971647, 0.4363323129985824,
                                0.9337511498169663, 1.3089969389957472,
                                1.509709802975095, 1.509709802975095])
        self.declare_parameter('finger.distal_rad',
                               [0.0, 0.0, -0.2792526803190927,
                                -0.5148721293383273, -0.7853981633974483,
                                -1.0777408131064985, -0.6894050545377601,
                                -0.6894050545377601, -0.6894050545377601,
                                -0.6894050545377601, -0.6894050545377601,
                                -0.6894050545377601])
        self.declare_parameter('pinch_finger.breakpoints',
                               [0, 6, 32, 60, 96, 112, 113, 255])
        self.declare_parameter('pinch_finger.proximal_rad',
                               [0.0, 0.0, 0.20943951023931956,
                                0.4886921905584123, 0.7853981633974483,
                                0.8552113334772214, 0.8552113334772214,
                                0.8552113334772214])
        self.declare_parameter('pinch_finger.middle_rad', [0.0] * 8)
        self.declare_parameter('pinch_finger.distal_rad',
                               [0.0, 0.0, -0.2792526803190927,
                                -0.5148721293383273, -0.7853981633974483,
                                -0.8552113334772214, -0.8552113334772214,
                                -0.8552113334772214])
        self.declare_parameter('scissor.breakpoints',
                               [0, 15, 24, 32, 60, 92, 128, 137,
                                164, 192, 220, 225, 233, 255])
        self.declare_parameter('scissor.angle_rad',
                               [0.2399827721492203, 0.2399827721492203,
                                0.22252947962927702, 0.20943951023931956,
                                0.14835298641951802, 0.08726646259971647,
                                0.017453292519943295, 0.0,
                                -0.05235987755982989, -0.10471975511965978,
                                -0.16406094968746698, -0.1684242728174528,
                                -0.17016960206944712,
                                -0.17016960206944712])

    def _read_mapping_parameters(self):
        value = lambda name: self.get_parameter(name).value
        self._update_rate = float(value('update_rate_hz'))
        self._minimum_speed = float(value('minimum_raw_speed_per_s'))
        self._maximum_speed = float(value('maximum_raw_speed_per_s'))
        if self._update_rate <= 0 or self._minimum_speed <= 0:
            raise ValueError('simulation rates must be positive')
        self._mode_scissor = {
            Robotiq3FCommand.MODE_BASIC: int(value('mode_scissor.basic')),
            Robotiq3FCommand.MODE_PINCH: int(value('mode_scissor.pinch')),
            Robotiq3FCommand.MODE_WIDE: int(value('mode_scissor.wide')),
        }
        self._finger_inputs = list(value('finger.breakpoints'))
        self._finger_outputs = [
            list(value('finger.proximal_rad')),
            list(value('finger.middle_rad')),
            list(value('finger.distal_rad')),
        ]
        self._pinch_inputs = list(value('pinch_finger.breakpoints'))
        self._pinch_outputs = [
            list(value('pinch_finger.proximal_rad')),
            list(value('pinch_finger.middle_rad')),
            list(value('pinch_finger.distal_rad')),
        ]
        self._scissor_inputs = list(value('scissor.breakpoints'))
        self._scissor_outputs = list(value('scissor.angle_rad'))

    def _receive_command(self, command):
        if command.command_id == 0 or command.mode > command.MODE_SCISSOR:
            return
        self._status.last_command_id = command.command_id
        self._status.mode = command.mode
        self._status.go_to = command.go_to
        self._status.requested_position_a = command.position_a
        self._status.requested_position_b = command.position_b
        self._status.requested_position_c = command.position_c
        self._status.requested_position_scissor = command.position_scissor

        if not command.activate:
            self._status.activated = False
            self._status.initialization_status = (
                Robotiq3FStatus.INITIALIZATION_RESET)
            self._status.motion_status = Robotiq3FStatus.MOTION_REACHED
            return

        self._status.activated = True
        self._status.initialization_status = (
            Robotiq3FStatus.INITIALIZATION_READY)
        if not command.go_to:
            self._target = list(self._actual)
            self._status.motion_status = Robotiq3FStatus.MOTION_REACHED
            return

        a = float(command.position_a)
        if command.mode == Robotiq3FCommand.MODE_SCISSOR and not (
                command.individual_scissor_control):
            targets = [6.0, 6.0, 6.0, a]
        else:
            b = float(command.position_b if
                      command.individual_finger_control else command.position_a)
            c = float(command.position_c if
                      command.individual_finger_control else command.position_a)
            if command.individual_scissor_control:
                scissor = float(command.position_scissor)
            else:
                scissor = float(self._mode_scissor.get(command.mode, 137))
            targets = [a, b, c, scissor]
        self._target = targets
        self._speeds = [
            command.speed_a,
            command.speed_b if command.individual_finger_control
            else command.speed_a,
            command.speed_c if command.individual_finger_control
            else command.speed_a,
            command.speed_scissor if command.individual_scissor_control
            else command.speed_a,
        ]
        self._status.motion_status = Robotiq3FStatus.MOTION_MOVING

    def _raw_speed(self, raw):
        return self._minimum_speed + (self._maximum_speed - self._minimum_speed) * (
            float(raw) / 255.0)

    def _update(self):
        now = self.get_clock().now()
        dt = max(0.0, (now - self._last_update).nanoseconds / 1.0e9)
        self._last_update = now
        moving = False
        if self._status.activated and self._status.go_to:
            for index, target in enumerate(self._target):
                error = target - self._actual[index]
                step = self._raw_speed(self._speeds[index]) * dt
                if abs(error) > step:
                    self._actual[index] += math.copysign(step, error)
                    moving = True
                else:
                    self._actual[index] = target
            self._status.motion_status = (
                Robotiq3FStatus.MOTION_MOVING if moving
                else Robotiq3FStatus.MOTION_REACHED)

        self._publish_joint_targets()
        self._publish_status(moving)

    def _finger(self, raw, pinch):
        inputs = self._pinch_inputs if pinch else self._finger_inputs
        outputs = self._pinch_outputs if pinch else self._finger_outputs
        return [interpolate(inputs, values, raw) for values in outputs]

    def _publish_joint_targets(self):
        pinch = self._status.mode == Robotiq3FStatus.MODE_PINCH
        a = self._finger(self._actual[0], pinch)
        b = self._finger(self._actual[1], pinch)
        c = self._finger(self._actual[2], pinch)
        scissor = interpolate(
            self._scissor_inputs, self._scissor_outputs, self._actual[3])
        message = Float64MultiArray()
        message.data = [
            *a,
            scissor, *b,
            -scissor, *c,
        ]
        self._position_publisher.publish(message)

    def _publish_status(self, moving):
        raw = [max(0, min(255, round(value))) for value in self._actual]
        self._status.stamp = self.get_clock().now().to_msg()
        self._status.actual_position_a = raw[0]
        self._status.actual_position_b = raw[1]
        self._status.actual_position_c = raw[2]
        self._status.actual_position_scissor = raw[3]
        object_status = (Robotiq3FStatus.OBJECT_MOVING if moving else
                         Robotiq3FStatus.OBJECT_REACHED)
        self._status.object_status_a = object_status
        self._status.object_status_b = object_status
        self._status.object_status_c = object_status
        self._status.object_status_scissor = object_status
        self._status_publisher.publish(self._status)


def main(args=None):
    rclpy.init(args=args)
    node = Robotiq3FSimBackend()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, RuntimeError):
            pass
        try:
            rclpy.try_shutdown()
        except (KeyboardInterrupt, RuntimeError):
            pass


if __name__ == '__main__':
    main()
