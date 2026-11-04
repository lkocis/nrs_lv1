"""The arm and articulated gripper must hold their startup positions."""

import math
import os
import signal
import subprocess
import time
import uuid

import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState


def test_idle_simulation(tmp_path):
    domain = 90 + os.getpid() % 100
    environment = dict(os.environ, ROS_DOMAIN_ID=str(domain),
                       GZ_PARTITION='nrs_test_' + uuid.uuid4().hex)
    context = Context()
    rclpy.init(context=context, domain_id=domain)
    node = Node('idle_simulation_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    samples = []
    subscription = node.create_subscription(JointState, '/joint_states', samples.append, 100)
    controllers = node.create_client(ListControllers, '/controller_manager/list_controllers')
    logfile = tmp_path / 'simulation.log'
    output = logfile.open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'ur5_workcell_sim', 'sim.launch.py',
        'gazebo_gui:=false', 'launch_rviz:=false', 'launch_moveit:=false',
    ], env=environment, stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        deadline = time.monotonic() + 120
        started = None
        future = None
        active = set()
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if controllers.service_is_ready() and future is None:
                future = controllers.call_async(ListControllers.Request())
            if future is not None and future.done():
                active = {c.name for c in future.result().controller if c.state == 'active'}
                future = None
            if samples and len(active) == 3:
                stamp = samples[-1].header.stamp
                now = stamp.sec + stamp.nanosec / 1e9
                if started is None:
                    started = now
                if now - started >= 30:
                    break
        else:
            raise AssertionError('Simulation did not reach 30 seconds\n' + logfile.read_text()[-5000:])

        expected = dict(zip([
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
        ], [0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0]))
        expected.update({name: 0.0 for name in samples[-1].name if name.startswith('robotiq_3f_')})
        expected['robotiq_3f_finger_b_scissor_joint'] = -0.16406094968746698
        expected['robotiq_3f_finger_c_scissor_joint'] = 0.16406094968746698
        assert len(expected) == 17
        settled = [msg for msg in samples
                   if msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9 >= started + 3]
        assert settled
        for name, target in expected.items():
            error = max(abs(msg.position[msg.name.index(name)] - target) for msg in settled)
            assert error < 0.002, f'{name} drifted by {error:.6f} rad'
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        output.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)
