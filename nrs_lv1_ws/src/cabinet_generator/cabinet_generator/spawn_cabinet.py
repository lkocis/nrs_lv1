"""Generate and spawn one cabinet, then publish its initial configuration."""

import json
from pathlib import Path
import subprocess
import tempfile

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from tf_transformations import quaternion_from_matrix

from cabinet_generator.cabinet_model import Cabinet
from cabinet_generator.placement import collision_corners, sample_pose, transform


class CabinetSpawner(Node):
    def __init__(self):
        super().__init__('cabinet_spawner')
        defaults = {
            'door_width': 0.28, 'door_height': 0.35, 'panel_thickness': 0.018,
            'cabinet_depth': 0.30, 'axis_pos': -1, 'initial_angle_deg': 30.0,
            'nominal_x': 0.0, 'nominal_y': -0.45, 'nominal_yaw': np.pi / 2,
            'position_jitter': 0.035, 'yaw_jitter': 0.15, 'seed': -1,
            'table_width': 0.9, 'table_length': 1.5, 'table_edge_offset': 0.21,
            'table_top_z': 0.0, 'table_margin': 0.04, 'world_name': 'workcell',
        }
        self.config = {key: self.declare_parameter(key, value).value
                       for key, value in defaults.items()}
        cfg = self.config
        if cfg['axis_pos'] not in (-1, 1):
            raise ValueError('axis_pos must be -1 or 1')
        for key in ('door_width', 'door_height', 'panel_thickness', 'cabinet_depth',
                    'table_width', 'table_length'):
            if cfg[key] <= 0:
                raise ValueError(f'{key} must be positive')
        if min(cfg['position_jitter'], cfg['yaw_jitter'], cfg['table_margin']) < 0:
            raise ValueError('Jitter and table margin must be nonnegative')
        self.directory = tempfile.TemporaryDirectory(prefix='nrs_cabinet_')
        model_file = Path(self.directory.name) / 'cabinet.urdf'
        self.cabinet = Cabinet(
            door_params=np.array([cfg['door_width'], cfg['door_height'],
                                  cfg['panel_thickness'], cfg['cabinet_depth']]),
            axis_pos=cfg['axis_pos'], r=np.array([0.0, -cfg['door_width'] / 2]),
            initial_angle_deg=cfg['initial_angle_deg'], has_handle=False,
            save_path=str(model_file))
        self.pose = sample_pose(collision_corners(model_file.read_text()), cfg)
        self.publisher = self.create_publisher(
            String, '/cabinet/config',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf = StaticTransformBroadcaster(self)
        x, y, z, yaw = self.pose
        self.process = subprocess.Popen([
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', cfg['world_name'], '-file', str(model_file),
            '-name', 'cabinet', '-allow_renaming', 'false',
            '-x', str(x), '-y', str(y), '-z', str(z), '-Y', str(yaw)])
        self.timer = self.create_timer(0.2, self.check_spawn)

    def check_spawn(self):
        result = self.process.poll()
        if result is None:
            return
        self.timer.cancel()
        if result != 0:
            self.get_logger().error(f'Cabinet spawn failed (exit {result})')
            return
        x, y, z, yaw = self.pose
        cabinet_pose = transform((x, y, z), (0, 0, yaw))
        hinge = self.cabinet.T_A_O_init
        opening = transform(rpy=(0, 0, self.config['axis_pos'] *
                                 np.deg2rad(self.config['initial_angle_deg'])))
        point_d = cabinet_pose @ hinge @ opening @ self.cabinet.T_D_A_init
        transforms = []
        for name, matrix in [('cabinet_origin', cabinet_pose),
                             ('cabinet_D_initial', point_d)]:
            msg = TransformStamped()
            msg.header.frame_id = 'world'
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.child_frame_id = name
            msg.transform.translation.x = float(matrix[0, 3])
            msg.transform.translation.y = float(matrix[1, 3])
            msg.transform.translation.z = float(matrix[2, 3])
            q = quaternion_from_matrix(matrix)
            (msg.transform.rotation.x, msg.transform.rotation.y,
             msg.transform.rotation.z, msg.transform.rotation.w) = map(float, q)
            transforms.append(msg)
        self.tf.sendTransform(transforms)
        data = dict(self.config, pose=self.pose, T_D_world_initial=point_d.tolist(),
                    T_A_cabinet_closed=hinge.tolist(),
                    T_D_hinge_closed=self.cabinet.T_D_A_init.tolist())
        self.publisher.publish(String(data=json.dumps(data)))
        self.get_logger().info(f'Cabinet pose [x, y, z, yaw]: {self.pose}')

    def destroy_node(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.directory.cleanup()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CabinetSpawner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
