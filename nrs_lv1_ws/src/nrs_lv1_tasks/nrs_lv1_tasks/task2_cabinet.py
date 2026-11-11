#!/usr/bin/env python3
"""Zadatak 2: geometrija ormarića.

Prima JSON s teme /cabinet/config i računa:
  * os rotacije vrata u svijetu:          ^W T_A = ^W T_O ^O T_A,
  * ^A T_D(0) po jednadžbi (1) iz uputa (uspoređuje se s T_D_hinge_closed iz poruke),
  * početnu pozu sustava S_D u svijetu po jednadžbi (2):
        ^W T_D(phi) = ^W T_O ^O T_A Rz(phi) ^A T_D(0),   phi = axis_pos * initial_angle,
i rezultat uspoređuje s T_D_world_initial iz poruke i s TF okvirom cabinet_D_initial.

Pokretanje (simulacija mora raditi):
    ros2 run nrs_lv1_tasks task2_cabinet --ros-args -p use_sim_time:=true
"""

import sys
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from nrs_lv1_tasks import cabinet_utils as cu


def tf_to_matrix(msg):
    t, q = msg.transform.translation, msg.transform.rotation
    return cu.make_T(cu.R_from_quat([q.x, q.y, q.z, q.w]), [t.x, t.y, t.z])


def compare(name, T_calc, T_ref):
    dp = np.linalg.norm(T_calc[:3, 3] - T_ref[:3, 3])
    dr = np.rad2deg(cu.rot_angle_between(T_calc[:3, :3], T_ref[:3, :3]))
    ok = 'OK' if dp < 1e-4 and dr < 1e-2 else 'RAZLIKA!'
    print(f'  {name:<36s} |dp| = {dp * 1000:8.4f} mm   |dR| = {dr:8.5f} deg   {ok}')


class Task2(Node):
    def __init__(self):
        super().__init__('task2_cabinet')
        self.cfg = None
        self.create_subscription(
            String, '/cabinet/config', self.on_config,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_config(self, msg):
        if self.cfg is None:
            self.cfg = cu.parse_config(msg.data)

    def lookup(self, child):
        try:
            return tf_to_matrix(self.tf_buffer.lookup_transform(
                'world', child, rclpy.time.Time(), timeout=Duration(seconds=5.0)))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'TF world->{child} nije dostupan: {exc}')
            return None


def report(node, cfg):
    np.set_printoptions(precision=4, suppress=True, linewidth=120)
    mu = cfg['axis_pos']
    x, y, z, yaw = cfg['pose']
    lx, ly, lz = cu.door_size(cfg)
    theta0 = cfg['initial_angle_deg']
    print('\n=== Konfiguracija ormarića (/cabinet/config) ===')
    print(f'  pose (x, y, z, yaw) = ({x:.4f}, {y:.4f}, {z:.4f}, {np.rad2deg(yaw):.2f} deg)')
    print(f'  l = [lx ly lz] = [{lx:.3f} {ly:.3f} {lz:.3f}] m,  mu = {mu},  theta0 = {theta0} deg')

    print('\n=== ^W T_O ===')
    print(cu.T_W_O(cfg))
    print('\n=== ^O T_A (T_A_cabinet_closed) ===')
    print(cu.T_O_A(cfg))

    T_WA = cu.T_W_A(cfg)
    print('\n=== Os rotacije vrata u svijetu (^W T_A) ===')
    print(f'  točka na osi (ishodište S_A): {T_WA[:3, 3]}')
    print(f'  smjer osi (z_A):              {T_WA[:3, 2]}')

    T_AD = cu.T_A_D0(cfg)
    print('\n=== Jednadžba (1): ^A T_D(0) ===')
    print(T_AD)
    compare('jedn. (1) vs T_D_hinge_closed', T_AD, cfg['T_D_hinge_closed'])

    T_WD = cu.T_W_D(cfg, theta0)
    print(f'\n=== Jednadžba (2): ^W T_D(phi), phi = mu*theta0 = {mu * theta0:.1f} deg ===')
    print(T_WD)

    print('\n=== Usporedba ===')
    compare('jedn. (2) vs T_D_world_initial', T_WD, cfg['T_D_world_initial'])
    T_tf = node.lookup('cabinet_D_initial')
    if T_tf is not None:
        compare('jedn. (2) vs TF cabinet_D_initial', T_WD, T_tf)
    T_tf_O = node.lookup('cabinet_origin')
    if T_tf_O is not None:
        compare('^W T_O vs TF cabinet_origin', cu.T_W_O(cfg), T_tf_O)


def main(args=None):
    rclpy.init(args=args)
    node = Task2()
    try:
        deadline = time.monotonic() + 20.0
        while rclpy.ok() and node.cfg is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.cfg is None:
            node.get_logger().error('Nije stigla poruka /cabinet/config (radi li simulacija?)')
            return 1
        for _ in range(20):                      # pusti TF da stigne
            rclpy.spin_once(node, timeout_sec=0.1)
        report(node, node.cfg)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
