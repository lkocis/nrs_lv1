#!/usr/bin/env python3
"""Zadatak 2: geometrija ormarića.

Prima JSON s teme /cabinet/config, računa
  * položaj i smjer osi rotacije vrata (z-os okvira A) u svijetu,
  * početni položaj sustava S_D u svijetu (jednadžba (2)),
i uspoređuje rezultat s matricom T_D_world_initial iz poruke i s TF okvirom
cabinet_D_initial (te cabinet_origin).

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
    print(f'  {name:<34s} |dp| = {dp * 1000:9.4f} mm   |dR| = {dr:9.5f} deg   '
          f'{"OK" if dp < 1e-4 and dr < 1e-2 else "RAZLIKA!"}')
    return dp, dr


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
    np.set_printoptions(precision=5, suppress=True, linewidth=120)
    mu = cfg['axis_pos']
    x, y, z, yaw = cfg['pose']
    lx, ly, lz = cu.door_size(cfg)
    print('\n=== Konfiguracija ormarića (/cabinet/config) ===')
    print(f'  pose (x, y, z, yaw) = ({x:.4f}, {y:.4f}, {z:.4f}, {np.rad2deg(yaw):.2f} deg)')
    print(f'  vrata l = [lx ly lz] = [{lx:.3f} {ly:.3f} {lz:.3f}] m, '
          f'mu = axis_pos = {mu}, početni kut = {cfg["initial_angle_deg"]} deg')

    T_WO = cu.T_O_W(cfg)
    T_OA = cu.T_A_O_closed(cfg)
    T_WA = cu.T_A_world(cfg)
    print('\n=== Os rotacije vrata ===')
    print('  ^O T_A (provjera s JSON-om):')
    compare('^O T_A  vs T_A_cabinet_closed', T_OA, cfg['T_A_cabinet_closed'])
    print(f'  hinge točka (ishodište A) u svijetu   : {T_WA[:3, 3]}')
    print(f'  smjer osi zgloba (z_A) u svijetu      : {T_WA[:3, 2]}')
    print(f'  zglob u okviru ormarića O             : {T_OA[:3, 3]}')

    T_DA = cu.T_D_A_closed(cfg)
    print('\n=== Jednadžba (1): ^A T_D(0) ===')
    print(T_DA)
    compare('^A T_D(0) vs T_D_hinge_closed', T_DA, cfg['T_D_hinge_closed'])

    phi = cu.phi_from_opening(cfg, cfg['initial_angle_deg'])
    T_WD = cu.T_D_world(cfg, cfg['initial_angle_deg'])
    print(f'\n=== Jednadžba (2): ^W T_D(phi), phi = mu*theta = {np.rad2deg(phi):.1f} deg ===')
    print(T_WD)

    print('\n=== Usporedba ===')
    compare('izračun vs T_D_world_initial (JSON)', T_WD, cfg['T_D_world_initial'])
    T_tf_D = node.lookup('cabinet_D_initial')
    if T_tf_D is not None:
        compare('izračun vs TF cabinet_D_initial', T_WD, T_tf_D)
    T_tf_O = node.lookup('cabinet_origin')
    if T_tf_O is not None:
        compare('^W T_O vs TF cabinet_origin', T_WO, T_tf_O)

    print('\n=== Poza jastučića prsta u svijetu (jednadžba (3)) za nekoliko kutova ===')
    for th in (cfg['initial_angle_deg'], 40.0, 60.0, 80.0):
        T = cu.T_TCP_world(cfg, th)
        print(f'  theta = {th:5.1f} deg: p = {T[:3, 3]}  z_TCP = {T[:3, 2]}')


def main(args=None):
    rclpy.init(args=args)
    node = Task2()
    try:
        deadline = time.monotonic() + 20.0
        while rclpy.ok() and node.cfg is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.cfg is None:
            node.get_logger().error('Nije stigla poruka /cabinet/config (je li simulacija pokrenuta?)')
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
