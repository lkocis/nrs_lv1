#!/usr/bin/env python3
"""Zadatak 3, samo račun poza (bez MoveIt-a).

W_T_TCP(phi) = W_T_D(phi) @ D_T_TCP
W_T_tool0    = W_T_TCP @ inv(tool0_T_TCP)
Poze se objavljuju kao TF okviri u 'world' radi provjere u RViz2.
"""
import json
import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

try:
    from nrs_lv1_tasks.task2_cabinet import T_O_A, T_W_D, T_W_O, quat_to_matrix, rot_z
except ImportError:  # pokretanje izravno iz iste mape kao task2_cabinet.py
    from task2_cabinet import T_O_A, T_W_D, T_W_O, quat_to_matrix, rot_z

np.set_printoptions(precision=4, suppress=True)


def mat_to_quat(R):
    """3x3 rotacija -> kvaternion (x, y, z, w)."""
    tr = np.trace(R)
    if tr > 0:
        s = 2.0 * math.sqrt(tr + 1.0)
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s,
                (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s,
                (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s,
            0.25 * s, (R[1, 0] - R[0, 1]) / s)


def make_tf(parent, child, T, stamp):
    m = TransformStamped()
    m.header.stamp = stamp
    m.header.frame_id = parent
    m.child_frame_id = child
    m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = \
        (float(v) for v in T[:3, 3])
    q = mat_to_quat(T[:3, :3])
    m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = \
        (float(v) for v in q)
    return m


class Task3Poses(Node):
    def __init__(self):
        super().__init__("task3_poses")
        self.declare_parameter("dphi_deg", 10.0)        # korak kuta
        self.declare_parameter("phi_end_deg", 60.0)     # krajnji kut otvorenosti
        self.declare_parameter("approach_dist", 0.08)   # pre-kontakt: pomak po +x_D [m]
        self.declare_parameter("tcp_x", -0.015)         # D_T_TCP, translacija u S_D [m]
        self.declare_parameter("tcp_y", 0.155)
        self.declare_parameter("tcp_z", 0.01)
        self.declare_parameter("tcp_roll_deg", 0.0)     # zakret hvata oko osi prsta

        self.cfg = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/cabinet/config", self._on_cfg, qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.broadcaster = StaticTransformBroadcaster(self)

    def _on_cfg(self, msg):
        if self.cfg is None:
            self.cfg = json.loads(msg.data)

    def p(self, name):
        return self.get_parameter(name).value

    def spin_until(self, cond, timeout_s):
        for _ in range(int(timeout_s / 0.1)):
            if cond():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return cond()

    def lookup_tool0_tcp(self):
        """tool0_T_TCP iz TF-a; None ako TF nije dostupan."""
        box = {}

        def ok():
            try:
                box["tf"] = self.tf_buffer.lookup_transform("tool0", "finger_contact_tip",
                                                            rclpy.time.Time())
                return True
            except Exception:
                return False

        if not self.spin_until(ok, 10.0):
            return None
        t, q = box["tf"].transform.translation, box["tf"].transform.rotation
        T = quat_to_matrix(q.x, q.y, q.z, q.w)
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def run(self):
        if not self.spin_until(lambda: self.cfg is not None, 30.0):
            self.get_logger().error("nema poruke na /cabinet/config (radi li simulacija?)")
            return
        cfg = self.cfg

        # ---- D_T_TCP: prst gleda u -x_D (prilaz sa strane slobodnog ruba) ----
        R0 = np.array([[0.0, 0.0, -1.0],
                       [0.0, 1.0, 0.0],
                       [1.0, 0.0, 0.0]])
        D_T_TCP = np.eye(4)
        D_T_TCP[:3, :3] = R0 @ rot_z(math.radians(self.p("tcp_roll_deg")))[:3, :3]
        D_T_TCP[:3, 3] = [self.p("tcp_x"), self.p("tcp_y"), self.p("tcp_z")]
        D_T_TCP_pre = D_T_TCP.copy()
        D_T_TCP_pre[0, 3] += self.p("approach_dist")     # izvan ruba, po +x_D

        # ---- niz kutova ----
        phi0_deg = cfg["initial_angle_deg"]
        n = max(1, int(round((self.p("phi_end_deg") - phi0_deg) / self.p("dphi_deg"))))
        phis = [math.radians(phi0_deg + k * self.p("dphi_deg")) for k in range(n + 1)]

        # ---- provjere ----
        W_T_A = T_W_O(cfg) @ T_O_A(cfg)
        p_axis, axis = W_T_A[:3, 3], W_T_A[:3, 2]
        radii = []
        for phi in phis:
            v = (T_W_D(cfg, phi) @ D_T_TCP)[:3, 3] - p_axis
            v -= axis * np.dot(v, axis)
            radii.append(np.linalg.norm(v))
        print(f"\n[provjera 1] radijus vrška od osi: min {min(radii):.4f}, max {max(radii):.4f} m "
              f"-> {'OK (isti)' if max(radii) - min(radii) < 1e-9 else 'NIJE ISTI'}")

        d0, d1 = T_W_D(cfg, phis[0]), T_W_D(cfg, phis[1])
        s = float(np.dot(d1[:3, 3] - d0[:3, 3], d0[:3, 2]))
        gura = s * D_T_TCP[2, 3] < 0
        print(f"[provjera 2] gibanje ruba duž z_D = {s:+.4f}, prst na z_D = {D_T_TCP[2, 3]:+.3f} "
              f"-> prst gura krilo: {gura}")
        if not gura:
            print("   UPOZORENJE: prst je na pogrešnoj strani panela ili je predznak kuta kriv")

        # ---- poze u svijetu ----
        W_T_tcp_pre = T_W_D(cfg, phis[0]) @ D_T_TCP_pre
        W_T_tcp = [T_W_D(cfg, phi) @ D_T_TCP for phi in phis]

        tool0_T_tcp = self.lookup_tool0_tcp()
        if tool0_T_tcp is None:
            print("\n[upozorenje] TF tool0 -> finger_contact_tip nije dostupan, preskačem poze tool0")
        else:
            print(f"\ntool0_T_TCP:\n{tool0_T_tcp}")
        inv_t = np.linalg.inv(tool0_T_tcp) if tool0_T_tcp is not None else None

        print(f"\nposa prije kontakta, TCP u svijetu:\n{W_T_tcp_pre}")
        print("\nkontaktne poze (položaj TCP-a u svijetu):")
        for k, (phi, T) in enumerate(zip(phis, W_T_tcp)):
            print(f"  k={k:2d}  phi={math.degrees(phi):5.1f}°  xyz = {T[:3, 3]}")

        # ---- objava TF okvira za RViz ----
        stamp = self.get_clock().now().to_msg()
        msgs = [make_tf("world", "tcp_pre", W_T_tcp_pre, stamp)]
        msgs += [make_tf("world", f"tcp_c{k:02d}", T, stamp) for k, T in enumerate(W_T_tcp)]
        if inv_t is not None:
            msgs.append(make_tf("world", "tool0_pre", W_T_tcp_pre @ inv_t, stamp))
            msgs += [make_tf("world", f"tool0_c{k:02d}", T @ inv_t, stamp)
                     for k, T in enumerate(W_T_tcp)]
        self.broadcaster.sendTransform(msgs)
        print(f"\nobjavljeno {len(msgs)} TF okvira (tcp_pre, tcp_c00..., tool0_pre, tool0_c00...). "
              f"Ctrl+C za kraj.")
        rclpy.spin(self)


def main():
    rclpy.init()
    node = Task3Poses()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()