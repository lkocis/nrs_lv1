#!/usr/bin/env python3
"""Zadatak 3: otvaranje vrata ormarića povlačenjem sa stražnje strane krila.

W_T_TCP(phi) = W_T_D(phi) @ D_T_TCP                 (jedn. (3))
W_T_tool0    = W_T_TCP @ inv(tool0_T_TCP)           (MoveIt grupa završava na tool0)

Slijed: poza prije kontakta (OMPL) -> prilaz do kontakta (Cartesian)
        -> niz kontaktnih poza za rastući phi (Cartesian luk).
"""
import json
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (Constraints, MoveItErrorCodes,
                             OrientationConstraint, PositionConstraint)
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

try:
    from nrs_lv1_tasks.task2_cabinet import T_O_A, T_W_D, T_W_O, quat_to_matrix, rot_z
except ImportError:  # pokretanje izravno iz iste mape kao task2_cabinet.py
    from task2_cabinet import T_O_A, T_W_D, T_W_O, quat_to_matrix, rot_z

np.set_printoptions(precision=4, suppress=True)

GROUP = "ur_manipulator"
TIP_LINK = "tool0"
PLANNING_FRAME = "base_link"
SUCCESS = MoveItErrorCodes.SUCCESS


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


def mat_to_pose(T):
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
    qx, qy, qz, qw = mat_to_quat(T[:3, :3])
    p.orientation.x, p.orientation.y = float(qx), float(qy)
    p.orientation.z, p.orientation.w = float(qz), float(qw)
    return p


def pose_goal(pose, frame, pos_tol=0.002, ori_tol=0.02):
    """Ciljna poza za TIP_LINK kao MoveIt ograničenja (pozicija + orijentacija)."""
    pc = PositionConstraint()
    pc.header.frame_id = frame
    pc.link_name = TIP_LINK
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [pos_tol]
    pc.constraint_region.primitives.append(sphere)
    pc.constraint_region.primitive_poses.append(pose)
    pc.weight = 1.0

    oc = OrientationConstraint()
    oc.header.frame_id = frame
    oc.link_name = TIP_LINK
    oc.orientation = pose.orientation
    oc.absolute_x_axis_tolerance = ori_tol
    oc.absolute_y_axis_tolerance = ori_tol
    oc.absolute_z_axis_tolerance = ori_tol
    oc.weight = 1.0

    c = Constraints()
    c.position_constraints.append(pc)
    c.orientation_constraints.append(oc)
    return c


class Task3(Node):
    def __init__(self):
        super().__init__("task3_open_door")

        # parametri (mijenjaju se s --ros-args -p ime:=vrijednost)
        self.declare_parameter("dphi_deg", 5.0)         # korak kuta vrata
        self.declare_parameter("phi_end_deg", 60.0)     # krajnji kut otvorenosti
        self.declare_parameter("approach_dist", 0.08)   # poza prije kontakta: pomak po +x_D [m]
        self.declare_parameter("tcp_x", -0.015)         # D_T_TCP, translacija u S_D [m]
        self.declare_parameter("tcp_y", 0.155)
        self.declare_parameter("tcp_z", 0.01)
        self.declare_parameter("tcp_roll_deg", 0.0)     # zakret hvata oko osi prsta
        self.declare_parameter("vel_scale", 0.1)
        self.declare_parameter("avoid_collisions", True)
        self.declare_parameter("execute", True)         # False = samo planiraj
        self.declare_parameter("max_attempts", 10)      # broj pokušaja planiranja
        self.declare_parameter("min_fraction", 0.99)    # najmanji prihvatljiv fraction luka

        self.cfg = None
        self.last_js = None   # (imena zglobova, položaji) na kraju zadnje isplanirane putanje
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, "/cabinet/config", self._on_cfg, qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.move_client = ActionClient(self, MoveGroup, "move_action")
        self.exec_client = ActionClient(self, ExecuteTrajectory, "execute_trajectory")
        self.cart_client = self.create_client(GetCartesianPath, "compute_cartesian_path")

    # ---------------- pomoćne metode ----------------
    def _on_cfg(self, msg):
        if self.cfg is None:
            self.cfg = json.loads(msg.data)

    def p(self, name):
        return self.get_parameter(name).value

    def spin_until(self, cond, timeout, what):
        t0 = time.time()
        while rclpy.ok() and not cond():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > timeout:
                raise RuntimeError(f"isteklo vrijeme: {what}")

    def wait_future(self, fut, timeout, what):
        self.spin_until(fut.done, timeout, what)
        return fut.result()

    def lookup(self, target, source, timeout=15.0):
        """Matrica target_T_source iz TF-a."""
        box = {}

        def ok():
            try:
                box["tf"] = self.tf_buffer.lookup_transform(target, source, rclpy.time.Time())
                return True
            except Exception:
                return False

        self.spin_until(ok, timeout, f"TF {target} -> {source}")
        t = box["tf"].transform.translation
        q = box["tf"].transform.rotation
        T = quat_to_matrix(q.x, q.y, q.z, q.w)
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def remember_end(self, robot_traj):
        jt = robot_traj.joint_trajectory
        if jt.points:
            self.last_js = (list(jt.joint_names), list(jt.points[-1].positions))

    def move_to_pose(self, T_base_tool0):
        """Planira (OMPL) i izvršava gibanje do poze tool0 zadane u PLANNING_FRAME."""
        if not self.move_client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("nema akcije /move_action (je li MoveIt pokrenut?)")
        goal = MoveGroup.Goal()
        goal.request.group_name = GROUP
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [pose_goal(mat_to_pose(T_base_tool0), PLANNING_FRAME)]
        goal.planning_options.plan_only = not self.p("execute")

        gh = self.wait_future(self.move_client.send_goal_async(goal), 15.0, "MoveGroup prihvat")
        if not gh.accepted:
            raise RuntimeError("MoveGroup je odbio cilj")
        res = self.wait_future(gh.get_result_async(), 180.0, "MoveGroup rezultat").result
        if res.error_code.val != SUCCESS:
            raise RuntimeError(f"MoveGroup nije uspio, error_code = {res.error_code.val}")
        self.remember_end(res.planned_trajectory)

    def cartesian(self, poses_base):
        """Cartesian putanja kroz niz poza tool0; vraća RobotTrajectory."""
        if not self.cart_client.wait_for_service(timeout_sec=20.0):
            raise RuntimeError("nema servisa /compute_cartesian_path")
        req = GetCartesianPath.Request()
        req.header.frame_id = PLANNING_FRAME
        if self.p("execute") or self.last_js is None:
            req.start_state.is_diff = True            # polazi od stvarnog stanja robota
        else:
            names, pos = self.last_js                 # probno pokretanje: polazi od kraja prethodnog plana
            req.start_state.joint_state.name = names
            req.start_state.joint_state.position = pos
        req.group_name = GROUP
        req.link_name = TIP_LINK
        req.waypoints = [mat_to_pose(T) for T in poses_base]
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = bool(self.p("avoid_collisions"))
        if hasattr(req, "max_velocity_scaling_factor"):
            req.max_velocity_scaling_factor = float(self.p("vel_scale"))
            req.max_acceleration_scaling_factor = float(self.p("vel_scale"))
        res = self.wait_future(self.cart_client.call_async(req), 60.0, "compute_cartesian_path")
        self.get_logger().info(f"Cartesian putanja: fraction = {res.fraction:.3f}")
        if res.error_code.val != SUCCESS or res.fraction < 0.99:
            raise RuntimeError(f"Cartesian putanja nije potpuna (fraction = {res.fraction:.3f}, "
                               f"error_code = {res.error_code.val})")
        self.remember_end(res.solution)
        return res.solution

    def end_state(self, traj):
        """(imena zglobova, položaji) na kraju RobotTrajectory."""
        jt = traj.joint_trajectory
        return list(jt.joint_names), list(jt.points[-1].positions)

    def plan_to_pose(self, T_base_tool0):
        """OMPL plan do poze tool0 (samo planiranje, bez izvršavanja); vraća RobotTrajectory."""
        if not self.move_client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("nema akcije /move_action (je li MoveIt pokrenut?)")
        goal = MoveGroup.Goal()
        goal.request.group_name = GROUP
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.2
        goal.request.max_acceleration_scaling_factor = 0.2
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [pose_goal(mat_to_pose(T_base_tool0), PLANNING_FRAME)]
        goal.planning_options.plan_only = True

        gh = self.wait_future(self.move_client.send_goal_async(goal), 15.0, "MoveGroup prihvat")
        if not gh.accepted:
            raise RuntimeError("MoveGroup je odbio cilj")
        res = self.wait_future(gh.get_result_async(), 180.0, "MoveGroup rezultat").result
        if res.error_code.val != SUCCESS:
            raise RuntimeError(f"MoveGroup nije uspio, error_code = {res.error_code.val}")
        return res.planned_trajectory

    def cartesian_try(self, poses_base, start):
        """Cartesian putanja iz zadanog stanja (imena, položaji); vraća (fraction, RobotTrajectory)."""
        if not self.cart_client.wait_for_service(timeout_sec=20.0):
            raise RuntimeError("nema servisa /compute_cartesian_path")
        req = GetCartesianPath.Request()
        req.header.frame_id = PLANNING_FRAME
        names, pos = start
        req.start_state.joint_state.name = names
        req.start_state.joint_state.position = pos
        req.group_name = GROUP
        req.link_name = TIP_LINK
        req.waypoints = [mat_to_pose(T) for T in poses_base]
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = bool(self.p("avoid_collisions"))
        if hasattr(req, "max_velocity_scaling_factor"):
            req.max_velocity_scaling_factor = float(self.p("vel_scale"))
            req.max_acceleration_scaling_factor = float(self.p("vel_scale"))
        res = self.wait_future(self.cart_client.call_async(req), 60.0, "compute_cartesian_path")
        if res.error_code.val != SUCCESS:
            return 0.0, res.solution
        return float(res.fraction), res.solution

    def execute(self, trajectory):
        if not self.p("execute"):
            self.get_logger().info("execute:=false -> preskačem izvršavanje")
            return
        if not self.exec_client.wait_for_server(timeout_sec=20.0):
            raise RuntimeError("nema akcije /execute_trajectory")
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        gh = self.wait_future(self.exec_client.send_goal_async(goal), 15.0, "Execute prihvat")
        if not gh.accepted:
            raise RuntimeError("ExecuteTrajectory je odbijen")
        res = self.wait_future(gh.get_result_async(), 300.0, "Execute rezultat").result
        if res.error_code.val != SUCCESS:
            raise RuntimeError(f"izvršavanje nije uspjelo, error_code = {res.error_code.val}")

    # ---------------- provjere bez simulacije ----------------
    def sanity_checks(self, cfg, phis, D_T_TCP):
        W_T_A = T_W_O(cfg) @ T_O_A(cfg)
        p0, ax = W_T_A[:3, 3], W_T_A[:3, 2]
        radii = []
        for phi in phis:
            v = (T_W_D(cfg, phi) @ D_T_TCP)[:3, 3] - p0
            v = v - ax * np.dot(v, ax)
            radii.append(np.linalg.norm(v))
        self.get_logger().info(
            f"radijus vrška od osi: min {min(radii):.4f} m, max {max(radii):.4f} m (mora biti isti)")

        # prst mora biti na suprotnoj strani panela od smjera u kojem se krilo giba
        d0, d1 = T_W_D(cfg, phis[0]), T_W_D(cfg, phis[1])
        s = float(np.dot(d1[:3, 3] - d0[:3, 3], d0[:3, 2]))   # gibanje ruba duž z_D
        gura = s * D_T_TCP[2, 3] < 0
        self.get_logger().info(
            f"gibanje ruba duž z_D = {s:+.4f}, prst na z_D = {D_T_TCP[2, 3]:+.3f} "
            f"-> prst gura krilo: {gura}")
        if not gura:
            self.get_logger().warn("Prst je na pogrešnoj strani panela (ili je predznak kuta kriv)!")

    # ---------------- glavni tok ----------------
    def run(self):
        self.spin_until(lambda: self.cfg is not None, 30.0, "/cabinet/config")
        cfg = self.cfg
        inv = np.linalg.inv

        T_base_world = self.lookup(PLANNING_FRAME, "world")
        T_tool0_tcp = self.lookup(TIP_LINK, "finger_contact_tip")
        self.get_logger().info(f"tool0_T_TCP:\n{T_tool0_tcp}")

        # D_T_TCP: vršak prsta u sustavu S_D; prst gleda u -x_D (prilaz sa strane slobodnog ruba)
        R0 = np.array([[0.0, 0.0, -1.0],
                       [0.0, 1.0, 0.0],
                       [1.0, 0.0, 0.0]])
        D_T_TCP = np.eye(4)
        D_T_TCP[:3, :3] = R0 @ rot_z(math.radians(self.p("tcp_roll_deg")))[:3, :3]
        D_T_TCP[:3, 3] = [self.p("tcp_x"), self.p("tcp_y"), self.p("tcp_z")]

        D_T_TCP_pre = D_T_TCP.copy()
        D_T_TCP_pre[0, 3] += self.p("approach_dist")     # +x_D: izvan slobodnog ruba

        # niz kutova
        phi0_deg = cfg["initial_angle_deg"]
        n = max(1, int(round((self.p("phi_end_deg") - phi0_deg) / self.p("dphi_deg"))))
        phis = [math.radians(phi0_deg + k * self.p("dphi_deg")) for k in range(n + 1)]
        self.get_logger().info(
            f"kutovi vrata: {phi0_deg:.1f}° -> {phi0_deg + n * self.p('dphi_deg'):.1f}° ({n} koraka)")

        self.sanity_checks(cfg, phis, D_T_TCP)

        def tool0_in_base(phi, D_T):
            return T_base_world @ T_W_D(cfg, phi) @ D_T @ inv(T_tool0_tcp)

        T_pre = tool0_in_base(phis[0], D_T_TCP_pre)
        T_contacts = [tool0_in_base(phi, D_T_TCP) for phi in phis]

        # planiranje cijelog lanca unaprijed; ponavlja se dok luk ne prođe
        min_f = float(self.p("min_fraction"))
        trajs = None
        best = 0.0
        for attempt in range(1, int(self.p("max_attempts")) + 1):
            try:
                t_pre = self.plan_to_pose(T_pre)
            except RuntimeError as e:
                self.get_logger().warn(f"pokušaj {attempt}: poza prije kontakta nije isplanirana ({e})")
                continue
            f1, t_app = self.cartesian_try([T_contacts[0]], self.end_state(t_pre))
            if f1 < min_f:
                self.get_logger().info(f"pokušaj {attempt}: prilaz fraction = {f1:.3f}, ponavljam")
                continue
            f2, t_arc = self.cartesian_try(T_contacts[1:], self.end_state(t_app))
            best = max(best, f2)
            self.get_logger().info(f"pokušaj {attempt}: prilaz = {f1:.3f}, luk = {f2:.3f}")
            if f2 >= min_f:
                trajs = (t_pre, t_app, t_arc)
                break

        if trajs is None:
            raise RuntimeError(f"nijedan pokušaj nije dao potpun luk (najbolji fraction = {best:.3f}); "
                               f"smanji phi_end_deg ili promijeni tcp_roll_deg")

        names = ("1/3 poza prije kontakta", "2/3 prilaz do kontakta", "3/3 otvaranje vrata")
        for name, traj in zip(names, trajs):
            self.get_logger().info(name)
            self.execute(traj)
        self.get_logger().info("Gotovo.")


def main():
    rclpy.init()
    node = Task3()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(str(e))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()