#!/usr/bin/env python3
"""Zadatak 3: otvaranje vrata ormarića povlačenjem/guranjem s unutarnje (stražnje) strane krila.

Postupak (sve poze u okviru `world`, planira se za tool0 jer je to vrh kinematičkog lanca
grupe ur_manipulator; poza TCP-a `finger_tip` pretvara se u pozu tool0 preko TF-a):

  1. učitaj /cabinet/config, dodaj ormarić (ploče + krilo) u MoveIt planning scene,
  2. T_TCP(theta) = ^W T_D(theta) ^D T_TCP            (jednadžbe (2) i (3))
  3. staging  : ispred otvora, sa strane slobodnog ruba, jastučić `clearance` iza krila  (OMPL)
  4. pre      : klizanje prsta kroz procjep iza krila                                      (Cartesian, s kolizijama)
  5. contact  : primicanje jastučića unutarnjoj plohi krila                                (Cartesian)
  6. sweep    : niz kontaktnih poza T_TCP(theta_k), theta_k = theta_0 ... theta_max        (Cartesian)
  7. retreat  : odmak od krila i povratak u `home`.

Pokretanje:
  ros2 run nrs_lv1_tasks task3_open_door --ros-args -p use_sim_time:=true            # puni postupak
  ros2 run nrs_lv1_tasks task3_open_door --ros-args -p use_sim_time:=true -p check_only:=true
      # samo IK provjera dohvatljivosti poza (bez gibanja), korisno za više položaja ormarića

Parametri: theta_max_deg (80), step_deg (2.5), inset (0.03), psi_deg (30), clearance (0.06),
           vel_scale (0.1; na stvarnom robotu <= 0.2), check_only (false), do_home (true).
"""

import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (BoundingVolume, CollisionObject, Constraints, JointConstraint,
                             MotionPlanRequest, OrientationConstraint, PlanningOptions,
                             PlanningScene, PositionConstraint)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from nrs_lv1_tasks import cabinet_utils as cu

GROUP = 'ur_manipulator'
PLANNING_FRAME = 'world'
TOOL = 'tool0'
TCP = 'finger_tip'
HOME = {'shoulder_pan_joint': 0.0, 'shoulder_lift_joint': -1.5707963267948966,
        'elbow_joint': 0.0, 'wrist_1_joint': -1.5707963267948966,
        'wrist_2_joint': 0.0, 'wrist_3_joint': 0.0}     # grupno stanje "home" iz SRDF-a
MAX_JOINT_JUMP = 0.6    # rad između susjednih točaka Cartesian putanje (detekcija IK preskoka)


def pose_msg(T):
    p = Pose()
    p.position.x, p.position.y, p.position.z = map(float, T[:3, 3])
    q = cu.quat_from_R(T[:3, :3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
    return p


class DoorOpener(Node):
    def __init__(self):
        super().__init__('task3_open_door')
        P = self.declare_parameter
        self.theta_max = P('theta_max_deg', 80.0).value
        self.step = P('step_deg', 2.5).value
        self.inset = P('inset', cu.DEFAULT_INSET).value
        self.psi = np.deg2rad(P('psi_deg', float(np.rad2deg(cu.DEFAULT_PSI))).value)
        self.clearance = P('clearance', 0.06).value
        self.vel_scale = P('vel_scale', 0.1).value
        self.check_only = P('check_only', False).value
        self.do_home = P('do_home', True).value

        self.cfg = None
        self.create_subscription(
            String, '/cabinet/config', self._on_cfg,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.move_client = ActionClient(self, MoveGroup, 'move_action')
        self.exec_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self.cart_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.scene_client = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        self.T_tool_tcp = None

    # ------------------------------------------------------------ pomoćno
    def _on_cfg(self, msg):
        if self.cfg is None:
            self.cfg = cu.parse_config(msg.data)

    def spin_until(self, predicate, timeout, what):
        t0 = time.monotonic()
        while rclpy.ok() and not predicate():
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - t0 > timeout:
                raise RuntimeError(f'Istek vremena: {what}')

    def call(self, client, request, timeout=30.0):
        future = client.call_async(request)
        self.spin_until(future.done, timeout, f'servis {client.srv_name}')
        return future.result()

    def wait_ready(self):
        self.get_logger().info('Čekam /cabinet/config i MoveIt ...')
        self.spin_until(lambda: self.cfg is not None, 30.0, '/cabinet/config')
        for cl in (self.cart_client, self.scene_client, self.ik_client):
            if not cl.wait_for_service(timeout_sec=30.0):
                raise RuntimeError(f'Servis {cl.srv_name} nije dostupan (je li move_group pokrenut?)')
        for ac in (self.move_client, self.exec_client):
            if not ac.wait_for_server(timeout_sec=30.0):
                raise RuntimeError('Akcijski poslužitelj MoveIt-a nije dostupan')
        self.spin_until(lambda: self.tf_buffer.can_transform(TOOL, TCP, rclpy.time.Time()),
                        20.0, f'TF {TOOL}->{TCP} (je li ur5_tooling.xacro ažuriran?)')
        t = self.tf_buffer.lookup_transform(TOOL, TCP, rclpy.time.Time()).transform
        self.T_tool_tcp = cu.make_T(
            cu.R_from_quat([t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w]),
            [t.translation.x, t.translation.y, t.translation.z])
        self.get_logger().info(f'T_tool0_TCP: p = {np.round(self.T_tool_tcp[:3, 3], 4)}')

    def tool_pose_for_tcp(self, T_w_tcp):
        """Poza tool0 koja daje željenu pozu TCP-a: ^W T_tool0 = ^W T_TCP (^tool0 T_TCP)^-1."""
        return T_w_tcp @ cu.inv_T(self.T_tool_tcp)

    def tcp(self, theta, clearance=0.0, shift_x=0.0):
        """^W T_TCP(theta) s odmakom `clearance` iza krila i pomakom `shift_x` duž x_TCP."""
        T = cu.T_TCP_world(self.cfg, theta, self.inset, None, clearance, self.psi)
        if shift_x:
            T = T @ cu.make_T(None, [shift_x, 0, 0])
        return T

    # ------------------------------------------------------------ planning scene
    def collision_objects(self, theta):
        objs = []
        for name, size, T in cu.collision_boxes_world(self.cfg, theta):
            o = CollisionObject()
            o.header.frame_id = PLANNING_FRAME
            o.id = f'cabinet_{name}'
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.BOX
            prim.dimensions = [float(v) for v in size]
            o.primitives = [prim]
            o.primitive_poses = [pose_msg(T)]
            o.operation = CollisionObject.ADD
            objs.append(o)
        return objs

    def apply_scene(self, objs):
        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene()
        req.scene.is_diff = True
        req.scene.world.collision_objects = objs
        if not self.call(self.scene_client, req).success:
            raise RuntimeError('ApplyPlanningScene nije uspio')

    def move_door_object(self, theta):
        door = [o for o in self.collision_objects(theta) if o.id == 'cabinet_door'][0]
        door.operation = CollisionObject.MOVE
        self.apply_scene([door])

    # ------------------------------------------------------------ IK provjera
    def ik_ok(self, T_tool, avoid_collisions):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = GROUP
        r.ik_link_name = TOOL
        r.avoid_collisions = avoid_collisions
        r.pose_stamped.header.frame_id = PLANNING_FRAME
        r.pose_stamped.pose = pose_msg(T_tool)
        r.timeout = Duration(seconds=0.5).to_msg()
        return self.call(self.ik_client, req).error_code.val == 1

    def check_reachability(self):
        thetas = np.arange(self.cfg['initial_angle_deg'], self.theta_max + 1e-6, 5.0)
        self.get_logger().info('theta[deg]  IK(bez kolizija)  IK(s kolizijama, vrata na tom kutu)')
        ok_all = True
        for th in thetas:
            self.move_door_object(th)
            T_tool = self.tool_pose_for_tcp(self.tcp(th))
            a = self.ik_ok(T_tool, False)
            b = self.ik_ok(T_tool, True)       # jastučić dodiruje krilo -> često "kolizija" za 1-2 mm
            ok_all &= a
            print(f'  {th:6.1f}      {"da" if a else "NE":>3s}              {"da" if b else "NE":>3s}')
        self.move_door_object(self.cfg['initial_angle_deg'])
        return ok_all

    # ------------------------------------------------------------ planiranje i izvršavanje
    def plan_to_tool_pose(self, T_tool, label):
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.pipeline_id = 'ompl'
        req.planner_id = 'RRTConnectkConfigDefault'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.vel_scale
        req.start_state.is_diff = True

        pc = PositionConstraint()
        pc.header.frame_id = PLANNING_FRAME
        pc.link_name = TOOL
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.002]
        pc.constraint_region = BoundingVolume(primitives=[sphere],
                                              primitive_poses=[pose_msg(T_tool)])
        pc.weight = 1.0
        oc = OrientationConstraint()
        oc.header.frame_id = PLANNING_FRAME
        oc.link_name = TOOL
        oc.orientation = pose_msg(T_tool).orientation
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = 0.01
        oc.absolute_z_axis_tolerance = 0.01
        oc.weight = 1.0
        c = Constraints()
        c.position_constraints = [pc]
        c.orientation_constraints = [oc]
        req.goal_constraints = [c]
        return self._send_move_goal(req, label)

    def plan_to_joints(self, joints, label):
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.pipeline_id = 'ompl'
        req.planner_id = 'RRTConnectkConfigDefault'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.vel_scale
        req.start_state.is_diff = True
        c = Constraints()
        for name, val in joints.items():
            jc = JointConstraint()
            jc.joint_name, jc.position = name, float(val)
            jc.tolerance_above = jc.tolerance_below = 0.001
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        req.goal_constraints = [c]
        return self._send_move_goal(req, label)

    def _send_move_goal(self, req, label):
        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False          # planiraj i izvrši
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        fut = self.move_client.send_goal_async(goal)
        self.spin_until(fut.done, 30.0, 'prihvat cilja move_action')
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error(f'[{label}] cilj odbijen')
            return False
        res_fut = handle.get_result_async()
        self.spin_until(res_fut.done, 180.0, f'izvršenje [{label}]')
        code = res_fut.result().result.error_code.val
        self.get_logger().info(f'[{label}] MoveIt error_code = {code} ({"OK" if code == 1 else "NEUSPJEH"})')
        return code == 1

    def cartesian(self, tcp_poses, label, avoid_collisions=True, max_step=0.005):
        """Cartesian putanja kroz poze TCP-a; vraća (uspjeh, fraction). Izvršava ono što je izračunato."""
        req = GetCartesianPath.Request()
        req.header.frame_id = PLANNING_FRAME
        req.group_name = GROUP
        req.link_name = TOOL
        req.waypoints = [pose_msg(self.tool_pose_for_tcp(T)) for T in tcp_poses]
        req.max_step = max_step
        req.jump_threshold = 0.0               # preskoke zglobova provjeravamo sami (MAX_JOINT_JUMP)
        req.avoid_collisions = avoid_collisions
        res = self.call(self.cart_client, req)
        traj = res.solution.joint_trajectory
        n = len(traj.points)
        if n < 2:
            self.get_logger().error(f'[{label}] prazna putanja (fraction={res.fraction:.2f})')
            return False, res.fraction
        q = np.array([p.positions for p in traj.points])
        jumps = np.abs(np.diff(q, axis=0)).max(axis=1)
        if jumps.max() > MAX_JOINT_JUMP:
            cut = int(np.argmax(jumps > MAX_JOINT_JUMP)) + 1
            self.get_logger().warning(f'[{label}] preskok IK rješenja ({jumps.max():.2f} rad), '
                                      f'putanja skraćena na {cut}/{n} točaka')
            traj.points = traj.points[:cut]
            res.fraction *= cut / n
            q = q[:cut]
        self.get_logger().info(f'[{label}] Cartesian fraction = {res.fraction:.3f}, točaka = {len(traj.points)}')
        self._retime(traj, q)
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution
        goal.trajectory.joint_trajectory = traj
        fut = self.exec_client.send_goal_async(goal)
        self.spin_until(fut.done, 30.0, 'prihvat execute_trajectory')
        handle = fut.result()
        if not handle.accepted:
            self.get_logger().error(f'[{label}] izvršenje odbijeno')
            return False, res.fraction
        rf = handle.get_result_async()
        self.spin_until(rf.done, 300.0, f'izvršenje [{label}]')
        code = rf.result().result.error_code.val
        return code == 1 and res.fraction > 0.95, res.fraction

    def _retime(self, traj, q):
        """Vremenska parametrizacija: ograničena brzina zglobova (rad/s) skalirana s vel_scale."""
        max_vel = 1.0 * self.vel_scale * 3.0           # ~0.3 rad/s za vel_scale = 0.1
        t = 0.0
        for i, pt in enumerate(traj.points):
            if i > 0:
                t += max(float(np.abs(q[i] - q[i - 1]).max()) / max_vel, 0.02)
            pt.time_from_start = Duration(seconds=t).to_msg()
            pt.velocities = []
            pt.accelerations = []

    # ------------------------------------------------------------ glavni postupak
    def run(self):
        self.wait_ready()
        cfg = self.cfg
        th0 = float(cfg['initial_angle_deg'])
        self.get_logger().info(
            f'Ormarić pose={np.round(cfg["pose"], 3).tolist()}, axis_pos={cfg["axis_pos"]}, theta0={th0} deg')
        self.apply_scene(self.collision_objects(th0))

        if self.check_only:
            ok = self.check_reachability()
            return 0 if ok else 2

        # 1) staging: izvan procjepa (u ravnini krila, `stage_back` od mjesta kontakta), jastučić iza krila
        stage_back = 0.10
        T_pre = self.tcp(th0, clearance=self.clearance)
        T_stage = self.tcp(th0, clearance=self.clearance, shift_x=-stage_back)
        if not self.plan_to_tool_pose(self.tool_pose_for_tcp(T_stage), 'staging (OMPL)'):
            return 3
        # 2) klizanje prsta kroz procjep iza krila (s provjerom kolizija)
        ok, frac = self.cartesian([T_pre], 'pre-contact (klizanje)', True)
        if not ok:
            return 4
        # 3) primicanje jastučića plohi krila (zadnji centimetar bez provjere kolizije s krilom)
        ok, frac = self.cartesian([self.tcp(th0, clearance=0.01)], 'primicanje', True)
        if not ok:
            return 5
        ok, frac = self.cartesian([self.tcp(th0, clearance=0.0)], 'kontakt', False)
        if not ok:
            return 6

        # 4) niz kontaktnih poza za rastući kut (jednadžba (3))
        thetas = np.arange(th0 + self.step, self.theta_max + 1e-9, self.step)
        poses = [self.tcp(th) for th in thetas]
        ok, frac = self.cartesian(poses, f'otvaranje {th0:.0f} -> {self.theta_max:.0f} deg', False)
        reached = th0 + frac * (self.theta_max - th0)
        self.get_logger().info(f'Doseženi kut otvorenosti ~ {reached:.1f} deg '
                               f'({"puni raspon" if ok else "DJELOMIČNO - pogledaj poruke iznad"})')

        # 5) odmak od krila (suprotno od z_TCP), krilo u sceni na doseženi kut, povratak u home
        self.move_door_object(reached)
        T_back = self.tcp(reached, clearance=self.clearance)
        self.cartesian([T_back], 'odmak od krila', False)
        if self.do_home and not self.plan_to_joints(HOME, 'home'):
            self.get_logger().warning('Povratak u home nije uspio - provjeri scenu u RVizu')
        return 0 if ok else 7


def main(args=None):
    rclpy.init(args=args)
    node = DoorOpener()
    code = 1
    try:
        code = node.run()
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
