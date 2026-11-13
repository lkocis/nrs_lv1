#!/usr/bin/env python3
"""Zadatak 3: otvaranje vrata ormarića povlačenjem s unutarnje strane krila.

Slijed:
  1. ormarić (nepomični dio + krilo na početnom kutu) doda se u MoveIt planning scenu,
  2. poze:  pre      - iznad gornjeg ruba, iza ravnine krila        (poza prije kontakta)
            gap      - prst spušten iza krila, još bez dodira
            contact  - jastučić na unutarnjoj plohi, theta = theta0   (poza pri početnom kontaktu)
            pull[k]  - kontaktne poze za theta = theta0 + k*step ... theta_max
            release, up - odmak od krila i podizanje prsta,
  3. PRIJE gibanja: IK za 'gap' (s provjerom kolizija) i 'pre' iz iste konfiguracije ruke, pa
     Cartesian putanje (compute_cartesian_path) pre->gap, gap->contact, contact->pull, ->up.
     Ako neka ne uspije, pokuša se s drugim početnim stanjem za IK (druga konfiguracija ruke),
  4. izvrši: OMPL do 'pre' -> spuštanje -> kontakt -> povlačenje -> odmak -> OMPL home.

Krilo je u sceni dok se prst spušta iza njega; za kontakt i povlačenje krilo se uklanja iz
scene (prst ga namjerno dodiruje), a nepomični dio ormarića i stol i dalje se provjeravaju.

Pokretanje (simulacija mora raditi):
  ros2 run nrs_lv1_tasks task3_open_door --ros-args -p use_sim_time:=true
  ros2 run nrs_lv1_tasks task3_open_door --ros-args -p use_sim_time:=true -p check_only:=true
      # samo planiranje i prikaz putanje u RVizu, bez gibanja
"""

import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (CollisionObject, Constraints, DisplayTrajectory, JointConstraint,
                             MotionPlanRequest, PlanningOptions, PlanningScene, RobotState,
                             RobotTrajectory)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionIK
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from nrs_lv1_tasks import cabinet_utils as cu

GROUP = 'ur_manipulator_tcp'      # base_link -> finger_tip 
TCP = 'finger_tip'                # link čiju pozu zadajemo
FRAME = 'world'
ARM_JOINTS = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
HOME = [0.0, -np.pi / 2, 0.0, -np.pi / 2, 0.0, 0.0]          # SRDF "home"
JOINT_LIMIT = np.array([2 * np.pi, 2 * np.pi, np.pi, 2 * np.pi, 2 * np.pi, 2 * np.pi]) - 0.15
MAX_STEP = 0.005          # Cartesian putanja u koracima od 5 mm
MAX_JUMP = 0.3            # najveći dopušteni skok zgloba između dvije točke (rad)
UR_MAX_VEL = 3.14         # najveća brzina zgloba UR5 (rad/s)


def pose_msg(T):
    p = Pose()
    p.position.x, p.position.y, p.position.z = map(float, T[:3, 3])
    q = cu.quat_from_R(T[:3, :3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = map(float, q)
    return p
# MoveIt servisi primaju poze kao Pose, a mi računamo s matricama, pa je ovo pretvorba (obrnuto od tf_to_matrix u zadatku 2).

def robot_state(q):
    s = RobotState()
    s.is_diff = True               # ostali zglobovi (šaka) ostaju kao u trenutnom stanju
    s.joint_state.name = list(ARM_JOINTS)
    s.joint_state.position = [float(v) for v in q]
    return s


def nearest_turn(q, q_ref): # ispravlja zglobove pomaknute za puni krug (pomak zglobova za k*2pi)
    q = np.array(q, float)
    for j in range(6):
        cands = [q[j] + 2 * np.pi * k for k in (-2, -1, 0, 1, 2)]
        cands = [c for c in cands if abs(c) <= JOINT_LIMIT[j]]
        if cands:
            q[j] = min(cands, key=lambda c: abs(c - q_ref[j]))
    return q


class DoorOpener(Node):
    def __init__(self):
        super().__init__('task3_open_door')
        P = self.declare_parameter
        self.theta_max = P('theta_max_deg', 80.0).value   # do kojeg kuta se vrata otvaraju
        self.step = P('step_deg', 2.5).value              # razmak kontaktnih poza
        self.pre_height = P('pre_height', 0.08).value     # m iznad gornjeg ruba krila
        self.gap = P('gap', 0.03).value                   # m iza krila pri spuštanju i odmaku
        self.press = P('press', 0.002).value              # m pritiska u krilo
        self.vel_scale = P('vel_scale', 0.1).value        # 10 % najveće brzine robota
        self.contact_speed = P('contact_speed', 0.02).value   # brzina prsta uz krilo
        self.attempts = P('ik_attempts', 15).value        # koliko nasumičnih pokušaja IK-a
        self.check_only = P('check_only', False).value    # samo planira, bez gibanja

        self.cfg = None
        self.q_now = None
        self.create_subscription(
            String, '/cabinet/config', self._on_cfg,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.move_client = ActionClient(self, MoveGroup, 'move_action')
        self.exec_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')
        self.scene_client = self.create_client(ApplyPlanningScene, 'apply_planning_scene')
        self.ik_client = self.create_client(GetPositionIK, 'compute_ik')
        self.cart_client = self.create_client(GetCartesianPath, 'compute_cartesian_path')
        self.display_pub = self.create_publisher(DisplayTrajectory, '/display_planned_path', 1)
        self.rng = np.random.default_rng(0)

    # ------------------------------------------------------------ ROS pomoćno
    def _on_cfg(self, msg): # poruka s /cabinet/config
        if self.cfg is None:
            self.cfg = cu.parse_config(msg.data)

    def _on_js(self, msg): # poruka s /joint_states
        pos = dict(zip(msg.name, msg.position))
        if all(j in pos for j in ARM_JOINTS):
            self.q_now = np.array([pos[j] for j in ARM_JOINTS]) # Napravi rječnik ime → kut i iz njega uzme 6 zglobova ruke, redom kao u ARM_JOINTS.

    def spin_until(self, done, timeout, what): # Koristi se za čekanje na sve: poruke, servise, akcije.
        t0 = time.monotonic()
        while rclpy.ok() and not done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.monotonic() - t0 > timeout:
                raise RuntimeError(f'Istek vremena: {what}')

    def call(self, client, req, timeout=30.0): # Pozove servis i čeka odgovor
        fut = client.call_async(req)
        self.spin_until(fut.done, timeout, client.srv_name)
        return fut.result()

    def current_q(self): # Pola sekunde vrti čvor, da stigne najnovija poruka /joint_states, i vrati trenutne kutove zglobova.
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.q_now.copy()

    def wait_ready(self): # Prije početka čeka: konfiguraciju ormarića, stanje zglobova, tri MoveIt servisa, dvije MoveIt akcije.
        self.get_logger().info('Čekam /cabinet/config, /joint_states i MoveIt ...')
        self.spin_until(lambda: self.cfg is not None, 30.0, '/cabinet/config')
        self.spin_until(lambda: self.q_now is not None, 30.0, '/joint_states')
        for cl in (self.scene_client, self.ik_client, self.cart_client):
            if not cl.wait_for_service(timeout_sec=30.0):
                raise RuntimeError(f'Servis {cl.srv_name} nije dostupan')
        for ac in (self.move_client, self.exec_client):
            if not ac.wait_for_server(timeout_sec=30.0):
                raise RuntimeError('Akcije MoveIt-a nisu dostupne')

    # ------------------------------------------------------------ poze (jednadžba (3))
    def tcp(self, theta, drop=cu.DROP, clearance=0.0): # Ulazi su kut vrata, visina u odnosu na gornji rub (drop) i odmak od plohe (clearance)
        return cu.T_W_TCP(self.cfg, theta, cu.INSET, drop, clearance) # Poza vrha prsta u svijetu

    # ------------------------------------------------------------ planning scene
    def set_scene(self, theta, with_door=True):
        objs = []
        for name, size, T in cu.collision_boxes_world(self.cfg, theta):
            o = CollisionObject()
            o.header.frame_id = FRAME
            o.id = f'cabinet_{name}'
            if name == 'door' and not with_door:
                o.operation = CollisionObject.REMOVE
            else:
                prim = SolidPrimitive(type=SolidPrimitive.BOX,
                                      dimensions=[float(v) for v in size])
                o.primitives = [prim]
                o.pose = pose_msg(T)
                o.primitive_poses = [Pose()]
                o.operation = CollisionObject.ADD       # Ako objekt s tim imenom već postoji, zamijeni se, pa se tako krilo “pomiče” na novi kut.
            objs.append(o)
        req = ApplyPlanningScene.Request()
        req.scene = PlanningScene(is_diff=True)
        req.scene.world.collision_objects = objs
        if not self.call(self.scene_client, req).success:
            raise RuntimeError('ApplyPlanningScene nije uspio')

    # ------------------------------------------------------------ MoveIt servisi
    def ik(self, T, seed): # inverzna kinematika
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = GROUP
        r.ik_link_name = TCP
        r.avoid_collisions = True
        r.pose_stamped.header.frame_id = FRAME
        r.pose_stamped.pose = pose_msg(T) # željena poza prsta
        r.robot_state = robot_state(seed) # Seed odakle solver kreće
        r.timeout = Duration(seconds=0.1).to_msg()
        res = self.call(self.ik_client, req)
        if res.error_code.val != 1:
            return None
        pos = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return np.array([pos[j] for j in ARM_JOINTS])

    def cartesian(self, q_start, poses):
        """Cartesian putanja TCP-a kroz poze iz stanja q_start -> (niz q, fraction)."""
        # MoveIt vodi prst ravnom linijom od trenutne pozicije do prve poze, pa do druge, …, 
        # u koracima od 5 mm, i za svaki korak računa IK (sjeme je prethodni korak, pa ruka ostaje u istoj konfiguraciji).
        req = GetCartesianPath.Request()
        req.header.frame_id = FRAME
        req.start_state = robot_state(q_start)
        req.group_name = GROUP
        req.link_name = TCP
        req.waypoints = [pose_msg(T) for T in poses]
        req.max_step = MAX_STEP
        req.jump_threshold = 0.0            # skokove provjeravamo sami (MAX_JUMP)
        req.avoid_collisions = True
        res = self.call(self.cart_client, req)
        qs = [np.array(p.positions) for p in res.solution.joint_trajectory.points] # niz kutova zglobova, po jedan za svaku točku putanje.
        names = list(res.solution.joint_trajectory.joint_names)
        if not qs:
            return [np.array(q_start)], 0.0
        idx = [names.index(j) for j in ARM_JOINTS]
        qs = [q[idx] for q in qs]
        fraction = res.fraction # koji dio puta je uspio (1.0 = cijeli)
        for i in range(1, len(qs)):                      # odreži na prvom skoku
            if np.abs(qs[i] - qs[i - 1]).max() > MAX_JUMP:
                fraction *= i / len(qs)
                qs = qs[:i]
                break
        # Ako se neki zglob između dvije točke (5 mm pomaka prsta) promijeni više od 0.3 rad, IK je “preskočio” u drugu konfiguraciju ruke. 
        # To bi bio nagli i opasan pokret, pa se putanja tu odreže i fraction se smanji.
        return qs, fraction

    # ------------------------------------------------------------ planiranje
    def plan(self):
        """Isplanira sve dijelove gibanja za jednu konfiguraciju ruke; vraća rječnik ili None."""
        th0 = float(self.cfg['initial_angle_deg'])
        thetas = np.arange(th0 + self.step, self.theta_max + 1e-9, self.step)
        T_pre = self.tcp(th0, drop=-self.pre_height, clearance=self.gap) # poza prije kontakta
        T_gap = self.tcp(th0, clearance=self.gap) # prst iza krila
        T_contact = self.tcp(th0, clearance=-self.press)
        pulls = [self.tcp(th, clearance=-self.press) for th in thetas] # niz kontaktnih poza

        q_ref = self.current_q()
        best = None
        seeds = [q_ref] + [self.rng.uniform(-np.pi, np.pi, 6) for _ in range(self.attempts)]
        for n, seed in enumerate(seeds):
            self.set_scene(th0, with_door=True)
            q_gap = self.ik(T_gap, seed)                  # za pozu prst iza krila, odabire konfig. ruke
            if q_gap is None:
                continue
            q_gap = nearest_turn(q_gap, q_ref) # ispravak 2pi
            q_pre = self.ik(T_pre, q_gap)                 # ik za pozu prije kontakta, ista konfiguracija ruke
            if q_pre is None or np.abs(q_pre - q_gap).max() > 1.0:
                continue
            # Cartesian za svaki dio pokreta (spuštanje, dodir, povlačenje, odmak)
            down, f_down = self.cartesian(q_pre, [T_gap])
            if f_down < 0.999:
                continue
            self.set_scene(th0, with_door=False)          
            touch, f_touch = self.cartesian(down[-1], [T_contact])
            if f_touch < 0.999:
                continue
            pull, f_pull = self.cartesian(touch[-1], pulls)
            n_ok = int(np.floor(f_pull * len(pulls) + 1e-6))
            th_end = thetas[n_ok - 1] if n_ok > 0 else th0
            out, f_out = self.cartesian(pull[-1], [
                self.tcp(th_end, clearance=self.gap),
                self.tcp(th_end, drop=-self.pre_height, clearance=self.gap)])
            self.get_logger().info(
                f'  pokušaj {n}: povlačenje do {th_end:.1f} deg, odmak {100 * f_out:.0f} %')
            if best is None or th_end > best['theta_end']:
                best = dict(q_pre=q_pre, down=down, touch=touch, pull=pull, out=out,
                            theta_end=th_end)
            if th_end >= self.theta_max - 1e-6 and f_out > 0.999:
                break
        self.set_scene(th0, with_door=True)               # scena natrag na stvarno stanje
        return best

    # ------------------------------------------------------------ izvršavanje
    def move_to_joints(self, q, label): # koristi MoveIt akciju move_action
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.pipeline_id = 'ompl'
        req.planner_id = 'RRTConnectkConfigDefault'
        req.num_planning_attempts = 4
        req.allowed_planning_time = 15.0
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.vel_scale
        req.start_state.is_diff = True
        c = Constraints()
        for name, val in zip(ARM_JOINTS, q):
            c.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(val),
                tolerance_above=0.001, tolerance_below=0.001, weight=1.0))
        req.goal_constraints = [c]
        goal = MoveGroup.Goal(request=req, planning_options=PlanningOptions())
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        fut = self.move_client.send_goal_async(goal)
        self.spin_until(fut.done, 30.0, 'move_action')
        res_fut = fut.result().get_result_async()
        self.spin_until(res_fut.done, 300.0, label)
        code = res_fut.result().result.error_code.val
        status = 'OK' if code == 1 else 'NEUSPJEH'
        self.get_logger().info(f'[{label}] error_code = {code} ({status})')
        return code == 1

    def trajectory(self, qs, dt_min): # točkama putanje dodijeli vremena i brzine
        qs = np.asarray(qs)
        t = [0.0]
        for i in range(1, len(qs)):
            dq = float(np.abs(qs[i] - qs[i - 1]).max())
            t.append(t[-1] + max(dq / (UR_MAX_VEL * self.vel_scale), dt_min))
        t = np.array(t)
        v = np.zeros_like(qs)
        if len(qs) > 2:
            v[1:-1] = (qs[2:] - qs[:-2]) / (t[2:] - t[:-2])[:, None]
        jt = JointTrajectory(joint_names=list(ARM_JOINTS))
        for qi, vi, ti in zip(qs, v, t):
            jt.points.append(JointTrajectoryPoint(
                positions=[float(x) for x in qi], velocities=[float(x) for x in vi],
                time_from_start=Duration(seconds=float(ti)).to_msg()))
        return jt

    def execute(self, qs, label): # koristi akciju execute_trajectory
        if len(qs) < 2:
            return True
        q_now = self.current_q()
        if np.abs(q_now - qs[0]).max() > 0.05:
            self.get_logger().error(f'[{label}] robot nije na početku putanje')
            return False
        qs = [q_now] + list(qs[1:])
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = RobotTrajectory(
            joint_trajectory=self.trajectory(qs, MAX_STEP / self.contact_speed))
        fut = self.exec_client.send_goal_async(goal)
        self.spin_until(fut.done, 30.0, 'execute_trajectory')
        res_fut = fut.result().get_result_async()
        self.spin_until(res_fut.done, 600.0, label)
        code = res_fut.result().result.error_code.val
        self.get_logger().info(f'[{label}] {len(qs)} točaka, error_code = {code}')
        return code == 1

    def display(self, p):
        qs = p['down'] + p['touch'][1:] + p['pull'][1:] + p['out'][1:]
        msg = DisplayTrajectory(trajectory_start=robot_state(qs[0]))
        msg.trajectory = [RobotTrajectory(joint_trajectory=self.trajectory(qs, 0.05))]
        self.display_pub.publish(msg)

    # ------------------------------------------------------------ glavni postupak
    def run(self):
        self.wait_ready()
        th0 = float(self.cfg['initial_angle_deg'])
        self.get_logger().info(f'Ormarić pose = {np.round(self.cfg["pose"], 3).tolist()}, '
                               f'axis_pos = {self.cfg["axis_pos"]}, theta0 = {th0} deg')
        np.set_printoptions(precision=4, suppress=True)
        self.get_logger().info(f'^D T_TCP =\n{cu.T_D_TCP(self.cfg)}')
        self.get_logger().info(f'^W T_TCP (početni kontakt) =\n{self.tcp(th0)}')

        p = self.plan()
        if p is None:
            self.get_logger().error('Nema izvedivog plana (IK/kolizija) za ovaj položaj ormarića')
            return 2
        self.get_logger().info(f'Plan: otvaranje do {p["theta_end"]:.1f} deg, '
                               f'q_pre = {np.round(p["q_pre"], 3).tolist()}')
        self.display(p)
        if self.check_only:
            return 0

        # Iz home se slobodno kreće do položaja iznad gornjeg ruba vrata
        if not self.move_to_joints(p['q_pre'], '1 OMPL do poze prije kontakta'):
            return 3
        # prst se spušta ravno dolje iza krila
        if not self.execute(p['down'], '2 spuštanje prsta iza krila'):
            return 4
        # prst se primakne krilu i dodirne ga s unutarnje strane
        self.set_scene(th0, with_door=False)
        if not self.execute(p['touch'], '3 kontakt s unutarnjom plohom'):
            return 5
        # prst gura krilo po luku vrata
        if not self.execute(p['pull'], f'4 povlačenje {th0:.0f} -> {p["theta_end"]:.0f} deg'):
            return 6
        # prst se odmakne od krila i podigne iznad njega
        if not self.execute(p['out'], '5 odmak i podizanje prsta'):
            return 7
        # robot se vraća u home zaobilazeći otvorena vrata
        self.set_scene(p['theta_end'], with_door=True)
        self.move_to_joints(HOME, '6 OMPL home')
        self.get_logger().info(f'Gotovo: vrata otvorena na ~{p["theta_end"]:.1f} deg')
        return 0


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
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
