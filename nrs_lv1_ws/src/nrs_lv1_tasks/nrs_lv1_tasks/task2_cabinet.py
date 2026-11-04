import json

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

np.set_printoptions(precision=4, suppress=True)


# ---------- pomoćne funkcije: svaka je jedna matrica iz formule ----------

def rot_z(angle):
    """4x4 homogena matrica: rotacija oko z za kut u radijanima."""
    c, s = np.cos(angle), np.sin(angle)
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    return T


def translation(x, y, z):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T


def quat_to_matrix(x, y, z, w):
    """Kvaternion (x, y, z, w) -> 4x4 matrica (samo rotacija)."""
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    T = np.eye(4)
    T[:3, :3] = [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]
    return T


def T_W_O(cfg):
    """Ormarić u svijetu, iz poze [x, y, z, yaw]."""
    x, y, z, yaw = cfg["pose"]
    return translation(x, y, z) @ rot_z(yaw)


def T_O_A(cfg):
    """Sustav osi zgloba A u sustavu ormarića O (isto kao T_A_O_init u cabinet_model.py)."""
    mu = cfg["axis_pos"]
    static_d = cfg["cabinet_depth"]
    d_door = cfg["panel_thickness"]
    w_door = cfg["door_width"]
    T = rot_z(np.pi) if mu < 0 else np.eye(4)   # ako je os zgloba na strani axis_pos = -1, sustav A je zakrenut za 180° oko z. Inače je bez rotacije.
    T[:3, 3] = [static_d / 2.0 - d_door / 2.0, mu * w_door / 2.0, 0.0]
    # u smjeru x pola dubine ormarića minus pola debljine vrata, u smjeru y pola širine vrata sa stranom (µ), a z je 0
    return T


def T_A_D_closed(cfg):
    """Matrica (1) iz upute: S_D u odnosu na S_A kad su vrata zatvorena (phi = 0)."""
    mu = cfg["axis_pos"]
    lx = cfg["panel_thickness"]   # debljina
    ly = cfg["door_width"]        # širina
    lz = cfg["door_height"]       # visina
    rx, ry = 0.0, -ly / 2.0       # os zgloba je na rubu krila (vidi cabinet_model.py)
    return np.array([
        [0.0, 0.0, -mu, rx - mu * lx / 2.0],
        [mu, 0.0, 0.0, ry - ly / 2.0],
        [0.0, -1.0, 0.0, lz / 2.0],
        [0.0, 0.0, 0.0, 1.0],
    ])


def T_W_D(cfg, phi):
    """Poza S_D u svijetu pri kutu otvorenosti phi (rad). Za µ = -1 vrata se okreću za -phi."""
    mu = cfg["axis_pos"]
    return T_W_O(cfg) @ T_O_A(cfg) @ rot_z(mu * phi) @ T_A_D_closed(cfg)


def compare(name, mine, ref, atol=1e-5):
    ref = np.array(ref, dtype=float)
    diff = np.abs(mine - ref).max()
    print(f"{name:<32} poklapanje: {np.allclose(mine, ref, atol=atol)}   max|razlika| = {diff:.2e}")


# ---------- ROS čvor ----------

class Task2Cabinet(Node):
    def __init__(self):
        super().__init__("task2_cabinet")
        self.cfg = None
        self.W_T_D_mine = None
        self.done = False

        # /cabinet/config je objavljen jednom, zato transient_local (isto kao u echo naredbi)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(String, "/cabinet/config", self.on_config, qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_timer(0.5, self.check_tf)

    def on_config(self, msg):
        if self.cfg is not None:
            return
        self.cfg = json.loads(msg.data)
        cfg = self.cfg
        phi = np.radians(cfg["initial_angle_deg"])

        W_T_A = T_W_O(cfg) @ T_O_A(cfg)
        self.W_T_D_mine = T_W_D(cfg, phi)

        print("\n=== Os rotacije vrata u svijetu ===")
        print("točka na osi (ishodište S_A):", W_T_A[:3, 3])
        print("smjer osi (z-os S_A):        ", W_T_A[:3, 2])

        print("\n=== Moja W_T_D(phi0) ===")
        print(self.W_T_D_mine)

        print("\n=== Usporedba s porukom /cabinet/config ===")
        compare("O_T_A      vs T_A_cabinet_closed", T_O_A(cfg), cfg["T_A_cabinet_closed"])
        compare("A_T_D(0)   vs T_D_hinge_closed", T_A_D_closed(cfg), cfg["T_D_hinge_closed"])
        compare("W_T_D      vs T_D_world_initial", self.W_T_D_mine, cfg["T_D_world_initial"])

    def check_tf(self):
        if self.W_T_D_mine is None or self.done:
            return
        try:
            tf = self.tf_buffer.lookup_transform("world", "cabinet_D_initial", rclpy.time.Time())
        except Exception as e:  # TF još nije stigao
            self.get_logger().warn(f"čekam TF world -> cabinet_D_initial: {e}", throttle_duration_sec=5.0)
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        W_T_D_tf = quat_to_matrix(q.x, q.y, q.z, q.w)
        W_T_D_tf[:3, 3] = [t.x, t.y, t.z]

        print("\n=== Usporedba s TF-om (world -> cabinet_D_initial) ===")
        print(W_T_D_tf)
        compare("W_T_D      vs TF", self.W_T_D_mine, W_T_D_tf)
        self.done = True


def main():
    rclpy.init()
    node = Task2Cabinet()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()