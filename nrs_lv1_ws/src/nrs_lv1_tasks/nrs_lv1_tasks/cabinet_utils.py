"""Geometrija vrata ormarića (poglavlje III uputa) - bez ROS ovisnosti, samo numpy.

Oznake kao u uputama:
  W - svijet, O - ishodište ormarića, A - središte zgloba (z paralelna osi zgloba),
  D - rubna točka na krilu najdalja od osi (z_D = vanjska normala plohe na kojoj leži D),
  phi - SIGNIRANI kut zakreta oko z_A: phi = axis_pos * theta, theta >= 0 je otvorenost vrata.
"""

import json

import numpy as np


# ---------------------------------------------------------------- osnovno
def rot_z(angle):
    c, s = np.cos(angle), np.sin(angle)
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    return T


def make_T(R=None, p=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if p is not None:
        T[:3, 3] = p
    return T


def inv_T(T):
    R, p = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = R.T
    out[:3, 3] = -R.T @ p
    return out


def rot_angle_between(Ra, Rb):
    """Kut (rad) rotacije koja prevodi Ra u Rb."""
    c = (np.trace(Ra.T @ Rb) - 1.0) / 2.0
    return float(np.arccos(np.clip(c, -1.0, 1.0)))


def quat_from_R(R):
    """Kvaternion (x, y, z, w) iz rotacijske matrice."""
    t = np.trace(R)
    if t > 0:
        s = 2.0 * np.sqrt(t + 1.0)
        return np.array([(R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s,
                         (R[1, 0] - R[0, 1]) / s, 0.25 * s])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
    q = np.zeros(4)
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    q[3] = (R[k, j] - R[j, k]) / s
    return q


def R_from_quat(q):
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def T_from_pose_xyzyaw(x, y, z, yaw):
    T = rot_z(yaw)
    T[:3, 3] = [x, y, z]
    return T


# ---------------------------------------------------------------- konfiguracija
def parse_config(json_text):
    """JSON s teme /cabinet/config -> rječnik s numpy matricama."""
    cfg = json.loads(json_text)
    for key in ('T_D_world_initial', 'T_A_cabinet_closed', 'T_D_hinge_closed'):
        cfg[key] = np.array(cfg[key])
    return cfg


def door_size(cfg):
    """l = [lx, ly, lz] = [debljina, širina, visina] vrata."""
    return np.array([cfg['panel_thickness'], cfg['door_width'], cfg['door_height']])


# ---------------------------------------------------------------- jednadžbe (1)-(3)
def T_D_A_closed(cfg):
    """Jednadžba (1): ^A T_D(0).  r = [0, -ly/2] (središte krila), mu = axis_pos."""
    mu = cfg['axis_pos']
    lx, ly, lz = door_size(cfg)
    rx, ry = 0.0, -ly / 2.0
    return np.array([[0, 0, -mu, rx - mu * lx / 2],
                     [mu, 0, 0, ry - ly / 2],
                     [0, -1, 0, lz / 2],
                     [0, 0, 0, 1.0]])


def T_A_O_closed(cfg):
    """^O T_A: položaj osi zgloba u okviru ormarića (kao Cabinet.T_A_O_init)."""
    mu = cfg['axis_pos']
    lx, ly, _ = door_size(cfg)
    T = rot_z(np.pi) if mu < 0 else np.eye(4)
    T[:3, 3] = [cfg['cabinet_depth'] / 2.0 - lx / 2.0, mu * ly / 2.0, 0.0]
    return T


def T_O_W(cfg):
    """^W T_O: ormarić u svijetu iz pose = [x, y, z, yaw]."""
    x, y, z, yaw = cfg['pose']
    return T_from_pose_xyzyaw(x, y, z, yaw)


def phi_from_opening(cfg, theta_deg):
    """Signirani kut oko z_A za otvorenost theta (stupnjevi, >= 0)."""
    return cfg['axis_pos'] * np.deg2rad(theta_deg)


def T_A_world(cfg):
    """Os zgloba (okvir A, vrata zatvorena) u svijetu: ^W T_O ^O T_A."""
    return T_O_W(cfg) @ T_A_O_closed(cfg)


def T_D_world(cfg, theta_deg):
    """Jednadžba (2): ^W T_D(phi) = ^W T_O ^O T_A Rz(phi) ^A T_D(0)."""
    return (T_A_world(cfg) @ rot_z(phi_from_opening(cfg, theta_deg)) @ T_D_A_closed(cfg))


DEFAULT_INSET = 0.03                 # m od slobodnog ruba prema zglobu
DEFAULT_PSI = np.deg2rad(30.0)       # zakret prsta oko normale jastučića (odabrano offline provjerom dohvatljivosti)


def D_T_TCP(cfg, inset=DEFAULT_INSET, drop=None, clearance=0.0, psi=DEFAULT_PSI):
    """^D T_TCP: poza jastučića prsta (okvir finger_tip) u okviru D.

    z_TCP = -z_D  (jastučić pritišće unutarnju plohu krila prema van),
    za psi = 0: x_TCP = smjer prema zglobu (prst leži iza krila, paralelno s njim),
    y_TCP = z_TCP x x_TCP; psi je dodatni zakret oko z_TCP (slobodan stupanj, ne mijenja kontakt).
    Ishodište: `inset` od slobodnog ruba prema zglobu, `drop` ispod vrha krila
    (zadano: pola visine), `clearance` iza plohe (smjer +z_D, u unutrašnjost ormarića).
    """
    mu = cfg['axis_pos']
    if drop is None:
        drop = cfg['door_height'] / 2.0
    R0 = np.array([[mu, 0, 0],
                   [0, -mu, 0],
                   [0, 0, -1.0]])          # stupci: x_TCP, y_TCP, z_TCP izraženi u D
    R = R0 @ rot_z(psi)[:3, :3]
    p = np.array([mu * inset, drop, clearance])
    return make_T(R, p)


def T_TCP_world(cfg, theta_deg, inset=DEFAULT_INSET, drop=None, clearance=0.0, psi=DEFAULT_PSI):
    """Jednadžba (3): ^W T_TCP(phi) = ^W T_D(phi) ^D T_TCP."""
    return T_D_world(cfg, theta_deg) @ D_T_TCP(cfg, inset, drop, clearance, psi)


# ---------------------------------------------------------------- kolizijski kvadri
def collision_boxes_world(cfg, theta_deg=None):
    """Kolizijski kvadri ormarića u svijetu (za MoveIt planning scene).

    Vraća listu (naziv, dimenzije[3], T_W_box). Vrata su na kutu theta_deg
    (zadano: početni kut). Geometrija je ista kao u
    Cabinet.generate_cabinet_urdf_from_door_panel.
    """
    if theta_deg is None:
        theta_deg = cfg['initial_angle_deg']
    d = cfg['panel_thickness']
    w = cfg['door_width']
    h = cfg['door_height']
    sd = cfg['cabinet_depth']
    m = 0.005                      # moving_to_static_part_distance
    sw = 0.018                     # static_side_width
    bottom = sw
    W = w + 2 * (m + sw) + m
    H = h + 2 * m + sw + bottom
    panels = [
        ('bottom', [sd, w + 3 * m + 2 * sw, bottom], [0, -m * 0.5, -h * 0.5 - m - bottom * 0.5]),
        ('side_a', [sd, d, H], [0, w * 0.5 + m + sw * 0.5, 0]),
        ('side_b', [sd, d, H], [0, -w * 0.5 - 2 * m - sw * 0.5, 0]),
        ('top', [sd, W, d], [0, -m * 0.5, h * 0.5 + m + d * 0.5]),
        ('back', [d, W, H], [-(sd / 2 - d / 2), -m * 0.5, 0]),
    ]
    T_WO = T_O_W(cfg)
    out = [(n, np.array(s), T_WO @ make_T(None, p)) for n, s, p in panels]
    T_door = T_A_world(cfg) @ rot_z(phi_from_opening(cfg, theta_deg))
    out.append(('door', np.array([d, w, h]), T_door @ make_T(None, [0.0, -w / 2.0, 0.0])))
    return out
