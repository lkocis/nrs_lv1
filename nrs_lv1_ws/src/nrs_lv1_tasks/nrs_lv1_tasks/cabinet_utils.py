"""Geometrija vrata ormarića (poglavlje III uputa) - samo numpy, bez ROS-a.

Oznake kao u uputama:
  W - svijet, O - ishodište ormarića, A - os zgloba vrata (z_A paralelna osi),
  D - rubna točka krila najudaljenija od osi, TCP - vrh prsta (finger_tip).
  phi = mu * theta, theta >= 0 je kut otvorenosti, mu = axis_pos.
Matrica T_X_Y u kodu znači ^X T_Y (poza sustava Y izražena u sustavu X).
"""

import json

import numpy as np


# ---------------------------------------------------------------- homogene transformacije
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
    return make_T(R.T, -R.T @ p)


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


# ---------------------------------------------------------------- konfiguracija (/cabinet/config)
def parse_config(json_text):
    """JSON s teme /cabinet/config -> rječnik; matrice kao numpy."""
    cfg = json.loads(json_text)
    for key in ('T_D_world_initial', 'T_A_cabinet_closed', 'T_D_hinge_closed'):
        cfg[key] = np.array(cfg[key])
    return cfg


def door_size(cfg):
    """l = [lx, ly, lz] = [debljina, širina, visina] krila."""
    return np.array([cfg['panel_thickness'], cfg['door_width'], cfg['door_height']])


# ---------------------------------------------------------------- jednadžbe (1)-(3)
def T_A_D0(cfg):
    """Jednadžba (1): ^A T_D(0). r = [0, -ly/2] (središte krila na pola širine od osi)."""
    mu = cfg['axis_pos']
    lx, ly, lz = door_size(cfg)
    rx, ry = 0.0, -ly / 2.0
    return np.array([[0, 0, -mu, rx - mu * lx / 2],
                     [mu, 0, 0, ry - ly / 2],
                     [0, -1, 0, lz / 2],
                     [0, 0, 0, 1.0]])


def T_W_O(cfg):
    """^W T_O iz pose = [x, y, z, yaw]."""
    x, y, z, yaw = cfg['pose']
    return make_T(rot_z(yaw)[:3, :3], [x, y, z])


def T_O_A(cfg):
    """^O T_A: os zgloba u okviru ormarića (iz poruke, = Cabinet.T_A_O_init)."""
    return cfg['T_A_cabinet_closed']


def T_W_A(cfg):
    """Os zgloba (vrata zatvorena) u svijetu: ^W T_O ^O T_A."""
    return T_W_O(cfg) @ T_O_A(cfg)


def T_W_D(cfg, theta_deg):
    """Jednadžba (2): ^W T_D(phi) = ^W T_O ^O T_A Rz(phi) ^A T_D(0), phi = mu*theta."""
    phi = cfg['axis_pos'] * np.deg2rad(theta_deg)
    return T_W_A(cfg) @ rot_z(phi) @ T_A_D0(cfg)


# Hvat prsta: prst prelazi preko gornjeg ruba krila i jastučićem pritišće unutarnju plohu.
INSET = 0.035      # m od slobodnog ruba prema osi zgloba
DROP = 0.018       # m ispod gornjeg ruba (rub krila mora pasti u ravni dio jastučića, < 2 cm)


def T_D_TCP(cfg, inset=INSET, drop=DROP, clearance=0.0):
    """^D T_TCP: poza vrha prsta u sustavu D.

    Orijentacija:  x_TCP = +y_D  (prst gleda prema dolje),
                   z_TCP = -z_D  (jastučić gleda prema unutarnjoj plohi krila, gura je van),
                   y_TCP = z_TCP x x_TCP = +x_D.
    Položaj:       mu*inset duž x_D (od ruba prema osi), drop duž y_D (prema dolje),
                   clearance duž z_D (> 0 = odmaknut iza krila, < 0 = pritisak u krilo).
    """
    R = np.array([[0, 1, 0],
                  [1, 0, 0],
                  [0, 0, -1.0]])
    return make_T(R, [cfg['axis_pos'] * inset, drop, clearance])


def T_W_TCP(cfg, theta_deg, inset=INSET, drop=DROP, clearance=0.0):
    """Jednadžba (3): ^W T_TCP(phi) = ^W T_D(phi) ^D T_TCP."""
    return T_W_D(cfg, theta_deg) @ T_D_TCP(cfg, inset, drop, clearance)


# ---------------------------------------------------------------- kolizijski kvadri za MoveIt
def collision_boxes_world(cfg, theta_deg=None):
    """Kvadri ormarića u svijetu: lista (naziv, dimenzije[3], ^W T_kvadar).

    Geometrija je ista kao u Cabinet.generate_cabinet_urdf_from_door_panel;
    krilo je na kutu theta_deg (zadano: početni kut).
    """
    if theta_deg is None:
        theta_deg = cfg['initial_angle_deg']
    d, w, h = cfg['panel_thickness'], cfg['door_width'], cfg['door_height']
    sd = cfg['cabinet_depth']
    m = 0.005                      # razmak krila i nepomičnog dijela
    sw = 0.018                     # debljina bočnih ploča
    W = w + 3 * m + 2 * sw
    H = h + 2 * m + 2 * sw
    panels = [
        ('bottom', [sd, w + 3 * m + 2 * sw, sw], [0, -m / 2, -h / 2 - m - sw / 2]),
        ('side_a', [sd, d, H], [0, w / 2 + m + sw / 2, 0]),
        ('side_b', [sd, d, H], [0, -w / 2 - 2 * m - sw / 2, 0]),
        ('top', [sd, W, d], [0, -m / 2, h / 2 + m + d / 2]),
        ('back', [d, W, H], [-(sd / 2 - d / 2), -m / 2, 0]),
    ]
    out = [(n, np.array(s), T_W_O(cfg) @ make_T(None, p)) for n, s, p in panels]
    phi = cfg['axis_pos'] * np.deg2rad(theta_deg)
    T_door = T_W_A(cfg) @ rot_z(phi) @ make_T(None, [0.0, -w / 2, 0.0])
    out.append(('door', np.array([d, w, h]), T_door))
    return out
