"""Cabinet placement using the generated model's collision boxes."""

import random
import xml.etree.ElementTree as ET

import numpy as np
from tf_transformations import euler_matrix


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    result = euler_matrix(*rpy)
    result[:3, 3] = xyz
    return result


def origin(element):
    if element is None:
        return np.eye(4)
    return transform([float(x) for x in element.get('xyz', '0 0 0').split()],
                     [float(x) for x in element.get('rpy', '0 0 0').split()])


def collision_corners(urdf):
    """Return box corners in the cabinet frame at its initial opening."""
    tree = ET.fromstring(urdf)
    poses = {'world': np.eye(4)}
    pending = list(tree.findall('joint'))
    while pending:
        ready = [j for j in pending if j.find('parent').get('link') in poses]
        if not ready:
            raise ValueError('Cabinet has a disconnected joint tree')
        for joint in ready:
            poses[joint.find('child').get('link')] = (
                poses[joint.find('parent').get('link')] @ origin(joint.find('origin')))
            pending.remove(joint)
    points = []
    for link in tree.findall('link'):
        for collision in link.findall('collision'):
            box = collision.find('geometry/box')
            if box is None:
                raise ValueError('Placement check expects a cabinet without a handle')
            size = np.array([float(v) for v in box.get('size').split()])
            pose = poses[link.get('name')] @ origin(collision.find('origin'))
            for x in (-1, 1):
                for y in (-1, 1):
                    for z in (-1, 1):
                        points.append((pose @ np.r_[size * [x, y, z] / 2, 1])[:3])
    return np.asarray(points)


def sample_pose(corners, config):
    rng = random.Random(None if config['seed'] < 0 else config['seed'])
    xmin = -config['table_width'] / 2 + config['table_margin']
    xmax = config['table_width'] / 2 - config['table_margin']
    ymin = config['table_edge_offset'] - config['table_length'] + config['table_margin']
    ymax = config['table_edge_offset'] - config['table_margin']
    for _ in range(1000):
        jitter = config['position_jitter']
        x = config['nominal_x'] + rng.uniform(-jitter, jitter)
        y = config['nominal_y'] + rng.uniform(-jitter, jitter)
        yaw = config['nominal_yaw'] + rng.uniform(-config['yaw_jitter'], config['yaw_jitter'])
        z = config['table_top_z'] - float(corners[:, 2].min())
        pose = transform((x, y, z), (0, 0, yaw))
        placed = (pose @ np.c_[corners, np.ones(len(corners))].T).T
        if (placed[:, 0].min() >= xmin and placed[:, 0].max() <= xmax
                and placed[:, 1].min() >= ymin and placed[:, 1].max() <= ymax):
            return [x, y, z, yaw]
    raise ValueError('No cabinet placement fits the table; check dimensions and nominal pose')
