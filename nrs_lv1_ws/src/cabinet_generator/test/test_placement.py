import tempfile
from pathlib import Path

import numpy as np
import pytest

from cabinet_generator.cabinet_model import Cabinet
from cabinet_generator.placement import collision_corners, sample_pose, transform


def model(directory, axis=-1):
    path = Path(directory) / 'cabinet.urdf'
    Cabinet(door_params=np.array([0.28, 0.35, 0.018, 0.30]),
            r=np.array([0.0, -0.14]), axis_pos=axis,
            initial_angle_deg=30, save_path=str(path))
    return collision_corners(path.read_text())


def config(seed):
    return dict(seed=seed, table_width=0.9, table_length=1.5,
                table_edge_offset=0.21, table_margin=0.04, table_top_z=0.0,
                nominal_x=0.0, nominal_y=-0.45, nominal_yaw=np.pi / 2,
                position_jitter=0.035, yaw_jitter=0.15)


@pytest.mark.parametrize('axis', [-1, 1])
def test_random_placements_fit_initial_open_door(axis):
    with tempfile.TemporaryDirectory() as directory:
        corners = model(directory, axis)
        for seed in range(100):
            cfg = config(seed)
            x, y, z, yaw = sample_pose(corners, cfg)
            points = (transform((x, y, z), (0, 0, yaw)) @
                      np.c_[corners, np.ones(len(corners))].T).T
            assert points[:, 0].min() >= -0.41
            assert points[:, 0].max() <= 0.41
            assert points[:, 1].min() >= -1.25
            assert points[:, 1].max() <= 0.17
            assert abs(points[:, 2].min()) < 1e-12
            assert sample_pose(corners, cfg) == [x, y, z, yaw]


def test_impossible_placement_is_rejected():
    with tempfile.TemporaryDirectory() as directory:
        cfg = config(1)
        cfg['table_width'] = 0.05
        with pytest.raises(ValueError, match='No cabinet placement'):
            sample_pose(model(directory), cfg)
