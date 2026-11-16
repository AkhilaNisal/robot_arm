"""Numerical checks for Cartesian position and orientation interpolation."""

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from robot_arm_trajectory.cartesian_space.cartesian_path import (
    generate_linear_cartesian_path,
    quaternion_slerp,
    quaternion_to_rotation,
)


def test_cartesian_path_has_linear_position_and_exact_endpoints():
    """Every path point lies on the segment and includes both endpoints."""
    start = np.eye(4)
    goal = np.eye(4)
    start[:3, 3] = [0.2, -0.1, 0.4]
    goal[:3, 3] = [0.8, 0.3, -0.2]
    goal[:3, :3] = Rotation.from_rotvec([0.0, 0.0, 1.2]).as_matrix()

    path = generate_linear_cartesian_path(start, goal, num_waypoints=7)

    assert len(path) == 7
    assert np.allclose(path[0], start)
    assert np.allclose(path[-1], goal)
    for index, pose in enumerate(path):
        fraction = index / 6
        expected = start[:3, 3] + fraction * (goal[:3, 3] - start[:3, 3])
        assert np.allclose(pose[:3, 3], expected)
        assert np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3))


@pytest.mark.parametrize(
    'angle', [0.0, 1e-8, np.pi - 1e-9, np.pi]
)
def test_cartesian_orientation_path_is_finite_and_endpoint_correct(angle):
    """SLERP handles identical, small, and near-180-degree rotations."""
    start = np.eye(4)
    goal = np.eye(4)
    goal[:3, :3] = Rotation.from_rotvec([angle, 0.0, 0.0]).as_matrix()

    path = generate_linear_cartesian_path(start, goal, num_waypoints=9)

    assert all(np.isfinite(pose).all() for pose in path)
    assert np.allclose(path[0][:3, :3], start[:3, :3])
    assert np.allclose(path[-1][:3, :3], goal[:3, :3], atol=1e-12)
    for left, right in zip(path[:-1], path[1:]):
        relative = left[:3, :3].T @ right[:3, :3]
        assert np.arccos(np.clip((np.trace(relative) - 1) / 2, -1, 1)) < np.pi


def test_slerp_treats_opposite_quaternions_as_the_same_orientation():
    """Equivalent quaternion signs take the shortest zero-length path."""
    quaternion = np.array([0.0, 0.0, 0.0, 1.0])

    for fraction in np.linspace(0.0, 1.0, 11):
        interpolated = quaternion_slerp(
            quaternion, -quaternion, fraction
        )
        assert np.allclose(
            quaternion_to_rotation(interpolated), np.eye(3), atol=1e-12
        )


def test_cartesian_path_rejects_invalid_pose_and_waypoint_count():
    """Non-rigid transforms and undersized paths fail before interpolation."""
    invalid = np.eye(4)
    invalid[0, 0] = 2.0
    with pytest.raises(ValueError, match='orthonormal'):
        generate_linear_cartesian_path(np.eye(4), invalid)
    with pytest.raises(ValueError, match='At least two'):
        generate_linear_cartesian_path(np.eye(4), np.eye(4), 1)
