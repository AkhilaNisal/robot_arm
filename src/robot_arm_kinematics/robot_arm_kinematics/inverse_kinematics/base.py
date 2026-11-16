from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass
class IKResult:
    """Result returned by an inverse kinematics solver."""

    joint_positions: np.ndarray
    success: bool
    iterations: int
    position_error: float
    orientation_error: float
    message: str


def validate_target_pose(target_pose):
    """Return a finite rigid homogeneous target transform."""
    try:
        pose = np.asarray(target_pose, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError('Target pose must be a numeric 4x4 matrix.') from error

    if pose.shape != (4, 4):
        raise ValueError('Target pose must be a 4x4 matrix.')
    if not np.all(np.isfinite(pose)):
        raise ValueError('Target pose must contain finite values.')
    if not np.allclose(pose[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
        raise ValueError('Target pose must be homogeneous.')

    rotation = pose[:3, :3]
    if (
        not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
        or not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-6)
    ):
        raise ValueError('Target orientation must be a valid rotation matrix.')
    return pose


class IKSolver(ABC):
    """Define the common interface for inverse kinematics algorithms."""

    def __init__(
        self,
        robot,
        position_tolerance=1e-4,
        orientation_tolerance=1e-3,
        max_iterations=1000
    ):

        self.robot = robot
        self.position_tolerance = position_tolerance
        self.orientation_tolerance = orientation_tolerance
        self.max_iterations = max_iterations

    @abstractmethod
    def solve(self, target_pose, q_initial):
        """
        Solve inverse kinematics.

        target_pose: Desired 4x4 transformation.
        q_initial: Initial joint configuration in radians.
        """
        pass
