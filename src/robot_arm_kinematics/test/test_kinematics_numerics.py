"""Numerical regression tests for robot kinematics and Cartesian IK."""

from pathlib import Path

import numpy as np
import pytest

from robot_arm_kinematics.forward_kinematics import forward_kinematics
from robot_arm_kinematics.inverse_kinematics.damped_least_squares import (
    DampedLeastSquaresIK,
    rotation_error,
)
from robot_arm_kinematics.inverse_kinematics.jacobian_transpose import (
    JacobianTransposeIK,
)
from robot_arm_kinematics.inverse_kinematics.pseudoinverse import (
    PseudoinverseIK,
)
from robot_arm_kinematics.jacobian import calculate_jacobian
from robot_arm_kinematics.robot_model import RobotModel
from robot_arm_trajectory.cartesian_space.cartesian_ik import (
    solve_cartesian_path,
)
from robot_arm_trajectory.cartesian_space.cartesian_path import (
    generate_linear_cartesian_path,
)


def _robot():
    """Load the generic six-axis robot model used by script validation."""
    config = (
        Path(__file__).parents[1]
        / 'robot_arm_kinematics'
        / 'config'
        / 'generic_robot.yaml'
    )
    return RobotModel(str(config))


def test_geometric_jacobian_matches_finite_difference_twist():
    """Compare geometric Jacobian columns to FK finite differences."""
    robot = _robot()
    q = np.deg2rad([20, 35, 20, 10, 25, 10])
    jacobian, pose = calculate_jacobian(robot, q)
    eps = 1e-7
    for joint in range(robot.dof):
        q_plus = q.copy()
        q_minus = q.copy()
        q_plus[joint] += eps
        q_minus[joint] -= eps
        p_plus = forward_kinematics(robot, q_plus)
        p_minus = forward_kinematics(robot, q_minus)
        linear = (p_plus[:3, 3] - p_minus[:3, 3]) / (2 * eps)
        angular_matrix = (
            (p_plus[:3, :3] - p_minus[:3, :3])
            @ pose[:3, :3].T / (2 * eps)
        )
        angular = np.array([
            angular_matrix[2, 1] - angular_matrix[1, 2],
            angular_matrix[0, 2] - angular_matrix[2, 0],
            angular_matrix[1, 0] - angular_matrix[0, 1],
        ]) / 2
        assert np.allclose(linear, jacobian[:3, joint], atol=2e-8)
        assert np.allclose(angular, jacobian[3:, joint], atol=2e-8)


def test_rotation_error_zero_and_near_pi():
    """Check stable orientation errors at zero and near pi radians."""
    assert np.allclose(rotation_error(np.eye(3), np.eye(3)), 0.0, atol=1e-12)
    angle = np.pi - 1e-7
    desired = np.array([
        [1, 0, 0],
        [0, np.cos(angle), -np.sin(angle)],
        [0, np.sin(angle), np.cos(angle)],
    ])
    error = rotation_error(desired, np.eye(3))
    assert np.isfinite(error).all()
    assert np.isclose(np.linalg.norm(error), angle, atol=1e-6)


def test_sequential_dls_ik_tracks_every_cartesian_waypoint():
    """Check full-pose accuracy and limits at all sequential IK points."""
    robot = _robot()
    q_start = np.deg2rad([20, 35, 20, 10, 25, 10])
    q_goal = np.deg2rad([30, 45, 30, 20, 40, 25])
    start = forward_kinematics(robot, q_start)
    goal = forward_kinematics(robot, q_goal)
    path = generate_linear_cartesian_path(start, goal, 50)
    solver = DampedLeastSquaresIK(robot)
    result = solve_cartesian_path(path, solver, q_start)
    assert result.success
    assert len(result.joint_trajectory) == len(path)
    assert np.isfinite(np.diff(result.joint_trajectory, axis=0)).all()
    for target, q in zip(path, result.joint_trajectory):
        actual = forward_kinematics(robot, q)
        assert (
            np.linalg.norm(target[:3, 3] - actual[:3, 3])
            <= solver.position_tolerance
        )
        assert (
            np.linalg.norm(
                rotation_error(target[:3, :3], actual[:3, :3])
            )
            <= solver.orientation_tolerance
        )
        assert robot.check_joint_limits(q)


def test_dls_handles_near_singular_target_without_nonfinite_output():
    """Check DLS returns a bounded finite solution at a singular pose."""
    robot = _robot()
    q = np.zeros(robot.dof)
    target = forward_kinematics(robot, q)
    solver = DampedLeastSquaresIK(robot, max_iterations=20)
    result = solver.solve(target, q)
    assert result.success
    assert np.isfinite(result.joint_positions).all()
    assert robot.check_joint_limits(result.joint_positions)


@pytest.mark.parametrize(
    'solver_type',
    [DampedLeastSquaresIK, JacobianTransposeIK, PseudoinverseIK],
)
def test_ik_solvers_reject_non_rigid_target_poses_consistently(solver_type):
    """Every public solver validates homogeneous rigid target matrices."""
    robot = _robot()
    solver = solver_type(robot)
    q = np.deg2rad([20, 35, 20, 10, 25, 10])

    with pytest.raises(ValueError, match='numeric 4x4|4x4'):
        solver.solve([[1.0, 2.0]], q)
    invalid_rotation = np.eye(4)
    invalid_rotation[0, 0] = 2.0
    with pytest.raises(ValueError, match='rotation'):
        solver.solve(invalid_rotation, q)


@pytest.mark.parametrize(
    'solver_type',
    [DampedLeastSquaresIK, JacobianTransposeIK, PseudoinverseIK],
)
def test_ik_solvers_accept_numeric_list_target_poses(solver_type):
    """Solver input normalization accepts equivalent numeric pose lists."""
    robot = _robot()
    solver = solver_type(robot)
    q = np.deg2rad([20, 35, 20, 10, 25, 10])
    target = forward_kinematics(robot, q).tolist()
    result = solver.solve(target, q)

    assert result.success
    assert result.position_error <= solver.position_tolerance
    assert result.orientation_error <= solver.orientation_tolerance
