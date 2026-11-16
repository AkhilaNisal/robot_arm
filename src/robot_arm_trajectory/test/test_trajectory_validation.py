"""Regression tests for independent trajectory validation."""

from dataclasses import replace
from pathlib import Path

import numpy as np

import pytest

from robot_arm_kinematics.forward_kinematics import forward_kinematics
from robot_arm_kinematics.robot_model import RobotModel

from robot_arm_trajectory.cartesian_space.cartesian_path import (
    generate_linear_cartesian_path,
)
from robot_arm_trajectory.joint_space.joint_trajectory import (
    generate_joint_trajectory,
)
from robot_arm_trajectory.validation import validate_trajectory


@pytest.fixture
def robot():
    """Load the generic robot model."""
    config = (
        Path(__file__).parents[2]
        / 'robot_arm_kinematics'
        / 'robot_arm_kinematics'
        / 'config'
        / 'generic_robot.yaml'
    )
    return RobotModel(str(config))


def _trajectory(robot, duration=4.0, samples=101, waypoints=None):
    """Create a smooth in-limit joint trajectory for validation tests."""
    if waypoints is None:
        waypoints = np.deg2rad([
            [20, 35, 20, 10, 25, 10],
            [23, 38, 23, 13, 28, 13],
        ])
    return generate_joint_trajectory(
        waypoints,
        duration=duration,
        num_samples=samples,
        interpolation_method='quintic_polynomial',
    )


def _copy_with(trajectory, **updates):
    """Copy a trajectory and its arrays before changing test values."""
    values = {
        name: None if getattr(trajectory, name) is None else
        np.array(getattr(trajectory, name), copy=True)
        for name in (
            'time', 'positions', 'velocities', 'accelerations', 'jerks'
        )
    }
    values.update(updates)
    return replace(trajectory, **values)


def _check(result, name):
    return next(check for check in result.checks if check.name == name)


def test_valid_trajectory_reports_extrema_and_singularity_warning(robot):
    """A valid trajectory passes constraints and reports per-joint metrics."""
    trajectory = _trajectory(robot)
    trajectory_snapshot = {
        name: np.array(getattr(trajectory, name), copy=True)
        for name in (
            'time', 'positions', 'velocities', 'accelerations', 'jerks'
        )
    }
    lower_snapshot = robot.lower_limits.copy()
    result = validate_trajectory(trajectory, robot)

    assert result.success
    assert not result.errors
    assert len(result.checks) > 0
    assert len(result.summary['minimum_position_per_joint']) == robot.dof
    assert len(result.summary['maximum_position_per_joint']) == robot.dof
    assert len(
        result.summary['maximum_absolute_velocity_per_joint']
    ) == robot.dof
    assert len(
        result.summary['maximum_absolute_acceleration_per_joint']
    ) == robot.dof
    assert len(
        result.summary['maximum_absolute_jerk_per_joint']
    ) == robot.dof
    for name, values in trajectory_snapshot.items():
        assert np.array_equal(getattr(trajectory, name), values)
    assert np.array_equal(robot.lower_limits, lower_snapshot)


def test_joint_position_limit_violation(robot):
    """Position limit violations are reported as an error check."""
    q = np.array([[0.0] * robot.dof, [0.0] * robot.dof])
    q[-1, 0] = robot.upper_limits[0] + 0.2
    result = validate_trajectory(_trajectory(robot, waypoints=q), robot)
    assert not result.success
    assert not _check(result, 'joint_position_limits').passed


def test_joint_velocity_limit_violation(robot):
    """Model velocity limits are applied per joint."""
    trajectory = _trajectory(robot, duration=0.02)
    result = validate_trajectory(trajectory, robot)
    assert not result.success
    assert not _check(result, 'joint_velocity_limits').passed


def test_acceleration_and_jerk_limit_violations(robot):
    """Optional acceleration and jerk limits are enforced in reports."""
    trajectory = _trajectory(robot, duration=0.5)
    result = validate_trajectory(
        trajectory,
        robot,
        acceleration_limits=np.full(robot.dof, 1e-5),
        jerk_limits=np.full(robot.dof, 1e-5),
    )
    assert not result.success
    assert not _check(result, 'joint_acceleration_limits').passed
    assert not _check(result, 'joint_jerk_limits').passed


def test_invalid_time_sequence_is_rejected(robot):
    """Repeated or descending timestamps fail safely."""
    trajectory = _trajectory(robot)
    time = trajectory.time.copy()
    time[5] = time[4]
    result = validate_trajectory(_copy_with(trajectory, time=time), robot)
    assert not result.success
    assert not _check(result, 'time_strictly_increasing').passed


@pytest.mark.parametrize('bad_value', [np.nan, np.inf, -np.inf])
def test_nonfinite_trajectory_values_are_rejected(robot, bad_value):
    """Nonfinite values fail numerical checks without raising."""
    trajectory = _trajectory(robot)
    positions = trajectory.positions.copy()
    positions[3, 2] = bad_value
    result = validate_trajectory(
        _copy_with(trajectory, positions=positions), robot
    )
    assert not result.success
    assert not _check(result, 'positions_finite').passed


def test_incompatible_dimensions_are_rejected(robot):
    """Mismatched sample or joint dimensions return structured errors."""
    trajectory = _trajectory(robot)
    velocities = trajectory.velocities[:-1]
    result = validate_trajectory(
        _copy_with(trajectory, velocities=velocities), robot
    )
    assert not result.success
    assert not _check(result, 'array_dimensions').passed

    wrong_dof = trajectory.positions[:, :-1]
    result = validate_trajectory(
        _copy_with(trajectory, positions=wrong_dof), robot
    )
    assert not result.success
    assert not _check(result, 'joint_count').passed


def test_cartesian_position_tracking_error_is_measured(robot):
    """A displaced reference path reports position error in metres."""
    trajectory = _trajectory(robot)
    q0 = trajectory.positions[0]
    q1 = trajectory.positions[-1]
    start = forward_kinematics(robot, q0)
    goal = forward_kinematics(robot, q1)
    path = generate_linear_cartesian_path(start, goal, 7)
    for pose in path:
        pose[:3, 3] += [0.25, 0.0, 0.0]
    result = validate_trajectory(
        trajectory, robot, path, cartesian_position_tolerance=0.01
    )
    assert not _check(result, 'cartesian_tracking').passed
    assert result.summary['cartesian_max_position_error_m'] > 0.1
    assert 'cartesian_rms_position_error_m' in result.summary


def test_cartesian_orientation_tracking_error_is_measured(robot):
    """An orientation mismatch reports SO(3) error in radians."""
    trajectory = _trajectory(robot)
    identity = np.eye(4)
    path = [identity.copy() for _ in range(5)]
    result = validate_trajectory(
        trajectory, robot, path, cartesian_orientation_tolerance=0.01
    )
    assert not _check(result, 'cartesian_tracking').passed
    assert result.summary['cartesian_max_orientation_error_rad'] > 0.1
    assert 'cartesian_rms_orientation_error_rad' in result.summary


def test_cartesian_reference_with_different_sample_count(robot):
    """Reference waypoints are interpolated by time, not compared by index."""
    trajectory = _trajectory(robot, samples=101)
    poses = [
        forward_kinematics(robot, q)
        for q in np.linspace(
            trajectory.positions[0], trajectory.positions[-1], 9
        )
    ]
    result = validate_trajectory(trajectory, robot, poses)
    cartesian_check = _check(result, 'cartesian_tracking')
    assert cartesian_check.measured['maximum_position_error_m'] < 0.01
    assert cartesian_check.measured['maximum_orientation_error_rad'] < 0.05


def test_singular_and_near_singular_jacobians_are_warnings(robot):
    """Singularity proximity warns but does not itself fail validation."""
    singular_q = np.zeros((2, robot.dof))
    singular = _trajectory(robot, waypoints=singular_q)
    result = validate_trajectory(
        singular, robot, singularity_threshold=1e-4
    )
    assert result.success
    singular_check = _check(result, 'jacobian_singularity')
    assert not singular_check.passed
    assert singular_check.severity == 'warning'
    assert result.warnings

    near_q = np.vstack((
        np.full(robot.dof, 1e-3),
        np.full(robot.dof, 2e-3),
    ))
    near = _trajectory(robot, waypoints=near_q)
    near_result = validate_trajectory(
        near, robot, singularity_threshold=0.5
    )
    assert near_result.success
    assert _check(near_result, 'jacobian_singularity').severity == 'warning'
