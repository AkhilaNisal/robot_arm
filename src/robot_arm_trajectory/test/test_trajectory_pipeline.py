"""End-to-end regression tests for Cartesian trajectory generation."""

from pathlib import Path

import numpy as np

import pytest

from robot_arm_kinematics.forward_kinematics import forward_kinematics
from robot_arm_kinematics.inverse_kinematics.damped_least_squares import (
    DampedLeastSquaresIK,
    rotation_error,
)
from robot_arm_kinematics.robot_model import RobotModel

from robot_arm_trajectory import generate_cartesian_joint_trajectory
from robot_arm_trajectory.cartesian_path import validate_pose


@pytest.fixture
def setup_robot():
    """Load the generic arm and a nearby reachable Cartesian goal."""
    config = (
        Path(__file__).parents[2]
        / 'robot_arm_kinematics'
        / 'robot_arm_kinematics'
        / 'config'
        / 'generic_robot.yaml'
    )
    robot = RobotModel(str(config))
    q_start = np.deg2rad([20, 35, 20, 10, 25, 10])
    T_start = forward_kinematics(robot, q_start)
    q_goal = np.deg2rad([30, 45, 30, 20, 40, 25])
    T_goal = forward_kinematics(robot, q_goal)
    solver = DampedLeastSquaresIK(robot)
    return robot, solver, q_start, T_start, T_goal


def _generate(robot, solver, q_start, T_goal, **kwargs):
    """Call the pipeline with compact defaults used by the tests."""
    options = {
        'robot_model': robot,
        'ik_solver': solver,
        'q_initial': q_start,
        'goal_pose': T_goal,
        'num_waypoints': 20,
        'duration': 3.0,
        'num_samples': 151,
    }
    options.update(kwargs)
    return generate_cartesian_joint_trajectory(**options)


def test_pipeline_generates_full_cartesian_to_joint_trajectory(setup_robot):
    """The pipeline returns accurate poses and correctly shaped arrays."""
    robot, solver, q_start, T_start, T_goal = setup_robot
    result = _generate(robot, solver, q_start, T_goal)

    assert result.success
    assert result.failed_stage is None
    assert result.ik_result.success
    assert len(result.cartesian_path) == 20
    assert result.joint_trajectory.time.shape == (151,)
    assert result.joint_trajectory.positions.shape == (151, robot.dof)
    assert result.joint_trajectory.velocities.shape == (151, robot.dof)
    assert result.joint_trajectory.accelerations.shape == (151, robot.dof)

    # Start pose is derived from FK of q_initial and used as the path start.
    assert np.allclose(result.cartesian_path[0], T_start, atol=1e-12)
    assert np.allclose(result.cartesian_path[-1], T_goal, atol=1e-12)

    final_pose = forward_kinematics(
        robot, result.joint_trajectory.positions[-1]
    )
    assert np.linalg.norm(final_pose[:3, 3] - T_goal[:3, 3]) <= 1e-4
    assert np.linalg.norm(
        rotation_error(T_goal[:3, :3], final_pose[:3, :3])
    ) <= 1e-3

    waypoints = np.asarray(result.ik_result.joint_trajectory)
    assert waypoints.shape == (20, robot.dof)
    assert np.isfinite(np.diff(waypoints, axis=0)).all()
    assert np.max(np.linalg.norm(np.diff(waypoints, axis=0), axis=1)) < 0.3


def test_pipeline_accepts_user_selected_cartesian_start_pose(setup_robot):
    """The path begins at a selected pose instead of FK of the seed joints."""
    robot, solver, q_seed, _, T_goal = setup_robot
    q_selected_start = q_seed + np.deg2rad([2, -2, 1, 0, 1, -1])
    selected_start = forward_kinematics(robot, q_selected_start)
    result = _generate(
        robot, solver, q_seed, T_goal,
        start_pose=selected_start,
        num_waypoints=20,
    )

    assert result.success
    assert np.allclose(result.cartesian_path[0], selected_start, atol=1e-12)
    actual_start = forward_kinematics(
        robot, result.joint_trajectory.positions[0]
    )
    assert np.linalg.norm(actual_start[:3, 3] - selected_start[:3, 3]) < 1e-4


def test_pipeline_rejects_invalid_initial_configuration(setup_robot):
    """Malformed, non-finite, and out-of-limit starts raise ValueError."""
    robot, solver, q_start, _, T_goal = setup_robot
    with pytest.raises(ValueError, match='shape'):
        _generate(robot, solver, q_start[:-1], T_goal)
    invalid = q_start.copy()
    invalid[0] = np.nan
    with pytest.raises(ValueError, match='finite'):
        _generate(robot, solver, invalid, T_goal)
    invalid = q_start.copy()
    invalid[0] = robot.upper_limits[0] + 0.1
    with pytest.raises(ValueError, match='position limits'):
        _generate(robot, solver, invalid, T_goal)


def test_pipeline_rejects_invalid_goal_pose(setup_robot):
    """Invalid homogeneous transforms and rotations are rejected."""
    robot, solver, q_start, _, T_goal = setup_robot
    with pytest.raises(ValueError, match='4x4'):
        _generate(robot, solver, q_start, np.eye(3))
    invalid = T_goal.copy()
    invalid[0, 0] = 2.0
    with pytest.raises(ValueError, match='orthonormal'):
        _generate(robot, solver, q_start, invalid)


def test_pipeline_returns_structured_failure_when_ik_cannot_reach_goal(
    setup_robot,
):
    """Unreachable targets stop at IK and never expose a joint trajectory."""
    robot, solver, q_start, _, T_goal = setup_robot
    unreachable = T_goal.copy()
    unreachable[:3, 3] += [100.0, 0.0, 0.0]
    result = _generate(robot, solver, q_start, unreachable)

    assert not result.success
    assert result.failed_stage == 'ik'
    assert result.cartesian_path is not None
    assert result.ik_result is not None
    assert not result.ik_result.success
    assert result.ik_result.failed_waypoint >= 0
    assert str(result.ik_result.failed_waypoint) in result.message
    assert result.joint_trajectory is None


def test_pipeline_returns_failure_when_trajectory_constraints_fail(
    setup_robot,
):
    """Constraint rejection is stage-tagged and returns no trajectory."""
    robot, solver, q_start, _, T_goal = setup_robot
    result = _generate(
        robot,
        solver,
        q_start,
        T_goal,
        joint_acceleration_limits=np.full(robot.dof, 1e-8),
        enforce_acceleration_limits=True,
    )

    assert not result.success
    assert result.failed_stage == 'joint_trajectory'
    assert result.ik_result.success
    assert result.joint_trajectory is None
    assert 'violates configured joint limits' in result.message


def test_pipeline_rejects_large_sequential_ik_joint_steps(setup_robot):
    """Configured continuity checks stop a branch jump at its waypoint."""
    robot, solver, q_start, _, T_goal = setup_robot
    result = _generate(
        robot,
        solver,
        q_start,
        T_goal,
        max_joint_waypoint_step=1e-5,
    )

    assert not result.success
    assert result.failed_stage == 'ik'
    assert result.joint_trajectory is None
    assert not result.ik_result.success
    assert result.ik_result.failed_waypoint > 0
    assert 'Joint discontinuity' in result.message
    assert result.ik_result.maximum_joint_step > 1e-5


def test_pipeline_validates_waypoint_sample_and_time_arguments(setup_robot):
    """Invalid path dimensions and duration fail before producing a result."""
    robot, solver, q_start, _, T_goal = setup_robot
    for kwargs in (
        {'num_waypoints': 1},
        {'num_samples': 1},
        {'duration': 0.0},
        {'duration': np.inf},
        {'max_joint_waypoint_step': 0.0},
        {'max_joint_waypoint_step': np.nan},
    ):
        with pytest.raises(ValueError):
            _generate(robot, solver, q_start, T_goal, **kwargs)


def test_pipeline_uses_existing_pose_validation(setup_robot):
    """The goal accepted by the pipeline follows the shared pose validator."""
    robot, solver, q_start, _, T_goal = setup_robot
    assert np.array_equal(validate_pose(T_goal), T_goal)
    result = _generate(robot, solver, q_start, T_goal, num_waypoints=2)
    assert result.success
