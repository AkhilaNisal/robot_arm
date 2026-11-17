"""Coordinate Cartesian or joint-space planning and time parameterization."""

from dataclasses import dataclass

import numpy as np

from robot_arm_kinematics.forward_kinematics import forward_kinematics

from .cartesian_space.cartesian_ik import (
    CartesianIKResult,
    solve_cartesian_path,
)
from .cartesian_space.cartesian_path import (
    generate_arc_cartesian_path,
    generate_linear_cartesian_path,
    validate_pose,
)
from .joint_space.planner import plan_joint_space
from .joint_space.joint_trajectory import (
    generate_joint_trajectory,
    JointTrajectory,
)


@dataclass
class CartesianJointTrajectoryResult:
    """Result of generating a joint trajectory from a Cartesian goal."""

    success: bool
    cartesian_path: list | None
    ik_result: CartesianIKResult | None
    joint_trajectory: JointTrajectory | None
    message: str
    failed_stage: str | None = None


def generate_cartesian_joint_trajectory(
    robot_model,
    ik_solver,
    q_initial,
    goal_pose,
    num_waypoints=50,
    duration=4.0,
    num_samples=101,
    interpolation_method='cubic_spline',
    time_parameterization='quintic_scaling',
    interpolation_options=None,
    time_parameterization_options=None,
    joint_position_limits=None,
    joint_velocity_limits=None,
    joint_acceleration_limits=None,
    joint_jerk_limits=None,
    enforce_position_limits=False,
    enforce_velocity_limits=False,
    enforce_acceleration_limits=False,
    enforce_jerk_limits=False,
    max_joint_waypoint_step=None,
    start_pose=None,
    path_method='cartesian_linear',
    cartesian_arc_height=0.1,
    max_refinement_depth=8,
    ik_restarts=6,
):
    """
    Generate a time-parameterized joint trajectory for a Cartesian goal.

    Invalid inputs raise ``ValueError``. IK and trajectory-generation failures
    return a result with ``success=False``, a stage name in ``failed_stage``,
    and no executable joint trajectory.
    """
    q = _validate_initial_configuration(robot_model, q_initial)
    target = validate_pose(goal_pose)
    _validate_pipeline_options(
        robot_model, ik_solver, num_waypoints, duration, num_samples
    )
    num_waypoints = int(num_waypoints)
    duration = float(duration)
    num_samples = int(num_samples)

    if start_pose is None:
        start_pose = forward_kinematics(robot_model, q)
    else:
        start_pose = validate_pose(start_pose)
    if path_method == 'cartesian_linear':
        cartesian_path = generate_linear_cartesian_path(
            start_pose, target, num_waypoints=num_waypoints
        )
        ik_result = solve_cartesian_path(
            cartesian_path, ik_solver, q,
            max_joint_waypoint_step=max_joint_waypoint_step,
            adaptive_refinement=True,
            max_refinement_depth=max_refinement_depth,
        )
    elif path_method == 'cartesian_arc':
        cartesian_path = generate_arc_cartesian_path(
            start_pose, target, num_waypoints=num_waypoints,
            arc_height=cartesian_arc_height,
        )
        ik_result = solve_cartesian_path(
            cartesian_path, ik_solver, q,
            max_joint_waypoint_step=max_joint_waypoint_step,
            adaptive_refinement=True,
            max_refinement_depth=max_refinement_depth,
        )
    elif path_method == 'joint_space':
        ik_result, cartesian_path, joint_space_error = plan_joint_space(
            robot_model, ik_solver, q, target, num_waypoints,
            ik_restarts=ik_restarts,
        )
        if joint_space_error is not None:
            return CartesianJointTrajectoryResult(
                success=False, cartesian_path=None, ik_result=None,
                joint_trajectory=None,
                message=f'Goal-pose IK failed: {joint_space_error}',
                failed_stage='ik',
            )
    else:
        raise ValueError(
            'path_method must be cartesian_linear, cartesian_arc, or '
            'joint_space.'
        )
    if not ik_result.success:
        return CartesianJointTrajectoryResult(
            success=False,
            cartesian_path=cartesian_path,
            ik_result=ik_result,
            joint_trajectory=None,
            message=(
                f'IK stage failed at waypoint {ik_result.failed_waypoint}: '
                f'{ik_result.message}'
            ),
            failed_stage='ik',
        )
    if ik_result.cartesian_path is not None:
        cartesian_path = ik_result.cartesian_path

    try:
        trajectory = generate_joint_trajectory(
            ik_result.joint_trajectory,
            duration=duration,
            num_samples=num_samples,
            interpolation_method=interpolation_method,
            time_parameterization=time_parameterization,
            interpolation_options=interpolation_options,
            time_parameterization_options=time_parameterization_options,
            joint_position_limits=joint_position_limits,
            joint_velocity_limits=joint_velocity_limits,
            joint_acceleration_limits=joint_acceleration_limits,
            joint_jerk_limits=joint_jerk_limits,
            enforce_position_limits=enforce_position_limits,
            enforce_velocity_limits=enforce_velocity_limits,
            enforce_acceleration_limits=enforce_acceleration_limits,
            enforce_jerk_limits=enforce_jerk_limits,
            robot_model=robot_model,
        )
    except (ValueError, RuntimeError, np.linalg.LinAlgError) as error:
        return CartesianJointTrajectoryResult(
            success=False,
            cartesian_path=cartesian_path,
            ik_result=ik_result,
            joint_trajectory=None,
            message=f'Joint trajectory stage failed: {error}',
            failed_stage='joint_trajectory',
        )

    return CartesianJointTrajectoryResult(
        success=True,
        cartesian_path=cartesian_path,
        ik_result=ik_result,
        joint_trajectory=trajectory,
        message='Cartesian path converted to a joint trajectory successfully.',
    )


def _validate_initial_configuration(robot_model, q_initial):
    """Validate initial joint values and configured position limits."""
    if not hasattr(robot_model, 'dof'):
        raise ValueError('Robot model must provide a degree-of-freedom count.')
    try:
        q = np.asarray(q_initial, dtype=float)
    except (TypeError, ValueError) as error:
        raise ValueError(
            'Initial joint configuration must be numeric.'
        ) from error

    if q.shape != (robot_model.dof,):
        raise ValueError(
            f'Initial joint configuration must have shape '
            f'({robot_model.dof},).'
        )
    if not np.all(np.isfinite(q)):
        raise ValueError('Initial joint configuration must be finite.')
    if not robot_model.check_joint_limits(q):
        raise ValueError(
            'Initial joint configuration violates position limits.'
        )
    return q.copy()


def _validate_pipeline_options(
    robot_model, ik_solver, num_waypoints, duration, num_samples
):
    """Validate pipeline dimensions, timing, and solver/model compatibility."""
    if (
        isinstance(num_waypoints, bool)
        or not isinstance(num_waypoints, (int, np.integer))
    ):
        raise ValueError('Number of Cartesian waypoints must be an integer.')
    if num_waypoints < 2:
        raise ValueError('At least two Cartesian waypoints are required.')
    if (
        isinstance(num_samples, bool)
        or not isinstance(num_samples, (int, np.integer))
    ):
        raise ValueError('Number of trajectory samples must be an integer.')
    if num_samples < 2:
        raise ValueError('At least two trajectory samples are required.')
    if (
        isinstance(duration, (bool, np.bool_))
        or not isinstance(duration, (int, float, np.integer, np.floating))
        or not np.isfinite(duration)
        or duration <= 0
    ):
        raise ValueError('Trajectory duration must be positive and finite.')

    solver_robot = getattr(ik_solver, 'robot', None)
    if solver_robot is None or not callable(getattr(ik_solver, 'solve', None)):
        raise ValueError(
            'IK solver must expose a robot model and solve method.'
        )
    if getattr(solver_robot, 'dof', None) != robot_model.dof:
        raise ValueError(
            'IK solver robot DOF does not match the pipeline robot.'
        )
    if not np.array_equal(
        getattr(solver_robot, 'lower_limits', None),
        getattr(robot_model, 'lower_limits', None),
    ) or not np.array_equal(
        getattr(solver_robot, 'upper_limits', None),
        getattr(robot_model, 'upper_limits', None),
    ) or not np.array_equal(
        getattr(solver_robot, 'velocity_limits', None),
        getattr(robot_model, 'velocity_limits', None),
    ):
        raise ValueError(
            'IK solver robot joint limits do not match the pipeline robot.'
        )
