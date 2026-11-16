from dataclasses import dataclass

import numpy as np

from robot_arm_kinematics.forward_kinematics import forward_kinematics

from .cartesian_path import generate_linear_cartesian_path


@dataclass
class CartesianIKResult:
    joint_trajectory: list
    success: bool
    failed_waypoint: int
    message: str
    maximum_joint_step: float = 0.0
    cartesian_path: list | None = None


def solve_cartesian_path(
    path, ik_solver, q_initial, max_joint_waypoint_step=None,
    adaptive_refinement=False, max_refinement_depth=8,
):

    q = np.asarray(q_initial, dtype=float).copy()

    if len(path) < 2:
        raise ValueError('Cartesian path must contain at least two waypoints.')

    if q.shape != (ik_solver.robot.dof,):
        raise ValueError('Initial joint configuration has incorrect shape.')

    if not np.all(np.isfinite(q)):
        raise ValueError('Initial joint configuration must be finite.')
    if max_joint_waypoint_step is not None:
        if isinstance(max_joint_waypoint_step, (bool, np.bool_)):
            raise ValueError(
                'Maximum joint waypoint step must be positive and finite.'
            )
        try:
            max_joint_waypoint_step = float(max_joint_waypoint_step)
        except (TypeError, ValueError) as error:
            raise ValueError(
                'Maximum joint waypoint step must be positive and finite.'
            ) from error
        if (
            not np.isfinite(max_joint_waypoint_step)
            or max_joint_waypoint_step <= 0
        ):
            raise ValueError(
                'Maximum joint waypoint step must be positive and finite.'
            )

    q = ik_solver.robot.enforce_joint_limits(q)

    joint_trajectory = []
    accepted_path = []
    maximum_joint_step = 0.0
    if isinstance(max_refinement_depth, bool) or max_refinement_depth < 0:
        raise ValueError('max_refinement_depth must be a nonnegative integer.')

    def solve_segment(target_pose, waypoint_index, depth):
        nonlocal q, maximum_joint_step
        result = ik_solver.solve(target_pose, q)
        next_q = None
        failure = result.message
        if result.success:
            next_q = np.asarray(result.joint_positions, dtype=float)
            if (
                next_q.shape != (ik_solver.robot.dof,)
                or not np.all(np.isfinite(next_q))
                or not ik_solver.robot.check_joint_limits(next_q)
            ):
                next_q = None
                failure = 'IK returned an invalid or out-of-limit solution.'
        step = (
            float(np.max(np.abs(next_q - q)))
            if next_q is not None else float('inf')
        )
        if (
            next_q is not None
            and (max_joint_waypoint_step is None
                 or step <= max_joint_waypoint_step)
        ):
            q = next_q.copy()
            joint_trajectory.append(q.copy())
            accepted_path.append(np.asarray(target_pose, dtype=float))
            maximum_joint_step = max(maximum_joint_step, step)
            return None

        if adaptive_refinement and depth < max_refinement_depth:
            previous_pose = (
                accepted_path[-1]
                if accepted_path
                else forward_kinematics(ik_solver.robot, q)
            )
            midpoint = generate_linear_cartesian_path(
                previous_pose, target_pose, num_waypoints=3
            )[1]
            error = solve_segment(midpoint, waypoint_index, depth + 1)
            if error:
                return error
            return solve_segment(target_pose, waypoint_index, depth + 1)
        detail = (
            f'joint step {step:.6g} rad exceeds '
            f'{max_joint_waypoint_step:.6g} rad'
            if next_q is not None and max_joint_waypoint_step is not None
            else failure
        )
        return (
            f'Waypoint {waypoint_index} could not be reached after '
            f'{depth} refinement levels: {detail}. The segment may cross '
            'a singularity or an unreachable region.'
        )

    for index, target_pose in enumerate(path):
        error = solve_segment(target_pose, index, 0)
        if error:
            return CartesianIKResult(
                joint_trajectory=joint_trajectory, success=False,
                failed_waypoint=index, message=error,
                maximum_joint_step=maximum_joint_step,
                cartesian_path=accepted_path,
            )

    return CartesianIKResult(
        joint_trajectory=joint_trajectory,
        success=True,
        failed_waypoint=-1,
        message='Cartesian path successfully converted to joint trajectory.',
        maximum_joint_step=maximum_joint_step,
        cartesian_path=accepted_path,
    )


def solve_ik_with_restarts(ik_solver, target_pose, initial_configuration):
    """Retry a difficult IK target from deterministic alternate joint seeds."""
    robot = ik_solver.robot
    initial = robot.enforce_joint_limits(
        np.asarray(initial_configuration, dtype=float)
    )
    rng = np.random.default_rng(2026)
    seeds = [initial, 0.5 * (robot.lower_limits + robot.upper_limits)]
    seeds.extend(
        rng.uniform(robot.lower_limits, robot.upper_limits)
        for _ in range(6)
    )
    last_result = None
    for seed in seeds:
        last_result = ik_solver.solve(target_pose, seed)
        if last_result.success:
            return last_result
    return last_result
