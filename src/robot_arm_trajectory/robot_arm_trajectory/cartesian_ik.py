from dataclasses import dataclass

import numpy as np


@dataclass
class CartesianIKResult:
    joint_trajectory: list
    success: bool
    failed_waypoint: int
    message: str
    maximum_joint_step: float = 0.0


def solve_cartesian_path(
    path, ik_solver, q_initial, max_joint_waypoint_step=None
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
    maximum_joint_step = 0.0

    for i, target_pose in enumerate(path):

        result = ik_solver.solve(target_pose, q)

        if not result.success:
            return CartesianIKResult(
                joint_trajectory=joint_trajectory,
                success=False,
                failed_waypoint=i,
                message=f'IK failed at waypoint {i}: {result.message}',
                maximum_joint_step=maximum_joint_step,
            )

        next_q = np.asarray(result.joint_positions, dtype=float)
        if (
            next_q.shape != (ik_solver.robot.dof,)
            or not np.all(np.isfinite(next_q))
            or not ik_solver.robot.check_joint_limits(next_q)
        ):
            return CartesianIKResult(
                joint_trajectory=joint_trajectory,
                success=False,
                failed_waypoint=i,
                message=(
                    f'IK returned an invalid or out-of-limit solution at '
                    f'waypoint {i}.'
                ),
                maximum_joint_step=maximum_joint_step,
            )
        joint_step = float(np.max(np.abs(next_q - q)))
        maximum_joint_step = max(maximum_joint_step, joint_step)
        if (
            max_joint_waypoint_step is not None
            and joint_step > max_joint_waypoint_step
        ):
            return CartesianIKResult(
                joint_trajectory=joint_trajectory,
                success=False,
                failed_waypoint=i,
                message=(
                    f'Joint discontinuity at waypoint {i}: maximum joint '
                    f'step {joint_step:.6g} rad exceeds '
                    f'{max_joint_waypoint_step:.6g} rad.'
                ),
                maximum_joint_step=maximum_joint_step,
            )

        q = next_q.copy()

        joint_trajectory.append(q.copy())

    return CartesianIKResult(
        joint_trajectory=joint_trajectory,
        success=True,
        failed_waypoint=-1,
        message='Cartesian path successfully converted to joint trajectory.',
        maximum_joint_step=maximum_joint_step,
    )
