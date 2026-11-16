"""Plan a joint-space path from an IK goal configuration."""

import numpy as np

from robot_arm_kinematics.forward_kinematics import forward_kinematics

from ..cartesian_space.cartesian_ik import (
    CartesianIKResult,
    solve_ik_with_restarts,
)


def plan_joint_space(robot_model, ik_solver, q_initial, goal_pose,
                     num_waypoints):
    """Solve endpoint IK and interpolate joint waypoints to that solution.

    Returns the IK result and FK poses for those joint-space waypoints. The
    tool path between waypoints is unconstrained and can curve in Cartesian
    space.
    """
    start = np.asarray(q_initial, dtype=float)
    endpoint = solve_ik_with_restarts(ik_solver, goal_pose, start)
    if not endpoint.success:
        return None, None, endpoint.message

    end = robot_model.enforce_joint_limits(
        np.asarray(endpoint.joint_positions, dtype=float)
    )
    joint_waypoints = np.linspace(start, end, num_waypoints)
    cartesian_path = [
        forward_kinematics(robot_model, configuration)
        for configuration in joint_waypoints
    ]
    ik_result = CartesianIKResult(
        joint_trajectory=[row.copy() for row in joint_waypoints],
        success=True,
        failed_waypoint=-1,
        message='Joint-space path generated from endpoint IK.',
    )
    return ik_result, cartesian_path, None
