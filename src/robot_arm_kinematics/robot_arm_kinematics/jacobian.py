import numpy as np

from robot_arm_kinematics.forward_kinematics import dh_transform


def calculate_jacobian(robot, q):
    """
    Calculate the 6xN geometric Jacobian.

    The upper three rows represent linear velocity.
    The lower three rows represent angular velocity.

    Parameters:
        robot: RobotModel instance.
        q: Joint angles in radians.

    Returns:
        J: 6xN geometric Jacobian.
        T: 4x4 end-effector transformation.
    """

    q = np.asarray(q, dtype=float)

    if q.shape != (robot.dof,):
        raise ValueError(
            f"Expected {robot.dof} joint angles, got {q.shape}."
        )

    if not np.all(np.isfinite(q)):
        raise ValueError("Joint angles must be finite.")

    T = robot.base_transform.copy()

    origins = []
    axes = []

    # Calculate intermediate transformations
    for i in range(robot.dof):

        # Joint origin and rotation axis in base frame
        origins.append(T[:3, 3].copy())
        axes.append(T[:3, 2].copy())

        a, alpha, d, offset = robot.dh_parameters[i]

        theta = q[i] + offset

        T = T @ dh_transform(a, alpha, d, theta)

    # End-effector position
    p_end = T[:3, 3]

    # Construct geometric Jacobian
    J = np.zeros((6, robot.dof))

    for i in range(robot.dof):

        # Linear velocity contribution
        J[:3, i] = np.cross(
            axes[i],
            p_end - origins[i]
        )

        # Angular velocity contribution
        J[3:, i] = axes[i]

    return J, T