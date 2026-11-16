import numpy as np


def dh_transform(a, alpha, d, theta):
    """
    Calculate the standard DH homogeneous transformation.

    Parameters:
        a: Link length (m)
        alpha: Link twist (rad)
        d: Link offset (m)
        theta: Joint angle (rad)

    Returns:
        4x4 homogeneous transformation matrix.
    """

    ct = np.cos(theta)
    st = np.sin(theta)

    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,        sa,       ca,     d],
        [0,         0,        0,     1]
    ])


def forward_kinematics(robot, q):
    """
    Calculate the end-effector transformation using standard DH.

    Parameters:
        robot: RobotModel instance.
        q: Joint angles in radians.

    Returns:
        4x4 transformation matrix from base to end-effector.
    """

    q = np.asarray(q, dtype=float)

    if q.shape != (robot.dof,):
        raise ValueError(
            f"Expected {robot.dof} joint angles, got {q.shape}."
        )

    if not np.all(np.isfinite(q)):
        raise ValueError("Joint angles must be finite.")

    T = robot.base_transform.copy()

    for i in range(robot.dof):

        a, alpha, d, offset = robot.dh_parameters[i]

        theta = q[i] + offset

        T = T @ dh_transform(
            a, alpha, d, theta
        )

    return T @ robot.tool_transform
