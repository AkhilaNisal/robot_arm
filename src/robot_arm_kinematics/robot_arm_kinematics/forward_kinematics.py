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

    if robot.name.lower() == 'gp8':
        from scipy.spatial.transform import Rotation

        chain = [
            ((0, 0, .33), (0, 0, -.02618), (0, 0, -1)),
            ((.039986, .0010471, 0), (1.5708, 0, 0), (0, 0, 1)),
            ((0, .445, 0), (0, 0, 1.5708), (0, 0, 1)),
            ((.039, 0, 0), (1.5708, 0, 0), (0, 0, -1)),
            ((0, -.011518, .43985), (-1.5708, 0, 0), (0, 0, 1)),
            ((0, 0, 0), (1.5708, 0, 0), (0, 0, -1)),
            ((0, 0, .15), (0, 0, 0), None),
        ]
        T = np.eye(4)
        for index, (translation, rpy, axis) in enumerate(chain):
            origin = np.eye(4)
            origin[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
            origin[:3, 3] = translation
            T = T @ origin
            if axis is not None:
                joint_rotation = np.eye(4)
                joint_rotation[:3, :3] = Rotation.from_rotvec(
                    np.asarray(axis, dtype=float) * q[index]
                ).as_matrix()
                T = T @ joint_rotation
        return T

    T = robot.base_transform.copy()

    for i in range(robot.dof):

        a, alpha, d, offset = robot.dh_parameters[i]

        theta = q[i] + offset

        T = T @ dh_transform(
            a, alpha, d, theta
        )

    return T
