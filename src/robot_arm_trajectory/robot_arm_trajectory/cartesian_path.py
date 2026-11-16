import numpy as np


def rotation_to_quaternion(R):
    """Convert a 3x3 rotation matrix to quaternion [x, y, z, w]."""

    trace = np.trace(R)

    if trace > 0:
        s = 2.0 * np.sqrt(trace + 1.0)
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s

    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s

    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s

    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def quaternion_to_rotation(q):
    """Convert quaternion [x, y, z, w] to a 3x3 rotation matrix."""

    x, y, z, w = q / np.linalg.norm(q)

    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ])


def quaternion_slerp(q0, q1, s):
    """Spherical linear interpolation between two quaternions."""

    dot = np.dot(q0, q1)

    # Use the shortest rotational path.
    if dot < 0:
        q1 = -q1
        dot = -dot

    dot = np.clip(dot, -1.0, 1.0)

    # Nearly identical orientations: use linear interpolation.
    if dot > 0.9995:
        q = q0 + s * (q1 - q0)
        return q / np.linalg.norm(q)

    theta = np.arccos(dot)
    sin_theta = np.sin(theta)

    q = (
        np.sin((1 - s) * theta) / sin_theta * q0
        + np.sin(s * theta) / sin_theta * q1
    )

    return q / np.linalg.norm(q)


def validate_pose(T):
    """Validate a homogeneous transformation matrix."""

    T = np.asarray(T, dtype=float)

    if T.shape != (4, 4):
        raise ValueError("Pose must be a 4x4 matrix.")

    if not np.all(np.isfinite(T)):
        raise ValueError("Pose must contain finite values.")

    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError("Invalid homogeneous transformation matrix.")

    R = T[:3, :3]

    if not np.allclose(R.T @ R, np.eye(3), atol=1e-6):
        raise ValueError("Rotation matrix must be orthonormal.")

    if not np.isclose(np.linalg.det(R), 1.0, atol=1e-6):
        raise ValueError("Rotation matrix determinant must be +1.")

    return T


def generate_linear_cartesian_path(start_pose, goal_pose, num_waypoints=50):
    """
    Generate a linear Cartesian path.

    Position: Linear interpolation.
    Orientation: Quaternion SLERP.

    Returns:
        List of 4x4 homogeneous transformation matrices.
    """

    start_pose = validate_pose(start_pose)
    goal_pose = validate_pose(goal_pose)

    if not isinstance(num_waypoints, int) or num_waypoints < 2:
        raise ValueError("At least two waypoints are required.")

    p0 = start_pose[:3, 3]
    p1 = goal_pose[:3, 3]

    q0 = rotation_to_quaternion(start_pose[:3, :3])
    q1 = rotation_to_quaternion(goal_pose[:3, :3])

    path = []

    for s in np.linspace(0.0, 1.0, num_waypoints):

        position = p0 + s * (p1 - p0)

        orientation = quaternion_slerp(q0, q1, s)

        T = np.eye(4)
        T[:3, :3] = quaternion_to_rotation(orientation)
        T[:3, 3] = position

        path.append(T)

    return path
