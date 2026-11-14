import numpy as np


def dh_transform(a, alpha, d, theta):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)

    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,       sa,     ca,    d],
        [0,        0,      0,    1]
    ])


def calculate_jacobian(q):

    # DH parameters: a, alpha, d, theta_offset
    dh_table = [
        [0.00, -np.pi/2, 0.30,  0],
        [0.25,  0,       0.00, -np.pi/2],
        [0.00,  np.pi/2, 0.00,  np.pi/2],
        [0.00, -np.pi/2, 0.32,  0],
        [0.00,  np.pi/2, 0.00,  0],
        [0.00,  0,       0.20,  0]
    ]

    # Fixed transformation from base_link to DH frame 0
    T = np.eye(4)
    T[2, 3] = 0.15

    origins = []
    axes = []

    # Calculate joint origins and axes
    for i in range(6):

        origins.append(T[:3, 3].copy())
        axes.append(T[:3, 2].copy())

        a, alpha, d, offset = dh_table[i]
        theta = q[i] + offset

        T = T @ dh_transform(a, alpha, d, theta)

    # Final end-effector position
    p_end = T[:3, 3]

    # Calculate position Jacobian
    J = np.zeros((3, 6))

    for i in range(6):
        J[:, i] = np.cross(
            axes[i],
            p_end - origins[i]
        )

    return J, p_end


# Test configuration
joint_angles_deg = [30, 45, 30, 0, 0, 0]
q = np.deg2rad(joint_angles_deg)

J, position = calculate_jacobian(q)

np.set_printoptions(precision=4, suppress=True)

print("Joint angles (degrees):", joint_angles_deg)

print("\nEnd-effector position (m):")
print(position)

print("\nPosition Jacobian:")
print(J)
