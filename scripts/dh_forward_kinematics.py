import numpy as np


# Standard DH transformation
def dh_transform(a, alpha, d, theta):

    ct = np.cos(theta)
    st = np.sin(theta)

    ca = np.cos(alpha)
    sa = np.sin(alpha)

    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,       sa,     ca,    d],
        [0,        0,      0,    1]
    ])


def forward_kinematics_dh(q):

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

    for i in range(6):

        a, alpha, d, offset = dh_table[i]

        theta = q[i] + offset

        T = T @ dh_transform(a, alpha, d, theta)

    return T


# Test configuration
joint_angles_deg = [30, 45, 30, 0, 0, 0]

q = np.deg2rad(joint_angles_deg)

T = forward_kinematics_dh(q)

np.set_printoptions(precision=4, suppress=True)

print("Joint angles (degrees):", joint_angles_deg)
print("\nDH Forward Kinematics:")
print(T)

print("\nEnd-effector position (m):")
print(T[:3, 3])
