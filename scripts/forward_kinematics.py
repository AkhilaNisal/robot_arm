import numpy as np


def rot_x(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([
        [1, 0, 0, 0],
        [0, c, -s, 0],
        [0, s, c, 0],
        [0, 0, 0, 1]
    ])


def rot_y(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([
        [c, 0, s, 0],
        [0, 1, 0, 0],
        [-s, 0, c, 0],
        [0, 0, 0, 1]
    ])


def rot_z(q):
    c, s = np.cos(q), np.sin(q)
    return np.array([
        [c, -s, 0, 0],
        [s, c, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])


def trans_z(d):
    T = np.eye(4)
    T[2, 3] = d
    return T


def forward_kinematics(q):

    lengths = [0.15, 0.30, 0.25, 0.20, 0.12, 0.10]

    axes = [rot_z, rot_y, rot_y, rot_z, rot_y, rot_z]

    T = np.eye(4)

    for i in range(6):
        T = T @ trans_z(lengths[i]) @ axes[i](q[i])

    # Fixed tool transformation
    T = T @ trans_z(0.10)

    return T


# Joint angles in degrees
joint_angles_deg = [30, 45, 30, 0, 0, 0]

# Convert to radians
q = np.deg2rad(joint_angles_deg)

T = forward_kinematics(q)

np.set_printoptions(precision=4, suppress=True)

print("Joint angles (degrees):", joint_angles_deg)
print("\nEnd-effector transformation:")
print(T)

print("\nEnd-effector position (m):")
print(T[:3, 3])
