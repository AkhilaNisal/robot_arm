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


# DH parameters: a, alpha, d, theta_offset
dh_table = [
    [0.00, -np.pi/2, 0.30,  0],
    [0.25,  0,       0.00, -np.pi/2],
    [0.00,  np.pi/2, 0.00,  np.pi/2],
    [0.00, -np.pi/2, 0.32,  0],
    [0.00,  np.pi/2, 0.00,  0],
    [0.00,  0,       0.20,  0]
]


# Calculate FK and position Jacobian together
def forward_kinematics_and_jacobian(q):

    T = np.eye(4)
    T[2, 3] = 0.15

    origins = []
    axes = []

    for i in range(6):

        # Joint axis and origin before applying this joint
        origins.append(T[:3, 3].copy())
        axes.append(T[:3, 2].copy())

        a, alpha, d, offset = dh_table[i]

        T = T @ dh_transform(
            a, alpha, d, q[i] + offset
        )

    # End-effector position
    p = T[:3, 3]

    # Position Jacobian
    J = np.zeros((3, 6))

    for i in range(6):

        J[:, i] = np.cross(
            axes[i],
            p - origins[i]
        )

    return p, J


# Position IK using Damped Least Squares
def inverse_kinematics(target_position):

    # Initial joint angles (radians)
    q = np.zeros(6)

    # Robot joint limits (radians)
    lower_limits = np.array([
        -3.14, -2.0, -2.5, -3.14, -2.0, -3.14
    ])

    upper_limits = np.array([
        3.14, 2.0, 2.5, 3.14, 2.0, 3.14
    ])

    damping = 0.05
    step_size = 0.5
    tolerance = 1e-4

    for iteration in range(1000):

        current_position, J = \
            forward_kinematics_and_jacobian(q)

        error = target_position - current_position

        error_norm = np.linalg.norm(error)

        if error_norm < tolerance:
            print("\nIK converged!")
            break

        # Damped Least Squares
        dq = J.T @ np.linalg.solve(
            J @ J.T + damping**2 * np.eye(3),
            error
        )

        # Update joint angles
        q = q + step_size * dq

        # Enforce joint limits
        q = np.clip(q, lower_limits, upper_limits)

    else:
        print("\nMaximum iterations reached.")

    final_position, _ = \
        forward_kinematics_and_jacobian(q)

    return q, final_position, error_norm, iteration + 1


# Desired end-effector position (metres)
target = np.array([0.4, 0.2, 0.7])

# Solve IK
q_solution, final_position, error, iterations = \
    inverse_kinematics(target)


# Display results
print("\nDesired position (m):")
print(target)

print("\nCalculated joint angles (degrees):")
print(np.round(np.rad2deg(q_solution), 3))

print("\nFinal end-effector position (m):")
print(np.round(final_position, 4))

print("\nFinal position error (m):")
print(f"{np.linalg.norm(target - final_position):.6f}")

print("\nIterations:")
print(iterations)
