import numpy as np

from robot_arm_kinematics.robot_model import RobotModel
from robot_arm_kinematics.forward_kinematics import forward_kinematics
from robot_arm_kinematics.jacobian import calculate_jacobian


def rotation_vector(R):
    """
    Extract the small-angle rotation vector from a rotation matrix.
    Assumes the rotation angle is small.
    """

    return 0.5 * np.array([
        R[2, 1] - R[1, 2],
        R[0, 2] - R[2, 0],
        R[1, 0] - R[0, 1]
    ])


def numerical_jacobian(robot, q, epsilon=1e-6):

    J_num = np.zeros((6, robot.dof))

    for i in range(robot.dof):

        q_plus = q.copy()
        q_minus = q.copy()

        q_plus[i] += epsilon
        q_minus[i] -= epsilon

        T_plus = forward_kinematics(robot, q_plus)
        T_minus = forward_kinematics(robot, q_minus)

        # Numerical linear velocity
        J_num[:3, i] = (
            T_plus[:3, 3] - T_minus[:3, 3]
        ) / (2 * epsilon)

        # Relative rotation in the base frame
        R_delta = (
            T_plus[:3, :3] @ T_minus[:3, :3].T
        )

        # Numerical angular velocity
        J_num[3:, i] = (
            rotation_vector(R_delta) / (2 * epsilon)
        )

    return J_num


# Load robot
config_path = (
    'src/robot_arm_kinematics/'
    'robot_arm_kinematics/config/generic_robot.yaml'
)

robot = RobotModel(config_path)

# Test configuration
joint_angles_deg = [30, 45, 30, 0, 0, 0]
q = np.deg2rad(joint_angles_deg)

# Analytical Jacobian
J_analytical, _ = calculate_jacobian(robot, q)

# Numerical Jacobian
J_numerical = numerical_jacobian(robot, q)

# Difference
difference = J_analytical - J_numerical

print("\nAnalytical Jacobian:")
print(J_analytical)

print("\nNumerical Jacobian:")
print(J_numerical)

print("\nDifference:")
print(difference)

print("\nMaximum absolute error:")
print(np.max(np.abs(difference)))

print("\nFrobenius norm of error:")
print(np.linalg.norm(difference, ord='fro'))

if np.allclose(
    J_analytical,
    J_numerical,
    atol=1e-6,
    rtol=1e-5
):
    print("\nVALIDATION PASSED!")
else:
    print("\nVALIDATION FAILED!")
