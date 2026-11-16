import numpy as np

from robot_arm_kinematics.robot_model import RobotModel
from robot_arm_kinematics.jacobian import calculate_jacobian


config_path = (
    'src/robot_arm_kinematics/'
    'robot_arm_kinematics/config/generic_robot.yaml'
)

robot = RobotModel(config_path)

# Test configuration
joint_angles_deg = [30, 45, 30, 0, 0, 0]
q = np.deg2rad(joint_angles_deg)

# Calculate Jacobian
J, T = calculate_jacobian(robot, q)

np.set_printoptions(precision=4, suppress=True)

print("\nJoint angles (degrees):")
print(joint_angles_deg)

print("\nEnd-effector position (m):")
print(T[:3, 3])

print("\nFull Geometric Jacobian (6x6):")
print(J)

print("\nLinear Jacobian (3x6):")
print(J[:3, :])

print("\nAngular Jacobian (3x6):")
print(J[3:, :])

print("\nJacobian dimensions:")
print(J.shape)

print("\nJacobian rank:")
print(np.linalg.matrix_rank(J))

print("\nJacobian determinant:")
print(np.linalg.det(J))
