import numpy as np

from robot_arm_kinematics.robot_model import RobotModel
from robot_arm_kinematics.forward_kinematics import forward_kinematics


# Load robot configuration
config_path = (
    'src/robot_arm_kinematics/'
    'robot_arm_kinematics/config/generic_robot.yaml'
)

robot = RobotModel(config_path)


# Test 1: Zero joint configuration
q_zero = np.zeros(robot.dof)

T_zero = forward_kinematics(robot, q_zero)

print("\n========== TEST 1: ZERO CONFIGURATION ==========")

print("\nTransformation matrix:")
print(T_zero)

print("\nEnd-effector position (m):")
print(T_zero[:3, 3])


# Test 2: Non-zero joint configuration
joint_angles_deg = [30, 45, 30, 0, 0, 0]

q = np.deg2rad(joint_angles_deg)

T = forward_kinematics(robot, q)

print("\n========== TEST 2: NON-ZERO CONFIGURATION ==========")

print("\nJoint angles (degrees):")
print(joint_angles_deg)

print("\nTransformation matrix:")
print(T)

print("\nEnd-effector position (m):")
print(T[:3, 3])

print("\nEnd-effector orientation:")
print(T[:3, :3])
