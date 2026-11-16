import numpy as np

from robot_arm_kinematics.robot_model import RobotModel
from robot_arm_kinematics.forward_kinematics import (
    forward_kinematics
)

from robot_arm_kinematics.inverse_kinematics.damped_least_squares import (
    DampedLeastSquaresIK
)


# Load robot
config_path = (
    'src/robot_arm_kinematics/'
    'robot_arm_kinematics/config/generic_robot.yaml'
)

robot = RobotModel(config_path)


# Define a known reachable target configuration
q_target = np.deg2rad([
    30, 45, 30, 20, 40, 25
])

# Generate target pose using FK
T_target = forward_kinematics(
    robot,
    q_target
)


# Choose a different initial configuration
q_initial = np.deg2rad([
    20, 35, 20, 10, 25, 10
])


# Create DLS solver
solver = DampedLeastSquaresIK(
    robot=robot,
    damping=0.05,
    characteristic_length=0.25,
    position_tolerance=1e-4,
    orientation_tolerance=1e-3,
    max_iterations=1000
)


# Solve full-pose IK
result = solver.solve(
    target_pose=T_target,
    q_initial=q_initial
)


# Calculate final pose
T_final = forward_kinematics(
    robot,
    result.joint_positions
)


np.set_printoptions(precision=6, suppress=True)

print("\n========== FULL-POSE IK TEST ==========")

print("\nTarget joint angles (degrees):")
print(np.rad2deg(q_target))

print("\nInitial joint angles (degrees):")
print(np.rad2deg(q_initial))

print("\nCalculated joint angles (degrees):")
print(np.rad2deg(result.joint_positions))

print("\nTarget end-effector position:")
print(T_target[:3, 3])

print("\nCalculated end-effector position:")
print(T_final[:3, 3])

print("\nTarget orientation:")
print(T_target[:3, :3])

print("\nCalculated orientation:")
print(T_final[:3, :3])

print("\nPosition error (m):")
print(result.position_error)

print("\nOrientation error (rad):")
print(result.orientation_error)

print("\nIterations:")
print(result.iterations)

print("\nSuccess:")
print(result.success)

print("\nMessage:")
print(result.message)
