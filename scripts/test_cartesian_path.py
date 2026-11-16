import numpy as np
from pathlib import Path

from robot_arm_kinematics.robot_model import RobotModel
from robot_arm_kinematics.forward_kinematics import forward_kinematics

from robot_arm_trajectory.cartesian_path import (
    generate_linear_cartesian_path
)

# Load robot model
config = (
    Path(__file__).resolve().parents[1]
    / "src/robot_arm_kinematics/robot_arm_kinematics/config/generic_robot.yaml"
)

robot = RobotModel(str(config))

# Define starting and goal joint configurations
q_start = np.radians([20, 30, 40, 10, 30, 20])
q_goal = np.radians([60, 50, 20, 70, 60, 90])

# Calculate corresponding end-effector poses
T_start = forward_kinematics(robot, q_start)
T_goal = forward_kinematics(robot, q_goal)

# Generate Cartesian trajectory
path = generate_linear_cartesian_path(
    T_start,
    T_goal,
    num_waypoints=50
)

print("\n========== CARTESIAN PATH TEST ==========\n")

print("Number of waypoints:", len(path))

print("\nStart position:")
print(path[0][:3, 3])

print("\nGoal position:")
print(path[-1][:3, 3])

print("\nStart position error:",
      np.linalg.norm(path[0][:3, 3] - T_start[:3, 3]))

print("Goal position error:",
      np.linalg.norm(path[-1][:3, 3] - T_goal[:3, 3]))

# Validate all waypoints
valid = True

for i, T in enumerate(path):

    R = T[:3, :3]

    if not np.allclose(R.T @ R, np.eye(3), atol=1e-6):
        valid = False
        print(f"Invalid rotation at waypoint {i}")

    if not np.isclose(np.linalg.det(R), 1.0, atol=1e-6):
        valid = False
        print(f"Invalid rotation determinant at waypoint {i}")

    if not np.all(np.isfinite(T)):
        valid = False
        print(f"Non-finite values at waypoint {i}")

# Check linear position spacing
distances = [
    np.linalg.norm(path[i + 1][:3, 3] - path[i][:3, 3])
    for i in range(len(path) - 1)
]

print("\nMaximum waypoint spacing:", max(distances), "m")
print("Minimum waypoint spacing:", min(distances), "m")

print("\nPath validation:", "PASSED" if valid else "FAILED")
