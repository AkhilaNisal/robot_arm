import numpy as np
from pathlib import Path

from robot_arm_kinematics.robot_model import RobotModel
from robot_arm_kinematics.forward_kinematics import forward_kinematics

from robot_arm_kinematics.inverse_kinematics.damped_least_squares import (
    DampedLeastSquaresIK,
    rotation_error
)

from robot_arm_trajectory.cartesian_path import (
    generate_linear_cartesian_path
)

from robot_arm_trajectory.cartesian_ik import (
    solve_cartesian_path
)


# Load robot model
config = (
    Path(__file__).resolve().parents[1]
    / "src/robot_arm_kinematics/robot_arm_kinematics/config/generic_robot.yaml"
)

robot = RobotModel(str(config))


# Define start and goal configurations
q_start = np.radians([20, 30, 40, 10, 30, 20])
q_goal = np.radians([60, 50, 20, 70, 60, 90])


# Generate Cartesian path
T_start = forward_kinematics(robot, q_start)
T_goal = forward_kinematics(robot, q_goal)

path = generate_linear_cartesian_path(
    T_start,
    T_goal,
    num_waypoints=50
)


# Initialize IK solver
solver = DampedLeastSquaresIK(
    robot,
    damping=0.05,
    characteristic_length=0.25,
    position_tolerance=1e-4,
    orientation_tolerance=1e-3,
    max_iterations=1000
)


# Solve Cartesian path
result = solve_cartesian_path(path, solver, q_start)

if not result.success:
    raise RuntimeError(result.message)


# Validate every waypoint using forward kinematics
position_errors = []
orientation_errors = []

for target_pose, q in zip(path, result.joint_trajectory):

    calculated_pose = forward_kinematics(robot, q)

    position_error = np.linalg.norm(
        target_pose[:3, 3] - calculated_pose[:3, 3]
    )

    orientation_error = np.linalg.norm(
        rotation_error(
            target_pose[:3, :3],
            calculated_pose[:3, :3]
        )
    )

    position_errors.append(position_error)
    orientation_errors.append(orientation_error)


position_errors = np.array(position_errors)
orientation_errors = np.array(orientation_errors)

trajectory = np.array(result.joint_trajectory)

joint_steps = np.diff(trajectory, axis=0)


# Print results
print("\n========== CARTESIAN TRAJECTORY VALIDATION ==========\n")

print("Number of waypoints:", len(path))

print("\nMaximum position error (mm):",
      np.max(position_errors) * 1000)

print("Mean position error (mm):",
      np.mean(position_errors) * 1000)

print("\nMaximum orientation error (degrees):",
      np.degrees(np.max(orientation_errors)))

print("Mean orientation error (degrees):",
      np.degrees(np.mean(orientation_errors)))

print("\nMaximum joint change per waypoint (degrees):")
print(np.degrees(np.max(np.abs(joint_steps), axis=0)))

print("\nAll joint configurations within limits:",
      all(robot.check_joint_limits(q) for q in trajectory))

print("\nPosition tolerance satisfied:",
      np.all(position_errors <= solver.position_tolerance))

print("Orientation tolerance satisfied:",
      np.all(orientation_errors <= solver.orientation_tolerance))
