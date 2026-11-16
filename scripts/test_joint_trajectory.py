import numpy as np
from pathlib import Path

from robot_arm_kinematics.robot_model import RobotModel
from robot_arm_kinematics.forward_kinematics import forward_kinematics

from robot_arm_kinematics.inverse_kinematics.damped_least_squares import (
    DampedLeastSquaresIK
)

from robot_arm_trajectory.cartesian_path import (
    generate_linear_cartesian_path
)

from robot_arm_trajectory.cartesian_ik import (
    solve_cartesian_path
)

from robot_arm_trajectory.joint_trajectory import (
    generate_joint_trajectory
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

cartesian_path = generate_linear_cartesian_path(
    T_start,
    T_goal,
    num_waypoints=50
)


# Initialize IK solver
ik_solver = DampedLeastSquaresIK(
    robot,
    damping=0.05,
    characteristic_length=0.25,
    position_tolerance=1e-4,
    orientation_tolerance=1e-3,
    max_iterations=1000
)


# Convert Cartesian path to joint waypoints
ik_result = solve_cartesian_path(
    cartesian_path,
    ik_solver,
    q_start
)

if not ik_result.success:
    raise RuntimeError(ik_result.message)


# Generate time-parameterized joint trajectory
trajectory = generate_joint_trajectory(
    ik_result.joint_trajectory,
    duration=4.0,
    num_samples=101
)


# Display results
print("\n========== JOINT TRAJECTORY TEST ==========\n")

print("Duration:", trajectory.time[-1], "seconds")
print("Number of time samples:", len(trajectory.time))
print("Number of joints:", trajectory.positions.shape[1])

print("\nInitial joint positions (degrees):")
print(np.round(np.degrees(trajectory.positions[0]), 4))

print("\nFinal joint positions (degrees):")
print(np.round(np.degrees(trajectory.positions[-1]), 4))

print("\nInitial joint velocities (rad/s):")
print(trajectory.velocities[0])

print("\nFinal joint velocities (rad/s):")
print(trajectory.velocities[-1])

print("\nInitial joint accelerations (rad/s^2):")
print(trajectory.accelerations[0])

print("\nFinal joint accelerations (rad/s^2):")
print(trajectory.accelerations[-1])

print("\nMaximum joint velocities (rad/s):")
print(np.max(np.abs(trajectory.velocities), axis=0))

print("\nMaximum joint accelerations (rad/s^2):")
print(np.max(np.abs(trajectory.accelerations), axis=0))

print("\nJoint trajectory shape:", trajectory.positions.shape)

print("\nVelocity shape:", trajectory.velocities.shape)

print("Acceleration shape:", trajectory.accelerations.shape)
