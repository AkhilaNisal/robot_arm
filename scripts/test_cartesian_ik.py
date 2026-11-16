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


# Load robot model
config = (
    Path(__file__).resolve().parents[1]
    / "src/robot_arm_kinematics/robot_arm_kinematics/config/generic_robot.yaml"
)

robot = RobotModel(str(config))


# Define starting and goal joint configurations
q_start = np.radians([20, 30, 40, 10, 30, 20])
q_goal = np.radians([60, 50, 20, 70, 60, 90])


# Calculate starting and goal poses
T_start = forward_kinematics(robot, q_start)
T_goal = forward_kinematics(robot, q_goal)


# Generate Cartesian path
path = generate_linear_cartesian_path(
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


# Solve IK for all Cartesian waypoints
result = solve_cartesian_path(
    path,
    ik_solver,
    q_start
)


print("\n========== CARTESIAN IK TEST ==========\n")

print("Number of Cartesian waypoints:", len(path))
print("Number of joint configurations:", len(result.joint_trajectory))

print("\nSuccess:", result.success)
print("Message:", result.message)

if result.success:

    trajectory = np.array(result.joint_trajectory)

    print("\nStarting joint configuration (degrees):")
    print(np.round(np.degrees(trajectory[0]), 3))

    print("\nFinal joint configuration (degrees):")
    print(np.round(np.degrees(trajectory[-1]), 3))

    joint_changes = np.diff(trajectory, axis=0)

    print("\nMaximum joint change between consecutive waypoints (degrees):")
    print(np.round(np.degrees(np.max(np.abs(joint_changes), axis=0)), 3))

    # Verify all joint configurations satisfy limits
    limits_valid = all(
        robot.check_joint_limits(q)
        for q in trajectory
    )

    print("\nAll joint configurations within limits:", limits_valid)

    # Verify final end-effector pose
    T_final = forward_kinematics(robot, trajectory[-1])

    final_position_error = np.linalg.norm(
        T_goal[:3, 3] - T_final[:3, 3]
    )

    print("\nFinal position error (m):", final_position_error)

    print("\nJoint trajectory shape:", trajectory.shape)

else:

    print("\nFailed at Cartesian waypoint:", result.failed_waypoint)
