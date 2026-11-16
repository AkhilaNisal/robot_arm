import numpy as np
from pathlib import Path

from robot_arm_kinematics.robot_model import RobotModel
from robot_arm_kinematics.forward_kinematics import forward_kinematics
from robot_arm_kinematics.inverse_kinematics.damped_least_squares import (
    DampedLeastSquaresIK
)

config = (
    Path(__file__).resolve().parents[1]
    / "src/robot_arm_kinematics/robot_arm_kinematics/config/generic_robot.yaml"
)

robot = RobotModel(str(config))

solver = DampedLeastSquaresIK(
    robot,
    damping=0.05,
    characteristic_length=0.25,
    position_tolerance=1e-4,
    orientation_tolerance=1e-3,
    max_iterations=1000
)

target_configs_deg = [
    [30, 45, 30, 20, 40, 25],
    [-45, 30, -40, 60, -35, 20],
    [90, -30, 60, -45, 50, -60],
    [-120, 50, 70, 30, -60, 90],
    [15, -60, 80, -90, 30, 45]
]

initial_offset_deg = np.array([10, -8, 7, -6, 8, -5])

print("\n========== MULTI-CONFIGURATION IK VALIDATION ==========\n")

passed = 0

for i, target_deg in enumerate(target_configs_deg):

    q_target = np.radians(target_deg)
    q_initial = q_target + np.radians(initial_offset_deg)

    target_pose = forward_kinematics(robot, q_target)

    result = solver.solve(target_pose, q_initial)

    print(f"Test {i + 1}")
    print("Success:", result.success)
    print("Iterations:", result.iterations)
    print("Position error:", result.position_error, "m")
    print("Orientation error:", result.orientation_error, "rad")
    print("Calculated joints:", np.round(np.degrees(result.joint_positions), 3))
    print("----------------------------------------")

    if result.success:
        passed += 1

print(f"\nRESULT: {passed}/{len(target_configs_deg)} tests passed.")
