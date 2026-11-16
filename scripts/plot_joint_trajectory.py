"""Reproduce and plot the reference Cartesian-to-joint trajectory experiment."""

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from robot_arm_kinematics.robot_model import RobotModel
from robot_arm_kinematics.forward_kinematics import forward_kinematics

from robot_arm_kinematics.inverse_kinematics.damped_least_squares import (
    DampedLeastSquaresIK,
    rotation_error,
)
from robot_arm_trajectory.cartesian_path import (
    generate_linear_cartesian_path,
    quaternion_slerp,
    quaternion_to_rotation,
    rotation_to_quaternion,
)
from robot_arm_trajectory.cartesian_ik import (
    solve_cartesian_path,
)
from robot_arm_trajectory.joint_trajectory import (
    generate_joint_trajectory,
)


# --------------------------------------------------
# 1. Load robot model
# --------------------------------------------------

config = (
    Path(__file__).resolve().parents[1]
    / "src/robot_arm_kinematics/robot_arm_kinematics/config/generic_robot.yaml"
)

robot = RobotModel(str(config))


# --------------------------------------------------
# 2. Define start and goal configurations
# --------------------------------------------------

q_start = np.deg2rad([20, 35, 20, 10, 25, 10])
q_goal = np.deg2rad([30, 45, 30, 20, 40, 25])


# --------------------------------------------------
# 3. Generate Cartesian path
# --------------------------------------------------

T_start = forward_kinematics(robot, q_start)
T_goal = forward_kinematics(robot, q_goal)

cartesian_path = generate_linear_cartesian_path(
    T_start,
    T_goal,
    num_waypoints=50
)


# --------------------------------------------------
# 4. Solve Cartesian inverse kinematics
# --------------------------------------------------

ik_solver = DampedLeastSquaresIK(robot)

ik_result = solve_cartesian_path(
    cartesian_path,
    ik_solver,
    q_start
)

if not ik_result.success:
    raise RuntimeError(ik_result.message)

joint_waypoints = np.asarray(ik_result.joint_trajectory)

print("\nCartesian IK completed successfully.")
print("Number of waypoints:", len(joint_waypoints))
print("Joint waypoint shape:", joint_waypoints.shape)

# --------------------------------------------------
# Diagnose local changes in IK waypoints
# --------------------------------------------------

# First differences: change between consecutive waypoints
delta_q = np.diff(joint_waypoints, axis=0)

# Second differences: change in consecutive waypoint changes
delta2_q = np.diff(delta_q, axis=0)

delta_q_deg = np.rad2deg(delta_q)
delta2_q_deg = np.rad2deg(delta2_q)

print("\n" + "=" * 60)
print("LOCAL IK WAYPOINT SMOOTHNESS")
print("=" * 60)

print("\nMaximum consecutive joint changes (degrees):")
print(np.round(np.max(np.abs(delta_q_deg), axis=0), 5))

print("\nMaximum second differences (degrees):")
print(np.round(np.max(np.abs(delta2_q_deg), axis=0), 5))

# Identify locations with the largest second differences
for joint in range(robot.dof):

    index = np.argmax(np.abs(delta2_q_deg[:, joint]))

    print(
        f"Joint {joint + 1}: "
        f"largest second difference at waypoint {index + 1}, "
        f"value = {delta2_q_deg[index, joint]:.6f} degrees"
    )


# Plot joint-space waypoints
plt.figure(figsize=(12, 6))

for joint in range(robot.dof):

    plt.plot(
        np.linspace(0, 1, len(joint_waypoints)),
        np.rad2deg(joint_waypoints[:, joint]),
        label=f"Joint {joint + 1}"
    )

plt.title("Joint-Space Waypoints from Cartesian IK")
plt.xlabel("Normalized Path Parameter")
plt.ylabel("Joint Position (degrees)")
plt.grid(True)
plt.legend()
plt.tight_layout()
# --------------------------------------------------
# 5. Diagnose waypoint smoothness
# --------------------------------------------------

# Normalized path parameter
s_waypoints = np.linspace(
    0.0,
    1.0,
    len(joint_waypoints)
)

# First derivative dq/ds
waypoint_velocity = np.gradient(
    joint_waypoints,
    s_waypoints,
    axis=0,
    edge_order=2
)

# Second derivative d²q/ds²
waypoint_acceleration = np.gradient(
    waypoint_velocity,
    s_waypoints,
    axis=0,
    edge_order=2
)

# Convert to degrees for readability
waypoint_velocity_deg = np.rad2deg(waypoint_velocity)
waypoint_acceleration_deg = np.rad2deg(waypoint_acceleration)

print("\n" + "=" * 60)
print("WAYPOINT DERIVATIVE DIAGNOSTICS")
print("=" * 60)

print(
    "Maximum absolute first derivative (deg/path unit):"
)

print(
    np.round(
        np.max(np.abs(waypoint_velocity_deg), axis=0),
        3
    )
)

print(
    "\nMaximum absolute second derivative (deg/path unit²):"
)

print(
    np.round(
        np.max(np.abs(waypoint_acceleration_deg), axis=0),
        3
    )
)


# --------------------------------------------------
# 6. Generate trajectories using both interpolators
# --------------------------------------------------

duration = 4.0
num_samples = 401

trajectories = {}

for method in ["cubic_spline", "quintic_polynomial"]:

    print(f"\nGenerating trajectory using {method}...")

    trajectory = generate_joint_trajectory(
        joint_waypoints=joint_waypoints,
        duration=duration,
        num_samples=num_samples,
        interpolation_method=method,
        time_parameterization="quintic_scaling",
        robot_model=robot
    )

    trajectories[method] = trajectory

# Dense full-trajectory Cartesian tracking check. The reference path uses the
# same straight position and quaternion SLERP definition as the waypoint path.
tracking = {}
for method, trajectory in trajectories.items():
    position_errors = []
    orientation_errors = []
    for t, q in zip(trajectory.time, trajectory.positions):
        u = t / duration
        scale = 10*u**3 - 15*u**4 + 6*u**5
        target_position = T_start[:3, 3] + scale * (T_goal[:3, 3] - T_start[:3, 3])
        target_rotation = quaternion_to_rotation(quaternion_slerp(
            rotation_to_quaternion(T_start[:3, :3]),
            rotation_to_quaternion(T_goal[:3, :3]), scale))
        actual = forward_kinematics(robot, q)
        position_errors.append(np.linalg.norm(target_position - actual[:3, 3]))
        orientation_errors.append(np.linalg.norm(rotation_error(target_rotation, actual[:3, :3])))
    tracking[method] = {
        "max_position_error_m": float(np.max(position_errors)),
        "max_orientation_error_rad": float(np.max(orientation_errors)),
    }


# --------------------------------------------------
# 7. Compare trajectory performance
# --------------------------------------------------

print("\n" + "=" * 60)
print("TRAJECTORY PERFORMANCE COMPARISON")
print("=" * 60)

for method, trajectory in trajectories.items():

    positions_deg = np.rad2deg(trajectory.positions)
    velocities_deg = np.rad2deg(trajectory.velocities)
    accelerations_deg = np.rad2deg(trajectory.accelerations)
    jerks_deg = np.rad2deg(trajectory.jerks)

    print(f"\n{'=' * 55}")
    print(f"Interpolation: {method}")
    print(f"{'=' * 55}")

    print(
        "Maximum velocity (deg/s):",
        np.round(
            np.max(np.abs(velocities_deg), axis=0),
            3
        )
    )

    print(
        "Maximum acceleration (deg/s²):",
        np.round(
            np.max(np.abs(accelerations_deg), axis=0),
            3
        )
    )

    print(
        "Overall maximum acceleration:",
        round(
            np.max(np.abs(accelerations_deg)),
            3
        ),
        "deg/s²"
    )

    print("Maximum jerk (deg/s³):", np.round(np.max(np.abs(jerks_deg), axis=0), 3))
    print("Cartesian maximum position error (mm):", round(1000*tracking[method]["max_position_error_m"], 4))
    print("Cartesian maximum orientation error (deg):", round(np.degrees(tracking[method]["max_orientation_error_rad"]), 4))
    print("Position limit violations:", bool(np.any((trajectory.positions < robot.lower_limits) | (trajectory.positions > robot.upper_limits))))
    print("Velocity limit violations:", bool(np.any(np.abs(trajectory.velocities) > robot.velocity_limits)))


# --------------------------------------------------
# 8. Plot trajectory comparison
# --------------------------------------------------

plot_data = [
    (
        "positions",
        "Joint Position Comparison",
        "Position (degrees)"
    ),
    (
        "velocities",
        "Joint Velocity Comparison",
        "Velocity (degrees/s)"
    ),
    (
        "accelerations",
        "Joint Acceleration Comparison",
        "Acceleration (degrees/s²)"
    ),
    (
        "jerks",
        "Joint Jerk",
        "Jerk (degrees/s³)"
    )
]


for attribute, title, ylabel in plot_data:

    plt.figure(figsize=(12, 6))

    for method, trajectory in trajectories.items():

        values = np.rad2deg(
            getattr(trajectory, attribute)
        )

        for i in range(robot.dof):

            plt.plot(
                trajectory.time,
                values[:, i],
                label=f"{method} - J{i + 1}"
            )

    plt.title(title)
    plt.xlabel("Time (seconds)")
    plt.ylabel(ylabel)

    plt.grid(True)
    plt.legend(ncol=2, fontsize=8)
    plt.tight_layout()


output_dir = Path("trajectory_validation_plots")
output_dir.mkdir(exist_ok=True)

# Seven separate readable plots, including geometry versus path parameter.
for attribute, title, ylabel in plot_data:
    fig, axes = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
    for method, trajectory in trajectories.items():
        values = np.rad2deg(getattr(trajectory, attribute))
        for joint in range(robot.dof):
            axes[joint].plot(trajectory.time, values[:, joint], label=method)
        axes[joint].set_ylabel(f"J{joint+1}")
        axes[joint].grid(True)
    axes[0].legend()
    axes[-1].set_xlabel("Time (s)")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(output_dir / f"{attribute}_vs_time.png", dpi=150)
    plt.close(fig)

s_plot = np.linspace(0.0, 1.0, 1001)
for derivative, title, ylabel in (
    (0, "Joint Position versus Path Parameter", "Position (degrees)"),
    (1, "First Path Derivative", "dq/ds (degrees/path unit)"),
    (2, "Second Path Derivative", "d²q/ds² (degrees/path unit²)"),
):
    fig, axes = plt.subplots(6, 1, figsize=(10, 12), sharex=True)
    for method in ("cubic_spline", "quintic_polynomial"):
        from robot_arm_trajectory.interpolation import INTERPOLATORS
        interpolation = INTERPOLATORS[method]().fit(joint_waypoints)
        values = np.rad2deg(interpolation.evaluate(s_plot, derivative))
        for joint in range(robot.dof):
            axes[joint].plot(s_plot, values[:, joint], label=method)
        axes[joint].set_ylabel(f"J{joint+1}")
        axes[joint].grid(True)
    axes[0].legend()
    axes[-1].set_xlabel("Path parameter s")
    fig.suptitle(title)
    fig.tight_layout()
    fig.savefig(output_dir / f"path_derivative_{derivative}.png", dpi=150)
    plt.close(fig)

summary = {
    method: {
        "max_velocity_deg_s": np.max(np.abs(np.rad2deg(traj.velocities)), axis=0).tolist(),
        "max_acceleration_deg_s2": np.max(np.abs(np.rad2deg(traj.accelerations)), axis=0).tolist(),
        "max_jerk_deg_s3": np.max(np.abs(np.rad2deg(traj.jerks)), axis=0).tolist(),
        **tracking[method],
    }
    for method, traj in trajectories.items()
}
(output_dir / "summary.json").write_text(json.dumps(summary, indent=2))
print(f"\nSaved plots and summary to {output_dir.resolve()}")
