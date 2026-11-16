# ROS 2 Six-DOF Robot Arm Framework

A reusable six-axis arm framework for ROS 2 Jazzy. The current project stage
covers robot description, kinematics, inverse kinematics, Cartesian and joint
trajectory generation, validation, and RViz trajectory preview. Gazebo and
hardware execution are not included yet.

## Implemented packages

- **`robot_arm_description`** — generic six-joint Xacro/URDF model and launch
  files for model display and trajectory preview.
- **`robot_arm_kinematics`** — standard DH forward kinematics, geometric
  Jacobian, robot model and joint limits, and full-pose IK solvers (damped
  least squares, Jacobian transpose, and pseudoinverse).
- **`robot_arm_trajectory`** — Cartesian path generation, sequential IK,
  linear/cubic/quintic joint interpolation, quintic/trapezoidal/seventh-order
  time parameterization, trajectory validation, and an RViz preview.

The preview is visualization-only. It publishes `/joint_states` for
`robot_state_publisher`; it does not command hardware, start Gazebo, or run a
controller.

## Requirements

- Ubuntu 24.04 and ROS 2 Jazzy
- Python 3, NumPy, SciPy, PyYAML
- ROS packages: `xacro`, `robot_state_publisher`, `rviz2`,
  `interactive_markers`

Install ROS dependencies if needed:

```bash
sudo apt update
sudo apt install ros-jazzy-xacro ros-jazzy-robot-state-publisher \
  ros-jazzy-rviz2 ros-jazzy-interactive-markers python3-numpy \
  python3-scipy python3-yaml
```

## Build

From the workspace root:

```bash
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to robot_arm_description robot_arm_trajectory
source install/setup.bash
```

## Select Cartesian poses and preview a trajectory

Launch the full application in one terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_arm/install/setup.bash
ros2 launch robot_arm_description trajectory_preview.launch.py
```

The default `pose_input_mode:=rviz` displays editable blue **Start Pose** and
orange **Goal Pose** interactive markers in `base_link`. Select RViz's
**Interact** tool to translate and rotate each pose. Right-click either marker
and select **Plan & Preview Trajectory**. Planning is never started
automatically. Reset actions are also available in the marker menu.

Alternatively, set both complete poses in one YAML file and launch in YAML
mode:

```bash
ros2 launch robot_arm_description trajectory_preview.launch.py \
  pose_input_mode:=yaml
```

The default file is
`src/robot_arm_trajectory/robot_arm_trajectory/config/preview_poses.yaml`.
Set `pose_config:=/path/to/poses.yaml` to use another file. It contains both
poses and the planner settings used in YAML mode:

```yaml
poses:
  start:
    position: [0.5604379913, 0.2196020866, 0.8741139828]
    orientation_xyzw: [-0.0333377208, 0.6398709371, 0.2788298409, 0.7153374729]
  goal:
    position: [0.5540330285, 0.3706424487, 0.6325639287]
    orientation_xyzw: [0.0159379926, 0.8320655822, 0.3912271307, 0.3928793448]
trajectory:
  num_cartesian_waypoints: 50
  duration: 4.0
  num_samples: 401
  interpolation_method: cubic_spline
  interpolation_options:
    bc_type: not-a-knot
  time_parameterization_method: quintic_scaling
  time_parameterization_options: {}
  ik_solver: damped_least_squares
  max_joint_waypoint_step: 0.5
  cartesian_position_tolerance: 0.002
  cartesian_orientation_tolerance: 0.02
```

Quaternions use normalized ROS order `[x, y, z, w]`. In YAML mode the markers
show the configured poses without drag controls; use the context menu or
`/trajectory_preview/plan` service to plan. The default planner choices are
cubic spline interpolation and quintic time scaling: the spline is C2 across
waypoints, and the timing profile has zero path velocity and acceleration at
both ends. They are a smooth general-purpose preview profile, not a guarantee
that every custom pose pair is reachable or satisfies a user-specific speed
limit. YAML mode uses the trajectory settings from this file; launch-supplied
ROS parameter overrides still take precedence.
In RViz mode the marker values are the live pose input. Both modes use the same
start-pose IK, sequential Cartesian IK, trajectory generation, and validation
steps. The `initial_joint_configuration` parameter remains the start IK seed.

YAML mode markers are read-only but include a menu control. If the RViz context
menu is inconvenient, explicitly request planning from another sourced
terminal:

```bash
ros2 service call /trajectory_preview/plan std_srvs/srv/Trigger {}
```

This service does not plan automatically; it performs the same validation and
playback flow as **Plan & Preview Trajectory**.

Useful launch options:

```bash
ros2 launch robot_arm_description trajectory_preview.launch.py \
  pose_input_mode:=rviz preview_rate:=30.0 marker_publish_rate:=1.0 \
  looping:=false use_sim_time:=false
```

`pose_input_mode` accepts `rviz` or `yaml`. `pose_config` selects the YAML file;
`robot_config` selects the robot DH and joint-limit configuration. Other preview
parameters include `preview_rate`, `marker_publish_rate`, `looping`,
`use_sim_time`, `initial_joint_configuration`, `num_cartesian_waypoints`,
`trajectory_duration`, `num_trajectory_samples`, `interpolation_method`,
`time_parameterization_method`, and `ik_solver`.

The preview solves full-pose IK for the selected start from the configured
joint seed. It then plans the Cartesian path to the selected goal using the
existing sequential IK pipeline. Unreachable poses, joint-limit or waypoint
continuity failures, and failed trajectory validation prevent playback; the
node reports the failure in the launch terminal. Changing a pose invalidates a
previous plan. A new explicit planning request is required.

Topics:

- `/joint_states` — `sensor_msgs/msg/JointState`
- `/trajectory_preview/reference_path` — desired Cartesian `LINE_STRIP`
- `/trajectory_preview/actual_path` — FK-derived Cartesian `LINE_STRIP`
- `/trajectory_preview/poses/update` — interactive marker updates

Path markers publish immediately after successful validation and continue at
`marker_publish_rate`, including after playback completes. `looping:=true`
plays forward and backward. `use_sim_time:=true` makes playback follow `/clock`.

## Algorithms and validation

Trajectory path interpolation and time parameterization are independent.
Supported interpolation methods are `linear`, `cubic_spline`, and
`quintic_polynomial`; supported timing methods are `quintic_scaling`,
`trapezoidal`, and `seventh_order`. The `s_curve` name remains a deprecated
alias for the seventh-order polynomial and is not a conventional constant-jerk
S-curve.

Trajectory validation checks finite data, configured joint constraints, and
optional Cartesian tracking. Position extrema use floating-point polynomial
root solving; derivative extrema include numerical searches. These are
numerical checks, not formal continuous-time certificates. No generic joint
acceleration or jerk limits are assumed.

See [`src/robot_arm_trajectory/README.md`](src/robot_arm_trajectory/README.md)
for the Python trajectory API and validation details. Standalone diagnostic
and demonstration scripts are in `scripts/`; package regression tests are under
each package's `test/` directory.

Run relevant tests with:

```bash
colcon test --packages-select robot_arm_kinematics robot_arm_trajectory
colcon test-result --verbose
```

## Current scope and next stage

The current milestone ends at a validated trajectory preview in RViz. The next
planned milestone is Gazebo simulation, followed by dynamics and controller
integration. No Gazebo or hardware dependencies are required for the current
workflow.
