# ROS 2 Robot Arm Trajectory Preview

ROS 2 Jazzy workspace for a generic six-axis arm and the GP8 arm. The trajectory
preview plans and visualizes in RViz; a separate GP8 package executes a
pick-and-place demo in Gazebo Sim through `gz_ros2_control`.

Requirements: Ubuntu 24.04, ROS 2 Jazzy, Python 3, NumPy, SciPy, and PyYAML.
Matplotlib is needed only for optional trajectory plots.

## Build and launch GP8

```bash
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_arm_kinematics robot_arm_trajectory robot_arm_description
source install/setup.bash
ros2 launch robot_arm_description gp8_trajectory_preview.launch.py
```

To use the editable RViz start and goal markers, keep the default
`pose_input_mode:=rviz`. Select RViz's **Interact** tool to move the markers.
Right-click a marker to access planning, reset, gripper, and workspace actions.
Planning is explicit; the arm does not start a trajectory on launch.

Use `pose_input_mode:=yaml` to load the fixed GP8 poses and planner defaults
from `src/robot_arm_trajectory/robot_arm_trajectory/config/gp8_preview_poses.yaml`.
In YAML mode the pose markers are not draggable, but their context menu remains
available. To display a different pose config, edit the GP8 config file or
launch the shared `trajectory_preview.launch.py` with a custom `pose_config`.

## Choose a path method

The marker menu offers three approaches:

| RViz menu | YAML value | Behavior |
| --- | --- | --- |
| **Plan: Cartesian Line** | `cartesian_linear` | Tool position follows a straight line; orientation uses quaternion SLERP. Sequential IK solves the intermediate poses. |
| **Plan: Raised Cartesian Arc** | `cartesian_arc` | Tool follows a raised quadratic arc; `cartesian_arc_height` sets the rise in metres. Intermediate poses must still be reachable. |
| **Plan: Joint Space** | `joint_space` | IK solves the goal, then joint angles are interpolated. The tool path may curve and is not constrained to a line or arc. |

Cartesian line and arc paths refine difficult segments up to
`max_cartesian_refinement_depth`, keeping individual IK joint steps below
`max_joint_waypoint_step` when possible. IK retries alternate deterministic
seeds for selected start and joint-space goal poses. A failure can still mean
the endpoint or a Cartesian intermediate is unreachable, or lies near a
singularity. None of the path methods performs collision checking.

## Optional trajectory plots

Start the plot window only when desired:

```bash
ros2 launch robot_arm_description gp8_trajectory_preview.launch.py plot_trajectory:=true
```

After a trajectory validates, a Matplotlib window plots all arm joint
positions in degrees, velocities in rad/s, accelerations in rad/s², and jerk
in rad/s³ against time. The plot updates after each successful plan. If the
selected timing method does not provide jerk, the jerk panel says so. Plotting
is disabled by default (`plot_trajectory:=false`).

## Approximate reachable workspace

Right-click either pose marker and select **Show Approximate Reachable
Workspace**. RViz displays TCP points computed by sampling joint values
within the configured limits. Select **Hide Reachable Workspace** to remove
the cloud.

GP8 launch defaults to 12,000 samples. Change density on launch, for example:

```bash
ros2 launch robot_arm_description gp8_trajectory_preview.launch.py workspace_samples:=30000
```

More samples give a denser cloud but take longer to calculate and render. This
is a sampled position envelope: it does not show every reachable point, test
tool orientations, or account for self-collision or obstacles.

## Other RViz controls

- **Open Gripper** and **Close Gripper** publish the GP8 finger positions.
- **Reset Start Pose**, **Reset Goal Pose**, and **Reset Both Poses** restore
  the model's default marker poses.
- **joint_state_publisher:=true** opens the manual joint slider GUI; leave it
  false while preview playback is driving `/joint_states`.
- `preview_rate` sets playback updates per second; `looping:=true` plays the
  trajectory forward and then backward.
- `marker_publish_rate` sets how often the displayed reference and actual
  Cartesian paths are refreshed.
- `use_sim_time:=true` makes playback use `/clock` instead of wall time.

For the generic arm, use `trajectory_preview.launch.py`. Its defaults load the
generic URDF, kinematics config, and `preview_poses.yaml`. Both launch files
accept `plot_trajectory`, `workspace_samples`, `preview_rate`, `looping`, and
`pose_input_mode` as applicable.

The shared `trajectory_preview.launch.py` also exposes `marker_publish_rate`,
`use_sim_time`, `model`, `rviz_config`, `robot_config`, and `pose_config`.
GP8's wrapper selects the GP8 model and configs, and exposes `preview_rate`,
`looping`, `joint_state_publisher`, `plot_trajectory`, `workspace_samples`,
and `pose_input_mode`.

## Trajectory settings

The GP8 defaults live in `gp8_preview_poses.yaml`; generic defaults live in
`preview_poses.yaml`. The `trajectory` section supports:

- `path_method`: `cartesian_linear`, `cartesian_arc`, or `joint_space`.
- `cartesian_arc_height`: arc rise in metres.
- `num_cartesian_waypoints`: initial path discretization; adaptive refinement
  may add Cartesian points.
- `duration` and `num_samples`: trajectory duration and output samples.
- `interpolation_method`: `linear`, `cubic_spline`, or
  `quintic_polynomial` for the joint waypoints produced by any path method.
- `time_parameterization_method`: `quintic_scaling`, `trapezoidal`, or
  `seventh_order`; this assigns timestamps and derivatives to the interpolated
  joint path. `s_curve` is a deprecated alias for `seventh_order`, not a
  conventional constant-jerk S-curve.
- `ik_solver`: `damped_least_squares`, `jacobian_transpose`, or `pseudoinverse`.
- `max_joint_waypoint_step` and `max_cartesian_refinement_depth`: continuity
  threshold and maximum adaptive subdivision depth.
- `cartesian_position_tolerance` and `cartesian_orientation_tolerance`:
  Cartesian tracking validation tolerances.
- `interpolation_options` and `time_parameterization_options`: method-specific
  dictionaries.

Interpolation and time parameterization are shared by all path methods. Both
produce a time-parameterized joint trajectory that is checked for joint
limits, velocity limits, and applicable Cartesian tracking before playback.

## Gazebo pick and place

Build and launch the separate simulation package:

```bash
colcon build --packages-up-to robot_arm_gazebo
source install/setup.bash
ros2 launch robot_arm_gazebo gp8_pick_place.launch.py
```

RViz provides Initial, Pick, and Place markers. Select **Interact**, move the
markers, and choose **Plan and Execute Pick & Place** from a marker menu. Set
`pose_input_mode:=yaml` to use the three poses in the package YAML instead.
The same YAML selects Cartesian line, raised arc, or joint-space planning,
interpolation, timing, IK, and approach height for each segment. See the
[`robot_arm_gazebo` guide](src/robot_arm_gazebo/README.md) for the full option
list and grasp behavior.

## Package guides

- [`robot_arm_description`](src/robot_arm_description/README.md): launch files,
  model files, and RViz configuration.
- [`robot_arm_kinematics`](src/robot_arm_kinematics/README.md): standard DH
  models, joint limits, FK, Jacobian, and IK solvers.
- [`robot_arm_trajectory`](src/robot_arm_trajectory/README.md): path planning,
  interpolation, timing, validation, plotting, and preview details.
- [`robot_arm_gazebo`](src/robot_arm_gazebo/README.md): GP8 Gazebo Sim pick and
  place bringup, RViz pose selection, controller execution, and YAML options.
