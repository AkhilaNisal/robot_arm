# Trajectory generation

Interpolation describes the joint path as a function of a dimensionless path
parameter `s`. Time parameterization describes how `s` changes over time. They
are separate so any supported path can be paired with any supported profile.

```python
trajectory = generate_joint_trajectory(
    joint_waypoints,
    interpolation_method="cubic_spline",
    time_parameterization="quintic_scaling",
    duration=4.0,
    num_samples=101,
)
```

Interpolation choices are `linear`, `cubic_spline` and `quintic_polynomial`.
Linear interpolation is piecewise affine: position is continuous, but its
derivative can jump at knots. Cubic splines use SciPy `CubicSpline`, default to
not-a-knot boundary conditions and provide continuous position, velocity and
acceleration. Quintic Hermite segments satisfy per-waypoint position,
path-parameter velocity and acceleration. If derivative arrays are omitted,
they are computed analytically from a global not-a-knot cubic spline. The
piecewise quintic then reproduces that cubic spline to numerical precision and
is C2 across knots. Explicit derivatives retain general quintic Hermite
behavior; shared position, velocity and acceleration give C2 continuity.
Neither cubic splines nor ordinary piecewise quintic Hermite interpolation
guarantees C3 continuity. Joint jerk can jump at waypoints; a C3-continuous
quintic spline method is not currently implemented.

Timing choices are `quintic_scaling`, `trapezoidal` and `seventh_order`. Quintic
scaling uses `10u^3 - 15u^4 + 6u^5`. The trapezoidal profile has constant
acceleration, optional cruise and constant deceleration; acceleration is
discontinuous at phase changes, including the initial acceleration phase and
the final deceleration phase. Explicit velocity and acceleration limits are
checked against the analytically feasible acceleration-time interval.
`seventh_order` is the polynomial `35u^4 - 84u^5 + 70u^6 - 20u^7`; its
maximum absolute path jerk is `52.5 / duration^3`. It has zero endpoint
velocity, acceleration and jerk, but it is not a conventional seven-phase
constant-jerk S-curve. The old `s_curve` registry name and `SCurveProfile`
import remain temporarily as deprecated aliases. Configure its path jerk cap
through `time_parameterization_options={"max_jerk": ...}`; an infeasible
duration is rejected. A conventional piecewise constant-jerk S-curve is not
implemented.

Joint velocities, accelerations and jerks are calculated using the chain rule.
Joint position, velocity, acceleration and jerk limits can be supplied to the
generator. Acceleration and jerk limits are deliberately not assigned generic
robot values. Enabling enforcement without configured limits raises an error.
Position limits check polynomial stationary points and segment endpoints using
floating-point root solvers. Derivative enforcement checks generated samples
and bounded numerical extrema searches between them for the supported timing
profiles; these are numerical estimates, not formal interval certificates.
The independent validator checks position and derivative constraints at the
trajectory's supplied samples, so it cannot certify inter-sample limits by
itself. The trapezoidal profile has acceleration discontinuities and provides
no jerk output, so it cannot support jerk-limit enforcement.
The generic RViz preview config is
`robot_arm_trajectory/config/preview_poses.yaml`; the GP8 config is
`robot_arm_trajectory/config/gp8_preview_poses.yaml`. Each contains its
Cartesian start and goal poses and planner defaults. The general Python
trajectory APIs remain configuration-independent; pass their path,
interpolation, timing, and constraint options directly to the generator.

For the GP8 mesh model, build and launch the model-specific preview:

```bash
colcon build --packages-select robot_arm_kinematics robot_arm_trajectory robot_arm_description
source install/setup.bash
ros2 launch robot_arm_description gp8_trajectory_preview.launch.py
```

Add `plot_trajectory:=true` to open a separate Matplotlib window after a
trajectory validates. It plots all joint positions (degrees), velocities
(rad/s), accelerations (rad/s²), and jerks (rad/s³) versus time. It updates
with each newly validated trajectory. The option defaults to `false`; jerk is
shown as unavailable if the selected time-parameterization method supplies no
jerk values.

Move the Start Pose and Goal Pose interactive markers in RViz, then select
**Plan: Cartesian Line**, **Plan: Raised Cartesian Arc**, or
**Plan: Joint Space** from either marker menu. The same menu contains
**Open Gripper** and **Close Gripper**. YAML users can select `path_method`
as `cartesian_linear`, `cartesian_arc`, or `joint_space` in the pose config;
`cartesian_arc_height` sets the arc's rise in metres. GP8 forward kinematics
and Jacobian calculations use the standard DH table and fixed base/tool
transforms in `robot_arm_kinematics/config/gp8.yaml`.
The marker menu also offers **Show Approximate Reachable Workspace**. This
samples joint-limit configurations and displays their TCP positions as RViz
points; select **Hide Reachable Workspace** to clear them. On the GP8 launch,
`workspace_samples` defaults to 12,000; increase it for a denser cloud, for
example `workspace_samples:=30000`. The shared launch defaults to 3,000. More
samples increase sampling and rendering time. The cloud is a sampled position
envelope; it is not an exhaustive workspace solution, does not show reachable
tool orientations, and does not account for self-collision or obstacles.

The GP8 path menu choices are Cartesian Line, Raised Cartesian Arc, and Joint
Space. The generic and GP8 YAML files include the same planner options:

- `path_method`: `cartesian_linear`, `cartesian_arc`, or `joint_space`.
- `cartesian_arc_height`: arc rise above the midpoint, in metres.
- `num_cartesian_waypoints`: initial discretization. Cartesian line and arc
  planners can add points through adaptive refinement.
- `duration`, `num_samples`: total motion time and sampled output count.
- `interpolation_method`: `linear`, `cubic_spline`, or
  `quintic_polynomial`, applied to the joint waypoints from every path type.
- `time_parameterization_method`: `quintic_scaling`, `trapezoidal`, or
  `seventh_order`. The old `s_curve` name is a deprecated alias for the
  seventh-order polynomial.
- `ik_solver`: `damped_least_squares`, `jacobian_transpose`, or `pseudoinverse`.
- `max_joint_waypoint_step`, `max_cartesian_refinement_depth`: joint
  continuity threshold and subdivision depth for Cartesian IK.
- `cartesian_position_tolerance`, `cartesian_orientation_tolerance`: path
  tracking validation thresholds.
- `interpolation_options`, `time_parameterization_options`: dictionaries for
  settings specific to the chosen algorithms.

Use **Plan: Cartesian Line** when tool motion must be straight; use **Plan:
Raised Cartesian Arc** for a raised tool route; use **Plan: Joint Space** when
the endpoint matters and the intermediate tool path can curve. Cartesian
approaches solve IK along every intermediate pose and may fail near singular
or unreachable sections. Joint Space solves the goal and interpolates joint
angles, so it does not follow a prescribed Cartesian route. All approaches
enforce configured joint position and velocity limits and run trajectory
validation; none performs collision checking.

The generator expects finite waypoint rows with consistent joint order and
units (normally radians). It does not do collision checking or dynamic torque
planning. The piecewise linear derivative is defined per segment and is
ambiguous exactly at a knot; cubic spline is smoother. The seventh-order
profile parameterizes normalized path only, so its jerk limit is a
path-parameter jerk limit, not a per-joint jerk constraint.

Run the package tests from a sourced ROS 2 workspace with:

```bash
colcon test --packages-select robot_arm_trajectory
colcon test-result --verbose
```

Existing scripts remain under `scripts/`, including
`test_joint_trajectory.py` and `test_quintic_time_scaling.py`.

## Cartesian-to-joint pipeline

The implementation is divided by path space: Cartesian path generation and
waypoint IK are in `robot_arm_trajectory/cartesian_space/`; endpoint IK and
joint-space planning plus joint trajectory interpolation are in
`robot_arm_trajectory/joint_space/`. `pipeline.py` selects a path planner and
then passes its joint waypoints to the shared time parameterization flow.

`generate_cartesian_joint_trajectory()` selects Cartesian line, raised arc,
or joint-space planning, then uses the shared joint trajectory API. By default
it computes the start end-effector pose from the robot model and initial joint
configuration. An optional `start_pose` transform instead supplies a selected
Cartesian start.
Cartesian modes create the selected tool path and solve each waypoint using
the prior IK solution as the next seed. Joint-space mode solves the endpoint
and interpolates between joint configurations. All modes then interpolate and
time-parameterize the joint waypoints.

```python
from robot_arm_trajectory import generate_cartesian_joint_trajectory

result = generate_cartesian_joint_trajectory(
    robot_model=robot,
    ik_solver=ik_solver,
    q_initial=q_start,
    goal_pose=goal_pose,
    num_waypoints=50,
    duration=4.0,
    num_samples=401,
    interpolation_method='cubic_spline',
    time_parameterization='quintic_scaling',
    enforce_position_limits=True,
    enforce_velocity_limits=True,
)

if result.success:
    trajectory = result.joint_trajectory
else:
    print(result.failed_stage, result.message)
```

The initial configuration must have one finite, in-limit value per robot joint;
the goal and optional start must be valid 4x4 homogeneous poses. The IK solver
must use a robot with matching degrees of freedom and joint position limits.
The returned
`CartesianJointTrajectoryResult` contains `success`, `cartesian_path`,
`ik_result`, `joint_trajectory`, `message`, and `failed_stage`. Invalid inputs
raise `ValueError`; IK or trajectory-generation failures return a structured
failure with `joint_trajectory=None`, so no partial trajectory is exposed.
The lower-level path, IK, and joint trajectory functions remain available.

The RViz preview supports editable marker poses (`pose_input_mode:=rviz`) and
poses loaded from one YAML file (`pose_input_mode:=yaml`). The YAML schema and
launch examples are documented in the workspace [README](../../README.md).
In YAML mode, its `trajectory` section selects planner defaults as well as
poses. The generic shipped profile uses C2 cubic-spline joint interpolation,
quintic time scaling, damped-least-squares IK, 50 Cartesian waypoints, and 401
trajectory samples. GP8 uses its own values in `gp8_preview_poses.yaml`,
including 30 initial Cartesian waypoints and 121 trajectory samples. Position,
velocity, and applicable Cartesian tracking checks remain enabled by the
preview node.

## RViz preview topics and launch settings

The preview publishes the planned TCP line strip on
`/trajectory_preview/reference_path` and the TCP path reconstructed from the
current joint states on `/trajectory_preview/actual_path`. Both are
`visualization_msgs/msg/Marker`; the RViz config displays each as a colored
line. The preview also publishes sampled workspace points on
`/trajectory_preview/reachable_workspace` after the workspace menu action.
When plotting is enabled, joint trajectory arrays are sent as JSON in
`std_msgs/msg/String` on `/trajectory_preview/trajectory_data` and consumed by
the `trajectory_plot` executable.

The shared `trajectory_preview.launch.py` accepts `model`, `rviz_config`,
`robot_config`, `pose_config`, `pose_input_mode`, `preview_rate`,
`marker_publish_rate`, `looping`, `use_sim_time`, `joint_state_publisher`,
`plot_trajectory`, and `workspace_samples`. `gp8_trajectory_preview.launch.py`
selects the GP8 model, GP8 DH YAML, GP8 pose YAML and the standard trajectory
preview RViz file. Its exposed arguments are `pose_input_mode` (`rviz`),
`plot_trajectory` (`false`), `workspace_samples` (`12000`), `preview_rate`
(`30.0`), `looping` (`false`), and `joint_state_publisher` (`false`). For
example:

```bash
ros2 launch robot_arm_description gp8_trajectory_preview.launch.py \
  pose_input_mode:=yaml plot_trajectory:=true workspace_samples:=30000
```

Use **Open Gripper** or **Close Gripper** from either pose marker menu to set
the GP8 finger joint positions. The RViz preview models these positions; it
does not simulate contact. The trajectory preview and Gazebo pick/place are
separate launches. See the `robot_arm_gazebo` guide for controller execution,
the Gazebo gripper, Box A/Box B, and the planning progress panel.

## Independent trajectory validation

Trajectory validation is a separate, read-only operation. It checks numerical
array validity, joint limits, optional Cartesian tracking, and Jacobian
singularity proximity. Acceleration and jerk limits are only checked when
provided. A singularity warning does not by itself fail validation.

```python
from robot_arm_trajectory import validate_trajectory

report = validate_trajectory(
    result.joint_trajectory,
    robot,
    cartesian_path=result.cartesian_path,
    acceleration_limits=[4.0] * robot.dof,
    jerk_limits=[30.0] * robot.dof,
    cartesian_position_tolerance=0.002,
    cartesian_orientation_tolerance=0.02,
)

if not report.success:
    print(report.errors)
print(report.summary)
```

`TrajectoryValidationResult` contains `success`, detailed `checks`, `errors`,
`warnings`, and a `summary` of per-joint extrema and Cartesian tracking
statistics. The validator maps each trajectory time through the selected time
parameterization (default `quintic_scaling`) and interpolates uniformly spaced
Cartesian path waypoints using linear position and quaternion SLERP. Pass the
matching `time_parameterization` and options when validating a trajectory
generated with non-default timing. Because trajectory objects do not store
their timing method, the validator cannot infer that choice automatically.
