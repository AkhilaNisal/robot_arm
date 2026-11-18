# Trajectory planning and preview

`robot_arm_trajectory` provides Cartesian and joint-space path planning,
joint interpolation, time parameterization, limit validation, and the RViz
preview nodes. Path planning, interpolation, and timing are separate stages:
the path method creates joint-space knots, the interpolator fills in joint
values between them, and the timing profile maps the path to time.

## Build and run the standalone preview

```bash
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to robot_arm_description
source install/setup.bash
ros2 launch robot_arm_description gp7_trajectory_preview.launch.py
```

The RViz-only preview uses `robot_state_publisher`, RViz and the
`trajectory_preview` node; it does not start Gazebo or send commands to a
physical robot. To run the waypoint mode:

```bash
ros2 launch robot_arm_description gp7_trajectory_preview.launch.py \
  planning_mode:=joint_space_waypoints
```

For marker controls, launch arguments, and launch files, see the
[`robot_arm_description` guide](../robot_arm_description/README.md).

## Path methods

| Method | YAML key | Path behavior and use |
| --- | --- | --- |
| Cartesian line | `cartesian_linear` | TCP position follows a straight segment; IK is solved at intermediate TCP poses. Use when straight tool motion matters. |
| Raised Cartesian arc | `cartesian_arc` | TCP follows a raised route; `cartesian_arc_height` specifies rise in metres. Use when a path should clear an object or surface. |
| Endpoint joint space | `joint_space` | Solve endpoint IK and interpolate joint coordinates. The TCP path can curve; use when only endpoints matter. |
| Ordered pose waypoints | `joint_space_waypoints` | Solve IK sequentially for initial pose, each user knot, and goal, seeding from the preceding solution. The TCP is constrained at the listed poses and may curve between them. |

Cartesian IK uses the prior solution as the next seed and may adaptively
subdivide when joint steps are too large or convergence fails. A reachable
goal does not imply every point on a Cartesian line/arc is reachable. A
waypoint route gives more shape control than endpoint joint space, but every
knot needs valid IK.

## Interpolation and timing

Interpolation choices are `linear`, `cubic_spline`, and
`quintic_polynomial`:

- `linear` is piecewise affine. Joint position is continuous, while velocity
  may jump at knots.
- `cubic_spline` uses SciPy `CubicSpline`; the default boundary type is
  `not-a-knot`. It provides continuous position, velocity, and acceleration.
- `quintic_polynomial` uses quintic Hermite segments. With inferred
  derivatives it reproduces the global cubic spline to numerical precision;
  explicitly supplied shared position/velocity/acceleration knot values give
  C2 continuity. These methods do not guarantee C3 continuity, so jerk may
  jump at knots.

Time profiles are `quintic_scaling`, `trapezoidal`, and `seventh_order`:

- `quintic_scaling` uses `10u^3 - 15u^4 + 6u^5` with zero endpoint velocity
  and acceleration.
- `trapezoidal` has constant-acceleration, optional cruise, and
  constant-deceleration phases. Acceleration jumps at phase boundaries and
  this profile does not provide jerk values.
- `seventh_order` uses `35u^4 - 84u^5 + 70u^6 - 20u^7`; it has zero endpoint
  velocity, acceleration, and jerk. It is not a conventional seven-phase
  constant-jerk S-curve.

The old `s_curve` registry key and `SCurveProfile` import are deprecated
aliases for `seventh_order`. The profile's `max_jerk` option is a path
parameter jerk cap, not a per-joint jerk cap. An infeasible requested duration
is rejected when the cap is enabled.

## Planner YAML options and accepted domains

The `trajectory` mapping in
`robot_arm_trajectory/config/preview_poses.yaml` and
`gp7_preview_poses.yaml` configures non-waypoint planning. The waypoint file
also has a `joint_space_trajectory` mapping. These values are shared by the
preview and, where applicable, Gazebo planner configs.

| Key | Units / accepted values | Effect |
| --- | --- | --- |
| `path_method` | `cartesian_linear`, `cartesian_arc`, `joint_space` | Selects the regular path planner. Waypoint preview selects `joint_space_waypoints` via the launch mode/menu. |
| `cartesian_arc_height` | metres; finite | Vertical rise for the arc planner. |
| `num_cartesian_waypoints` | integer ≥ 2 | Initial path resolution. More points add IK work; adaptive refinement may add more. |
| `duration` | positive finite seconds | Total trajectory duration (per segment in Gazebo). |
| `num_samples` | integer ≥ 2 | Number of output samples (per leg in waypoint/Gazebo execution). Higher values make smoother playback data and increase validation/render work. |
| `interpolation_method` | `linear`, `cubic_spline`, `quintic_polynomial` | Joint interpolation algorithm. |
| `interpolation_options` | mapping | Algorithm-specific options; spline accepts `bc_type`. |
| `time_parameterization_method` | `quintic_scaling`, `trapezoidal`, `seventh_order` | Timing profile. `s_curve` remains a deprecated alias. |
| `time_parameterization_options` | mapping | Profile-specific options, e.g. trapezoid limits or seventh-order `max_jerk`; see source profile classes. |
| `ik_solver` | `damped_least_squares`, `jacobian_transpose`, `pseudoinverse` | Full-pose IK method. |
| `ik_max_iterations` | integer ≥ 1 | Iteration cap per IK attempt. A lower cap can reduce time but may fail harder targets. |
| `ik_position_tolerance` | positive finite metres | IK TCP position convergence tolerance. |
| `ik_orientation_tolerance` | positive finite radians | IK orientation convergence tolerance. |
| `ik_restarts` | integer ≥ 0 | Additional IK seeds/attempts where supported; increases robustness and planning work. |
| `max_joint_waypoint_step` | positive finite radians | Preferred maximum change between successive Cartesian IK joint knots; lower values can trigger more refinement. |
| `max_cartesian_refinement_depth` | integer ≥ 0 | Maximum recursive refinement depth for Cartesian paths. Larger values can improve continuity/reachability at extra cost. |
| `cartesian_position_tolerance` | positive finite metres | Maximum allowed path tracking deviation in applicable validation. |
| `cartesian_orientation_tolerance` | positive finite radians | Maximum allowed path orientation error in applicable validation. |

Reduce `num_cartesian_waypoints`, `max_cartesian_refinement_depth`, IK
iterations/restarts, or output samples to make planning lighter; restore
resolution and retry budget if tracking or reachability checks fail. These
settings affect calculation and preview fidelity, not the underlying path
method. Configured joint position and velocity limits remain enforced by the
pipeline.

## Standalone waypoint YAML and RViz editing

The shipped route is
`robot_arm_trajectory/config/gp7_joint_waypoints.yaml`. Its structure is:

```yaml
poses:
  initial:
    position: [0.63, 0.0, 0.81]
    orientation_xyzw: [0.7071, 0.0, 0.7071, 0.0]
  goal:
    position: [0.62, 0.25, 0.23]
    orientation_xyzw: [0.0, 1.0, 0.0, 0.0]
joint_space_trajectory:
  duration: 5.0
  num_samples: 121
  waypoints:
    - position: [0.62, -0.12, 0.48]
      orientation_xyzw: [0.0, 1.0, 0.0, 0.0]
trajectory:
  # Settings for Cartesian line, arc and endpoint joint-space menu methods.
  path_method: cartesian_linear
  num_cartesian_waypoints: 16
  duration: 4.0
  num_samples: 81
```

Pose coordinates are metres in `base_link`; orientation is normalized
`[x,y,z,w]`. `waypoints` is an ordered list and can be empty. Add a pose by
copying the list-entry schema; maintain reachability and a smooth route. In
RViz, use Start's **Add Joint Space Waypoint** menu action, move the marker,
and plan with **Plan: Joint Space Waypoints**. Use the waypoint marker menu to
remove a knot. Interactive changes last for the current process; edit YAML to
make them persistent.

The waypoint route applies one requested duration across the full route, and
`num_samples` sets samples across that route. IK is computed sequentially from
initial through knots to goal, using each solution as the next seed. The
interpolated tool path is not guaranteed to pass exactly through straight
segments joining pose markers.

The waypoint YAML's top-level `trajectory` section supplies tuned settings
for the alternate Cartesian line, arc, and endpoint joint-space methods. The
shipped GP7 profile uses 16 initial Cartesian waypoints, 81 samples, a 250
iteration cap, 2 restarts and refinement depth 5. This shorter budget helps
preview planning time; it can reduce the available resolution/retry budget
for difficult targets.

## Package modules, nodes and scripts

| Path / executable | Responsibility |
| --- | --- |
| `cartesian_space/cartesian_path.py` | Cartesian line and raised-arc pose path generation. |
| `cartesian_space/cartesian_ik.py` | Sequential IK and adaptive Cartesian refinement. |
| `joint_space/planner.py` | Endpoint IK and joint-space waypoint planning helpers. |
| `joint_space/joint_trajectory.py` | Joint interpolation, derivatives, and position/velocity limit enforcement. |
| `interpolation/linear.py` | Piecewise linear joint interpolation. |
| `interpolation/cubic_spline.py` | Cubic spline interpolation. |
| `interpolation/quintic_polynomial.py` | Quintic Hermite segments and derivative handling. |
| `time_parameterization.py`, `time_parameterization/` | Time profile registry and quintic, trapezoidal, seventh-order profiles. |
| `pipeline.py` | Shared orchestration from path method through IK, interpolation and timing. |
| `validation.py` | Independent numerical trajectory, joint-limit and optional Cartesian checks. |
| `interactive_poses.py` | RViz interactive pose/waypoint marker utilities. |
| `preview_utils.py` | Pose conversions, YAML and preview helper functions. |
| `trajectory_preview` | ROS node: plan/validate, marker interactions, joint-state playback, TCP path/workspace publishing. |
| `trajectory_plot` | ROS node: subscribes to trajectory JSON and plots joint position/velocity/acceleration/jerk using Matplotlib. |
| `test/` | Package automated test suite; run only when verification is desired. |

`trajectory_preview` topics include `/trajectory_preview/reference_path`,
`actual_path`, `reachable_workspace`, and `trajectory_data`. The independent
validator checks supplied samples; position limits also inspect polynomial
stationary points, and derivative enforcement performs bounded numerical
extrema searches between samples. Derivative extrema are numerical estimates,
not formal interval certificates. The planner does not check collisions,
torques, or full dynamic feasibility.

## Python API sketch

```python
from robot_arm_trajectory import generate_cartesian_joint_trajectory

result = generate_cartesian_joint_trajectory(
    robot_model=robot,
    ik_solver=ik_solver,
    q_initial=q_start,                 # radians, one in-limit value per joint
    goal_pose=goal_pose,               # 4x4 homogeneous transform
    path_method='cartesian_linear',
    num_waypoints=50,
    duration=4.0,
    num_samples=401,
    interpolation_method='cubic_spline',
    time_parameterization='quintic_scaling',
)
if result.success:
    trajectory = result.joint_trajectory
else:
    print(result.failed_stage, result.message)
```

Use `validate_trajectory()` to check an existing trajectory separately. Pass
the same time-parameterization name/options used to generate it; trajectory
objects do not infer which profile generated their timestamps. Python APIs
expect radians, finite values, matching dimensions and valid 4×4 homogeneous
poses. Invalid inputs raise `ValueError`; planner failures return structured
failure results.

## Interpreting timing and validation

Joint velocity, acceleration and jerk follow the chain rule. Position limits
check polynomial stationary points and endpoints. Derivative enforcement
checks samples plus bounded numerical searches for local extrema; it is not a
formal certificate. The independent validator checks array validity, joint
limits, optional Cartesian tracking, and Jacobian singularity proximity. A
singularity warning does not necessarily fail validation. Acceleration and
jerk limits are only checked when supplied, and enabling their enforcement
without limits is an error. Trapezoidal timing has acceleration jumps and no
jerk output, so jerk-limit enforcement is unsupported for that profile.

No path method performs collision checking, torque analysis, or a full
dynamics feasibility analysis. Inspect the generated reference path and
validate target clearance in the model/workcell.
