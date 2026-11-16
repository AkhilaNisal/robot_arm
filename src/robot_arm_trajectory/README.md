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
The RViz preview's single configuration file is
`robot_arm_trajectory/config/preview_poses.yaml`. It contains the Cartesian
poses and preview planner defaults. The general Python trajectory APIs remain
configuration-independent; pass their interpolation, timing, and constraint
options directly to the generator.

For the GP8 mesh model, build and launch the model-specific preview:

```bash
colcon build --packages-select robot_arm_kinematics robot_arm_trajectory robot_arm_description
source install/setup.bash
ros2 launch robot_arm_description gp8_trajectory_preview.launch.py
```

Move the Start Pose and Goal Pose interactive markers in RViz, then select
**Plan: Cartesian Line**, **Plan: Raised Cartesian Arc**, or
**Plan: Joint Space** from either marker menu. The same menu contains
**Open Gripper** and **Close Gripper**. YAML users can select `path_method`
as `cartesian_linear`, `cartesian_arc`, or `joint_space` in the pose config;
`cartesian_arc_height` sets the arc's rise in metres. GP8 forward kinematics
and Jacobian calculations use the standard DH table and fixed base/tool
transforms in `robot_arm_kinematics/config/gp8.yaml`.

Cartesian paths constrain the tool to a line or raised arc, so they can fail
when intermediate poses are unreachable or IK changes joint branches. Joint
space solves the goal once and interpolates between joint configurations; the
tool may take a curved route. All modes still validate joint limits and the
time-parameterized trajectory. The preview does not perform collision checks.
Line and arc paths now subdivide difficult segments automatically when IK
fails or a joint step exceeds `max_joint_waypoint_step`; the recursion limit is
set by `max_cartesian_refinement_depth`. Start and joint-space goal IK also
retry deterministic alternate joint seeds. Persistent failures can indicate
an unreachable endpoint, an infeasible Cartesian intermediate, or collision
(which this preview does not check).

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

`generate_cartesian_joint_trajectory()` connects the existing Cartesian path,
sequential IK, and joint trajectory APIs. By default it computes the start
end-effector pose from the robot model and initial joint configuration. An
optional `start_pose` transform instead supplies a selected Cartesian start.
It creates a linear position path with quaternion SLERP orientation, solves
each waypoint using the prior IK solution as the next seed, then
time-parameterizes the joint waypoints.

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
the goal and optional start must be valid 4x4 homogeneous poses. The IK solver must use a robot
with matching degrees of freedom and joint position limits. The returned
`CartesianJointTrajectoryResult` contains `success`, `cartesian_path`,
`ik_result`, `joint_trajectory`, `message`, and `failed_stage`. Invalid inputs
raise `ValueError`; IK or trajectory-generation failures return a structured
failure with `joint_trajectory=None`, so no partial trajectory is exposed.
The lower-level path, IK, and joint trajectory functions remain available.

The RViz preview supports editable marker poses (`pose_input_mode:=rviz`) and
poses loaded from one YAML file (`pose_input_mode:=yaml`). The YAML schema and
launch examples are documented in the workspace [README](../../README.md).
In YAML mode, its `trajectory` section selects planner defaults as well as
poses. The shipped profile uses C2 cubic-spline joint interpolation, quintic
time scaling, damped-least-squares IK, 50 Cartesian waypoints, and 401
trajectory samples. Position, estimated velocity, and Cartesian tracking
checks remain enabled by the preview node.

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
