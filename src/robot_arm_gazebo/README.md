# GP8 Gazebo pick and place

This package starts Gazebo Sim with the GP8 arm, a small cube, a work surface,
`gz_ros2_control` controllers, and an RViz pose selector. It reuses
`robot_arm_kinematics` for GP8 FK/IK and `robot_arm_trajectory` for Cartesian
line, raised arc, and joint-space planning, interpolation, timing, and
trajectory validation.

## Build and launch

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to robot_arm_gazebo
source install/setup.bash
ros2 launch robot_arm_gazebo gp8_pick_place.launch.py
```

## Assignment workcell

Launch the separate Box A / Box B assignment world with:

```bash
ros2 launch robot_arm_gazebo gp8_assignment_pick_place.launch.py
```

The planner publishes the concatenated reference TCP path and measured TCP
path to `/trajectory_preview/reference_path` and
`/trajectory_preview/actual_path` for the configured RViz displays. To open
joint position, velocity, acceleration, and jerk graphs, enable the optional
plot window:

```bash
ros2 launch robot_arm_gazebo gp8_assignment_pick_place.launch.py plot_trajectory:=true
```

Leave `plot_trajectory:=false` (the default) to run without the graph window.

This starts a ground plane, Box A, the GP8, and Box B. Box A is composed into
the Gazebo world from YAML before Gazebo starts, so it appears as a world model
in the Gazebo Entity Tree. RViz also displays a Box A marker from the same pose.
The task configuration
is `config/gp8_assignment.yaml`. Box A is set to 30 × 30 × 20 cm and Box B to
5 × 5 × 5 cm. The selected reachable Box A centre is `[0.62, 0.25, 0.10]` m;
the GP8 cannot reach the task sheet's `[1, 1, 0]` m location from the origin.
Box B starts at `[0.62, -0.25, 0.025]` m. The Place target is calculated from
Box A's pose and dimensions so Box B lands at the centre of its top surface.

Use RViz **Interact** to adjust the Initial, Pick, and Place markers and choose
**Plan and Execute Pick & Place** from a marker menu. Set
`pose_input_mode:=yaml` to make the markers read-only. The default assignment
planner uses a raised Cartesian arc and matching Pick/Place orientations to
keep Box B upright during the transfer.

Edit `objects.box_a` or `objects.box_b` in `gp8_assignment.yaml` to change the
box poses, orientations, or dimensions. Box B must remain a cube. Its default
Pick marker follows the configured Box B pose, and the Place marker is computed
from Box A's top surface. Restart the launch after editing the file.

Gazebo opens with the GP8, a 4.5 cm cube sized for the gripper, and a table. RViz provides Initial,
Pick, and Place markers. Select **Interact** to move them, then right-click a
marker and choose **Plan and Execute Pick & Place**. The sequence opens the
gripper, moves to the selected initial pose, approaches and picks the cube,
lifts it, transfers it, releases it at Place, retreats, and returns to Initial.
Each motion segment is planned and validated before any motion starts. If a
segment cannot be planned or validated, the sequence stops before execution.
The table is positioned at `x=0.72 m` with its near edge at `x=0.32 m`, away
from the base rotation area. Default Pick and Place targets are shifted outward
to remain over this table. The Gazebo URDF includes a fixed world-to-base joint
so gravity and arm motion cannot tip or slide the robot base.

Use fixed YAML poses instead of draggable markers with:

```bash
ros2 launch robot_arm_gazebo gp8_pick_place.launch.py pose_input_mode:=yaml
```

The same sequence can be triggered without the marker menu:

```bash
ros2 service call /gp8_pick_place/plan_and_execute std_srvs/srv/Trigger {}
```

## Pose and planner configuration

The default config is `config/gp8_pick_place.yaml`. Pose positions are metres
in `base_link`; orientations are normalized ROS `[x, y, z, w]` quaternions.
For the shipped vertical grasp, Pick and Place positions represent the cube
centre. Starting the sequence moves the cube to the selected Pick marker, so
RViz edits and the simulated box stay aligned. The box is attached to the live
measured TCP after the close action only when the TCP is within
`grasp_tolerance` of the box; otherwise the sequence stops with a grasp error.
While grasped, the box follows the measured arm joint state through a throttled
Gazebo pose update. At Place, it is set to the selected target and released.
The config has `initial`, `pick`, and `place` poses, plus these `pick_place`
settings:

- `pose_input_mode`: `rviz` or `yaml`. The launch argument overrides this
  value; `rviz` permits editing and `yaml` makes markers read-only. Its
  default is read from this YAML setting.
- `path_method`: `cartesian_linear`, `cartesian_arc`, or `joint_space`.
  The chosen method is used for each segment. Cartesian paths constrain the
  tool to a line or raised arc; joint space interpolates joint configurations.
- `cartesian_arc_height`: arc rise in metres.
- `num_cartesian_waypoints`: initial path discretization.
- `duration`, `num_samples`: duration in seconds and trajectory sample count
  for each segment.
- `interpolation_method`: `linear`, `cubic_spline`, or `quintic_polynomial`.
- `interpolation_options`: options for the selected interpolator, such as
  spline `bc_type`.
- `time_parameterization_method`: `quintic_scaling`, `trapezoidal`, or
  `seventh_order` (`s_curve` remains a deprecated alias).
- `time_parameterization_options`: options for the timing profile.
- `ik_solver`: `damped_least_squares`, `jacobian_transpose`, or `pseudoinverse`.
- `max_joint_waypoint_step`, `max_cartesian_refinement_depth`: Cartesian IK
  continuity threshold in radians and adaptive subdivision depth.
- `approach_height`: vertical clearance above Pick and Place in metres.
- `gripper_open_position`, `gripper_closed_position`: finger joint positions
  in metres (the shipped URDF uses 0.0 m open and 0.015 m closed).
- `cube_initial_pose`, `cube_place_pose`: optional cube center XYZ positions
  in the Gazebo world frame. If omitted, the YAML Pick and Place positions are
  used. When RViz markers are moved, the selected Pick marker becomes the cube
  position for the next sequence.
- `cube_size`: cube width in metres, matching its collision and visual geometry.
- `cube_grasp_follow_rate`: maximum TCP-follow update rate in Hz while grasped.
- `grasp_offset`: cube-centre offset from the TCP in TCP coordinates (metres).
- `grasp_tolerance`: maximum TCP-to-cube-centre distance allowed at grasp.

Use a different pose config with `pose_config:=/absolute/path/to/file.yaml`.
The YAML top level must contain `poses.initial`, `poses.pick`, `poses.place`,
and `pick_place` settings.

## Launch arguments

- `pose_input_mode:=rviz|yaml`
- `pose_config:=...` selects the three-pose and planner YAML.
- `world:=...` selects an SDF world. The default is `worlds/pick_place.sdf`.

The supported path, interpolation, timing, and IK choices match the existing
trajectory preview. None of the current planners performs collision checking
or torque/dynamic feasibility analysis. The shipped cube attachment is a
deterministic simulation aid: it moves the Gazebo cube with the tool rather
than relying on contact friction, and does not model a physical grasp force.
The shipped cube is 5 cm wide and has a mass of 0.08 kg in the launch SDF.
