# GP8 Gazebo Sim pick and place

This package launches the GP8 arm in Gazebo Sim with `gz_ros2_control`, ROS 2
joint controllers, RViz pose markers, and a pick/place planner. It uses
`robot_arm_kinematics` for FK/IK and `robot_arm_trajectory` for Cartesian or
joint-space path planning, interpolation, timing and validation. The planner
checks every motion segment before it starts arm execution.

## Build

From the workspace root:

```bash
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to robot_arm_gazebo
source install/setup.bash
```

This builds the Gazebo package and its dependencies, including the custom
`robot_arm_rviz_panel` C++ plugin. Re-source `install/setup.bash` after each
build before launching.

## Standard workcell

```bash
ros2 launch robot_arm_gazebo gp8_pick_place.launch.py
```

The standard SDF contains a ground plane and work surface. The launch spawns
the GP8 and a small cube, then starts the state publisher, Gazebo/ROS bridge,
joint-state broadcaster, arm and gripper controllers, RViz and pick/place
planner. The GP8 base is fixed to the simulated world. The cube has increased
contact friction, but grasp following is handled by the planner's deterministic
Gazebo pose updates, rather than relying on simulated finger friction alone.

## Box A / Box B assignment workcell

```bash
ros2 launch robot_arm_gazebo gp8_assignment_pick_place.launch.py
```

This launch builds a temporary Gazebo world from
`worlds/gp8_assignment.sdf` and `config/gp8_assignment.yaml`. Box A is inserted
as a static Gazebo model and displayed in RViz; Box B is spawned as the cube
the gripper picks up. The current YAML sets Box A's centre to
`[0.62, 0.25, 0.10]` m with size `[0.30, 0.30, 0.20]` m, and Box B's centre to
`[0.62, -0.25, 0.025]` m with size `[0.05, 0.05, 0.05]` m. Those are selected
reachable positions for this workcell, rather than the task sheet's farther
coordinates. The Place pose is computed at the centre of Box A's top surface,
using the two box heights and Box A's orientation.

Edit `objects.box_a` or `objects.box_b` in the assignment YAML to move or
rotate either box or to change its dimensions. Box B must remain a cube. The
Pick marker follows Box B's configured pose when starting in YAML/config mode.
Restart the launch after YAML edits; the assignment world is generated once
at launch. The configuration also has the Initial arm pose and all planner
and grasp settings documented below.

## Pose selection and operation

The RViz configuration is `rviz/gp8_pick_place.rviz`. It displays the robot,
interactive Initial/Pick/Place poses, Box A, the reference TCP path, the
measured TCP path, and the Pick and Place Progress panel. Use the **Interact**
tool to move a pose marker. Open a marker's context menu and select **Plan and
Execute Pick & Place**. The same command can be called as a ROS service:

```bash
ros2 service call /gp8_pick_place/plan_and_execute std_srvs/srv/Trigger {}
```

For the eight motion segments (initial approach, pick approach, descend, lift,
transfer, place, retreat, return), the node plans and validates every segment
before sending any arm goal. If one segment fails planning, validation, or IK,
execution stops before arm motion. On success it opens the gripper, moves to
Initial, approaches Pick, closes the gripper, checks the grasp, lifts and
transfers the box, releases it at Place, retreats and returns to Initial.
Moving the Pick marker also moves the cube to that selected target when the
next sequence starts. The box is attached to the measured TCP after a
successful grasp check and is released at Place.

The RViz progress panel consumes `/gp8_pick_place/planning_progress`. It
reports segment planning and validation percentages, planning failure, plan
readiness, and sequence completion. Updates occur at segment boundaries, so
the percentage may hold while a long segment is being planned. The panel is a
plugin from `robot_arm_rviz_panel`; `colcon build --packages-up-to
robot_arm_gazebo` builds and installs it. If you previously saw an undefined
panel vtable error, close the old RViz instance, rebuild and source the
workspace before relaunching.

## Path and plot visualization

After the full plan validates, the planner publishes the concatenated planned
TCP path to `/trajectory_preview/reference_path` as a blue `Marker/LINE_STRIP`.
During arm execution, joint-state measurements are passed through GP8 forward
kinematics and published as a green path on
`/trajectory_preview/actual_path`. Both topics use transient-local marker QoS
so RViz can display the latest marker after subscribing. The RViz displays
are enabled in `gp8_pick_place.rviz`.

The optional plot window displays each joint's position, velocity,
acceleration and jerk against time. Enable it for either launch:

```bash
ros2 launch robot_arm_gazebo gp8_pick_place.launch.py plot_trajectory:=true
ros2 launch robot_arm_gazebo gp8_assignment_pick_place.launch.py plot_trajectory:=true
```

`plot_trajectory` defaults to `false`. The plot window opens after a valid
plan is ready. Jerk is unavailable for timing profiles that do not supply it.
The serialized samples are published on `/trajectory_preview/trajectory_data`
for `robot_arm_trajectory`'s plotter node.

## Pose input modes and launch arguments

Both workcell launches accept:

| Argument | Default | Meaning |
| --- | --- | --- |
| `pose_input_mode` | `config` | `config` reads the mode in the selected YAML. `rviz` enables interactive marker editing; `yaml` uses fixed configured poses. |
| `pose_config` | workcell-specific YAML | YAML file containing Initial pose and planner settings; assignment YAML also defines both boxes. |
| `plot_trajectory` | `false` | Start the optional joint graph window. |

The standard `gp8_pick_place.launch.py` additionally accepts:

| Argument | Default | Meaning |
| --- | --- | --- |
| `world` | `worlds/pick_place.sdf` | SDF world file. |
| `world_name` | `pick_place` | Gazebo world name used by bridge and entity service. |
| `cube_name` | `gp8_demo_cube` | Gazebo entity name for the graspable cube. |
| `cube_initial_pose` | `0.62 0.0 0.625` | Spawn centre XYZ in metres, as a space-separated string. |
| `cube_size` | `0.045` | Cube side length in metres. |
| `gz_partition` | `robot_arm_gp8_pick_place` | Gazebo transport partition. |

The assignment launch accepts the shared `pose_input_mode`, `pose_config`,
and `plot_trajectory` arguments. Its config determines Box A/Box B spawn
poses and sizes; it passes the Box B spawn position and size into the standard
workcell launch. For example:

```bash
ros2 launch robot_arm_gazebo gp8_assignment_pick_place.launch.py pose_input_mode:=yaml plot_trajectory:=true
ros2 launch robot_arm_gazebo gp8_pick_place.launch.py pose_input_mode:=yaml
```

For joint-space waypoint trajectories, use the separate assignment launch and
config:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/robot_arm
colcon build --packages-up-to robot_arm_gazebo
source install/setup.bash
ros2 launch robot_arm_gazebo gp8_assignment_waypoint_pick_place.launch.py plot_trajectory:=true
```

It loads `config/gp8_assignment_waypoints.yaml`. Set `poses.initial`,
`poses.pick`, and `poses.place`, then add TCP poses to
`joint_space_trajectory.legs.initial_to_pick`, `pick_to_place`, or
`place_to_initial`. Each waypoint only needs `position` and
`orientation_xyzw`. The planner infers smooth joint velocities and
accelerations at intermediate knots, and starts and ends each leg at rest so
the gripper can operate at Pick and Place. Move the Initial, Pick, and Place
markers in RViz, then open a marker menu and choose **Plan and Execute Pick &
Place**. The reference path, measured path, progress panel, and optional joint
graphs are available in RViz. Enable the graphs with `plot_trajectory:=true`.
`duration` is per leg and `num_samples` controls output density. Each leg is
sent as one controller action goal.

To use another configuration file, pass an absolute path, for example:

```bash
ros2 launch robot_arm_gazebo gp8_assignment_pick_place.launch.py pose_config:=/home/user/my_gp8_assignment.yaml
```

## YAML reference

### Pose schema and frames

Pose position is XYZ in metres. `orientation_xyzw` is the normalized ROS
quaternion `[x, y, z, w]`. The workcell's `base_link` origin is coincident
with the Gazebo world origin. Standard workcell YAML contains:

```yaml
poses:
  initial:
    position: [x, y, z]
    orientation_xyzw: [x, y, z, w]
  pick:
    position: [x, y, z]
    orientation_xyzw: [x, y, z, w]
  place:
    position: [x, y, z]
    orientation_xyzw: [x, y, z, w]
```

In assignment YAML, `objects.box_a` and `objects.box_b` provide `position`,
`orientation_xyzw`, and `size: [x, y, z]`. Box A's specified position is its
centre. The Pick pose is loaded from Box B. The Place pose is derived from
Box A's top face, so it should not be specified as a separate `poses.place`.
The assignment config still needs `poses.initial`.

### `pick_place` options

These values are in `gp8_pick_place.yaml` and `gp8_assignment.yaml` under
`pick_place`:

| Key | Default/example | Meaning |
| --- | --- | --- |
| `pose_input_mode` | `rviz` | Default marker mode used when launch argument is `config`; explicit `rviz`/`yaml` launch values override it. |
| `path_method` | `joint_space` | `cartesian_linear`, `cartesian_arc`, or `joint_space`, used for each segment. |
| `cartesian_arc_height` | `0.08` m; assignment `0.24` m | Raised arc height for `cartesian_arc`. |
| `num_cartesian_waypoints` | `20`; assignment `30` | Initial Cartesian path discretization. |
| `duration` | `3.0` s; assignment `4.0` s | Duration assigned to each arm segment. |
| `num_samples` | `91`; assignment `121` | Generated samples per segment. |
| `interpolation_method` | `cubic_spline` | `linear`, `cubic_spline`, or `quintic_polynomial`. |
| `interpolation_options` | `{}` | Method-specific joint interpolation options; e.g. `bc_type` for spline. |
| `time_parameterization_method` | `quintic_scaling` | `quintic_scaling`, `trapezoidal`, or `seventh_order` (`s_curve` is a deprecated alias). |
| `time_parameterization_options` | `{}` | Method-specific timing options. |
| `ik_solver` | `damped_least_squares` | `damped_least_squares`, `jacobian_transpose`, or `pseudoinverse`. |
| `max_joint_waypoint_step` | `0.5` rad | Preferred maximum joint step during Cartesian waypoint IK. |
| `max_cartesian_refinement_depth` | `8` | Maximum adaptive subdivision depth for Cartesian IK. |
| `approach_height` | `0.10` m; assignment `0.12` m | Vertical clearance used for approach/retreat targets. |
| `gripper_open_position` | `0.0` m | Finger joint target for opening. |
| `gripper_closed_position` | `0.015` m | Finger joint target for closing. |
| `cube_size` | `0.045` m; assignment `0.05` m | Cube side length expected by the gripper/config. |
| `cube_grasp_follow_rate` | `100.0` Hz | Maximum update rate for moving the attached cube with the arm. |
| `grasp_offset` | `[0, 0, 0]` m | Cube-centre offset from the TCP, expressed in TCP coordinates. |
| `grasp_tolerance` | `0.10` m | Maximum TCP-to-cube-centre distance allowed by grasp check. |
| `cube_initial_pose` | optional XYZ | Overrides initial cube centre in standard workcell. Assignment launch supplies this from Box B. |
| `cube_place_pose` | optional XYZ | Optional explicit cube centre at release; normally derived from Place. |

The current node uses the configured Pick marker to position the cube when
starting. For the standard workcell, Pick and Place poses are interpreted as
the grasp/release target used by the shipped vertical grasp; the configured
offset adjusts the cube centre relative to the TCP. In the assignment scene,
the box dimensions determine the top-surface Place height.

## Planner behavior and limits

- **Cartesian line** constrains the tool to a straight line and solves IK at
  intermediate poses. It can fail if any intermediate is unreachable or near
  a singularity.
- **Raised Cartesian arc** uses a raised tool route. A reachable endpoint does
  not guarantee every point on the arc is reachable.
- **Joint space** solves the endpoint and interpolates the joint values. Its
  tool path may curve and may avoid Cartesian waypoint IK failures.

All methods then interpolate and time-parameterize joint waypoints and check
the trajectory against configured position/velocity limits and applicable
Cartesian tracking tolerances. This package does not perform collision
checking, torque analysis, or dynamic feasibility analysis. Review RViz and
Gazebo paths and adjust targets/planner settings when a plan fails or would
intersect the work surface.

## Troubleshooting and useful topics

- If a controller reports that no clock is available, wait for Gazebo's world
  clock and controller manager to initialize; the launch remaps `/clock` to
  the active world's clock topic.
- If the arm does not move, confirm the launch has finished spawning the arm,
  `joint_state_broadcaster`, `arm_controller`, and `gripper_controller`, then
  check the planner log and progress panel. The plan action does not execute
  until all segments validate.
- If a path is not visible, ensure the RViz **Reference Path** and **Actual
  Path** displays are enabled and fixed frame is `base_link`. Reference path
  is sent after planning succeeds; actual path is sent during arm motion.
- If the progress panel class cannot load, build `robot_arm_gazebo` and its
  dependencies, source `install/setup.bash`, and launch a fresh RViz process.
- Use `ros2 topic echo /gp8_pick_place/planning_progress` to inspect
  `percent|status` updates. Use `ros2 topic info
  /trajectory_preview/reference_path` and the corresponding `actual_path`
  topic to check marker publishers.

No physical grasp force is simulated: the cube follows the measured arm pose
after the TCP-to-cube tolerance check. The simplified attachment is intended
for repeatable task visualization and planning demonstration.
