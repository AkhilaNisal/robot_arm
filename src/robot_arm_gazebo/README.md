# GP7 Gazebo Sim pick and place

`robot_arm_gazebo` launches the GP7 in Gazebo Sim with `gz_ros2_control`, ROS
2 controllers, RViz interactive poses, and a pick/place planner. It uses
`robot_arm_kinematics` for FK/IK, `robot_arm_trajectory` for planning and
validation, and `robot_arm_rviz_panel` for the progress display. Each motion
segment is planned and validated before the sequence sends arm controller
goals.

## Build and launch

```bash
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to robot_arm_gazebo
source install/setup.bash
```

Standard workcell:

```bash
ros2 launch robot_arm_gazebo gp7_pick_place.launch.py
```

Box A / Box B assignment workcell:

```bash
ros2 launch robot_arm_gazebo gp7_assignment_pick_place.launch.py
```

Waypoint-enabled assignment workcell:

```bash
ros2 launch robot_arm_gazebo gp7_assignment_waypoint_pick_place.launch.py
```

The standard world has a ground plane, work surface and graspable cube. The
assignment launch creates a temporary SDF world from the assignment YAML,
spawns Box A as a static model, and spawns Box B as the grasp target. The
shipped assignment coordinates are chosen to be reachable in this workcell.

## Operating the pick/place task

Use RViz's **Interact** tool to move Initial, Pick and Place poses. Right-click
a pose marker and select **Plan and Execute Pick & Place**. This can also be
triggered by service:

```bash
ros2 service call /gp7_pick_place/plan_and_execute std_srvs/srv/Trigger {}
```

The planner builds the eight-segment sequence (initial approach, pick
approach, descend, lift, transfer, place, retreat, return) and validates all
segments before sending the first arm goal. On success, the sequence opens
the gripper, approaches Pick, closes and checks the grasp, lifts/transfers the
object, releases at Place, retreats, and returns to Initial. The simulated
cube follows the measured TCP after the grasp tolerance check; this is a
repeatable visualization attachment rather than a contact-force/friction
grasp model.

The RViz layout shows the reference path (blue), measured TCP path (green),
workcell objects, pose markers, and **Pick and Place Progress** panel. Path
topics use transient-local marker QoS so late RViz subscribers receive the
last marker. Progress is reported at segment boundaries; a long planning call
may keep the bar unchanged while it runs.

## Select a path method and edit route knots

Open a pose marker's context menu. The method choices are:

| Menu method | Config value | Behavior |
| --- | --- | --- |
| Cartesian Line | `cartesian_linear` | Straight TCP line with intermediate-pose IK. |
| Cartesian Arc | `cartesian_arc` | Raised Cartesian route using `cartesian_arc_height`. |
| Joint Space | `joint_space` | Endpoint IK and joint interpolation; TCP path may curve. |
| Joint-Space Waypoints | `joint_space_waypoints` | Sequential IK through ordered TCP knots on each of the three task legs. |

After choosing a method, use **Plan and Execute Pick & Place**. The regular
Cartesian Line, Cartesian Arc, and Joint Space methods use planner settings
from the selected YAML `pick_place` block. Joint-Space Waypoints uses the
additional `joint_space_trajectory` settings and leg routes.

The waypoint launch is an assignment workcell wrapper using
`config/gp7_assignment_waypoints.yaml` by default. It uses the waypoint node
and starts in waypoint mode. Add intermediate knots from a pose marker menu:

- **Add Waypoint on Initial to Pick Leg**
- **Add Waypoint on Pick to Place Leg**
- **Add Waypoint on Place to Initial Leg**

Move/rotate each new pose marker. Use its own context menu to remove it. **Reset
All Poses** restores the original configured object/pose/waypoint state. Knots
are followed in list order. The waypoint path is divided at Pick and Place so
the gripper action can happen at those targets; the controller receives one
action goal for each planned leg segment. Add knots to make the transfer route
clear obstacles or shape the TCP motion. Every knot must admit IK.

Waypoint planning solves each pose using the preceding joint solution as its
seed, then generates a joint-space trajectory through the resulting joint
knots. The TCP path can curve between pose markers. It does not itself perform
collision checking.

## Plot and launch options

Set `plot_trajectory:=true` to open the optional Matplotlib graphs after a
plan is ready:

```bash
ros2 launch robot_arm_gazebo gp7_assignment_waypoint_pick_place.launch.py \
  plot_trajectory:=true
```

The plot shows joint position in degrees and velocity, acceleration, and
jerk in radians-based units. Jerk is unavailable for timing profiles that do
not provide it.

| Launch file | Arguments | Default behavior |
| --- | --- | --- |
| `gp7_pick_place.launch.py` | `pose_input_mode`, `pose_config`, `plot_trajectory`, `world`, `world_name`, `cube_name`, `cube_initial_pose`, `cube_size`, `gz_partition`, `planner_executable` | Standard world, `pick_place` node, shipped standard YAML. |
| `gp7_assignment_pick_place.launch.py` | `pose_input_mode`, `pose_config`, `plot_trajectory`, `planner_executable` | Assignment world generated from Box A/Box B YAML, standard `pick_place` node. |
| `gp7_assignment_waypoint_pick_place.launch.py` | `pose_input_mode`, `pose_config`, `plot_trajectory` | Assignment waypoint YAML and `waypoint_pick_place` node. |

Detailed launch arguments:

| Argument | Default | Units / accepted values and effect |
| --- | --- | --- |
| `pose_input_mode` | Standard: `config`; assignment: `config`; waypoint wrapper: `rviz` | `config` reads `pick_place.pose_input_mode` from YAML, `rviz` makes pose markers editable, `yaml` uses fixed configured poses. |
| `pose_config` | Workcell-specific installed YAML | Path to pose/planner YAML. For waypoint launch, point to a file matching the assignment waypoint schema. |
| `plot_trajectory` | `false` | Boolean; open optional joint plots after successful planning. |
| `world` | `worlds/pick_place.sdf` | Standard launch SDF file path. |
| `world_name` | `pick_place` | Gazebo world name used by bridge/services. |
| `cube_name` | `gp7_demo_cube` | Gazebo entity name for standard workcell object. |
| `cube_initial_pose` | `0.62 0.0 0.625` | Space-separated XYZ in metres. |
| `cube_size` | `0.045` m | Positive cube side length for standard object. |
| `gz_partition` | `robot_arm_gp7_pick_place` | Gazebo transport partition. Assignment wrapper selects its own partition. |
| `planner_executable` | `pick_place` | `pick_place` or `waypoint_pick_place`; waypoint wrapper sets the latter. |

`gp7_assignment_pick_place.launch.py` reads Box A/Box B from YAML, generates
a temporary SDF containing Box A, and passes Box B pose and cube size into the
standard launch. `objects.box_b.size` must contain equal positive dimensions
because the spawned grasp object is a cube. Restart after editing the YAML;
the world is generated once at launch.

## YAML pose and object schema

All TCP positions are metres in `base_link`; this launch aligns `base_link`
with the Gazebo world origin. `orientation_xyzw` is a normalized ROS
quaternion `[x,y,z,w]`. An ordinary workcell pose block looks like:

```yaml
poses:
  initial:
    position: [0.63, 0.0, 0.81]
    orientation_xyzw: [0.7071, 0.0, 0.7071, 0.0]
  pick:
    position: [0.62, -0.25, 0.025]
    orientation_xyzw: [0.0, 1.0, 0.0, 0.0]
  place:
    position: [0.5, 0.5, 0.125]
    orientation_xyzw: [0.0, 1.0, 0.0, 0.0]
```

Assignment configuration also defines object centres and dimensions:

```yaml
objects:
  box_a:
    position: [0.5, 0.5, 0.0]  # centre, metres in base_link
    orientation_xyzw: [0.0, 0.0, 0.0, 1.0]
    size: [0.30, 0.30, 0.20]  # task sheet: 30 cm x 30 cm x 20 cm
  box_b:
    position: [0.62, -0.25, 0.025]
    orientation_xyzw: [0.0, 1.0, 0.0, 0.0]
    size: [0.05, 0.05, 0.05]  # equal dimensions required
```

Box A `position` is its centre. Assignment Pick follows Box B; Place is
computed from Box A's top surface and box heights, with the configured
orientation. `poses.initial` remains user-configurable. Keep objects and
approach poses within reach and clear of the GP7 pedestal/work surface.
Gazebo and the RViz Box A marker both take Box A's pose and dimensions from
the selected assignment YAML, so the displayed box follows these values.
For the waypoint assignment, the corresponding Place TCP target is
`[0.5, 0.5, 0.125]` m: Box A's 0.20 m height plus Box B's 0.05 m height
places the grasp cube centre 0.125 m above Box A's centre.

## Planner and gripper configuration

The files `gp7_pick_place.yaml`, `gp7_assignment.yaml`, and
`gp7_assignment_waypoints.yaml` contain a `pick_place` mapping. The first two
select path/interpolation/timing/IK settings for the regular planner. The
waypoint assignment uses these settings for alternate methods and contains an
additional waypoint configuration below.

| `pick_place` key | Meaning / units / range |
| --- | --- |
| `pose_input_mode` | Default input mode: `rviz` or `yaml`; `config` launch value delegates to this setting. |
| `path_method` | `cartesian_linear`, `cartesian_arc`, or `joint_space`; waypoint YAML also uses `joint_space_waypoints`. |
| `cartesian_arc_height` | Arc lift in metres; must produce a reachable route. |
| `num_cartesian_waypoints` | Integer ≥ 2; initial Cartesian path resolution. More samples add IK work. |
| `duration` | Positive finite seconds per planned segment. |
| `num_samples` | Integer ≥ 2 output samples per segment. |
| `interpolation_method` | `linear`, `cubic_spline`, or `quintic_polynomial`. |
| `interpolation_options` | Mapping for selected interpolator; e.g. spline `bc_type`. |
| `time_parameterization_method` | `quintic_scaling`, `trapezoidal`, or `seventh_order`; `s_curve` is a deprecated alias. |
| `time_parameterization_options` | Mapping of selected timing profile options. |
| `ik_solver` | `damped_least_squares`, `jacobian_transpose`, or `pseudoinverse`. |
| `ik_max_iterations` | Integer ≥ 1; maximum IK iterations per attempt. |
| `ik_position_tolerance` | Positive finite TCP position tolerance in metres. |
| `ik_orientation_tolerance` | Positive finite orientation tolerance in radians. |
| `ik_restarts` | Integer ≥ 0; more attempts can improve robustness but increase planning time. |
| `max_joint_waypoint_step` | Positive radians; preferred max joint change between Cartesian IK knots. |
| `max_cartesian_refinement_depth` | Integer ≥ 0; recursive subdivision cap for Cartesian IK. |
| `approach_height` | Positive metres; vertical offset used for approach/lift/retreat poses. |
| `gripper_open_position`, `gripper_closed_position` | Finger joint target values in metres; respect controller/joint limits. |
| `cube_size` | Positive metres; grasp object side length. |
| `cube_grasp_follow_rate` | Positive Hz; cap for simulated attached-cube pose updates. |
| `grasp_offset` | XYZ offset in metres in TCP coordinates. |
| `grasp_tolerance` | Positive metres; max allowed TCP-to-cube-centre distance for grasp check. |

The waypoint assignment adds:

```yaml
joint_space_trajectory:
  duration: 5.0
  num_samples: 81
  approach_height: 0.12
  legs:
    initial_to_pick: []
    pick_to_place:
      - position: [0.62, -0.12, 0.48]
        orientation_xyzw: [0.0, 1.0, 0.0, 0.0]
    place_to_initial: []
```

`duration` is applied to each leg trajectory; `num_samples` is per leg. Each
leg accepts an ordered list of zero or more pose entries. The shipped YAML
contains two transfer knots on Pick-to-Place. Every knot uses metre XYZ and
normalized `orientation_xyzw` in `base_link`. Add knots in RViz or edit YAML;
the YAML route is restored on reset/relaunch.

The standard workcell uses a default `path_method: joint_space`; its planner
and grasp values are in `config/gp7_pick_place.yaml`. The regular assignment
uses the tuned `config/gp7_assignment.yaml`. The waypoint assignment defaults
to `joint_space_waypoints` and uses a relatively short IK budget. Settings
such as fewer Cartesian knots, lower refinement depth, fewer restarts, and
fewer samples reduce planning work without changing the selected method, but
can reduce route fidelity or solve success for hard targets.

## Config, launch and source inventory

| Path | Responsibility |
| --- | --- |
| `launch/gp7_pick_place.launch.py` | Starts standard Gazebo world, robot, cube, ROS-Gazebo bridge, controllers, RViz and planner. |
| `launch/gp7_assignment_pick_place.launch.py` | Reads assignment YAML, creates temporary world with Box A, passes Box B spawn geometry and pose to standard launch. |
| `launch/gp7_assignment_waypoint_pick_place.launch.py` | Assignment wrapper selecting waypoint YAML and `waypoint_pick_place` executable. |
| `robot_arm_gazebo/pick_place_node.py` | Standard pick/place node, marker menus, method selection, segment planning/validation, execution, progress and cube attachment. |
| `robot_arm_gazebo/waypoint_pick_place_node.py` | Adds ordered waypoint lists per task leg and sequential waypoint IK planning. |
| `config/controllers.yaml` | `gz_ros2_control` controller manager and joint state/arm/gripper controller configuration. |
| `config/gp7_pick_place.yaml` | Standard workcell poses and planner/grasp settings. |
| `config/gp7_assignment.yaml` | Box A/Box B geometry, task poses, and tuned regular planner configuration. |
| `config/gp7_assignment_waypoints.yaml` | Assignment waypoint planner defaults and ordered route knots for each task leg. |
| `worlds/pick_place.sdf` | Standard ground/work surface and Gazebo world. |
| `worlds/gp7_assignment.sdf` | Assignment world template; Box A is added from YAML at launch. |
| `urdf/gp7_gazebo.urdf.xacro` | GP7 model plus Gazebo and ros2_control integration. |
| `rviz/gp7_pick_place.rviz` | Robot, poses, objects, TCP paths and progress panel displays. |
| `pick_place` executable | Runs `PickPlaceNode`. |
| `waypoint_pick_place` executable | Runs `WaypointPickPlaceNode`. |

## Topics, services and troubleshooting

| Name | Type | Use |
| --- | --- | --- |
| `/gp7_pick_place/plan_and_execute` | `std_srvs/srv/Trigger` | Plan and execute configured sequence. |
| `/gp7_pick_place/planning_progress` | `std_msgs/msg/String` | `percent|status` updates for the RViz panel. |
| `/gp7_pick_place/waypoints` | Interactive markers | Editable per-leg TCP waypoint route in waypoint mode. |
| `/trajectory_preview/reference_path` | `visualization_msgs/msg/Marker` | Planned TCP reference line. |
| `/trajectory_preview/actual_path` | `visualization_msgs/msg/Marker` | TCP path from measured joints during execution. |
| `/trajectory_preview/trajectory_data` | `std_msgs/msg/String` | JSON samples for optional plotter. |

- If controller startup reports no clock, wait for Gazebo's world clock and
  controller manager; the launch bridges the active world's clock.
- If the arm does not move, check that spawning and controllers completed,
  then inspect planner output and `/gp7_pick_place/planning_progress`. The
  first controller goal is sent only after all segments validate.
- If paths are missing, enable the RViz Reference/Actual Path displays and
  use `base_link` as the fixed frame.
- If the panel plugin fails to load, rebuild `robot_arm_gazebo`, source the
  workspace, close old RViz instances, and relaunch.
- Check progress with `ros2 topic echo /gp7_pick_place/planning_progress`.

The planner validates joint position/velocity and applicable Cartesian
tracking limits, but does not do collision detection, torque calculation, or
full dynamics analysis. Inspect the path in the actual workcell and tune
targets/configuration before treating a successful plan as safe for hardware.
