# ROS 2 robot arm planning and simulation

This ROS 2 Jazzy workspace contains a generic six-axis arm and a GP8 arm. It
includes URDF/Xacro models, DH-based FK/IK, Cartesian and joint-space
trajectory planning, a standalone RViz trajectory preview, and a Gazebo Sim
pick-and-place workcell. The standalone preview is the place to inspect
trajectories without starting Gazebo; Gazebo launches add controllers and
simulated execution.

## Requirements and build

The workspace targets Ubuntu 24.04 and ROS 2 Jazzy. The preview needs the ROS
desktop/RViz stack, `colcon`, Python 3, NumPy, SciPy and PyYAML. Gazebo
simulation also uses `ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control` and ROS 2
control/controller packages. Matplotlib is needed only for optional joint
plots. Building the RViz progress panel requires Qt 5 and RViz development
dependencies.

Build the Gazebo stack and all dependencies:

```bash
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to robot_arm_gazebo
source install/setup.bash
```

Or build just the standalone preview stack:

```bash
colcon build --packages-up-to robot_arm_description
source install/setup.bash
```

After each build, source `install/setup.bash` in the terminal used to launch.

## Standalone RViz trajectory preview

Launch GP8 with editable Start and Goal poses:

```bash
ros2 launch robot_arm_description gp8_trajectory_preview.launch.py
```

In RViz, choose **Interact**, move a pose marker, and open its context menu.
Select Cartesian Line, Raised Cartesian Arc, or Joint Space to plan and
validate the path. The model preview animates joint states and displays the
TCP reference path after a valid plan. Gripper menu actions change the
visualized finger joints; the preview does not simulate grasp contact.

### Joint-space waypoint route

Select the ordered TCP waypoint planner:

```bash
ros2 launch robot_arm_description gp8_trajectory_preview.launch.py \
  planning_mode:=joint_space_waypoints
```

The default file is
`robot_arm_trajectory/config/gp8_joint_waypoints.yaml`. To use another YAML
route:

```bash
ros2 launch robot_arm_description gp8_trajectory_preview.launch.py \
  planning_mode:=joint_space_waypoints \
  waypoint_config:=/absolute/path/to/route.yaml
```

The route YAML contains `poses.initial`, an ordered
`joint_space_trajectory.waypoints` list, and `poses.goal`. Each entry uses
`position: [x,y,z]` in metres and a normalized
`orientation_xyzw: [x,y,z,w]` quaternion, expressed in `base_link`. RViz's
Start marker menu adds waypoints; move/rotate them, remove knots from their
own context menus, then select **Plan: Joint Space Waypoints**. Edits in RViz
last for the current process. Change YAML for a repeatable route.

`joint_space_trajectory.duration` is a positive finite number of seconds for
the route, and `num_samples` is an integer of at least 2. The waypoint list
can be empty. IK is solved in order, each pose seeded from the previous joint
solution. The joint path passes through the supplied IK knots; the TCP may
curve between them. See the
[`robot_arm_description` guide](src/robot_arm_description/README.md) for
launch options and the
[`robot_arm_trajectory` guide](src/robot_arm_trajectory/README.md) for every
planner and trajectory option.

### Preview launch options

| Argument | Default | Purpose / useful range |
| --- | --- | --- |
| `planning_mode` | `cartesian` | `cartesian` or `joint_space_waypoints`. |
| `waypoint_config` | installed GP8 waypoint YAML | Route file used in waypoint mode. |
| `pose_input_mode` | `rviz` | Editable `rviz` markers or fixed `yaml` poses for Cartesian mode. |
| `preview_rate` | `30.0` Hz | Positive playback update frequency. |
| `looping` | `false` | Boolean; reverse and repeat when enabled. |
| `joint_state_publisher` | `false` | Boolean; open manual joint slider GUI. |
| `plot_trajectory` | `false` | Boolean; open Matplotlib joint trajectory plots. |
| `workspace_samples` | `12000` | Random joint-limit samples for approximate TCP workspace; minimum effective count 100. |

The generic robot can be previewed with
`ros2 launch robot_arm_description trajectory_preview.launch.py`; that launch
also accepts model, RViz, robot config, pose config, `use_sim_time`, and marker
refresh settings.

## Gazebo pick-and-place simulation

Standard workcell:

```bash
ros2 launch robot_arm_gazebo gp8_pick_place.launch.py
```

Box A / Box B assignment workcell:

```bash
ros2 launch robot_arm_gazebo gp8_assignment_pick_place.launch.py
```

Waypoint-enabled assignment workcell:

```bash
ros2 launch robot_arm_gazebo gp8_assignment_waypoint_pick_place.launch.py
```

In RViz, edit Initial/Pick/Place and select **Plan and Execute Pick & Place**.
The planner builds and validates all eight motion segments before execution.
The Gazebo waypoint launch adds ordered knots for Initial-to-Pick,
Pick-to-Place, and Place-to-Initial legs. Choose Cartesian Line, Cartesian
Arc, Joint Space, or Joint-Space Waypoints from a marker menu, add/move/remove
per-leg waypoint markers, then plan and execute. The reference TCP route is
blue; the measured TCP trace is green. The progress panel reports segment
planning and execution state.

The default waypoint assignment config is
`robot_arm_gazebo/config/gp8_assignment_waypoints.yaml`. Use
`pose_config:=/absolute/path/to/file.yaml` to select another compatible
configuration. Enable plots with `plot_trajectory:=true`. More launch and YAML
options, object schema and operational details are in the
[`robot_arm_gazebo` guide](src/robot_arm_gazebo/README.md).

The simulated cube follows measured TCP pose after a grasp tolerance check;
it is a deterministic attachment for demonstration, not a physical
contact/friction model.

## Planner methods and shared options

| Method | Config value | Meaning |
| --- | --- | --- |
| Cartesian line | `cartesian_linear` | Straight TCP position route with IK at intermediate poses. |
| Cartesian arc | `cartesian_arc` | Raised TCP route; configured by `cartesian_arc_height` in metres. |
| Endpoint joint space | `joint_space` | Solve the goal pose and interpolate joint coordinates; TCP path may curve. |
| Ordered joint-space waypoints | `joint_space_waypoints` | Sequential IK through a list of TCP poses; available in standalone preview and waypoint Gazebo assignment. |

Interpolation choices are `linear`, `cubic_spline`, and
`quintic_polynomial`. Time profiles are `quintic_scaling`, `trapezoidal`, and
`seventh_order`; `s_curve` is a deprecated alias for `seventh_order`.
Important tunables include Cartesian waypoint count (integer ≥ 2), duration
(positive finite seconds), output samples (integer ≥ 2), IK iteration cap
(integer ≥ 1), IK restarts (integer ≥ 0), positive finite IK tolerances,
Cartesian refinement depth (integer ≥ 0), and positive joint waypoint step
(radians). Lower resolution/retry settings can reduce planning time; higher
resolution and more retries can help difficult paths. See each package guide
for the complete keys, units, profile behavior, and limitations.

These planners are not collision or dynamics planners. They do not certify
collision clearance, torques, or full dynamic feasibility. Review reference
and measured paths in the applicable workcell.

## Workspace package map

| Package | Main contents |
| --- | --- |
| [`robot_arm_description`](src/robot_arm_description/README.md) | Generic/GP8 Xacro, link meshes, RViz layouts, display and preview launch files. |
| [`robot_arm_kinematics`](src/robot_arm_kinematics/README.md) | Robot YAML models, DH FK, Jacobian and three IK solvers. |
| [`robot_arm_trajectory`](src/robot_arm_trajectory/README.md) | Cartesian/joint planning, interpolation, timing, validation, RViz preview/plotter nodes and pose YAML. |
| [`robot_arm_gazebo`](src/robot_arm_gazebo/README.md) | Gazebo worlds, GP8 ros2_control model, pick/place nodes, workcell launch/config files. |
| [`robot_arm_rviz_panel`](src/robot_arm_rviz_panel/README.md) | RViz C++ progress panel plugin for Gazebo pick/place. |

## Main ROS interfaces

| Topic/service | Type | Purpose |
| --- | --- | --- |
| `/trajectory_preview/reference_path` | `visualization_msgs/msg/Marker` | Planned TCP path. |
| `/trajectory_preview/actual_path` | `visualization_msgs/msg/Marker` | TCP path reconstructed from published/measured joints. |
| `/trajectory_preview/reachable_workspace` | `visualization_msgs/msg/Marker` | Approximate preview workspace cloud. |
| `/trajectory_preview/trajectory_data` | `std_msgs/msg/String` | JSON trajectory samples for optional plotter. |
| `/gp8_pick_place/planning_progress` | `std_msgs/msg/String` | `percent|status` for progress panel. |
| `/gp8_pick_place/plan_and_execute` | `std_srvs/srv/Trigger` | Trigger Gazebo pick/place planning and execution. |

For details about all nodes, scripts, launch files, YAML keys, examples and
configurable values, start with each package's README linked above.

## License

This project is distributed under the MIT License. See [LICENSE](LICENSE).
When sharing or redistributing this project or substantial portions of it,
retain the copyright and permission notice. Third-party dependencies and
assets may have separate licenses.
