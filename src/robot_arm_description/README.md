# Robot descriptions, display and standalone preview

`robot_arm_description` installs the robot models and RViz layouts, and owns
the launch files that display a robot or run the standalone trajectory
preview. The preview does not start Gazebo: it publishes joint states from a
validated plan so the model and planned TCP path can be inspected in RViz.

## Build and launch

```bash
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to robot_arm_description
source install/setup.bash
ros2 launch robot_arm_description gp7_trajectory_preview.launch.py
```

The GP7 launch starts `robot_state_publisher`, `rviz2`, and the
`robot_arm_trajectory/trajectory_preview` node. Use RViz's **Interact** tool
to move the Start and Goal markers. A marker's context menu offers Cartesian
Line, Raised Cartesian Arc, Joint Space, gripper open/close, and approximate
workspace actions. Selecting a plan method plans and validates before
publishing the preview. The blue reference line is the FK path of the planned
joint trajectory; the robot animates when playback is active.

## Selecting waypoint planning

The GP7 wrapper has two planning modes: `cartesian` (default) and
`joint_space_waypoints`. The latter loads a YAML route with an initial TCP
pose, zero or more ordered intermediate TCP poses, and a goal TCP pose:

```bash
ros2 launch robot_arm_description gp7_trajectory_preview.launch.py \
  planning_mode:=joint_space_waypoints
```

This selects `robot_arm_trajectory/config/gp7_joint_waypoints.yaml` and YAML
pose input automatically. To load another file:

```bash
ros2 launch robot_arm_description gp7_trajectory_preview.launch.py \
  planning_mode:=joint_space_waypoints \
  waypoint_config:=/absolute/path/to/my_route.yaml
```

The YAML schema is `poses.initial`, `joint_space_trajectory.waypoints`, and
`poses.goal`. Each pose has `position: [x,y,z]` in metres and
`orientation_xyzw: [x,y,z,w]` as a normalized ROS quaternion, all in
`base_link`. The planner solves IK in route order, seeding each pose with the
preceding solution, and interpolates the resulting joint route. The sampled
FK path is shown in RViz as the trajectory reference.

To add knots interactively, use the marker menu action **Add Joint Space
Waypoint** on Start. Move/rotate the new marker and add more as needed. Use a
waypoint's own context menu to remove it. Choose **Plan: Joint Space
Waypoints** after editing. The YAML file remains the starting route each time
the node launches; save edits to YAML if they should persist across launches.

Example route:

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
    - position: [0.62, 0.12, 0.48]
      orientation_xyzw: [0.0, 1.0, 0.0, 0.0]
```

`duration` must be positive and finite. `num_samples` must be an integer at
least 2. The waypoint list may be empty; its length is otherwise determined
by the route. Each additional pose adds an IK solve and a joint knot. Keep
poses reachable and avoid abrupt route changes. This planner constrains the
TCP at the supplied knots; the interpolated TCP can curve between them.

## GP7 launch arguments

| Argument | Default | Values and effect |
| --- | --- | --- |
| `planning_mode` | `cartesian` | `cartesian` uses the regular Start/Goal planner menu; `joint_space_waypoints` loads the route YAML and waypoint menu. |
| `waypoint_config` | installed `gp7_joint_waypoints.yaml` | Path to waypoint route YAML; used in waypoint mode. |
| `pose_input_mode` | `rviz` | For Cartesian mode, `rviz` enables marker editing and `yaml` loads fixed poses. Waypoint mode forces YAML route input. |
| `preview_rate` | `30.0` Hz | Positive playback update rate. |
| `looping` | `false` | Boolean; when true, playback reverses at the end and repeats. |
| `joint_state_publisher` | `false` | Boolean; start manual joint slider GUI. Avoid simultaneous slider control and playback. |
| `plot_trajectory` | `false` | Boolean; launch the optional Matplotlib trajectory plotter. |
| `workspace_samples` | `12000` | Random joint-limit samples for approximate reachable workspace. Values below 100 are raised to 100; more points cost more time and RViz rendering. |

Examples:

```bash
# Fixed GP7 poses, with graphs
ros2 launch robot_arm_description gp7_trajectory_preview.launch.py \
  pose_input_mode:=yaml plot_trajectory:=true

# Waypoint route and dense approximate workspace cloud
ros2 launch robot_arm_description gp7_trajectory_preview.launch.py \
  planning_mode:=joint_space_waypoints workspace_samples:=30000
```

The workspace cloud is an approximate sampled TCP position set. It does not
represent tool orientations, collision-free space, or an exhaustive boundary.

## Shared generic preview launch

For the generic six-axis model:

```bash
ros2 launch robot_arm_description trajectory_preview.launch.py
```

| Argument | Default | Meaning |
| --- | --- | --- |
| `model` | `urdf/generic/robot_arm.urdf.xacro` | Xacro model path. |
| `rviz_config` | `rviz/trajectory_preview.rviz` | RViz layout path. |
| `robot_config` | kinematics `generic_robot.yaml` | DH model and joint limits. |
| `pose_config` | trajectory `preview_poses.yaml` | Start/goal poses and planner settings. |
| `pose_input_mode` | `rviz` | Editable RViz poses or fixed YAML poses. |
| `preview_rate` | `30.0` Hz | Must be positive. |
| `marker_publish_rate` | `1.0` Hz | Must be positive; marker refresh rate. |
| `looping` | `false` | Forward playback once or forward/backward looping. |
| `use_sim_time` | `false` | Use `/clock` if true. |
| `joint_state_publisher` | `false` | Enable manual joint sliders. |
| `plot_trajectory` | `false` | Enable separate joint graph window. |
| `workspace_samples` | `3000` | Approximate workspace sample count; minimum effective count 100. |

The generic preview currently uses the standard Cartesian preview controls;
GP7's wrapper adds the waypoint mode. For all `trajectory` YAML planner
fields, algorithms, and supported values, see the
[`robot_arm_trajectory` guide](../robot_arm_trajectory/README.md).

## Other launch files and installed assets

| File | Purpose |
| --- | --- |
| `launch/gp7_display.launch.py` | GP7 model, state publisher, RViz and manual joint sliders; no planner. |
| `launch/display.launch.py` | Generic arm display and manual sliders; no planner. |
| `launch/trajectory_preview.launch.py` | Shared generic launch implementation; starts state publisher, RViz, preview node, optional sliders and plotter. |
| `launch/gp7_trajectory_preview.launch.py` | GP7 wrapper; selects the GP7 model, DH YAML, RViz config and preview/waypoint YAML. |
| `urdf/gp7/gp7.urdf.xacro` | GP7 visual/robot description. |
| `urdf/generic/robot_arm.urdf.xacro` | Generic six-axis description. |
| `meshes/*.STL` | GP7 link visual meshes. |
| `rviz/trajectory_preview.rviz` | Robot, marker, reference path, actual path and workspace displays. |

To view GP7 without planning, run `ros2 launch robot_arm_description
gp7_display.launch.py`. `robot_state_publisher` publishes the model TF tree;
the preview node publishes joint states and path markers. Gazebo controller
execution and pick/place are in the separate
[`robot_arm_gazebo` package](../robot_arm_gazebo/README.md).
