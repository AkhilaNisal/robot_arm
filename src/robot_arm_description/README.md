# Robot arm descriptions and RViz launches

`robot_arm_description` installs the generic six-axis arm, the GP8 URDF/Xacro
and mesh assets, RViz configurations, and launch files for model display and
trajectory preview. The GP8 trajectory preview uses the GP8 DH model from
`robot_arm_kinematics` and the planning pipeline from
`robot_arm_trajectory`.

## Build

From the workspace root:

```bash
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_arm_kinematics robot_arm_trajectory robot_arm_description
source install/setup.bash
```

## GP8 trajectory preview

```bash
ros2 launch robot_arm_description gp8_trajectory_preview.launch.py
```

This starts `robot_state_publisher`, RViz and the trajectory planner. Choose
the RViz **Interact** tool to move Start and Goal markers, then use a marker's
context menu to plan. Available methods are Cartesian line, raised Cartesian
arc, and joint space. Planning is initiated explicitly; the arm model does
not move just because RViz starts. After a plan validates, the preview
publishes the joint-state trajectory to animate the model.

### GP8 preview options

| Argument | Default | Effect |
| --- | --- | --- |
| `pose_input_mode` | `rviz` | `rviz` enables editable marker poses. `yaml` loads fixed poses from `gp8_preview_poses.yaml`. |
| `plot_trajectory` | `false` | Opens an optional Matplotlib window with joint position, velocity, acceleration and jerk plots. |
| `workspace_samples` | `12000` | Number of random joint-limit samples used to build the approximate TCP workspace cloud on request. |
| `preview_rate` | `30.0` Hz | Playback joint-state publish frequency. |
| `looping` | `false` | `true` plays the trajectory forwards and backwards. |
| `joint_state_publisher` | `false` | Starts the manual joint slider GUI. Avoid using it to drive the same joint states during planner playback. |

Examples:

```bash
# Fixed poses from YAML and plot window
ros2 launch robot_arm_description gp8_trajectory_preview.launch.py pose_input_mode:=yaml plot_trajectory:=true

# Denser workspace cloud when requested from the marker menu
ros2 launch robot_arm_description gp8_trajectory_preview.launch.py workspace_samples:=30000
```

The GP8 pose and planner defaults are in
`robot_arm_trajectory/config/gp8_preview_poses.yaml`. Pose positions are in
metres in `base_link`; orientations use normalized `[x, y, z, w]` ROS
quaternions. See the trajectory package guide for every planner YAML key.

### Approximate workspace and plots

Right-click a pose marker and choose **Show Approximate Reachable Workspace**
to publish sampled TCP positions within configured joint limits. Use **Hide
Reachable Workspace** to clear them. `workspace_samples` trades calculation
and rendering time for cloud density. It is not an exhaustive reachable set,
does not include tool orientation, and does not account for collisions.

When `plot_trajectory:=true`, validated joint position, velocity,
acceleration, and jerk data is plotted in a separate Matplotlib window. Joint
position is displayed in degrees; derivatives use radians per second,
radians per second squared, and radians per second cubed. Some time profiles
do not return jerk, in which case the plot indicates that it is unavailable.

### Shared preview launch

The GP8 wrapper above supplies the GP8 model and configs. The shared launch
can also be called directly for the generic robot:

```bash
ros2 launch robot_arm_description trajectory_preview.launch.py
```

| Argument | Generic default | Meaning |
| --- | --- | --- |
| `model` | generic arm Xacro | Robot description Xacro path. |
| `rviz_config` | `trajectory_preview.rviz` | RViz layout file. |
| `robot_config` | `generic_robot.yaml` | DH kinematics and joint limits. |
| `pose_config` | `preview_poses.yaml` | Start/goal and planner settings. |
| `pose_input_mode` | `rviz` | `rviz` or `yaml`. |
| `preview_rate` | `30.0` | Playback frequency in Hz. |
| `marker_publish_rate` | `1.0` | Frequency used to refresh path markers. |
| `looping` | `false` | Play once or forward/backward. |
| `use_sim_time` | `false` | Use ROS `/clock` when true. |
| `joint_state_publisher` | `false` | Start manual joint slider GUI. |
| `plot_trajectory` | `false` | Start optional plot window. |
| `workspace_samples` | `3000` | Samples in the approximate workspace cloud. |

Example selecting explicit model/config paths:

```bash
ros2 launch robot_arm_description trajectory_preview.launch.py \
  model:=/absolute/path/to/arm.urdf.xacro \
  robot_config:=/absolute/path/to/robot.yaml \
  pose_config:=/absolute/path/to/poses.yaml \
  pose_input_mode:=yaml
```

### Other display launches

- `gp8_display.launch.py`: GP8 model, robot state publisher, RViz and manual
  joint sliders; no trajectory planner.
- `display.launch.py`: generic model display and manual joint sliders.

The Gazebo pick-and-place simulation is provided by the separate
[`robot_arm_gazebo`](../robot_arm_gazebo/README.md) package.
