# Robot arm descriptions and RViz launch files

This package installs the generic six-axis model, the GP8 URDF/Xacro and STL
meshes, and the shared RViz trajectory-preview configuration.

## GP8 preview

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_arm_kinematics robot_arm_trajectory robot_arm_description
source install/setup.bash
ros2 launch robot_arm_description gp8_trajectory_preview.launch.py
```

The launch starts `robot_state_publisher`, RViz, and the trajectory preview.
In RViz select **Interact** and right-click either pose marker. Planning is
manual: choose a path method from the menu. The model does not move until a
plan has passed validation and playback starts.

### GP8 launch arguments

- `plot_trajectory:=true` starts the optional plot window. It plots joint
  position, velocity, acceleration, and jerk after a successful plan; default
  is `false`.
- `workspace_samples:=12000` sets the number of random in-limit configurations
  used when the menu's **Show Approximate Reachable Workspace** action is
  selected. Increase it for a denser cloud, such as `workspace_samples:=30000`.
- `pose_input_mode:=rviz` uses draggable markers. `pose_input_mode:=yaml` loads
  start and goal from the GP8 pose config and disables dragging.
- `preview_rate:=30.0` sets joint-state updates per second.
- `looping:=false` plays once; `true` plays forward and back.
- `joint_state_publisher:=false` controls the optional manual slider GUI.

For example, launch editable markers and trajectory plots with a denser
workspace cloud:

```bash
ros2 launch robot_arm_description gp8_trajectory_preview.launch.py \
  pose_input_mode:=rviz plot_trajectory:=true workspace_samples:=30000
```

The workspace cloud shows sampled TCP positions within the configured joint
limits. It is approximate and does not check tool orientation, self-collision,
or obstacles. The plot window updates after a trajectory validates.

## Generic arm preview

```bash
ros2 launch robot_arm_description trajectory_preview.launch.py
```

The shared launch defaults to the generic model and generic configs. Its
`workspace_samples` default is 3,000. It accepts `pose_input_mode`,
`preview_rate`, `marker_publish_rate`, `looping`, `use_sim_time`,
`plot_trajectory`, `workspace_samples`, and `joint_state_publisher`.
`model`, `robot_config`, `pose_config`, and `rviz_config` select alternate
model and configuration files. The GP8 wrapper exposes only its documented
arguments above; use the shared launch directly to override `marker_publish_rate`
or `use_sim_time`.

## Other launch files

- `gp8_display.launch.py`: display the GP8 model and joint sliders without the
  trajectory planner.
- `display.launch.py`: display the generic arm.
- `trajectory_preview.launch.py`: shared generic-by-default model, planner,
  RViz, optional plotting, and workspace preview.
