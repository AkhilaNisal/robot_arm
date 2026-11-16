# GP8 pick and place progress panel

This package builds an RViz panel plugin that shows the current GP8 Gazebo
pick-and-place planning percentage and status. It is loaded by
`robot_arm_gazebo/rviz/gp8_pick_place.rviz` and is a runtime dependency of the
Gazebo package.

## Build and load

Build the Gazebo package and its dependencies from the workspace root:

```bash
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to robot_arm_gazebo
source install/setup.bash
ros2 launch robot_arm_gazebo gp8_pick_place.launch.py
```

The RViz layout adds the panel named **Pick and Place Progress**. Close old
RViz instances before rebuilding or after a plugin load error: a running RViz
process keeps the previously loaded shared library in memory.

## Progress topic format

The panel subscribes to:

```text
/gp8_pick_place/planning_progress  std_msgs/msg/String
```

Messages use `integer_percentage|status text`, for example:

```text
37|Planning segment 4 of 8
50|Validated segment 4 of 8
100|Plan ready; executing
```

The pick/place planner reports progress when it begins a segment, when that
segment passes planning and validation, when the complete plan is ready, and
when execution finishes. An error is shown in the status text and resets the
bar to zero. The percentage updates per segment rather than continuously
within one IK/planning call; a long segment may therefore leave the displayed
value unchanged for several seconds.

Check the topic from a sourced workspace while the Gazebo launch is running:

```bash
ros2 topic echo /gp8_pick_place/planning_progress
```

The custom class ID in the RViz plugin description is
`robot_arm_rviz_panel/PlanningProgressPanel`.
