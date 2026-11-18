# GP7 pick and place progress panel

`robot_arm_rviz_panel` is a C++/Qt RViz plugin that subscribes to the Gazebo
pick/place planner's progress string and displays a percentage bar with a
status message. It is loaded by
`robot_arm_gazebo/rviz/gp7_pick_place.rviz`; it has no standalone ROS node or
launch file.

## Build and use

Build this package with the Gazebo package so plugin dependencies and
install-time plugin discovery are configured together:

```bash
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to robot_arm_gazebo
source install/setup.bash
ros2 launch robot_arm_gazebo gp7_pick_place.launch.py
```

The RViz panel is titled **Pick and Place Progress**. After a plugin rebuild,
close existing RViz processes and start a fresh launch so RViz loads the new
shared library.

## Topic contract

The panel subscribes to:

```text
/gp7_pick_place/planning_progress  std_msgs/msg/String
```

Messages are formatted as `integer_percentage|status text`:

```text
37|Planning segment 4 of 8
50|Validated segment 4 of 8
100|Plan ready; executing
```

Percentage is expected from 0 through 100. The planner publishes updates at
segment planning/validation boundaries, when the whole plan is ready, and
when execution completes. A long IK/planning call can leave the current
percentage visible until that call returns. Failures are reported in the
status text and reset progress.

Inspect messages while a pick/place launch is running:

```bash
ros2 topic echo /gp7_pick_place/planning_progress
```

The plugin class ID is `robot_arm_rviz_panel/PlanningProgressPanel`, declared
in `plugin_description.xml`. The implementation is in
`src/planning_progress_panel.cpp` with its Qt/RViz interface in
`include/robot_arm_rviz_panel/planning_progress_panel.hpp`.
