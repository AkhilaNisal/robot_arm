# Robot arm kinematics

`robot_arm_kinematics` provides configurable robot models, standard
Denavit–Hartenberg (DH) forward kinematics, a geometric Jacobian, and three
full-pose inverse-kinematics solvers. The trajectory preview and Gazebo
pick/place planner both use this package for GP8 FK/IK.

## Build

```bash
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_arm_kinematics
source install/setup.bash
```

## Robot YAML files

Configs are installed under `robot_arm_kinematics/config/`:

- `generic_robot.yaml`: original generic six-axis robot model.
- `gp8.yaml`: GP8 model derived from its URDF joint-axis geometry.

Each file describes:

| Field | Units / content |
| --- | --- |
| `robot.name`, `robot.dof`, `robot.joint_names` | Model name, degrees of freedom and ordered joint names. |
| `base_transform.translation` | Fixed base transform translation in metres. |
| `base_transform.rotation_rpy_deg` | Fixed XYZ roll/pitch/yaw angles in degrees. |
| `dh_parameters` | One standard DH row per joint: `a`, `d` in metres; `alpha_deg`; and `theta_offset_deg`. |
| `tool_transform.translation` | Fixed TCP transform translation in metres, relative to DH chain end. |
| `tool_transform.rotation_rpy_deg` | Fixed XYZ rotation in degrees. |
| `joint_limits.lower_deg`, `upper_deg` | Per-joint position limits in degrees, in `joint_names` order. |
| `joint_limits.velocity` | Per-joint maximum angular velocity in radians/second. |

The GP8's base and tool transforms align the DH frames with the URDF
`base_link` and tool center point. The same model and FK/Jacobian code serve
both robots; GP8 has no separate hard-coded kinematic chain. If changing
joint names or DH values, keep them consistent with the URDF/controller
configuration and trajectory joint ordering.

## Forward kinematics and Jacobian

Forward kinematics maps a joint vector in radians to a homogeneous transform
from the configured base frame to the TCP. `calculate_jacobian()` returns the
six-by-DOF geometric Jacobian together with the TCP transform. Joint vectors
must follow configured joint order. Position values in transforms use metres.

## Inverse kinematics choices

The full-pose solvers are:

- `damped_least_squares`: damped Jacobian update; often more robust near
  singular configurations but can converge slowly or fail from poor seeds.
- `jacobian_transpose`: iterative transpose-Jacobian update.
- `pseudoinverse`: Jacobian pseudoinverse update.

All solvers use the selected robot model and joint limits. Solver convergence
depends on reachability, orientation, seed configuration and proximity to
singularities. A solver success alone is not a collision-free guarantee.
Select a solver through `ik_solver` in a trajectory YAML file.

## Python API sketch

```python
from robot_arm_kinematics.robot_model import RobotModel
from robot_arm_kinematics.forward_kinematics import forward_kinematics
from robot_arm_kinematics.inverse_kinematics.damped_least_squares import (
    DampedLeastSquaresIK,
)

robot = RobotModel('/path/to/gp8.yaml')
q = [0.0] * robot.dof  # radians, in robot.joint_names order
tcp_transform = forward_kinematics(robot, q)
ik = DampedLeastSquaresIK(robot)
```

For exact function arguments and result types, see the package Python modules
and the trajectory package's pipeline example.

## Where the configs are used

- `gp8_trajectory_preview.launch.py` loads `gp8.yaml` by default.
- `trajectory_preview.launch.py` loads `generic_robot.yaml` by default.
- Gazebo GP8 pick/place loads `gp8.yaml` for measured TCP computation and
  planning.

Launch wrappers allow a different config where they expose `robot_config` or
`pose_config`; the generic trajectory preview has explicit `robot_config`.
Check the launch file's declared arguments for each entry point.
