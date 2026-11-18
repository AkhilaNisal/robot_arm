# Robot arm kinematics

`robot_arm_kinematics` is a Python library for configurable robot models,
standard Denavit–Hartenberg (DH) forward kinematics, a geometric Jacobian and
three full-pose inverse-kinematics solvers. The standalone preview and Gazebo
pick/place planner both use the GP7 configuration from this package.

## Build

```bash
cd ~/robot_arm
source /opt/ros/jazzy/setup.bash
colcon build --packages-select robot_arm_kinematics
source install/setup.bash
```

## Model YAML files

Installed model files are under `robot_arm_kinematics/config/`:

| File | Model |
| --- | --- |
| `generic_robot.yaml` | Generic six-axis example model. |
| `gp7.yaml` | GP7 DH chain and fixed transforms aligned with the URDF. |

The top-level keys and units are:

| Key | Content / units |
| --- | --- |
| `robot.name` | Display/model name. |
| `robot.dof` | Positive integer number of joints. |
| `robot.joint_names` | Ordered list with exactly one unique name per DOF. This order is used throughout the library and controller mapping. |
| `base_transform.translation` | XYZ translation in metres. |
| `base_transform.rotation_rpy_deg` | Fixed XYZ roll, pitch, yaw in degrees. |
| `dh_parameters` | One standard DH row per joint. `a` and `d` are metres; `alpha_deg` and `theta_offset_deg` are degrees. |
| `tool_transform.translation` | TCP translation relative to the final DH frame, metres. |
| `tool_transform.rotation_rpy_deg` | Fixed TCP roll, pitch, yaw in degrees. |
| `joint_limits.lower_deg`, `upper_deg` | One lower/upper angle per joint, degrees, matching `joint_names` order. |
| `joint_limits.velocity` | Positive maximum angular speed per joint, radians/second. |

Keep the joint names/order consistent with the URDF, controller YAML and
trajectory data. The fixed GP7 base/tool transforms align the DH frames to
`base_link` and the TCP; changing them changes pose targets and FK output.

## Library modules and conventions

| Module | Responsibility |
| --- | --- |
| `robot_model.py` | Loads and validates robot configuration; exposes DOF, joint names, limits and transforms. |
| `forward_kinematics.py` | Builds the base-to-TCP homogeneous transform from joint values. |
| `jacobian.py` | Calculates the geometric Jacobian and TCP transform. |
| `inverse_kinematics/base.py` | Shared solver interface and iterative result handling. |
| `inverse_kinematics/damped_least_squares.py` | Damped Jacobian least-squares updates. |
| `inverse_kinematics/jacobian_transpose.py` | Jacobian-transpose updates. |
| `inverse_kinematics/pseudoinverse.py` | Jacobian pseudoinverse updates. |

Joint vectors `q` are finite radians in configured joint order. Transforms are
4×4 homogeneous matrices; positions are metres. Full-pose IK needs a reachable
target and a useful seed. Joint limits are enforced by the solver, but success
does not mean a path to the target is collision-free.

## Forward kinematics and Jacobian

Forward kinematics maps a joint vector to a transform from the configured base
frame to the TCP. `calculate_jacobian()` returns the six-by-DOF geometric
Jacobian together with the TCP transform. The upper Jacobian rows describe
linear TCP velocity, and the lower rows describe angular velocity.

```python
from robot_arm_kinematics.robot_model import RobotModel
from robot_arm_kinematics.forward_kinematics import forward_kinematics
from robot_arm_kinematics.inverse_kinematics.damped_least_squares import (
    DampedLeastSquaresIK,
)

robot = RobotModel('/path/to/gp7.yaml')
q = [0.0] * robot.dof  # radians, in robot.joint_names order
tcp_transform = forward_kinematics(robot, q)
ik = DampedLeastSquaresIK(robot)
```

## IK solver selection and tuning

| Solver key | Behavior |
| --- | --- |
| `damped_least_squares` | Adds damping to the Jacobian update; often more robust near singularities, though convergence depends on seed and tolerances. |
| `jacobian_transpose` | Uses a transpose-Jacobian update; may need tuning for convergence speed. |
| `pseudoinverse` | Uses a pseudoinverse update; can be sensitive near singular configurations. |

Planner configs select the solver with `ik_solver`. Preview/Gazebo planner
configs also provide `ik_max_iterations`, `ik_position_tolerance` (metres),
`ik_orientation_tolerance` (radians), and `ik_restarts`. Iterations must be
at least 1 and restart count must be 0 or greater. Smaller tolerances demand a
closer pose match and can increase solve time or cause failure. Restarts give
IK more seeds to try, increasing the planning work. Use values appropriate
for the target resolution and robot; all tolerances must be positive and
finite.

## Consumers in this workspace

- `robot_arm_description/launch/trajectory_preview.launch.py` defaults to
  `generic_robot.yaml` and accepts a `robot_config` path.
- `robot_arm_description/launch/gp7_trajectory_preview.launch.py` selects
  `gp7.yaml`.
- Gazebo's GP7 planner uses the same model for IK and measured TCP FK.

The package installs a Python library rather than a standalone executable.
Trajectory planning and the RViz/Gazebo nodes are documented in the
[`robot_arm_trajectory`](../robot_arm_trajectory/README.md) and
[`robot_arm_gazebo`](../robot_arm_gazebo/README.md) guides.
