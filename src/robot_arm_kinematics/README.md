# Robot arm kinematics

`robot_arm_kinematics` provides a configurable robot model, standard Denavit–
Hartenberg (DH) forward kinematics, a geometric Jacobian, and three full-pose
inverse-kinematics solvers.

## Robot configurations

- `robot_arm_kinematics/config/generic_robot.yaml`: DH table and joint limits
  for the original generic arm.
- `robot_arm_kinematics/config/gp8.yaml`: DH table derived from the GP8 URDF's
  six revolute joint axes, plus fixed transforms that align the DH frames with
  `base_link` and the TCP.

Each config defines DOF, joint names, a base transform, one standard DH row per
joint (`a`, `alpha_deg`, `d`, `theta_offset_deg`), and joint position/velocity
limits. Distance values use metres; angles use degrees. Optional
`rotation_rpy_deg` fields store fixed XYZ Euler rotations for the DH base and
tool transforms. The GP8 table is used directly by the same FK and Jacobian
implementation as the generic table; there is no GP8-specific hard-coded
kinematic chain.

## Kinematics and IK

Forward kinematics maps a joint vector in radians to the TCP transform.
`calculate_jacobian()` returns the six-by-DOF geometric Jacobian and the same
TCP transform. The solver choices are:

- `damped_least_squares`: robust near singularities, but can converge slowly
  or fail from a poor seed.
- `jacobian_transpose`: simple iterative solver.
- `pseudoinverse`: Jacobian pseudoinverse update.

All solvers use the selected model's joint limits. The trajectory preview
uses the GP8 config when launched with `gp8_trajectory_preview.launch.py` and
the generic config for `trajectory_preview.launch.py` unless overridden.

## Build

```bash
colcon build --packages-select robot_arm_kinematics
source install/setup.bash
```
