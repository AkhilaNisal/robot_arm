import numpy as np

from robot_arm_kinematics.inverse_kinematics.base import (
    IKResult,
    IKSolver,
    validate_target_pose,
)

from robot_arm_kinematics.jacobian import calculate_jacobian


def rotation_error(R_desired, R_current):
    """
    Calculate orientation error as a rotation vector
    expressed in the base frame.
    """

    R_error = R_desired @ R_current.T

    cos_theta = np.clip(
        (np.trace(R_error) - 1.0) / 2.0,
        -1.0,
        1.0
    )

    theta = np.arccos(cos_theta)

    vee = np.array([
        R_error[2, 1] - R_error[1, 2],
        R_error[0, 2] - R_error[2, 0],
        R_error[1, 0] - R_error[0, 1]
    ])

    if theta < 1e-8:
        return 0.5 * vee

    if np.pi - theta < 1e-5:
        # Near 180 degrees, use the eigenvector
        # associated with the largest eigenvalue.
        values, vectors = np.linalg.eig(R_error)

        index = np.argmin(np.abs(values - 1.0))

        axis = np.real(vectors[:, index])
        axis = axis / np.linalg.norm(axis)

        return theta * axis

    return (theta / (2.0 * np.sin(theta))) * vee


class DampedLeastSquaresIK(IKSolver):

    def __init__(
        self,
        robot,
        damping=0.05,
        characteristic_length=0.25,
        max_joint_step=0.15,
        position_tolerance=1e-4,
        orientation_tolerance=1e-3,
        max_iterations=1000
    ):

        super().__init__(
            robot,
            position_tolerance,
            orientation_tolerance,
            max_iterations
        )

        self.damping = damping
        self.characteristic_length = characteristic_length
        self.max_joint_step = max_joint_step

        if self.damping <= 0:
            raise ValueError("Damping must be positive.")

        if self.characteristic_length <= 0:
            raise ValueError(
                "Characteristic length must be positive."
            )

        if self.max_joint_step <= 0:
            raise ValueError(
                "Maximum joint step must be positive."
            )

        # Position and orientation task weighting
        self.W = np.diag([
            1.0, 1.0, 1.0,
            characteristic_length,
            characteristic_length,
            characteristic_length
        ])

    def solve(self, target_pose, q_initial):

        target_pose = validate_target_pose(target_pose)
        q = np.asarray(q_initial, dtype=float).copy()

        if q.shape != (self.robot.dof,):
            raise ValueError("Invalid initial joint configuration.")

        if not np.all(np.isfinite(q)):
            raise ValueError("Initial joint angles must be finite.")

        R_desired = target_pose[:3, :3]
        p_desired = target_pose[:3, 3]

        q = self.robot.enforce_joint_limits(q)

        for iteration in range(self.max_iterations):

            J, T_current = calculate_jacobian(
                self.robot, q
            )

            p_current = T_current[:3, 3]
            R_current = T_current[:3, :3]

            # Calculate position and orientation errors
            e_position = p_desired - p_current

            e_orientation = rotation_error(
                R_desired,
                R_current
            )

            position_norm = np.linalg.norm(e_position)
            orientation_norm = np.linalg.norm(e_orientation)

            # Check convergence
            if (
                position_norm < self.position_tolerance
                and orientation_norm < self.orientation_tolerance
            ):

                return IKResult(
                    joint_positions=q,
                    success=True,
                    iterations=iteration,
                    position_error=position_norm,
                    orientation_error=orientation_norm,
                    message="Full-pose IK converged."
                )

            # Combine pose errors
            error = np.concatenate([
                e_position,
                e_orientation
            ])

            # Weighted task space
            J_weighted = self.W @ J
            error_weighted = self.W @ error

            # Damped Least Squares
            dq = J_weighted.T @ np.linalg.solve(
                J_weighted @ J_weighted.T
                + self.damping**2 * np.eye(6),
                error_weighted
            )

            # Limit the maximum joint update
            max_step = np.max(np.abs(dq))

            if max_step > self.max_joint_step:
                dq *= self.max_joint_step / max_step

            # Update and enforce joint limits
            q = self.robot.enforce_joint_limits(q + dq)

        # Calculate final errors after maximum iterations
        _, T_final = calculate_jacobian(self.robot, q)

        final_position_error = np.linalg.norm(
            p_desired - T_final[:3, 3]
        )

        final_orientation_error = np.linalg.norm(
            rotation_error(
                R_desired,
                T_final[:3, :3]
            )
        )

        success = (
            final_position_error < self.position_tolerance
            and final_orientation_error < self.orientation_tolerance
        )

        return IKResult(
            joint_positions=q,
            success=success,
            iterations=self.max_iterations,
            position_error=final_position_error,
            orientation_error=final_orientation_error,
            message=(
                "Full-pose IK converged."
                if success
                else "Maximum iterations reached without convergence."
            )
        )
