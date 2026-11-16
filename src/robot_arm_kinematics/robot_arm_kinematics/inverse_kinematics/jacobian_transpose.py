import numpy as np

from robot_arm_kinematics.inverse_kinematics.base import (
    IKResult,
    IKSolver,
    validate_target_pose,
)
from robot_arm_kinematics.inverse_kinematics.damped_least_squares import (
    rotation_error
)
from robot_arm_kinematics.jacobian import calculate_jacobian


class JacobianTransposeIK(IKSolver):

    def __init__(
        self,
        robot,
        gain=0.5,
        characteristic_length=0.25,
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

        self.gain = gain
        self.characteristic_length = characteristic_length

        if gain <= 0 or characteristic_length <= 0:
            raise ValueError("Gain and characteristic length must be positive.")

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
            raise ValueError("Initial joint vector has incorrect shape.")

        if not np.all(np.isfinite(target_pose)) or not np.all(np.isfinite(q)):
            raise ValueError("Inputs must contain finite values.")

        q = self.robot.enforce_joint_limits(q)

        for iteration in range(self.max_iterations):

            J, current_pose = calculate_jacobian(self.robot, q)

            position_error = (
                target_pose[:3, 3] - current_pose[:3, 3]
            )

            orientation_error = rotation_error(
                target_pose[:3, :3],
                current_pose[:3, :3]
            )

            p_error = np.linalg.norm(position_error)
            r_error = np.linalg.norm(orientation_error)

            if (
                p_error < self.position_tolerance
                and r_error < self.orientation_tolerance
            ):
                return IKResult(
                    q, True, iteration,
                    p_error, r_error,
                    "Jacobian Transpose IK converged."
                )

            error = np.concatenate([
                position_error,
                orientation_error
            ])

            J_weighted = self.W @ J
            error_weighted = self.W @ error

            dq = self.gain * J_weighted.T @ error_weighted

            max_step = 0.15
            max_change = np.max(np.abs(dq))

            if max_change > max_step:
                dq *= max_step / max_change

            q = self.robot.enforce_joint_limits(q + dq)

        J, current_pose = calculate_jacobian(self.robot, q)

        p_error = np.linalg.norm(
            target_pose[:3, 3] - current_pose[:3, 3]
        )

        r_error = np.linalg.norm(
            rotation_error(
                target_pose[:3, :3],
                current_pose[:3, :3]
            )
        )

        return IKResult(
            q, False, self.max_iterations,
            p_error, r_error,
            "Maximum iterations reached."
        )
