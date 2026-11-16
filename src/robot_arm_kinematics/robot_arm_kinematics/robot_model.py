import numpy as np
import yaml


class RobotModel:
    """
    Loads and stores the kinematic parameters of a robot.

    DH parameters use the standard DH convention:
        [a, alpha, d, theta_offset]

    All angular values are converted to radians internally.
    """

    def __init__(self, config_file):

        # Load YAML configuration
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)

        robot_config = config['robot']

        # Basic robot information
        self.name = robot_config['name']
        self.dof = int(robot_config['dof'])

        self.joint_names = robot_config['joint_names']

        # Check number of joints
        if len(self.joint_names) != self.dof:
            raise ValueError(
                "Number of joint names must match robot DOF."
            )

        # Base transformation
        self.base_transform = self._load_transform(
            robot_config['base_transform'], 'Base'
        )
        self.tool_transform = self._load_transform(
            robot_config.get('tool_transform', {}), 'Tool'
        )

        # Load DH parameters
        dh_data = robot_config['dh_parameters']

        if len(dh_data) != self.dof:
            raise ValueError(
                "Number of DH rows must match robot DOF."
            )

        self.dh_parameters = np.array([
            [
                row['a'],
                np.deg2rad(row['alpha_deg']),
                row['d'],
                np.deg2rad(row['theta_offset_deg'])
            ]
            for row in dh_data
        ], dtype=float)

        # Load joint limits
        limits = robot_config['joint_limits']

        self.lower_limits = np.deg2rad(
            np.asarray(limits['lower_deg'], dtype=float)
        )

        self.upper_limits = np.deg2rad(
            np.asarray(limits['upper_deg'], dtype=float)
        )

        self.velocity_limits = np.asarray(
            limits['velocity'], dtype=float
        )

        # Validate dimensions and values
        if (
            self.lower_limits.shape != (self.dof,)
            or self.upper_limits.shape != (self.dof,)
            or self.velocity_limits.shape != (self.dof,)
        ):
            raise ValueError(
                "Joint limit arrays must match robot DOF."
            )

        if (not np.all(np.isfinite(self.lower_limits))
                or not np.all(np.isfinite(self.upper_limits))):
            raise ValueError("Joint position limits must be finite.")

        if not np.all(np.isfinite(self.velocity_limits)):
            raise ValueError("Joint velocity limits must be finite.")

        if not np.all(np.isfinite(self.dh_parameters)):
            raise ValueError("DH parameters must be finite.")

        if np.any(self.lower_limits >= self.upper_limits):
            raise ValueError(
                "Every lower joint limit must be below its upper limit."
            )

        if np.any(self.velocity_limits <= 0):
            raise ValueError(
                "Joint velocity limits must be positive."
            )

        if not np.all(np.isfinite(self.base_transform)):
            raise ValueError("Base transform must be finite.")

        print(f"Robot model loaded: {self.name}")
        print(f"Degrees of freedom: {self.dof}")

    @staticmethod
    def _load_transform(data, label):
        """Load translation and optional XYZ Euler rotation in degrees."""
        translation = np.asarray(data.get('translation', [0, 0, 0]), dtype=float)
        rpy = np.deg2rad(np.asarray(
            data.get('rotation_rpy_deg', [0, 0, 0]), dtype=float
        ))
        if translation.shape != (3,) or rpy.shape != (3,):
            raise ValueError(
                f'{label} transform translation and rotation must each '
                'contain three values.'
            )
        if not np.all(np.isfinite(translation)) or not np.all(np.isfinite(rpy)):
            raise ValueError(f'{label} transform values must be finite.')
        rx, ry, rz = rpy
        cx, sx = np.cos(rx), np.sin(rx)
        cy, sy = np.cos(ry), np.sin(ry)
        cz, sz = np.cos(rz), np.sin(rz)
        rotation = np.array([
            [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
            [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
            [-sy, cy * sx, cy * cx],
        ])
        transform = np.eye(4)
        transform[:3, :3] = rotation
        transform[:3, 3] = translation
        return transform

    def check_joint_limits(self, q):
        """
        Check whether a joint configuration satisfies joint limits.
        """

        q = np.asarray(q, dtype=float)

        if q.shape != (self.dof,):
            raise ValueError("Invalid joint configuration size.")

        if not np.all(np.isfinite(q)):
            raise ValueError("Joint configuration must contain finite values.")

        return np.all(
            (q >= self.lower_limits)
            & (q <= self.upper_limits)
        )

    def enforce_joint_limits(self, q):
        """
        Clamp joint angles to their permitted ranges.
        """

        q = np.asarray(q, dtype=float)

        if q.shape != (self.dof,):
            raise ValueError("Invalid joint configuration size.")

        if not np.all(np.isfinite(q)):
            raise ValueError("Joint configuration must contain finite values.")

        return np.clip(
            q,
            self.lower_limits,
            self.upper_limits
        )

    def get_joint_limit_summary(self):
        """
        Return joint limits in degrees for display.
        """

        return {
            'lower_deg': np.rad2deg(self.lower_limits),
            'upper_deg': np.rad2deg(self.upper_limits),
            'velocity_rad_s': self.velocity_limits
        }
