"""ROS 2 robotic arm trajectory planning utilities."""

from .interpolation import (
    CubicSplineInterpolation,
    LinearInterpolation,
    QuinticPolynomialInterpolation,
)
from .joint_space.joint_trajectory import (
    generate_joint_trajectory,
    JointTrajectory,
)
from .pipeline import (
    CartesianJointTrajectoryResult,
    generate_cartesian_joint_trajectory,
)
from .time_parameterization import (
    QuinticScaling,
    SCurveProfile,
    SeventhOrderProfile,
    TrapezoidalProfile,
)
from .validation import (
    TrajectoryValidationResult,
    validate_trajectory,
    ValidationCheck,
)

__all__ = [
    'generate_joint_trajectory',
    'JointTrajectory',
    'LinearInterpolation',
    'CubicSplineInterpolation',
    'QuinticPolynomialInterpolation',
    'QuinticScaling',
    'TrapezoidalProfile',
    'SeventhOrderProfile',
    'SCurveProfile',
    'CartesianJointTrajectoryResult',
    'generate_cartesian_joint_trajectory',
    'ValidationCheck',
    'TrajectoryValidationResult',
    'validate_trajectory',
]
