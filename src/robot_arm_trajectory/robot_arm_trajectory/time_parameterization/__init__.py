"""Path time parameterization profiles."""

from .base import TimeParameterizer
from .quintic_scaling import QuinticScaling
from .trapezoidal import TrapezoidalProfile
from .seventh_order import SeventhOrderProfile
from .s_curve import SCurveProfile

TIME_PARAMETERIZERS = {
    "quintic_scaling": QuinticScaling,
    "trapezoidal": TrapezoidalProfile,
    "seventh_order": SeventhOrderProfile,
    # Deprecated compatibility alias. New applications should select
    # seventh_order, which accurately names this polynomial profile.
    "s_curve": SCurveProfile,
}

__all__ = ["QuinticScaling", "TrapezoidalProfile", "SeventhOrderProfile", "SCurveProfile",
           "TIME_PARAMETERIZERS", "generate_quintic_time_scaling"]


def generate_quintic_time_scaling(duration=4.0, num_samples=101):
    """Backward-compatible function for the original quintic profile API."""
    return QuinticScaling().generate(duration, num_samples)
