"""Backward-compatible quintic time scaling API."""

import numpy as np
from .time_parameterization.base import TimeScalingProfile


def generate_quintic_time_scaling(duration=4.0, num_samples=101):
    """Generate a rest-to-rest quintic path scaling profile."""
    from .time_parameterization.quintic_scaling import QuinticScaling
    return QuinticScaling().generate(duration, num_samples)
