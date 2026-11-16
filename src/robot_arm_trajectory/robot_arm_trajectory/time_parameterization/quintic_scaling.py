"""Quintic rest-to-rest path scaling."""

import numpy as np
from .base import TimeScalingProfile
from .base import TimeParameterizer


class QuinticScaling(TimeParameterizer):
    def generate(self, duration=4.0, num_samples=101, **kwargs):
        _validate(duration, num_samples)
        t = np.linspace(0.0, duration, num_samples)
        u = t / duration
        s = 10*u**3 - 15*u**4 + 6*u**5
        v = (30*u**2 - 60*u**3 + 30*u**4) / duration
        a = (60*u - 180*u**2 + 120*u**3) / duration**2
        j = (60 - 360*u + 360*u**2) / duration**3
        return TimeScalingProfile(t, s, v, a, j)


def _validate(duration, num_samples):
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError("Duration must be positive and finite.")
    if not isinstance(num_samples, int) or num_samples < 2:
        raise ValueError("At least two samples are required.")
