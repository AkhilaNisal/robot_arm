"""Seventh-order smoothstep time scaling (not a seven-phase S-curve)."""

import numpy as np

from .base import TimeParameterizer, TimeScalingProfile
from .quintic_scaling import _validate


class SeventhOrderProfile(TimeParameterizer):
    """Seventh-degree rest-to-rest polynomial with zero endpoint jerk."""

    def generate(self, duration=4.0, num_samples=101, max_jerk=None, **kwargs):
        _validate(duration, num_samples)
        peak_jerk = 52.5 / duration**3
        if max_jerk is not None:
            if not np.isfinite(max_jerk) or max_jerk <= 0:
                raise ValueError("max_jerk must be positive and finite.")
            if peak_jerk > max_jerk:
                raise ValueError(
                    f"Requested duration requires peak path jerk {peak_jerk:g}, "
                    f"above max_jerk {max_jerk:g}."
                )
        t = np.linspace(0.0, duration, num_samples)
        u = t / duration
        s = 35*u**4 - 84*u**5 + 70*u**6 - 20*u**7
        velocity = (140*u**3 - 420*u**4 + 420*u**5 - 140*u**6) / duration
        acceleration = (420*u**2 - 1680*u**3 + 2100*u**4 - 840*u**5) / duration**2
        jerk = (840*u - 5040*u**2 + 8400*u**3 - 4200*u**4) / duration**3
        return TimeScalingProfile(t, s, velocity, acceleration, jerk)
