"""Symmetric acceleration, cruise and deceleration path profile."""

import numpy as np

from .base import TimeParameterizer, TimeScalingProfile
from .quintic_scaling import _validate


class TrapezoidalProfile(TimeParameterizer):
    """Trapezoidal velocity profile with analytically feasible phase times.

    Acceleration is piecewise constant and therefore discontinuous at phase
    transitions. At exact transitions this implementation reports the phase
    beginning at that time.
    """

    def generate(self, duration=4.0, num_samples=101, max_velocity=None,
                 max_acceleration=None, **kwargs):
        _validate(duration, num_samples)
        vmax = float(max_velocity) if max_velocity is not None else 2.0 / duration
        amax = float(max_acceleration) if max_acceleration is not None else 4.0 / duration**2
        if not np.isfinite(vmax) or not np.isfinite(amax) or vmax <= 0 or amax <= 0:
            raise ValueError("Velocity and acceleration limits must be positive and finite.")

        min_acceleration = 4.0 / duration**2
        # A profile with a nonzero acceleration phase requires ta>0, so the
        # lower bound is strict in theory; 1/T is its limiting peak velocity.
        min_velocity = 1.0 / duration
        if amax < min_acceleration - 1e-12:
            raise ValueError(
                f"Acceleration limit {amax:g} is infeasible for duration {duration:g}; "
                f"minimum required acceleration is {min_acceleration:g}."
            )
        if vmax < min_velocity - 1e-12:
            raise ValueError(
                f"Velocity limit {vmax:g} is infeasible for duration {duration:g}; "
                f"minimum required peak velocity is {min_velocity:g}."
            )

        # a_peak(ta) = 1/(ta*(T-ta)); feasible ta values form an interval
        # around T/2. The velocity constraint adds ta <= T - 1/vmax.
        disc = max(0.0, duration**2 - 4.0 / amax)
        ta_min = 0.5 * (duration - np.sqrt(disc))
        ta_max = min(duration - ta_min, duration - 1.0 / vmax)
        if ta_max < ta_min - 1e-12:
            raise ValueError(
                "No acceleration duration satisfies both max_velocity and "
                "max_acceleration for the requested duration."
            )
        # Larger ta up to T/2 lowers peak speed while retaining feasibility.
        t_acc = min(duration / 2.0, ta_max)
        t_cruise = duration - 2.0 * t_acc
        peak = 1.0 / (duration - t_acc)
        a_peak = 1.0 / (t_acc * (duration - t_acc))

        t = np.linspace(0.0, duration, num_samples)
        s = np.empty_like(t)
        v = np.empty_like(t)
        a = np.empty_like(t)
        for k, tk in enumerate(t):
            if tk < t_acc:
                a[k] = a_peak
                v[k] = a_peak * tk
                s[k] = 0.5 * a_peak * tk**2
            elif tk < t_acc + t_cruise:
                dt = tk - t_acc
                a[k] = 0.0
                v[k] = peak
                s[k] = 0.5 * a_peak * t_acc**2 + peak * dt
            else:
                dt = tk - t_acc - t_cruise
                a[k] = -a_peak
                v[k] = peak - a_peak * dt
                s[k] = (0.5 * a_peak * t_acc**2 + peak * t_cruise
                        + peak * dt - 0.5 * a_peak * dt**2)
        # Preserve exact boundary positions/velocities; keep phase acceleration
        # values at both endpoints rather than concealing its discontinuity.
        s[0], s[-1] = 0.0, 1.0
        v[0], v[-1] = 0.0, 0.0
        return TimeScalingProfile(t, s, v, a, None)
