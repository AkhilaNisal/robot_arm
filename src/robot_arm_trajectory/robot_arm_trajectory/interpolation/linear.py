"""Piecewise linear interpolation with analytic derivatives."""

import numpy as np

from .base import Interpolator, validate_parameter, validate_waypoints


class LinearInterpolation(Interpolator):
    """Interpolate joint waypoints with piecewise linear segments."""

    def fit(self, waypoints, parameters=None, **kwargs):
        """Fit waypoint positions and calculate segment slopes."""
        self.q, self.s = validate_waypoints(waypoints, parameters)
        self.slopes = np.diff(self.q, axis=0) / np.diff(self.s)[:, None]
        return self

    def evaluate(self, parameter, derivative=0):
        """Evaluate position or a supported path-parameter derivative."""
        x = validate_parameter(parameter, self.s)
        idx = np.clip(
            np.searchsorted(self.s, x, side='right') - 1,
            0,
            len(self.s) - 2
        )

        if derivative == 0:
            out = self.q[idx] + (
                (x - self.s[idx])[..., None] * self.slopes[idx]
            )
            return out

        if derivative == 1:
            return self.slopes[idx]

        if derivative in (2, 3):
            return np.zeros(x.shape + (self.q.shape[1],))

        raise ValueError('Only derivatives 0 through 3 are supported.')

    def position_extrema(self):
        """
        Calculate global minimum and maximum joint positions.

        Linear interpolation cannot overshoot its waypoint values.
        Therefore, the extrema occur at the waypoints.
        """
        minimum = np.min(self.q, axis=0)
        maximum = np.max(self.q, axis=0)

        return minimum, maximum
