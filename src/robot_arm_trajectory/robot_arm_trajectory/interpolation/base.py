"""Shared interpolation interface."""

from abc import ABC, abstractmethod

import numpy as np


class Interpolator(ABC):
    """Define the common API for fitted joint path interpolators."""

    @abstractmethod
    def fit(self, waypoints, parameters=None, **kwargs):
        """Fit waypoints against a strictly increasing path parameter."""

    @abstractmethod
    def evaluate(self, parameter, derivative=0):
        """Evaluate a derivative with respect to path parameter."""

    @abstractmethod
    def position_extrema(self):
        """Return the minimum and maximum position for each joint."""
        pass


def validate_waypoints(waypoints, parameters=None):
    """Validate waypoint shape and their strictly increasing parameters."""
    q = np.asarray(waypoints, dtype=float)

    if q.ndim != 2 or q.shape[0] < 2 or q.shape[1] < 1:
        raise ValueError('Waypoints must have shape (at least 2, dof).')

    if not np.all(np.isfinite(q)):
        raise ValueError('Waypoints must contain finite values.')

    s = (
        np.linspace(0.0, 1.0, len(q))
        if parameters is None
        else np.asarray(parameters, dtype=float)
    )

    if (
        s.shape != (len(q),)
        or not np.all(np.isfinite(s))
        or np.any(np.diff(s) <= 0)
    ):
        raise ValueError(
            'Parameters must be finite, strictly increasing, '
            'and match waypoints.'
        )

    return q, s


def validate_parameter(parameter, parameters):
    """Validate finite evaluation parameters within the fitted domain."""
    x = np.asarray(parameter, dtype=float)

    if not np.all(np.isfinite(x)):
        raise ValueError(
            'Evaluation parameters must contain only finite values.'
        )

    lower = parameters[0]
    upper = parameters[-1]
    if np.any(x < lower):
        raise ValueError(
            'Evaluation parameter is below the interpolation '
            f'domain [{lower}, {upper}].'
        )
    if np.any(x > upper):
        raise ValueError(
            'Evaluation parameter is above the interpolation '
            f'domain [{lower}, {upper}].'
        )

    return x
