"""Joint-space interpolation implementations."""

from .cubic_spline import CubicSplineInterpolation
from .linear import LinearInterpolation
from .quintic_polynomial import QuinticPolynomialInterpolation

INTERPOLATORS = {
    'linear': LinearInterpolation,
    'cubic_spline': CubicSplineInterpolation,
    'quintic_polynomial': QuinticPolynomialInterpolation,
}

__all__ = [
    'CubicSplineInterpolation',
    'LinearInterpolation',
    'QuinticPolynomialInterpolation',
    'INTERPOLATORS',
]
