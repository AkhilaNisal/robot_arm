"""Cubic spline interpolation backed by SciPy."""

import numpy as np

from scipy.interpolate import CubicSpline

from .base import Interpolator, validate_parameter, validate_waypoints


class CubicSplineInterpolation(Interpolator):
    """Interpolate joint waypoints with SciPy's cubic spline."""

    def fit(self, waypoints, parameters=None, bc_type='not-a-knot', **kwargs):
        """Fit cubic splines using the selected boundary conditions."""
        self.q, self.s = validate_waypoints(waypoints, parameters)

        self.spline = CubicSpline(
            self.s,
            self.q,
            axis=0,
            bc_type=bc_type
        )

        return self

    def evaluate(self, parameter, derivative=0):
        """Evaluate position or a path-parameter derivative through order 3."""
        if derivative > 3 or derivative < 0:
            raise ValueError(
                'Cubic spline supports derivatives 0 through 3.'
            )

        x = validate_parameter(parameter, self.s)
        return self.spline(x, nu=derivative)

    def position_extrema(self):
        """Return global minimum and maximum joint positions."""
        minimum = np.full(self.q.shape[1], np.inf)
        maximum = np.full(self.q.shape[1], -np.inf)

        # SciPy stores coefficients in descending-power order:
        #
        # q(x) = c[0]*x^3 + c[1]*x^2 + c[2]*x + c[3]
        #
        # where x = s - s[i].

        for segment in range(len(self.s) - 1):

            x_start = 0.0
            x_end = self.s[segment + 1] - self.s[segment]

            for joint in range(self.q.shape[1]):

                c = self.spline.c[:, segment, joint]
                # Derivative:
                # q'(x) = 3*c[0]*x^2 + 2*c[1]*x + c[2]

                derivative_coefficients = np.array([
                    3.0 * c[0],
                    2.0 * c[1],
                    c[2]
                ])

                # Always include segment endpoints.
                candidates = [x_start, x_end]

                # Remove leading coefficients that are effectively zero.
                scale = np.max(np.abs(derivative_coefficients))

                if scale > 0.0:
                    normalized = derivative_coefficients / scale

                    first_nonzero = np.flatnonzero(
                        np.abs(normalized) > 1e-12
                    )

                    if len(first_nonzero) > 0:

                        polynomial = normalized[first_nonzero[0]:]

                        if len(polynomial) > 1:

                            roots = np.roots(polynomial)

                            for root in roots:

                                if abs(root.imag) > 1e-10:
                                    continue

                                x = float(root.real)

                                tolerance = 1e-10 * max(
                                    1.0,
                                    abs(x_start),
                                    abs(x_end)
                                )

                                if (
                                    x_start - tolerance
                                    <= x
                                    <= x_end + tolerance
                                ):
                                    x = np.clip(x, x_start, x_end)
                                    candidates.append(x)

                # Evaluate the original cubic polynomial.
                values = [
                    np.polyval(c, x)
                    for x in candidates
                ]

                minimum[joint] = min(
                    minimum[joint],
                    min(values)
                )

                maximum[joint] = max(
                    maximum[joint],
                    max(values)
                )

        return minimum, maximum
