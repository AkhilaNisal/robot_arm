"""Piecewise quintic Hermite interpolation with endpoint derivatives."""

import math

import numpy as np

from scipy.interpolate import CubicSpline

from .base import Interpolator, validate_parameter, validate_waypoints


class QuinticPolynomialInterpolation(Interpolator):
    """Interpolate waypoints with piecewise quintic Hermite polynomials."""

    def fit(
        self, waypoints, parameters=None, velocities=None, accelerations=None,
        **kwargs
    ):
        """Fit quintic segments from positions and endpoint derivatives."""
        self.q, self.s = validate_waypoints(waypoints, parameters)

        n, dof = self.q.shape

        # Use derivatives of a single global cubic spline as coherent default
        # Hermite data. Differentiating sampled q and then differentiating the
        # result again with np.gradient amplifies roundoff and endpoint noise.
        # Explicit derivatives continue to take precedence.
        derivative_spline = None
        if velocities is None or accelerations is None:
            derivative_spline = CubicSpline(
                self.s, self.q, axis=0, bc_type='not-a-knot'
            )

        if velocities is None:
            self.v = derivative_spline(self.s, 1)
        else:
            self.v = np.asarray(velocities, dtype=float)

        # Estimate waypoint accelerations if not explicitly supplied.
        if accelerations is None:
            self.a = derivative_spline(self.s, 2)
        else:
            self.a = np.asarray(accelerations, dtype=float)

        if self.v.shape != self.q.shape or self.a.shape != self.q.shape:
            raise ValueError(
                'Velocity and acceleration boundary arrays must match '
                'waypoint shape.'
            )

        if not np.all(np.isfinite(self.v)) or not np.all(np.isfinite(self.a)):
            raise ValueError('Boundary derivatives must be finite.')

        if np.any(np.diff(self.s) <= 0):
            raise ValueError('Parameters must be strictly increasing.')

        self.coefficients = []

        for i, h in enumerate(np.diff(self.s)):
            c = np.zeros((6, dof))

            c[0] = self.q[i]
            c[1] = self.v[i]
            c[2] = self.a[i] / 2.0

            rhs = np.stack((
                self.q[i + 1] - c[0] - c[1] * h - c[2] * h**2,
                self.v[i + 1] - c[1] - 2 * c[2] * h,
                self.a[i + 1] - 2 * c[2]
            ))

            mat = np.array([
                [h**3, h**4, h**5],
                [3*h**2, 4*h**3, 5*h**4],
                [6*h, 12*h**2, 20*h**3]
            ])

            c[3:] = np.linalg.solve(mat, rhs)

            self.coefficients.append(c)

        return self

    def evaluate(self, parameter, derivative=0):
        """Evaluate position or a path-parameter derivative through order 3."""
        if derivative > 3 or derivative < 0:
            raise ValueError(
                'Quintic interpolation supports derivatives 0 through 3.'
            )
        x = validate_parameter(parameter, self.s)
        idx = np.clip(
            np.searchsorted(self.s, x, side='right') - 1,
            0,
            len(self.s) - 2,
        )
        tau = x - self.s[idx]
        c = np.asarray(self.coefficients)[idx]
        result = np.zeros(x.shape + (self.q.shape[1],))
        for k in range(derivative, 6):
            factor = math.factorial(k) / math.factorial(k-derivative)
            result += c[..., k, :] * factor * tau[..., None]**(k-derivative)
        return result

    def position_extrema(self):
        """Calculate exact position extrema over all quintic segments."""
        dof = self.q.shape[1]

        minimum = np.full(dof, np.inf)
        maximum = np.full(dof, -np.inf)

        for segment, c_matrix in enumerate(self.coefficients):

            h = self.s[segment + 1] - self.s[segment]

            for joint in range(dof):

                # Coefficients are stored in ascending-power order:
                # q(tau) = c0 + c1*tau + ... + c5*tau^5
                c = c_matrix[:, joint]

                # First derivative coefficients, also ascending order.
                derivative_coefficients = np.array([
                    c[1],
                    2.0 * c[2],
                    3.0 * c[3],
                    4.0 * c[4],
                    5.0 * c[5]
                ])

                # Remove negligible highest-order coefficients.
                scale = np.max(np.abs(derivative_coefficients))

                if scale > 0.0:
                    tolerance = 1e-12 * scale

                    while (
                        len(derivative_coefficients) > 1
                        and abs(derivative_coefficients[-1]) <= tolerance
                    ):
                        derivative_coefficients = derivative_coefficients[:-1]

                # Find stationary points.
                roots = np.polynomial.polynomial.polyroots(
                    derivative_coefficients
                )

                # Segment endpoints are always candidates.
                candidates = [0.0, h]

                for root in roots:

                    # Accept only approximately real roots.
                    if abs(root.imag) > 1e-9:
                        continue

                    tau = float(root.real)

                    # Keep roots inside this segment.
                    if -1e-10 <= tau <= h + 1e-10:
                        tau = np.clip(tau, 0.0, h)
                        candidates.append(tau)

                # Evaluate position at all candidates.
                positions = np.polynomial.polynomial.polyval(
                    candidates, c
                )

                minimum[joint] = min(
                    minimum[joint], np.min(positions)
                )

                maximum[joint] = max(
                    maximum[joint], np.max(positions)
                )

        return minimum, maximum
