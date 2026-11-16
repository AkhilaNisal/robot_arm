"""Joint trajectory generation from independent geometry and timing choices."""

from dataclasses import dataclass
import numpy as np
from scipy.optimize import minimize_scalar

from .interpolation import INTERPOLATORS
from .time_parameterization import TIME_PARAMETERIZERS
from .time_parameterization.quintic_scaling import QuinticScaling
from .time_parameterization.trapezoidal import TrapezoidalProfile
from .time_parameterization.seventh_order import SeventhOrderProfile


@dataclass
class JointTrajectory:
    time: np.ndarray
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray
    jerks: np.ndarray | None = None


def generate_joint_trajectory(
    joint_waypoints, duration=4.0, num_samples=101,
    interpolation_method="linear", time_parameterization="quintic_scaling",
    interpolation_options=None, time_parameterization_options=None,
    joint_position_limits=None, joint_velocity_limits=None,
    joint_acceleration_limits=None, joint_jerk_limits=None,
    enforce_position_limits=False, enforce_velocity_limits=False,
    enforce_acceleration_limits=False, enforce_jerk_limits=False,
    robot_model=None,
):
    """Generate a joint trajectory using independent interpolation and timing.

    When ``robot_model`` is supplied, its position and velocity limits are
    used unless corresponding explicit limits are also given. Explicit and
    model limits must match within numeric tolerance.

    Position-limit enforcement checks analytical interpolation extrema,
    including positions between generated trajectory samples.
    """
    q = np.asarray(joint_waypoints, dtype=float)

    if (
        q.ndim != 2
        or q.shape[0] < 2
        or q.shape[1] < 1
        or not np.all(np.isfinite(q))
    ):
        raise ValueError(
            "Joint waypoints must be a finite (at least 2, dof) array."
        )

    # Resolve limits from RobotModel when provided.
    if robot_model is not None:
        if robot_model.dof != q.shape[1]:
            raise ValueError(
                f"RobotModel DOF ({robot_model.dof}) does not match "
                f"waypoint DOF ({q.shape[1]})."
            )

        model_position = _position_limits_from_model(robot_model)

        joint_position_limits = _resolve_model_limit(
            "position", joint_position_limits, model_position
        )

        joint_velocity_limits = _resolve_model_limit(
            "velocity", joint_velocity_limits,
            robot_model.velocity_limits
        )

    # Validate selected algorithms.
    if interpolation_method not in INTERPOLATORS:
        raise ValueError(
            f"Unknown interpolation method: {interpolation_method}"
        )

    if time_parameterization not in TIME_PARAMETERIZERS:
        raise ValueError(
            f"Unknown time parameterization: {time_parameterization}"
        )

    # Validate supplied constraints even when enforcement is disabled.
    if joint_position_limits is not None:
        _validate_position_limits(joint_position_limits, q.shape[1])

    for name, limits in (
        ("velocity", joint_velocity_limits),
        ("acceleration", joint_acceleration_limits),
        ("jerk", joint_jerk_limits),
    ):
        if limits is not None:
            _validate_magnitude_limits(name, limits, q.shape[1])

    # Fit the selected interpolation method.
    s_waypoints = np.linspace(0.0, 1.0, len(q))

    interpolator = INTERPOLATORS[interpolation_method]().fit(
        q,
        s_waypoints,
        **(interpolation_options or {})
    )

    # Generate the selected time parameterization profile.
    profile = TIME_PARAMETERIZERS[time_parameterization]().generate(
        duration=duration,
        num_samples=num_samples,
        **(time_parameterization_options or {})
    )

    s = profile.s

    # Evaluate interpolation derivatives.
    positions = interpolator.evaluate(s, 0)
    dq = interpolator.evaluate(s, 1)
    d2q = interpolator.evaluate(s, 2)
    d3q = interpolator.evaluate(s, 3)

    # Apply the chain rule to obtain time derivatives.
    velocities = dq * profile.velocity[:, None]

    accelerations = (
        d2q * profile.velocity[:, None]**2
        + dq * profile.acceleration[:, None]
    )

    jerks = None

    if profile.jerk is not None:
        jerks = (
            d3q * profile.velocity[:, None]**3
            + 3 * d2q * profile.velocity[:, None]
            * profile.acceleration[:, None]
            + dq * profile.jerk[:, None]
        )

    # ---------------------------------------------------------
    # Position-limit enforcement
    # ---------------------------------------------------------
    if enforce_position_limits:

        lower, upper = _validate_position_limits(
            joint_position_limits,
            q.shape[1]
        )

        # Calculate analytical extrema over the entire
        # interpolation domain, including between samples.
        minimum, maximum = interpolator.position_extrema()

        if (
            np.any(minimum < lower - 1e-10)
            or np.any(maximum > upper + 1e-10)
        ):
            raise ValueError(
                "Interpolated trajectory violates joint position limits."
            )

        # Additional validation of generated sample positions.
        if (
            np.any(positions < lower[None, :] - 1e-10)
            or np.any(positions > upper[None, :] + 1e-10)
        ):
            raise ValueError(
                "Generated positions violate joint position limits."
            )

    # ---------------------------------------------------------
    # Velocity, acceleration and jerk-limit enforcement
    # ---------------------------------------------------------
    _check_limit(
        "velocity",
        velocities,
        joint_velocity_limits,
        enforce_velocity_limits,
        interpolator=interpolator,
        profile=profile,
        parameterizer=time_parameterization,
        derivative=1,
    )

    _check_limit(
        "acceleration",
        accelerations,
        joint_acceleration_limits,
        enforce_acceleration_limits,
        interpolator=interpolator,
        profile=profile,
        parameterizer=time_parameterization,
        derivative=2,
    )

    _check_limit(
        "jerk",
        jerks,
        joint_jerk_limits,
        enforce_jerk_limits,
        interpolator=interpolator,
        profile=profile,
        parameterizer=time_parameterization,
        derivative=3,
    )

    return JointTrajectory(
        profile.time,
        positions,
        velocities,
        accelerations,
        jerks
    )


def _check_limit(name, values, limits, enforce, interpolator=None,
                 profile=None, parameterizer=None, derivative=1):
    """Check absolute derivative values against configured limits."""

    if not enforce:
        return

    if limits is None:
        raise ValueError(
            f"{name.capitalize()} limit enforcement requested "
            "but limits are unavailable."
        )

    if values is None:
        raise ValueError(
            f"{name.capitalize()} limits requested but this "
            f"profile does not provide {name}."
        )

    bound = _validate_magnitude_limits(name, limits, values.shape[1])

    maxima = np.max(np.abs(values), axis=0)
    if interpolator is not None and profile is not None:
        maxima = _estimate_derivative_extrema(
            interpolator, profile, derivative, values,
            parameterizer=parameterizer,
        )

    if np.any(maxima > bound + 1e-10):
        raise ValueError(
            f"Generated {name} violates configured joint limits "
            "(continuous-domain extrema estimate)."
        )


def _estimate_derivative_extrema(interpolator, profile, derivative, sampled,
                                 parameterizer=None):
    """Estimate time-derivative extrema by bounded scalar optimization.

    Profile derivatives and polynomial interpolation derivatives are
    analytical. The scalar time composition is not generally polynomial
    across segment crossings, so each joint/sample interval is optimized
    numerically with endpoints included. This is conservative in coverage,
    though unlike polynomial root isolation it is not a formal certificate.
    """
    time = np.unique(np.concatenate((
        profile.time,
        np.linspace(profile.time[0], profile.time[-1], max(1001, len(profile.time))),
    )))

    duration = float(profile.time[-1])

    def evaluate(t):
        if parameterizer is QuinticScaling:
            u = t / duration
            s = 10*u**3 - 15*u**4 + 6*u**5
            sd = (30*u**2 - 60*u**3 + 30*u**4) / duration
            sdd = (60*u - 180*u**2 + 120*u**3) / duration**2
            sj = (60 - 360*u + 360*u**2) / duration**3
        elif parameterizer is SeventhOrderProfile:
            u = t / duration
            s = 35*u**4 - 84*u**5 + 70*u**6 - 20*u**7
            sd = (140*u**3 - 420*u**4 + 420*u**5 - 140*u**6) / duration
            sdd = (420*u**2 - 1680*u**3 + 2100*u**4 - 840*u**5) / duration**2
            sj = (840*u - 5040*u**2 + 8400*u**3 - 4200*u**4) / duration**3
        elif parameterizer is TrapezoidalProfile:
            # Profile transition events are on the sampling grid used for
            # optimization below. The piecewise profile itself is not C2/C3.
            s = np.interp(t, profile.time, profile.s)
            sd = np.interp(t, profile.time, profile.velocity)
            sdd = np.interp(t, profile.time, profile.acceleration)
            sj = 0.0
        else:
            s = np.interp(t, profile.time, profile.s)
            sd = np.interp(t, profile.time, profile.velocity)
            sdd = np.interp(t, profile.time, profile.acceleration)
            sj = 0.0 if profile.jerk is None else np.interp(t, profile.time, profile.jerk)
        q1 = np.asarray(interpolator.evaluate(s, 1))
        q2 = np.asarray(interpolator.evaluate(s, 2))
        q3 = np.asarray(interpolator.evaluate(s, 3))
        if derivative == 1:
            return q1 * sd
        if derivative == 2:
            return q2 * sd**2 + q1 * sdd
        return q3 * sd**3 + 3.0 * q2 * sd * sdd + q1 * sj

    maxima = np.max(np.abs(sampled), axis=0)
    for left, right in zip(time[:-1], time[1:]):
        # Include optimizer probes around the interval; this is substantially
        # safer than checking only caller-selected output samples.
        for joint in range(sampled.shape[1]):
            result = minimize_scalar(
                lambda t: -abs(evaluate(t)[joint]),
                bounds=(left, right), method="bounded",
                options={"xatol": 1e-12},
            )
            maxima[joint] = max(
                maxima[joint], abs(evaluate(left)[joint]),
                abs(evaluate(right)[joint]), -result.fun
            )
    return maxima


def _validate_position_limits(limits, dof):
    """Validate lower and upper joint position limits."""

    if limits is None:
        raise ValueError(
            "Position limit enforcement requested "
            "but position limits are unavailable."
        )

    bounds = np.asarray(limits, dtype=float)

    if bounds.shape == (2, dof):
        lower, upper = bounds

    elif bounds.shape == (dof, 2):
        lower, upper = bounds[:, 0], bounds[:, 1]

    else:
        raise ValueError(
            "Position limits must have shape (2,dof) or (dof,2)."
        )

    if (
        not np.all(np.isfinite(lower))
        or not np.all(np.isfinite(upper))
        or np.any(lower >= upper)
    ):
        raise ValueError(
            "Position limits must be finite and each lower "
            "limit below its upper limit."
        )

    return lower, upper


def _validate_magnitude_limits(name, limits, dof):
    """Validate positive finite velocity, acceleration or jerk limits."""

    bound = np.asarray(limits, dtype=float)

    if bound.ndim == 0:
        bound = np.full(dof, bound)

    if (
        bound.shape != (dof,)
        or not np.all(np.isfinite(bound))
        or np.any(bound <= 0)
    ):
        raise ValueError(
            f"{name.capitalize()} limits must be positive finite "
            "values matching DOF."
        )

    return bound


def _position_limits_from_model(robot_model):
    """Extract position limits from a RobotModel."""

    lower = np.asarray(robot_model.lower_limits, dtype=float)
    upper = np.asarray(robot_model.upper_limits, dtype=float)

    if (
        lower.shape != (robot_model.dof,)
        or upper.shape != (robot_model.dof,)
    ):
        raise ValueError(
            "RobotModel position limits must match its DOF."
        )

    return np.stack((lower, upper))


def _resolve_model_limit(name, explicit, model):
    """Resolve explicit limits against RobotModel limits."""

    model_values = np.asarray(model, dtype=float)

    if explicit is None:
        return model_values

    explicit_values = np.asarray(explicit, dtype=float)

    if (
        name == "position"
        and explicit_values.shape == (model_values.shape[1], 2)
    ):
        explicit_values = explicit_values.T

    if explicit_values.shape != model_values.shape:
        raise ValueError(
            f"Explicit {name} limits and RobotModel limits have "
            "incompatible shapes; provide only one source "
            "or matching limits."
        )

    if not np.allclose(
        explicit_values,
        model_values,
        rtol=0.0,
        atol=1e-12
    ):
        raise ValueError(
            f"Explicit {name} limits conflict with RobotModel limits; "
            "provide only one source or matching limits."
        )

    return explicit_values
