"""Read-only numerical, constraint, Cartesian, and singularity validation."""

from dataclasses import dataclass, field

import numpy as np

from robot_arm_kinematics.forward_kinematics import forward_kinematics
from robot_arm_kinematics.jacobian import calculate_jacobian

from .cartesian_space.cartesian_path import (
    quaternion_slerp,
    quaternion_to_rotation,
    rotation_to_quaternion,
    validate_pose,
)
from .time_parameterization import TIME_PARAMETERIZERS


@dataclass
class ValidationCheck:
    """One measurable validation result."""

    name: str
    passed: bool
    measured: object = None
    limit: object = None
    message: str = ''
    severity: str = 'error'


@dataclass
class TrajectoryValidationResult:
    """Combined result with detailed checks and an aggregate summary."""

    success: bool
    checks: list[ValidationCheck] = field(default_factory=list)
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)
    summary: dict = field(default_factory=dict)


def validate_trajectory(
    trajectory,
    robot_model,
    cartesian_path=None,
    acceleration_limits=None,
    jerk_limits=None,
    position_tolerance=1e-9,
    velocity_tolerance=1e-9,
    acceleration_tolerance=1e-9,
    jerk_tolerance=1e-9,
    cartesian_position_tolerance=None,
    cartesian_orientation_tolerance=None,
    singularity_threshold=1e-6,
    time_parameterization='quintic_scaling',
    time_parameterization_options=None,
):
    """
    Validate trajectory arrays, joint limits, Cartesian tracking and rank.

    Cartesian references are interpreted as uniformly spaced path waypoints.
    Their positions are interpolated linearly and their orientations by
    quaternion SLERP. ``time_parameterization`` maps trajectory time to path
    progress; it defaults to the trajectory pipeline's quintic scaling.
    This function does not modify the trajectory or robot model.
    """
    checks = []
    errors = []
    warnings = []
    summary = {}

    def record(
        name, passed, measured=None, limit=None, message='', severity='error'
    ):
        check = ValidationCheck(
            name, bool(passed), measured, limit, message, severity
        )
        checks.append(check)
        if not passed:
            target = warnings if severity == 'warning' else errors
            target.append(message or name)

    joint_tolerances = (
        position_tolerance,
        velocity_tolerance,
        acceleration_tolerance,
        jerk_tolerance,
    )
    cartesian_tolerances = (
        cartesian_position_tolerance,
        cartesian_orientation_tolerance,
    )
    tolerances_valid = all(
        _valid_tolerance(value) for value in joint_tolerances
    ) and all(
        value is None or _valid_tolerance(value)
        for value in cartesian_tolerances
    )
    record(
        'tolerances',
        tolerances_valid,
        message=(
            'Validation tolerances are finite and nonnegative.'
            if tolerances_valid else
            'Tolerances must be finite, nonnegative scalar values.'
        ),
    )
    if not tolerances_valid:
        return _result(checks, errors, warnings, summary)

    arrays, conversion_errors = _trajectory_arrays(trajectory)
    for message in conversion_errors:
        record('trajectory_arrays', False, message=message)
    if conversion_errors:
        return _result(checks, errors, warnings, summary)

    time = arrays['time']
    positions = arrays['positions']
    velocities = arrays['velocities']
    accelerations = arrays['accelerations']
    jerks = arrays['jerks']
    dof = getattr(robot_model, 'dof', None)

    time_finite = bool(np.all(np.isfinite(time)))
    record(
        'time_finite', time_finite,
        message=(
            'Time values must be finite.' if not time_finite
            else 'Time values are finite.'
        ),
    )
    time_increasing = bool(
        time.ndim == 1 and len(time) >= 2 and np.all(np.diff(time) > 0)
    )
    record(
        'time_strictly_increasing', time_increasing,
        message=(
            'Time must contain at least two strictly increasing values.'
            if not time_increasing else 'Time is strictly increasing.'
        ),
    )

    sample_count = len(time) if time.ndim == 1 else -1
    shape_valid = (
        positions.ndim == 2
        and velocities.shape == positions.shape
        and accelerations.shape == positions.shape
        and positions.shape[0] == sample_count
        and (jerks is None or jerks.shape == positions.shape)
        and sample_count >= 2
    )
    record(
        'array_dimensions',
        shape_valid,
        measured={
            'time': time.shape,
            'positions': positions.shape,
            'velocities': velocities.shape,
            'accelerations': accelerations.shape,
            'jerks': None if jerks is None else jerks.shape,
        },
        message=(
            'Trajectory arrays have compatible dimensions.'
            if shape_valid else
            'Time and trajectory arrays have incompatible dimensions.'
        ),
    )

    joint_count_valid = (
        positions.ndim == 2
        and dof is not None
        and positions.shape[1] == dof
    )
    record(
        'joint_count',
        joint_count_valid,
        measured=None if positions.ndim != 2 else positions.shape[1],
        limit=dof,
        message=(
            'Trajectory joint count matches the robot.'
            if joint_count_valid else
            'Trajectory joint count does not match the robot model.'
        ),
    )

    finite_valid = True
    for name in ('positions', 'velocities', 'accelerations'):
        passed = bool(np.all(np.isfinite(arrays[name])))
        finite_valid &= passed
        record(
            f'{name}_finite', passed,
            message=(
                f'{name.capitalize()} are finite.' if passed else
                f'{name.capitalize()} contain NaN or infinite values.'
            ),
        )
    if jerks is not None:
        passed = bool(np.all(np.isfinite(jerks)))
        finite_valid &= passed
        record(
            'jerks_finite', passed,
            message=(
                'Jerks are finite.' if passed else
                'Jerks contain NaN or infinite values.'
            ),
        )
    else:
        record(
            'jerks_available', True,
            message='Jerk data is unavailable; jerk checks were skipped.',
            severity='info',
        )

    basic_valid = (
        shape_valid and joint_count_valid and time_finite and time_increasing
        and finite_valid
    )
    if not basic_valid:
        return _result(checks, errors, warnings, summary)

    summary['minimum_position_per_joint'] = np.min(positions, axis=0).tolist()
    summary['maximum_position_per_joint'] = np.max(positions, axis=0).tolist()
    summary['maximum_absolute_velocity_per_joint'] = np.max(
        np.abs(velocities), axis=0
    ).tolist()
    summary['maximum_absolute_acceleration_per_joint'] = np.max(
        np.abs(accelerations), axis=0
    ).tolist()
    summary['maximum_absolute_jerk_per_joint'] = (
        None if jerks is None else np.max(np.abs(jerks), axis=0).tolist()
    )

    _validate_joint_constraints(
        record,
        warnings,
        robot_model,
        positions,
        velocities,
        accelerations,
        jerks,
        acceleration_limits,
        jerk_limits,
        position_tolerance,
        velocity_tolerance,
        acceleration_tolerance,
        jerk_tolerance,
    )

    if cartesian_path is not None:
        _validate_cartesian_tracking(
            record,
            summary,
            cartesian_path,
            time,
            positions,
            robot_model,
            cartesian_position_tolerance,
            cartesian_orientation_tolerance,
            time_parameterization,
            time_parameterization_options,
        )

    _validate_singularity(
        record, summary, robot_model, positions, singularity_threshold
    )
    return _result(checks, errors, warnings, summary)


def _trajectory_arrays(trajectory):
    """Return non-mutating floating views of all provided trajectory arrays."""
    arrays = {}
    issues = []
    for name in ('time', 'positions', 'velocities', 'accelerations'):
        try:
            arrays[name] = np.asarray(getattr(trajectory, name), dtype=float)
        except (AttributeError, TypeError, ValueError) as error:
            issues.append(
                f'Trajectory {name} data is missing or nonnumeric: {error}'
            )
    try:
        raw_jerks = getattr(trajectory, 'jerks', None)
        arrays['jerks'] = (
            None if raw_jerks is None else np.asarray(raw_jerks, dtype=float)
        )
    except (TypeError, ValueError) as error:
        issues.append(f'Trajectory jerk data is nonnumeric: {error}')
    return arrays, issues


def _validate_joint_constraints(
    record, warnings, robot, q, qd, qdd, qddd, acceleration_limits,
    jerk_limits, q_tol, qd_tol, qdd_tol, qddd_tol,
):
    """Measure extrema and compare available joint limits."""
    lower = getattr(robot, 'lower_limits', None)
    upper = getattr(robot, 'upper_limits', None)
    if lower is not None and upper is not None:
        lower = np.asarray(lower, dtype=float)
        upper = np.asarray(upper, dtype=float)
        if lower.shape == upper.shape == (q.shape[1],):
            passed = bool(
                np.all(np.min(q, axis=0) >= lower - q_tol)
                and np.all(np.max(q, axis=0) <= upper + q_tol)
            )
            record(
                'joint_position_limits', passed,
                measured={
                    'minimum': np.min(q, axis=0).tolist(),
                    'maximum': np.max(q, axis=0).tolist(),
                },
                limit={'lower': lower.tolist(), 'upper': upper.tolist()},
                message=(
                    'Joint positions satisfy position limits.' if passed else
                    'Joint positions violate position limits.'
                ),
            )
        else:
            warnings.append(
                'Robot position limits are unavailable or have invalid '
                'dimensions.'
            )
    else:
        warnings.append(
            'Robot position limits are unavailable; position constraints '
            'were skipped.'
        )

    model_velocity = getattr(robot, 'velocity_limits', None)
    _check_magnitude_constraint(
        record, warnings, 'joint_velocity_limits', qd, model_velocity,
        qd_tol,
        'Robot velocity limits are unavailable; velocity constraints '
        'were skipped.',
    )
    _check_magnitude_constraint(
        record, warnings, 'joint_acceleration_limits', qdd,
        acceleration_limits, qdd_tol,
        'Acceleration limits were not supplied; acceleration constraints '
        'were skipped.',
    )
    if qddd is None:
        record(
            'joint_jerk_limits', True,
            message='Jerk data is unavailable; jerk constraints were skipped.',
            severity='info',
        )
    elif jerk_limits is None:
        warnings.append(
            'Jerk limits were not supplied; jerk constraints were skipped.'
        )
    else:
        _check_magnitude_constraint(
            record, warnings, 'joint_jerk_limits', qddd, jerk_limits,
            qddd_tol, 'Jerk limits were not supplied.',
        )


def _check_magnitude_constraint(
    record, warnings, name, values, limits, tolerance, unavailable_message
):
    """Compare per-joint absolute maxima with scalar or vector bounds."""
    if limits is None:
        warnings.append(unavailable_message)
        return
    try:
        bound = np.asarray(limits, dtype=float)
        if bound.ndim == 0:
            bound = np.full(values.shape[1], float(bound))
    except (TypeError, ValueError):
        record(
            name, False, measured=None, limit=limits,
            message=f'{name} must be positive finite limits per joint.',
        )
        return
    valid = (
        bound.shape == (values.shape[1],)
        and np.all(np.isfinite(bound))
        and np.all(bound > 0)
    )
    if not valid:
        record(
            name, False, measured=None, limit=bound.tolist(),
            message=f'{name} must be positive finite limits per joint.',
        )
        return
    maxima = np.max(np.abs(values), axis=0)
    passed = bool(np.all(maxima <= bound + tolerance))
    record(
        name, passed, measured=maxima.tolist(), limit=bound.tolist(),
        message=(
            f'{name} satisfied.' if passed else f'{name} violated.'
        ),
    )


def _validate_cartesian_tracking(
    record, summary, path, time, positions, robot, position_tolerance,
    orientation_tolerance, time_parameterization, time_options,
):
    """Compare FK with path poses interpolated at trajectory sample times."""
    try:
        poses = [validate_pose(pose).copy() for pose in path]
        if len(poses) < 2:
            raise ValueError(
                'Cartesian reference path needs at least two poses.'
            )
        if time_parameterization not in TIME_PARAMETERIZERS:
            raise ValueError(
                f'Unknown time parameterization: {time_parameterization}'
            )
        duration = float(time[-1] - time[0])
        if duration <= 0.0:
            raise ValueError('Trajectory duration must be positive.')
        profile = TIME_PARAMETERIZERS[time_parameterization]().generate(
            duration=duration,
            num_samples=max(1001, 10 * len(time)),
            **(time_options or {}),
        )
        local_time = time - time[0]
        progress = np.interp(local_time, profile.time, profile.s)
        reference = _interpolate_cartesian_path(poses, progress)

        position_errors = []
        orientation_errors = []
        for q, desired in zip(positions, reference):
            actual = forward_kinematics(robot, q)
            position_errors.append(
                float(np.linalg.norm(desired[:3, 3] - actual[:3, 3]))
            )
            orientation_errors.append(
                _rotation_angle(desired[:3, :3], actual[:3, :3])
            )
        position_errors = np.asarray(position_errors)
        orientation_errors = np.asarray(orientation_errors)
        summary['cartesian_max_position_error_m'] = float(
            np.max(position_errors)
        )
        summary['cartesian_rms_position_error_m'] = float(
            np.sqrt(np.mean(position_errors**2))
        )
        summary['cartesian_max_orientation_error_rad'] = float(
            np.max(orientation_errors)
        )
        summary['cartesian_rms_orientation_error_rad'] = float(
            np.sqrt(np.mean(orientation_errors**2))
        )
        passed = True
        if position_tolerance is not None:
            passed &= (
                summary['cartesian_max_position_error_m']
                <= position_tolerance
            )
        if orientation_tolerance is not None:
            passed &= (
                summary['cartesian_max_orientation_error_rad']
                <= orientation_tolerance
            )
        record(
            'cartesian_tracking', passed,
            measured={
                'maximum_position_error_m':
                    summary['cartesian_max_position_error_m'],
                'rms_position_error_m':
                    summary['cartesian_rms_position_error_m'],
                'maximum_orientation_error_rad':
                    summary['cartesian_max_orientation_error_rad'],
                'rms_orientation_error_rad':
                    summary['cartesian_rms_orientation_error_rad'],
            },
            limit={
                'position_m': position_tolerance,
                'orientation_rad': orientation_tolerance,
            },
            message=(
                'Cartesian tracking errors measured.' if passed else
                'Cartesian tracking tolerance exceeded.'
            ),
        )
    except (ValueError, TypeError, np.linalg.LinAlgError) as error:
        record(
            'cartesian_tracking', False,
            message=f'Cartesian tracking could not be validated: {error}',
        )


def _interpolate_cartesian_path(poses, progress):
    """Interpolate poses with linear position and quaternion SLERP."""
    path_position = np.clip(progress, 0.0, 1.0) * (len(poses) - 1)
    segment = np.minimum(np.floor(path_position).astype(int), len(poses) - 2)
    fraction = path_position - segment
    result = []
    quaternions = [rotation_to_quaternion(pose[:3, :3]) for pose in poses]
    for index, u in zip(segment, fraction):
        start = poses[index]
        end = poses[index + 1]
        pose = np.eye(4)
        pose[:3, 3] = start[:3, 3] + u * (end[:3, 3] - start[:3, 3])
        quaternion = quaternion_slerp(
            quaternions[index], quaternions[index + 1], float(u)
        )
        pose[:3, :3] = quaternion_to_rotation(quaternion)
        result.append(pose)
    return result


def _rotation_angle(desired, actual):
    """Return geodesic rotation distance in radians."""
    relative = desired @ actual.T
    cosine = np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0)
    return float(np.arccos(cosine))


def _valid_tolerance(value):
    """Return whether a tolerance is a finite nonnegative scalar."""
    if (
        not np.isscalar(value)
        or isinstance(value, (bool, np.bool_))
        or not isinstance(value, (int, float, np.integer, np.floating))
    ):
        return False
    return bool(np.isfinite(value) and value >= 0)


def _validate_singularity(record, summary, robot, positions, threshold):
    """Measure Jacobian singular values and warn near configured rank loss."""
    if (
        not np.isscalar(threshold)
        or isinstance(threshold, (bool, np.bool_))
        or not isinstance(threshold, (int, float, np.integer, np.floating))
        or not np.isfinite(threshold)
        or threshold <= 0
    ):
        record(
            'jacobian_singularity', False, measured=threshold,
            message='Singularity threshold must be positive and finite.',
        )
        return
    minimum_singular_values = []
    condition_numbers = []
    failed_samples = []
    for index, q in enumerate(positions):
        try:
            jacobian, _ = calculate_jacobian(robot, q)
            singular_values = np.linalg.svd(jacobian, compute_uv=False)
            largest = (
                float(singular_values[0]) if len(singular_values) else 0.0
            )
            smallest = (
                float(singular_values[-1]) if len(singular_values) else 0.0
            )
            minimum_singular_values.append(smallest)
            if smallest <= np.finfo(float).eps * max(largest, 1.0):
                condition_numbers.append(float('inf'))
            else:
                condition_numbers.append(largest / smallest)
        except (ValueError, np.linalg.LinAlgError, FloatingPointError):
            failed_samples.append(index)
    if failed_samples:
        message = (
            'Jacobian singularity analysis failed at samples '
            f'{failed_samples}.'
        )
        record(
            'jacobian_singularity', False,
            measured={'failed_samples': failed_samples},
            limit=threshold,
            message=message,
            severity='warning',
        )
        return
    minimum = float(np.min(minimum_singular_values))
    condition = float(np.max(condition_numbers))
    near_singular = minimum <= threshold
    summary['minimum_jacobian_singular_value'] = minimum
    summary['maximum_jacobian_condition_number'] = condition
    summary['near_singular_sample_count'] = int(
        np.count_nonzero(np.asarray(minimum_singular_values) <= threshold)
    )
    message = (
        f'Jacobian approaches singularity: minimum singular value '
        f'{minimum:.6g} <= threshold {threshold:.6g}.'
        if near_singular else
        'No Jacobian singular values crossed the threshold.'
    )
    record(
        'jacobian_singularity', not near_singular,
        measured={
            'minimum_singular_value': minimum,
            'maximum_condition_number': condition,
            'near_singular_sample_count':
                summary['near_singular_sample_count'],
        },
        limit=threshold,
        message=message,
        severity='warning',
    )


def _result(checks, errors, warnings, summary):
    """Build the overall result; warnings do not make validation fail."""
    success = not errors
    return TrajectoryValidationResult(
        success=success,
        checks=checks,
        errors=errors,
        warnings=warnings,
        summary=summary,
    )
