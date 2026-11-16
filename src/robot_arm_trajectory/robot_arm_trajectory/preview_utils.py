"""ROS-independent conversion helpers for trajectory preview data."""

from dataclasses import dataclass
import json

import numpy as np

from robot_arm_kinematics.forward_kinematics import forward_kinematics

from .validation import TrajectoryValidationResult, validate_trajectory


@dataclass
class JointStateSample:
    """Joint names, positions and trajectory-relative publication time."""

    relative_time: float
    names: list[str]
    positions: list[float]


@dataclass
class PreviewData:
    """Validated data prepared for a ROS preview node."""

    validation: TrajectoryValidationResult
    joint_samples: list[JointStateSample]
    reference_points: list[tuple[float, float, float]]
    actual_points: list[tuple[float, float, float]]


def prepare_preview_data(
    trajectory,
    robot_model,
    cartesian_path,
    time_parameterization='quintic_scaling',
    time_parameterization_options=None,
    acceleration_limits=None,
    jerk_limits=None,
    cartesian_position_tolerance=None,
    cartesian_orientation_tolerance=None,
    validate_cartesian_tracking=True,
):
    """Validate and convert trajectory and path data without ROS messages."""
    validation = validate_trajectory(
        trajectory,
        robot_model,
        cartesian_path=(
            cartesian_path if validate_cartesian_tracking else None
        ),
        time_parameterization=time_parameterization,
        time_parameterization_options=time_parameterization_options,
        acceleration_limits=acceleration_limits,
        jerk_limits=jerk_limits,
        cartesian_position_tolerance=cartesian_position_tolerance,
        cartesian_orientation_tolerance=cartesian_orientation_tolerance,
    )
    if not validation.success:
        return PreviewData(validation, [], [], [])

    names = list(robot_model.joint_names)
    samples = [
        JointStateSample(
            relative_time=float(time - trajectory.time[0]),
            names=names.copy(),
            positions=np.asarray(position, dtype=float).tolist(),
        )
        for time, position in zip(trajectory.time, trajectory.positions)
    ]
    reference_points = [
        tuple(float(value) for value in np.asarray(pose)[:3, 3])
        for pose in cartesian_path
    ]
    actual_points = [
        tuple(
            float(value)
            for value in forward_kinematics(robot_model, position)[:3, 3]
        )
        for position in trajectory.positions
    ]
    return PreviewData(validation, samples, reference_points, actual_points)


def due_sample_indices(samples, elapsed, next_index=0):
    """Return chronological samples whose trajectory times have elapsed."""
    if not np.isfinite(elapsed) or elapsed < 0:
        raise ValueError(
            'Elapsed preview time must be finite and nonnegative.'
        )
    if next_index < 0 or next_index > len(samples):
        raise ValueError(
            'Next sample index is outside the sample sequence.'
        )
    due = []
    index = next_index
    while index < len(samples) and samples[index].relative_time <= elapsed:
        due.append(index)
        index += 1
    return due


def latest_due_sample_index(samples, elapsed, next_index=0):
    """Return at most one sample index due at a preview tick."""
    due = due_sample_indices(samples, elapsed, next_index)
    return due[-1] if due else None


def build_playback_samples(samples, looping=False):
    """Extend a sample sequence with a continuous reverse leg for looping."""
    if not samples:
        return []
    if not looping:
        return list(samples)
    if len(samples) < 2 or samples[0].relative_time < 0:
        raise ValueError('Looping requires at least two timed samples.')

    duration = samples[-1].relative_time
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError('Looping requires a positive finite duration.')

    reverse_leg = [
        JointStateSample(
            relative_time=float(2.0 * duration - sample.relative_time),
            names=sample.names.copy(),
            positions=sample.positions.copy(),
        )
        for sample in reversed(samples[:-1])
    ]
    return list(samples) + reverse_leg


def parse_parameter_options(value):
    """Parse JSON text for ROS parameters that represent option mappings."""
    try:
        options = json.loads(value)
    except (TypeError, json.JSONDecodeError) as error:
        raise ValueError(
            'Parameter options must be a JSON object string.'
        ) from error
    if not isinstance(options, dict):
        raise ValueError('Parameter options must decode to a JSON object.')
    return options


def validate_positive_rate(value, name='rate'):
    """Return a finite positive rate or raise a descriptive ValueError."""
    rate = float(value)
    if not np.isfinite(rate) or rate <= 0:
        raise ValueError(f'{name} must be positive and finite.')
    return rate
