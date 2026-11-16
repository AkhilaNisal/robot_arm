"""Tests for ROS-independent trajectory preview conversions."""

from pathlib import Path

import numpy as np

import pytest

from robot_arm_kinematics.forward_kinematics import forward_kinematics
from robot_arm_kinematics.robot_model import RobotModel

from robot_arm_trajectory.cartesian_path import generate_linear_cartesian_path
from robot_arm_trajectory.joint_trajectory import generate_joint_trajectory
from robot_arm_trajectory.preview_utils import (
    build_playback_samples,
    due_sample_indices,
    latest_due_sample_index,
    parse_parameter_options,
    prepare_preview_data,
)


@pytest.fixture
def robot():
    """Load the generic robot configuration."""
    config = (
        Path(__file__).parents[2]
        / 'robot_arm_kinematics'
        / 'robot_arm_kinematics'
        / 'config'
        / 'generic_robot.yaml'
    )
    return RobotModel(str(config))


@pytest.fixture
def preview_data_inputs(robot):
    """Build a short, valid trajectory and its corresponding path."""
    q0 = np.deg2rad([20, 35, 20, 10, 25, 10])
    q1 = np.deg2rad([23, 38, 23, 13, 28, 13])
    trajectory = generate_joint_trajectory(
        [q0, q1], duration=2.0, num_samples=9,
        interpolation_method='quintic_polynomial',
    )
    path = generate_linear_cartesian_path(
        forward_kinematics(robot, q0),
        forward_kinematics(robot, q1),
        num_waypoints=5,
    )
    return trajectory, path


def test_joint_state_samples_map_names_positions_and_times(
    robot, preview_data_inputs
):
    """Prepared samples preserve joint ordering and relative timestamps."""
    trajectory, path = preview_data_inputs
    data = prepare_preview_data(trajectory, robot, path)

    assert data.validation.success
    assert len(data.joint_samples) == len(trajectory.time)
    for index, sample in enumerate(data.joint_samples):
        assert sample.names == robot.joint_names
        assert np.allclose(sample.positions, trajectory.positions[index])
        assert sample.relative_time == pytest.approx(
            trajectory.time[index] - trajectory.time[0]
        )


def test_due_sample_selection_respects_trajectory_timestamps(
    robot, preview_data_inputs
):
    """Playback selection does not advance a sample before its timestamp."""
    trajectory, path = preview_data_inputs
    data = prepare_preview_data(trajectory, robot, path)
    due = due_sample_indices(data.joint_samples, elapsed=0.6, next_index=0)

    assert due == [0, 1, 2]
    assert data.joint_samples[due[-1]].relative_time <= 0.6
    assert data.joint_samples[due[-1] + 1].relative_time > 0.6


def test_reference_and_fk_actual_marker_points(robot, preview_data_inputs):
    """Path points and actual FK positions match their source geometry."""
    trajectory, path = preview_data_inputs
    data = prepare_preview_data(trajectory, robot, path)

    assert len(data.reference_points) == len(path)
    assert np.allclose(data.reference_points[0], path[0][:3, 3])
    assert len(data.actual_points) == len(trajectory.positions)
    for point, q in zip(data.actual_points, trajectory.positions):
        assert np.allclose(
            point,
            forward_kinematics(robot, q)[:3, 3],
        )


def test_invalid_trajectory_is_rejected_before_preview_data(
    robot, preview_data_inputs
):
    """Failed validation yields no samples or marker points to publish."""
    trajectory, path = preview_data_inputs
    trajectory.positions[0, 0] = robot.upper_limits[0] + 0.1
    data = prepare_preview_data(trajectory, robot, path)

    assert not data.validation.success
    assert not data.joint_samples
    assert not data.reference_points
    assert not data.actual_points


def test_due_samples_reject_invalid_elapsed_time(robot, preview_data_inputs):
    """Invalid elapsed times cannot advance preview playback."""
    trajectory, path = preview_data_inputs
    data = prepare_preview_data(trajectory, robot, path)
    with pytest.raises(ValueError, match='Elapsed preview time'):
        due_sample_indices(data.joint_samples, elapsed=np.nan)


def test_preview_rate_selects_only_the_latest_due_configuration(
    robot, preview_data_inputs
):
    """A slow timer publishes one current state instead of a burst."""
    trajectory, path = preview_data_inputs
    data = prepare_preview_data(trajectory, robot, path)

    assert latest_due_sample_index(data.joint_samples, 0.6, 0) == 2
    assert latest_due_sample_index(data.joint_samples, 0.1, 3) is None


def test_loop_samples_return_to_start_without_a_position_jump(
    robot, preview_data_inputs
):
    """Looping adds a reverse leg so the next cycle starts continuously."""
    trajectory, path = preview_data_inputs
    data = prepare_preview_data(trajectory, robot, path)
    samples = build_playback_samples(data.joint_samples, looping=True)
    duration = data.joint_samples[-1].relative_time

    assert len(samples) == 2 * len(data.joint_samples) - 1
    assert samples[-1].positions == data.joint_samples[0].positions
    assert samples[-1].relative_time == pytest.approx(2 * duration)
    assert np.allclose(samples[-2].positions, data.joint_samples[1].positions)
    assert all(
        right.relative_time > left.relative_time
        for left, right in zip(samples[:-1], samples[1:])
    )


@pytest.mark.parametrize(
    'value, expected',
    [('{"max_jerk": 0.5}', {'max_jerk': 0.5}), ('{}', {})],
)
def test_time_parameter_options_parse_from_ros_string(value, expected):
    """ROS string parameters decode to profile option mappings."""
    assert parse_parameter_options(value) == expected


@pytest.mark.parametrize('value', ['not-json', '[]', 'null', '1'])
def test_time_parameter_options_reject_non_object_values(value):
    """Malformed timing option parameters fail with a clear ValueError."""
    with pytest.raises(ValueError, match='JSON object'):
        parse_parameter_options(value)
