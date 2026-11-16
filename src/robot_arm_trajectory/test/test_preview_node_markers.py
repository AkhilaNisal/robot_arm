"""Tests for preview marker publication and independent timer behavior."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from robot_arm_trajectory.preview_utils import validate_positive_rate
from robot_arm_trajectory.trajectory_preview_node import TrajectoryPreviewNode
from visualization_msgs.msg import Marker


def make_node_shell():
    """Create a lightweight node-shaped object for callback unit tests."""
    node = TrajectoryPreviewNode.__new__(TrajectoryPreviewNode)
    node.preview_data = SimpleNamespace(
        validation=SimpleNamespace(success=True, warnings=[]),
        reference_points=[(0.0, 0.1, 0.2), (1.0, 1.1, 1.2)],
        actual_points=[(0.0, -0.1, 0.2), (1.0, 0.9, 1.2)],
    )
    node.reference_publisher = Mock()
    node.actual_publisher = Mock()
    node.get_clock = Mock(return_value=SimpleNamespace(
        now=Mock(return_value=SimpleNamespace(to_msg=lambda: 'stamp'))
    ))
    return node


def test_markers_publish_reference_and_actual_geometry():
    """Both marker messages preserve topic-specific style and path points."""
    node = make_node_shell()

    node._publish_markers()

    reference = node.reference_publisher.publish.call_args.args[0]
    actual = node.actual_publisher.publish.call_args.args[0]
    assert (reference.header.frame_id, reference.ns, reference.id) == (
        'base_link', 'cartesian_reference', 0
    )
    assert (actual.header.frame_id, actual.ns, actual.id) == (
        'base_link', 'cartesian_actual', 1
    )
    assert reference.type == actual.type == Marker.LINE_STRIP
    assert reference.header.stamp == actual.header.stamp == 'stamp'
    assert reference.scale.x > 0 and actual.scale.x > 0
    assert reference.lifetime.sec == actual.lifetime.sec == 0
    assert (reference.color.r, reference.color.g, reference.color.b) == (
        0.1, 0.35, 1.0
    )
    assert (actual.color.r, actual.color.g, actual.color.b) == (
        0.1, 0.9, 0.2
    )
    assert [(p.x, p.y, p.z) for p in reference.points] == (
        node.preview_data.reference_points
    )
    assert [(p.x, p.y, p.z) for p in actual.points] == (
        node.preview_data.actual_points
    )


def test_playback_completion_keeps_publishing_final_joint_state():
    """Completed playback holds the final state for fresh TF transforms."""
    node = make_node_shell()
    node.get_clock = Mock(return_value=SimpleNamespace(
        now=Mock(return_value=FakeTime(0))
    ))
    node.playback_start_ros = FakeTime(0)
    node.samples = [SimpleNamespace(relative_time=0.0, positions=[0.0], names=['j'])]
    node.index = 0
    node.looping = False
    node.state = node.READY
    playback_timer = Mock()
    node.playback_timer = playback_timer
    node.marker_timer = Mock()
    node.q_initial = [0.0]
    node._publish_joint_positions = Mock()
    node.get_logger = Mock(return_value=Mock())

    node._playback_tick()

    playback_timer.cancel.assert_not_called()
    node.marker_timer.cancel.assert_not_called()
    args = node._publish_joint_positions.call_args.args
    assert args[0] == [0.0]
    assert args[1].nanoseconds == 0
    assert args[2] == ['j']
    assert node.state == node.COMPLETED
    node.reference_publisher.publish.assert_not_called()
    node.actual_publisher.publish.assert_not_called()


def test_completed_playback_republishes_held_joint_state():
    """The final state continues to refresh after trajectory completion."""
    node = make_node_shell()
    node.state = node.COMPLETED
    node.get_clock = Mock(return_value=SimpleNamespace(
        now=Mock(return_value=FakeTime(100))
    ))
    node.samples = [SimpleNamespace(
        relative_time=1.0, positions=[0.5], names=['joint_1']
    )]
    node._publish_joint_positions = Mock()

    node._playback_tick()

    args = node._publish_joint_positions.call_args.args
    assert args[0] == [0.5]
    assert args[1].nanoseconds == 100
    assert args[2] == ['joint_1']


class FakeTime:
    """Minimal ROS time double supporting subtraction and duration addition."""

    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def __sub__(self, other):
        return SimpleNamespace(nanoseconds=self.nanoseconds - other.nanoseconds)

    def __add__(self, duration):
        return FakeTime(self.nanoseconds + int(duration.nanoseconds))

    def to_msg(self):
        return 'stamp'


@pytest.mark.parametrize('rate', [0.0, -1.0, float('nan'), float('inf')])
def test_marker_publish_rate_rejects_invalid_values(rate):
    """Node parameter validation requires a positive finite marker rate."""
    with pytest.raises(ValueError, match='positive and finite'):
        validate_positive_rate(rate, 'marker_publish_rate')
