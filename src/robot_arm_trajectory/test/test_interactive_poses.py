"""Tests for interactive Cartesian poses and start-pose planning."""

from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

from robot_arm_trajectory import interactive_poses
from robot_arm_trajectory.interactive_poses import (
    InteractivePoseControls,
    load_pose_config,
    load_preview_config,
    matrix_to_pose_message,
    plan_selected_poses,
    pose_message_to_matrix,
)
from robot_arm_trajectory.preview_utils import JointStateSample
from robot_arm_trajectory.trajectory_preview_node import TrajectoryPreviewNode
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import InteractiveMarkerFeedback


class FakeServer:
    """Capture interactive marker server calls without DDS."""

    def __init__(self, *args, **kwargs):
        self.markers = {}
        self.marker_contexts = {}
        self.pending_updates = {}
        self.applied = 0

    def insert(self, marker, *, feedback_callback):
        self.markers[marker.name] = (marker, feedback_callback)

    def applyChanges(self):
        self.applied += 1

    def setPose(self, name, pose):
        self.poses[name] = pose

    def clear(self):
        pass


class FakeMenu:
    """Provide stable IDs for context-menu callback tests."""

    def __init__(self):
        self.next_id = 1

    def insert(self, title, **kwargs):
        result = self.next_id
        self.next_id += 1
        return result

    def apply(self, server, marker_name):
        pass


@pytest.fixture
def marker_manager(monkeypatch):
    """Create two captured markers with callbacks and identity orientations."""
    monkeypatch.setattr(interactive_poses, 'InteractiveMarkerServer', FakeServer)
    monkeypatch.setattr(interactive_poses, 'MenuHandler', FakeMenu)
    node = SimpleNamespace()
    changed = []
    menus = []
    manager = InteractivePoseControls(
        node, np.eye(4), np.diag([1.0, -1.0, -1.0, 1.0]),
        lambda *args: changed.append(args), menus.append,
    )
    manager.server.poses = {}
    return manager, changed, menus


def test_start_and_goal_markers_have_base_frame_and_six_axis_controls(
    marker_manager,
):
    """Both initial markers are visible in base_link with 6-DOF controls."""
    manager, _, _ = marker_manager

    assert set(manager.server.markers) == {'start_pose', 'goal_pose'}
    for name, (marker, _) in manager.server.markers.items():
        assert marker.header.frame_id == 'base_link'
        assert marker.description == (
            'Start Pose' if name == 'start_pose' else 'Goal Pose'
        )
        modes = [control.interaction_mode for control in marker.controls]
        assert modes.count(InteractiveMarkerControl.MOVE_AXIS) == 3
        assert modes.count(InteractiveMarkerControl.ROTATE_AXIS) == 3
        orientation = marker.pose.orientation
        assert np.linalg.norm([
            orientation.x, orientation.y, orientation.z, orientation.w
        ]) == pytest.approx(1.0)


def test_translation_and_rotation_feedback_updates_normalized_pose(
    marker_manager,
):
    """Feedback stores the latest translation and normalized xyzw rotation."""
    manager, changed, _ = marker_manager
    marker, callback = manager.server.markers['start_pose']
    feedback = SimpleNamespace(
        event_type=InteractiveMarkerFeedback.POSE_UPDATE,
        POSE_UPDATE=InteractiveMarkerFeedback.POSE_UPDATE,
        marker_name='start_pose',
        pose=matrix_to_pose_message(np.eye(4)),
    )
    feedback.pose.position.x = 0.45
    feedback.pose.orientation.z = 0.8
    feedback.pose.orientation.w = 0.8

    callback(feedback)

    result = manager.get_pose_matrix('start_pose')
    assert result[:3, 3] == pytest.approx([0.45, 0.0, 0.0])
    q = manager.poses['start_pose'].orientation
    assert np.linalg.norm([q.x, q.y, q.z, q.w]) == pytest.approx(1.0)
    assert changed[-1][0] == 'start_pose'
    assert marker.name == 'start_pose'


def test_pose_conversion_rejects_zero_and_nonfinite_quaternions():
    """Invalid ROS pose quaternion data does not silently become a pose."""
    pose = matrix_to_pose_message(np.eye(4))
    pose.orientation.w = 0.0
    with pytest.raises(ValueError, match='nonzero'):
        pose_message_to_matrix(pose)
    pose.orientation.w = float('nan')
    with pytest.raises(ValueError, match='finite'):
        pose_message_to_matrix(pose)


def test_plan_solves_start_ik_then_calls_existing_pipeline(monkeypatch):
    """Start IK result seeds the existing Cartesian planning pipeline."""
    robot = SimpleNamespace(
        dof=2,
        check_joint_limits=lambda q: True,
    )
    solution = SimpleNamespace(success=True, joint_positions=[0.2, 0.3])
    solver = SimpleNamespace(solve=lambda pose, seed: solution)
    pipeline_result = SimpleNamespace(success=True)
    calls = []

    def fake_pipeline(**kwargs):
        calls.append(kwargs)
        return pipeline_result

    monkeypatch.setattr(
        interactive_poses, 'generate_cartesian_joint_trajectory', fake_pipeline
    )
    start = np.eye(4)
    goal = np.eye(4)
    result = plan_selected_poses(
        robot, solver, [0.0, 0.0], start, goal, {'num_waypoints': 3}
    )

    assert result.success
    assert np.allclose(result.initial_configuration, [0.2, 0.3])
    assert calls[0]['q_initial'] == pytest.approx([0.2, 0.3])
    assert np.array_equal(calls[0]['start_pose'], start)
    assert np.array_equal(calls[0]['goal_pose'], goal)


def test_start_ik_failure_prevents_pipeline(monkeypatch):
    """An unreachable selected start does not attempt goal planning."""
    robot = SimpleNamespace(dof=1, check_joint_limits=lambda q: True)
    solver = SimpleNamespace(solve=lambda pose, seed: SimpleNamespace(
        success=False, message='start unreachable',
        position_error=1.0, orientation_error=0.5, iterations=100,
    ))
    called = []
    monkeypatch.setattr(
        interactive_poses,
        'generate_cartesian_joint_trajectory',
        lambda **kwargs: called.append(kwargs),
    )

    result = plan_selected_poses(
        robot, solver, [0.0], np.eye(4), np.eye(4), {}
    )

    assert not result.success
    assert result.failed_stage == 'start_ik'
    assert 'unreachable' in result.message
    assert not called


def test_goal_pipeline_ik_failure_is_reported():
    """A failed existing goal pipeline is returned without a trajectory."""
    robot = SimpleNamespace(dof=1, check_joint_limits=lambda q: True)
    solver = SimpleNamespace(solve=lambda pose, seed: SimpleNamespace(
        success=True, joint_positions=[0.0]
    ))
    failure = SimpleNamespace(
        success=False, failed_stage='ik', message='goal unreachable'
    )
    original = interactive_poses.generate_cartesian_joint_trajectory
    interactive_poses.generate_cartesian_joint_trajectory = (
        lambda **kwargs: failure
    )
    try:
        result = plan_selected_poses(
            robot, solver, [0.0], np.eye(4), np.eye(4), {}
        )
    finally:
        interactive_poses.generate_cartesian_joint_trajectory = original

    assert not result.success
    assert result.failed_stage == 'ik'
    assert 'goal unreachable' in result.message


def test_marker_initialization_does_not_request_planning(marker_manager):
    """Creating the RViz controls waits for explicit menu selection."""
    manager, _, menu_actions = marker_manager
    assert manager.server.applied == 1
    assert menu_actions == []
    assert manager.menu_ids['plan'] != manager.menu_ids['reset_both']


def test_marker_manager_shutdown_clears_local_state_without_publish(
    marker_manager,
):
    """Teardown does not publish after the ROS context begins shutting down."""
    manager, _, _ = marker_manager
    applied = manager.server.applied

    manager.shutdown()

    assert manager.server.applied == applied
    assert not manager.server.marker_contexts
    assert not manager.server.pending_updates


def test_yaml_pose_configuration_has_complete_normalized_poses():
    """The bundled single-file pose option supplies two base-frame poses."""
    path = (
        Path(__file__).parents[1]
        / 'robot_arm_trajectory' / 'config' / 'preview_poses.yaml'
    )
    start, goal = load_pose_config(path)
    _, _, trajectory = load_preview_config(path)

    assert start.shape == goal.shape == (4, 4)
    assert np.isfinite(start).all() and np.isfinite(goal).all()
    for transform in (start, goal):
        assert np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0])
        assert np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3))
    assert trajectory['interpolation_method'] == 'cubic_spline'
    assert trajectory['time_parameterization_method'] == 'quintic_scaling'
    assert trajectory['num_samples'] >= trajectory['num_cartesian_waypoints']
    assert trajectory['interpolation_options']['bc_type'] == 'not-a-knot'
    assert trajectory['ik_solver'] == 'damped_least_squares'
    assert trajectory['max_joint_waypoint_step'] == pytest.approx(0.5)
    assert trajectory['cartesian_position_tolerance'] == pytest.approx(0.002)


def test_yaml_mode_pose_markers_are_visible_but_not_editable(marker_manager):
    """YAML input shows poses while omitting drag and rotation controls."""
    manager, _, _ = marker_manager
    readonly = InteractivePoseControls(
        manager.node, np.eye(4), np.eye(4), lambda *args: None,
        lambda action: None, editable=False,
    )

    marker, _ = readonly.server.markers['start_pose']
    assert len(marker.controls) == 2
    assert marker.controls[0].always_visible
    assert marker.controls[1].interaction_mode == InteractiveMarkerControl.MENU


def _planning_node_shell():
    """Create a minimal node instance for deterministic planning lifecycle tests."""
    node = TrajectoryPreviewNode.__new__(TrajectoryPreviewNode)
    node._planning = False
    node.state = node.WAITING_FOR_POSES
    node.preview_data = None
    node.playback_timer = None
    node.marker_timer = None
    node.looping = False
    node.preview_rate = 30.0
    node.marker_publish_rate = 1.0
    node.startup_delay = 0.0
    node.robot = SimpleNamespace()
    node.solver = object()
    node.time_parameterization_options = {}
    node.q_initial = np.array([0.0])
    node.pose_controls = SimpleNamespace(
        START='start_pose', GOAL='goal_pose',
        get_pose_matrix=lambda name: np.eye(4),
    )
    node.get_parameter = lambda name: SimpleNamespace(value={
        'time_parameterization_method': 'quintic_scaling',
        'cartesian_position_tolerance': 0.002,
        'cartesian_orientation_tolerance': 0.02,
    }[name])
    node._pipeline_options = lambda: {}
    node.get_clock = lambda: SimpleNamespace(now=lambda: FakeNodeTime())
    node.create_timer = Mock(side_effect=lambda *args: SimpleNamespace(
        cancel=Mock()
    ))
    node._publish_joint_positions = Mock()
    node._publish_markers = Mock()
    node._delete_path_markers = Mock()
    node.get_logger = Mock(return_value=SimpleNamespace(
        warning=Mock(), info=Mock(), error=Mock()
    ))
    return node


class FakeNodeTime:
    """ROS-time value used to initialize planning timers."""

    def to_msg(self):
        return 'stamp'


def _successful_plan_data(monkeypatch, validation_success=True):
    """Stub successful IK/pipeline and controllable validation outcome."""
    planned = SimpleNamespace(
        success=True,
        initial_configuration=np.array([0.1]),
        trajectory_result=SimpleNamespace(
            joint_trajectory=object(),
            cartesian_path=[np.eye(4), np.eye(4)],
        ),
    )
    preview = SimpleNamespace(
        validation=SimpleNamespace(
            success=validation_success,
            errors=['test validation failure'],
            warnings=[],
        ),
        joint_samples=[JointStateSample(0.0, ['joint'], [0.1])],
        reference_points=[(0.0, 0.0, 0.0)],
        actual_points=[(0.0, 0.0, 0.0)],
    )
    monkeypatch.setattr(
        'robot_arm_trajectory.trajectory_preview_node.plan_selected_poses',
        lambda *args: planned,
    )
    monkeypatch.setattr(
        'robot_arm_trajectory.trajectory_preview_node.prepare_preview_data',
        lambda *args, **kwargs: preview,
    )


def test_explicit_plan_starts_playback_and_marker_timers(monkeypatch):
    """A menu planning request creates both timers only after validation."""
    _successful_plan_data(monkeypatch)
    node = _planning_node_shell()

    node._plan_and_preview()

    assert node.state == node.READY
    assert node.create_timer.call_count == 2
    assert node._publish_joint_positions.call_count == 1
    assert node._publish_markers.call_count == 1


def test_failed_validation_does_not_start_either_timer(monkeypatch):
    """Rejected preview validation prevents path and joint playback timers."""
    _successful_plan_data(monkeypatch, validation_success=False)
    node = _planning_node_shell()

    node._plan_and_preview()

    assert node.state == node.FAILED
    node.create_timer.assert_not_called()
    node._publish_joint_positions.assert_not_called()
    node._publish_markers.assert_not_called()


def test_plan_service_is_an_explicit_trigger(monkeypatch):
    """The fallback service invokes the same explicit plan workflow."""
    _successful_plan_data(monkeypatch)
    node = _planning_node_shell()
    response = SimpleNamespace(success=False, message='')

    returned = node._plan_service(None, response)

    assert returned is response
    assert response.success
    assert 'validated' in response.message
    assert node.create_timer.call_count == 2


def test_replanning_cancels_previous_timers_and_ignores_overlap(monkeypatch):
    """New plans replace active timers, while concurrent requests are ignored."""
    _successful_plan_data(monkeypatch)
    node = _planning_node_shell()
    previous_playback = SimpleNamespace(cancel=Mock())
    previous_markers = SimpleNamespace(cancel=Mock())
    node.playback_timer = previous_playback
    node.marker_timer = previous_markers
    node.preview_data = object()

    node._plan_and_preview()

    previous_playback.cancel.assert_called_once_with()
    previous_markers.cancel.assert_called_once_with()
    assert node.create_timer.call_count == 2
    node._planning = True
    node._plan_and_preview()
    assert node.create_timer.call_count == 2


def test_pose_adjustment_invalidates_previous_plan():
    """Changing either marker cancels old playback and path refresh timers."""
    node = _planning_node_shell()
    node.state = node.COMPLETED
    node.preview_data = object()
    playback = SimpleNamespace(cancel=Mock())
    markers = SimpleNamespace(cancel=Mock())
    node.playback_timer = playback
    node.marker_timer = markers

    node._pose_changed('goal_pose', None)

    assert node.state == node.WAITING_FOR_POSES
    assert node.preview_data is None
    playback.cancel.assert_called_once_with()
    markers.cancel.assert_called_once_with()
    node._delete_path_markers.assert_called_once_with()
