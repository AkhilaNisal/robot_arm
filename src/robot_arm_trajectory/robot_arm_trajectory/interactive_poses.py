"""Interactive RViz pose controls and selected-pose planning helpers."""

from dataclasses import dataclass

from geometry_msgs.msg import Pose
from interactive_markers import InteractiveMarkerServer, MenuHandler
from visualization_msgs.msg import InteractiveMarker
from visualization_msgs.msg import InteractiveMarkerControl
from visualization_msgs.msg import Marker

from .cartesian_path import quaternion_to_rotation
from .pipeline import generate_cartesian_joint_trajectory
from .cartesian_ik import solve_ik_with_restarts


def load_preview_config(path):
    """Load Cartesian poses and optional planner defaults from YAML."""
    from pathlib import Path

    import numpy as np
    import yaml

    try:
        data = yaml.safe_load(Path(path).read_text())
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f'Could not read pose configuration {path}: {error}') from error
    if not isinstance(data, dict) or not isinstance(data.get('poses'), dict):
        raise ValueError('Pose YAML must contain a top-level poses mapping.')

    transforms = {}
    for name in ('start', 'goal'):
        pose_data = data['poses'].get(name)
        if not isinstance(pose_data, dict):
            raise ValueError(f'Pose YAML is missing poses.{name}.')
        try:
            position = np.asarray(pose_data.get('position'), dtype=float)
            quaternion = np.asarray(
                pose_data.get('orientation_xyzw'), dtype=float
            )
        except (TypeError, ValueError) as error:
            raise ValueError(
                f'poses.{name} position and quaternion must be numeric.'
            ) from error
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError(
                f'poses.{name}.position must contain three finite metres.'
            )
        if (
            quaternion.shape != (4,)
            or not np.all(np.isfinite(quaternion))
            or np.linalg.norm(quaternion) <= np.finfo(float).eps
        ):
            raise ValueError(
                f'poses.{name}.orientation_xyzw must be a finite nonzero '
                'quaternion.'
            )
        quaternion /= np.linalg.norm(quaternion)
        transform = np.eye(4)
        transform[:3, :3] = quaternion_to_rotation(quaternion)
        transform[:3, 3] = position
        transforms[name] = transform
    trajectory = data.get('trajectory', {})
    if not isinstance(trajectory, dict):
        raise ValueError('Pose YAML trajectory section must be a mapping.')
    for name in ('interpolation_options', 'time_parameterization_options'):
        if name in trajectory and not isinstance(trajectory[name], dict):
            raise ValueError(f'trajectory.{name} must be a mapping.')
    return transforms['start'], transforms['goal'], trajectory


def load_pose_config(path):
    """Load start and goal transforms, ignoring optional planner defaults."""
    start_pose, goal_pose, _ = load_preview_config(path)
    return start_pose, goal_pose


def pose_message_to_matrix(pose):
    """Convert a ROS pose with xyzw quaternion into a validated transform."""
    import numpy as np

    quaternion = np.array([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ], dtype=float)
    position = np.array([
        pose.position.x,
        pose.position.y,
        pose.position.z,
    ], dtype=float)
    if not np.all(np.isfinite(position)):
        raise ValueError('Interactive pose position must be finite.')
    if not np.all(np.isfinite(quaternion)):
        raise ValueError('Interactive pose quaternion must be finite.')
    norm = float(np.linalg.norm(quaternion))
    if norm <= np.finfo(float).eps:
        raise ValueError('Interactive pose quaternion must be nonzero.')
    quaternion /= norm
    result = np.eye(4)
    result[:3, :3] = quaternion_to_rotation(quaternion)
    result[:3, 3] = position
    return result


def matrix_to_pose_message(matrix):
    """Convert a homogeneous transform to a normalized ROS pose message."""
    from .cartesian_path import rotation_to_quaternion, validate_pose

    import numpy as np

    transform = validate_pose(matrix)
    quaternion = rotation_to_quaternion(transform[:3, :3])
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(
        float, transform[:3, 3]
    )
    (pose.orientation.x, pose.orientation.y,
     pose.orientation.z, pose.orientation.w) = map(float, quaternion)
    if not np.isclose(np.linalg.norm(quaternion), 1.0):
        raise ValueError('Pose conversion produced a non-normalized quaternion.')
    return pose


@dataclass
class SelectedPosePlan:
    """Outcome of solving the selected start pose before pipeline planning."""

    success: bool
    initial_configuration: object | None
    trajectory_result: object | None
    failed_stage: str | None
    message: str


def plan_selected_poses(
    robot,
    solver,
    seed_configuration,
    start_pose,
    goal_pose,
    pipeline_options,
):
    """Solve start-pose IK, then use the existing Cartesian pipeline."""
    import numpy as np

    start_solution = solve_ik_with_restarts(
        solver, start_pose, seed_configuration
    )
    if not start_solution.success:
        details = (
            f'{start_solution.message} Position error: '
            f'{start_solution.position_error:.6g} m; orientation error: '
            f'{start_solution.orientation_error:.6g} rad; iterations: '
            f'{start_solution.iterations}.'
        )
        return SelectedPosePlan(
            False, None, None, 'start_ik', details
        )
    configuration = np.asarray(start_solution.joint_positions, dtype=float)
    if (
        configuration.shape != (robot.dof,)
        or not np.all(np.isfinite(configuration))
        or not robot.check_joint_limits(configuration)
    ):
        return SelectedPosePlan(
            False, None, None, 'start_ik',
            'Start-pose IK returned invalid or out-of-limit joints.',
        )
    result = generate_cartesian_joint_trajectory(
        robot_model=robot,
        ik_solver=solver,
        q_initial=configuration,
        start_pose=start_pose,
        goal_pose=goal_pose,
        **pipeline_options,
    )
    if not result.success:
        return SelectedPosePlan(
            False, configuration, result, result.failed_stage, result.message
        )
    return SelectedPosePlan(
        True, configuration, result, None,
        getattr(result, 'message', 'Trajectory planning succeeded.'),
    )


class InteractivePoseControls:
    """Own start/goal 6-DOF controls, menus, and feedback callbacks."""

    START = 'start_pose'
    GOAL = 'goal_pose'

    def __init__(
        self, node, start_pose, goal_pose, pose_callback, menu_callback,
        editable=True, gripper_callback=None,
    ):
        self.node = node
        self.server = InteractiveMarkerServer(
            node, '/trajectory_preview/poses'
        )
        self.pose_callback = pose_callback
        self.menu_callback = menu_callback
        self.gripper_callback = gripper_callback
        self.editable = editable
        self.poses = {
            self.START: matrix_to_pose_message(start_pose),
            self.GOAL: matrix_to_pose_message(goal_pose),
        }
        self.menu = MenuHandler()
        self.menu_ids = {
            'plan_linear': self.menu.insert(
                'Plan: Cartesian Line', callback=self._menu_feedback
            ),
            'plan_arc': self.menu.insert(
                'Plan: Raised Cartesian Arc', callback=self._menu_feedback
            ),
            'plan_joint': self.menu.insert(
                'Plan: Joint Space', callback=self._menu_feedback
            ),
            'reset_start': self.menu.insert(
                'Reset Start Pose', callback=self._menu_feedback
            ),
            'reset_goal': self.menu.insert(
                'Reset Goal Pose', callback=self._menu_feedback
            ),
            'reset_both': self.menu.insert(
                'Reset Both Poses', callback=self._menu_feedback
            ),
        }
        if self.gripper_callback is not None:
            self.menu_ids['gripper_open'] = self.menu.insert(
                'Open Gripper', callback=self._menu_feedback
            )
            self.menu_ids['gripper_close'] = self.menu.insert(
                'Close Gripper', callback=self._menu_feedback
            )
        self._insert_marker(self.START, 'Start Pose', (0.1, 0.5, 1.0, 1.0))
        self._insert_marker(self.GOAL, 'Goal Pose', (1.0, 0.35, 0.1, 1.0))
        self.server.applyChanges()

    def _insert_marker(self, name, description, color):
        """Create visible pose handle with translation and rotation axes."""
        marker = InteractiveMarker()
        marker.header.frame_id = 'base_link'
        marker.name = name
        marker.description = description
        marker.scale = 0.22
        marker.pose = self.poses[name]

        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual = Marker()
        visual.type = Marker.SPHERE
        visual.pose.orientation.w = 1.0
        visual.scale.x = visual.scale.y = visual.scale.z = 0.07
        visual.color.r, visual.color.g, visual.color.b, visual.color.a = color
        visual_control.markers.append(visual)
        marker.controls.append(visual_control)

        if self.editable:
            root_half = 2.0 ** -0.5
            for axis, quaternion in (
                ('x', (root_half, 0.0, 0.0, root_half)),
                ('y', (0.0, root_half, 0.0, root_half)),
                ('z', (0.0, 0.0, root_half, root_half)),
            ):
                for mode, suffix in (
                    (InteractiveMarkerControl.MOVE_AXIS, 'move'),
                    (InteractiveMarkerControl.ROTATE_AXIS, 'rotate'),
                ):
                    control = InteractiveMarkerControl()
                    control.name = f'{axis}_{suffix}'
                    control.orientation.x, control.orientation.y = quaternion[:2]
                    control.orientation.z, control.orientation.w = quaternion[2:]
                    control.interaction_mode = mode
                    marker.controls.append(control)
        else:
            menu_control = InteractiveMarkerControl()
            menu_control.name = 'pose_menu'
            menu_control.interaction_mode = InteractiveMarkerControl.MENU
            marker.controls.append(menu_control)

        self.server.insert(marker, feedback_callback=self._feedback)
        self.menu.apply(self.server, name)

    def _feedback(self, feedback):
        """Store normalized feedback and notify node of changed poses."""
        if feedback.event_type != feedback.POSE_UPDATE:
            return
        try:
            pose = matrix_to_pose_message(
                pose_message_to_matrix(feedback.pose)
            )
        except ValueError as error:
            self.node.get_logger().error(
                f'Ignoring invalid {feedback.marker_name} pose: {error}'
            )
            return
        self.poses[feedback.marker_name] = pose
        self.pose_callback(feedback.marker_name, pose)

    def _menu_feedback(self, feedback):
        """Dispatch selected menu command to the preview node."""
        command = next(
            name for name, entry_id in self.menu_ids.items()
            if entry_id == feedback.menu_entry_id
        )
        if command.startswith('gripper_'):
            self.gripper_callback(command.removeprefix('gripper_'))
        elif command.startswith('plan_'):
            self.menu_callback(command)
        else:
            self.menu_callback(command)

    def set_pose(self, marker_name, pose):
        """Set a marker pose from a ROS pose or transform matrix."""
        if not isinstance(pose, Pose):
            pose = matrix_to_pose_message(pose)
        pose_message_to_matrix(pose)
        self.poses[marker_name] = pose
        self.server.setPose(marker_name, pose)
        self.server.applyChanges()

    def get_pose_matrix(self, marker_name):
        """Return latest marker pose as a homogeneous transform."""
        return pose_message_to_matrix(self.poses[marker_name])

    def shutdown(self):
        """Clear local server state without publishing during node teardown."""
        self.server.marker_contexts.clear()
        self.server.pending_updates.clear()
