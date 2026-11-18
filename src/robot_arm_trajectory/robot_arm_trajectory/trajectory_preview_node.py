"""ROS 2 RViz preview for validated Cartesian-to-joint trajectories."""

import json
import math

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from robot_arm_kinematics.forward_kinematics import forward_kinematics
from robot_arm_kinematics.inverse_kinematics.damped_least_squares import (
    DampedLeastSquaresIK,
)
from robot_arm_kinematics.inverse_kinematics.jacobian_transpose import (
    JacobianTransposeIK,
)
from robot_arm_kinematics.inverse_kinematics.pseudoinverse import (
    PseudoinverseIK,
)
from robot_arm_kinematics.robot_model import RobotModel

from robot_arm_trajectory.cartesian_space.cartesian_path import (
    quaternion_to_rotation,
    rotation_to_quaternion,
)
from robot_arm_trajectory.interactive_poses import (
    InteractivePoseControls,
    load_preview_config,
    plan_selected_poses,
    pose_message_to_matrix,
)
from robot_arm_trajectory.preview_utils import (
    build_playback_samples,
    latest_due_sample_index,
    parse_parameter_options,
    prepare_preview_data,
    validate_positive_rate,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from interactive_markers import MenuHandler
from robot_arm_trajectory.joint_space.joint_trajectory import (
    generate_joint_trajectory,
)
from robot_arm_trajectory.cartesian_space.cartesian_ik import (
    solve_ik_with_restarts,
)


class TrajectoryPreviewNode(Node):
    """Select Cartesian poses in RViz and preview validated trajectories."""

    WAITING_FOR_POSES = 'WAITING_FOR_POSES'
    PLANNING = 'PLANNING'
    VALIDATING = 'VALIDATING'
    READY = 'READY'
    PLAYING = 'PLAYING'
    COMPLETED = 'COMPLETED'
    FAILED = 'FAILED'

    def __init__(self):
        """Initialize configuration and wait for explicit planning action."""
        super().__init__('robot_arm_trajectory_preview')
        self.preview_ready = True
        self.state = self.WAITING_FOR_POSES
        self._planning = False
        self.preview_data = None
        self.playback_timer = None
        self.marker_timer = None
        default_config = (
            get_package_share_directory('robot_arm_kinematics')
            + '/config/generic_robot.yaml'
        )
        self.declare_parameter('robot_config', default_config)
        default_pose_config = (
            get_package_share_directory('robot_arm_trajectory')
            + '/config/preview_poses.yaml'
        )
        self.declare_parameter('pose_input_mode', 'rviz')
        self.declare_parameter('pose_config', default_pose_config)
        self.pose_input_mode = str(
            self.get_parameter('pose_input_mode').value
        ).lower()
        if self.pose_input_mode not in ('rviz', 'yaml'):
            raise ValueError('pose_input_mode must be either rviz or yaml.')
        self.pose_configuration = None
        self.waypoint_configuration = None
        if self.pose_input_mode == 'yaml':
            self.waypoint_configuration = self._read_waypoint_config(
                self.get_parameter('pose_config').value
            )
            if self.waypoint_configuration is None:
                self.pose_configuration = load_preview_config(
                    self.get_parameter('pose_config').value
                )
        planner_config = (
            self.pose_configuration[2]
            if self.pose_configuration is not None
            else (
                self.waypoint_configuration.get('trajectory', {})
                if self.waypoint_configuration is not None else {}
            )
        )
        if not isinstance(planner_config, dict):
            raise ValueError('Preview YAML trajectory section must be a mapping.')
        self.robot = RobotModel(
            self.get_parameter('robot_config').get_parameter_value().string_value
        )
        if self.robot.name.lower() == 'gp7':
            q_default = np.zeros(6)
            goal_default_q = np.deg2rad([5, 0, 0, 0, 0, 0])
        else:
            q_default = np.deg2rad([20, 35, 20, 10, 25, 10])
            goal_default_q = np.deg2rad([30, 45, 30, 20, 40, 25])
        self.default_start_pose = forward_kinematics(self.robot, q_default)
        self.default_goal_pose = forward_kinematics(self.robot, goal_default_q)
        self.declare_parameter(
            'initial_joint_configuration', q_default.tolist()
        )
        self.declare_parameter(
            'goal_position', self.default_goal_pose[:3, 3].tolist()
        )
        self.declare_parameter(
            'goal_orientation',
            rotation_to_quaternion(self.default_goal_pose[:3, :3]).tolist(),
        )
        self.declare_parameter(
            'num_cartesian_waypoints',
            planner_config.get('num_cartesian_waypoints', 50),
        )
        self.declare_parameter(
            'path_method', planner_config.get('path_method', 'cartesian_linear')
        )
        self.declare_parameter(
            'cartesian_arc_height',
            planner_config.get('cartesian_arc_height', 0.1),
        )
        self.declare_parameter(
            'trajectory_duration', planner_config.get('duration', 4.0)
        )
        self.declare_parameter(
            'num_trajectory_samples', planner_config.get('num_samples', 401)
        )
        self.declare_parameter(
            'interpolation_method',
            planner_config.get('interpolation_method', 'cubic_spline'),
        )
        self.declare_parameter(
            'time_parameterization_method',
            planner_config.get(
                'time_parameterization_method', 'quintic_scaling'
            ),
        )
        self.declare_parameter(
            'interpolation_options',
            json.dumps(planner_config.get('interpolation_options', {})),
        )
        self.declare_parameter(
            'time_parameterization_options',
            json.dumps(
                planner_config.get('time_parameterization_options', {})
            ),
        )
        self.declare_parameter(
            'ik_solver', planner_config.get('ik_solver', 'damped_least_squares')
        )
        self.declare_parameter(
            'ik_max_iterations', planner_config.get('ik_max_iterations', 1000)
        )
        self.declare_parameter(
            'ik_position_tolerance',
            planner_config.get('ik_position_tolerance', 1e-4),
        )
        self.declare_parameter(
            'ik_orientation_tolerance',
            planner_config.get('ik_orientation_tolerance', 1e-3),
        )
        self.declare_parameter(
            'ik_restarts', planner_config.get('ik_restarts', 6)
        )
        self.declare_parameter('preview_rate', 30.0)
        self.declare_parameter('marker_publish_rate', 1.0)
        self.declare_parameter('looping', False)
        self.declare_parameter('waypoint_duration', planner_config.get('duration', 4.0))
        self.declare_parameter('waypoint_samples', planner_config.get('num_samples', 121))
        self.declare_parameter('gripper_open_position', 0.0)
        self.declare_parameter('gripper_closed_position', 0.015)
        self.declare_parameter('workspace_samples', 3000)
        self.declare_parameter('startup_delay', 0.5)
        self.declare_parameter(
            'max_joint_waypoint_step',
            planner_config.get('max_joint_waypoint_step', 0.5),
        )
        self.declare_parameter(
            'max_cartesian_refinement_depth',
            planner_config.get('max_cartesian_refinement_depth', 8),
        )
        self.declare_parameter(
            'cartesian_position_tolerance',
            planner_config.get('cartesian_position_tolerance', 0.002),
        )
        self.declare_parameter(
            'cartesian_orientation_tolerance',
            planner_config.get('cartesian_orientation_tolerance', 0.02),
        )
        self.interpolation_options = parse_parameter_options(
            self.get_parameter('interpolation_options').value
        )
        self.time_parameterization_options = parse_parameter_options(
            self.get_parameter('time_parameterization_options').value
        )
        self.preview_rate = validate_positive_rate(
            self.get_parameter('preview_rate').value, 'preview_rate'
        )
        self.marker_publish_rate = validate_positive_rate(
            self.get_parameter('marker_publish_rate').value,
            'marker_publish_rate',
        )
        self.looping = bool(self.get_parameter('looping').value)
        self.waypoint_duration = float(self.get_parameter('waypoint_duration').value)
        self.waypoint_samples = int(self.get_parameter('waypoint_samples').value)
        if not math.isfinite(self.waypoint_duration) or self.waypoint_duration <= 0:
            raise ValueError('waypoint_duration must be positive and finite.')
        if self.waypoint_samples < 2:
            raise ValueError('waypoint_samples must be at least 2.')
        self.startup_delay = float(self.get_parameter('startup_delay').value)
        if not math.isfinite(self.startup_delay) or self.startup_delay < 0:
            raise ValueError('startup_delay must be finite and nonnegative.')
        self.q_initial = np.asarray(
            self.get_parameter('initial_joint_configuration').value,
            dtype=float,
        )
        if (
            self.q_initial.shape != (self.robot.dof,)
            or not np.all(np.isfinite(self.q_initial))
            or not self.robot.check_joint_limits(self.q_initial)
        ):
            raise ValueError(
                'initial_joint_configuration must contain finite, in-limit '
                f'values with shape ({self.robot.dof},).'
            )
        self.default_start_pose = forward_kinematics(
            self.robot, self.q_initial
        )

        if self.pose_input_mode == 'yaml':
            if self.waypoint_configuration is not None:
                waypoint_poses = self.waypoint_configuration['poses']
                self.default_start_pose = self._pose_yaml_to_matrix(
                    waypoint_poses['initial'], 'poses.initial'
                )
                goal_name = 'goal' if 'goal' in waypoint_poses else 'place'
                self.default_goal_pose = self._pose_yaml_to_matrix(
                    waypoint_poses[goal_name], f'poses.{goal_name}'
                )
            else:
                self.default_start_pose, self.default_goal_pose = (
                    self.pose_configuration[:2]
                )
        else:
            goal_position = np.asarray(
                self.get_parameter('goal_position').value, dtype=float
            )
            goal_quaternion = np.asarray(
                self.get_parameter('goal_orientation').value, dtype=float
            )
            if (
                goal_position.shape != (3,)
                or not np.all(np.isfinite(goal_position))
            ):
                raise ValueError(
                    'goal_position must contain three finite values.'
                )
            if (
                goal_quaternion.shape != (4,)
                or not np.all(np.isfinite(goal_quaternion))
                or np.linalg.norm(goal_quaternion) <= np.finfo(float).eps
            ):
                raise ValueError(
                    'goal_orientation must be a finite nonzero [x, y, z, w] '
                    'quaternion.'
                )
            self.default_goal_pose = np.eye(4)
            self.default_goal_pose[:3, 3] = goal_position
            self.default_goal_pose[:3, :3] = quaternion_to_rotation(
                goal_quaternion
            )

        solver_name = str(self.get_parameter('ik_solver').value)
        solver_types = {
            'damped_least_squares': DampedLeastSquaresIK,
            'jacobian_transpose': JacobianTransposeIK,
            'pseudoinverse': PseudoinverseIK,
        }
        if solver_name not in solver_types:
            raise ValueError(
                'ik_solver must be damped_least_squares, '
                'jacobian_transpose, or pseudoinverse.'
            )
        self.ik_max_iterations = int(
            self.get_parameter('ik_max_iterations').value
        )
        self.ik_position_tolerance = float(
            self.get_parameter('ik_position_tolerance').value
        )
        self.ik_orientation_tolerance = float(
            self.get_parameter('ik_orientation_tolerance').value
        )
        self.ik_restarts = int(self.get_parameter('ik_restarts').value)
        if self.ik_max_iterations < 1 or self.ik_restarts < 0:
            raise ValueError('IK iteration and restart counts are invalid.')
        if (
            not math.isfinite(self.ik_position_tolerance)
            or self.ik_position_tolerance <= 0
            or not math.isfinite(self.ik_orientation_tolerance)
            or self.ik_orientation_tolerance <= 0
        ):
            raise ValueError('IK tolerances must be positive and finite.')
        self.solver = solver_types[solver_name](
            self.robot,
            position_tolerance=self.ik_position_tolerance,
            orientation_tolerance=self.ik_orientation_tolerance,
            max_iterations=self.ik_max_iterations,
        )

        self.reference_publisher = self.create_publisher(
            Marker,
            '/trajectory_preview/reference_path',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.actual_publisher = self.create_publisher(
            Marker,
            '/trajectory_preview/actual_path',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.joint_publisher = self.create_publisher(
            JointState, '/joint_states', 10
        )
        self.trajectory_plot_publisher = self.create_publisher(
            String,
            '/trajectory_preview/trajectory_data',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.workspace_publisher = self.create_publisher(
            Marker,
            '/trajectory_preview/reachable_workspace',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.gripper_position = float(
            self.get_parameter('gripper_open_position').value
        )
        self.gripper_open_position = self.gripper_position
        self.gripper_closed_position = float(
            self.get_parameter('gripper_closed_position').value
        )
        self.plan_service = self.create_service(
            Trigger, '/trajectory_preview/plan', self._plan_service
        )
        self.pose_controls = InteractivePoseControls(
            self,
            self.default_start_pose,
            self.default_goal_pose,
            self._pose_changed,
            self._menu_action,
            editable=(
                self.pose_input_mode == 'rviz'
                or self.waypoint_configuration is not None
            ),
            gripper_callback=(
                self._set_gripper
                if self.robot.name.lower() == 'gp7'
                else None
            ),
            workspace_callback=self._workspace_action,
        )
        self.waypoint_markers = []
        self._next_waypoint_id = 1
        if self.pose_input_mode == 'yaml' and self.waypoint_configuration:
            self._load_waypoint_pose_configuration()
        self.get_logger().info(
            f'Pose input mode: {self.pose_input_mode}. '
            'Waiting for Start Pose and Goal Pose. '
            'Use a marker menu or call /trajectory_preview/plan to plan.'
        )
        if self.robot.name.lower() == 'gp7':
            self._publish_joint_positions(
                self.q_initial, self.get_clock().now()
            )
            self._publish_gripper_state(self.get_clock().now())

    def _set_gripper(self, command):
        """Set the GP7 gripper slider position and publish its joint state."""
        self.gripper_position = (
            self.gripper_open_position
            if command == 'open'
            else self.gripper_closed_position
        )
        self._publish_gripper_state(self.get_clock().now())
        self.get_logger().info(f'GP7 gripper {command}.')

    def _publish_gripper_state(self, stamp):
        """Publish both mimic finger positions for RViz RobotModel."""
        message = JointState()
        message.header.stamp = stamp.to_msg()
        message.name = ['finger_left_joint', 'finger_right_joint']
        message.position = [self.gripper_position, self.gripper_position]
        self.joint_publisher.publish(message)

    def _plan_service(self, request, response):
        """Offer an explicit service trigger when the RViz menu is awkward."""
        del request
        self._plan_and_preview()
        response.success = self.state in (
            self.READY, self.PLAYING, self.COMPLETED
        )
        response.message = (
            'Trajectory validated; preview started.'
            if response.success
            else f'Planning ended in state {self.state}.'
        )
        return response

    def _pose_changed(self, marker_name, pose):
        """Invalidate old trajectory whenever either selected pose changes."""
        del marker_name, pose
        if self.preview_data is not None or self.state in (
            self.READY, self.PLAYING, self.COMPLETED
        ):
            self._cancel_timer('playback_timer')
            self._cancel_timer('marker_timer')
            self._delete_path_markers()
            self.preview_data = None
        self.state = self.WAITING_FOR_POSES

    def _menu_action(self, action):
        """Handle pose reset and explicit planning menu actions."""
        path_methods = {
            'plan_linear': 'cartesian_linear',
            'plan_arc': 'cartesian_arc',
            'plan_joint': 'joint_space',
        }
        if action in path_methods:
            self._plan_and_preview(path_methods[action])
        elif action == 'plan_waypoints':
            self._plan_joint_space_waypoints()
        elif action == 'reset_start':
            self.pose_controls.set_pose(
                self.pose_controls.START, self.default_start_pose
            )
        elif action == 'reset_goal':
            self.pose_controls.set_pose(
                self.pose_controls.GOAL, self.default_goal_pose
            )
        elif action == 'reset_both':
            self.pose_controls.set_pose(self.pose_controls.START, self.default_start_pose)
            self.pose_controls.set_pose(self.pose_controls.GOAL, self.default_goal_pose)

    def _load_waypoint_pose_configuration(self):
        """Load one initial pose, ordered waypoint list, and goal pose."""
        data = self.waypoint_configuration
        poses = data['poses']
        config = data['joint_space_trajectory']
        initial = self._pose_yaml_to_matrix(poses['initial'], 'poses.initial')
        goal_name = 'goal' if 'goal' in poses else 'place'
        goal = self._pose_yaml_to_matrix(
            poses[goal_name], f'poses.{goal_name}'
        )
        self.pose_controls.set_pose(self.pose_controls.START, initial)
        self.pose_controls.set_pose(self.pose_controls.GOAL, goal)
        self.waypoint_duration = float(config.get('duration', self.waypoint_duration))
        self.waypoint_samples = int(config.get('num_samples', self.waypoint_samples))
        entries = config.get('waypoints', [])
        if not isinstance(entries, list):
            raise ValueError('joint_space_trajectory.waypoints must be a list.')
        self.waypoint_markers.clear()
        for index, entry in enumerate(entries):
            pose = self._pose_yaml_to_matrix(
                entry, f'joint_space_trajectory.waypoints[{index}]'
            )
            self._insert_waypoint_marker('route', pose)

    @staticmethod
    def _read_waypoint_config(path):
        """Return explicit joint-space waypoint YAML, or None for pose YAML."""
        from pathlib import Path
        import yaml

        try:
            data = yaml.safe_load(Path(path).read_text())
        except (OSError, yaml.YAMLError) as error:
            raise ValueError(f'Cannot load preview YAML {path}: {error}') from error
        if not isinstance(data, dict):
            raise ValueError('Preview YAML must contain a top-level mapping.')
        if 'joint_space_trajectory' not in data:
            return None
        poses = data.get('poses')
        trajectory = data.get('joint_space_trajectory')
        if not isinstance(poses, dict) or not isinstance(trajectory, dict):
            raise ValueError(
                'Waypoint YAML requires poses and joint_space_trajectory mappings.'
            )
        if 'initial' not in poses or not (
            'goal' in poses or 'place' in poses
        ):
            raise ValueError(
                'Waypoint YAML poses must contain initial and goal '
                '(or legacy place).'
            )
        waypoints = trajectory.get('waypoints', [])
        if not isinstance(waypoints, list):
            raise ValueError('joint_space_trajectory.waypoints must be a list.')
        return data

    @staticmethod
    def _pose_yaml_to_matrix(value, name):
        if not isinstance(value, dict):
            raise ValueError(f'{name} must be a mapping with position and orientation_xyzw.')
        position = np.asarray(value.get('position'), dtype=float)
        quaternion = np.asarray(value.get('orientation_xyzw'), dtype=float)
        if (position.shape != (3,) or quaternion.shape != (4,)
                or not np.all(np.isfinite(position)) or not np.all(np.isfinite(quaternion))
                or np.linalg.norm(quaternion) <= np.finfo(float).eps):
            raise ValueError(f'{name} must contain finite xyz and a nonzero xyzw quaternion.')
        result = np.eye(4)
        result[:3, 3] = position
        result[:3, :3] = quaternion_to_rotation(quaternion / np.linalg.norm(quaternion))
        return result

    def _waypoint_menu_action(self, command, marker_name):
        if command == 'add_waypoint':
            self._insert_waypoint_marker(
                'route', self.pose_controls.get_pose_matrix(
                    self.pose_controls.GOAL
                )
            )
        elif command == 'remove_waypoint' and self.waypoint_markers:
            name, _, _ = self.waypoint_markers.pop()
            self.waypoint_server.erase(name)
            self.waypoint_server.applyChanges()
            self._pose_changed(name, None)

    def _insert_waypoint_marker(self, leg, pose):
        from interactive_markers import InteractiveMarkerServer
        if not hasattr(self, 'waypoint_server'):
            self.waypoint_server = InteractiveMarkerServer(self, '/trajectory_preview/waypoints')
        name = f'waypoint_{self._next_waypoint_id}'
        self._next_waypoint_id += 1
        marker = InteractiveMarker()
        marker.header.frame_id = 'base_link'
        marker.name = name
        marker.description = f'Waypoint {name.removeprefix("waypoint_")} ({leg})'
        marker.scale = 0.20
        marker.pose = self._matrix_pose(pose)
        visual = InteractiveMarkerControl()
        visual.always_visible = True
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.055
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = (0.9, 0.75, 0.1, 1.0)
        visual.markers.append(sphere)
        marker.controls.append(visual)
        root_half = 2.0 ** -0.5
        for axis, quaternion in (
            ('x', (root_half, 0.0, 0.0, root_half)),
            ('y', (0.0, root_half, 0.0, root_half)),
            ('z', (0.0, 0.0, root_half, root_half)),
        ):
            for mode, suffix in ((InteractiveMarkerControl.MOVE_AXIS, 'move'), (InteractiveMarkerControl.ROTATE_AXIS, 'rotate')):
                control = InteractiveMarkerControl()
                control.name = f'{axis}_{suffix}'
                (
                    control.orientation.x,
                    control.orientation.y,
                    control.orientation.z,
                    control.orientation.w,
                ) = map(float, quaternion)
                control.interaction_mode = mode
                marker.controls.append(control)
        self.waypoint_server.insert(marker, feedback_callback=self._waypoint_feedback)
        menu = MenuHandler()
        menu.insert('Remove This Waypoint', callback=lambda feedback: self._remove_waypoint(feedback.marker_name))
        menu.apply(self.waypoint_server, name)
        self.waypoint_server.applyChanges()
        self.waypoint_markers.append([name, leg, np.asarray(pose, dtype=float).copy()])

    def _remove_waypoint(self, marker_name):
        for index, item in enumerate(self.waypoint_markers):
            if item[0] == marker_name:
                self.waypoint_markers.pop(index)
                self.waypoint_server.erase(marker_name)
                self.waypoint_server.applyChanges()
                self._pose_changed(marker_name, None)
                return

    @staticmethod
    def _matrix_pose(transform):
        from .interactive_poses import matrix_to_pose_message
        return matrix_to_pose_message(transform)

    def _waypoint_feedback(self, feedback):
        if feedback.event_type == feedback.POSE_UPDATE:
            for item in self.waypoint_markers:
                if item[0] == feedback.marker_name:
                    item[2] = pose_message_to_matrix(feedback.pose)
                    break
            self._pose_changed(feedback.marker_name, feedback.pose)

    def _plan_joint_space_waypoints(self):
        """Plan the ordered initial-waypoints-goal route in joint space."""
        if self._planning:
            self.get_logger().warning('A planning request is already active.')
            return
        self._planning = True
        self.state = self.PLANNING
        self._cancel_timer('playback_timer')
        self._cancel_timer('marker_timer')
        self.preview_data = None
        try:
            start = self.pose_controls.get_pose_matrix(self.pose_controls.START)
            goal = self.pose_controls.get_pose_matrix(self.pose_controls.GOAL)
            route_poses = (
                [start]
                + [item[2] for item in self.waypoint_markers]
                + [goal]
            )
            q_waypoints = []
            seed = self.q_initial.copy()
            for index, pose in enumerate(route_poses):
                result = solve_ik_with_restarts(
                    self.solver, pose, seed,
                    num_restarts=self.ik_restarts,
                )
                if not result.success:
                    raise RuntimeError(
                        f'Waypoint {index} IK failed: {result.message}'
                    )
                seed = np.asarray(result.joint_positions, dtype=float)
                if not self.robot.check_joint_limits(seed):
                    raise RuntimeError(
                        f'Waypoint {index} IK is outside joint limits.'
                    )
                q_waypoints.append(seed.copy())
            trajectory = generate_joint_trajectory(
                q_waypoints, duration=self.waypoint_duration,
                num_samples=self.waypoint_samples,
                interpolation_method='quintic_polynomial',
                waypoint_times=np.linspace(
                    0.0, self.waypoint_duration, len(q_waypoints)
                ),
                robot_model=self.robot, enforce_position_limits=True,
                enforce_velocity_limits=True,
            )
            data = prepare_preview_data(
                trajectory, self.robot, route_poses,
                validate_cartesian_tracking=False,
            )
            if not data.validation.success:
                raise RuntimeError('; '.join(data.validation.errors))
            self._publish_trajectory_plot_data(trajectory)
            # For a joint-space route the commanded Cartesian reference is its
            # FK path, including the curvature between pose waypoints.
            data.reference_points = list(data.actual_points)
            self.preview_data = data
            self._delete_path_markers()
            self.q_start = q_waypoints[0].copy()
            self.samples = build_playback_samples(data.joint_samples, self.looping)
            self.index = 0
            self.playback_start_ros = None
            self.startup_start_ros = self.get_clock().now()
            self._publish_joint_positions(self.q_start, self.get_clock().now())
            self._publish_markers()
            self.marker_timer = self.create_timer(1.0 / self.marker_publish_rate, self._publish_markers)
            self.playback_timer = self.create_timer(1.0 / self.preview_rate, self._playback_tick)
            self.state = self.READY
            self.get_logger().info(
                'Joint-space waypoint trajectory ready with '
                f'{len(route_poses)} poses and {len(self.samples)} samples.'
            )
        except (ValueError, RuntimeError, np.linalg.LinAlgError) as error:
            self.state = self.FAILED
            self.get_logger().error(f'Waypoint planning failed: {error}')
        finally:
            self._planning = False

    def _cancel_timer(self, name):
        """Cancel and clear one existing timer without touching other timers."""
        timer = getattr(self, name, None)
        if timer is not None:
            timer.cancel()
            setattr(self, name, None)

    def _delete_path_markers(self):
        """Clear stale path markers after selected poses have changed."""
        stamp = self.get_clock().now().to_msg()
        for publisher, namespace, marker_id in (
            (self.reference_publisher, 'cartesian_reference', 0),
            (self.actual_publisher, 'cartesian_actual', 1),
        ):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = stamp
            marker.ns = namespace
            marker.id = marker_id
            marker.action = Marker.DELETE
            publisher.publish(marker)

    def _plan_and_preview(self, path_method=None):
        """Solve selected start IK, generate, validate, then start playback."""
        if self._planning:
            self.get_logger().warning('A planning request is already active.')
            return
        self._planning = True
        self.state = self.PLANNING
        if self.preview_data is not None:
            self._delete_path_markers()
        self._cancel_timer('playback_timer')
        self._cancel_timer('marker_timer')
        self.preview_data = None
        try:
            start_pose = self.pose_controls.get_pose_matrix(
                self.pose_controls.START
            )
            goal_pose = self.pose_controls.get_pose_matrix(
                self.pose_controls.GOAL
            )
            planned = plan_selected_poses(
                self.robot, self.solver, self.q_initial, start_pose, goal_pose,
                self._pipeline_options(path_method),
                num_ik_restarts=self.ik_restarts,
            )
            if not planned.success:
                self.state = self.FAILED
                self.get_logger().error(
                    f'Planning failed at {planned.failed_stage}: '
                    f'{planned.message}'
                )
                return

            self.state = self.VALIDATING
            result = planned.trajectory_result
            selected_method = self._pipeline_options(path_method)['path_method']
            preview_data = prepare_preview_data(
                result.joint_trajectory, self.robot, result.cartesian_path,
                time_parameterization=str(
                    self.get_parameter('time_parameterization_method').value
                ),
                time_parameterization_options=self.time_parameterization_options,
                cartesian_position_tolerance=float(
                    self.get_parameter('cartesian_position_tolerance').value
                ),
                cartesian_orientation_tolerance=float(
                    self.get_parameter('cartesian_orientation_tolerance').value
                ),
                validate_cartesian_tracking=(
                    selected_method != 'joint_space'
                ),
            )
            if not preview_data.validation.success:
                self.state = self.FAILED
                self.get_logger().error(
                    'Trajectory validation failed; refusing preview: '
                    + '; '.join(preview_data.validation.errors)
                )
                return
            self._publish_trajectory_plot_data(result.joint_trajectory)
            for warning in preview_data.validation.warnings:
                self.get_logger().warning(warning)

            self.preview_data = preview_data
            self.q_start = planned.initial_configuration.copy()
            self.samples = build_playback_samples(
                preview_data.joint_samples, self.looping
            )
            self.index = 0
            self.playback_start_ros = None
            self.startup_start_ros = self.get_clock().now()
            self._publish_joint_positions(self.q_start, self.get_clock().now())
            self._publish_markers()
            self.marker_timer = self.create_timer(
                1.0 / self.marker_publish_rate, self._publish_markers
            )
            self.playback_timer = self.create_timer(
                1.0 / self.preview_rate, self._playback_tick
            )
            self.state = self.READY
            self.get_logger().info(
                f'Trajectory validated: {len(self.samples)} samples, '
                f'{len(result.cartesian_path)} Cartesian waypoints. '
                'Playback is ready.'
            )
        except (ValueError, RuntimeError, np.linalg.LinAlgError) as error:
            self.state = self.FAILED
            self.get_logger().error(f'Planning failed: {error}')
        finally:
            self._planning = False

    def _pipeline_options(self, path_method=None):
        """Collect existing trajectory pipeline settings from ROS parameters."""
        return {
            'path_method': (
                str(self.get_parameter('path_method').value)
                if path_method is None
                else path_method
            ),
            'cartesian_arc_height': float(
                self.get_parameter('cartesian_arc_height').value
            ),
            'num_waypoints': int(
                self.get_parameter('num_cartesian_waypoints').value
            ),
            'duration': float(self.get_parameter('trajectory_duration').value),
            'num_samples': int(
                self.get_parameter('num_trajectory_samples').value
            ),
            'interpolation_method': str(
                self.get_parameter('interpolation_method').value
            ),
            'interpolation_options': self.interpolation_options,
            'time_parameterization': str(
                self.get_parameter('time_parameterization_method').value
            ),
            'time_parameterization_options': self.time_parameterization_options,
            'max_joint_waypoint_step': float(
                self.get_parameter('max_joint_waypoint_step').value
            ),
            'max_refinement_depth': int(
                self.get_parameter('max_cartesian_refinement_depth').value
            ),
            'ik_restarts': self.ik_restarts,
            'enforce_position_limits': True,
            'enforce_velocity_limits': True,
        }

    def _publish_trajectory_plot_data(self, trajectory):
        """Send validated position and derivative samples to the plotter."""
        message = String()
        message.data = json.dumps({
            'joint_names': self.robot.joint_names,
            'time': trajectory.time.tolist(),
            'positions': trajectory.positions.tolist(),
            'velocities': trajectory.velocities.tolist(),
            'accelerations': trajectory.accelerations.tolist(),
            'jerks': (
                None if trajectory.jerks is None
                else trajectory.jerks.tolist()
            ),
        })
        self.trajectory_plot_publisher.publish(message)

    def _workspace_action(self, action):
        """Show or remove sampled FK points for the configured joint limits."""
        if action == 'hide':
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'reachable_workspace'
            marker.id = 0
            marker.action = Marker.DELETE
            self.workspace_publisher.publish(marker)
            return

        sample_count = int(self.get_parameter('workspace_samples').value)
        if sample_count < 100:
            self.get_logger().warning(
                'workspace_samples below 100; using 100 samples.'
            )
            sample_count = 100
        rng = np.random.default_rng(17)
        configurations = rng.uniform(
            self.robot.lower_limits,
            self.robot.upper_limits,
            size=(sample_count, self.robot.dof),
        )
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'reachable_workspace'
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.008
        marker.color.r = 0.1
        marker.color.g = 0.75
        marker.color.b = 1.0
        marker.color.a = 0.32
        marker.points = [
            Point(x=float(position[0]), y=float(position[1]),
                  z=float(position[2]))
            for position in (
                forward_kinematics(self.robot, q)[:3, 3]
                for q in configurations
            )
        ]
        self.workspace_publisher.publish(marker)
        self.get_logger().info(
            f'Published {sample_count} approximate reachable TCP positions '
            'from joint-limit samples; collision checking is not included.'
        )

    def _playback_tick(self):
        """Publish due trajectory samples and handle end/loop behavior."""
        now = self.get_clock().now()
        if self.state == self.COMPLETED:
            final_sample = self.samples[-1]
            self._publish_joint_positions(
                final_sample.positions, now, final_sample.names
            )
            return
        if self.playback_start_ros is None:
            startup_elapsed = (
                now - self.startup_start_ros
            ).nanoseconds / 1e9
            if startup_elapsed < self.startup_delay:
                self._publish_joint_positions(self.q_start, now)
                return
            self.playback_start_ros = now
            self.state = self.PLAYING

        elapsed = (now - self.playback_start_ros).nanoseconds / 1e9
        if elapsed < 0:
            self.index = 0
            self.playback_start_ros = now
            self.startup_start_ros = now
            self._publish_joint_positions(self.q_start, now)
            return
        index = latest_due_sample_index(self.samples, elapsed, self.index)
        if index is not None:
            sample = self.samples[index]
            self._publish_joint_positions(sample.positions, now, sample.names)
            self.index = index + 1

        if self.index < len(self.samples):
            return
        if self.looping:
            self.index = 0
            self.playback_start_ros = now
        else:
            self.state = self.COMPLETED
            self.get_logger().info('Trajectory preview completed.')

    def destroy_node(self):
        """Stop timers and interactive marker server during shutdown."""
        self._cancel_timer('playback_timer')
        self._cancel_timer('marker_timer')
        if hasattr(self, 'pose_controls'):
            self.pose_controls.shutdown()
        super().destroy_node()

    def _publish_joint_positions(self, positions, stamp, names=None):
        """Publish one JointState with the requested trajectory timestamp."""
        message = JointState()
        message.header.stamp = stamp.to_msg()
        message.name = list(self.robot.joint_names if names is None else names)
        message.position = [float(value) for value in positions]
        if self.robot.name.lower() == 'gp7':
            message.name.extend([
                'finger_left_joint', 'finger_right_joint'
            ])
            message.position.extend([
                self.gripper_position, self.gripper_position
            ])
        self.joint_publisher.publish(message)

    def _publish_markers(self):
        """Publish the Cartesian reference and FK-derived actual paths."""
        if self.preview_data is None:
            return
        stamp = self.get_clock().now().to_msg()
        self.actual_publisher.publish(self._line_marker(
            'cartesian_actual', 1, stamp, self.preview_data.actual_points,
            (0.1, 0.9, 0.2, 1.0),
        ))
        self.reference_publisher.publish(self._line_marker(
            'cartesian_reference', 0, stamp, self.preview_data.reference_points,
            (0.1, 0.35, 1.0, 1.0),
        ))

    @staticmethod
    def _line_marker(namespace, marker_id, stamp, positions, color):
        """Build a base_link LINE_STRIP marker."""
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.006
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = [Point(x=x, y=y, z=z) for x, y, z in positions]
        return marker


def main(args=None):
    """Run the trajectory preview node until ROS shutdown."""
    rclpy.init(args=args)
    node = None
    status = 0
    try:
        node = TrajectoryPreviewNode()
        if node.preview_ready:
            rclpy.spin(node)
        else:
            status = 1
    except KeyboardInterrupt:
        status = 0
    except Exception as error:
        logger = (
            node.get_logger()
            if node is not None
            else rclpy.logging.get_logger('robot_arm_trajectory_preview')
        )
        logger.error(f'Preview startup failed: {error}')
        status = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return status
