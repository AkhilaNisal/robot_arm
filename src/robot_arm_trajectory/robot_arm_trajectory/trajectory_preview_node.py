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
)
from robot_arm_trajectory.preview_utils import (
    build_playback_samples,
    latest_due_sample_index,
    parse_parameter_options,
    prepare_preview_data,
    validate_positive_rate,
)
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker


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
        self.pose_configuration = load_preview_config(
            self.get_parameter('pose_config').value
        )
        planner_config = (
            self.pose_configuration[2]
            if self.pose_configuration is not None
            else {}
        )
        self.robot = RobotModel(
            self.get_parameter('robot_config').get_parameter_value().string_value
        )
        if self.robot.name.lower() == 'gp8':
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
        self.declare_parameter('preview_rate', 30.0)
        self.declare_parameter('marker_publish_rate', 1.0)
        self.declare_parameter('looping', False)
        self.declare_parameter('gripper_open_position', 0.015)
        self.declare_parameter('gripper_closed_position', 0.0)
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
        self.solver = solver_types[solver_name](self.robot)

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
            editable=self.pose_input_mode == 'rviz',
            gripper_callback=(
                self._set_gripper
                if self.robot.name.lower() == 'gp8'
                else None
            ),
        )
        self.get_logger().info(
            f'Pose input mode: {self.pose_input_mode}. '
            'Waiting for Start Pose and Goal Pose. '
            'Use a marker menu or call /trajectory_preview/plan to plan.'
        )
        if self.robot.name.lower() == 'gp8':
            self._publish_joint_positions(
                self.q_initial, self.get_clock().now()
            )
            self._publish_gripper_state(self.get_clock().now())

    def _set_gripper(self, command):
        """Set the GP8 gripper slider position and publish its joint state."""
        self.gripper_position = (
            self.gripper_open_position
            if command == 'open'
            else self.gripper_closed_position
        )
        self._publish_gripper_state(self.get_clock().now())
        self.get_logger().info(f'GP8 gripper {command}.')

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
        elif action == 'reset_start':
            self.pose_controls.set_pose(
                self.pose_controls.START, self.default_start_pose
            )
        elif action == 'reset_goal':
            self.pose_controls.set_pose(
                self.pose_controls.GOAL, self.default_goal_pose
            )
        elif action == 'reset_both':
            self.pose_controls.set_pose(
                self.pose_controls.START, self.default_start_pose
            )
            self.pose_controls.set_pose(
                self.pose_controls.GOAL, self.default_goal_pose
            )

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
            'enforce_position_limits': True,
            'enforce_velocity_limits': True,
        }

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
        if self.robot.name.lower() == 'gp8':
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
        self.reference_publisher.publish(self._line_marker(
            'cartesian_reference', 0, stamp, self.preview_data.reference_points,
            (0.1, 0.35, 1.0, 1.0),
        ))
        self.actual_publisher.publish(self._line_marker(
            'cartesian_actual', 1, stamp, self.preview_data.actual_points,
            (0.1, 0.9, 0.2, 1.0),
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
