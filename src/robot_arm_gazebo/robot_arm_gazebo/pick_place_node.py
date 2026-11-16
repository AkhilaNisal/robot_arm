"""Plan and execute a configurable GP8 pick-and-place sequence in Gazebo."""

from pathlib import Path
import json
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose
from interactive_markers import InteractiveMarkerServer, MenuHandler
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
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from robot_arm_trajectory.cartesian_space.cartesian_path import (
    quaternion_to_rotation,
    rotation_to_quaternion,
)
from robot_arm_trajectory.interactive_poses import matrix_to_pose_message
from robot_arm_trajectory.pipeline import generate_cartesian_joint_trajectory
from robot_arm_trajectory.preview_utils import prepare_preview_data
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from visualization_msgs.msg import Marker
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
import yaml


class PickPlaceNode(Node):
    """RViz pose selection and validated Gazebo trajectory execution."""

    POSES = ('initial', 'pick', 'place')

    def __init__(self):
        super().__init__('gp8_pick_place')
        default_config = str(
            Path(get_package_share_directory('robot_arm_gazebo'))
            / 'config/gp8_pick_place.yaml'
        )
        self.declare_parameter('robot_config', '')
        self.declare_parameter('pose_config', default_config)
        self.declare_parameter('pose_input_mode', 'rviz')
        self.declare_parameter('world_name', 'pick_place')
        self.declare_parameter('cube_name', 'gp8_demo_cube')
        path = Path(self.get_parameter('pose_config').value)
        try:
            data = yaml.safe_load(path.read_text())
        except (OSError, yaml.YAMLError) as error:
            raise ValueError(f'Cannot load pick/place YAML {path}: {error}') from error
        if not isinstance(data, dict) or not isinstance(data.get('poses'), dict):
            raise ValueError('Pick/place YAML needs a top-level poses mapping.')
        self.options = data.get('pick_place', {})
        object_data = data.get('objects') or {}
        self.cube_initial = np.asarray(self.options.get(
            'cube_initial_pose', object_data.get('box_b', {}).get(
                'position', data.get('poses', {}).get('pick', {}).get(
                    'position', [0.52, 0.0, 0.625]
                )
            )
        ), dtype=float)
        self.cube_place = np.asarray(self.options.get(
            'cube_place_pose', data.get('poses', {}).get('place', {}).get(
                'position', object_data.get('box_a', {}).get(
                    'position', [0.48, 0.32, 0.625]
                )
            )
        ), dtype=float)
        self.world_name = str(self.get_parameter('world_name').value)
        self.cube_name = str(self.get_parameter('cube_name').value)
        self.pose_mode = str(self.get_parameter('pose_input_mode').value).lower()
        if self.pose_mode == 'config':
            self.pose_mode = str(
                self.options.get('pose_input_mode', 'rviz')
            ).lower()
        if self.pose_mode not in ('rviz', 'yaml'):
            raise ValueError('pose_input_mode must be rviz or yaml.')
        robot_config = self.get_parameter('robot_config').value
        if not robot_config:
            robot_config = str(
                Path(get_package_share_directory('robot_arm_kinematics'))
                / 'config/gp8.yaml'
            )
        self.robot = RobotModel(robot_config)
        pose_config = data['poses']
        object_config = object_data
        self.poses = {
            'initial': self._load_pose(pose_config.get('initial'), 'initial')
        }
        box_b_config = object_config.get('box_b')
        box_a_config = object_config.get('box_a')
        self.box_a_transform = None
        self.box_a_size = None
        if isinstance(box_b_config, dict):
            self.poses['pick'] = self._load_pose(box_b_config, 'objects.box_b')
        else:
            self.poses['pick'] = self._load_pose(pose_config.get('pick'), 'pick')
        if isinstance(box_a_config, dict) and isinstance(box_b_config, dict):
            self.poses['place'] = self._load_pose(
                box_b_config, 'objects.box_b'
            )
            box_a_transform = self._load_pose(box_a_config, 'objects.box_a')
            box_a_size = np.asarray(box_a_config.get('size'), dtype=float)
            box_b_size = np.asarray(box_b_config.get('size'), dtype=float)
            if box_a_size.shape != (3,) or box_b_size.shape != (3,):
                raise ValueError('Box A and Box B sizes must each contain xyz.')
            self.box_a_transform = box_a_transform
            self.box_a_size = box_a_size
            self.poses['place'][:3, 3] = box_a_transform[:3, 3] + (
                box_a_transform[:3, :3]
                @ np.array([0.0, 0.0, (box_a_size[2] + box_b_size[2]) / 2.0])
            )
        else:
            self.poses['place'] = self._load_pose(
                pose_config.get('place'), 'place'
            )
        self.cube_spawn_transform = (
            self._load_pose(box_b_config, 'objects.box_b')
            if isinstance(box_b_config, dict)
            else self.poses['pick'].copy()
        )
        self.pose_defaults = {name: pose.copy() for name, pose in self.poses.items()}
        self.solver = self._make_solver(self.options.get(
            'ik_solver', 'damped_least_squares'
        ))
        self.path_method = str(self.options.get('path_method', 'joint_space'))
        self.planner_options = {
            'path_method': self.path_method,
            'cartesian_arc_height': float(self.options.get('cartesian_arc_height', 0.08)),
            'num_waypoints': int(self.options.get('num_cartesian_waypoints', 20)),
            'duration': float(self.options.get('duration', 3.0)),
            'num_samples': int(self.options.get('num_samples', 91)),
            'interpolation_method': str(self.options.get('interpolation_method', 'cubic_spline')),
            'interpolation_options': self.options.get('interpolation_options', {}),
            'time_parameterization': str(self.options.get(
                'time_parameterization_method', 'quintic_scaling'
            )),
            'time_parameterization_options': self.options.get(
                'time_parameterization_options', {}
            ),
            'max_joint_waypoint_step': float(self.options.get(
                'max_joint_waypoint_step', 0.5
            )),
            'max_refinement_depth': int(self.options.get(
                'max_cartesian_refinement_depth', 8
            )),
            'enforce_position_limits': True,
            'enforce_velocity_limits': True,
        }
        self.approach_height = float(self.options.get('approach_height', 0.10))
        self.cube_size = float(self.options.get('cube_size', 0.05))
        self.grasp_offset = np.asarray(
            self.options.get('grasp_offset', [0.0, 0.0, 0.0]), dtype=float
        )
        self.open_value = float(self.options.get('gripper_open_position', 0.0))
        self.closed_value = float(self.options.get('gripper_closed_position', 0.015))
        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory'
        )
        self.cube_pose_client = self.create_client(
            SetEntityPose,
            f'/gp8_pick_place_sim/world/{self.world_name}/set_pose'
        )
        self.plan_service = self.create_service(
            Trigger, '/gp8_pick_place/plan_and_execute', self._trigger_sequence
        )
        self.state = 'idle'
        self.plans = []
        self.reference_path_points = []
        self.actual_path_points = []
        self.last_actual_path_publish = 0.0
        self.segment_index = 0
        self.pending = None
        self.attached = False
        self.cube_initialized = False
        self.cube_init_pending = False
        self.cube_relative = np.eye(4)
        self.cube_update_pending = False
        follow_rate = max(
            10.0, float(self.options.get('cube_grasp_follow_rate', 100.0))
        )
        self._publish_timer = self.create_timer(1.0 / follow_rate, self._tick)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        path_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.reference_path_pub = self.create_publisher(
            Marker, '/trajectory_preview/reference_path', path_qos
        )
        self.actual_path_pub = self.create_publisher(
            Marker, '/trajectory_preview/actual_path', path_qos
        )
        self.trajectory_plot_pub = self.create_publisher(
            String, '/trajectory_preview/trajectory_data', path_qos
        )
        self.planning_progress_pub = self.create_publisher(
            String, '/gp8_pick_place/planning_progress', 10
        )
        self.current_arm_configuration = np.zeros(self.robot.dof)
        self.have_joint_state = False
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, 20
        )
        self.pose_server = InteractiveMarkerServer(self, '/gp8_pick_place/poses')
        self._setup_markers()
        if self.box_a_transform is not None:
            self.box_a_marker_pub = self.create_publisher(
                Marker, '/gp8_pick_place/box_a', 10
            )
            self.box_a_marker_timer = self.create_timer(
                0.5, self._publish_box_a_marker
            )
        if self.pose_mode == 'yaml':
            for marker_name in self.POSES:
                self.poses[marker_name] = self.pose_defaults[marker_name].copy()
        self.get_logger().info(
            f'Pick/place pose mode: {self.pose_mode}. Choose Plan and Execute '
            'Pick & Place from an RViz marker menu or call '
            '/gp8_pick_place/plan_and_execute.'
        )

    @staticmethod
    def _load_pose(value, name):
        if not isinstance(value, dict):
            raise ValueError(f'Pose YAML is missing poses.{name}.')
        position = np.asarray(value.get('position'), dtype=float)
        quaternion = np.asarray(value.get('orientation_xyzw'), dtype=float)
        if position.shape != (3,) or quaternion.shape != (4,):
            raise ValueError(f'poses.{name} must have xyz and xyzw values.')
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(quaternion)):
            raise ValueError(f'poses.{name} contains non-finite values.')
        if np.linalg.norm(quaternion) <= np.finfo(float).eps:
            raise ValueError(f'poses.{name} quaternion must be nonzero.')
        transform = np.eye(4)
        transform[:3, 3] = position
        transform[:3, :3] = quaternion_to_rotation(quaternion)
        return transform

    def _make_solver(self, name):
        solvers = {
            'damped_least_squares': DampedLeastSquaresIK,
            'jacobian_transpose': JacobianTransposeIK,
            'pseudoinverse': PseudoinverseIK,
        }
        if name not in solvers:
            raise ValueError(f'Unknown IK solver {name!r}.')
        return solvers[name](self.robot)

    def _setup_markers(self):
        self.menu = MenuHandler()
        self.execute_menu_id = self.menu.insert(
            'Plan and Execute Pick & Place', callback=self._menu_cb
        )
        self.reset_menu_id = self.menu.insert(
            'Reset All Poses', callback=self._menu_cb
        )
        for name, color in (
            ('initial', (0.15, 0.5, 1.0, 1.0)),
            ('pick', (0.2, 1.0, 0.3, 1.0)),
            ('place', (1.0, 0.45, 0.1, 1.0)),
        ):
            marker = InteractiveMarker()
            marker.header.frame_id = 'base_link'
            marker.name = name
            marker.description = name.title() + ' Pose'
            marker.scale = 0.22
            marker.pose = matrix_to_pose_message(self.poses[name])
            visual = InteractiveMarkerControl()
            visual.always_visible = True
            sphere = Marker()
            sphere.type = Marker.SPHERE
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.06
            sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = color
            visual.markers.append(sphere)
            marker.controls.append(visual)
            menu_control = InteractiveMarkerControl()
            menu_control.name = 'pose_menu'
            menu_control.interaction_mode = InteractiveMarkerControl.MENU
            marker.controls.append(menu_control)
            if self.pose_mode == 'rviz':
                root_half = 2.0 ** -0.5
                for axis, q in (
                    ('x', (root_half, 0.0, 0.0, root_half)),
                    ('y', (0.0, root_half, 0.0, root_half)),
                    ('z', (0.0, 0.0, root_half, root_half)),
                ):
                    for mode, suffix in (
                        (InteractiveMarkerControl.MOVE_AXIS, 'move'),
                        (InteractiveMarkerControl.ROTATE_AXIS, 'rotate'),
                    ):
                        control = InteractiveMarkerControl()
                        control.name = axis + '_' + suffix
                        control.orientation.x, control.orientation.y = q[:2]
                        control.orientation.z, control.orientation.w = q[2:]
                        control.interaction_mode = mode
                        marker.controls.append(control)
            self.pose_server.insert(
                marker, feedback_callback=self._feedback
            )
            self.menu.apply(self.pose_server, name)
        self.pose_server.applyChanges()

    def _feedback(self, feedback):
        if feedback.event_type != feedback.POSE_UPDATE:
            return
        p = feedback.pose.position
        q = feedback.pose.orientation
        quat = np.array([q.x, q.y, q.z, q.w], dtype=float)
        quat /= np.linalg.norm(quat)
        transform = np.eye(4)
        transform[:3, 3] = [p.x, p.y, p.z]
        transform[:3, :3] = quaternion_to_rotation(quat)
        self.poses[feedback.marker_name] = transform

    def _publish_box_a_marker(self):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'assignment_objects'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = matrix_to_pose_message(self.box_a_transform)
        marker.scale.x, marker.scale.y, marker.scale.z = map(
            float, self.box_a_size
        )
        marker.color.r = 0.16
        marker.color.g = 0.42
        marker.color.b = 0.86
        marker.color.a = 0.9
        self.box_a_marker_pub.publish(marker)

    def _publish_path_marker(self, publisher, points, namespace, color):
        if not points:
            return
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.015
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        marker.points = [Point(x=x, y=y, z=z) for x, y, z in points]
        publisher.publish(marker)

    def _publish_planning_progress(self, percent, status):
        message = String()
        message.data = f'{int(percent)}|{status}'
        self.planning_progress_pub.publish(message)

    def _delete_path_marker(self, publisher, namespace):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = 0
        marker.action = Marker.DELETE
        publisher.publish(marker)

    def _publish_trajectory_plot_data(self):
        if not self.plans:
            return
        times = []
        positions = []
        velocities = []
        accelerations = []
        jerks = []
        time_offset = 0.0
        all_jerks_available = True
        for plan_index, trajectory in enumerate(self.plans):
            local_time = trajectory.time - trajectory.time[0] + time_offset
            start = 0 if plan_index == 0 else 1
            times.extend(local_time[start:].tolist())
            positions.extend(trajectory.positions[start:].tolist())
            velocities.extend(trajectory.velocities[start:].tolist())
            accelerations.extend(trajectory.accelerations[start:].tolist())
            if trajectory.jerks is None:
                all_jerks_available = False
            else:
                jerks.extend(trajectory.jerks[start:].tolist())
            time_offset = float(local_time[-1])
        message = String()
        message.data = json.dumps({
            'joint_names': self.robot.joint_names,
            'time': times,
            'positions': positions,
            'velocities': velocities,
            'accelerations': accelerations,
            'jerks': jerks if all_jerks_available else None,
        })
        self.trajectory_plot_pub.publish(message)

    def _joint_state_cb(self, message):
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in self.robot.joint_names):
            self.current_arm_configuration = np.asarray(
                [positions[name] for name in self.robot.joint_names], dtype=float
            )
            self.have_joint_state = True
            moving_states = {
                'moving_to_initial', 'moving_to_pick_approach', 'moving_to_pick',
                'lifting', 'transferring', 'placing', 'retreating', 'returning',
            }
            if self.state in moving_states:
                position = forward_kinematics(
                    self.robot, self.current_arm_configuration
                )[:3, 3]
                if (
                    not self.actual_path_points
                    or np.linalg.norm(
                        position - np.asarray(self.actual_path_points[-1])
                    ) > 0.001
                ):
                    self.actual_path_points.append(tuple(map(float, position)))
                    now = time.monotonic()
                    if now - self.last_actual_path_publish >= 0.1:
                        self._publish_path_marker(
                            self.actual_path_pub,
                            self.actual_path_points,
                            'actual_path',
                            (0.12, 0.95, 0.24, 1.0),
                        )
                        self.last_actual_path_publish = now

    def _menu_cb(self, feedback):
        if feedback.menu_entry_id == self.reset_menu_id:
            for name in self.POSES:
                self.poses[name] = self.pose_defaults[name].copy()
                self.pose_server.setPose(name, matrix_to_pose_message(self.poses[name]))
            self.pose_server.applyChanges()
        else:
            self._begin_sequence()

    def _trigger_sequence(self, request, response):
        del request
        started = self._begin_sequence()
        response.success = started
        response.message = 'Planning started.' if started else f'Busy: {self.state}'
        return response

    def _begin_sequence(self):
        if self.state not in ('idle', 'completed', 'failed'):
            self.get_logger().warning(f'Cannot start pick/place while {self.state}.')
            return False
        if not self.have_joint_state:
            self.get_logger().error(
                'Cannot plan pick/place until live arm joint states arrive.'
            )
            return False
        # Interactive markers are expressed in base_link, coincident with the
        # Gazebo world origin. Place the simulated cube at the selected grasp
        # target before planning so changing Pick in RViz cannot leave the box
        # somewhere else in the scene.
        self.cube_initial = (
            self.poses['pick'][:3, 3]
            + self.poses['pick'][:3, :3] @ self.grasp_offset
        )
        if not self.cube_pose_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().error('Gazebo set_pose service is unavailable.')
            return False
        self.reference_path_points = []
        self.actual_path_points = []
        self.last_actual_path_publish = 0.0
        self._publish_planning_progress(0, 'Preparing pick and place plan')
        self._delete_path_marker(self.reference_path_pub, 'reference_path')
        self._delete_path_marker(self.actual_path_pub, 'actual_path')
        self.state = 'syncing_cube'
        self._set_cube_pose(
            self._cube_transform_from_pick(), self._cube_sync_response
        )
        return True

    def _cube_transform_from_pick(self):
        transform = self.poses['pick'].copy()
        transform[:3, 3] += transform[:3, :3] @ self.grasp_offset
        return transform

    def _plan_and_start(self):
        self._publish_planning_progress(0, 'Planning 0 of 8 segments')
        try:
            self.plans = self._plan_all_segments()
        except (ValueError, RuntimeError, np.linalg.LinAlgError) as error:
            self.state = 'failed'
            self._publish_planning_progress(0, f'Planning failed: {error}')
            self.get_logger().error(f'Pick/place planning failed: {error}')
            return False
        if not self.plans:
            self.state = 'failed'
            return False
        self.segment_index = 0
        self.state = 'opening'
        self.pending = None
        self._publish_path_marker(
            self.reference_path_pub,
            self.reference_path_points,
            'reference_path',
            (0.12, 0.35, 1.0, 1.0),
        )
        self._publish_trajectory_plot_data()
        self._publish_planning_progress(100, 'Plan ready; executing')
        self.get_logger().info(
            f'Published reference path ({len(self.reference_path_points)} points) '
            'on /trajectory_preview/reference_path.'
        )
        self.get_logger().info(
            f'All {len(self.plans)} motion segments validated; executing.'
        )
        return True

    def _cube_sync_response(self, future):
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError('Gazebo rejected the selected cube pose.')
            self._plan_and_start()
        except Exception as error:
            self.state = 'failed'
            self.get_logger().error(f'Could not align cube to Pick pose: {error}')

    def _offset(self, pose, z):
        out = pose.copy()
        out[2, 3] += z
        return out

    def _plan_all_segments(self):
        initial = self.poses['initial']
        pick = self.poses['pick']
        place = self.poses['place']
        targets = [
            self._offset(initial, self.approach_height), self._offset(pick, self.approach_height), pick,
            self._offset(pick, self.approach_height),
            self._offset(place, self.approach_height), place,
            self._offset(place, self.approach_height), initial,
        ]
        q = self.current_arm_configuration.copy()
        plans = []
        total_segments = len(targets)
        for index, target in enumerate(targets):
            self._publish_planning_progress(
                100.0 * index / total_segments,
                f'Planning segment {index + 1} of {total_segments}',
            )
            result = generate_cartesian_joint_trajectory(
                robot_model=self.robot,
                ik_solver=self.solver,
                q_initial=q,
                start_pose=forward_kinematics(self.robot, q),
                goal_pose=target,
                **self.planner_options,
            )
            if not result.success:
                raise RuntimeError(
                    f'Segment {index + 1} planning failed at '
                    f'{result.failed_stage}: {result.message}'
                )
            checked = prepare_preview_data(
                result.joint_trajectory, self.robot, result.cartesian_path,
                time_parameterization=self.planner_options[
                    'time_parameterization'
                ],
                time_parameterization_options=self.planner_options[
                    'time_parameterization_options'
                ],
                validate_cartesian_tracking=(
                    self.path_method != 'joint_space'
                ),
            )
            if not checked.validation.success:
                raise RuntimeError(
                    f'Segment {index + 1} validation failed: '
                    + '; '.join(checked.validation.errors)
                )
            for point in checked.reference_points:
                if (
                    not self.reference_path_points
                    or np.linalg.norm(
                        np.asarray(point)
                        - np.asarray(self.reference_path_points[-1])
                    ) > 1e-6
                ):
                    self.reference_path_points.append(tuple(map(float, point)))
            plans.append(result.joint_trajectory)
            q = result.joint_trajectory.positions[-1].copy()
            self._publish_planning_progress(
                100.0 * (index + 1) / total_segments,
                f'Validated segment {index + 1} of {total_segments}',
            )
        return plans

    def _tick(self):
        if not self.cube_initialized:
            if (
                not self.cube_init_pending
                and self.cube_pose_client.wait_for_service(timeout_sec=0.0)
            ):
                self.cube_init_pending = True
                self._set_cube_pose(
                    self.cube_spawn_transform,
                    self._cube_initial_response,
                )
            return
        if self.state == 'opening':
            self._send_gripper(self.open_value)
            self.state = 'wait_open'
        elif self.state == 'wait_open' and self.pending is None:
            self.state = 'moving_to_initial'
            self._send_arm(self.plans[0])
        elif self.state == 'moving_to_initial' and self.pending is None:
            self.state = 'moving_to_pick_approach'
            self._send_arm(self.plans[1])
        elif self.state == 'moving_to_pick_approach' and self.pending is None:
            self.state = 'moving_to_pick'
            self._send_arm(self.plans[2])
        elif self.state == 'moving_to_pick' and self.pending is None:
            self.state = 'closing'
            self._send_gripper(self.closed_value)
        elif self.state == 'closing' and self.pending is None:
            if self._set_attached(True):
                self.state = 'lifting'
                self._send_arm(self.plans[3])
        elif self.state == 'lifting' and self.pending is None:
            self.state = 'transferring'
            self._send_arm(self.plans[4])
        elif self.state == 'transferring' and self.pending is None:
            self.state = 'placing'
            self._send_arm(self.plans[5])
        elif (
            self.state == 'placing' and self.pending is None
            and not self.cube_update_pending
        ):
            self.state = 'releasing'
            if self._set_attached(False):
                self._send_gripper(self.open_value)
        elif self.state == 'releasing' and self.pending is None:
            self.state = 'retreating'
            self._send_arm(self.plans[6])
        elif self.state == 'retreating' and self.pending is None:
            self.state = 'returning'
            self._send_arm(self.plans[7])
        elif self.state == 'returning' and self.pending is None:
            self.state = 'completed'
            if self.actual_path_points:
                self._publish_path_marker(
                    self.actual_path_pub,
                    self.actual_path_points,
                    'actual_path',
                    (0.12, 0.95, 0.24, 1.0),
                )
            self._publish_planning_progress(100, 'Pick and place completed')
            self.get_logger().info('Pick and place completed.')
        if self.attached and not self.cube_update_pending:
            self._update_attached_cube()

    def _send_arm(self, trajectory):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = self._to_message(trajectory, self.robot.joint_names)
        if not self.arm_client.wait_for_server(timeout_sec=0.1):
            self.state = 'failed'
            self.get_logger().error('Arm trajectory controller is unavailable.')
            return
        self.pending = self.arm_client.send_goal_async(goal)
        self.pending.add_done_callback(self._goal_response)

    def _send_gripper(self, value):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['finger_left_joint', 'finger_right_joint']
        point = JointTrajectoryPoint()
        point.positions = [value, value]
        point.time_from_start.sec = 1
        goal.trajectory.points = [point]
        if not self.gripper_client.wait_for_server(timeout_sec=0.1):
            self.state = 'failed'
            self.get_logger().error('Gripper trajectory controller is unavailable.')
            return
        self.pending = self.gripper_client.send_goal_async(goal)
        self.pending.add_done_callback(self._goal_response)

    def _goal_response(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self.state = 'failed'
                self.get_logger().error('Gazebo controller rejected goal.')
                self.pending = None
                return
            result_future = handle.get_result_async()
            self.pending = result_future
            result_future.add_done_callback(self._action_result)
        except Exception as error:
            self.state = 'failed'
            self.get_logger().error(f'Controller goal failed: {error}')
            self.pending = None

    def _action_result(self, future):
        try:
            result = future.result()
            if result.status != 4:
                self.state = 'failed'
                self.get_logger().error(
                    f'Controller action ended with status {result.status}.'
                )
        except Exception as error:
            self.state = 'failed'
            self.get_logger().error(f'Gazebo execution failed: {error}')
        finally:
            self.pending = None

    @staticmethod
    def _to_message(trajectory, joint_names):
        message = JointTrajectory()
        message.joint_names = list(joint_names)
        for index, seconds in enumerate(trajectory.time):
            point = JointTrajectoryPoint()
            point.positions = trajectory.positions[index].tolist()
            point.velocities = trajectory.velocities[index].tolist()
            point.accelerations = trajectory.accelerations[index].tolist()
            point.time_from_start.sec = int(seconds)
            point.time_from_start.nanosec = int((seconds % 1.0) * 1e9)
            message.points.append(point)
        return message

    def _set_attached(self, attached):
        if not self.cube_pose_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().error('Gazebo set_pose service is unavailable.')
            self.state = 'failed'
            return False
        if attached:
            # Attach from the measured configuration after the close action.
            tool = forward_kinematics(self.robot, self.current_arm_configuration)
            cube = tool.copy()
            cube[:3, 3] += tool[:3, :3] @ self.grasp_offset
            delta = np.linalg.norm(cube[:3, 3] - self.cube_initial)
            grasp_tolerance = float(self.options.get('grasp_tolerance', 0.10))
            if delta > grasp_tolerance:
                self.get_logger().error(
                    f'Grasp rejected: TCP is {delta:.3f} m from cube center '
                    f'(limit {grasp_tolerance:.3f} m). Align Pick marker and cube.'
                )
                self.state = 'failed'
                return False
            self.cube_relative = np.linalg.inv(tool) @ cube
        self.attached = attached
        if not attached:
            # Place pose describes the TCP; use the configured grasp offset to
            # derive the intended cube-centre pose from that target.
            released = self.poses['place'].copy()
            released[:3, 3] += (
                released[:3, :3] @ self.grasp_offset
            )
            self._set_cube_pose(released)
        return True

    def _update_attached_cube(self):
        # Gazebo's SetEntityPose bridge is used as a deterministic grasp
        # constraint. Keep at most one update in flight and drive it from live
        # joint feedback, not from a precomputed waypoint or a guessed pose.
        tool = forward_kinematics(self.robot, self.current_arm_configuration)
        cube = tool @ self.cube_relative
        self.cube_update_pending = True
        self._set_cube_pose(cube, self._cube_follow_response)

    def _cube_follow_response(self, future):
        self.cube_update_pending = False
        self._cube_pose_response(future)

    def _set_cube_position(self, xyz, response_callback=None):
        transform = np.eye(4)
        transform[:3, 3] = xyz
        return self._set_cube_pose(transform, response_callback)

    def _set_cube_pose(self, transform, response_callback=None):
        request = SetEntityPose.Request()
        request.entity.name = self.cube_name
        request.entity.type = Entity.MODEL
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(
            float, transform[:3, 3]
        )
        quaternion = rotation_to_quaternion(transform[:3, :3])
        (pose.orientation.x, pose.orientation.y,
         pose.orientation.z, pose.orientation.w) = map(float, quaternion)
        request.pose = pose
        future = self.cube_pose_client.call_async(request)
        future.add_done_callback(
            response_callback or self._cube_pose_response
        )
        return future

    def _cube_initial_response(self, future):
        self.cube_init_pending = False
        try:
            response = future.result()
            self.cube_initialized = bool(response.success)
            if not self.cube_initialized:
                self.get_logger().info(
                    'Waiting for Gazebo to register the pick/place cube.'
                )
        except Exception as error:
            self.get_logger().warning(
                f'Waiting for cube initialization: {error}'
            )

    def _cube_pose_response(self, future):
        try:
            result = future.result()
            if not result.success:
                self.get_logger().warning('Gazebo did not update the cube pose.')
        except Exception as error:
            self.get_logger().warning(f'Cube pose update failed: {error}')

    def destroy_node(self):
        self.pose_server.marker_contexts.clear()
        self.pose_server.pending_updates.clear()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PickPlaceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
