"""Execute YAML-defined joint-space waypoint trajectories in Gazebo."""

from pathlib import Path

import numpy as np
import rclpy
import yaml

from robot_arm_kinematics.forward_kinematics import forward_kinematics
from robot_arm_trajectory.cartesian_space.cartesian_ik import (
    solve_ik_with_restarts,
)
from robot_arm_trajectory.joint_space.joint_trajectory import (
    generate_joint_trajectory,
)
from robot_arm_trajectory.preview_utils import prepare_preview_data
from robot_arm_trajectory.interactive_poses import pose_message_to_matrix
from interactive_markers import InteractiveMarkerServer, MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl
from visualization_msgs.msg import Marker

from .pick_place_node import PickPlaceNode


class WaypointPickPlaceNode(PickPlaceNode):
    """Pick/place executor that plans a joint waypoint path for each leg."""

    SUPPORTS_WAYPOINTS = True

    def __init__(self):
        super().__init__()
        config_path = Path(self.get_parameter('pose_config').value)
        try:
            data = yaml.safe_load(config_path.read_text())
        except (OSError, yaml.YAMLError) as error:
            raise ValueError(
                f'Cannot load waypoint trajectory YAML {config_path}: {error}'
            ) from error

        pose_data = data.get('poses') if isinstance(data, dict) else None
        trajectory_data = (
            data.get('joint_space_trajectory')
            if isinstance(data, dict) else None
        )
        if not isinstance(pose_data, dict):
            raise ValueError(
                'Waypoint YAML requires poses.initial/pick/place.'
            )
        if not isinstance(trajectory_data, dict):
            raise ValueError('Waypoint YAML requires joint_space_trajectory.')

        self.poses = {
            name: self._load_pose(pose_data.get(name), name)
            for name in self.POSES
        }
        self.pose_defaults = {
            name: pose.copy() for name, pose in self.poses.items()
        }
        for name, pose in self.poses.items():
            self.pose_server.setPose(name, self._pose_to_message(pose))
        self.pose_server.applyChanges()

        self.approach_height = float(
            trajectory_data.get('approach_height', self.approach_height)
        )
        self.duration = float(trajectory_data.get('duration', 4.0))
        self.num_samples = int(trajectory_data.get('num_samples', 81))
        if not np.isfinite(self.duration) or self.duration <= 0:
            raise ValueError(
                'joint_space_trajectory.duration must be positive.'
            )
        if self.num_samples < 2:
            raise ValueError(
                'joint_space_trajectory.num_samples must be >= 2.'
            )

        self.waypoint_legs = self._load_waypoint_legs(
            trajectory_data.get('legs', {})
        )
        self.waypoint_markers = []
        self._next_waypoint_id = 1
        self.waypoint_defaults = {
            leg: [entry['pose'].copy() for entry in entries]
            for leg, entries in self.waypoint_legs.items()
        }
        for leg, entries in self.waypoint_legs.items():
            for entry in entries:
                self._insert_waypoint_marker(leg, entry['pose'])

    @staticmethod
    def _pose_to_message(transform):
        from robot_arm_trajectory.interactive_poses import (
            matrix_to_pose_message,
        )

        return matrix_to_pose_message(transform)

    def _load_waypoint_legs(self, values):
        if not isinstance(values, dict):
            raise ValueError('joint_space_trajectory.legs must be a mapping.')
        legs = {}
        for leg_name in (
            'initial_to_pick', 'pick_to_place', 'place_to_initial'
        ):
            entries = values.get(leg_name, [])
            if not isinstance(entries, list):
                raise ValueError(f'legs.{leg_name} must be a list.')
            legs[leg_name] = [
                self._load_waypoint(entry, f'legs.{leg_name}[{index}]')
                for index, entry in enumerate(entries)
            ]
        return legs

    def _add_waypoint_marker(self, leg, pose):
        self.waypoint_legs[leg].append({'pose': np.asarray(pose).copy()})
        self._insert_waypoint_marker(leg, pose)

    def _insert_waypoint_marker(self, leg, pose):
        if not hasattr(self, 'waypoint_server'):
            self.waypoint_server = InteractiveMarkerServer(
                self, '/gp8_pick_place/waypoints'
            )
        name = f'waypoint_{self._next_waypoint_id}'
        self._next_waypoint_id += 1
        marker = InteractiveMarker()
        marker.header.frame_id = 'base_link'
        marker.name = name
        marker.description = f'Waypoint {name.removeprefix("waypoint_")} ({leg})'
        marker.scale = 0.20
        marker.pose = self._pose_to_message(pose)

        visual = InteractiveMarkerControl()
        visual.always_visible = True
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.05
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = (
            0.95, 0.72, 0.12, 1.0
        )
        visual.markers.append(sphere)
        marker.controls.append(visual)

        half = 2.0 ** -0.5
        for axis, orientation in (
            ('x', (half, 0.0, 0.0, half)),
            ('y', (0.0, half, 0.0, half)),
            ('z', (0.0, 0.0, half, half)),
        ):
            for mode, suffix in (
                (InteractiveMarkerControl.MOVE_AXIS, 'move'),
                (InteractiveMarkerControl.ROTATE_AXIS, 'rotate'),
            ):
                control = InteractiveMarkerControl()
                control.name = f'{axis}_{suffix}'
                (
                    control.orientation.x,
                    control.orientation.y,
                    control.orientation.z,
                    control.orientation.w,
                ) = map(float, orientation)
                control.interaction_mode = mode
                marker.controls.append(control)

        self.waypoint_server.insert(
            marker, feedback_callback=self._waypoint_feedback
        )
        menu = MenuHandler()
        menu.insert(
            'Remove This Waypoint',
            callback=lambda feedback: self._remove_waypoint(
                feedback.marker_name
            ),
        )
        menu.apply(self.waypoint_server, name)
        self.waypoint_server.applyChanges()
        self.waypoint_markers.append([
            name, leg, np.asarray(pose, dtype=float).copy()
        ])

    def _waypoint_feedback(self, feedback):
        if feedback.event_type != feedback.POSE_UPDATE:
            return
        for item in self.waypoint_markers:
            if item[0] == feedback.marker_name:
                item[2] = pose_message_to_matrix(feedback.pose)
                self._replace_waypoint_pose(item[1], item[0], item[2])
                return

    def _replace_waypoint_pose(self, leg, marker_name, pose):
        entry_index = next(
            index for index, item in enumerate(self.waypoint_markers)
            if item[0] == marker_name
        )
        leg_index = sum(
            1 for item in self.waypoint_markers[:entry_index]
            if item[1] == leg
        )
        self.waypoint_legs[leg][leg_index] = {'pose': pose.copy()}

    def _remove_waypoint(self, marker_name):
        for index, item in enumerate(self.waypoint_markers):
            if item[0] != marker_name:
                continue
            _, leg, _ = self.waypoint_markers.pop(index)
            leg_index = sum(
                1 for previous in self.waypoint_markers[:index]
                if previous[1] == leg
            )
            self.waypoint_legs[leg].pop(leg_index)
            self.waypoint_server.erase(marker_name)
            self.waypoint_server.applyChanges()
            return

    def _reset_waypoint_markers(self):
        if not hasattr(self, 'waypoint_server'):
            return
        for name, _, _ in self.waypoint_markers:
            self.waypoint_server.erase(name)
        self.waypoint_markers.clear()
        self.waypoint_legs = {leg: [] for leg in self.waypoint_defaults}
        for leg, poses in self.waypoint_defaults.items():
            for pose in poses:
                self.waypoint_legs[leg].append({'pose': pose.copy()})
                self._insert_waypoint_marker(leg, pose)
        self.waypoint_server.applyChanges()

    def _load_waypoint(self, value, name):
        if not isinstance(value, dict):
            raise ValueError(f'{name} must be a mapping.')
        return {'pose': self._load_pose(value, name)}

    def _plan_all_segments(self):
        if self.selected_path_method != 'joint_space_waypoints':
            return super()._plan_all_segments()
        initial = {'pose': self.poses['initial']}
        pick = {'pose': self.poses['pick']}
        place = {'pose': self.poses['place']}
        pick_approach = {
            'pose': self._offset(self.poses['pick'], self.approach_height)
        }
        place_approach = {
            'pose': self._offset(self.poses['place'], self.approach_height)
        }

        routes = [
            [initial]
            + self.waypoint_legs['initial_to_pick']
            + [pick_approach, pick],
            [pick_approach]
            + self.waypoint_legs['pick_to_place']
            + [place_approach, place],
            [place_approach]
            + self.waypoint_legs['place_to_initial']
            + [initial],
        ]
        q = self.current_arm_configuration.copy()
        plans = []
        for leg_index, route in enumerate(routes):
            self._publish_planning_progress(
                100.0 * leg_index / len(routes),
                f'Planning waypoint leg {leg_index + 1} of {len(routes)}',
            )
            joint_waypoints = [q.copy()]
            reference_path = [forward_kinematics(self.robot, q)]

            for waypoint_index, waypoint in enumerate(route):
                progress = 100.0 * (
                    leg_index + waypoint_index / len(route)
                ) / len(routes)
                self._publish_planning_progress(
                    progress,
                    f'Planning leg {leg_index + 1}/{len(routes)}, waypoint '
                    f'{waypoint_index + 1}/{len(route)}',
                )
                result = solve_ik_with_restarts(
                    self.solver, waypoint['pose'], q,
                    num_restarts=self.planner_options['ik_restarts'],
                )
                if not result.success:
                    raise RuntimeError(
                        f'Waypoint leg {leg_index + 1} IK failed: '
                        f'{result.message}'
                    )
                q = self.robot.enforce_joint_limits(
                    np.asarray(result.joint_positions, dtype=float)
                )
                joint_waypoints.append(q.copy())
                reference_path.append(waypoint['pose'])

            try:
                trajectory = generate_joint_trajectory(
                    joint_waypoints,
                    duration=self.duration,
                    num_samples=self.num_samples,
                    interpolation_method='quintic_polynomial',
                    waypoint_times=np.linspace(
                        0.0, self.duration, len(joint_waypoints)
                    ),
                    joint_velocity_limits=self.robot.velocity_limits,
                    enforce_position_limits=True,
                    enforce_velocity_limits=True,
                    robot_model=self.robot,
                )
            except (ValueError, RuntimeError, np.linalg.LinAlgError) as error:
                raise RuntimeError(
                    f'Waypoint leg {leg_index + 1} trajectory failed: {error}'
                ) from error
            checked = prepare_preview_data(
                trajectory, self.robot, reference_path,
                validate_cartesian_tracking=False,
            )
            if not checked.validation.success:
                raise RuntimeError(
                    f'Waypoint leg {leg_index + 1} validation failed: '
                    + '; '.join(checked.validation.errors)
                )
            for point in checked.actual_points:
                if (
                    not self.reference_path_points
                    or np.linalg.norm(
                        np.asarray(point)
                        - np.asarray(self.reference_path_points[-1])
                    ) > 1e-6
                ):
                    self.reference_path_points.append(tuple(map(float, point)))
            plans.append(trajectory)
            q = trajectory.positions[-1].copy()
            self._publish_planning_progress(
                100.0 * (leg_index + 1) / len(routes),
                f'Validated waypoint leg {leg_index + 1} of {len(routes)}',
            )
        return plans

    def _tick(self):
        if self.selected_path_method != 'joint_space_waypoints':
            return super()._tick()
        if not self.cube_initialized:
            if (
                not self.cube_init_pending
                and self.cube_pose_client.wait_for_service(timeout_sec=0.0)
            ):
                self.cube_init_pending = True
                self._set_cube_pose(
                    self.cube_spawn_transform, self._cube_initial_response
                )
            return
        if self.state == 'opening':
            self._send_gripper(self.open_value)
            self.state = 'wait_open'
        elif self.state == 'wait_open' and self.pending is None:
            self.state = 'moving_to_pick'
            self._send_arm(self.plans[0])
        elif self.state == 'moving_to_pick' and self.pending is None:
            self.state = 'closing'
            self._send_gripper(self.closed_value)
        elif self.state == 'closing' and self.pending is None:
            if self._set_attached(True):
                self.state = 'transferring'
                self._send_arm(self.plans[1])
        elif self.state == 'transferring' and self.pending is None:
            self.state = 'placing'
        elif (
            self.state == 'placing' and self.pending is None
            and not self.cube_update_pending
        ):
            self.state = 'releasing'
            if self._set_attached(False):
                self._send_gripper(self.open_value)
        elif self.state == 'releasing' and self.pending is None:
            self.state = 'retreating'
            self._send_arm(self.plans[2])
        elif self.state == 'retreating' and self.pending is None:
            self.state = 'completed'
            if self.actual_path_points:
                self._publish_path_marker(
                    self.actual_path_pub,
                    self.actual_path_points,
                    'actual_path',
                    (0.12, 0.95, 0.24, 1.0),
                )
            self._publish_planning_progress(100, 'Pick and place completed')
            self.get_logger().info('Waypoint pick and place completed.')
        if self.attached and not self.cube_update_pending:
            self._update_attached_cube()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPickPlaceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
