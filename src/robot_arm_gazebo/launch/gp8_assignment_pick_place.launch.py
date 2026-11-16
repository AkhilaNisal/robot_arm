"""Launch the assignment workcell using Box A and Box B poses from YAML."""

import math
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import yaml


def _quaternion_to_rpy(quaternion):
    x, y, z, w = quaternion
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sin_pitch) if abs(sin_pitch) >= 1.0 else math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _model_sdf(name, pose, size, color):
    roll, pitch, yaw = _quaternion_to_rpy(pose['orientation_xyzw'])
    px, py, pz = pose['position']
    sx, sy, sz = size
    return (
        f'<sdf version="1.9"><model name="{name}"><static>true</static>'
        f'<pose>{px} {py} {pz} {roll} {pitch} {yaw}</pose><link name="body">'
        '<collision name="collision"><geometry><box>'
        f'<size>{sx} {sy} {sz}</size></box></geometry></collision>'
        '<visual name="visual"><geometry><box>'
        f'<size>{sx} {sy} {sz}</size></box></geometry>'
        f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse>'
        '</material></visual></link></model></sdf>'
    )


def _launch_setup(context):
    pkg = FindPackageShare('robot_arm_gazebo')
    config_path = LaunchConfiguration('pose_config').perform(context)
    with open(config_path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    objects = config.get('objects', {})
    box_a = objects['box_a']
    box_b = objects['box_b']
    size_b = box_b['size']
    if not (size_b[0] == size_b[1] == size_b[2]):
        raise ValueError('Box B must be a cube; set equal dimensions in YAML.')
    cube_initial = ' '.join(str(value) for value in box_b['position'])

    package_share = Path(get_package_share_directory('robot_arm_gazebo'))
    world_template = package_share / 'worlds' / 'gp8_assignment.sdf'
    world_tree = ET.parse(world_template)
    world_element = world_tree.getroot().find('world')
    if world_element is None:
        raise ValueError(f'World file has no <world> element: {world_template}')
    box_a_model = ET.fromstring(
        _model_sdf('box_a', box_a, box_a['size'], '0.12 0.32 0.72 1')
    ).find('model')
    if box_a_model is None:
        raise ValueError('Could not construct Box A model from YAML.')
    world_element.append(box_a_model)
    with tempfile.NamedTemporaryFile(
        mode='wb', prefix='gp8_assignment_', suffix='.sdf', delete=False
    ) as runtime_world:
        world_tree.write(runtime_world, encoding='utf-8', xml_declaration=True)
        assignment_world = runtime_world.name

    include_pick_place = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, 'launch', 'gp8_pick_place.launch.py'])
        ),
        launch_arguments={
            'pose_input_mode': LaunchConfiguration('pose_input_mode'),
            'pose_config': LaunchConfiguration('pose_config'),
            'world': assignment_world,
            'world_name': 'gp8_assignment',
            'cube_name': 'box_b',
            'gz_partition': 'robot_arm_gp8_assignment',
            'cube_initial_pose': cube_initial,
            'cube_size': str(size_b[0]),
        }.items(),
    )
    return [include_pick_place]


def generate_launch_description():
    pkg = FindPackageShare('robot_arm_gazebo')
    default_config = PathJoinSubstitution([
        pkg, 'config', 'gp8_assignment.yaml'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('pose_input_mode', default_value='config'),
        DeclareLaunchArgument('pose_config', default_value=default_config),
        SetEnvironmentVariable(
            name='GZ_PARTITION', value='robot_arm_gp8_assignment'
        ),
        OpaqueFunction(function=_launch_setup),
    ])
