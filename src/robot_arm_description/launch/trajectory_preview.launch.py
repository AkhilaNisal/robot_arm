"""Launch the robot model, RViz, and the validated trajectory preview."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start one robot_state_publisher, RViz, and the preview node."""
    description_share = FindPackageShare('robot_arm_description')
    kinematics_share = FindPackageShare('robot_arm_kinematics')
    trajectory_share = FindPackageShare('robot_arm_trajectory')
    robot_config = LaunchConfiguration('robot_config')
    pose_config = LaunchConfiguration('pose_config')
    pose_input_mode = LaunchConfiguration('pose_input_mode')
    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rviz_config')
    preview_rate = LaunchConfiguration('preview_rate')
    marker_publish_rate = LaunchConfiguration('marker_publish_rate')
    looping = LaunchConfiguration('looping')
    use_sim_time = LaunchConfiguration('use_sim_time')
    joint_state_publisher = LaunchConfiguration('joint_state_publisher')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=PathJoinSubstitution([
                description_share,
                'urdf',
                'generic',
                'robot_arm.urdf.xacro',
            ]),
            description='Robot xacro model.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                description_share, 'rviz', 'trajectory_preview.rviz'
            ]),
            description='RViz configuration file.',
        ),
        DeclareLaunchArgument(
            'robot_config',
            default_value=PathJoinSubstitution([
                kinematics_share, 'config', 'generic_robot.yaml'
            ]),
            description='Robot kinematics and joint-limit configuration.',
        ),
        DeclareLaunchArgument(
            'pose_config',
            default_value=PathJoinSubstitution([
                trajectory_share, 'config', 'preview_poses.yaml'
            ]),
            description='Start and goal pose YAML configuration.',
        ),
        DeclareLaunchArgument(
            'pose_input_mode',
            default_value='rviz',
            description='Pose source: editable rviz markers or yaml file.',
        ),
        DeclareLaunchArgument('preview_rate', default_value='30.0'),
        DeclareLaunchArgument('marker_publish_rate', default_value='1.0'),
        DeclareLaunchArgument('looping', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'joint_state_publisher', default_value='false',
            description='Run joint_state_publisher_gui for manual joint control.',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='trajectory_preview_robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command([
                    FindExecutable(name='xacro'), ' ', model
                ]),
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(joint_state_publisher),
        ),
        Node(
            package='robot_arm_trajectory',
            executable='trajectory_preview',
            name='robot_arm_trajectory_preview',
            output='screen',
            parameters=[{
                'robot_config': robot_config,
                'pose_config': pose_config,
                'pose_input_mode': pose_input_mode,
                'preview_rate': preview_rate,
                'marker_publish_rate': marker_publish_rate,
                'looping': looping,
                'use_sim_time': use_sim_time,
            }],
        ),
    ])
