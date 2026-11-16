"""Launch GP8 model, Cartesian trajectory planner, and RViz preview."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Include the shared preview with GP8-specific model and DH config."""
    description_share = FindPackageShare('robot_arm_description')
    kinematics_share = FindPackageShare('robot_arm_kinematics')
    trajectory_share = FindPackageShare('robot_arm_trajectory')
    pose_input_mode = LaunchConfiguration('pose_input_mode')
    preview_launch = PathJoinSubstitution([
        description_share, 'launch', 'trajectory_preview.launch.py'
    ])
    return LaunchDescription([
        DeclareLaunchArgument('preview_rate', default_value='30.0'),
        DeclareLaunchArgument('looping', default_value='false'),
        DeclareLaunchArgument('joint_state_publisher', default_value='false'),
        DeclareLaunchArgument('plot_trajectory', default_value='false'),
        DeclareLaunchArgument('workspace_samples', default_value='12000'),
        DeclareLaunchArgument('pose_input_mode', default_value='rviz'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(preview_launch),
            launch_arguments={
                'model': PathJoinSubstitution([
                    description_share, 'urdf', 'gp8', 'gp8.urdf.xacro'
                ]),
                'robot_config': PathJoinSubstitution([
                    kinematics_share, 'config', 'gp8.yaml'
                ]),
                'pose_config': PathJoinSubstitution([
                    trajectory_share, 'config', 'gp8_preview_poses.yaml'
                ]),
                'pose_input_mode': pose_input_mode,
                'rviz_config': PathJoinSubstitution([
                    description_share, 'rviz', 'trajectory_preview.rviz'
                ]),
                'preview_rate': LaunchConfiguration('preview_rate'),
                'looping': LaunchConfiguration('looping'),
                'joint_state_publisher': LaunchConfiguration(
                    'joint_state_publisher'
                ),
                'plot_trajectory': LaunchConfiguration('plot_trajectory'),
                'workspace_samples': LaunchConfiguration('workspace_samples'),
            }.items(),
        ),
    ])
