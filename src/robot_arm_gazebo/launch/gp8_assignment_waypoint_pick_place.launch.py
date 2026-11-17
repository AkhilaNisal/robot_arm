"""Launch the assignment workcell with joint-space waypoint planning."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package = FindPackageShare('robot_arm_gazebo')
    default_config = PathJoinSubstitution([
        package, 'config', 'gp8_assignment_waypoints.yaml'
    ])
    assignment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            package, 'launch', 'gp8_assignment_pick_place.launch.py'
        ])),
        launch_arguments={
            'pose_input_mode': LaunchConfiguration('pose_input_mode'),
            'plot_trajectory': LaunchConfiguration('plot_trajectory'),
            'pose_config': LaunchConfiguration('pose_config'),
            'planner_executable': 'waypoint_pick_place',
        }.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument('pose_input_mode', default_value='rviz'),
        DeclareLaunchArgument('plot_trajectory', default_value='false'),
        DeclareLaunchArgument('pose_config', default_value=default_config),
        assignment,
    ])
