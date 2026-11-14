from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    robot_model = LaunchConfiguration('model')

    default_model = PathJoinSubstitution([
        FindPackageShare('robot_arm_description'),
        'urdf',
        'generic',
        'robot_arm.urdf.xacro'
    ])

    robot_description = Command([
        FindExecutable(name='xacro'),
        ' ',
        robot_model
    ])

    return LaunchDescription([

        DeclareLaunchArgument(
            name='model',
            default_value=default_model,
            description='Path to the robot Xacro model'
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description
            }]
        ),

        # Joint State Publisher GUI
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     output='screen'
        # ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )

    ])
