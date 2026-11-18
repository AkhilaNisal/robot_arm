from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    """Start Gazebo, the controlled GP7, a cube, and the pick/place UI."""
    pkg = FindPackageShare('robot_arm_gazebo')
    description_prefix = FindPackagePrefix('robot_arm_description')
    kinematics = FindPackageShare('robot_arm_kinematics')
    launch_share = FindPackageShare('ros_gz_sim')
    pose_mode = LaunchConfiguration('pose_input_mode')
    pose_config = LaunchConfiguration('pose_config')
    world = LaunchConfiguration('world')
    world_name = LaunchConfiguration('world_name')
    cube_name = LaunchConfiguration('cube_name')
    gz_partition = LaunchConfiguration('gz_partition')
    plot_trajectory = LaunchConfiguration('plot_trajectory')
    planner_executable = LaunchConfiguration('planner_executable')
    world_clock = ['/world/', world_name, '/clock']
    world_set_pose = ['/world/', world_name, '/set_pose']
    gz_args = ['-r -v 2 ', world]
    xacro_path = PathJoinSubstitution([pkg, 'urdf', 'gp7_gazebo.urdf.xacro'])
    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', xacro_path,
            ' world_name:=', world_name,
        ]), value_type=str
    )
    config_path = PathJoinSubstitution([pkg, 'config', 'gp7_pick_place.yaml'])
    cube_initial = LaunchConfiguration('cube_initial_pose')
    cube_size = LaunchConfiguration('cube_size')
    rviz_config = PathJoinSubstitution([pkg, 'rviz', 'gp7_pick_place.rviz'])
    spawn_robot = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', world_name, '-topic', 'robot_description', '-name', 'gp7',
                   '-x', '0', '-y', '0', '-z', '0'],
    )
    spawn_cube = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=[
            '-string',
            ['<sdf version="1.9"><model name="', cube_name, '"><pose>', cube_initial, ' 0 0 0</pose><link name="cube"><inertial><mass>0.08</mass><inertia><ixx>2.7e-5</ixx><iyy>2.7e-5</iyy><izz>2.7e-5</izz></inertia></inertial><collision name="collision"><geometry><box><size>', cube_size, ' ', cube_size, ' ', cube_size, '</size></box></geometry><surface><friction><ode><mu>3.0</mu><mu2>3.0</mu2></ode></friction><contact><ode><kp>1000000</kp><kd>100</kd></ode></contact></surface></collision><visual name="visual"><geometry><box><size>', cube_size, ' ', cube_size, ' ', cube_size, '</size></box></geometry><material><ambient>0.9 0.15 0.08 1</ambient><diffuse>0.95 0.2 0.1 1</diffuse></material></visual></link></model></sdf>'],
            '-world', world_name, '-name', cube_name,
        ],
    )
    broadcaster = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager-timeout', '120',
        ],
    )
    arm_controller = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[
            'arm_controller',
            '--param-file', PathJoinSubstitution([
                pkg, 'config', 'controllers.yaml'
            ]),
            '--controller-manager-timeout', '120',
        ],
    )
    gripper_controller = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=[
            'gripper_controller',
            '--param-file', PathJoinSubstitution([
                pkg, 'config', 'controllers.yaml'
            ]),
            '--controller-manager-timeout', '120',
        ],
    )
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([launch_share, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments={'gz_args': gz_args}.items(),
    )
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen', parameters=[{'robot_description': robot_description,
                                      'use_sim_time': True}],
        remappings=[
            ('/clock', world_clock),
        ],
    )
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            world_clock + ['@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            world_set_pose + ['@ros_gz_interfaces/srv/SetEntityPose'],
        ],
        remappings=[
            (world_set_pose,
             ['/gp7_pick_place_sim/world/', world_name, '/set_pose']),
        ],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', rviz_config], parameters=[{'use_sim_time': True}],
        remappings=[('/clock', world_clock)],
    )
    plotter = Node(
        package='robot_arm_trajectory', executable='trajectory_plot',
        output='screen', condition=IfCondition(plot_trajectory),
    )
    planner = Node(
        package='robot_arm_gazebo', executable=planner_executable,
        output='screen',
        parameters=[{
            'robot_config': PathJoinSubstitution([
                kinematics, 'config', 'gp7.yaml'
            ]),
            'pose_config': pose_config,
            'pose_input_mode': pose_mode,
            'world_name': world_name,
            'cube_name': cube_name,
            'use_sim_time': True,
        }],
        remappings=[
            ('/clock', world_clock),
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument('gz_partition', default_value='robot_arm_gp7_pick_place'),
        SetEnvironmentVariable(
            name='GZ_PARTITION', value=gz_partition
        ),
        SetEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=[
                PathJoinSubstitution([description_prefix, 'share']),
                ':',
                EnvironmentVariable(
                    'GZ_SIM_RESOURCE_PATH', default_value=''
                ),
            ],
        ),
        DeclareLaunchArgument('pose_input_mode', default_value='config'),
        DeclareLaunchArgument('pose_config', default_value=config_path),
        DeclareLaunchArgument('world_name', default_value='pick_place'),
        DeclareLaunchArgument('cube_name', default_value='gp7_demo_cube'),
        DeclareLaunchArgument('plot_trajectory', default_value='false'),
        DeclareLaunchArgument(
            'planner_executable', default_value='pick_place'
        ),
        DeclareLaunchArgument('cube_initial_pose', default_value='0.62 0.0 0.625'),
        DeclareLaunchArgument('cube_size', default_value='0.045'),
        DeclareLaunchArgument('world', default_value=PathJoinSubstitution([
            pkg, 'worlds', 'pick_place.sdf'
        ])),
        gz, bridge, rsp,
        RegisterEventHandler(OnProcessExit(
            target_action=spawn_robot,
            on_exit=[TimerAction(period=3.0, actions=[broadcaster])],
        )),
        RegisterEventHandler(OnProcessExit(target_action=broadcaster,
                                           on_exit=[arm_controller])),
        RegisterEventHandler(OnProcessExit(target_action=arm_controller,
                                           on_exit=[gripper_controller])),
        RegisterEventHandler(OnProcessExit(target_action=gripper_controller,
                                           on_exit=[spawn_cube])),
        RegisterEventHandler(OnProcessExit(target_action=spawn_cube,
                                           on_exit=[rviz, plotter, planner])),
        spawn_robot,
    ])
