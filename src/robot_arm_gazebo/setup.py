from setuptools import find_packages, setup

package_name = 'robot_arm_gazebo'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/gp7_pick_place.launch.py',
            'launch/gp7_assignment_pick_place.launch.py',
            'launch/gp7_assignment_waypoint_pick_place.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/controllers.yaml', 'config/gp7_pick_place.yaml',
            'config/gp7_assignment.yaml',
            'config/gp7_assignment_waypoints.yaml',
        ]),
        ('share/' + package_name + '/worlds', [
            'worlds/pick_place.sdf', 'worlds/gp7_assignment.sdf',
        ]),
        ('share/' + package_name + '/urdf', ['urdf/gp7_gazebo.urdf.xacro']),
        ('share/' + package_name + '/rviz', ['rviz/gp7_pick_place.rviz']),
    ],
    install_requires=['setuptools', 'numpy', 'PyYAML'],
    zip_safe=True,
    maintainer='akhila-wedamestrige',
    maintainer_email='wedamestrigean@gmail.com',
    description='Gazebo Sim pick and place demo for the GP7 robot arm.',
    license='MIT',
    entry_points={'console_scripts': [
        'pick_place = robot_arm_gazebo.pick_place_node:main',
        'waypoint_pick_place = robot_arm_gazebo.waypoint_pick_place_node:main',
    ]},
)
