from setuptools import find_packages, setup

package_name = 'robot_arm_trajectory'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', [
            'robot_arm_trajectory/config/preview_poses.yaml',
            'robot_arm_trajectory/config/gp8_preview_poses.yaml',
        ]),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'PyYAML'],
    zip_safe=True,
    maintainer='akhila-wedamestrige',
    maintainer_email='wedamestrigean@gmail.com',
    description='Modular joint and Cartesian trajectory generation for ROS 2.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'trajectory_preview = robot_arm_trajectory.trajectory_preview_node:main',
        ],
    },
)
