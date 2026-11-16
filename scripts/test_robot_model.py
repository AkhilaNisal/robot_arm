import numpy as np

from robot_arm_kinematics.robot_model import RobotModel


# Configuration file
config_path = (
    'src/robot_arm_kinematics/'
    'robot_arm_kinematics/config/generic_robot.yaml'
)

# Load robot
robot = RobotModel(config_path)

print("\nRobot name:")
print(robot.name)

print("\nNumber of DOF:")
print(robot.dof)

print("\nJoint names:")
print(robot.joint_names)

print("\nDH parameters:")
print(robot.dh_parameters)

print("\nBase transformation:")
print(robot.base_transform)

print("\nJoint limits (degrees):")
print(robot.get_joint_limit_summary())

# Test joint configuration
q = np.deg2rad([30, 45, 30, 0, 0, 0])

print("\nTest joint configuration (degrees):")
print(np.rad2deg(q))

print("\nWithin joint limits?")
print(robot.check_joint_limits(q))
