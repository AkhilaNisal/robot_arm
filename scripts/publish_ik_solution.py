import time
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import JointState

from ik_position import inverse_kinematics


class IKJointPublisher(Node):

    def __init__(self):
        super().__init__('ik_joint_publisher')

        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Desired end-effector position
        target = np.array([0.4, 0.2, 0.7])

        # Calculate IK solution
        self.q_goal, _, _, _ = inverse_kinematics(target)

        # Initial joint configuration
        self.q_start = np.zeros(6)

        # Trajectory duration
        self.duration = 4.0

        self.start_time = time.monotonic()

        self.joint_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6'
        ]

        # Publish at 50 Hz
        self.timer = self.create_timer(
            0.02,
            self.publish_trajectory
        )

        self.get_logger().info(
            'Executing smooth joint-space trajectory...'
        )

    def publish_trajectory(self):

        elapsed = time.monotonic() - self.start_time

        # Normalized time
        tau = min(elapsed / self.duration, 1.0)

        # Quintic time scaling
        s = 10*tau**3 - 15*tau**4 + 6*tau**5

        # Derivative of time scaling
        ds_dt = (
            30*tau**2 - 60*tau**3 + 30*tau**4
        ) / self.duration

        # Joint positions
        q = self.q_start + s * (self.q_goal - self.q_start)

        # Joint velocities
        q_dot = ds_dt * (self.q_goal - self.q_start)

        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = q.tolist()
        msg.velocity = q_dot.tolist()

        self.publisher.publish(msg)

        if tau >= 1.0:
            self.timer.cancel()
            self.get_logger().info(
                'Trajectory completed!'
            )


def main():

    rclpy.init()

    node = IKJointPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()