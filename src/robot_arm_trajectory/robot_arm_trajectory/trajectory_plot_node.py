"""Optional Matplotlib window for validated trajectory derivatives."""

import json

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


class TrajectoryPlotNode(Node):
    """Plot joint positions, velocities, accelerations, and jerk samples."""

    def __init__(self):
        super().__init__('robot_arm_trajectory_plot')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.figure, self.axes = plt.subplots(4, 1, sharex=True, figsize=(11, 9))
        self.figure.suptitle('Planned Joint Trajectory')
        self.subscription = self.create_subscription(
            String,
            '/trajectory_preview/trajectory_data',
            self._plot_message,
            qos,
        )
        self.gui_timer = self.create_timer(0.05, self._pump_gui)
        plt.show(block=False)

    def _plot_message(self, message):
        """Replace the plot with the latest validated trajectory."""
        try:
            data = json.loads(message.data)
            time = np.asarray(data['time'], dtype=float)
            names = data['joint_names']
            positions = np.asarray(data['positions'], dtype=float)
            velocities = np.asarray(data['velocities'], dtype=float)
            accelerations = np.asarray(data['accelerations'], dtype=float)
            raw_jerks = data.get('jerks')
            jerks = (
                None if raw_jerks is None
                else np.asarray(raw_jerks, dtype=float)
            )
            if positions.shape != (len(time), len(names)):
                raise ValueError('Position sample dimensions do not match.')
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().error(f'Invalid trajectory plot data: {error}')
            return

        for axis in self.axes:
            axis.clear()
            axis.grid(True, alpha=0.3)
        samples = (
            np.rad2deg(positions), velocities, accelerations, jerks
        )
        labels = (
            'Position (deg)', 'Velocity (rad/s)',
            'Acceleration (rad/s²)', 'Jerk (rad/s³)',
        )
        for axis, values, label in zip(self.axes, samples, labels):
            axis.set_ylabel(label)
            if values is None:
                axis.text(
                    0.5, 0.5, 'Jerk is unavailable for this timing method',
                    ha='center', va='center', transform=axis.transAxes,
                )
                continue
            for index, name in enumerate(names):
                axis.plot(time, values[:, index], label=name)
        self.axes[0].legend(ncol=min(6, len(names)), fontsize='small')
        self.axes[-1].set_xlabel('Time (s)')
        self.figure.tight_layout()
        self.figure.canvas.draw_idle()
        self.get_logger().info('Updated trajectory plots.')

    @staticmethod
    def _pump_gui():
        """Keep the Matplotlib window responsive while ROS spins."""
        if plt.get_fignums():
            plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        plt.close('all')


if __name__ == '__main__':
    main()
