#!/usr/bin/env python3
"""Bridge Gazebo Classic quadrotor physics to Lab 3 controller topics."""

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Wrench
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

# Match controller_node.cpp physical constants.
CF = 1e-3
CD = 1e-5
D = 0.3
A = CF * D / math.sqrt(2.0)

F2W = np.array([
    [CF, CF, CF, CF],
    [A, A, -A, -A],
    [-A, A, A, -A],
    [CD, -CD, CD, -CD],
])


class QuadrotorBridge(Node):
    def __init__(self):
        super().__init__('quadrotor_bridge')
        self.declare_parameter('odom_topic', '/quadrotor/ground_truth/odometry')
        self.declare_parameter('wrench_topic', '/quadrotor/wrench')

        odom_topic = self.get_parameter('odom_topic').value
        wrench_topic = self.get_parameter('wrench_topic').value

        self._tf_br = TransformBroadcaster(self)
        self._wrench_pub = self.create_publisher(Wrench, wrench_topic, 10)
        self._state_pub = self.create_publisher(Odometry, '/current_state', 10)

        self.create_subscription(Actuators, 'rotor_speed_cmds', self._on_cmd, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

        self.get_logger().info(
            f'Bridge active: {odom_topic} -> /current_state, '
            f'rotor_speed_cmds -> {wrench_topic}'
        )

    def _on_odom(self, msg: Odometry):
        self._state_pub.publish(msg)

        tf = TransformStamped()
        tf.header = msg.header
        tf.header.frame_id = msg.header.frame_id or 'world'
        tf.child_frame_id = 'base_link'
        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation
        self._tf_br.sendTransform(tf)

    def _on_cmd(self, msg: Actuators):
        if len(msg.angular_velocities) < 4:
            return

        speeds = np.array(msg.angular_velocities[:4], dtype=float)
        signed_square = speeds * np.abs(speeds)
        wrench = F2W @ signed_square

        out = Wrench()
        out.force.x = 0.0
        out.force.y = 0.0
        out.force.z = float(wrench[0])
        out.torque.x = float(wrench[1])
        out.torque.y = float(wrench[2])
        out.torque.z = float(wrench[3])
        self._wrench_pub.publish(out)


def main():
    rclpy.init()
    node = QuadrotorBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
