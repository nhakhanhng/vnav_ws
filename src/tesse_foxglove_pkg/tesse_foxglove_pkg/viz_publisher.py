#!/usr/bin/env python3
"""Publish Foxglove markers from TESSE odometry and controller trajectory."""

from collections import deque

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.node import Node
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray


class TesseVizPublisher(Node):
    """Draw the measured quadrotor pose and the requested trajectory."""

    def __init__(self):
        super().__init__('tesse_viz_publisher')
        self.declare_parameter('odom_topic', '/current_state')
        self.declare_parameter('desired_topic', '/desired_state')
        self.declare_parameter('visuals_topic', '/tesse/visuals')
        self.declare_parameter('trail_length', 300)

        odom_topic = self.get_parameter('odom_topic').value
        desired_topic = self.get_parameter('desired_topic').value
        visuals_topic = self.get_parameter('visuals_topic').value
        trail_length = max(1, int(self.get_parameter('trail_length').value))

        self._actual = None
        self._desired = None
        self._frame_id = 'world'
        self._actual_trail = deque(maxlen=trail_length)
        self._desired_trail = deque(maxlen=trail_length)
        self._last_time_ns = None

        self._pub = self.create_publisher(MarkerArray, visuals_topic, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(
            MultiDOFJointTrajectoryPoint, desired_topic, self._on_desired, 10)
        self.create_timer(0.1, self._publish)
        self.get_logger().info(
            f'TESSE Foxglove: {odom_topic}, {desired_topic} -> {visuals_topic}')

    def _on_odom(self, msg):
        self._frame_id = msg.header.frame_id or 'world'
        self._actual = msg.pose.pose
        self._actual_trail.append(Point(
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            z=msg.pose.pose.position.z,
        ))

    def _on_desired(self, msg):
        if not msg.transforms:
            return
        transform = msg.transforms[0]
        self._desired = transform
        self._desired_trail.append(Point(
            x=transform.translation.x,
            y=transform.translation.y,
            z=transform.translation.z,
        ))

    def _marker(self, namespace, marker_id, marker_type, stamp):
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _trail_marker(self, namespace, marker_id, points, color, stamp):
        marker = self._marker(namespace, marker_id, Marker.LINE_STRIP, stamp)
        marker.scale.x = 0.05
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 0.9
        marker.points = list(points)
        return marker

    def _publish(self):
        now = self.get_clock().now()
        if self._last_time_ns is not None and now.nanoseconds < self._last_time_ns:
            self._actual_trail.clear()
            self._desired_trail.clear()
        self._last_time_ns = now.nanoseconds
        stamp = now.to_msg()

        markers = MarkerArray()
        if self._actual is not None:
            body = self._marker('tesse_quadrotor', 0, Marker.CUBE, stamp)
            body.pose = self._actual
            body.scale.x = 0.45
            body.scale.y = 0.45
            body.scale.z = 0.12
            body.color.r = 0.25
            body.color.g = 0.52
            body.color.b = 1.0
            body.color.a = 1.0
            markers.markers.append(body)
            markers.markers.append(self._trail_marker(
                'actual_trail', 0, self._actual_trail, (0.25, 0.52, 1.0), stamp))

        if self._desired is not None:
            target = self._marker('desired_pose', 0, Marker.SPHERE, stamp)
            target.pose.position.x = self._desired.translation.x
            target.pose.position.y = self._desired.translation.y
            target.pose.position.z = self._desired.translation.z
            target.scale.x = target.scale.y = target.scale.z = 0.25
            target.color.r = 1.0
            target.color.g = 0.55
            target.color.b = 0.15
            target.color.a = 0.85
            markers.markers.append(target)
            markers.markers.append(self._trail_marker(
                'desired_trail', 0, self._desired_trail, (1.0, 0.55, 0.15), stamp))

        if markers.markers:
            self._pub.publish(markers)


def main():
    rclpy.init()
    node = TesseVizPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
