#!/usr/bin/env python3
"""Foxglove visualization: environment, drone, desired path (Unity/TESSE-style)."""

from __future__ import annotations

import math
import os
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point, PoseArray
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros

# Matches generate_lab4_world.py / traj_vertices_publisher.cpp (Unity Y-up -> ROS Z-up).
LAB4_OBSTACLE_PREFIXES = (
    "Wall_Simple_01",
    "Wall_Arc_90_01",
    "Wall_Arc_90_02",
    "Wall_HalfArc_90_02",
    "Wall_HalfArc_90_03",
    "Column_01_Top",
    "Door_Arch_01",
    "Door_Left_01",
)

LAB4_OBSTACLE_SCALES: dict[str, tuple[float, float, float]] = {
    "Wall_Simple_01": (4.5, 0.15, 3.0),
    "Wall_Arc_90_01": (2.25, 0.15, 3.0),
    "Wall_Arc_90_02": (2.25, 0.15, 3.0),
    "Wall_HalfArc_90_02": (1.2, 0.15, 3.0),
    "Wall_HalfArc_90_03": (1.2, 0.15, 3.0),
    "Column_01_Top": (0.45, 0.45, 3.0),
    "Door_Arch_01": (2.0, 0.12, 2.8),
    "Door_Left_01": (0.12, 1.0, 2.8),
}


def _quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny = 2.0 * (qw * qz + qx * qy)
    cosy = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny, cosy)


def _unity_to_ros_pose(x: float, y: float, z: float,
                       qx: float, qy: float, qz: float, qw: float):
    rx, ry, rz = x, z, y
    yaw = _quat_to_yaw(-qx, -qz, -qy, qw)
    return rx, ry, rz, yaw


def _default_lab4_csv() -> Path | None:
    candidates = [
        Path(os.environ.get("VNAV_LAB4_DATA", "")) / "StreamingAssets/vnav-2020-lab4_static_tfs.csv",
        Path(__file__).resolve().parents[2] / "lab4/data/StreamingAssets/vnav-2020-lab4_static_tfs.csv",
    ]
    for path in candidates:
        if path.is_file():
            return path
    return None


def _load_lab4_obstacle_markers(csv_path: Path) -> list[Marker]:
    seen: set[tuple] = set()
    markers: list[Marker] = []
    marker_id = 100

    with csv_path.open(encoding="utf-8") as handle:
        for line in handle:
            parts = line.strip().split(",")
            if len(parts) < 8:
                continue
            name = parts[0]
            spec_name = next((p for p in LAB4_OBSTACLE_PREFIXES if name.startswith(p)), None)
            if spec_name is None:
                continue

            x, y, z = (float(parts[1]), float(parts[2]), float(parts[3]))
            qx, qy, qz, qw = (float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7]))
            rx, ry, rz, yaw = _unity_to_ros_pose(x, y, z, qx, qy, qz, qw)
            key = (spec_name, round(rx, 2), round(ry, 2), round(rz, 2), round(math.degrees(yaw), 0))
            if key in seen:
                continue
            seen.add(key)

            sx, sy, sz = LAB4_OBSTACLE_SCALES[spec_name]
            z_center = rz + sz / 2.0

            marker = Marker()
            marker.header.frame_id = "world"
            marker.ns = "lab4_obstacles"
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = rx
            marker.pose.position.y = ry
            marker.pose.position.z = z_center
            marker.pose.orientation.z = math.sin(yaw / 2.0)
            marker.pose.orientation.w = math.cos(yaw / 2.0)
            marker.scale.x, marker.scale.y, marker.scale.z = sx, sy, sz
            marker.color.r = 0.62
            marker.color.g = 0.60
            marker.color.b = 0.58
            marker.color.a = 0.35
            markers.append(marker)

    return markers


class Lab3VizPublisher(Node):
    def __init__(self):
        super().__init__('lab3_viz_publisher')
        self.declare_parameter('show_traj_vertices', False)
        self.declare_parameter('show_reference_circle', True)
        self.declare_parameter('lab4_csv_path', '')
        self._show_traj_vertices = self.get_parameter('show_traj_vertices').value
        self._show_reference_circle = self.get_parameter('show_reference_circle').value
        csv_arg = self.get_parameter('lab4_csv_path').value
        self._lab4_csv = Path(csv_arg) if csv_arg else _default_lab4_csv()

        self._pub = self.create_publisher(MarkerArray, '/visuals', 10)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._trail_actual: list[Point] = []
        self._trail_desired: list[Point] = []
        self._trail_len = 600 if self._show_traj_vertices else 300
        self._traj_vertices: list[Point] = []

        self._static = self._build_static_markers()
        if self._show_traj_vertices:
            self.create_subscription(
                PoseArray, '/desired_traj_vertices', self._on_traj_vertices, 10)
        self.create_timer(0.05, self._on_timer)

    def _on_traj_vertices(self, msg: PoseArray):
        self._traj_vertices = [p.position for p in msg.poses]

    def _circle_points(self, radius: float, z: float, n: int = 64) -> list[Point]:
        pts = []
        for i in range(n + 1):
            t = 2.0 * math.pi * i / n
            p = Point()
            p.x = radius * math.sin(t)
            p.y = radius * math.cos(t)
            p.z = z
            pts.append(p)
        return pts

    def _campus_grid(self, stamp) -> Marker:
        """Ground grid for the MIT Unity lab4 campus (~55 m x 35 m)."""
        grid = Marker()
        grid.header.frame_id = 'world'
        grid.header.stamp = stamp
        grid.ns = 'environment'
        grid.id = 1
        grid.type = Marker.LINE_LIST
        grid.action = Marker.ADD
        grid.scale.x = 0.015
        grid.color.r = 0.40
        grid.color.g = 0.42
        grid.color.b = 0.46
        grid.color.a = 0.45

        x_min, x_max = -55, 8
        y_min, y_max = -16, 22
        step = 5
        z = 0.02
        for x in range(x_min, x_max + 1, step):
            a, b = Point(), Point()
            a.x, a.y, a.z = float(x), float(y_min), z
            b.x, b.y, b.z = float(x), float(y_max), z
            grid.points.extend([a, b])
        for y in range(y_min, y_max + 1, step):
            c, d = Point(), Point()
            c.x, c.y, c.z = float(x_min), float(y), z
            d.x, d.y, d.z = float(x_max), float(y), z
            grid.points.extend([c, d])
        return grid

    def _build_static_markers(self) -> list[Marker]:
        stamp = self.get_clock().now().to_msg()
        markers = []

        if self._show_reference_circle:
            path = Marker()
            path.header.frame_id = 'world'
            path.header.stamp = stamp
            path.ns = 'reference_path'
            path.id = 0
            path.type = Marker.LINE_STRIP
            path.action = Marker.ADD
            path.scale.x = 0.06
            path.color.r = 0.2
            path.color.g = 0.75
            path.color.b = 0.35
            path.color.a = 0.85
            path.points = self._circle_points(5.0, 2.0)
            markers.append(path)

        if self._show_traj_vertices:
            markers.append(self._campus_grid(stamp))
            if self._lab4_csv is not None:
                obstacle_markers = _load_lab4_obstacle_markers(self._lab4_csv)
                for marker in obstacle_markers:
                    marker.header.stamp = stamp
                markers.extend(obstacle_markers)
                self.get_logger().info(
                    f'Lab4 campus viz: {len(obstacle_markers)} obstacle markers from {self._lab4_csv}')
        else:
            grid = Marker()
            grid.header.frame_id = 'world'
            grid.header.stamp = stamp
            grid.ns = 'environment'
            grid.id = 1
            grid.type = Marker.LINE_LIST
            grid.action = Marker.ADD
            grid.scale.x = 0.02
            grid.color.r = 0.45
            grid.color.g = 0.45
            grid.color.b = 0.48
            grid.color.a = 0.5
            for i in range(-5, 6):
                a, b = Point(), Point()
                a.x, a.y, a.z = float(i), -5.0, 0.01
                b.x, b.y, b.z = float(i), 5.0, 0.01
                grid.points.extend([a, b])
                c, d = Point(), Point()
                c.x, c.y, c.z = -5.0, float(i), 0.01
                d.x, d.y, d.z = 5.0, float(i), 0.01
                grid.points.extend([c, d])
            markers.append(grid)

            blocks = [
                (8.0, 3.0, 0.0, 2.0, 1.5, 6.0, 0.55, 0.52, 0.48),
                (-7.0, -4.0, 0.0, 3.0, 2.0, 4.0, 0.62, 0.58, 0.54),
                (2.0, -8.0, 0.0, 4.0, 1.2, 3.5, 0.48, 0.50, 0.55),
                (-3.0, 7.5, 0.0, 2.5, 2.5, 5.0, 0.58, 0.55, 0.50),
            ]
            for idx, (x, y, z, sx, sy, sz, r, g, b) in enumerate(blocks, start=2):
                m = Marker()
                m.header.frame_id = 'world'
                m.header.stamp = stamp
                m.ns = 'environment'
                m.id = idx
                m.type = Marker.CUBE
                m.action = Marker.ADD
                m.pose.position.x = x
                m.pose.position.y = y
                m.pose.position.z = z + sz / 2.0
                m.pose.orientation.w = 1.0
                m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
                m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.9
                markers.append(m)

        return markers

    def _lookup_xy(self, target_frame: str) -> Point | None:
        try:
            if not self._tf_buffer.can_transform(
                    'world', target_frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05)):
                return None
            t = self._tf_buffer.lookup_transform('world', target_frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None
        p = Point()
        p.x = t.transform.translation.x
        p.y = t.transform.translation.y
        p.z = t.transform.translation.z
        return p

    def _append_trail(self, trail: list[Point], point: Point) -> None:
        trail.append(point)
        while len(trail) > self._trail_len:
            trail.pop(0)

    def _trail_marker(self, points: list[Point], mid: int, r: float, g: float, b: float,
                      ns: str) -> Marker:
        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.04
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.85
        m.points = list(points)
        return m

    def _body_marker(self, target_frame: str, mid: int, r: float, g: float, b: float,
                     shape: int, scale: tuple[float, float, float]) -> Marker | None:
        try:
            if not self._tf_buffer.can_transform(
                    'world', target_frame, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.05)):
                return None
            t = self._tf_buffer.lookup_transform('world', target_frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            return None

        m = Marker()
        m.header.frame_id = 'world'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'vehicles'
        m.id = mid
        m.type = shape
        m.action = Marker.ADD
        m.pose.position.x = t.transform.translation.x
        m.pose.position.y = t.transform.translation.y
        m.pose.position.z = t.transform.translation.z
        m.pose.orientation = t.transform.rotation
        m.scale.x, m.scale.y, m.scale.z = scale
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 1.0
        return m

    def _on_timer(self):
        stamp = self.get_clock().now().to_msg()
        out = MarkerArray()
        for m in self._static:
            m.header.stamp = stamp
            out.markers.append(m)

        actual = self._lookup_xy('base_link')
        desired = self._lookup_xy('av-desired')
        if actual is not None:
            self._append_trail(self._trail_actual, actual)
        if desired is not None:
            self._append_trail(self._trail_desired, desired)

        if self._trail_actual:
            out.markers.append(
                self._trail_marker(self._trail_actual, 10, 0.25, 0.52, 1.0, 'trail_actual'))
        if self._trail_desired:
            out.markers.append(
                self._trail_marker(self._trail_desired, 11, 1.0, 0.55, 0.15, 'trail_desired'))

        drone = self._body_marker('base_link', 20, 0.35, 0.35, 0.38, Marker.CUBE,
                                (0.45, 0.45, 0.12))
        if drone is not None:
            out.markers.append(drone)

        target = self._body_marker('av-desired', 21, 1.0, 0.55, 0.15, Marker.SPHERE,
                                   (0.25, 0.25, 0.25))
        if target is not None:
            target.color.a = 0.75
            out.markers.append(target)

        if self._traj_vertices:
            gate_path = Marker()
            gate_path.header.frame_id = 'world'
            gate_path.header.stamp = stamp
            gate_path.ns = 'lab4_gates'
            gate_path.id = 30
            gate_path.type = Marker.LINE_STRIP
            gate_path.action = Marker.ADD
            gate_path.scale.x = 0.08
            gate_path.color.r = 0.95
            gate_path.color.g = 0.35
            gate_path.color.b = 0.15
            gate_path.color.a = 0.9
            gate_path.points = list(self._traj_vertices)
            out.markers.append(gate_path)

            for idx, pos in enumerate(self._traj_vertices):
                gate = Marker()
                gate.header.frame_id = 'world'
                gate.header.stamp = stamp
                gate.ns = 'lab4_gates'
                gate.id = 40 + idx
                gate.type = Marker.CUBE
                gate.action = Marker.ADD
                gate.pose.position = pos
                gate.pose.orientation.w = 1.0
                gate.scale.x = 0.35
                gate.scale.y = 0.05
                gate.scale.z = 0.8
                gate.color.r = 0.95
                gate.color.g = 0.2
                gate.color.b = 0.15
                gate.color.a = 0.85
                out.markers.append(gate)

        self._pub.publish(out)


def main():
    rclpy.init()
    node = Lab3VizPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
