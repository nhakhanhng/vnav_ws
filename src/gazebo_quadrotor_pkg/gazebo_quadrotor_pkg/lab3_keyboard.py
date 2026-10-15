#!/usr/bin/env python3
"""TESSE-style keyboard controls for Lab 3 Gazebo sim: W = arm motors, R = respawn."""

import math
import os
import select
import sys
import termios
import threading
import tty

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose
from mav_msgs.msg import Actuators
from rclpy.node import Node


class Lab3Keyboard(Node):
    def __init__(self):
        super().__init__('lab3_keyboard')

        # Hover-equivalent speed: 4 * cf * w^2 = m * g  (cf matches controller_node.cpp).
        default_idle = math.sqrt(9.81 / (4.0 * 1e-3))
        self.declare_parameter('idle_rotor_speed', default_idle)
        self.declare_parameter('entity_name', 'quadrotor')
        self.declare_parameter('initial_x', 0.0)
        self.declare_parameter('initial_y', 0.0)
        self.declare_parameter('initial_z', 0.5)
        self.declare_parameter('publish_rate_hz', 50.0)

        self._idle_speed = float(self.get_parameter('idle_rotor_speed').value)
        self._entity_name = str(self.get_parameter('entity_name').value)
        self._initial_pose = (
            float(self.get_parameter('initial_x').value),
            float(self.get_parameter('initial_y').value),
            float(self.get_parameter('initial_z').value),
        )
        rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self._armed = False
        self._rotor_pub = self.create_publisher(Actuators, 'rotor_speed_cmds', 10)
        self._delete_cli = self.create_client(DeleteEntity, '/delete_entity')
        self._spawn_cli = self.create_client(SpawnEntity, '/spawn_entity')

        urdf_path = os.path.join(
            get_package_share_directory('gazebo_quadrotor_pkg'), 'urdf', 'quadrotor.urdf')
        with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
            self._urdf_xml = urdf_file.read()

        self.create_timer(1.0 / rate_hz, self._on_timer)
        self._keyboard_thread = threading.Thread(target=self._keyboard_loop, daemon=True)
        self._keyboard_thread.start()

        self.get_logger().info(
            'Lab3 keyboard ready (focus this terminal): '
            'W = arm propellers, R = respawn, Q = quit'
        )

    def _on_timer(self):
        if not self._armed:
            return
        msg = Actuators()
        msg.angular_velocities = [self._idle_speed] * 4
        self._rotor_pub.publish(msg)

    def _respawn(self):
        if not self._delete_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Service /delete_entity unavailable')
            return
        if not self._spawn_cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Service /spawn_entity unavailable')
            return

        req = DeleteEntity.Request()
        req.name = self._entity_name
        future = self._delete_cli.call_async(req)
        future.add_done_callback(self._on_delete_done)

    def _on_delete_done(self, future):
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Delete entity failed: {exc}')
            return
        if not result.success:
            self.get_logger().warn(f'Delete entity: {result.status_message}')

        pose = Pose()
        pose.position.x = self._initial_pose[0]
        pose.position.y = self._initial_pose[1]
        pose.position.z = self._initial_pose[2]
        pose.orientation.w = 1.0

        req = SpawnEntity.Request()
        req.name = self._entity_name
        req.xml = self._urdf_xml
        req.initial_pose = pose
        req.reference_frame = 'world'
        spawn_future = self._spawn_cli.call_async(req)
        spawn_future.add_done_callback(self._on_spawn_done)

    def _on_spawn_done(self, future):
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Spawn failed: {exc}')
            return
        if result.success:
            self.get_logger().info(f'Respawned {self._entity_name} at initial pose')
        else:
            self.get_logger().warn(f'Spawn failed: {result.status_message}')

    def _handle_key(self, key: str):
        if key in ('w', 'W'):
            self._armed = True
            self.get_logger().info(
                f'Propellers ON (idle speed {self._idle_speed:.1f} rad/s each)'
            )
        elif key in ('r', 'R'):
            self._respawn()
        elif key in ('q', 'Q', '\x03'):
            self.get_logger().info('Quitting lab3_keyboard')
            rclpy.shutdown()

    def _keyboard_loop(self):
        tty_fd = None
        old_settings = None
        try:
            if sys.stdin.isatty():
                tty_fd = sys.stdin.fileno()
            else:
                tty_fd = os.open('/dev/tty', os.O_RDONLY)
            old_settings = termios.tcgetattr(tty_fd)
            tty.setcbreak(tty_fd)
            while rclpy.ok():
                ready, _, _ = select.select([tty_fd], [], [], 0.1)
                if not ready:
                    continue
                key = os.read(tty_fd, 1).decode('utf-8', errors='ignore')
                self._handle_key(key)
        except OSError as exc:
            self.get_logger().warn(
                f'Keyboard input unavailable ({exc}). '
                'Run in an interactive terminal: docker exec -it ros2-vnav-lab3-dev ...'
            )
        finally:
            if old_settings is not None and tty_fd is not None:
                termios.tcsetattr(tty_fd, termios.TCSADRAIN, old_settings)
            if tty_fd is not None and tty_fd != sys.stdin.fileno():
                os.close(tty_fd)


def main():
    rclpy.init()
    node = Lab3Keyboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
