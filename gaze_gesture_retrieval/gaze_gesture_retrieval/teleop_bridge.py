"""Hybrid gaze + gesture teleoperation bridge.

When the system is in ``NAVIGATION`` mode this node combines the latest
gesture (forward / backward / stop) and gaze direction (left / right) into
a ``geometry_msgs/Twist`` published on ``/cmd_vel``.

Gesture -> linear motion mapping (default):
    open_palm  -> forward
    fist       -> stop
    point      -> backward
    others     -> stop

Gaze state -> angular velocity:
    left   -> +angular_speed
    right  -> -angular_speed
    other  -> 0.0
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String


GESTURE_TO_LINEAR = {
    'open_palm': +1.0,
    'fist': 0.0,
    'point': -1.0,
    'peace': 0.0,
    'thumbs_up': 0.0,
    'grab': 0.0,
    'none': 0.0,
}


class TeleopBridge(Node):
    def __init__(self) -> None:
        super().__init__('teleop_bridge')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.18)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('publish_rate_hz', 20.0)

        self._lin_speed = float(self.get_parameter('linear_speed').value)
        self._ang_speed = float(self.get_parameter('angular_speed').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        topic = self.get_parameter('cmd_vel_topic').value

        self._pub = self.create_publisher(Twist, topic, 10)
        self.create_subscription(String, '/gesture/label', self._on_gesture, 10)
        self.create_subscription(String, '/gaze/state', self._on_gaze, 10)
        self.create_subscription(String, '/system/mode', self._on_mode, 10)

        self._gesture = 'none'
        self._gaze = 'none'
        self._mode = 'NAVIGATION'
        self._last_msg_t = time.monotonic()

        self.create_timer(1.0 / max(1.0, rate), self._tick)
        self.get_logger().info(
            f'teleop_bridge: publishing on {topic}, '
            f'linear={self._lin_speed} m/s, angular={self._ang_speed} rad/s'
        )

    def _on_gesture(self, msg: String) -> None:
        self._gesture = msg.data
        self._last_msg_t = time.monotonic()

    def _on_gaze(self, msg: String) -> None:
        self._gaze = msg.data

    def _on_mode(self, msg: String) -> None:
        self._mode = msg.data

    def _tick(self) -> None:
        twist = Twist()
        if self._mode != 'NAVIGATION':
            self._pub.publish(twist)
            return

        # Safety: if perception stops sending labels, stop the robot.
        if time.monotonic() - self._last_msg_t > 1.0:
            self._pub.publish(twist)
            return

        lin_dir = GESTURE_TO_LINEAR.get(self._gesture, 0.0)
        twist.linear.x = lin_dir * self._lin_speed
        if self._gaze == 'left':
            twist.angular.z = +self._ang_speed
        elif self._gaze == 'right':
            twist.angular.z = -self._ang_speed
        else:
            twist.angular.z = 0.0
        self._pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
