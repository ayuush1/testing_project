"""Mode manager.

Tracks the active interaction mode (NAVIGATION vs RETRIEVAL) and broadcasts it
on ``/system/mode``. The user toggles between modes with a configurable
gesture (default: ``thumbs_up``).

A configurable cooldown prevents accidental re-toggling when a transient
``thumbs_up`` reading appears mid-transition (for example as the user
folds the thumb down to make a closed fist for the lock-on gesture).
"""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ModeManager(Node):
    def __init__(self) -> None:
        super().__init__('mode_manager')
        self.declare_parameter('start_mode', 'NAVIGATION')
        self.declare_parameter('mode_switch_gesture', 'thumbs_up')
        self.declare_parameter('mode_switch_cooldown_s', 2.0)

        self._mode = str(self.get_parameter('start_mode').value).upper()
        self._switch = str(self.get_parameter('mode_switch_gesture').value)
        self._cooldown = float(self.get_parameter('mode_switch_cooldown_s').value)
        self._last_switch_t = 0.0

        self._pub = self.create_publisher(String, '/system/mode', 10)
        self.create_subscription(String, '/gesture/stable', self._on_gesture, 10)
        self.create_timer(0.5, self._heartbeat)

        self.get_logger().info(
            f'mode_manager: start={self._mode}, '
            f'switch_gesture="{self._switch}", '
            f'cooldown={self._cooldown}s'
        )

    def _on_gesture(self, msg: String) -> None:
        if msg.data != self._switch:
            return
        now = time.monotonic()
        if (now - self._last_switch_t) < self._cooldown:
            self.get_logger().debug(
                f'Ignoring switch gesture during {self._cooldown}s cooldown')
            return
        self._mode = 'RETRIEVAL' if self._mode == 'NAVIGATION' else 'NAVIGATION'
        self._last_switch_t = now
        self._pub.publish(String(data=self._mode))
        self.get_logger().info(f'Mode switched -> {self._mode}')

    def _heartbeat(self) -> None:
        self._pub.publish(String(data=self._mode))


def main(args=None):
    rclpy.init(args=args)
    node = ModeManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
