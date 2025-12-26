#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class TargetBroadcaster(Node):
    """Рассылает TF-фреймы для двух движущихся целей (carrot1, carrot2) и одного статического."""
    def __init__(self):
        super().__init__('target_broadcaster')
        self.radius = self.declare_parameter('radius', 2.0).get_parameter_value().double_value
        self.speed1 = self.declare_parameter('speed1', 1.0).get_parameter_value().double_value
        self.speed2 = self.declare_parameter('speed2', 0.8).get_parameter_value().double_value
        self.direction = self.declare_parameter('direction', 1).get_parameter_value().integer_value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self._on_timer)
        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info("TargetBroadcaster: carrot1, carrot2, static_target")

    def _on_timer(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = (now - self.start_time) * self.direction

        self._broadcast_tf('turtle1', 'carrot1',
                           self.radius * math.sin(self.speed1 * elapsed),
                           self.radius * math.cos(self.speed1 * elapsed))
        self._broadcast_tf('turtle3', 'carrot2',
                           self.radius * math.sin(-self.speed2 * elapsed),
                           self.radius * math.cos(-self.speed2 * elapsed))
        self._broadcast_tf('world', 'static_target', 8.0, 2.0)

    def _broadcast_tf(self, parent: str, child: str, x: float, y: float):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = TargetBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
