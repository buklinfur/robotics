#!/usr/bin/env python3
import math
import sys
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener, TransformException
from turtle_multi_target.msg import CurrentTarget

class SmartTurtleController(Node):
    def __init__(self):
        super().__init__('smart_turtle_controller')
        self.threshold = self.declare_parameter('switch_distance', 1.0).get_parameter_value().double_value
        self.targets = ['carrot1', 'carrot2', 'static_target']
        self.active = 0
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub_cmd = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.pub_target = self.create_publisher(CurrentTarget, '/current_target', 10)
        self.timer = self.create_timer(0.1, self.loop)
        self.get_logger().info(f" Контроллер → первая цель: {self.targets[self.active]}")

    def loop(self):
        target_name = self.targets[self.active]
        if self._key_pressed('n'):
            self._next_target(manual=True)
            target_name = self.targets[self.active]

        try:
            tf = self.tf_buffer.lookup_transform('turtle2', target_name, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().debug(f"TF нет для {target_name}: {e}")
            return

        dx = tf.transform.translation.x
        dy = tf.transform.translation.y
        dist = math.hypot(dx, dy)

        info = CurrentTarget()
        info.target_name = target_name
        info.target_x = dx
        info.target_y = dy
        info.distance_to_target = dist
        self.pub_target.publish(info)

        twist = Twist()
        twist.linear.x = dist
        twist.angular.z = 2.0 * math.atan2(dy, dx)
        self.pub_cmd.publish(twist)

        if dist < self.threshold:
            self._next_target()

    def _next_target(self, manual=False):
        self.active = (self.active + 1) % len(self.targets)
        self.get_logger().info(f"{'повтор' if manual else 'готово'} Новая цель: {self.targets[self.active]}")

    def _key_pressed(self, key: str) -> bool:
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.readline().strip().lower() == key
        return False

def main():
    rclpy.init()
    node = SmartTurtleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
