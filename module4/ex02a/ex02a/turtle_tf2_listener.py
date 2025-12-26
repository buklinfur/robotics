import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)
from turtlesim.srv import Spawn
import math


class TurtleFollower(Node):
    def __init__(self):
        super().__init__("turtle_tf2_listener")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Twist, "/turtle2/cmd_vel", 10)
        self.timer = self.create_timer(0.1, self.on_timer)
        self.spawned = False

        # пытаемся создать turtle2
        self.create_timer(1.0, self.try_spawn)

    def try_spawn(self):
        if self.spawned:
            return
        self.cli = self.create_client(Spawn, "/spawn")
        if not self.cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().info("Жду сервис /spawn...")
            return

        req = Spawn.Request()
        req.x, req.y, req.theta, req.name = 5.5, 5.5, 0.0, "turtle2"
        future = self.cli.call_async(req)
        future.add_done_callback(self.on_spawn_response)

    def on_spawn_response(self, future):
        try:
            res = future.result()
            self.get_logger().info(f"Создана черепаха: {res.name}")
        except Exception as e:
            self.get_logger().warn(
                f"Не удалось создать черепаху (возможно, уже есть): {e}"
            )
        self.spawned = True

    def on_timer(self):
        if not self.spawned:
            return
        try:
            trans = self.tf_buffer.lookup_transform(
                "turtle2", "carrot", rclpy.time.Time()
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        msg = Twist()
        msg.angular.z = 4.0 * math.atan2(
            trans.transform.translation.y, trans.transform.translation.x
        )
        msg.linear.x = 0.5 * math.sqrt(
            trans.transform.translation.x**2 + trans.transform.translation.y**2
        )
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = TurtleFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
