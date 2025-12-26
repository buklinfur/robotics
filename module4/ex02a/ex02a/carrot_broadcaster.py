import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations
import math


class CarrotBroadcaster(Node):
    def __init__(self):
        super().__init__("carrot_broadcaster")
        self.br = TransformBroadcaster(self)

        self.declare_parameter("radius", 2.0)
        self.declare_parameter("direction_of_rotation", 1)

        self.radius = float(self.get_parameter("radius").value)
        self.direction = int(self.get_parameter("direction_of_rotation").value)

        if self.direction not in [1, -1]:
            self.get_logger().warn(
                f"Invalid direction_of_rotation={self.direction}, defaulting to 1."
            )
            self.direction = 1

        self.angle = 0.0
        self.timer = self.create_timer(0.05, self.broadcast)

        self.get_logger().info(
            f"Carrot broadcaster started: radius={self.radius}, direction={self.direction}"
        )

    def broadcast(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "turtle1"
        t.child_frame_id = "carrot"

        t.transform.translation.x = self.radius * math.cos(self.angle)
        t.transform.translation.y = self.radius * math.sin(self.angle)
        t.transform.translation.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, self.angle)
        (
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        ) = q
        self.br.sendTransform(t)

        self.angle += self.direction * 0.05
        if self.angle > 2 * math.pi:
            self.angle -= 2 * math.pi


def main():
    rclpy.init()
    node = CarrotBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
