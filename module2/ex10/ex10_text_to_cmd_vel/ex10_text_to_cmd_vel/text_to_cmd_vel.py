import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

def make_move(linear_x: float = 0.0, angular_z: float = 0.0) -> Twist:
    t = Twist()
    t.linear.x = float(linear_x)
    t.linear.y = 0.0
    t.linear.z = 0.0
    t.angular.x = 0.0
    t.angular.y = 0.0
    t.angular.z = float(angular_z)
    return t

class TextToCmdVel(Node):
    def __init__(self) -> None:
        super().__init__('text_to_cmd_vel')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(String, 'cmd_text', self.cb_cmd, 10)
        self.get_logger().info('text_to_cmd_vel started, waiting for commands on "cmd_text"')

        self.linear_speed = 1.0
        self.angular_speed = 1.5

    def cb_cmd(self, msg: String) -> None:
        raw = msg.data if isinstance(msg.data, str) else ''
        cmd = raw.strip().lower()
        twist = None

        if cmd == 'move_forward':
            twist = make_move(linear_x=self.linear_speed, angular_z=0.0)
        elif cmd == 'move_backward':
            twist = make_move(linear_x=-self.linear_speed, angular_z=0.0)
        elif cmd == 'turn_left':
            twist = make_move(linear_x=0.0, angular_z=self.angular_speed)
        elif cmd == 'turn_right':
            twist = make_move(linear_x=0.0, angular_z=-self.angular_speed)
        else:
            twist = make_move(0.0, 0.0)
            self.get_logger().warn(f'Unknown command: "{raw}". Publishing stop.')

        self.pub.publish(twist)
        self.get_logger().info(f'Cmd "{raw}" -> Twist(linear.x={twist.linear.x}, angular.z={twist.angular.z})')


def main(args=None):
    rclpy.init(args=args)

    node = TextToCmdVel()

    try:
        rclpy.spin(node) # "it “spins” the node so its callbacks are called"
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down text_to_cmd_vel')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()