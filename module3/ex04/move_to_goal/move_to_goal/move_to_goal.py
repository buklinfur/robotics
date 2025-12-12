import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import sys

class TurtleGoToGoal(Node):
    def __init__(self, goal_x, goal_y, goal_theta):
        super().__init__('move_to_goal')
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_theta = goal_theta

        self.pose = None

        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)

        self.timer = self.create_timer(0.02, self.control_loop)

    def pose_callback(self, msg):
        self.pose = msg

    def control_loop(self):
        if self.pose is None:
            return

        dx = self.goal_x - self.pose.x
        dy = self.goal_y - self.pose.y
        dist = math.sqrt(dx*dx + dy*dy)

        msg = Twist()

        if dist > 0.05:
            target_angle = math.atan2(dy, dx)
            angle_error = self.normalize_angle(target_angle - self.pose.theta)

            msg.angular.z = 2.0 * angle_error

            if abs(angle_error) < 0.2:
                msg.linear.x = 1.5 * dist

        else:
            angle_error = self.normalize_angle(self.goal_theta - self.pose.theta)
            if abs(angle_error) > 0.05:
                msg.angular.z = 2.0 * angle_error

        self.cmd_pub.publish(msg)

    @staticmethod
    def normalize_angle(a):
        while a > math.pi:
            a -= 2 * math.pi
        while a < -math.pi:
            a += 2 * math.pi
        return a


def main(args=None):
    if len(sys.argv) != 4:
        print("Usage: ros2 run move_to_goal move_to_goal X Y THETA")
        return

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    theta = float(sys.argv[3])

    rclpy.init(args=args)
    node = TurtleGoToGoal(x, y, theta)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()