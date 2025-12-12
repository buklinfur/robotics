import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from action_cleaning_robot_interfaces.action import CleaningTask

class CleaningActionServer(Node):
    def __init__(self):
        super().__init__('cleaning_action_server')
        self._action_server = ActionServer(
            self,
            CleaningTask,
            'CleaningTask',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.pose = None
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self._cancel_requested = False

    def pose_cb(self, msg: Pose):
        self.pose = msg

    def goal_callback(self, goal_request):
        self.get_logger().info(
            f'Received goal: type={goal_request.task_type}, size={goal_request.area_size}, target=({goal_request.target_x},{goal_request.target_y})'
        )
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received')
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    def stop_motion(self):
        self.publish_cmd(0.0, 0.0)

    def publish_cmd(self, linear=0.0, angular=0.0):
        t = Twist()
        t.linear.x = linear
        t.angular.z = angular
        self.cmd_pub.publish(t)

    def wait_for_pose(self, timeout=5.0):
        start = time.time()
        while self.pose is None:
            if time.time() - start > timeout:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def drive_distance(self, distance, speed=1.0):
        if not self.wait_for_pose():
            return 0.0
        start_x = self.pose.x
        start_y = self.pose.y
        travelled = 0.0
        rate = 50.0
        step = 0.05
        while travelled < distance and not self._cancel_requested:
            self.publish_cmd(linear=speed)
            rclpy.spin_once(self, timeout_sec=1.0/rate)
            dx = self.pose.x - start_x
            dy = self.pose.y - start_y
            travelled = math.hypot(dx, dy)
        self.stop_motion()
        return travelled

    def rotate_angle(self, angle, angular_speed=1.0):
        if not self.wait_for_pose():
            return 0.0
        target_theta = (self.pose.theta + angle + math.pi) % (2*math.pi) - math.pi
        rate = 50.0
        while not self._cancel_requested:
            diff = ((target_theta - self.pose.theta + math.pi) % (2*math.pi)) - math.pi
            if abs(diff) < 0.01:
                break
            self.publish_cmd(angular=angular_speed if diff > 0 else -angular_speed)
            rclpy.spin_once(self, timeout_sec=1.0/rate)
        self.stop_motion()
        return abs(angle)

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        result = CleaningTask.Result()
        feedback_msg = CleaningTask.Feedback()

        if not self.wait_for_pose():
            result.success = False
            goal_handle.succeed()
            return result

        total_distance = 0.0
        cleaned_points = 0

        if goal.task_type == 'clean_circle':
            R = float(goal.area_size)
            r = 0.1
            while r <= R and not self._cancel_requested:
                circumference = 2 * math.pi * r
                steps = max(10, int(circumference / 0.1))
                step_len = (2 * math.pi * r) / steps
                for _ in range(steps):
                    if self._cancel_requested:
                        break
                    travelled = self.drive_distance(step_len, speed=0.5)
                    total_distance += travelled
                    cleaned_points += int(travelled / 0.05)
                    self.rotate_angle(2*math.pi/steps)
                    feedback_msg.current_cleaned_points = cleaned_points
                    feedback_msg.progress_percent = int(min(100, r/R*100))
                    feedback_msg.current_x = self.pose.x
                    feedback_msg.current_y = self.pose.y
                    goal_handle.publish_feedback(feedback_msg)
                r += 0.1

        elif goal.task_type == 'clean_square':
            S = float(goal.area_size)
            lane = 0.2
            num_lanes = max(1, int(math.ceil(S / lane)))
            turn_left = True  

            for i in range(num_lanes):
                if self._cancel_requested:
                    break

                travelled = self.drive_distance(S, speed=0.5)
                total_distance += travelled
                cleaned_points += int(travelled / 0.05)

                feedback_msg.current_cleaned_points = cleaned_points
                feedback_msg.progress_percent = int(min(100, (i+1)/num_lanes*100))
                feedback_msg.current_x = self.pose.x
                feedback_msg.current_y = self.pose.y
                goal_handle.publish_feedback(feedback_msg)

                if i < num_lanes - 1:
                    self.rotate_angle(math.pi/2 if turn_left else -math.pi/2)
                    self.drive_distance(lane, speed=0.5)
                    self.rotate_angle(math.pi/2 if turn_left else -math.pi/2)
                    turn_left = not turn_left

        elif goal.task_type == 'return_home':
            tx = float(goal.target_x)
            ty = float(goal.target_y)
            dx = tx - self.pose.x
            dy = ty - self.pose.y
            target_angle = math.atan2(dy, dx)
            self.rotate_angle(target_angle - self.pose.theta)
            distance = math.hypot(dx, dy)
            self.drive_distance(distance, speed=1.0)
            total_distance += distance
            cleaned_points += int(distance / 0.05)
            feedback_msg.current_cleaned_points = cleaned_points
            feedback_msg.progress_percent = 100
            feedback_msg.current_x = self.pose.x
            feedback_msg.current_y = self.pose.y
            goal_handle.publish_feedback(feedback_msg)

        result.success = not self._cancel_requested
        result.cleaned_points = cleaned_points
        result.total_distance = total_distance
        goal_handle.succeed()
        self.get_logger().info(
            f'Task finished: success={result.success}, cleaned={result.cleaned_points}, dist={result.total_distance:.2f}'
        )
        return result


def main(args=None):
    rclpy.init(args=args)
    node = CleaningActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
