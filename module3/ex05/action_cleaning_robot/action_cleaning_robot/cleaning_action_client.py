import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_cleaning_robot_interfaces.action import CleaningTask
import time

class CleaningActionClient(Node):
    def __init__(self):
        super().__init__('cleaning_action_client')
        self._client = ActionClient(self, CleaningTask, 'CleaningTask')

    def send_goal(self, task_type, area_size=0.0, target_x=0.0, target_y=0.0):
        self.get_logger().info(f'Waiting for action server...')
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available')
            return None

        goal_msg = CleaningTask.Goal()
        goal_msg.task_type = task_type
        goal_msg.area_size = area_size
        goal_msg.target_x = target_x
        goal_msg.target_y = target_y

        self._send_goal_future = self._client.send_goal_async(goal_msg, feedback_callback=self.feedback_cb)
        self._send_goal_future.add_done_callback(self.goal_response_cb)
        return self._send_goal_future

    def goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_cb)

    def feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {fb.progress_percent}% cleaned_points={fb.current_cleaned_points} pos=({fb.current_x:.2f},{fb.current_y:.2f})')

    def get_result_cb(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: success={result.success}, cleaned={result.cleaned_points}, total_distance={result.total_distance:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = CleaningActionClient()

    if len(sys.argv) < 2:
        print("Usage: ros2 run action_cleaning_robot cleaning_action_client <task_type> [area_size/target_x target_y]")
        return

    task_type = sys.argv[1]
    if task_type in ["clean_circle", "clean_square"]:
        area_size = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
        node.send_goal(task_type, area_size=area_size)
    elif task_type == "return_home":
        target_x = float(sys.argv[2]) if len(sys.argv) > 2 else 5.5
        target_y = float(sys.argv[3]) if len(sys.argv) > 3 else 5.5
        node.send_goal(task_type, target_x=target_x, target_y=target_y)
    else:
        print(f"Unknown task_type: {task_type}")
        return

    time.sleep(1)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
