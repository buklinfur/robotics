import sys
import rclpy
from rclpy.node import Node

from service_interfaces.srv import SummFullName


class ClientNameNode(Node):
    def __init__(self):
        super().__init__('client_name')
        self.cli = self.create_client(SummFullName, 'SummFullName')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service "SummFullName"...')

    def send_request(self, surname, name, patronymic):
        req = SummFullName.Request()
        req.surname = surname
        req.name = name
        req.patronymic = patronymic
        future = self.cli.call_async(req)
        
        return future


def main(args=None):
    rclpy.init(args=args)
    node = ClientNameNode()

    argv = sys.argv[1:]
    if len(argv) < 3:
        node.get_logger().info('Usage: ros2 run service_full_name client_name <surname> <name> <patronymic>')
        argv = ['Ivanov', 'Ivan', 'Ivanovich']

    surname, name, patronymic = argv[0], argv[1], argv[2]
    future = node.send_request(surname, name, patronymic)

    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        resp = future.result()
        node.get_logger().info(f'Result: "{resp.full_name}"')
    else:
        node.get_logger().error('Service call failed')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()