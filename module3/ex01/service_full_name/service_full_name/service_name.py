import rclpy
from rclpy.node import Node

from service_interfaces.srv import SummFullName


class ServiceNameNode(Node):
    def __init__(self):
        super().__init__('service_name')
        self.srv = self.create_service(SummFullName, 'SummFullName', self.handle_request)
        self.get_logger().info('Service "SummFullName" ready')

    def handle_request(self, request, response):
        parts = [request.surname.strip(), request.name.strip(), request.patronymic.strip()]
        parts = [p for p in parts if p]
        response.full_name = ' '.join(parts)
        self.get_logger().info(f'Request -> {parts} | Response -> "{response.full_name}"')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServiceNameNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()