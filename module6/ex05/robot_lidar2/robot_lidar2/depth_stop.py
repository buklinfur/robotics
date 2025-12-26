import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time

class DepthStop(Node):
    def __init__(self):
        super().__init__('depth_stop')
        
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Подписываемся на КАРТИНКУ ГЛУБИНЫ (а не LaserScan)
        # Убедись, что топик правильный! 
        # В bridge ex02 мы делали remapping: /depth_camera -> /image_raw
        # Если ты используешь launch из Ex02, то топик /image_raw
        self.subscription = self.create_subscription(
            Image, 
            '/image_raw', 
            self.listener_callback, 
            qos)
            
        self.start_time = time.time()
        self.get_logger().info('Depth Stop Node Init... Ждем 3 секунды.')

    def listener_callback(self, msg):
        current_time = time.time()
        
        # 1. ЗАДЕРЖКА
        if current_time - self.start_time < 3.0:
            self.get_logger().info(f'Wait: {3.0-(current_time-self.start_time):.1f}s', throttle_duration_sec=1)
            return

        cmd = Twist()
        min_distance = 10.0
        
        try:
            # 2. АНАЛИЗ КАРТИНКИ
            # Проверяем, что это глубина (32FC1 = float distance)
            if msg.encoding == '32FC1':
                # Конвертируем в numpy массив
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                
                # Берем центр картинки (квадратик 20x20 пикселей)
                h, w = cv_image.shape
                center_region = cv_image[h//2-10 : h//2+10, w//2-10 : w//2+10]
                
                # Игнорируем NaN (бесконечность/ошибки)
                valid_depths = center_region[~np.isnan(center_region)]
                
                if len(valid_depths) > 0:
                    min_distance = np.min(valid_depths)
                
                # 3. ЛОГИКА
                STOP_DIST = 1.0
                if min_distance < STOP_DIST:
                    cmd.linear.x = 0.0
                    self.get_logger().warning(f'🛑 WALL at {min_distance:.2f}m')
                else:
                    cmd.linear.x = 0.3
                    self.get_logger().info(f'🚀 Go. Depth: {min_distance:.2f}m', throttle_duration_sec=0.5)
            
            else:
                # Если пришел не тот формат (например rgb8)
                self.get_logger().error(f'Wrong encoding: {msg.encoding}. Need 32FC1 (Depth)!')
                cmd.linear.x = 0.0 # Стоим от греха подальше

        except Exception as e:
            self.get_logger().error(f'Error: {e}')
            
        self.publisher_.publish(cmd)

def main():
    rclpy.init()
    node = DepthStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
