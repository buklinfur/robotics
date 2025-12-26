import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time

class LidarStop(Node):
    def __init__(self):
        super().__init__('lidar_stop')
        
        # Паблишер скорости
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Настройка QoS (Best Effort для Gazebo)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Подписка на скан
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            qos)
            
        self.start_time = time.time()
        self.is_moving = False
        self.get_logger().info('Lidar Stop Node Init... Ждем 3 секунды перед стартом.')

    def listener_callback(self, msg):
        current_time = time.time()
        
        # 1. ЗАДЕРЖКА НА СТАРТЕ
        if current_time - self.start_time < 3.0:
            self.get_logger().info(f'Ждем... {3.0 - (current_time - self.start_time):.1f} сек', throttle_duration_sec=1)
            return

        # 2. АНАЛИЗ ЛИДАРА
        # Определяем индексы "переда".
        # Обычно это середина массива для 360-градусного лидара в Gazebo
        ranges = msg.ranges
        mid = len(ranges) // 2
        window = 30 # Смотрим сектор +/- 30 точек
        
        # Берем сектор спереди
        front_ranges = ranges[mid - window : mid + window]
        
        # Фильтруем валидные значения
        valid_ranges = [r for r in front_ranges if r > 0.05 and r < msg.range_max]
        
        # Если вдруг лидар перевернут (0 - это перед), добавим проверку краев
        # (Раскомментируй, если предыдущее не сработает)
        # ranges_start = ranges[:window] + ranges[-window:]
        # valid_ranges_start = [r for r in ranges_start if r > 0.05 and r < msg.range_max]
        # valid_ranges.extend(valid_ranges_start) # Смотрим везде!
        
        min_distance = float('inf')
        if valid_ranges:
            min_distance = min(valid_ranges)

        # 3. ЛОГИКА
        cmd = Twist()
        STOP_DISTANCE = 0.6  # Тормозим поближе, за 60 см (кубик маленький)
        
        if min_distance < STOP_DISTANCE:
            # СТОП
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.get_logger().warning(f'🛑 ПРЕПЯТСТВИЕ! {min_distance:.2f}м')
        else:
            # ЕДЕМ
            cmd.linear.x = 0.3 # Потише едем, дальше будем
            cmd.angular.z = 0.0
            self.get_logger().info(f'🚀 Путь свободен ({min_distance:.2f}м)', throttle_duration_sec=0.5)

        self.publisher_.publish(cmd)


def main():
    rclpy.init()
    node = LidarStop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Останавливаем при выходе
        node.publisher_.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
