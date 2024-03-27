from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSubscriber(Node):
    def __init__(self, visualizer):
        # Inicjalizacja węzła ROS2 z nazwą 'lidar_subscriber'.
        super().__init__('lidar_subscriber')
        self.visualizer = visualizer  # Przechowanie referencji do wizualizera.
        
        # Subskrypcja do tematu 'scan' z wiadomościami LaserScan.
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)  # Ustawienie rozmiaru kolejki na 10.
        self.get_logger().info('Subscriber initialized')

    def listener_callback(self, msg):
        # Callback wywoływany przy otrzymaniu nowej wiadomości.
        # Aktualizacja wizualizacji punktów na podstawie danych z wiadomości.
        self.visualizer.update_points(msg.ranges, msg.angle_min, msg.angle_increment)
