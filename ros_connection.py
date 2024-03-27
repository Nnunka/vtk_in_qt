from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSubscriber(Node):
    def __init__(self, visualizer):
        super().__init__('lidar_subscriber')
        self.visualizer = visualizer
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.get_logger().info('Subscriber initialized')

    def listener_callback(self, msg):
        self.visualizer.update_points(msg.ranges, msg.angle_min, msg.angle_increment)
