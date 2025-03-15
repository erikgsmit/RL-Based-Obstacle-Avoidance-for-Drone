import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan, "/lidar/x3", self.lidar_callback, 10)
        self.min_distance = float('inf')

    def lidar_callback(self, msg):
        """ Updates minimum obstacle distance from LiDAR data. """
        self.min_distance = min(msg.ranges)

    def get_min_distance(self):
        """ Returns the closest detected obstacle distance. """
        return self.min_distance