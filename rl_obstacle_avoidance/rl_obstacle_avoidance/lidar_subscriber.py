import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/lidar/x3',
            self.lidar_callback,
            10)
        self.get_logger().info("LiDAR Subscriber Node Started!")

    def lidar_callback(self, msg):
        # print(f"LiDAR Data \n : msg.ranges={msg.ranges} \n") # debug
        self.get_logger().info(f"LiDAR Data \n : msg.ranges={msg.ranges} \n")

def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
