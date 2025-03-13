import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.subscription = self.create_subscription(
            TFMessage,
            "/world/drone_world/dynamic_pose/info",  
            self.pose_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def pose_callback(self, msg):
        
        if len(msg.transforms) > 0:
            transform = msg.transforms[0]  # Get the first pose only
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            print(f"Drone Position: x={x}, y={y}, z={z} \n")  # debug
            self.get_logger().info(f"Drone Position: x={x}, y={y}, z={z}")
            

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()