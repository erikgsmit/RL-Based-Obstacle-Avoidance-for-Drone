import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class PoseSubscriber(Node):
    def __init__(self, name='pose_subscriber'):
        super().__init__(name)
        self.subscription = self.create_subscription(
            TFMessage,
            "/world/drone_world/dynamic_pose/info",  
            self.pose_callback,
            10)
        self.subscription  # Prevent unused variable warning
        self.current_pose = (-3, 0, 0)  # start position

    def pose_callback(self, msg):
        
        if len(msg.transforms) > 0:
            transform = msg.transforms[0]  # Get the first pose only
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            self.current_pose = (x, y, z)  # Update current position
            
    def get_pose(self):
        """ Returns the current drone position. """
        return self.current_pose