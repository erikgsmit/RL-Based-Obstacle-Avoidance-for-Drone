import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DroneMover(Node):
    def __init__(self):
        super().__init__('drone_mover')
        self.publisher = self.create_publisher(Twist, "/X3/cmd_vel", 10)

    def move(self, action):
        """ Move drone based on Q-learning action. """
        twist_msg = Twist()
        twist_msg.linear.x = action[0] 
        twist_msg.linear.y = action[1] 
        twist_msg.linear.z = action[2] 
        self.publisher.publish(twist_msg)