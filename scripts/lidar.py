import gz.transport13 as gz_transport
from gz.msgs10.laserscan_pb2 import LaserScan  # The lidar returns a LaserScan message

import time

def lidar_callback(msg):
    print(f"Recieved LiDAR data: {msg.ranges}")
        
def main():
    node = gz_transport.Node()  # Create a node
    topic_lidar = "/lidar" # Topic name
    node.subscribe(LaserScan, topic_lidar, lidar_callback) 
    
    # Wait for shutdown
    try:
      while True:
        time.sleep(0.001)
    except KeyboardInterrupt:
      pass
    print("\nShut down subsriber") 
    
if __name__ == "__main__":
    main()