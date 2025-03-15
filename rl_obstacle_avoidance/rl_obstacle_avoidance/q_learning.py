import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from threading import Thread

from rl_obstacle_avoidance.drone_mover import DroneMover
from rl_obstacle_avoidance.pose_subscriber import PoseSubscriber
from rl_obstacle_avoidance.lidar_subscriber import LidarSubscriber

class QLearningAgent(Node):
    def __init__(self, pose_node: PoseSubscriber, lidar_node: LidarSubscriber):
        super().__init__('q_learning_agent')

        self.pose_subscriber = pose_node
        self.lidar_subscriber = lidar_node

        # Initialize drone movement controller
        self.mover = DroneMover()

        # Define a 3D Q-table: (discretized x, y, z) x (6 possible actions)
        self.state_space_size = (20, 20, 20)
        self.action_space_size = 6  # Six movement directions
        self.q_table = np.zeros(self.state_space_size + (self.action_space_size,))

        self.learning_rate = 0.1
        self.discount_factor = 0.9
        self.epsilon = 1.0  # Exploration factor

        self.actions = [(2.0, 0.0, 0.0), (-2.0, 0.0, 0.0),
                        (0.0, 2.0, 0.0), (0.0, -2.0, 0.0),
                        (0.0, 2.0, 1.0), (0.0, 0.0, -2.0)]

        # Define start and goal positions in 3D
        self.current_state = (-3, 0, 0)  # Placeholder start position
        self.goal_state = (9, 0, 0)  # Placeholder goal position

        self.create_timer(1.0, self.update_q_learning)

        self.get_logger().info("Q-Learning Agent Initialized in 3D!")

    def get_discrete_state(self, position):
        """ Convert continuous position (x, y, z) to discrete grid state. """

        # Define the known bounds of your world (adjust as needed)
        min_bounds = (-20, -20, 0)  # Minimum world coordinates
        max_bounds = (20, 20, 10)   # Maximum world coordinates

        # Convert position to a discrete grid index
        grid_size = (self.state_space_size[0] - 1, 
                    self.state_space_size[1] - 1, 
                    self.state_space_size[2] - 1)

        x = int((position[0] - min_bounds[0]) / (max_bounds[0] - min_bounds[0]) * grid_size[0])
        y = int((position[1] - min_bounds[1]) / (max_bounds[1] - min_bounds[1]) * grid_size[1])
        z = int((position[2] - min_bounds[2]) / (max_bounds[2] - min_bounds[2]) * grid_size[2])

        discrete_state = (x, y, z)

        self.get_logger().info(f"Discrete State: {discrete_state} from Position: {position}")
        return discrete_state


    def update_q_learning(self):
        """ Q-learning logic: choose action, move, update Q-table. """

        # Read current drone position
        current_pose = self.pose_subscriber.get_pose()
        self.get_logger().info(f"Current Pose Read by Q-Learning: {current_pose}")
        current_state = self.get_discrete_state((current_pose[0], current_pose[1], current_pose[2]))

        # Read LiDAR data for obstacle detection
        min_distance = self.lidar_subscriber.get_min_distance()

        # Assign rewards
        if min_distance < 0.5:  # Obstacle detected
            reward = -10  # Penalty for collision
        elif current_state == self.goal_state:
            reward = 100  # Reward for reaching goal
        else:
            reward = -1  # Small penalty to encourage shortest path

        # Choose action using epsilon-greedy strategy
        if np.random.rand() < self.epsilon:
            action_index = np.random.choice(len(self.actions))  # Explore
        else:
            action_index = np.argmax(self.q_table[current_state])  # Exploit

        action = self.actions[action_index]

        # Move the drone
        self.mover.move(action)

        # Get new state after movement
        new_pose = self.pose_subscriber.get_pose()
        new_state = self.get_discrete_state((new_pose[0], new_pose[1], new_pose[2]))

        # Update Q-table
        old_q_value = self.q_table[current_state][action_index]
        future_max = np.max(self.q_table[new_state])

        new_q_value = (1 - self.learning_rate) * old_q_value + \
                      self.learning_rate * (reward + self.discount_factor * future_max)

        self.q_table[current_state][action_index] = new_q_value

        self.current_state = new_state

        # Reduce exploration over time
        self.epsilon *= 0.99

        # Logging
        self.get_logger().info(f"State: {self.current_state}, Action: {action}, Reward: {reward}")

        # Check if goal is reached
        if self.current_state == self.goal_state:
            self.get_logger().info("Goal reached!")

def main(args=None):
    rclpy.init(args=args)

    # Create Pose and Lidar subscriber nodes
    pose_subscriber_node = PoseSubscriber()
    lidar_subscriber_node = LidarSubscriber()

    # Create Q-Learning Agent node
    q_learning_node = QLearningAgent(pose_subscriber_node, lidar_subscriber_node)

    # Use a MultiThreadedExecutor to spin all nodes in parallel
    executor = MultiThreadedExecutor()
    executor.add_node(pose_subscriber_node)
    executor.add_node(lidar_subscriber_node)
    executor.add_node(q_learning_node)

    try:
        executor.spin()  # Run all nodes concurrently
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup nodes
        pose_subscriber_node.destroy_node()
        lidar_subscriber_node.destroy_node()
        q_learning_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
