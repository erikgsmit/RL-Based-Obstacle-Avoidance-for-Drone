import sys
import time
import os
import subprocess
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from rl_obstacle_avoidance.drone_mover import DroneMover
from rl_obstacle_avoidance.pose_subscriber import PoseSubscriber
from rl_obstacle_avoidance.lidar_subscriber import LidarSubscriber

class QLearningAgent(Node):
    def __init__(self, pose_node: PoseSubscriber, lidar_node: LidarSubscriber):
        super().__init__('q_learning_agent')

        self.pose_subscriber = pose_node
        self.lidar_subscriber = lidar_node
        self.mover = DroneMover()

        # Q-learning settings
        self.state_space_size = (20, 20, 20)  # Discretized 3D grid
        self.action_space_size = 6  # Six movement directions
        self.q_table_file = "q_table.npy"  # File to save/load Q-table

        self.learning_rate = 0.5   # Increase learning speed
        self.discount_factor = 0.8  # Reduce future reward dependence slightly
        self.epsilon = 0.9         # Reduce randomness gradually    


        self.actions = [(2.0, 0.0, 0.0), (-2.0, 0.0, 0.0),
                        (0.0, 2.0, 0.0), (0.0, -2.0, 0.0),
                        (0.0, 0.0, 2.0), (0.0, 0.0, -2.0)]

        # Environment and episodic learning settings
        self.min_bounds = (-20, -20, 0)  
        self.max_bounds = (20, 20, 10)  
        self.max_steps = 25  # Maximum steps per episode
        self.episode_count = 0  
        self.episode_reward = 0
        self.episode_steps = 0
        
        self.current_state = (-3, 0, 0)  # Placeholder start position
        self.goal_state = (9, 0, 0)  # Placeholder goal position


        # Load Q-table if it exists, otherwise initialize a new one
        self.load_q_table()


        # Timer to update Q-learning process
        self.create_timer(1.0, self.update_q_learning)

        self.get_logger().info("Q-Learning Agent Initialized with Episodic Learning!")

    def load_q_table(self):
        """ Load Q-table from file if available, otherwise initialize a new one. """
        if os.path.exists(self.q_table_file):
            self.q_table = np.load(self.q_table_file)
            self.get_logger().info("Q-table loaded from file.")
        else:
            self.q_table = np.zeros(self.state_space_size + (self.action_space_size,))
            self.get_logger().info("No saved Q-table found. Initializing a new one.")

        # DEBUG: Check if Q-table actually loaded
        self.get_logger().info(f"Loaded Q-table:\n {self.q_table}")


        
        
    def save_q_table(self):
        """ Save Q-table to file after each episode. """
        np.save(self.q_table_file, self.q_table)
        self.get_logger().info(f"Q-table saved to file. Current Q-table:\n {self.q_table}")


    def start_new_episode(self):
        """ Resets the Gazebo simulation and initializes a new episode. """
        self.get_logger().info(f"Resetting simulation for Episode {self.episode_count + 1}")
        # Save Q-table before starting a new episode
        self.save_q_table()

        # Reset Gazebo simulation
        reset_command = [
            "gz", "service", "-s", "/world/drone_world/control",
            "--reqtype", "gz.msgs.WorldControl",
            "--reptype", "gz.msgs.Boolean",
            "--timeout", "3000",
            "--req", "reset: {all: true}"
        ]

        try:
            subprocess.run(reset_command, check=True)
            self.get_logger().info("Gazebo simulation reset successfully.")
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to reset Gazebo: {e}")

        

        # Reset episode parameters
        self.current_state = (-3, 0, 0)  
        self.goal_state = (9, 0, 0)  
        self.episode_steps = 0
        self.episode_reward = 0
        self.episode_count += 1
        
        time.sleep(5)  # Wait for Gazebo to reset
        
        # DEBUG: Check LiDAR distance after reset
        min_distance = self.lidar_subscriber.get_min_distance()
        self.get_logger().info(f"🚨 LiDAR Check After Reset: {min_distance}m")
        self.get_logger().info(f"Q-table shape: {self.q_table.shape}")

        self.get_logger().info(f"Episode {self.episode_count} started.")
        
        

    def get_discrete_state(self, position):
        """ Convert continuous position (x, y, z) to a discrete grid state using normalization. """
        norm_x = (position[0] - self.min_bounds[0]) / (self.max_bounds[0] - self.min_bounds[0])
        norm_y = (position[1] - self.min_bounds[1]) / (self.max_bounds[1] - self.min_bounds[1])
        norm_z = (position[2] - self.min_bounds[2]) / (self.max_bounds[2] - self.min_bounds[2])

        # self.get_logger().info(f"Normalized Position: ({norm_x}, {norm_y}, {norm_z}),  Position: {position}")
        return (
            max(0, min(int(norm_x * (self.state_space_size[0] - 1)), self.state_space_size[0] - 1)),
            max(0, min(int(norm_y * (self.state_space_size[1] - 1)), self.state_space_size[1] - 1)),
            max(0, min(int(norm_z * (self.state_space_size[2] - 1)), self.state_space_size[2] - 1))
        )

    def compute_reward(self, old_position, new_position, goal_position, collision=False):
        """ Reward function: encourages moving toward goal and penalizes collisions. """
        old_distance = np.linalg.norm(np.array(goal_position) - np.array(old_position))
        new_distance = np.linalg.norm(np.array(goal_position) - np.array(new_position))

        # Encourage moving toward the goal
        if new_distance < old_distance:
            reward = 10  # More reward for getting closer
        else:
            reward = -10  # More penalty for moving away

        # Small penalty per step to encourage efficiency
        reward -= 1  

        # Heavy penalty for collisions
        if collision:
            reward -= 100  # Stronger penalty for crashing

        # Big reward for reaching the goal
        if new_distance < 0.5:
            reward += 200  

        self.get_logger().info(f"Episode: #{self.episode_count}. Reward: {reward}, Old Distance: {old_distance}, New Distance: {new_distance}, Collision: {collision}")
        return reward

    def update_q_learning(self):
        """ Q-learning logic: choose action, move, update Q-table, and handle episode termination. """
        current_pose = self.pose_subscriber.get_pose()
        current_state = self.get_discrete_state((current_pose[0], current_pose[1], current_pose[2]))

        # Check for collision using LiDAR
        min_distance = self.lidar_subscriber.get_min_distance()
        collision = min_distance < 0.5

        if np.random.rand() < self.epsilon:
            action_index = np.random.choice(len(self.actions))  
        else:
            action_index = np.argmax(self.q_table[current_state])  

        action = self.actions[action_index]
        self.mover.move(action)
        
        time.sleep(0.5)  # Wait for the drone to move

        new_pose = self.pose_subscriber.get_pose()
        new_state = self.get_discrete_state((new_pose[0], new_pose[1], new_pose[2]))

        reward = self.compute_reward(current_pose, new_pose, self.goal_state, collision)
        self.episode_reward += reward

        # Update Q-table using Bellman equation
        old_q_value = self.q_table[current_state][action_index]
        future_max = np.max(self.q_table[new_state])

        new_q_value = (1 - self.learning_rate) * old_q_value + \
                      self.learning_rate * (reward + self.discount_factor * future_max)

        self.get_logger().info(f"Updating Q-table at {current_state}, action {action_index}, old Q: {old_q_value}")

        self.q_table[current_state][action_index] = new_q_value
        
        self.get_logger().info(f"Inserted: {self.q_table[current_state][action_index]} at {current_state}, action {action_index}")
        self.current_state = new_state
        self.episode_steps += 1

        self.epsilon *= 0.99

        # Check if episode ended
        if new_state == self.goal_state or collision or self.episode_steps >= self.max_steps:
            self.get_logger().info(f"Episode {self.episode_count} ended (Goal: {new_state == self.goal_state}, Collision: {collision}, Steps: {self.episode_steps}, Reward: {self.episode_reward})")
            self.start_new_episode()
            
def main(args=None):
    rclpy.init(args=args)
    pose_subscriber_node = PoseSubscriber()
    lidar_subscriber_node = LidarSubscriber()
    q_learning_node = QLearningAgent(pose_subscriber_node, lidar_subscriber_node)

    executor = MultiThreadedExecutor()
    executor.add_node(pose_subscriber_node)
    executor.add_node(lidar_subscriber_node)
    executor.add_node(q_learning_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pose_subscriber_node.destroy_node()
        lidar_subscriber_node.destroy_node()
        q_learning_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
