import sys
import time
import os
import subprocess
import numpy as np
import json
import rclpy
import matplotlib.pyplot as plt
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from rl_obstacle_avoidance.drone_mover import DroneMover
from rl_obstacle_avoidance.pose_subscriber import PoseSubscriber
from rl_obstacle_avoidance.lidar_subscriber import LidarSubscriber


class QLearningAgent(Node):
    def __init__(self, pose_node: PoseSubscriber, 
                 lidar_node: LidarSubscriber, 
                 learning_rate: float, 
                 discount_factor: float, 
                 state_space_size=(20, 20, 20), 
                 action_space_size=6, 
                 min_bounds=(-20, -20, 0), 
                 max_bounds=(20, 20, 10), 
                 start_state=(-3, 0, 0), 
                 goal_state=(9, 0, 0), 
                 epsilon_start=1.0, 
                 epsilon_end=0.1, 
                 epsilon_decay_episodes=200, 
                 max_steps=25):
        super().__init__('q_learning_agent')

        # ROS2 nodes
        self.pose_subscriber = pose_node
        self.lidar_subscriber = lidar_node
        self.mover = DroneMover()

        # Q-learning settings
        self.state_space_size = state_space_size  # 20x20x20 grid
        self.action_space_size = action_space_size  # 6 actions

        # Hyperparameters
        filename = f"q_table_lr{learning_rate}_df{discount_factor}"
        self.learning_rate = learning_rate
        self.discount_factor = discount_factor

        # Epsilon-greedy exploration settings
        self.epsilon_start = epsilon_start
        self.epsilon_end = epsilon_end
        self.epsilon_decay_episodes = epsilon_decay_episodes
        self.epsilon = self.epsilon_start

        # Actions: (x, y, z) velocities
        self.actions = [(1.5, 0.0, 0.0), (-1.5, 0.0, 0.0),
                        (0.0, 1.5, 0.0), (0.0, -1.5, 0.0),
                        (0.0, 0.0, 1.5), (0.0, 0.0, -1.5)]

        # Environment and episodic learning settings
        self.min_bounds = min_bounds
        self.max_bounds = max_bounds 
        self.max_steps = max_steps  # Maximum steps per episode
        
        # Episodic settings
        self.episode_count = 0  
        self.episode_reward = 0
        self.episode_steps = 0
        
        self.current_state = start_state
        self.goal_state = goal_state
        
        # Initialize collision tracking
        self.collision_count = 0  # Counts collisions within current 200-episode window
        self.collision_history = []  # List of collision counts every 200 episodes
        self.collision_window = 200  # Episodes per tracking window


        # Load Q-table if it exists (resume training), otherwise initialize a new one
        self.load_q_table(filename=filename)
        
        # Timer to update Q-learning process
        self.create_timer(1.0, self.update_q_learning)

        self.get_logger().info("Q-Learning Agent Initialized with Episodic Learning!")

    def load_q_table(self, filename):
        """ Load Q-table and hyperparameters using hyperparameter-based filename. """
        folder = "rl_obstacle_avoidance/q_models/"
        filename_prefix = f"{folder}{filename}"

        q_table_file = f"{filename_prefix}.npy"
        hyperparams_file = f"{filename_prefix}.json"

        if os.path.exists(q_table_file):
            self.q_table = np.load(q_table_file)
            self.get_logger().info(f"Q-table loaded from {q_table_file}.")
        else:
            self.q_table = np.zeros(tuple(self.state_space_size) + (self.action_space_size,))
            self.get_logger().info("No saved Q-table found. Initializing a new one.")

        if os.path.exists(hyperparams_file):
            with open(hyperparams_file, "r") as f:
                hyperparams = json.load(f)
                self.learning_rate = hyperparams["learning_rate"]
                self.discount_factor = hyperparams["discount_factor"]
                self.epsilon = hyperparams["epsilon"]   # Load epsilon from file (resume training)
                self.episode_count = hyperparams["episode_count"] # Resume episode count
                self.state_space_size = tuple(hyperparams["state_space_size"])
                self.action_space_size = hyperparams["action_space_size"]
                self.collision_count = hyperparams.get("collision_count", 0)
                self.collision_history = hyperparams.get("collision_history", [])
                self.collision_window = hyperparams.get("collision_window", 200)


            self.get_logger().info(f"Hyperparameters loaded from {hyperparams_file}.Q table shape: {self.q_table.shape}")
        else:
            self.get_logger().info("No hyperparameter file found. Using default values.")
        
        
    def save_q_table(self):
        """ Save Q-table and hyperparameters with a unique filename based on hyperparameters. """
        folder = "rl_obstacle_avoidance/q_models/"
        filename_prefix = f"{folder}q_table_lr{self.learning_rate}_df{self.discount_factor}"

        np.save(f"{filename_prefix}.npy", self.q_table)

        # Save hyperparameters
        hyperparams = {
            "learning_rate": self.learning_rate,
            "discount_factor": self.discount_factor,
            "epsilon": self.epsilon,
            "episode_count": self.episode_count,
            "state_space_size": self.state_space_size,
            "action_space_size": self.action_space_size,
            "collision_count": self.collision_count,
            "collision_history": self.collision_history,
            "collision_window": self.collision_window

    }

        with open(f"{filename_prefix}.json", "w") as f:
            json.dump(hyperparams, f, indent=4)

        self.get_logger().info(f"Q-table and hyperparameters saved as {filename_prefix}.npy and {filename_prefix}.json")


    def start_new_episode(self):
        """ Resets the Gazebo simulation and initializes a new episode. """
        self.get_logger().info(f"Resetting simulation for Episode {self.episode_count + 1}")
        # Save Q-table before starting a new episode
        self.save_q_table()

        # Save/reset episodic rewards
        episode_data.append(self.episode_reward)
        self.episode_reward = 0

        # Linear epsilon decay
        if self.episode_count < self.epsilon_decay_episodes:
            decay_ratio = self.episode_count / self.epsilon_decay_episodes
            self.epsilon = self.epsilon_start - decay_ratio * (self.epsilon_start - self.epsilon_end)
        else:
            self.epsilon = self.epsilon_end
            
        # Every 200 episodes, save collision count and reset
        if self.episode_count % self.collision_window == 0:
            self.collision_history.append(self.collision_count)
            self.get_logger().info(f"Collision count in last {self.collision_window} episodes: {self.collision_count}")
            self.collision_count = 0  # Reset for next window

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
        
        time.sleep(3)  # Wait for Gazebo to reset
        
        # self.get_logger().info(f"Q-table shape: {self.q_table.shape}")
        self.get_logger().info(f"Episode {self.episode_count} started.")
    

    def get_discrete_state(self, position):
        # self.get_logger().info(f"Rec. Pos {position}")
        rounded_x = round(position[0])
        rounded_y = round(position[1])
        rounded_z = round(position[2])

        x_index = int(rounded_x - self.min_bounds[0]) 
        y_index = int(rounded_y - self.min_bounds[1])
        z_index = int(rounded_z - self.min_bounds[2])

        # self.get_logger().info(f"x={x_index}, y={y_index}, z={z_index}")
        
        clamped_x = max(0, min(x_index, self.state_space_size[0] - 1))
        clamped_y = max(0, min(y_index, self.state_space_size[1] - 1))
        clamped_z = max(0, min(z_index, self.state_space_size[2] - 1))

        # self.get_logger().info(f"DISC: x={clamped_x}, y={clamped_y}, z={clamped_z}")
        return (clamped_x, clamped_y, clamped_z)

    def compute_reward(self, old_position, new_position, goal_position, collision=False):
        """ Reward function: encourages moving toward goal and penalizes collisions. """
        old_distance = np.linalg.norm(np.array(goal_position[:2]) - np.array(old_position[:2]))
        new_distance = np.linalg.norm(np.array(goal_position[:2]) - np.array(new_position[:2]))

        old_distance_3D = np.linalg.norm(np.array(goal_position) - np.array(old_position))
        new_distance_3D = np.linalg.norm(np.array(goal_position) - np.array(new_position))

        # Encourage moving toward the goal
        if new_distance < old_distance:
            reward = 10  # More reward for getting closer
        else:
            reward = -10  # More penalty for moving away

        # Small penalty per step to encourage efficiency
        reward -= 1  

        # Heavy penalty for collisions
        if collision:
            reward -= 200  # Stronger penalty for crashing

        
        # In goal area
        if new_distance < 3:
            reward += 100

        #if new_distance_3D < 2
        #    reward += 50

        # Big reward for reaching the goal
        if new_distance_3D < 0.5:
            reward += 200  

        self.episode_reward += reward # Increment episode reward
        
        # Old printing
        # self.get_logger().info(f"Episode: #{self.episode_count}. Reward: {reward}, Old Distance: {old_distance}, New Distance: {new_distance}, Collision: {collision}")
        
        #New printing
        self.get_logger().info(f"Episode: #{self.episode_count}. Reward: {reward}. 3D Distance: {new_distance_3D}. 2D Distance: {new_distance}. Collision: {collision}")
        return reward

    def update_q_learning(self):
        """ Q-learning logic: choose action, move, update Q-table, and handle episode termination. """
        current_pose = self.pose_subscriber.get_pose()
        current_state = self.get_discrete_state((current_pose[0], current_pose[1], current_pose[2]))

       
        if np.random.rand() < self.epsilon:
            action_index = np.random.choice(len(self.actions))  
        else:
            action_index = np.argmax(self.q_table[current_state])  

        action = self.actions[action_index]
        self.mover.move(action)
        
         # Check for collision using LiDAR
        min_distance = self.lidar_subscriber.get_min_distance()
        collision = min_distance < 0.5
        
        # Check if episode ended
        if collision:
            self.collision_count += 1

        
        time.sleep(0.5)  # Wait for the drone to move

        new_pose = self.pose_subscriber.get_pose()
        new_state = self.get_discrete_state((new_pose[0], new_pose[1], new_pose[2]))

        reward = self.compute_reward(current_pose, new_pose, self.goal_state, collision)
        self.episode_reward += reward

        # Update Q-table using Bellman equation
        old_q_value = self.q_table[current_state][action_index]
        future_max = np.max(self.q_table[new_state])

        self.q_table[current_state][action_index] += self.learning_rate * (
            reward + self.discount_factor * future_max - self.q_table[current_state][action_index]
        )

        # self.get_logger().info(f"Updating Q-table at {current_state}, action {action_index}, old Q: {old_q_value}")
        
        self.current_state = new_state
        self.episode_steps += 1

        # self.epsilon *= 0.99 # Reduce exploration factor over time

       

        if new_state == self.goal_state or collision or self.episode_steps >= self.max_steps:
            self.get_logger().info(f"Episode {self.episode_count} ended (Goal: {new_state == self.goal_state}, Collision: {collision}, Steps: {self.episode_steps}, Reward: {self.episode_reward})")
            self.start_new_episode()
    
    def get_rewards():
        return self.rewards

    def get_filename_episode_reward():
        return self.filename_episode_reward

def save_data(data, file="rl_obstacle_avoidance/data/episode_data"):
    with open(file, "w") as f:
        json.dump(data, f, indent=4)   

def load_existing_data(file="rl_obstacle_avoidance/data/episode_data") -> list[int]:
    try:
        with open(file, "r") as f:
            return json.load(f)
    except FileNotFoundError:
        return [] 
    
def load_config(file_path="rl_obstacle_avoidance/config/parameters.json"):
    """ Load configuration parameters from a JSON file. """
    try:      
        with open(file_path, "r") as f:
            return json.load(f)
    except FileNotFoundError:
        return {}

episode_data = load_existing_data()
parameters = load_config()

def plot_rewards(save_path = "rl_obstacle_avoidance/data/episode_data.png"):
        """
        Plots episode rewards over time.
        Parameters: episode_data (list): each index is an episode number, value is the reward
        """
        episodes = list(range(len(episode_data)))
        rewards = episode_data
        
        # Plot
        plt.figure(figsize=(10, 5))
        plt.plot(episodes, rewards, label="Episode Reward")
        plt.xlabel("Episode")
        plt.ylabel("Reward")
        plt.title("Q-learning Agent Performance Over Time")
        plt.grid(True)
        plt.legend()
        plt.tight_layout()
        plt.savefig(save_path)
        plt.show()

def main(args=None):
    rclpy.init(args=args)

    # Initialize nodes
    pose_subscriber_node = PoseSubscriber()
    lidar_subscriber_node = LidarSubscriber()
    q_learning_node = QLearningAgent(
        pose_node=pose_subscriber_node,
        lidar_node=lidar_subscriber_node,
        learning_rate=parameters["learning_rate"],
        discount_factor=parameters["discount_factor"],
        state_space_size=parameters["state_space_size"],
        action_space_size=parameters["action_space_size"],
        min_bounds=parameters["min_bounds"],
        max_bounds=parameters["max_bounds"],
        start_state=parameters["start_state"],
        goal_state=parameters["goal_state"],
        epsilon_start=parameters["epsilon_start"],
        epsilon_end=parameters["epsilon_end"],
        epsilon_decay_episodes=parameters["epsilon_decay_episodes"],
        max_steps=parameters["max_steps"]
        
    )

    executor = MultiThreadedExecutor()
    executor.add_node(pose_subscriber_node)
    executor.add_node(lidar_subscriber_node)
    executor.add_node(q_learning_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        save_data(data=episode_data)
        plot_rewards()
        pass
    finally:
        pose_subscriber_node.destroy_node()
        lidar_subscriber_node.destroy_node()
        q_learning_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
