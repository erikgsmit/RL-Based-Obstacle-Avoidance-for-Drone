import os
import sys
import json
import numpy as np
import matplotlib.pyplot as plt


"""
Created on Thu Oct 12 14:00:00 2023

This script computes and plots the moving average of episode data from a JSON file.
The moving average is calculated using a specified window size.
The script reads episode data from a JSON file, computes the moving average,
and saves the plot as a PNG file.
"""

def plot_moving_average(filename, window_size=15):
    base_name = os.path.splitext(filename)[0]
    data_dir = 'rl_obstacle_avoidance/data'
    data_file = os.path.join(data_dir, 'episode_data')
    output_file = os.path.join(data_dir, f'{base_name}_ma.png')

    try:
        with open(data_file, 'r') as f:
            data = json.load(f)
    except Exception as e:
        print(f"Failed to load episode data: {e}")
        return

    if not isinstance(data, list) or not all(isinstance(x, (int, float)) for x in data):
        print("Invalid data format: expected a list of numbers.")
        return

    if len(data) < window_size:
        print("Not enough data points for the specified window size.")
        return

    # Compute moving average
    data_np = np.array(data)
    moving_avg = np.convolve(data_np, np.ones(window_size)/window_size, mode='valid')

    plt.figure(figsize=(10, 5))
    plt.plot(data, label='Original Data', alpha=0.4)
    plt.plot(range(window_size - 1, len(data)), moving_avg, label=f'{window_size}-Point Moving Average', color='red')
    plt.xlabel('Episode')
    plt.ylabel('Score / Reward')
    plt.title('Episode Data with Moving Average')
    plt.legend()
    plt.grid(True)

    plt.savefig(output_file)
    plt.close()
    print(f"Moving average plot saved to {output_file}")


def plot_collisions(filename):
    data_dir = 'rl_obstacle_avoidance/q_models'
    data_file = os.path.join(data_dir, filename)

    base_name = os.path.splitext(filename)[0]
    output_file = f"rl_obstacle_avoidance/data/{base_name}_collisions.png"

    try:
        with open(data_file, 'r') as f:
            data = json.load(f)
    except Exception as e:
        print(f"Failed to load data from {filename}: {e}")
        return

    collision_history = data.get('collision_history', [])
    collision_window = data.get('collision_window', 200)

    if not collision_history:
        print("No collision history found in the file.")
        return

    # Plotting
    plt.figure(figsize=(10, 5))
    plt.plot(range(2, len(collision_history) + 1), collision_history[1:], marker='o', linestyle='-')
    plt.xlabel(f'{collision_window}-Episode Window')
    plt.ylabel('Collisions')
    plt.title('Collisions per Window Over Training')
    plt.grid(True)
    plt.tight_layout()
    plt.savefig(output_file)
    plt.close()
    print(f"Collision plot saved to {output_file}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python plot_script.py <q_table_json_filename>")
        print("Example: python plot_script.py q_table_lr0.3_df0.6.json")
        sys.exit(1)

    filename = sys.argv[1]
    
    plot_collisions(filename)
    plot_moving_average(filename, window_size=15)
