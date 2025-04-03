import os
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

def plot_moving_average(window_size=3):
    data_dir = 'rl_obstacle_avoidance/data'
    data_file = os.path.join(data_dir, 'episode_data')
    output_file = os.path.join(data_dir, 'episode_data_ma.png')

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
    print(f"Plot saved to {output_file}")

if __name__ == "__main__":
    plot_moving_average(window_size=10)
