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

def plot_moving_average(filename, window_size=100):
    base_name = filename
    data_dir = 'rl_obstacle_avoidance/data'
    data_file = os.path.join(data_dir, filename)
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

def plot_collisions_combined():
    # Filvägar till json-filerna
    base_path = os.path.join(os.path.dirname(__file__), '..', 'q_models')
    filenames = [
        'q_table_lr0.1_df0.6.json',
        'q_table_lr0.1_df0.9.json',
        'q_table_lr0.3_df0.6.json',
        'q_table_lr0.3_df0.9.json'
    ]

    # Färger eller linjestilar för att skilja kurvorna
    #styles = ['#1abc9c', '#3498db', '#9b59b6', '#34495e']
    styles = ['#0e7490', '#2563eb', '#7c3aed', '#0f172a']


    
    plt.figure(figsize=(10, 6))

    for filename, style in zip(filenames, styles):
        filepath = os.path.join(base_path, filename)
        with open(filepath, 'r') as f:
            data = json.load(f)

        # Ignorera första värdet i collision_history
        collision_data = data['collision_history'][1:]

        # X-axel: 200 till 2000 med steg om 200 (10 datapunkter)
        x = list(range(200, 2001, 200))

        # Lägg till label baserat på filnamnet
        label = filename.replace('q_table_', '').replace('.json', '')

        plt.plot(x, collision_data, style, label=label)

    plt.ylim(25, 175)
    plt.xlim(200, 2000)
    plt.xlabel('Episode')
    plt.ylabel('Collision Count')
    plt.title('Collision History per Experiment')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":

    plot_moving_average('q_table_lr0.3_df0.9_episode_data', window_size=100)
    plot_collisions_combined()
    