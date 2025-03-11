SHELL=/bin/bash
ROS2_WS=$(shell pwd)

build:
	@echo "Building ROS 2 package..."
	colcon build --symlink-install

source:
	@echo "Sourcing ROS 2 workspace..."
	@source /opt/ros/jazzy/setup.bash && source $(ROS2_WS)/install/setup.bash

run: build source
	@echo "Launching Gazebo simulation..."
	@ros2 launch rl_obstacle_avoidance gz_sim.launch.py

clean:
	@echo "Cleaning build and install directories..."
	rm -rf build/ install/ log/

# Complete rebuild (clean + build + run)
all: clean build run

.PHONY: build source run clean all
