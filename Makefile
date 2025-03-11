# Use Bash instead of /bin/sh
SHELL=/bin/bash
ROS2_WS=$(shell pwd)

# Command to build the workspace
build:
	@echo "Building ROS 2 package..."
	colcon build --symlink-install

# Command to source the workspace
source:
	@echo "Sourcing ROS 2 workspace..."
	@bash -c "source /opt/ros/jazzy/setup.bash && source $(ROS2_WS)/install/setup.bash && env | grep ROS"

# Command to launch Gazebo with your simulation
run: build
	@echo "Launching Gazebo simulation..."
	@bash -c "source /opt/ros/jazzy/setup.bash && source $(ROS2_WS)/install/setup.bash && ros2 launch rl_obstacle_avoidance gz_sim.launch.py"

# Clean build files
clean:
	@echo "Cleaning build and install directories..."
	rm -rf build/ install/ log/

# Complete rebuild (clean + build + run)
all: clean build run

.PHONY: build source run clean all
