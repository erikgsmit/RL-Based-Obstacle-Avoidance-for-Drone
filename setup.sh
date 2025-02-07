#!/bin/bash


echo "Updating package list..."
sudo apt-get update

echo "Installing necessary dependencies..."
sudo apt-get install curl lsb-release gnupg


echo "Adding correct Gazebo GPG key..."
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "Adding Gazebo repository..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

echo "Updating package list..."
sudo apt update

echo "Installing Gazebo Harmonic..."
if command -v gz > /dev/null; then
    echo "Gazebo is already installed. Skipping installation."
else
    sudo apt-get install gz-harmonic
fi

# Verify installation
echo "Verifying Gazebo installation..."
if command -v gz > /dev/null; then
    echo "✅ Gazebo installed successfully! Run 'gz sim' to start the simulator."
else
    echo "❌ Installation failed. Please check the output above for errors."
    exit 1
fi

