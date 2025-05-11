# Installation Guide

This document aims to serve as an installation/setup guide to get started with running this project. Please note that this installation guide has these prerequisites

* **Ubuntu 24.04 Noble**
* **Python 3.12**

## Install Gazebo Harmonic:

To install Gazebo Harmonic, copy and paste these commands:

```bash
sudo apt-get update && sudo apt-get install curl lsb-release gnupg
```
Then proceed by entering these commands:

```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

## Install ROS2 Jazzy:

We had to install ROS2 Jazzy from source and here is a guide of how we did it:

First install some dependencies:

```bash
sudo apt install libignition-msgs-dev libignition-transport-dev libsdformat-dev libgz-sim-dev libgz-msgs-dev libgz-transport-dev -y
````

Then create a seperate workspace:
```bash
mkdir -p ~/ros_gz_ws/src
cd ~/ros_gz_ws/src
git clone https://github.com/gazebosim/ros_gz.git -b jazzy
```

Lastly, build the binary using the following command:
```bash
cd ~/ros_gz_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

The build might result in some `ModuleNotFoundError`. In this case just install all these modules. When the build is complete you can run the following command and add it to your `.bashrc`
file.

```bash
source install/setup.bash
echo "source ~/ros_gz_ws/install/setup.bash" >> ~/.bashrc
```

## Run the Project

In order to run our project and test that everything works, you can execute our Makefile:

```bash
make all
```



