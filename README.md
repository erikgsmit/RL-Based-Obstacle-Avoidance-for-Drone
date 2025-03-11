# RL-Based Obstacle Avoidance for Drone Simulation in Gazebo Harmonic

## 🚀 Project Overview

This project aims to simulate a drone with LIDAR in **Gazebo Harmonic** for **reinforcement learning-based obstacle avoidance**. The simulation environment is built using **Gazebo Harmonic** and integrated with ROS 2 Jazzy. This is built for **Ubuntu Noble 24.04**. 

---

## 📁 Project Structure

TO BE IMPLEMENTED

---

## ⚙️ Setup & Installation

### **1️⃣ Install Dependencies**
This project is currently implemented for **Ubuntu 24.04** in mind, which can be downloaded on Microsoft stroe alongside WSL

Run the following commands to install all necessary system dependencies:

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
```

Install Gazebo Harmonic by running the following command or follow their [installation guide](https://gazebosim.org/docs/harmonic/install_ubuntu/):


```bash
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic
```

### **2️⃣ Verify Gazebo Installation**

```bash
gz sim --verbose
```


### **3️⃣ Set Up Gazebo Resource Path**

```bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:~/RL-Based-Obstacle-Avoidance-for-Drone/install/rl_obstacle_avoidance/share/rl_obstacle_avoidance/models

```

To make this change permanent you can add it to your `.bashrc` file (might add .env file later):

### **4️⃣ Install ROS 2 Jazzy**

The next step is to install ROS 2 Jazzy. Please refer to this [installtion guide for ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).

---

## 🏗️ Running the Simulation

First you must build the ROS package, and for this you might have to install **colcon**. After installing colcon, plese run the following command to build the package:

```bash
colcon build --symlink-install
source install/setup.bash
```

Then you can launch the simulation using the following command:

```bash
ros2 launch rl_obstacle_avoidance gz_sim.launch.py
```

This will load the simulation world.

If you wish to test the drone movement, you can run the following command in a seperate terminal window

```bash
gz topic -t <drone_topic> --msgtype gz.msgs.Actuators -p 'velocity:[700, 700, 700, 700]'
```

To view all the available topics enter the following command:

```bash
gz topic -l
```




## 📜 License


---

## 📖 Resources

Here is a list of useful resources:
- [Gazebo Guide](https://gazebosim.org/docs/latest/getstarted/)
- [RL with Gazebo and ROS](https://github.com/vmayoral/basic_reinforcement_learning/blob/master/tutorial7/README.md)


## 🙌 Contributors

- **Erik Smit** - [erikgsmit](https://github.com/erikgsmit)
- **Hugo Larsson Wilhelmsson** - [hugoahus](https://github.com/hugoahus)



