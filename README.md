# RL-Based Obstacle Avoidance for Drone Simulation in Gazebo Harmonic

## 🚀 Project Overview

This project aims to simulate a drone with LIDAR in **Gazebo Harmonic** for **reinforcement learning-based obstacle avoidance**. The simulation environment is built using **Gazebo Harmonic**.

---

## 📁 Project Structure

```
RL-Based-Obstacle-Avoidance-for-Drone/
│── gazebo/                        # Gazebo simulation models & environments
│   ├── models/
│   │   ├── parrot_bebop_2/         # Prebuilt Bebop 2 drone model
│   ├── environments/
│   │   ├── environment.sdf         # Simulation world file
│
│── src/                            # Python RL & drone control code
|   ├── drone_control.py           # Drone control
│   ├── drone_env.py                # Gym RL environment wrapper
│   ├── train_rl.py                 # RL training script
│   ├── test_rl.py                  # RL model evaluation

│── README.md                       # Project documentation
│── requirements.txt                 # Python dependencies
│── setup.sh                         # Setup script for dependencies
│── .gitignore                       # Ignore unnecessary files
```

---

## ⚙️ Setup & Installation

### **1️⃣ Install Dependencies**
This project is currently implemented for **Ubuntu 24.04** in mind, which can be downloaded on Microsoft stroe alongside WSL

Run the following commands to install all necessary system dependencies:

```bash
sudo apt-get update
sudo apt-get install curl lsb-release gnupg
```

Install Gazebo by running the following command:


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
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/gazebo/models
```

To make this change permanent (might add .env file later):

```bash
echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/gazebo/models' >> ~/.bashrc
source ~/.bashrc
```

---

## 🏗️ Running the Simulation

At the moment you can try to run the simulation by running the follwoing command

```bash
gz sim gazebo/environments/drone_world.sdf 
```

This will load the simulation world and within this simulation world you should be able to spawn a drone by searching for **Resource Spawner**

If you wish to test the drone movement, you can run the following command in a seperate terminal window

```bash
gz topic -t /X3/gazebo/command/motor_speed --msgtype gz.msgs.Actuators -p 'velocity:[700, 700, 700, 700]'
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



