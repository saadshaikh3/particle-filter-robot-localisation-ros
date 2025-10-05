# 📍 Particle Filter-Based Robot Localisation (ROS Simulation)

## 🧭 Overview
This project implements a **particle filter-based localisation system** for a mobile robot in a **ROS simulation environment**.  
The goal is to enable the robot to accurately determine its **position and orientation** within a known map, even when affected by **sensor noise** and **odometry uncertainty**.  

The **particle filter (Monte Carlo localisation)** method is used to represent the robot’s belief about its position as a cloud of particles, each with an associated weight indicating how well it matches sensor readings. Over time, the particle cloud converges around the most probable robot pose.

---

## ⚙️ Core Features
- **Probabilistic Localisation:** Implements a Monte Carlo particle filter for pose estimation.  
- **Noise Modelling:** Adds Gaussian noise to particle states for uncertainty simulation.  
- **Resampling Mechanism:** Refocuses the particle cloud on highly weighted, probable states.  
- **Real-Time Pose Estimation:** Continuously updates robot position and orientation from particle distribution.  
- **Visualisation:** Displays particle convergence in **RViz** using the `/particlecloud` topic.

---

## 🚀 How to Run the Simulation

### 1. Launch ROS Core
```bash
cd catkin_ws/
source devel/setup.bash
roscore
```
### 2. Launch the Simulation World
```bash
cd catkin_ws/src/socspioneer/data
rosrun stage_ros stageros lgfloor.world
```
### 3. Launch Keyboard Teleoperation
```bash
roslaunch socspioneer keyboard_teleop.launch
```
### 4. Launch the Map Server
```bash
cd catkin_ws/src/socspioneer/data
rosrun map_server map_server lgfloor.yaml
```
### 5. Open RViz for Visualisation
```bash
rosrun rviz rviz
```
### 6. Run the Particle Filter Localisation Node
```bash
rosrun pf_localisation0 node.py
```

---

## 🧩 Implementation Summary
|Function |	Purpose |
|--------|---------|
|initialise_particle_cloud() |	Creates an initial particle set with Gaussian noise to represent uncertainty in the robot’s position.|
|update_particle_cloud() |	Updates particle weights based on sensor readings and performs resampling to refine estimates.|
|estimate_pose() |	Calculates the robot’s estimated position and orientation as the weighted mean of all particles.|
------

## 🧠 Testing and Results
The robot was manually navigated using keyboard teleoperation through various map regions.
The RViz particle cloud (published on /particlecloud) gradually converged around the robot’s true location, confirming accurate localisation despite odometry drift and sensor noise.

Key outcomes:

- Stable convergence of particles near true robot pose.

- Effective handling of non-linear motion and noisy data.

- Real-time visualisation confirmed reliable localisation performance.

## 🧩 Technical Details
Platform: ROS Noetic

Simulator: Stage + RViz

Algorithm: Particle Filter (Monte Carlo Localisation)

Topics Used: /particlecloud, /odom, /scan, /cmd_vel

Language: Python (ROS Node) 