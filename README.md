# FAST-LIO2-NeuPAN-navigation-system

# isaacsim_turtlebot_burger

This repository documents the integration of **TurtleBot3** with **NVIDIA Isaac Sim 5.0** using **ROS 2 Humble** on **Ubuntu 22.04**.  
The project focuses on building a complete simulation-based mobile robot pipeline including **sensor simulation, TF synchronization, SLAM, and autonomous navigation (Nav2)**.

---

## 1. Project Overview

### Objective
- Run TurtleBot3 in Isaac Sim 5.0
- Bridge simulated sensors to ROS2 Humble
- Perform SLAM using `slam_toolbox`
- Execute autonomous navigation using `Nav2`
- Maintain a consistent TF tree (`map → odom → base_footprint → sensors`)

### Key Features
- Isaac Sim LiDAR simulation
- ROS2 Bridge for sensor, TF, and control topics
- Differential drive control via `/cmd_vel`
- SLAM and map saving
- Nav2-based global & local planning

---

## 2. System Architecture

Isaac Sim 5.0

├─ TurtleBot3 USD

├─ LiDAR

├─ Physics & Time Step

└─ ROS2 Bridge

↓

ROS2 Humble

├─ TF (map → odom → base_link)

├─ /scan, /imu, /odom

├─ slam_toolbox

└─ Nav2 (planner, controller, bt_navigator)


---

## 3. Environment

| Component | Version |
|---------|--------|
| OS | Ubuntu 22.04 |
| Simulator | NVIDIA Isaac Sim 5.0 |
| ROS | ROS2 Humble |
| Robot | TurtleBot3 (Burger) |
| GPU | NVIDIA RTX 5070 |
