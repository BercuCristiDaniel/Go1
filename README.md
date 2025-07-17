# 🐾 Go1 Quadruped Robot Project

This repository contains the full development, simulation, and deployment pipeline for the **Unitree Go1** quadruped robot. The project includes both Gazebo simulation and real-world control systems using ROS.

---

## 📁 Project Structure

```
Go1/
├── Gazebo_simulation/       # ROS + Gazebo simulation environment
├── Real_robot/              # Real robot ROS configuration & control
├── Models/                  # Custom robot/environment models
├── Config/                  # Configuration and parameter files
└── README.md                # Main project readme
```

---

## 🚀 Quick Start

Clone the repository:

```bash
git clone https://github.com/BercuCristiDaniel/Go1.git
cd Go1
```

---

## 🧪 Simulation Setup

All Gazebo and ROS simulation files are in:

📁 [Gazebo_simulation/](Gazebo_simulation/README.md)

Includes:
- Robot model
- SLAM + mapping
- Sensor simulation
- Teleoperation

---

## 🤖 Real Robot Setup

Control the actual Go1 robot using:

📁 [Real_robot/](Real_robot/README.md)

Includes:
- ROS drivers and launch files
- Sensor integration
- Teleop and autonomous navigation

---

## 📦 Requirements

- Ubuntu 20.04+
- ROS Noetic
- Gazebo 11
- Unitree SDK (for real robot)
- Python 3.x

---

## 🔧 Build Instructions

1. Source ROS:

```bash
source /opt/ros/noetic/setup.bash
```

2. Build the workspace:

```bash
cd setup/src/ros_ws
catkin_make
source devel/setup.bash
```

---

## 📄 License

MIT License

---

Maintained by [Cristi Bercu](https://github.com/BercuCristiDaniel)
