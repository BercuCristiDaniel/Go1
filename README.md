# 🐾 Go1 Quadruped Robot Project

This repository contains a complete control framework for the Unitree **Go1 quadruped robot**, developed during a research internship. It includes low-level and high-level control systems built on top of ROS Noetic, with simulation support using Gazebo via Docker, and deployment to real hardware via UDP communication.

---

## 🔍 Project Focus

- ❌ No navigation or SLAM
- ✅ Model Predictive Control (MPC) for body motion
- ✅ Feedback linearization
- ✅ Low-level joint-space control using inverse kinematics and torque computation
- ✅ Step generation using 7th-order Bézier curves
- ✅ Real-time execution with ROS nodes
- ✅ Tested both in Gazebo simulation (Docker-based) and real robot

---

## 🧱 Project Structure

```
Go1/
├── Gazebo_simulation/                  # Simulation setup using Docker + Gazebo + ROS
│   └── Simulation/
│       └── setup/
│           └── src/ros_ws/            # Catkin workspace
├── Real_robot/                        # Real robot ROS interface and control nodes
```

---

## 🚀 Quick Start

### Clone the repository

```bash
git clone https://github.com/BercuCristiDaniel/Go1.git
cd Go1
```

---

## 🧪 Run Simulation (Docker)

Build and run the Docker container:

```bash
cd Gazebo_simulation/Simulation/docker
docker build -t go1_sim .
docker run --rm -it --net=host --privileged -v /dev:/dev -v $(pwd)/../setup:/home/dev/go1 go1_sim
```

Inside the container:

```bash
cd ~/go1/src/ros_ws
catkin_make
source devel/setup.bash
roslaunch go1_simulation simulation.launch
```


---

## 🧠 Control Architecture

### High-Level

- Simplified 2D kinematic torso model
- MPC with linear or nonlinear formulation (CasADi)
- Feedback linearization to reduce complexity
- Circle and square trajectory tracking

### Low-Level

- Body-to-leg velocity mapping
- Bézier curve step generation
- Inverse kinematics
- Torque control via Euler-Lagrange model

---

## 📦 Dependencies

- ROS Noetic
- Python 3.x
- CasADi (for MPC)
- Eigen / NumPy / SciPy
- Gazebo 11 (via Docker)
- Unitree SDK (for real robot)


---

Maintained by [Cristi Bercu](https://github.com/BercuCristiDaniel)
