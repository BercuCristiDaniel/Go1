# 🐾 Go1 Quadruped Simulation & Control

This repository implements **Model Predictive Control (MPC)** for high-level velocity commands and a **low-level torque control** of the Unitree Go1 robot in Gazebo simulation.

---

## 🔧 Docker Workflow

Run these from your **PC terminal**:

### ▶️ Create Docker Container
```bash
make create_con
```

### 🚪 Access Docker Container
```bash
make access_con
```

### ⬆️ Sync source code (PC → Docker)
```bash
make update_con_src
```

### ⬇️ Sync source code (Docker → PC)
```bash
make sync_src
```

---

## 🧠 High-Level Control (MPC-Based Velocity Tracking)

This layer computes linear/nonlinear MPC to send velocity commands (`/cmd_vel`) to the robot.

### ✅ Run Steps

1. **Launch the simulator:**
```bash
roslaunch unitree_gazebo robot_simulation.launch rname:=go1 rviz:=false
```

2. **Start robot base (keyboard) controller:**
```bash
rosrun unitree_guide junior_ctrl
```
Then press in sequence:
```
1 → 2 → 5
```

3. **Run high-level MPC controller:**
```bash
rosrun high_level_control main_linear         # Linear MPC
# or
rosrun high_level_control main_nonlinear      # Nonlinear MPC
```

---

## 🦿 Full-Body Control (Inverse Dynamics with Bézier Gait)

This controller executes low-level torque control based on Bézier foot trajectories and inverse dynamics via Pinocchio. It runs **after** high-level is up and publishing velocity.

### ✅ Run Steps

1. **Launch the simulator (same as above):**
```bash
roslaunch unitree_gazebo robot_simulation.launch rname:=go1 rviz:=false
```

2. **Launch full-body control:**
```bash
roslaunch full_body_control locomotion_control.launch
```

This will:
- Run a rise phase to lower the robot
- Wait for velocity commands from high-level controller
- Execute a trotting gait using torque control

---

## 📁 Output Logs

Several controllers log data as `.mat` files for offline analysis (MATLAB or Python):
- `state_xi`: robot actual state evolution
- `pos_ref`: reference trajectory
- `control_inputs`: velocity commands or torques
- `tau`: (if available) torque values per leg

---

## 📝 Notes

- Compatible with Unitree Go1 URDF and `unitree_ros_to_real`.
- Uses `pinocchio`, `CasADi`, and custom Bézier gait generator.
- Tune `Q`, `R`, constraints (`L`, `b`), or PD gains (`Kx`, `Bx`) to adjust controller behavior.

---
