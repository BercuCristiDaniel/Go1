# 🤖 Real Robot Control – Go1

This README explains how to run the control framework on the **physical Unitree Go1 quadruped** robot. It includes instructions for both **high-level** and **low-level** modes using the Unitree SDK and ROS Noetic.

---

## 📡 Network Connection

Before launching any ROS node, connect your PC to the **Go1 Wi-Fi network**.

```bash
UnitreeRobotics-*
```

This will establish the UDP communication channel needed for both high-level and low-level modes.

---

## 🔧 High-Level Mode

This is the **default mode** the robot boots into.

### ✅ Steps:

1. Power on the robot.
2. Connect your PC to Go1 Wi-Fi.
3. On your PC, run:

```bash
roslaunch unitree_legged_real high_level.launch
```

This will launch the ROS nodes to publish high-level control commands and subscribe to sensor/state data.

---

## ⚙️ Low-Level Mode (Full Control)

Low-level mode gives **raw motor access**, and should only be used with the robot **safely suspended** (legs off the ground!).

### 🕹 Remote Button Sequence:

Press the following buttons on the robot’s controller **in order**:

1. `L2 + A`  
2. `L2 + A` (again)  
3. `L2 + B`  
4. `L1 + L2 + Start`


### 🧠 Launch ROS:

Once in low-level mode, launch the same ROS interface:

```bash
roslaunch unitree_legged_real low_level.launch
```

This will now initialize the topics for **low-level raw motor control**.

---

## 🚨 Safety First

- **⚠️ Low-level mode bypasses internal safety loops**
- Always suspend the robot when testing low-level code
- Make sure topics like `/low_cmd` and `/low_state` are publishing correctly
- Monitor joint torques and positions via `rqt` or custom diagnostics

---

## 🧪 Recommended Testing Flow

1. Start with high-level control to verify ROS setup.
2. Switch to low-level mode with robot elevated.
3. Incrementally test torque commands and IK-based movement.
4. Only walk on the ground after full validation.

---


Maintained by [Cristi Bercu](https://github.com/BercuCristiDaniel)
