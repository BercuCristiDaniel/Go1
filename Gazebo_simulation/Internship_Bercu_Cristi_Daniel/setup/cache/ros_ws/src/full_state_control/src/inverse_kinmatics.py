#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

# === Go1 IK/Trajectory Module ===
L1 = 0.213  # thigh length (m)
L2 = 0.213  # calf length (m)

def go1_leg_inverse_kinematics(foot_pos):
    """
    Computes q = [q1, q2, q3] using the exact formulas from the document
    foot_pos: [x, y, z] position of the foot in the leg (hip) frame
    returns: [q1, q2, q3] in radians
    """
    x, y, z = foot_pos

    # q1: abduction/adduction
    q1 = np.arctan2(y, z)

    # q2: hip pitch
    norm_p = np.linalg.norm([x, y, z])
    cos_arg = norm_p / (L1**2 + L2**2)
    cos_arg = np.clip(cos_arg, -1.0, 1.0)
    first_term = np.arccos(cos_arg)

    second_term = np.arctan2(-x, np.sqrt(y**2 + z**2))
    q2 = first_term + second_term

    # q3: knee
    sin_arg = (-x / L2) - np.sin(q2)
    sin_arg = np.clip(sin_arg, -1.0, 1.0)
    q3 = np.arcsin(sin_arg) - q2

    return np.array([q1, q2, q3])

def jacobian(q):
    """
    Compute analytical Jacobian matrix for Go1 leg.
    q: [q1, q2, q3] in radians
    Returns: 3x3 Jacobian matrix
    """
    q1, q2, q3 = q
    l1, l2 = L1, L2
    q23 = q2 + q3

    # Partial derivatives
    dx_dq1 = 0
    dx_dq2 = -l1 * np.cos(q2) - l2 * np.cos(q23)
    dx_dq3 = -l2 * np.cos(q23)

    dy_dq1 = l1 * np.cos(q2) * np.cos(q1) + l2 * np.cos(q23) * np.cos(q1)
    dy_dq2 = -l1 * np.sin(q2) * np.sin(q1) - l2 * np.sin(q23) * np.sin(q1)
    dy_dq3 = -l2 * np.sin(q23) * np.sin(q1)

    dz_dq1 = -l1 * np.cos(q2) * np.sin(q1) - l2 * np.cos(q23) * np.sin(q1)
    dz_dq2 = -l1 * np.sin(q2) * np.cos(q1) - l2 * np.sin(q23) * np.cos(q1)
    dz_dq3 = -l2 * np.sin(q23) * np.cos(q1)

    J = np.array([
        [dx_dq1, dx_dq2, dx_dq3],
        [dy_dq1, dy_dq2, dy_dq3],
        [dz_dq1, dz_dq2, dz_dq3]
    ])

    return J


def compute_joint_trajectory(foot_trajectory, dt=0.01):
    q_des = []
    q_dot_des = []
    for i in range(len(foot_trajectory)):
        pos = foot_trajectory[i]
        q = go1_leg_inverse_kinematics(pos)
        q_des.append(q)
        if 0 < i < len(foot_trajectory) - 1:
            dp = (foot_trajectory[i + 1] - foot_trajectory[i - 1]) / (2 * dt)
        elif i == 0:
            dp = (foot_trajectory[i + 1] - foot_trajectory[i]) / dt
        else:
            dp = (foot_trajectory[i] - foot_trajectory[i - 1]) / dt
        J = jacobian(q)
        dq = J @ dp
        q_dot_des.append(dq)
    return np.array(q_des), np.array(q_dot_des)

# === Main for Testing ===
def create_dummy_foot_trajectory(num_points=100):
    x_vals = np.linspace(0.15, 0.25, num_points)
    y_val = -0.085
    z_val = -0.3
    return np.array([[x, y_val, z_val] for x in x_vals])

def plot_joint_trajectory(q_des, q_dot_des):
    time = np.linspace(0, len(q_des) * 0.01, len(q_des))
    fig, axs = plt.subplots(2, 1, figsize=(10, 8))

    axs[0].plot(time, q_des[:, 0], label="q1 (hip roll)")
    axs[0].plot(time, q_des[:, 1], label="q2 (hip pitch)")
    axs[0].plot(time, q_des[:, 2], label="q3 (knee)")
    axs[0].set_title("Joint Angles")
    axs[0].set_ylabel("Angle (rad)")
    axs[0].legend()
    axs[0].grid(True)

    axs[1].plot(time, q_dot_des[:, 0], label="q1_dot")
    axs[1].plot(time, q_dot_des[:, 1], label="q2_dot")
    axs[1].plot(time, q_dot_des[:, 2], label="q3_dot")
    axs[1].set_title("Joint Velocities")
    axs[1].set_ylabel("Velocity (rad/s)")
    axs[1].set_xlabel("Time (s)")
    axs[1].legend()
    axs[1].grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    print("Generating dummy trajectory...")
    foot_traj = create_dummy_foot_trajectory()
    print("Converting to joint space...")
    q_des, q_dot_des = compute_joint_trajectory(foot_traj)
    print(f"First joint position: {q_des[0]}")
    print(f"First joint velocity: {q_dot_des[0]}")
    plot_joint_trajectory(q_des, q_dot_des)
