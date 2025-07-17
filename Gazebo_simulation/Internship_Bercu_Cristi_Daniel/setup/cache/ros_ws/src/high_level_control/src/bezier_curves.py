#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from velocity_to_leg import QuadrupedVelocityTransformer
import inverse_kinmatics

def bezier_M_matrix():
    return np.array([
        [1, 0, 0, 0, 0, 0, 0],
        [-6, 6, 0, 0, 0, 0, 0],
        [15, -30, 15, 0, 0, 0, 0],
        [-20, 60, -60, 20, 0, 0, 0],
        [15, -60, 90, -60, 15, 0, 0],
        [-6, 30, -60, 60, -30, 6, 0],
        [1, -6, 15, -20, 15, -6, 1]
    ])

def duty_factor_u(t, T_p, beta_u):
    beta_t = 0.3
    a = (1 - beta_t) / 2 
    b = (1 + beta_t) / 2 
    if 0 <= t/T_p < a:
        return (t / T_p) * ((beta_u - 1) / (beta_t - 1))
    elif a <= t/T_p <= b:
        return (beta_u / (2 * beta_t)) * ((2 * t / T_p) - 1) + 0.5
    else:
        return beta_u/2 - (beta_u - 1)*(beta_t - (2*t)/T_p + 1)/(2*(beta_t - 1)) + 0.5

def generate_u_vals(T_p, beta_u=0.6):
    t_vals = np.linspace(0, T_p - 1e-6, 7)
    return np.array([duty_factor_u(t, T_p, beta_u) for t in t_vals])

def calc_cp_freqP(veld, cp_max=0.18, freqP_max=2.0, freqP_min=1.0):
    cp = cp_max
    freqP = freqP_max
    if veld > freqP_max * cp_max:
        return cp_max, freqP_max
    while True:
        if veld >= cp * (freqP - freqP_min) and veld <= cp * freqP:
            return veld / freqP, freqP
        elif freqP <= freqP_min:
            cp += 0.005
            if cp > cp_max:
                return 0, freqP_max
            freqP = freqP_max
        else:
            freqP -= 0.001

def compute_control_points(p3, v_leg, theta, v_max=2.0, h_min=0.0, h_max=0.2):
    d, freq = calc_cp_freqP(v_leg)
    h = h_min + (h_max - h_min) * (v_leg / v_max)
    dx = d * 0.7 * np.cos(theta)
    dy = d * 0.7 * np.sin(theta)
    p0 = p3 + np.array([0, 0, h])
    p1 = p3 + np.array([dx * 4/5, dy * 4/5, h * 3/5])
    p2 = p3 + np.array([dx * 5/5, dy * 5/5, h * 1/5])
    p4 = p3 - np.array([dx * 5/5, dy * 5/5, -h * 1/5])
    p5 = p3 - np.array([dx * 4/5, dy * 4/5, -h * 3/5])
    p6 = p0
    return np.vstack([p0, p1, p2, p3, p4, p5, p6]), h, freq

def fit_weights(P, u_vals):
    T = np.vander(u_vals, N=7, increasing=True)
    M = bezier_M_matrix()
    W = np.linalg.inv(M) @ np.linalg.inv(T.T @ T) @ T.T @ P
    return W

def evaluate_bezier(u_vals, W):
    M = bezier_M_matrix()
    return np.array([np.array([u**i for i in range(7)]) @ M @ W for u in u_vals])

def evaluate_bezier_velocity(u_vals, W):
    M = bezier_M_matrix()
    dW = 6 * (W[1:] - W[:-1])  # 6th-degree Bézier: velocity uses 6*(Wi+1 - Wi)
    M_vel = np.array([
        [1, 0, 0, 0, 0, 0],
        [-5, 5, 0, 0, 0, 0],
        [10, -20, 10, 0, 0, 0],
        [-10, 30, -30, 10, 0, 0],
        [5, -20, 30, -20, 5, 0],
        [-1, 5, -10, 10, -5, 1]
    ])
    return np.array([np.array([u**i for i in range(6)]) @ M_vel @ dW for u in u_vals])

def evaluate_bezier_acceleration(u_vals, W):
    dW = 6 * (W[1:] - W[:-1])
    ddW = 5 * (dW[1:] - dW[:-1])  # second derivative
    M_acc = np.array([
        [1, 0, 0, 0, 0],
        [-4, 4, 0, 0, 0],
        [6, -12, 6, 0, 0],
        [-4, 12, -12, 4, 0],
        [1, -4, 6, -4, 1]
    ])
    return np.array([np.array([u**i for i in range(5)]) @ M_acc @ ddW for u in u_vals])

def generate_all_leg_trajectories(vx, vy, omega_z, walking_type):
    L, C = 0.47, 0.30
    transformer = QuadrupedVelocityTransformer(L, C)
    velocities = transformer.transform_body_to_leg_velocities(vx, vy, omega_z)

    gait_phases = {
        "trol": {
            "front_right": 0.0, "front_left": 0.5, "rear_right": 0.5, "rear_left": 0.0
        },
        "walk": {
            "front_right": 0.0, "front_left": 0.25, "rear_right": 0.5, "rear_left": 0.75
        }
    }[walking_type]

    initial_foot_positions = {
        "front_right": np.array([ 0.2, -0.085, -0.3]),
        "front_left":  np.array([ 0.2,  0.085, -0.3]),
        "rear_right":  np.array([-0.2, -0.085, -0.3]),
        "rear_left":   np.array([-0.2,  0.085, -0.3])
    }

    leg_mapping = {
        "fd": "front_right", "fe": "front_left",
        "td": "rear_right",  "te": "rear_left"
    }

    joint_trajectories = {}
    joint_velocities = {}

    for leg_code, leg_name in leg_mapping.items():
        v = velocities[f"v_{leg_code}"]
        theta = velocities[f"theta_{leg_code}"]
        p3 = initial_foot_positions[leg_name]

        control_pts, h, freq = compute_control_points(p3, v, theta)
        T_p = 1.0 / freq
        beta_u = 0.6
        u_fit = generate_u_vals(T_p, beta_u)
        W = fit_weights(control_pts, u_fit)

        phase_offset = gait_phases[leg_name]
        u_eval = (np.linspace(0, 1, 150) + phase_offset) % 1.0

        foot_pos = evaluate_bezier(u_eval, W)
        foot_vel = evaluate_bezier_velocity(u_eval, W)

        q_list = []
        dq_list = []

        for i in range(len(foot_pos)):
            q = inverse_kinmatics.go1_leg_inverse_kinematics(foot_pos[i])
            J = inverse_kinmatics.jacobian(q)
            dq = J @ foot_vel[i]
            q_list.append(q)
            dq_list.append(dq)

        joint_trajectories[leg_name] = np.array(q_list)
        joint_velocities[leg_name] = np.array(dq_list)

    return joint_trajectories, joint_velocities

def plot_joint_data(joint_traj, joint_vel):
    t = np.linspace(0, 0.5, len(next(iter(joint_traj.values()))))
    for leg, q in joint_traj.items():
        dq = joint_vel[leg]
        fig, axs = plt.subplots(2, 1, figsize=(10, 6))
        axs[0].set_title(f"{leg} Joint Trajectory")
        axs[1].set_title(f"{leg} Joint Velocities")
        for i in range(3):
            axs[0].plot(t, q[:, i], label=f"q{i+1}")
            axs[1].plot(t, dq[:, i], label=f"q{i+1}_dot")
        axs[0].legend(); axs[1].legend()
        axs[0].set_ylabel("Angle [rad]"); axs[1].set_ylabel("Velocity [rad/s]")
        axs[1].set_xlabel("Time [s]")
        plt.tight_layout()
        plt.show()

# Example usage
if __name__ == "__main__":
    vx, vy, omega_z = 1.0, 0.0, 0.0
    q_des_all, q_dot_des_all = generate_all_leg_trajectories(vx, vy, omega_z, "trol")
    plot_joint_data(q_des_all, q_dot_des_all)