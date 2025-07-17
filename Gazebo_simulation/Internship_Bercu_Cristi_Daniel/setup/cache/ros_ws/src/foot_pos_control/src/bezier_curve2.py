import numpy as np
import matplotlib.pyplot as plt
from scipy.special import comb

def bezier_curve(P, u):
    n = len(P) - 1
    return sum(comb(n, i) * (1 - u) ** (n - i) * (u ** i) * P[i] for i in range(n + 1))

def generate_bezier_weights(P, T):
    M = np.array([[comb(6, i) * (t**i) * ((1 - t)**(6 - i)) for i in range(7)] for t in T])
    W = np.linalg.lstsq(M, P, rcond=None)[0]
    return W

def create_leg_trajectory(velocity, height=0.06, freq_range=(1.0, 2.0), max_step_len=0.18):
    # Calculate step frequency and length based on velocity
    v = np.linalg.norm(velocity)
    theta = np.arctan2(velocity[1], velocity[0])
    
    if v > freq_range[1] * max_step_len:
        v = freq_range[1] * max_step_len
    
    freq = freq_range[0] + (freq_range[1] - freq_range[0]) * (v / (freq_range[1] * max_step_len))
    step_length = v / freq
    
    dx = step_length * 0.7 * np.cos(theta)
    dy = step_length * 0.7 * np.sin(theta)
    
    P3 = np.array([0, 0, -0.3])
    P0 = P3 + np.array([0, 0, height])
    P1 = P3 + np.array([dx * 4/5, dy * 4/5, height * 3/5])
    P2 = P3 + np.array([dx, dy, height * 1/5])
    P4 = P3 - np.array([dx, dy, -height * 1/5])
    P5 = P3 - np.array([dx * 4/5, dy * 4/5, -height * 3/5])
    P6 = P0

    P_points = np.array([P0, P1, P2, P3, P4, P5, P6])
    T_vals = np.linspace(0, 1, 100)
    trajectory = np.array([bezier_curve(P_points, t) for t in T_vals])
    
    return trajectory, freq, step_length

# Example usage
velocity = np.array([0.15, 0.05])  # desired velocity (x, y) in m/s
trajectory, freq, step_length = create_leg_trajectory(velocity)

# Visualization
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.plot(trajectory[:, 0], trajectory[:, 1], trajectory[:, 2])
ax.set_title(f"Leg Trajectory | Freq: {freq:.2f} Hz | Step Len: {step_length:.3f} m")
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()