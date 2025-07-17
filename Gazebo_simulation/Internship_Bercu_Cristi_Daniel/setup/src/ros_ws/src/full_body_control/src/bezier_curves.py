#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import scipy.io
from velocity_to_leg import QuadrupedVelocityTransformer

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
    beta_t = 0.4
    beta_t = 0.24
    a = (1 - beta_t) / 2 
    b = (1 + beta_t) / 2 
    if 0 <= t/T_p < a:
        return (t / T_p) * ((beta_u - 1) / (beta_t - 1))
    elif a <= t/T_p <= b:
        return (beta_u / (2 * beta_t)) * ((2 * t / T_p) - 1) + 0.5
    else:
        return beta_u/2 - (beta_u - 1)*(beta_t - (2*t)/T_p + 1)/(2*(beta_t - 1)) + 0.5

def generate_u_vals(T_p, beta_u=0.6):
    t_vals = np.linspace(0, T_p, 7)
    return np.array([duty_factor_u(t, T_p, beta_u) for t in t_vals])

def calc_cp_freqP(veld, cp_max=0.30, freqP_max=2.0, freqP_min=1.0):
    a = True
    cp = cp_max
    freqP = freqP_max

    if veld > freqP_max * cp_max:
        return cp_max, freqP_max

    while a:
        if veld >= cp * (freqP - freqP_min) and veld <= cp * freqP:
            freqP = freqP + 0.001
            cp = veld / freqP
            a = False
            break
        elif freqP <= freqP_min:
            cp += 0.005
            freqP = freqP_max
            if cp > cp_max:
                cp = 0
        else:
            freqP -= 0.001

    return cp, freqP



def compute_control_points(p3, v_leg, theta, v_max=2.0, h_min=0.06, h_max=0.12):
    d, freq = calc_cp_freqP(v_leg)
    h = h_min + (h_max - h_min) * (v_leg / v_max)
    h= 0.08
    direction = np.sign(v_leg) if abs(v_leg) > 0.01 else 1.0
    dx =  d * 0.7 * np.cos(theta)
    dy =  d * 0.7 * np.sin(theta)

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
    # print(W)
    # print("eheeee")
    # print(W[1:] - W[:-1])
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



def generate_all_leg_trajectories(vx, vy, omega_z, walking_type, control_rate=100): 
    C = 0.47  
    L = 0.30  
    # if vy != 0:
    #     omega_z = 0.507 * vy + -0.091 + omega_z
    epsilon = 1e-3
    if abs(vx) < epsilon and abs(vy) < epsilon and abs(omega_z) < epsilon:
        static_positions = {
            "front_left":  np.array([ 0.225,  0.15, -0.30]),
            "front_right": np.array([ 0.225, -0.15, -0.30]),
            "rear_left":   np.array([-0.225,  0.15, -0.33]),
            "rear_right":  np.array([-0.225, -0.15, -0.33])
        }
        num_points = int(1.0 * control_rate)  # 1 second worth of static points

        trajectories = {leg: np.tile(pos, (num_points, 1)) for leg, pos in static_positions.items()}
        velocities = {leg: np.zeros((num_points, 3)) for leg in static_positions}
        accelerations = {leg: np.zeros((num_points, 3)) for leg in static_positions}
        
        return trajectories, velocities, accelerations

    transformer = QuadrupedVelocityTransformer(L, C)
    velocities = transformer.transform_body_to_leg_velocities(vx, vy, omega_z)

    gait_phases = {
        "trot": {"front_left": 0.5, "front_right": 0.0 + 1e-6, "rear_left": 0.0+ 1e-6, "rear_right": 0.5},
        "walk": {"front_left": 0.0, "front_right": 0.25, "rear_left": 0.5, "rear_right": 0.75},
    }[walking_type]

    initial_foot_positions = {
        "front_left": np.array([ 0.225,  0.15, -0.30]),
        "front_right": np.array([ 0.225, -0.15, -0.30]),
        "rear_left": np.array([-0.225,  0.15, -0.30]),
        "rear_right": np.array([-0.225, -0.15, -0.30])
    }

    leg_mapping = {"fe": "front_left", "fd": "front_right", "te": "rear_left", "td": "rear_right"}

    # Determine common frequency across all legs (synchronized gait)
    leg_frequencies = []
    for leg_code in leg_mapping:
        v_leg = velocities[f"v_{leg_code}"]
        _, freq = calc_cp_freqP(v_leg)
        leg_frequencies.append(freq)

    common_freq = max(leg_frequencies)
    T_p = 1.0 / common_freq
    num_points = int(np.round(T_p * control_rate))

    # Allocate outputs
    trajectories = {}
    bez_velocities = {}
    accelerations = {}
    control_points_dict = {}

    for leg_code, leg_name in leg_mapping.items():
        v_leg = velocities[f"v_{leg_code}"]
        theta_leg = velocities[f"theta_{leg_code}"]
        p3 = initial_foot_positions[leg_name]


        stride_length, _ = calc_cp_freqP(v_leg)  # only use cp, not freq
        control_pts, _, _ = compute_control_points(p3, stride_length, theta_leg)

        u_fit = generate_u_vals(T_p)
        W = fit_weights(control_pts, u_fit)

        phase_offset = gait_phases[leg_name]
        t_eval = np.linspace(0, T_p, num_points, endpoint=False)
        u_eval = (t_eval / T_p + phase_offset) % 1.0

        trajectory = evaluate_bezier(u_eval, W)
        trajectories[leg_name] = trajectory

        bez_velocity = evaluate_bezier_velocity(u_eval, W)
        bez_velocities[leg_name] = bez_velocity

        bez_acc = evaluate_bezier_acceleration(u_eval, W)
        accelerations[leg_name] = bez_acc

        control_points_dict[leg_name] = control_pts
    

    return trajectories, bez_velocities, accelerations


def plot_3d_trajectories(trajectories, highlight_idx_ratio=0.4):
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection="3d")
    
    for leg_name, traj in trajectories.items():
        ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], label=leg_name)

        # Highlight a point at given ratio (e.g., 50% of trajectory)
        highlight_idx = int(len(traj) * highlight_idx_ratio)
        highlight_point = traj[highlight_idx]
        ax.scatter(highlight_point[0], highlight_point[1], highlight_point[2], 
                   marker='o', s=50, label=f"{leg_name} highlight")
        
    ax.set_xlim(-0.2, 0.2)
    ax.set_ylim(-0.15, 0.15)
    ax.set_zlim(-0.35, 0.0)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("Bézier Trajectories for All Legs with Highlighted Points")
    ax.legend()
    plt.show()



def generate_rise_trajectories(z_start=-0.05, z_end=-0.30, duration=5.0, control_rate=100):
    """
    Generate a rise trajectory using 7th-degree Bézier curves for Z lift.
    All four legs follow the exact same timing, rising together in sync.
    """
    T_p = duration
    num_points = int(np.round(T_p * control_rate))
    u_vals = np.linspace(0, 1, num_points)
    dz = z_end - z_start

    initial_foot_positions = {
        "front_left":  np.array([ 0.225,  0.18, z_start]),
        "front_right": np.array([ 0.225, -0.18, z_start]),
        "rear_left":   np.array([-0.225,  0.18, z_start]),
        "rear_right":  np.array([-0.225, -0.18, z_start]),
    }

    trajectories = {}
    velocities = {}
    accelerations = {}
    control_points_dict = {}

    for leg_name, base_pos in initial_foot_positions.items():
        control_points = np.array([
            base_pos + np.array([0, 0, 0.00 * dz]),
            base_pos + np.array([0, 0, 0.05 * dz]),
            base_pos + np.array([0, 0, 0.25 * dz]),
            base_pos + np.array([0, 0, 0.50 * dz]),
            base_pos + np.array([0, 0, 0.75 * dz]),
            base_pos + np.array([0, 0, 0.95 * dz]),
            base_pos + np.array([0, 0, 1.00 * dz]),
        ])

        u_fit = generate_u_vals(T_p)
        W = fit_weights(control_points, u_fit)

        traj = evaluate_bezier(u_vals, W)
        vel = evaluate_bezier_velocity(u_vals, W) / T_p
        acc = evaluate_bezier_acceleration(u_vals, W) / T_p**2
        control_points_dict[leg_name] = control_points

        trajectories[leg_name] = traj
        velocities[leg_name] = vel
        accelerations[leg_name] = acc

    return trajectories, velocities, accelerations


def plot_single_leg_trajectory_scatter(trajectories, leg_name="rear_right"):
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    
    traj = trajectories[leg_name]
    ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], c='b', marker='')
    
    ax.set_xlim(-0.3, -0.1)
    ax.set_ylim(-0.15, 0.15)
    ax.set_zlim(-0.35, -0.15)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title(f"Trajectory Scatter for {leg_name}")
    plt.show()

# Example usage

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation

def animate_3d_trajectories(trajectories, interval=100):
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection="3d")
    
    # Set limits
    ax.set_xlim(-0.25, 0.25)
    ax.set_ylim(-0.2, 0.2)
    ax.set_zlim(-0.35, 0.0)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Animated Bézier Trajectories for All Legs')

    # Initialize scatter points for each leg
    scatters = {}
    colors = {
        "front_left": "red",
        "front_right": "blue",
        "rear_left": "green",
        "rear_right": "purple"
    }

    for leg_name in trajectories.keys():
        scatters[leg_name] = ax.scatter([], [], [], color=colors.get(leg_name, "black"), label=leg_name)

    num_points = len(next(iter(trajectories.values())))

    def update(frame):
        for leg_name, traj in trajectories.items():
            point = traj[frame]
            scatters[leg_name]._offsets3d = ([point[0]], [point[1]], [point[2]])
        return scatters.values()

    ani = animation.FuncAnimation(
        fig, update, frames=num_points, interval=interval, blit=False, repeat=True
    )

    ax.legend()
    plt.show()

    return ani

def save_to_mat(filename, t, state_xi, pos_ref, vel_ref=None, acc_ref=None, control_pts=None):
    data = {
        't': t,
        'state_xi': state_xi,
        'pos_ref': pos_ref
    }
    if vel_ref is not None:
        data['vel_ref'] = vel_ref
    if acc_ref is not None:
        data['acc_ref'] = acc_ref
    if control_pts is not None:
        data['control_points'] = control_pts

    scipy.io.savemat(filename, data)
    print(f"Data saved to {filename}.")

if __name__ == "__main__":
    import scipy.io

    # Set input velocities
    vx, vy, omega_z = 0.2, 0.0, 0.0
    control_rate = 100

    # # Generate trajectories, velocities, accelerations, and control points
    trajectories, vels, accs = generate_all_leg_trajectories(
        vx, vy, omega_z, walking_type="trot", control_rate=control_rate
    )
    # trajectories, vels, accs, control_points = generate_rise_trajectories()
    plot_3d_trajectories(trajectories,0)
    # # Create time vector
    num_points = len(next(iter(trajectories.values())))
    t = np.linspace(0, num_points / control_rate, num_points)

    # # State vector (can be more detailed if needed)
    state_xi = np.array([vx, vy, omega_z])

    # # Save everything to a .mat file
    save_to_mat(
        filename="trajectory_data_bezier.mat",
        t=t,
        state_xi=state_xi,
        pos_ref=trajectories,
        vel_ref=vels,
        acc_ref=accs,
        # control_pts=control_points
    )