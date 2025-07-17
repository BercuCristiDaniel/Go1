#!/usr/bin/python3

import matplotlib.pyplot as plt
import numpy as np
from tf.transformations import euler_from_quaternion
from get_robot_info import get_robot_coordinates
from give_command import move_robot
from create_dynamics_function import create_dynamics_function
from MPC import MPC
from scipy.ndimage import gaussian_filter
import time
import rospy

def low_pass_filter(signal, prev_signal, alpha=0.9):
    return alpha * prev_signal + (1 - alpha) * signal

def unwrap_angle(angles):
    return np.unwrap(angles)

class YawUnwrapper:
    def __init__(self):
        self.previous_yaw = None

    def unwrap(self, current_yaw):
        if self.previous_yaw is None:
            self.previous_yaw = current_yaw
            return current_yaw

        delta = current_yaw - self.previous_yaw

        # Adjust yaw to prevent large backward turns
        if delta > np.pi:
            current_yaw -= 2 * np.pi
        elif delta < -np.pi:
            current_yaw += 2 * np.pi

        # Ensure yaw remains forward-aligned
        if abs(current_yaw - self.previous_yaw) > np.pi / 2:
            current_yaw += np.pi  # Flip 180 degrees to maintain forward motion
            current_yaw = (current_yaw + np.pi) % (2 * np.pi) - np.pi  # Keep within [-π, π]

        self.previous_yaw = current_yaw
        return current_yaw
    

yaw_unwrapper = YawUnwrapper()

def calculate_trajectory_duration_from_points(trajectory, max_speed, safety_factor=1.5):
    if max_speed <= 0:
        raise ValueError("Max speed must be greater than zero.")
    if trajectory.shape[1] < 2:
        raise ValueError("Trajectory must have at least x and y columns.")

    # Compute distances between consecutive trajectory points
    distances = np.sqrt(np.diff(trajectory[:, 0])**2 + np.diff(trajectory[:, 1])**2)
    
    # Calculate total trajectory length
    total_length = np.sum(distances)

    # Calculate base duration (trajectory length / max speed)
    base_duration = total_length / max_speed

    # Apply safety factor
    recommended_duration = base_duration * safety_factor

    return recommended_duration


def generate_trajectory(traj_type, duration, dt):
    t = np.arange(0, duration, dt)
    N = len(t)
    trajectory = []

    if traj_type == 'line':
        x_start, y_start, x_end, y_end = 0.0, 0.0, 10.0, 10.0
        x = np.linspace(x_start, x_end, N)
        y = np.linspace(y_start, y_end, N)
        psi = np.full(N, np.arctan2(y_end - y_start, x_end - x_start))
        psi = unwrap_angle(psi)
        trajectory = np.stack((x, y, psi), axis=1)

    elif traj_type == 'circle':
        x_center, y_center, radius = 0.0, 0.0, 5.0
        omega = 2 * np.pi / duration
        angle = omega * t
        x = x_center + radius * np.cos(angle)
        y = y_center + radius * np.sin(angle)
        psi = angle + np.pi / 2
        psi = unwrap_angle(psi)
        trajectory = np.stack((x, y, psi), axis=1)

    elif traj_type == 'sine':
        x_start, x_end, amplitude, frequency = 0.0, 20.0, 10.0, 1 / duration
        x = np.linspace(x_start, x_end, N)
        y = amplitude * np.sin(2 * np.pi * frequency * t)
        psi = np.arctan2(np.gradient(y), np.gradient(x))
        psi = unwrap_angle(psi)
        trajectory = np.stack((x, y, psi), axis=1)

    elif traj_type == 'square':
        side_length = 10.0
        velocity = side_length / (duration / 4)
        x_waypoints = [0, side_length, side_length, 0, 0]
        y_waypoints = [0, 0, side_length, side_length, 0]
        
        x, y, psi = [], [], []
        
        for i in range(4):
            x_start, x_end = x_waypoints[i], x_waypoints[i + 1]
            y_start, y_end = y_waypoints[i], y_waypoints[i + 1]
            
            t_segment = np.linspace(0, duration / 4, N // 4, endpoint=False)
            
            if x_start == x_end:
                x_segment = np.full_like(t_segment, x_start)
            else:
                x_segment = np.linspace(x_start, x_end, len(t_segment))
            
            if y_start == y_end:
                y_segment = np.full_like(t_segment, y_start)
            else:
                y_segment = np.linspace(y_start, y_end, len(t_segment))
            
            psi_segment = np.full_like(t_segment, np.arctan2(y_end - y_start, x_end - x_start))
            
            x.extend(x_segment)
            y.extend(y_segment)
            psi.extend(psi_segment)
        
        x = np.array(x[:N])
        y = np.array(y[:N])
        psi = np.array(psi[:N])
        psi = unwrap_angle(psi)
        psi = gaussian_filter(psi, sigma=3)
        trajectory = np.stack((x, y, psi), axis=1)

    elif traj_type == 'random':
        x_start, x_end, amplitude, frequency = 0.0, 20.0, 10.0, 1 / duration
        period = 1 / frequency
        t = np.arange(0, duration, dt)
        N = len(t)

        x = np.zeros(N)
        for i in range(N):
            t_mod = (t[i] % period)
            if t_mod < period / 2:
                x[i] = x_start + (x_end - x_start) * (t_mod / (period / 2))
            else:
                x[i] = x_start + (x_end - x_start)

        y = amplitude * np.sin(2 * np.pi * frequency * t)
        psi = np.arctan2(np.gradient(y), np.gradient(x))
        psi = unwrap_angle(psi)
        trajectory = np.stack((x, y, psi), axis=1)

    else:
        raise ValueError(f"Unknown trajectory type: {traj_type}")

    # Smooth the yaw angle (psi) to reduce sharp transitions
    trajectory[:, 2] = gaussian_filter(trajectory[:, 2], sigma=5)

    return trajectory

if __name__ == '__main__':
    rospy.init_node('robot_coordinate_listener', anonymous=True)

    # MPC parameters
    Npred, dt, n, m = 15, 0.1, 3, 2

    xmin, xmax = np.array([-np.inf, -np.inf, -np.inf]), np.array([np.inf, np.inf, np.inf])  # State constraints
    x0, u0 = np.array([0, 0, 0, 0, 0]), np.array([0, 0])  # State initialize

    # MPC tuning
    Q = np.diag([10, 10, 10])  
    R = np.diag([30, 30])        
    P = Q

    umin = np.array([-0.5, -0.3])   
    umax = np.array([1.5, 0.3])

    delta_u_min = np.array([-1, -0.5])  
    delta_u_max = np.array([1, 0.5])

    traj_type = 'square'
    # Generate trajectory
    trajectory = generate_trajectory(traj_type, 50, dt)

    trajectory_time = calculate_trajectory_duration_from_points(trajectory, 1.5)

    trajectory = generate_trajectory(traj_type, trajectory_time, dt)

    num_prepended_points = 30  # Number of additional points to prepend
    first_point = trajectory[0]  # First point of the trajectory
    prepended_points = np.tile(first_point, (num_prepended_points, 1))  # Duplicate the first point
    trajectory = np.vstack((prepended_points, trajectory))  # Prepend the points

    rospy.loginfo(f"Prepended {num_prepended_points} points to the trajectory.")

    trajectory_x = trajectory[:, 0]
    trajectory_y = trajectory[:, 1]
    trajectory_psi = trajectory[:, 2]

    # Time vector for trajectory
    plot_time = np.arange(0, len(trajectory_x) * dt, dt)



    plt.ion()
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(10, 16))

    # Plot trajectory
    ax1.plot(trajectory_x, trajectory_y, 'r--', label='Reference Trajectory')
    robot_path_x, robot_path_y = [], []
    robot_plot, = ax1.plot([], [], 'b-', label='Robot Trajectory')
    ax1.legend()
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.set_title('Robot Trajectory in X-Y Space')
    ax1.grid()

    # Plot x vs time
    ax2.plot(plot_time, trajectory_x, 'g--', label='Reference X(t)')
    robot_time_x, robot_time = [], []
    robot_x_plot, = ax2.plot([], [], 'b-', label='Robot X(t)')
    ax2.legend()
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('X Position')
    ax2.set_title('X Position Over Time')
    ax2.grid()

    # Plot y vs time
    ax3.plot(plot_time, trajectory_y, 'c--', label='Reference Y(t)')
    robot_time_y, robot_time = [], []
    robot_y_plot, = ax3.plot([], [], 'b-', label='Robot Y(t)')
    ax3.legend()
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Y Position')
    ax3.set_title('Y Position Over Time')
    ax3.grid()

    # Plot psi (yaw) vs time
    ax4.plot(plot_time, trajectory_psi, 'm--', label='Reference Ψ(t)')
    robot_time_psi, robot_time = [], []
    robot_psi_plot, = ax4.plot([], [], 'b-', label='Robot Ψ(t)')
    ax4.legend()
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Yaw Angle (rad)')
    ax4.set_title('Yaw Angle Over Time')
    ax4.grid()

    rate = rospy.Rate(10)
    f = create_dynamics_function()
    trajectory_index = 0
    pre_alignment_done = False
    trajectory_extended = False

    average_comp_time = np.array([])
    tracking_errors = np.array([])
    try:

        while not rospy.is_shutdown():
            position, orientation = get_robot_coordinates()
            roll, pitch, yaw = euler_from_quaternion(orientation)
            yaw = yaw_unwrapper.unwrap(yaw)


            xref = []
            for k in range(Npred + 1):
                idx = min(trajectory_index + k, len(trajectory) - 1)
                xref.append(trajectory[idx])
            xref = np.array(xref).T
            
            ref_x, ref_y, ref_psi = trajectory[min(trajectory_index, len(trajectory) - 1)]
            tracking_error = np.sqrt((position[0] - ref_x)**2 + (position[1] - ref_y)**2)
            tracking_errors = np.append(tracking_errors,tracking_error)

            trajectory_index += 1
            x0 = np.array([position[0], position[1], yaw])

            start = time.time()

            try:
                u = MPC(Npred, x0, u0, n, m, dt, f, umin, umax, xmin, xmax, delta_u_min, delta_u_max, Q, R, P, xref)
            except RuntimeError as e:
                rospy.logerr(f"MPC solver failed: {e}")
                continue
            
            end = time.time()

            average_comp_time = np.append(average_comp_time, end - start)

            if trajectory_index == len(trajectory)-1:
                print("Computation time:")
                print(np.average(average_comp_time))
                print("Computation time:")
                print("Tracking error:")
                print(np.average(tracking_errors))

            move_robot(u[0], u[1])
            u0 = u

            robot_path_x.append(position[0])
            robot_path_y.append(position[1])
            robot_plot.set_data(robot_path_x, robot_path_y)

            robot_time_x.append(position[0])
            robot_time_y.append(position[1])
            robot_time_psi.append(yaw)
            robot_time.append(trajectory_index * dt)

            robot_x_plot.set_data(robot_time, robot_time_x)
            robot_y_plot.set_data(robot_time, robot_time_y)
            robot_psi_plot.set_data(robot_time, robot_time_psi)


            for ax in [ax1, ax2, ax3, ax4]:
                ax.relim()
                ax.autoscale_view()

            plt.draw()
            plt.pause(0.001)

            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down.")
