#!/usr/bin/python3

import matplotlib.pyplot as plt
import numpy as np
from tf.transformations import euler_from_quaternion
from get_robot_info import get_robot_coordinates
from give_command import move_robot
from create_dynamics_function import create_full_dynamics_function
from MPC import MPC
from scipy.ndimage import gaussian_filter
import scipy.io
import rospy
import time

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

        if delta > np.pi:
            current_yaw -= 2 * np.pi
        elif delta < -np.pi:
            current_yaw += 2 * np.pi

        if abs(current_yaw - self.previous_yaw) > np.pi / 2:
            current_yaw += np.pi
            current_yaw = (current_yaw + np.pi) % (2 * np.pi) - np.pi

        self.previous_yaw = current_yaw
        return current_yaw

yaw_unwrapper = YawUnwrapper()

def save_to_mat(filename, t, state_xi, pos_ref, control_inputs):
    data = {
        't': t,
        'state_xi': state_xi,
        'pos_ref': pos_ref,
        'control_inputs': control_inputs  # This will store [vx, vy, omega_z]
    }
    scipy.io.savemat(filename, data)
    print(f"Data saved to {filename}.")

def generate_trajectory(traj_type, duration, dt):
    t = np.arange(0, duration, dt)
    N = len(t)
    z_height = 0.3

    if traj_type == 'line':
        x = np.linspace(0, 10, N)
        y = np.linspace(0, 10, N)
        psi = np.full(N, np.arctan2(10, 10))
    elif traj_type == 'circle':
        omega = 2 * np.pi / duration
        angle = omega * t
        radius = 1
        x = radius * np.cos(angle)
        y = radius * np.sin(angle)
        psi = angle + np.pi / 2
    elif traj_type == 'sine':
        x = np.linspace(0, 20, N)
        y = 10 * np.sin(2 * np.pi * (1 / duration) * t)
        psi = np.arctan2(np.gradient(y), np.gradient(x))
    elif traj_type == 'square':
        side_length = 5.0
        segment_steps = int(N / 4)
        x, y, psi = [], [], []
        for i in range(4):
            x_start = [0, side_length, side_length, 0][i]
            x_end = [side_length, side_length, 0, 0][i]
            y_start = [0, 0, side_length, side_length][i]
            y_end = [0, side_length, side_length, 0][i]
            x_segment = np.linspace(x_start, x_end, segment_steps, endpoint=False)
            y_segment = np.linspace(y_start, y_end, segment_steps, endpoint=False)
            psi_segment = np.full(segment_steps, np.arctan2(y_end - y_start, x_end - x_start))
            x.extend(x_segment)
            y.extend(y_segment)
            psi.extend(psi_segment)
        x = np.array(x[:N])
        y = np.array(y[:N])
        psi = np.array(psi[:N])
    else:
        raise ValueError(f"Unknown trajectory type: {traj_type}")

    psi = gaussian_filter(np.unwrap(psi), sigma=3)

    trajectory = np.stack((x, y, psi), axis=1)

    # Add hold period at start
    hold_duration = 2.0
    hold_steps = int(hold_duration / dt)
    hold_segment = np.tile(trajectory[0], (hold_steps, 1))
    trajectory = np.vstack((hold_segment, trajectory))

    return trajectory

def stop_robot():
    try:
        rospy.loginfo("Stopping robot before shutdown...")
        move_robot(0, 0, 0)
        rospy.sleep(0.1)
    except Exception as e:
        rospy.logwarn(f"Failed to stop robot: {e}")

def main():
    rospy.init_node('robot_coordinate_listener', anonymous=True)
    Npred, dt, n, m = 7, 0.1, 3, 3
    xmin, xmax = np.array([-np.inf]*n), np.array([np.inf]*n)
    x0, u0 = np.zeros(n), np.zeros(m)
    Q = np.diag([120, 120, 120])
    R = np.diag([100, 100, 10])
    P = Q
    umin = np.array([-0.5, -0.5, -0.5])
    umax = np.array([1.5, 0.5, 0.5])

    traj_type = 'square'
    trajectory = generate_trajectory(traj_type, 30, dt)
    trajectory_x, trajectory_y, trajectory_psi = trajectory[:, 0], trajectory[:, 1], trajectory[:, 2]
    plot_time = np.arange(0, len(trajectory_x) * dt, dt)

    plt.ion()
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(10, 16))

    ax1.plot(trajectory_x, trajectory_y, 'r--', label='Reference Trajectory')
    robot_path_x, robot_path_y = [], []
    robot_plot, = ax1.plot([], [], 'b-', label='Robot Trajectory')
    ref_x_list, ref_y_list = [], []
    ref_plot, = ax1.plot([], [], 'g-', label='Followed Ref (mid-horizon)')
    ax1.legend()
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.set_title('Robot Trajectory in X-Y Space')
    ax1.grid()

    robot_time, robot_time_x, robot_time_y, robot_time_psi = [], [], [], []

    ax2.plot(plot_time, trajectory_x, 'g--', label='Reference X(t)')
    robot_x_plot, = ax2.plot([], [], 'b-', label='Robot X(t)')
    ax2.legend()
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('X Position')
    ax2.set_title('X Position Over Time')
    ax2.grid()

    ax3.plot(plot_time, trajectory_y, 'c--', label='Reference Y(t)')
    robot_y_plot, = ax3.plot([], [], 'b-', label='Robot Y(t)')
    ax3.legend()
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Y Position')
    ax3.set_title('Y Position Over Time')
    ax3.grid()

    ax4.plot(plot_time, trajectory_psi, 'm--', label='Reference Ψ(t)')
    robot_psi_plot, = ax4.plot([], [], 'b-', label='Robot Ψ(t)')
    ax4.legend()
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Yaw Angle (rad)')
    ax4.set_title('Yaw Angle Over Time')
    ax4.grid()

    rate = rospy.Rate(10)
    f = create_full_dynamics_function()
    trajectory_index = 0

    control_inputs = []

    try:
        while not rospy.is_shutdown() and trajectory_index < len(trajectory):
            position, orientation = get_robot_coordinates()
            roll, pitch, yaw = euler_from_quaternion(orientation)
            yaw = yaw_unwrapper.unwrap(yaw)
            xref = np.array([trajectory[min(trajectory_index + k, len(trajectory) - 1)] for k in range(Npred + 1)]).T
            x0 = np.array([position[0], position[1], yaw])

            try:
                start_time = time.time()
                u = MPC(Npred, x0, u0, n, m, dt, f, umin, umax, xmin, xmax, Q, R, P, xref)
                end_time = time.time()
            except RuntimeError as e:
                rospy.logerr(f"MPC solver failed: {e}")
                continue

            print(start_time - end_time)

            control_inputs.append(u.copy())

            robot_path_x.append(position[0])
            robot_path_y.append(position[1])
            robot_plot.set_data(robot_path_x, robot_path_y)

            robot_time.append(trajectory_index * dt)
            robot_time_x.append(position[0])
            robot_time_y.append(position[1])
            robot_time_psi.append(yaw)
            robot_x_plot.set_data(robot_time, robot_time_x)
            robot_y_plot.set_data(robot_time, robot_time_y)
            robot_psi_plot.set_data(robot_time, robot_time_psi)

            ref_idx = min(trajectory_index, len(trajectory) - 1)
            ref_x_list.append(trajectory[ref_idx, 0])
            ref_y_list.append(trajectory[ref_idx, 1])
            ref_plot.set_data(ref_x_list, ref_y_list)

            for ax in [ax1, ax2, ax3, ax4]:
                ax.relim()
                ax.autoscale_view()

            plt.draw()
            plt.pause(0.001)

            move_robot(u[0], u[1], u[2])
            u0 = u
            trajectory_index += 1
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down.")
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard Interrupt detected. Stopping robot.")
        stop_robot()
    finally:
        move_robot(0, 0, 0)
        rospy.loginfo("Saving data and exiting program.")
        # Time array
        t_array = np.array(robot_time)

        # Actual robot states (x, y, yaw)
        state_xi_array = np.vstack((robot_time_x, robot_time_y, robot_time_psi)).T

        # Reference states (x_ref, y_ref, yaw_ref)
        ref_indices = np.clip((t_array / dt).astype(int), 0, len(trajectory) - 1)
        pos_ref_array = trajectory[ref_indices]  # Shape: (N, 3) -> x, y, yaw

        # Save all to .mat
        control_inputs_array = np.array(control_inputs)
        save_to_mat("nonlinear_mpc_control_input.mat", t_array, state_xi_array, pos_ref_array, control_inputs_array)

if __name__ == '__main__':
    main()
