#!/usr/bin/python3

import numpy as np
from tf.transformations import euler_from_quaternion
from get_robot_info import get_robot_coordinates
from give_command import move_robot
from feedback_linearization import FeedbackLinearization
from LMPC import linear_MPC
import rospy
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter
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


def generate_trajectory(traj_type, duration, dt):
    t = np.arange(0, duration, dt)
    N = len(t)

    if traj_type == 'line':
        x_start, y_start, x_end, y_end = 0.0, 0.0, 10.0, 10.0
        x = np.linspace(x_start, x_end, N)
        y = np.linspace(y_start, y_end, N)
        psi = np.full(N, np.arctan2(y_end - y_start, x_end - x_start))

    elif traj_type == 'circle':
        x_center, y_center, radius = 0.0, 0.0, 5.0
        omega = 2 * np.pi / duration
        angle = omega * t
        x = x_center + radius * np.cos(angle)
        y = y_center + radius * np.sin(angle)
        psi = angle + np.pi / 2  # Tangent direction

    elif traj_type == 'sine':
        x_start, x_end, amplitude, frequency = 0.0, 20.0, 10.0, 1 / duration
        x = np.linspace(x_start, x_end, N)
        y = amplitude * np.sin(2 * np.pi * frequency * t)
        psi = np.arctan2(np.gradient(y), np.gradient(x))

    elif traj_type == 'square':
        side_length = 5.0
        total_perimeter = 4 * side_length
        segment_time = duration / 4
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

        # Ensure we have exactly N samples
        x.append(0)
        y.append(0)
        psi.append(np.arctan2(0, -1))

        x = np.array(x[:N])
        y = np.array(y[:N])
        psi = np.array(psi[:N])

    elif traj_type == 'random':
        x_start, x_end, amplitude, frequency = 0.0, 10.0, 2.0, 1 / duration
        period = 1 / frequency

        x = np.zeros(N)
        for i in range(N):
            t_mod = t[i] % period
            if t_mod < period / 2:
                x[i] = x_start + (x_end - x_start) * (t_mod / (period / 2))
            else:
                x[i] = x_end

        y = amplitude * np.sin(2 * np.pi * frequency * t)
        psi = np.arctan2(np.gradient(y), np.gradient(x))

    else:
        raise ValueError(f"Unknown trajectory type: {traj_type}")

    # Unwrap and smooth yaw angle (psi)
    psi = np.unwrap(psi)
    psi = gaussian_filter(psi, sigma=3)

    # Final Trajectory (N x 6): [x, y, z, roll, pitch, yaw]
    trajectory = np.stack((x, y, psi), axis=1)

    return trajectory

def main():
    rospy.init_node('robot_coordinate_listener', anonymous=True)

    # MPC parameters
    Npred, dt, n, m = 15, 0.1, 3, 2


    umin = np.array([-0.5, -0.5])
    umax = np.array([ 1.5,  0.5])
    

    Q = np.diag([20, 20, 5])       
    R = np.diag([30, 10])

    P = Q

    x0, u0 = np.zeros(n), np.zeros(m)

    traj_type = 'circle'
    trajectory = generate_trajectory(traj_type, 50, dt)
    
    trajectory_index = 0
    trajectory_x = trajectory[:, 0]
    trajectory_y = trajectory[:, 1]
    trajectory_psi = trajectory[:, 5]

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



    feedback_linearizer = FeedbackLinearization()

    rate = rospy.Rate(10)


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

            x0 = np.array([position[0], position[1], position[2], roll, pitch, yaw])
            
            start_time = time.time()
            u_des = linear_MPC(Npred, x0, u0, n, m, dt, Q, R, umin, umax, xref)

            u_body = FeedbackLinearization.transform_world_to_body(
                p_dot_des=u_des[:3],
                phi_dot_des=u_des[3],
                theta_dot_des=u_des[4],
                psi_dot_des=u_des[5],
                phi=roll, theta=pitch, psi=yaw
            )
            end_time = time.time()

            print(start_time - end_time)
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


            move_robot(*u_body)
            u0 = u_des

            robot_path_x.append(position[0])
            robot_path_y.append(position[1])
            robot_plot.set_data(robot_path_x, robot_path_y)

            ax.relim()
            ax.autoscale_view()
            plt.draw()
            plt.pause(0.001)

            trajectory_index += 1
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down.")


if __name__ == '__main__':
    main()
