#!/usr/bin/python3

import matplotlib.pyplot as plt
import numpy as np
from tf.transformations import euler_from_quaternion
from get_robot_info import get_robot_coordinates
from give_command import move_robot
from create_dynamics_function import create_full_dynamics_function
from MPC import MPC
from scipy.ndimage import gaussian_filter
import rospy

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



import numpy as np
from scipy.ndimage import gaussian_filter


def generate_trajectory(traj_type, duration, dt):
    t = np.arange(0, duration, dt)
    N = len(t)

    z_height = 0.3  # Constant height for the robot base
    roll = np.zeros(N)
    pitch = np.zeros(N)

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

    # Z is constant
    z = np.full(N, z_height)

    # Final Trajectory (N x 6): [x, y, z, roll, pitch, yaw]
    trajectory = np.stack((x, y, z, roll, pitch, psi), axis=1)

    return trajectory


def main():
    rospy.init_node('robot_coordinate_listener', anonymous=True)

    # MPC parameters
    Npred, dt, n, m = 15, 0.1, 6, 6

    xmin, xmax = np.array([-np.inf, -np.inf, 0.1, -np.pi/8, -np.pi/8, -np.inf]), \
                 np.array([np.inf, np.inf, 0.4, np.pi/8, np.pi/8, np.inf])  
    x0, u0 = np.zeros(n), np.zeros(m)  # State initialize


    Q = np.diag([10, 10, 5, 1, 1, 10])  
    R = np.diag([30, 10, 1, 1, 1, 30])
    # Q = np.diag([50, 50, 5, 1, 1, 50])      
    R = np.diag([30, 10, 1, 1, 1, 30])

    P = Q


    umin = np.array([-0.5, -0.2, -0.3, -0.5, -0.5, -0.3])
    umax = np.array([ 1.5,  0.2,  0.3,  0.5,  0.5,  0.3])

    delta_u_min = np.array([-0.3, -0.2, -0.1, -0.3, -0.3, -0.4])
    delta_u_max = np.array([ 0.3,  0.2,  0.1,  0.3,  0.3,  0.4])

    traj_type = 'circle'
    trajectory = generate_trajectory(traj_type, 50, dt)


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



    rate = rospy.Rate(10)
    f = create_full_dynamics_function()
    trajectory_index = 0

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
            
            # ref_x, ref_y, _, _, _, ref_psi = trajectory[min(trajectory_index, len(trajectory) - 1)]
            # tracking_error = np.sqrt((position[0] - ref_x)**2 + (position[1] - ref_y)**2)
            # tracking_errors = np.append(tracking_errors,tracking_error)

            trajectory_index += 1
            x0 = np.array([position[0], position[1], position[2], roll, pitch, yaw])
            
            try:
                u = MPC(Npred, x0, u0, n, m, dt, f, umin, umax, xmin, xmax, delta_u_min, delta_u_max, Q, R, P, xref)
            except RuntimeError as e:
                rospy.logerr(f"MPC solver failed: {e}")
                print(x0)
                continue


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

            move_robot(u[0], u[1], u[2],u[3], u[4], u[5])
            u0 = u
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down.")

if __name__ == '__main__':
    main()
