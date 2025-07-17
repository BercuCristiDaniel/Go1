#!/usr/bin/python3


import sys
sys.path.append('/root/ros_ws/src/foot_pos_control/src')

import numpy as np
from tf.transformations import euler_from_quaternion
from give_command import move_robot
from feedback_linearization import FeedbackLinearization
from geometry_msgs.msg import Twist
from LMPC import linear_MPC
import rospy
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter
import scipy.io
import time
from robot_state import Go1RobotData

from std_msgs.msg import Bool

robot_ready = False

def ready_callback(msg):
    global robot_ready
    robot_ready = msg.data

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

def save_to_mat(filename, t, state_xi, pos_ref):
    data = {
        't': t,
        'state_xi': state_xi,
        'pos_ref': pos_ref
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
            psi_segment = np.full(segment_steps, np.arctan2(y_end - y_start, x_end - x_start) ) 
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



def compute_command(u_global, yaw):
    """
    Transforms global control inputs to robot-frame velocities.

    Parameters:
        u_global : np.ndarray
            A (3,) array of global input [u_ax, u_ay, u_av]
        yaw : float
            The robot's orientation (theta) in radians

    Returns:
        np.ndarray: A (3,) array [v_rx, v_ry, omega_r]
    """
    u_ax, u_ay, u_av = u_global
    R = np.array([
        [np.cos(yaw),  np.sin(yaw), 0],
        [-np.sin(yaw), np.cos(yaw), 0],
        [0,            0,           1]
    ])
    u_global_vec = np.array([u_ax, u_ay, u_av])
    u_body = R @ u_global_vec
    return u_body

def stop_robot():
    try:
        rospy.loginfo("Stopping robot before shutdown...")
        move_robot(0, 0, 0)
        rospy.sleep(0.1)
    except Exception as e:
        rospy.logwarn(f"Failed to stop robot: {e}")

def save_to_mat(filename, t, state_xi, pos_ref):
    data = {
        't': t,
        'state_xi': state_xi,
        'pos_ref': pos_ref
    }
    scipy.io.savemat(filename, data)
    print(f"Data saved to {filename}.")

def main():
    rospy.init_node('mpc_planner_node', anonymous=True)
    pub = rospy.Publisher("/desired_velocity", Twist, queue_size=1)
    rospy.Subscriber("/robot_ready", Bool, ready_callback)  # Subscribe to robot readiness

    Npred, dt, n, m = 7, 0.1, 3, 3
    xmin = np.array([-np.inf]*n)
    xmax = np.array([np.inf]*n)
    Q = np.diag([120, 120, 10])
    R = np.diag([55, 55, 1])

    P = Q

    state_caller = Go1RobotData()

    x0, u0 = np.zeros(n), np.zeros(m)
    traj_type = 'square'
    trajectory = generate_trajectory(traj_type, 50, dt)
    trajectory_x, trajectory_y, trajectory_psi = trajectory[:, 0], trajectory[:, 1], trajectory[:, 2]
    plot_time = np.arange(0, len(trajectory_x) * dt, dt)

    plt.ion()
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, figsize=(10, 16))

    ax1.plot(trajectory_x, trajectory_y, 'r--', label='Reference Trajectory')
    robot_path_x, robot_path_y = [], []
    robot_plot, = ax1.plot([], [], 'b-', label='Robot Trajectory')
    ax1.legend()
    ax1.set_xlabel('X Position')
    ax1.set_ylabel('Y Position')
    ax1.set_title('Robot Trajectory in X-Y Space')
    ax1.grid()

    ax2.plot(plot_time, trajectory_x, 'g--', label='Reference X(t)')
    robot_time, robot_time_x = [], []
    robot_x_plot, = ax2.plot([], [], 'b-', label='Robot X(t)')
    ax2.legend()
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('X Position')
    ax2.set_title('X Position Over Time')
    ax2.grid()

    ax3.plot(plot_time, trajectory_y, 'c--', label='Reference Y(t)')
    robot_time_y = []
    robot_y_plot, = ax3.plot([], [], 'b-', label='Robot Y(t)')
    ax3.legend()
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Y Position')
    ax3.set_title('Y Position Over Time')
    ax3.grid()

    ax4.plot(plot_time, trajectory_psi, 'm--', label='Reference Ψ(t)')
    robot_time_psi = []
    robot_psi_plot, = ax4.plot([], [], 'b-', label='Robot Ψ(t)')
    ax4.legend()
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Yaw Angle (rad)')
    ax4.set_title('Yaw Angle Over Time')
    ax4.grid()

    yaw_unwrapper = YawUnwrapper()
    rate = rospy.Rate(10)
    trajectory_index = 0

    L = np.array([
        [1.0, 0.0, 0.0],
        [0.7071, 0.7071, 0.0],
        [0.0, 1.0, 0.0],
        [-0.7071, 0.7071, 0.0],
        [-1.0, 0.0, 0.0],
        [-0.7071, -0.7071, 0.0],
        [0.0, -1.0, 0.0],
        [0.7071, -0.7071, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, -1.0]
    ])
    b = np.array([
        0.6535,
        0.6541,
        0.6535,
        0.6541,
        0.6535,
        0.6541,
        0.6535,
        0.6541,
        0.5,
        0.5
    ])

    # === Wait for /robot_ready to be True ===
    rospy.loginfo("Waiting for robot to be ready...")
    while not rospy.is_shutdown() and not robot_ready:
        rospy.loginfo_throttle(2.0, "[HIGH LEVEL] Waiting for /robot_ready...")
        rate.sleep()
    rospy.loginfo("Robot is ready. Starting trajectory tracking...")

    try:
        while not rospy.is_shutdown() and trajectory_index < len(trajectory):
            _, position, _, orientation, _, _ = state_caller.get_robot_data()
            _, _, yaw = euler_from_quaternion(orientation)
            yaw = yaw_unwrapper.unwrap(yaw)

            xref = np.array([trajectory[min(trajectory_index + k, len(trajectory) - 1)] for k in range(Npred + 1)]).T
            x0 = np.array([position[0], position[1], yaw])

            start_time = time.time()
            u_des = linear_MPC(Npred, x0, u0, n, m, dt, Q, R, xmin, xmax, xref, L, b)
            end_time = time.time()

            print(end_time - start_time)

            u_body = compute_command(u_des, yaw)
            msg = Twist()
            msg.linear.x = u_body[0]
            msg.linear.y = u_body[1]
            msg.angular.z = u_body[2]

            pub.publish(msg)
            u0 = u_des

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

            for ax in [ax1, ax2, ax3, ax4]:
                ax.relim()
                ax.autoscale_view()

            plt.draw()
            plt.pause(0.001)

            trajectory_index += 1
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt detected.")
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard Interrupt detected. Stopping robot.")
        stop_robot()
    finally:
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.angular.z = 0

        pub.publish(msg)
        rospy.loginfo("Saving data and exiting program.")
        t_array = np.array(robot_time)
        state_xi_array = np.vstack((robot_time_x, robot_time_y, robot_time_psi)).T
        ref_indices = np.clip((t_array / dt).astype(int), 0, len(trajectory) - 1)
        pos_ref_array = trajectory[ref_indices]
        save_to_mat("linear_mpc_trajectory_log_circle_full_control.mat", t_array, state_xi_array, pos_ref_array)

if __name__ == '__main__':
    main()
