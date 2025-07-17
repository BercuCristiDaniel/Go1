#!/usr/bin/env python3

import rospy
from tf.transformations import euler_from_quaternion
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from get_robot_info import get_robot_coordinates
from give_command import move_robot
from create_dynamics_function import create_dynamics_function
from MPC import MPC
from Astar import AStarPlanner
from scipy.interpolate import CubicSpline
import os

# Global variable to store the planned path
planned_path = []

def stop_move_base():
    """Stops the move_base node if it is running."""
    nodes = os.popen("rosnode list").read().strip().split("\n")
    if "/move_base" in nodes:
        os.system("rosnode kill /move_base")
        rospy.loginfo("move_base node has been stopped.")
    else:
        rospy.loginfo("move_base node is not running.")

def check_cmd_vel_subscribers():
    """Check and log the subscribers of the /cmd_vel topic."""
    info = os.popen("rostopic info /cmd_vel").read()
    rospy.loginfo(f"/cmd_vel topic info:\n{info}")

def path_callback(msg):
    """Callback function for receiving the planned path."""
    global planned_path
    planned_path = msg.poses
    rospy.loginfo(f"Received planned path with {len(planned_path)} waypoints")

def generate_trajectory_from_astar(astar_path_x, astar_path_y, duration, dt):
    if len(astar_path_x) < 2 or len(astar_path_y) < 2:
        raise ValueError("A* path is too short to generate trajectory.")

    # Compute original time vector for A* waypoints
    t_original = np.linspace(0, duration, len(astar_path_x))

    # New time vector for interpolation
    t_interpolated = np.arange(0, duration, dt)

    # Spline interpolation for smooth trajectories
    spline_x = CubicSpline(t_original, astar_path_x, bc_type='clamped')
    spline_y = CubicSpline(t_original, astar_path_y, bc_type='clamped')

    # Interpolated points
    interp_x = spline_x(t_interpolated)
    interp_y = spline_y(t_interpolated)

    # Compute yaw angles
    dx = np.gradient(interp_x, dt)
    dy = np.gradient(interp_y, dt)
    yaw = np.arctan2(dy, dx)

    # Stack trajectory into [x, y, yaw, 0, 0]
    zero_states = np.zeros((len(t_interpolated), 2))
    trajectory = np.hstack((np.vstack((interp_x, interp_y, yaw)).T, zero_states))

    return trajectory

def calculate_trajectory_duration_from_points(astar_path_x, astar_path_y, max_speed, safety_factor=1.5):
    """Calculate the duration of the trajectory."""
    if len(astar_path_x) < 2 or len(astar_path_y) < 2:
        raise ValueError("Path must have at least two points.")

    distances = np.sqrt(np.diff(astar_path_x)**2 + np.diff(astar_path_y)**2)
    total_length = np.sum(distances)
    base_duration = total_length / max_speed
    return base_duration * safety_factor

if __name__ == '__main__':
    rospy.init_node('robot_controller', anonymous=True)

    # Stop move_base or remap cmd_vel topic to prevent conflicts
    stop_move_base()
    check_cmd_vel_subscribers()

    # MPC parameters
    Npred, dt, n, m = 30, 0.1, 5, 3
    astar = AStarPlanner()
    xmin, xmax = np.array([-np.inf, -np.inf, -np.inf, -0.15, -1.0]), np.array([np.inf, np.inf, np.inf, 0.15, 1.0])  # State constraints
    x0, u0 = np.array([0, 0, 0, 0, 0]), np.array([0, 0, 0])  # Initial state and control

    # MPC tuning
    Q = np.diag([60, 60, 10, 1, 1])  
    R = np.diag([15, 15, 50])        
    P = Q

    umin = np.array([-1.5, -0.3, -10])   # Input constraints
    umax = np.array([1.5, 0.3, 10])

    delta_u_min = np.array([-0.2, -0.1, -0.5])  # Rate of change constraints
    delta_u_max = np.array([0.2, 0.1, 0.5])

    rospy.Subscriber('/planned_path', Path, path_callback)  # Subscriber for planned path

    rate = rospy.Rate(20)
    f = create_dynamics_function()
    trajectory_index = 0
    trajectory_extended = False  # Flag to prevent repeated trajectory extension
    original_trajectory = None  # Store the original trajectory

    # Initialize live plotting
    plt.ion()
    fig, ax = plt.subplots()
    robot_path, = ax.plot([], [], "b-", label="Robot Path")
    reference_path, = ax.plot([], [], "g--", label="Reference Path")

    ax.set_title("Live Robot and Trajectory Plot")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.legend()
    ax.grid()

    robot_x_data = []
    robot_y_data = []
    trajectory_x_data = []
    trajectory_y_data = []
    trajectory_offset_applied = False
    try:
        while not rospy.is_shutdown():
            if not planned_path:
                rospy.logwarn("Waiting for planned path...")
                move_robot(0, 0, 0)
                rate.sleep()
                continue

            if astar.map_data is None:
                rospy.logwarn("Waiting for map...")
                rate.sleep()
                continue

            # Get robot coordinates and orientation
            position, orientation = get_robot_coordinates()
            roll, pitch, yaw = euler_from_quaternion(orientation)

            # Extract planned path

            if not trajectory_offset_applied:
                # Adjust trajectory to start at the robot's current position
                scale_factor = 0.85  # Example: each graph point represents 0.1 meters

                # Scale the planned path to real-world coordinates
                astar_path_x = np.array([p.pose.position.x for p in planned_path]) * scale_factor
                astar_path_y = np.array([p.pose.position.y for p in planned_path]) * scale_factor

                # Generate trajectory
                duration = calculate_trajectory_duration_from_points(astar_path_x, astar_path_y, 7.5)
                trajectory = generate_trajectory_from_astar(astar_path_x, astar_path_y, duration, dt)
                position, orientation = get_robot_coordinates()
                roll, pitch, yaw = euler_from_quaternion(orientation)

                # Extract the first point of the trajectory
                trajectory_start = trajectory[0, :2]

                # Calculate the offset
                offset_x = position[0] - trajectory_start[0]
                offset_y = position[1] - trajectory_start[1]

                # Apply the offset to the trajectory
                trajectory[:, 0] += offset_x
                trajectory[:, 1] += offset_y

                rospy.loginfo(f"Trajectory shifted by offset: ({offset_x}, {offset_y})")

                # Set the flag to prevent repeated offsetting
                trajectory_offset_applied = True

            if len(trajectory) == 0:
                rospy.logwarn("Empty trajectory generated. Skipping control step.")
                continue


            # Extend trajectory with stabilization points if not already extended
            if not trajectory_extended:
                num_prepended_points = 20  # Number of additional points to prepend
                first_point = trajectory[0]  # First point of the trajectory
                prepended_points = np.tile(first_point, (num_prepended_points, 1))  # Duplicate the first point
                trajectory = np.vstack((prepended_points, trajectory))  # Prepend the points

                rospy.loginfo(f"Prepended {num_prepended_points} points to the trajectory.")

                # Set the flag to prevent repeated prepending
                trajectory_extended = True

            # Update live plot data
            robot_x_data.append(position[0])
            robot_y_data.append(position[1])

            if len(trajectory_x_data) == 0:  # Initialize trajectory plot
                trajectory_x_data = trajectory[:, 0].tolist()
                trajectory_y_data = trajectory[:, 1].tolist()

            # Update live plot
            robot_path.set_data(robot_x_data, robot_y_data)
            reference_path.set_data(trajectory_x_data, trajectory_y_data)


            ax.relim()
            ax.autoscale_view()
            plt.draw()
            plt.pause(0.001)

            # Prepare trajectory reference for MPC
            xref = []
            for k in range(Npred + 1):
                idx = min(trajectory_index + k, len(trajectory) - 1)
                xref.append(trajectory[idx])
            xref = np.array(xref).T

            trajectory_index = min(trajectory_index + 1, len(trajectory) - 1)
            x0 = np.array([position[0], position[1], yaw, roll, 0])

            # Solve MPC
            try:
                u = MPC(Npred, x0, u0, n, m, dt, f, umin, umax, xmin, xmax, delta_u_min, delta_u_max, Q, R, P, xref)
            except RuntimeError as e:
                rospy.logerr(f"MPC solver failed: {e}")
                continue

            # Send control commands
            move_robot(u[0], u[1], u[2])
            u0 = u

            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down.")