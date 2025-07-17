#!/usr/bin/env python3

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "."))

import math
import rospy
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Path
from give_command import move_robot
from create_dynamics_function import create_dynamics_function
from MPC import MPC

planned_trajectory = []

def planned_path_callback(msg):
    global planned_trajectory
    planned_trajectory = [
        (pose.pose.position.x, pose.pose.position.y, math.atan2(
            pose.pose.orientation.z, pose.pose.orientation.w
        )) for pose in msg.poses
    ]

def generate_trajectory_from_path(planned_trajectory, dt):
    if not planned_trajectory:
        rospy.logwarn("No trajectory available!")
        return np.array([])

    trajectory = []
    for i, point in enumerate(planned_trajectory):
        x, y, yaw = point
        phi = 0       # Placeholder: roll angle (adjust if needed)
        phi_dot = 0   # Placeholder: roll angular velocity (adjust if needed)
        trajectory.append([x, y, yaw, phi, phi_dot])
    return np.array(trajectory)

def plot_trajectory(xref):
    plt.figure("Planned Trajectory in X-Y Space")
    plt.clf()
    plt.plot(xref[0, :], xref[1, :], marker="o", label="Planned Path")
    plt.xlabel("X Position (m)")
    plt.ylabel("Y Position (m)")
    plt.title("Planned Trajectory")
    plt.legend()
    plt.grid()
    plt.pause(0.1)

if __name__ == '__main__':
    rospy.init_node('robot_controller', anonymous=True)

    rospy.Subscriber('/planned_path', Path, planned_path_callback)
    rate = rospy.Rate(10)

    f = create_dynamics_function()
    Npred, dt = 30, 0.1
    n, m = 5, 3

    x0 = np.array([0, 0, 0, 0, 0])
    u0 = np.array([0, 0, 0])

    Q = np.diag([60, 60, 10, 200, 100])
    R = np.diag([15, 15, 20])
    P = Q

    umin, umax = np.array([-1.5, -0.3, -10]), np.array([1.5, 0.3, 10])
    xmin, xmax = np.array([-100, -100, -np.inf, -0.15, -1.0]), np.array([100, 100, np.inf, 0.15, 1.0])
    delta_u_min, delta_u_max = np.array([-0.2, -0.15, -1]), np.array([0.2, 0.15, 1])

    plt.ion()

    try:
        while not rospy.is_shutdown():
            if not planned_trajectory:
                rospy.logwarn("Waiting for planned trajectory...")
                rate.sleep()
                continue

            trajectory = generate_trajectory_from_path(planned_trajectory, dt)
            xref = trajectory.T

            plot_trajectory(xref)

            try:
                u = MPC(Npred, x0, u0, n, m, dt, f, umin, umax, xmin, xmax, delta_u_min, delta_u_max, Q, R, P, xref)
            except RuntimeError as e:
                rospy.logerr(f"MPC solver failed: {e}")
                continue

            move_robot(u[0], u[1], u[2])
            u0 = u

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
