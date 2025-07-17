#!/usr/bin/env python3

import sys
sys.path.append('/home/cristi/Desktop/custom_ws/src/foot_pos_control/src')

import rospy
import time
import numpy as np
from pathlib import Path
import pinocchio as pin
from std_msgs.msg import Bool, Empty
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from inverse_kinematics import Go1LegDynamics
from robot_state import Go1RobotData
from bezier_curves import generate_all_leg_trajectories, generate_rise_trajectories
from give_low_cmd import send_motor_commands_from_msgs
from unitree_legged_msgs.msg import MotorCmd
import scipy.io
import threading

# Shared global velocity (updated from /desired_velocity)
latest_velocity = np.zeros(3)
should_save_data = False
trajectories_generated = False  # Flag to track if trajectories have been generated

motor_publishers = {}

def velocity_callback(msg):
    global latest_velocity
    latest_velocity[0] = msg.linear.x
    latest_velocity[1] = msg.linear.y
    latest_velocity[2] = msg.angular.z

def save_data_callback(msg):
    global should_save_data
    should_save_data = True

def save_to_mat(filename, t, state_xi, pos_ref, tau=None):
    data = {
        't': t,
        'state_xi': state_xi,
        'pos_ref': pos_ref
    }
    if tau is not None:
        data['tau'] = tau
    scipy.io.savemat(filename, data)
    print(f"Data saved to {filename}.")

def reorder_joint_vector(q_raw):
    ordered_indices = [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]
    return q_raw[ordered_indices]

# Send motor commands for a specific leg
def send_motor_commands_for_leg(leg_name, cmd_msgs):
    JOINT_ORDER = [
        "motor_3", "motor_4", "motor_5",   # FL
        "motor_0", "motor_1", "motor_2",   # FR
        "motor_9", "motor_10", "motor_11", # RL
        "motor_6", "motor_7", "motor_8"    # RR
    ]

    leg_joint_map = {
        "front_left":  [0, 1, 2],
        "front_right": [3, 4, 5],
        "rear_left":   [6, 7, 8],
        "rear_right":  [9, 10, 11]
    }

    leg_motor_indices = leg_joint_map[leg_name]

    # Create MotorCmd messages for the current leg
    leg_cmd_msgs = [MotorCmd() for _ in range(3)]
    for idx, joint_idx in enumerate(leg_motor_indices):
        leg_cmd_msgs[idx].mode = 0x0A
        leg_cmd_msgs[idx].tau = cmd_msgs[joint_idx].tau
        leg_cmd_msgs[idx].q = cmd_msgs[joint_idx].q
        leg_cmd_msgs[idx].dq = cmd_msgs[joint_idx].dq
        leg_cmd_msgs[idx].Kp = cmd_msgs[joint_idx].Kp
        leg_cmd_msgs[idx].Kd = cmd_msgs[joint_idx].Kd

    # Send the motor commands for the specific leg
    for joint, msg in zip(leg_motor_indices, leg_cmd_msgs):
        motor_publishers[JOINT_ORDER[joint]].publish(msg)

# Initialize motor publishers for each joint
def init_motor_publishers():
    global motor_publishers
    JOINT_ORDER = [
        "motor_3", "motor_4", "motor_5",   # FL
        "motor_0", "motor_1", "motor_2",   # FR
        "motor_9", "motor_10", "motor_11", # RL
        "motor_6", "motor_7", "motor_8"    # RR
    ]
    
    for joint in JOINT_ORDER:
        topic = f"/{joint}/cmd"
        motor_publishers[joint] = rospy.Publisher(topic, MotorCmd, queue_size=1)
    rospy.sleep(0.1)  # Give some time to establish the publishers

def main():
    global should_save_data
    rospy.init_node('locomotion_control', anonymous=True)
    rospy.Subscriber("/desired_velocity", Twist, velocity_callback)
    rospy.Subscriber("/save_data_trigger", Empty, save_data_callback)
    rate = rospy.Rate(250)

    URDF_PATH = "/home/cristi/Desktop/custom_ws/src/foot_pos_control/robots/go1_description/urdf/go1.urdf"
    model = pin.buildModelFromUrdf(URDF_PATH)
    data = model.createData()

    leg_joint_map = {
        "front_left":  [0, 1, 2],
        "front_right": [3, 4, 5],
        "rear_left":   [6, 7, 8],
        "rear_right":  [9, 10, 11]
    }

    foot_frame_names = {
        "front_left":  "FL_foot",
        "front_right": "FR_foot",
        "rear_left":   "RL_foot",
        "rear_right":  "RR_foot"
    }

    state_caller = Go1RobotData()
    Kx = np.diag([1000, 1000, 1000])
    Bx = np.diag([44, 44, 44])

    phase = "rise"
    rise_traj, rise_vel, rise_acc = generate_rise_trajectories()
    rise_index = 0
    rise_length = len(next(iter(rise_traj.values())))

    traj_index = 0
    legs_traj, legs_vel, legs_acc = {}, {}, {}

    log_time = []
    log_actual_pos = {leg: [] for leg in leg_joint_map}
    log_desired_pos = {leg: [] for leg in leg_joint_map}
    log_tau = {leg: [] for leg in leg_joint_map}

    try:
        while not rospy.is_shutdown():
            if phase == "rise":
                legs_traj, legs_vel, legs_acc = rise_traj, rise_vel, rise_acc
                traj_length = len(next(iter(legs_traj.values())))
                traj_index = rise_index
                rise_index += 1
                if rise_index >= rise_length:
                    phase = "walk"
                    traj_index = 0
                    legs_traj, legs_vel, legs_acc = generate_all_leg_trajectories(-0.3, 0.0, -0.3, "trot")
                    traj_length = len(next(iter(legs_traj.values())))

            joint_state, _, _, base_orientation, _, _ = state_caller.get_robot_data()
            q = reorder_joint_vector(np.array(joint_state.position))
            qdot = reorder_joint_vector(np.array(joint_state.velocity))

            x, y, z, w = base_orientation
            quat = pin.Quaternion(w, x, y, z)
            R_base = quat.toRotationMatrix()

            cmd_msgs = [MotorCmd() for _ in range(12)]

            count = 0
            for leg_name in leg_joint_map:
                leg_dyn = Go1LegDynamics(model, data, foot_frame_names[leg_name])
                x = leg_dyn.forward_kinematics(q)
                x_world = R_base @ x

                J = leg_dyn.jacobian(q)
                M = leg_dyn.mass_matrix(q)
                C = leg_dyn.coriolis_matrix(q, qdot)
                g_full = leg_dyn.gravity_vector(q)
                Jdot = leg_dyn.jacobian_dot(q, qdot)

                q_leg = q[count * 3:count * 3 + 3]
                qdot_leg = qdot[count * 3:count * 3 + 3]
                M_leg = M[count * 3:count * 3 + 3, count * 3:count * 3 + 3]
                C_leg = C[count * 3:count * 3 + 3, count * 3:count * 3 + 3]
                g_leg = g_full[count * 3:count * 3 + 3]
                J_leg = J[0:3, count * 3:count * 3 + 3]
                Jdot_leg = Jdot[0:3, count * 3:count * 3 + 3]

                xdot_leg = J_leg @ qdot_leg
                x_des_leg = legs_traj[leg_name][traj_index]
                xdot_des_leg = legs_vel[leg_name][traj_index]
                xddot_des_leg = legs_acc[leg_name][traj_index]

                q_des_full = leg_dyn.inverse_kinematics_pinocchio(
                    model, data,
                    model.getFrameId(foot_frame_names[leg_name]),
                    q.copy(), x_des_leg
                )
                q_des_leg = q_des_full[count * 3:count * 3 + 3]
                qdot_des_leg = np.linalg.pinv(J_leg) @ xdot_des_leg

                Kq = J_leg.T @ Kx @ J_leg
                Bq = J_leg.T @ Bx @ J_leg

                tau_ff = (
                    M_leg @ np.linalg.pinv(J_leg) @ (xddot_des_leg - Jdot_leg @ qdot_leg)
                    + C_leg @ qdot_leg
                    + g_leg
                )
                tau_ff = np.clip(tau_ff, -21.7, 21.7)

                tau_c = []
                for j in range(3):
                    tau_pd = Kq[j, j] * (q_des_leg[j] - q_leg[j]) + Bq[j, j] * (qdot_des_leg[j] - qdot_leg[j])
                    tau_total = tau_ff[j] + tau_pd
                    tau_c.append(tau_total)

                    idx = count * 3 + j
                    cmd_msgs[idx].mode = 0x0A
                    cmd_msgs[idx].tau = tau_ff[j]
                    cmd_msgs[idx].q = q_des_leg[j]
                    cmd_msgs[idx].dq = qdot_des_leg[j]
                    cmd_msgs[idx].Kp = Kq[j, j]
                    cmd_msgs[idx].Kd = Bq[j, j]

                if count == 0:
                    log_time.append(rospy.Time.now().to_sec())

                log_actual_pos[leg_name].append(x_world.copy())
                log_desired_pos[leg_name].append(R_base @ x_des_leg)
                log_tau[leg_name].append(tau_c)

                count += 1

            send_motor_commands_from_msgs(cmd_msgs)

            if phase == "walk":
                traj_index = (traj_index + 1) % traj_length
            if should_save_data:
                def save_all_data():
                    try:
                        t_arr = np.array(log_time)
                        for leg in leg_joint_map:
                            if len(log_actual_pos[leg]) == 0:
                                continue
                            state_xi_arr = np.vstack(log_actual_pos[leg])
                            pos_ref_arr = np.vstack(log_desired_pos[leg])
                            tau_arr = np.vstack(log_tau[leg])
                            filename = f"/home/cristi/Desktop/{leg}_trajectory_walk.mat"
                            save_to_mat(filename, t_arr, state_xi_arr, pos_ref_arr, tau_arr)
                    except Exception as e:
                        rospy.logerr(f"Error saving data: {e}")

                threading.Thread(target=save_all_data).start()
                should_save_data = False    

            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down.")

if __name__ == '__main__':
    init_motor_publishers()
    main()
