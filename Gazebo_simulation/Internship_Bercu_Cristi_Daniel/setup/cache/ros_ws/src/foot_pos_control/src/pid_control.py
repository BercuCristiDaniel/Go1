#!/usr/bin/env python3

import rospy
import numpy as np
from pathlib import Path
import pinocchio as pin
import matplotlib.pyplot as plt
from collections import defaultdict
import time
from get_robot_info import get_robot_coordinates
from inverse_kinematics import Go1LegDynamics
from robot_state import Go1RobotData
from bezier_curves import generate_rise_trajectories, generate_all_leg_trajectories
from give_low_cmd import send_motor_commands_from_msgs
from unitree_legged_msgs.msg import MotorCmd
from tf.transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R
from balanceControl import BalanceController

def inverse_kinematics_pinocchio(model, data, frame_id, q_init, x_des, max_iters=40, eps=1e-4, alpha=0.5):
    q = q_init.copy()
    for i in range(max_iters):
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacement(model, data, frame_id)
        x_curr = data.oMf[frame_id].translation
        err = x_des - x_curr

        if np.linalg.norm(err) < eps:
            return q

        J = pin.computeFrameJacobian(model, data, q, frame_id, pin.LOCAL_WORLD_ALIGNED)[0:3, :]
        dq = alpha * np.linalg.pinv(J) @ err
        q[:len(dq)] += dq

    print("IK did not converge!")
    return q

def reorder_joint_vector(q_raw):
    def reorder_leg(j0, j1, j2):
        return [j1, j2, j0]
    indices = (
        reorder_leg(3, 4, 5) +
        reorder_leg(0, 1, 2) +
        reorder_leg(9, 10, 11) +
        reorder_leg(6, 7, 8)
    )
    return q_raw[indices]

def get_reordered_leg_joint_map():
    return {
        "front_left":  [0, 1, 2],
        "front_right": [3, 4, 5],
        "rear_left":   [6, 7, 8],
        "rear_right":  [9, 10, 11]
    }

def align(model, data, state_caller, foot_frame_names, align_positions, leg_joint_map):
    rate = rospy.Rate(100)
    joint_state, _, _, _, _ = state_caller.get_robot_data()
    q_current = reorder_joint_vector(np.array(joint_state.position))
    q_align = q_current.copy()

    for leg_name, x_target in align_positions.items():
        foot_id = model.getFrameId(foot_frame_names[leg_name])
        q_sol = inverse_kinematics_pinocchio(model, data, foot_id, q_align.copy(), x_target)
        idxs = leg_joint_map[leg_name]
        q_align[idxs] = q_sol[idxs]

    for i in range(100):
        alpha = i / 100.0
        q_interp = (1 - alpha) * q_current + alpha * q_align

        cmd_msgs = [MotorCmd() for _ in range(12)]
        for leg_name, idxs in leg_joint_map.items():
            for j in range(3):
                joint_index = idxs[j]
                cmd = cmd_msgs[joint_index]
                cmd.mode = 0x0A
                cmd.q = float(q_interp[joint_index])
                cmd.dq = 0.0
                cmd.Kp = 100.0
                cmd.Kd = 5.0
                cmd.tau = 0.0

        send_motor_commands_from_msgs(cmd_msgs)
        rate.sleep()


def rise(model, data, state_caller, foot_frame_names):
    rise_q_traj_cache, rise_qdot_traj_cache = {}, {}
    legs_traj, legs_vel, _ = generate_rise_trajectories()
    rise_length = len(next(iter(legs_traj.values())))
    leg_joint_map = get_reordered_leg_joint_map()
    rate = rospy.Rate(100)

    for index in range(rise_length):
        joint_state, _, _, _, _ = state_caller.get_robot_data()
        q = reorder_joint_vector(np.array(joint_state.position))
        qdot = reorder_joint_vector(np.array(joint_state.velocity))

        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacements(model, data)

        if index == 0:
            for leg_name in leg_joint_map:
                q_list, qd_list = [], []
                x_traj_local = legs_traj[leg_name]
                xdot_traj = legs_vel[leg_name]
                foot_name = foot_frame_names[leg_name]
                foot_id = model.getFrameId(foot_name)
                leg_dyn = Go1LegDynamics(model, data, foot_name)

                for i in range(len(x_traj_local)):
                    x_des_base = x_traj_local[i]
                    q_init = q.copy()
                    q_i = inverse_kinematics_pinocchio(model, data, foot_id, q_init, x_des_base)
                    q_full = q.copy()
                    q_full[leg_joint_map[leg_name]] = q_i[leg_joint_map[leg_name]]
                    J_i = leg_dyn.jacobian(q_full)[0:3, leg_joint_map[leg_name]]
                    qd_i = np.linalg.pinv(J_i) @ xdot_traj[i]
                    q_list.append(q_i[leg_joint_map[leg_name]])
                    qd_list.append(qd_i)

                rise_q_traj_cache[leg_name] = np.array(q_list)
                rise_qdot_traj_cache[leg_name] = np.array(qd_list)

        cmd_msgs = [MotorCmd() for _ in range(12)]
        for leg_name in leg_joint_map:
            idxs = leg_joint_map[leg_name]
            q_des_leg = rise_q_traj_cache[leg_name][index]
            qd_des_leg = rise_qdot_traj_cache[leg_name][index]

            for i in range(3):
                joint_index = idxs[i]
                cmd = cmd_msgs[joint_index]
                cmd.mode = 0x0A
                cmd.q = float(q_des_leg[i])
                cmd.dq = float(qd_des_leg[i])
                cmd.Kp = 50.0
                cmd.Kd = 4.0
                cmd.tau = 0.0

        send_motor_commands_from_msgs(cmd_msgs)
        rate.sleep()




# def walk(model, data, state_caller, foot_frame_names, vx, vy, omega_z):
#     import pinocchio as pin

#     walking_type = "trot"
#     leg_joint_map = get_reordered_leg_joint_map()
#     rate = rospy.Rate(100)

#     base_pos_correction = np.zeros(2)
#     all_leg_traj, all_leg_vel, _ = generate_all_leg_trajectories(vx, vy, omega_z, walking_type)
#     traj_len = len(next(iter(all_leg_traj.values())))
#     index = 0

#     while not rospy.is_shutdown():
#         joint_state, base_pos, base_rot, base_acc, forces = state_caller.get_robot_data()
#         q = reorder_joint_vector(np.array(joint_state.position))



#         traj_idx = index % traj_len
#         cmd_msgs = [MotorCmd() for _ in range(12)]

#         for leg_name in leg_joint_map:
#             idxs = leg_joint_map[leg_name]
#             foot_name = foot_frame_names[leg_name]
#             foot_id = model.getFrameId(foot_name)
#             leg_dyn = Go1LegDynamics(model, data, foot_name)

#             x_des_base = all_leg_traj[leg_name][traj_idx].copy()

#             q_leg_full = inverse_kinematics_pinocchio(model, data, foot_id, q.copy(), x_des_base, max_iters=20)
#             q_leg = q_leg_full[idxs]

#             q_full = q.copy()
#             q_full[idxs] = q_leg
#             J_i = leg_dyn.jacobian(q_full)[0:3, idxs]
#             xdot_traj = all_leg_vel[leg_name][traj_idx]
#             qd_i = np.linalg.pinv(J_i) @ xdot_traj

#             for i in range(3):
#                 joint_index = idxs[i]
#                 cmd = cmd_msgs[joint_index]
#                 cmd.mode = 0x0A
#                 cmd.q = float(q_leg[i])
#                 cmd.dq = float(qd_i[i])
#                 cmd.Kp = 100.0
#                 cmd.Kd = 10.0
#                 cmd.tau = 0.0

#         send_motor_commands_from_msgs(cmd_msgs)
#         index += 1
#         rate.sleep()


def walk(model, data, state_caller, foot_frame_names, vx, vy, omega_z):

    walking_type = "trot"
    leg_joint_map = get_reordered_leg_joint_map()
    rate = rospy.Rate(100)

    all_leg_traj, all_leg_vel, _ = generate_all_leg_trajectories(vx, vy, omega_z, walking_type)
    traj_len = len(next(iter(all_leg_traj.values())))
    index = 0

    # Balance controller setup
    mass = 12.0
    Ib = np.diag([0.1, 0.2, 0.25])
    pcb = np.array([0.0, 0.0, 0.0])
    balance = BalanceController(mass, Ib, pcb)

    contact_pattern = {
        "front_left":  lambda t: int((t % 1.0) < 0.5),
        "front_right": lambda t: int((t % 1.0) >= 0.5),
        "rear_left":   lambda t: int((t % 1.0) >= 0.5),
        "rear_right":  lambda t: int((t % 1.0) < 0.5),
    }

    while not rospy.is_shutdown():
        joint_state, base_pos, base_rot, base_acc, forces = state_caller.get_robot_data()
        q = reorder_joint_vector(np.array(joint_state.position))
        gyro = np.array(joint_state.velocity[-3:])  # assuming last 3 are base angular velocity
        traj_idx = index % traj_len
        t_norm = traj_idx / traj_len

        cmd_msgs = [MotorCmd() for _ in range(12)]

        # === Compute rotation and orientation
        rotM = np.array(base_rot)  # base_rot is already a 3x3 matrix
        quat_for_euler = R.from_matrix(rotM).as_quat()
        roll, pitch, yaw = euler_from_quaternion(quat_for_euler)

        # === Balance correction terms
        ddPcd = np.array([
            -20.0 * pitch - 3.0 * gyro[1],  # pitch correction
            -20.0 * roll  - 3.0 * gyro[0],  # roll correction
            0.0
        ])
        dWbd = np.array([
            0.0,
            0.0,
            -5.0 * yaw - 1.0 * gyro[2]
        ])

        # === Build feet pos + contact mask
        feet_pos_body = []
        contact = []

        for leg_name in ["front_left", "front_right", "rear_left", "rear_right"]:
            foot_id = model.getFrameId(foot_frame_names[leg_name])
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacement(model, data, foot_id)
            foot_pos_world = data.oMf[foot_id].translation
            foot_pos_body = rotM.T @ (foot_pos_world - base_pos)
            feet_pos_body.append(foot_pos_body)
            contact.append(contact_pattern[leg_name](t_norm))

        feet_pos_body = np.array(feet_pos_body)
        contact = np.array(contact)

        # === Balance controller
        foot_forces = balance.compute(ddPcd, dWbd, rotM, feet_pos_body, contact)

        # === Debug: print foot forces and torque sum
        F_total = np.sum(foot_forces, axis=0)
        torque_total = np.zeros(3)
        for i in range(4):
            r = feet_pos_body[i] - rotM @ pcb
            torque_total += np.cross(r, foot_forces[i])

        # print(f"[BalanceCtrl] F_total: {F_total.round(2)} N")
        # print(f"[BalanceCtrl] Torque_total: {torque_total.round(2)} Nm")
        # for i, f in enumerate(foot_forces):
        #     print(f"  Leg {i}: Force {f.round(2)} N")

        # === Generate motor commands
        for leg_name in leg_joint_map:
            idxs = leg_joint_map[leg_name]
            foot_id = model.getFrameId(foot_frame_names[leg_name])
            leg_dyn = Go1LegDynamics(model, data, foot_frame_names[leg_name])

            x_des_base = all_leg_traj[leg_name][traj_idx]
            q_leg_full = inverse_kinematics_pinocchio(model, data, foot_id, q.copy(), x_des_base)
            q_leg = q_leg_full[idxs]

            q_full = q.copy()
            q_full[idxs] = q_leg
            J_i = leg_dyn.jacobian(q_full)[0:3, idxs]
            xdot_traj = all_leg_vel[leg_name][traj_idx]
            qd_i = np.linalg.pinv(J_i) @ xdot_traj

            force_idx = list(leg_joint_map.keys()).index(leg_name)
            tau_ff = J_i.T @ foot_forces[force_idx] if contact[force_idx] else np.zeros(3)

            for i in range(3):
                joint_index = idxs[i]
                cmd = cmd_msgs[joint_index]
                cmd.mode = 0x0A
                cmd.q = float(q_leg[i])
                cmd.dq = float(qd_i[i])
                cmd.Kp = 100.0
                cmd.Kd = 10.0
                cmd.tau = float(tau_ff[i])

        send_motor_commands_from_msgs(cmd_msgs)
        index += 1
        rate.sleep()


def land():
    pass  # placeholder for future landing logic

def main():
    rospy.init_node('full_leg_dynamics_debug', anonymous=True)

    walking_type = "trot"
    leg_joint_map = get_reordered_leg_joint_map()
    rate = rospy.Rate(250)

    index = 0

    # Balance controller setup
    mass = 12.0
    Ib = np.diag([0.1, 0.2, 0.25])
    pcb = np.array([0.0, 0.0, 0.0])
    balance = BalanceController(mass, Ib, pcb)

    contact_pattern = {
        "front_left":  lambda t: int((t % 1.0) < 0.5),
        "front_right": lambda t: int((t % 1.0) >= 0.5),
        "rear_left":   lambda t: int((t % 1.0) >= 0.5),
        "rear_right":  lambda t: int((t % 1.0) < 0.5),
    }

    while not rospy.is_shutdown():

        cmd_msgs = [MotorCmd() for _ in range(12)]



        # print(f"[BalanceCtrl] F_total: {F_total.round(2)} N")
        # print(f"[BalanceCtrl] Torque_total: {torque_total.round(2)} Nm")
        # for i, f in enumerate(foot_forces):
        #     print(f"  Leg {i}: Force {f.round(2)} N")

        # === Generate motor commands
        for leg_name in leg_joint_map:
            idxs = leg_joint_map[leg_name]

            for i in range(3):
                joint_index = idxs[i]
                cmd = cmd_msgs[joint_index]
                cmd.mode = 0x0A
                cmd.q = 0
                cmd.dq = 0
                cmd.Kp = 0
                cmd.Kd = 0
                if i == 1:
                    cmd.tau = 20.65
                else:
                    cmd.tau = 0

        send_motor_commands_from_msgs(cmd_msgs)
        index += 1
        rate.sleep()

if __name__ == '__main__':
    main()
