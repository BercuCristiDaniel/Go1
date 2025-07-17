#!/usr/bin/env python3

import rospy
import time
import numpy as np
from pathlib import Path
import pinocchio as pin
from inverse_kinematics import Go1LegDynamics
from robot_state import Go1RobotData
from bezier_curves import generate_rise_trajectories, generate_all_leg_trajectories
from give_low_cmd import send_motor_commands_from_msgs, init_motor_publishers
from unitree_legged_msgs.msg import MotorCmd
from inverse_by_hand import jacobian, inverse_kinematics

def reorder_joint_vector(q_raw):
    ordered_indices = [1, 2, 0, 4, 5, 3, 7, 8, 6, 10, 11, 9]
    return q_raw[ordered_indices]

def generate_rise_trajectory():
    return generate_rise_trajectories()

def generate_walk_trajectory():
    return generate_all_leg_trajectories(0.4, 0.0, 0.0, "trot")

def main():
    rospy.init_node('full_leg_dynamics_debug', anonymous=True)

    home = Path.home()
    URDF_PATH = home / "ros_ws/src/foot_pos_control/robots/go1_description/urdf/go1.urdf"
    URDF_PATH = str(URDF_PATH.resolve())
    print("Using URDF:", URDF_PATH)

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
    rate = rospy.Rate(100)

    Kx = np.diag([1000, 1000, 1000])
    Bx = np.diag([44, 44, 44])

    # Phase sequence: rise for 5s, then walk forever
    phases = [
        (generate_rise_trajectory(), 5000),
        (generate_walk_trajectory(), None)
    ]
    phase_index = 0
    legs_traj, legs_vel, legs_acc = phases[phase_index][0]
    traj_length = len(next(iter(legs_traj.values())))
    start_time = rospy.Time.now()

    try:
        index = 0
        while not rospy.is_shutdown():
            if phases[phase_index][1] is not None:
                elapsed = (rospy.Time.now() - start_time).to_sec() * 1000.0
                if elapsed >= phases[phase_index][1]:
                    phase_index += 1
                    if phase_index >= len(phases):
                        break
                    print(f"Switched to phase {phase_index}: walking")
                    legs_traj, legs_vel, legs_acc = phases[phase_index][0]
                    traj_length = len(next(iter(legs_traj.values())))
                    start_time = rospy.Time.now()
                    index = 0

            joint_state, _, _, _, forces = state_caller.get_robot_data()
            q = reorder_joint_vector(np.array(joint_state.position))
            qdot = reorder_joint_vector(np.array(joint_state.velocity))

            cmd_msgs = [MotorCmd() for _ in range(12)]

            # Shared dynamics
            leg_dyn = Go1LegDynamics(model, data, foot_frame_names["front_left"])
            M = leg_dyn.mass_matrix(q)
            C = leg_dyn.coriolis_matrix(q, qdot)
            g_full = leg_dyn.gravity_vector(q)
            Jdot = leg_dyn.jacobian_dot(q, qdot)

            count = 0
            for leg_name in leg_joint_map:
                leg_dyn = Go1LegDynamics(model, data, foot_frame_names[leg_name])
                x = leg_dyn.forward_kinematics(q)
                J = leg_dyn.jacobian(q)
                
                M = leg_dyn.mass_matrix(q)
                C = leg_dyn.coriolis_matrix(q, qdot)
                g_full = leg_dyn.gravity_vector(q)
                Jdot = leg_dyn.jacobian_dot(q, qdot)

                q_leg = q[count*3:count*3+3]
                qdot_leg = qdot[count*3:count*3+3]
                M_leg = M[count*3:count*3+3, count*3:count*3+3]
                C_leg = C[count*3:count*3+3, count*3:count*3+3]
                g_leg = g_full[count*3:count*3+3]  #- 2 #2.7737719
                J_leg = J[0:3, count*3:count*3+3]
                Jdot_leg = Jdot[0:3, count*3:count*3+3]

                xdot_leg = J_leg @ qdot_leg
                x_des_leg = legs_traj[leg_name][index]
                xdot_des_leg = legs_vel[leg_name][index]
                xddot_des_leg = legs_acc[leg_name][index]

                # Inverse kinematics
                q_des_full = leg_dyn.inverse_kinematics_pinocchio(
                    model, data,
                    model.getFrameId(foot_frame_names[leg_name]),
                    q.copy(), x_des_leg
                )
                # print(x_des_leg[count*3:count*3+3])
                # q_des_hand = inverse_kinematics(x_des_leg[count*3:count*3+3])

                # print(q_des_hand - q_des_full[count*3:count*3+3])
                
                q_des_leg = q_des_full[count*3:count*3+3]
                qdot_des_leg = np.linalg.inv(J_leg) @ xdot_des_leg

                # Gain projection
                Kq = J_leg.T @ Kx @ J_leg
                Bq = J_leg.T @ Bx @ J_leg

                # force_vec = forces[leg_name].force
                # Fe = np.array([force_vec.x, force_vec.y, force_vec.z])
                # Feedforward torque
                tau_ff = (
                    M_leg @ np.linalg.inv(J_leg) @ (xddot_des_leg - Jdot_leg @ qdot_leg)
                    + C_leg @ qdot_leg
                    + g_leg
                )

                # Mx_inv = np.linalg.inv(J_leg.T) @ M_leg @ np.linalg.inv(J_leg)  # Cartesian inertia inverse
                # a_ext = np.linalg.inv(Mx_inv) @ Fe  # Acceleration from external force

                # tau_ff = (
                #     M_leg @ np.linalg.inv(J_leg) @ (xddot_des_leg - Jdot_leg @ qdot_leg - a_ext)
                #     + C_leg @ qdot_leg
                #     + g_leg
                #     + J_leg.T @ Fe
                # )
                # Feedback torque in joint space
                # tau_fb = Kq @ (q_des_leg - q_leg) + Bq @ (qdot_des_leg - qdot_leg)

                # Total torque
                tau_leg = tau_ff 
                # tau_leg = np.clip(tau_leg, -23.7, 23.7)

                for j in range(3):
                    idx = count * 3 + j
                    cmd_msgs[idx].mode = 0x0A
                    cmd_msgs[idx].tau = tau_leg[j]
                    cmd_msgs[idx].q = q_des_leg[j]
                    cmd_msgs[idx].dq = qdot_des_leg[j]
                    cmd_msgs[idx].Kp = Kq[j, j]
                    cmd_msgs[idx].Kd = Bq[j, j]

                count += 1

            send_motor_commands_from_msgs(cmd_msgs)
            index = (index + 1) % traj_length
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down.")

if __name__ == '__main__':
    init_motor_publishers()
    main()