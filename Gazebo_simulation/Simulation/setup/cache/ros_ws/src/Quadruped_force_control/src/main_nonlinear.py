#!/usr/bin/python3

import rospy
import numpy as np
from get_SRBM_info import StateEstimator
from joint_control_test import Go1WholeBodyController
from dynamical_model import create_go1_dynamics_function
from nonlinear_MPC import MPC
import casadi as ca


def compute_torques(hip_angle, thigh_angle, calf_angle, forces):
    l1 = 0.23  # Thigh length
    l2 = 0.23  # Calf length

    c_h, s_h = np.cos(hip_angle), np.sin(hip_angle)
    c_t, s_t = np.cos(thigh_angle), np.sin(thigh_angle)
    c_c, s_c = np.cos(calf_angle), np.sin(calf_angle)
    c12, s12 = np.cos(calf_angle - thigh_angle), np.sin(calf_angle - thigh_angle)

    J_v = np.array([
        [l1 * c_t - l2 * np.cos(thigh_angle - calf_angle), l2 * np.cos(thigh_angle - calf_angle), 0],  
        [-s_h * (l1 * s_t + l2 * np.sin(thigh_angle - calf_angle)), -l2 * s_h * s12, c_h * (l1 * c_t + l2 * c12)],  
        [-c_h * (l1 * s_t + l2 * np.sin(thigh_angle - calf_angle)), -l2 * c_h * s12, -s_h * (l1 * c_t + l2 * c12)] 
    ])

    c1, s1 = np.cos(hip_angle), np.sin(hip_angle)
    c2, s2 = np.cos(thigh_angle), np.sin(thigh_angle)
    c23, s23 = np.cos(thigh_angle + calf_angle), np.sin(thigh_angle + calf_angle)

    J_v = np.array([
        [-l1 * s2 - l2 * s23, -l2 * s23, 0],  
        [l1 * c2 + l2 * c23, l2 * c23, 0],  
        [0, 0, 1]  # The calf directly affects vertical movement
    ])


    # Compute torques: τ = J_v^T * F
    torques = np.dot(J_v.T, forces)
    
    return torques


def main():
    rospy.init_node('go1_main_controller')

    estimator = StateEstimator()
    controller = Go1WholeBodyController()
    
    rospy.sleep(1)  

    dt = 0.005  
    Npred = 25 
    n = 18  
    m = 12  

    f = create_go1_dynamics_function()

    # Adjust Go1 Ground Reaction Force (GRF) limits
    umin = np.tile(np.array([-10, -10, -10]), 4)  
    umax = np.tile(np.array([10, 10, 10]), 4)  

    # Adjust Go1 joint limits
    xmin = np.concatenate([[-10, -10, 0], [-2, -2, -2], np.ones(9) * -1, [-1, -1, -1]])
    xmax = np.concatenate([[10, 10, 1], [2, 2, 2], np.ones(9), [1, 1, 1]])
    delta_u_min = np.tile(np.array([-5, -5, -5]), 4)
    delta_u_max = np.tile(np.array([5, 5, 5]), 4)

    # Go1 Cost Matrices
    Q = np.diag(np.ones(n) * 10) 

    R = np.diag(np.ones(m) * 5) 
    P = np.diag(np.ones(n) * 10)

    # Reference State (Standing Pose)
    xref = np.tile(
        np.concatenate([[0, 0, 0.3], [0, 0, 0], np.eye(3).flatten(), [0, 0, 0]]), 
        (Npred+1, 1)
    ).T

    rate = rospy.Rate(200)  

    while not rospy.is_shutdown():
        state = estimator.get_state()
        position = state['p']
        velocity = state['p_dot']
        r_foot_position = state['foot_tips']

        r_vectors = [
            r_foot_position[i*3:(i+1)*3] - position for i in range(4)
        ]
        r = np.concatenate(r_vectors)

        x0 = np.concatenate([position, velocity, state['R'].flatten(), state['omega']])
        u0 = np.zeros(m)

        # Solve MPC
        u_opt = MPC(Npred, x0, u0, n, m, dt, f, umin, umax, xmin, xmax, delta_u_min, delta_u_max, Q, R, P, xref[:,1], r)

        # Ensure Forces Are Non-Negative (GRF Z > 0)
        # u_opt[2::3] = np.maximum(u_opt[2::3], 0)
        torques = np.zeros((4, 3))

        forces_per_leg = {
            "FL": u_opt[0:3],  # Forces Fx, Fy, Fz for Front Left (FL)
            "FR": u_opt[3:6],  # Forces for Front Right (FR)
            "RL": u_opt[6:9],  # Forces for Rear Left (RL)
            "RR": u_opt[9:12]  # Forces for Rear Right (RR)
        }

        # Select only one leg at a time
        active_leg = "FR"  # Change this to "FL", "RL", or "RR" to test other legs

        for i, leg in enumerate(["FL", "FR", "RL", "RR"]):
            hip_angle = estimator.joint_states.get(f"{leg}_hip_joint", 0.0)
            thigh_angle = estimator.joint_states.get(f"{leg}_thigh_joint", 0.0)
            calf_angle = estimator.joint_states.get(f"{leg}_calf_joint", 0.0)

                # Apply force only to the active leg, others get zero force

            F_leg = forces_per_leg[leg]  # Use computed force from u_opt
            # F_leg = [0,0,15]
            rospy.loginfo(f"{leg} Applied Force: {F_leg}")

            # if leg == "FL" or "RL":
            #     F_leg = -F_leg


            F_leg_local = F_leg 
            F_leg_local = np.dot(state['R'].T, F_leg)
                # Compute torques
            torques[i, :] = compute_torques(hip_angle, thigh_angle, calf_angle, F_leg_local)


            # rospy.loginfo(f"{leg} -> Hip: {hip_angle}, Thigh: {thigh_angle}, Calf: {calf_angle}")
        

        # rospy.loginfo(f"Forces: {u_opt}")

        joint_names = [
            "FL_hip", "FL_thigh", "FL_calf",
            "FR_hip", "FR_thigh", "FR_calf",
            "RL_hip", "RL_thigh", "RL_calf",
            "RR_hip", "RR_thigh", "RR_calf"
        ]

        torque_dict = {joint_names[i * 3 + j]: torques[i, j] for i in range(4) for j in range(3)}

        if np.isnan(torques).any():
            rospy.logwarn("Computed torques contain NaN values, skipping torque command.")
        else:
            controller.send_torques(torque_dict)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
