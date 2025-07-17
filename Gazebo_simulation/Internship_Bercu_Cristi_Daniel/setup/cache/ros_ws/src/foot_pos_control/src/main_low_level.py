# #!/usr/bin/env python3

# import sys
# sys.path.append('/root/ros_ws/src/foot_pos_control/src')

# import rospy
# import time
# import numpy as np
# from pathlib import Path
# import pinocchio as pin
# from std_msgs.msg import Bool, Empty
# from sensor_msgs.msg import JointState
# from geometry_msgs.msg import Twist
# from inverse_kinematics import Go1LegDynamics
# from robot_state import Go1RobotData
# from bezier_curves import generate_all_leg_trajectories, generate_rise_trajectories
# from give_low_cmd import send_motor_commands_from_msgs, init_motor_publishers
# from unitree_legged_msgs.msg import MotorCmd
# import scipy.io

# # Shared global velocity (updated from /desired_velocity)
# latest_velocity = np.zeros(3)  # vx, vy, omega_z
# should_save_data = False
# trajectories_generated = False  # Flag to track if trajectories have been generated

# motor_publishers = {}

# # Velocity callback from /desired_velocity topic
# def velocity_callback(msg):
#     global latest_velocity
#     latest_velocity[0] = msg.linear.x  # vx
#     latest_velocity[1] = msg.linear.y  # vy
#     latest_velocity[2] = msg.angular.z  # omega_z

# def save_data_callback(msg):
#     global should_save_data
#     should_save_data = True

# def save_to_mat(filename, t, state_xi, pos_ref):
#     data = {
#         't': t,
#         'state_xi': state_xi,
#         'pos_ref': pos_ref
#     }
#     scipy.io.savemat(filename, data)
#     print(f"Data saved to {filename}.")

# def reorder_joint_vector(q_raw):
#     ordered_indices = [1, 2, 0, 4, 5, 3, 7, 8, 6, 10, 11, 9]
#     return q_raw[ordered_indices]

# def main():
#     global latest_velocity, phase, traj_index
#     rospy.init_node('locomotion_control', anonymous=True)

#     # Publisher to notify when the robot is ready to start walking
#     ready_pub = rospy.Publisher("/robot_ready", Bool, queue_size=1)
    
#     rospy.Subscriber("/desired_velocity", Twist, velocity_callback)  # Subscribe to desired_velocity topic
#     rate = rospy.Rate(100)

#     home = Path.home()
#     URDF_PATH = home / "ros_ws/src/foot_pos_control/robots/go1_description/urdf/go1.urdf"
#     URDF_PATH = str(URDF_PATH.resolve())
#     print("Using URDF:", URDF_PATH)

#     model = pin.buildModelFromUrdf(URDF_PATH)
#     data = model.createData()

#     leg_joint_map = {
#         "front_left":  [0, 1, 2],
#         "front_right": [3, 4, 5],
#         "rear_left":   [6, 7, 8],
#         "rear_right":  [9, 10, 11]
#     }

#     foot_frame_names = {
#         "front_left":  "FL_foot",
#         "front_right": "FR_foot",
#         "rear_left":   "RL_foot",
#         "rear_right":  "RR_foot"
#     }

#     state_caller = Go1RobotData()
#     Kx = np.diag([1000, 1000, 1000])
#     Bx = np.diag([44, 44, 44])

#     # === PHASE CONTROL: Rise first, then walk ===
#     phase = "rise"
#     rise_traj, rise_vel, rise_acc = generate_rise_trajectories()
#     rise_index = 0
#     rise_length = len(next(iter(rise_traj.values())))

#     traj_index = 0
#     legs_traj, legs_vel, legs_acc = {}, {}, {}

#     log_time = []
#     log_actual_pos = {leg: [] for leg in leg_joint_map}
#     log_desired_pos = {leg: [] for leg in leg_joint_map}

#     # Initialize the last command messages to avoid empty initial values
#     last_cmd_msgs = [MotorCmd() for _ in range(12)]

#     try:
#         while not rospy.is_shutdown():
#             if phase == "rise":
#                 # Set up rise phase trajectories
#                 legs_traj, legs_vel, legs_acc = rise_traj, rise_vel, rise_acc
#                 traj_length = len(next(iter(legs_traj.values())))
#                 traj_index = rise_index
#                 rise_index += 1

#                 if rise_index >= rise_length:
#                     # Transition to walking once rise is complete
#                     print("Rise phase complete, transitioning to walk phase.")
#                     phase = "walk"
#                     traj_index = 0  # Reset trajectory index for walking
#                     vx = 0  # x velocity from live data
#                     vy = 0  # y velocity from live data
#                     omega_z = 0  # z angular velocity from live data
#                     legs_traj, legs_vel, legs_acc = generate_all_leg_trajectories(vx, vy, omega_z, "trot")  # Generate walking trajectories
#                     traj_length = len(next(iter(legs_traj.values())))  # Calculate length of trajectory
#                     print("Walking phase started.")
#                     # Optionally, publish robot readiness or transition message
#                     ready_pub.publish(Bool(data=True))

#             else:
#                 # Walking phase: update trajectory based on current live velocities
#                 vx = latest_velocity[0]  # Update with live velocity data
#                 vy = latest_velocity[1]
#                 omega_z = latest_velocity[2]
                
#                 # Regenerate the trajectories continuously as long as we are walking
#                 legs_traj, legs_vel, legs_acc = generate_all_leg_trajectories(vx, vy, omega_z, "trot")
#                 legs_traj, legs_vel, legs_acc = generate_all_leg_trajectories(0.2, 0, 0, "trot")
#                 traj_length = len(next(iter(legs_traj.values())))  # Length of the new walking trajectory
#                 traj_index = traj_index % traj_length  # Wrap around the trajectory if it exceeds the length

#             # === Robot state ===
#             joint_state, _, _, base_orientation, _, _ = state_caller.get_robot_data()
#             q = reorder_joint_vector(np.array(joint_state.position))
#             qdot = reorder_joint_vector(np.array(joint_state.velocity))

#             # Convert quaternion [x, y, z, w] to rotation matrix
#             x, y, z, w = base_orientation
#             quat = pin.Quaternion(w, x, y, z)  # Reordered
#             R_base = quat.toRotationMatrix()
#             cmd_msgs = [MotorCmd() for _ in range(12)]

#             count = 0
#             for leg_name in leg_joint_map:
#                 leg_dyn = Go1LegDynamics(model, data, foot_frame_names[leg_name])

#                 x = leg_dyn.forward_kinematics(q)
#                 x_new = R_base @ x

#                 J = leg_dyn.jacobian(q)
#                 M = leg_dyn.mass_matrix(q)
#                 C = leg_dyn.coriolis_matrix(q, qdot)
#                 g_full = leg_dyn.gravity_vector(q)
#                 Jdot = leg_dyn.jacobian_dot(q, qdot)

#                 q_leg = q[count * 3:count * 3 + 3]
#                 qdot_leg = qdot[count * 3:count * 3 + 3]
#                 M_leg = M[count * 3:count * 3 + 3, count * 3:count * 3 + 3]
#                 C_leg = C[count * 3:count * 3 + 3, count * 3:count * 3 + 3]
#                 g_leg = g_full[count * 3:count * 3 + 3]
#                 J_leg = J[0:3, count * 3:count * 3 + 3]
#                 Jdot_leg = Jdot[0:3, count * 3:count * 3 + 3]

#                 xdot_leg = J_leg @ qdot_leg
#                 x_des_leg = legs_traj[leg_name][traj_index]
#                 xdot_des_leg = legs_vel[leg_name][traj_index]
#                 xddot_des_leg = legs_acc[leg_name][traj_index]

#                 q_des_full = leg_dyn.inverse_kinematics_pinocchio(
#                     model, data,
#                     model.getFrameId(foot_frame_names[leg_name]),
#                     q.copy(), x_des_leg
#                 )
#                 q_des_leg = q_des_full[count * 3:count * 3 + 3]
#                 qdot_des_leg = np.linalg.pinv(J_leg) @ xdot_des_leg

#                 Kq = J_leg.T @ Kx @ J_leg
#                 Bq = J_leg.T @ Bx @ J_leg

#                 tau_ff = (
#                     M_leg @ np.linalg.pinv(J_leg) @ (xddot_des_leg - Jdot_leg @ qdot_leg)
#                     + C_leg @ qdot_leg
#                     + g_leg 
#                 )

#                 tau_ff = np.clip(tau_ff, -21.7, 21.7)
#                 if count == 0:
#                     log_time.append(rospy.Time.now().to_sec())

#                 x_des_world = R_base @ x_des_leg    
#                 log_actual_pos[leg_name].append(x_new.copy())
#                 log_desired_pos[leg_name].append(x_des_world.copy())

#                 # Prepare the message for each motor in the current leg
#                 for j in range(3):
#                     idx = count * 3 + j
#                     cmd_msgs[idx].mode = 0x0A
#                     cmd_msgs[idx].tau = tau_ff[j]
#                     cmd_msgs[idx].q = q_des_leg[j]
#                     cmd_msgs[idx].dq = qdot_des_leg[j]
#                     cmd_msgs[idx].Kp = Kq[j, j]
#                     cmd_msgs[idx].Kd = Bq[j, j]

#                 count += 1

#             send_motor_commands_from_msgs(cmd_msgs)

#             # Update traj_index for continuous cycling of the walking trajectory
#             if phase == "walk":
#                 traj_index = (traj_index + 1) % traj_length

#             rate.sleep()

#     except rospy.ROSInterruptException:
#         rospy.loginfo("Shutting down.")

# if __name__ == '__main__':
#     init_motor_publishers()
#     main()



#!/usr/bin/env python3

import sys
sys.path.append('/root/ros_ws/src/foot_pos_control/src')

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
from give_low_cmd import send_motor_commands_from_msgs, init_motor_publishers
from unitree_legged_msgs.msg import MotorCmd
import scipy.io

latest_velocity = np.zeros(3)
should_save_data = False


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
        'pos_ref': pos_ref,
    }
    if tau is not None:
        data['tau'] = tau
    scipy.io.savemat(filename, data)
    print(f"Data saved to {filename}.")

def reorder_joint_vector(q_raw):
    ordered_indices = [1, 2, 0, 4, 5, 3, 7, 8, 6, 10, 11, 9]
    return q_raw[ordered_indices]

def main():
    global should_save_data

    rospy.init_node('locomotion_control', anonymous=True)
    rospy.Subscriber("/desired_velocity", Twist, velocity_callback)
    rospy.Subscriber("/save_data_trigger", Empty, save_data_callback)
    ready_pub = rospy.Publisher("/robot_ready", Bool, queue_size=1)
    rate = rospy.Rate(250)

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
    Kx = np.diag([1200, 1200, 1200])
    Bx = np.diag([60, 60, 60])

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
                traj_index = rise_index
                rise_index += 1
                if rise_index >= rise_length:
                    phase = "walk"
                    traj_index = 0
                    print("Transitioning to WALK phase")
                    ready_pub.publish(Bool(data=True))
            else:
                vx, vy, omega_z = latest_velocity
                legs_traj, legs_vel, legs_acc = generate_all_leg_trajectories(vx, vy, omega_z, "trot")
                traj_length = len(next(iter(legs_traj.values())))
                traj_index = traj_index % traj_length


            joint_state, _, _, base_orientation, _, forces = state_caller.get_robot_data()
            q = reorder_joint_vector(np.array(joint_state.position))
            qdot = reorder_joint_vector(np.array(joint_state.velocity))

            x_q, y_q, z_q, w_q = base_orientation
            R_base = pin.Quaternion(w_q, x_q, y_q, z_q).toRotationMatrix()

            cmd_msgs = [MotorCmd() for _ in range(12)]

            count = 0
            for leg_name in leg_joint_map:
                leg_dyn = Go1LegDynamics(model, data, foot_frame_names[leg_name])

                x = leg_dyn.forward_kinematics(q)
                x_log = R_base @ x  # Actual position in world

                J = leg_dyn.jacobian(q)
                M = leg_dyn.mass_matrix(q)
                C = leg_dyn.coriolis_matrix(q, qdot)
                g_full = leg_dyn.gravity_vector(q)
                Jdot = leg_dyn.jacobian_dot(q, qdot)

                q_leg = q[count*3:count*3+3]
                qdot_leg = qdot[count*3:count*3+3]
                M_leg = M[count*3:count*3+3, count*3:count*3+3]
                C_leg = C[count*3:count*3+3, count*3:count*3+3]
                g_leg = g_full[count*3:count*3+3]
                J_leg = J[0:3, count*3:count*3+3]
                Jdot_leg = Jdot[0:3, count*3:count*3+3]

                xdot_leg = J_leg @ qdot_leg
                x_des_leg = legs_traj[leg_name][traj_index]
                xdot_des_leg = legs_vel[leg_name][traj_index]
                xddot_des_leg = legs_acc[leg_name][traj_index]

                x_des_world = R_base @ x_des_leg  # Reference in world frame

                q_des_full = leg_dyn.inverse_kinematics_pinocchio(
                    model, data,
                    model.getFrameId(foot_frame_names[leg_name]),
                    q.copy(), x_des_leg
                )
                q_des_leg = q_des_full[count*3:count*3+3]
                qdot_des_leg = np.linalg.pinv(J_leg) @ xdot_des_leg

                Kq = J_leg.T @ Kx @ J_leg
                Bq = J_leg.T @ Bx @ J_leg

                tau_ff = (
                    M_leg @ np.linalg.pinv(J_leg) @ (xddot_des_leg - Jdot_leg @ qdot_leg)
                    + C_leg @ qdot_leg
                    + g_leg
                )
                
                tau_ff = np.clip(tau_ff, -21.7, 21.7)

                # if count == 0:
                #     log_time.append(rospy.Time.now().to_sec())

                # log_actual_pos[leg_name].append(x_log.copy())
                # log_desired_pos[leg_name].append(x_des_world.copy())

                for j in range(3):
                    idx = count * 3 + j

                    # Compute full control torque: tau_c = tau_ff + PD
                    tau_pd = Kq[j, j] * (q_des_leg[j] - q_leg[j]) + Bq[j, j] * (qdot_des_leg[j] - qdot_leg[j])
                    tau_c = tau_ff[j] + tau_pd

                    cmd_msgs[idx].mode = 0x0A
                    cmd_msgs[idx].tau = tau_ff[j]
                    cmd_msgs[idx].q = q_des_leg[j]
                    cmd_msgs[idx].dq = qdot_des_leg[j]
                    cmd_msgs[idx].Kp = Kq[j, j]
                    cmd_msgs[idx].Kd = Bq[j, j]

                    # Log tau_c
                    log_tau[leg_name].append([tau_c])

                count += 1

            send_motor_commands_from_msgs(cmd_msgs)

            # if should_save_data:
            #     t_arr = np.array(log_time)
            #     state_xi_arr = {leg: np.vstack(log_actual_pos[leg]) for leg in leg_joint_map}
            #     pos_ref_arr = {leg: np.vstack(log_desired_pos[leg]) for leg in leg_joint_map}
            #     tau_arr = {leg: np.vstack(log_tau[leg]) for leg in leg_joint_map}

            #     for leg in leg_joint_map:
            #         filename = f"/tmp/{leg}_trajectory.mat"
            #         save_to_mat(filename, t_arr, state_xi_arr[leg], pos_ref_arr[leg], tau_arr[leg])

            #     should_save_data = False

            traj_index += 1
            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down.")

if __name__ == '__main__':
    init_motor_publishers()
    main()
