# #!/usr/bin/env python3

# import rospy
# from geometry_msgs.msg import Twist
# from nav_msgs.msg import Odometry
# from tf.transformations import euler_from_quaternion
# import numpy as np

# class FeedbackLinearizationTest:
#     def __init__(self):
#         rospy.init_node('feedback_linearization_test')

#         self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
#         self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

#         self.pose = None
#         self.orientation = None

#         # Desired position
#         self.target_position = np.array([10.0, 10.0, 0.0])
#         self.target_yaw = np.pi / 2

#         # PID Controllers
#         self.pid_x = PIDController(Kp=0.5, Ki=0.0, Kd=0.1)
#         self.pid_y = PIDController(Kp=0.5, Ki=0.0, Kd=0.1)
#         self.pid_z = PIDController(Kp=0.5, Ki=0.0, Kd=0.1)
#         self.pid_yaw = PIDController(Kp=0.5, Ki=0.0, Kd=0.1)

#         self.last_time = rospy.get_time()

#     def odom_callback(self, msg):
#         self.pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

#         orientation_q = msg.pose.pose.orientation
#         (roll, pitch, yaw) = euler_from_quaternion([
#             orientation_q.x,
#             orientation_q.y,
#             orientation_q.z,
#             orientation_q.w
#         ])

#         self.orientation = np.array([roll, pitch, yaw])

#     def feedback_linearization(self, p_dot_des, phi_dot_des, theta_dot_des, psi_dot_des, phi, theta, psi):
#         v_world = np.array(p_dot_des)
#         omega_world = np.array([phi_dot_des, theta_dot_des, psi_dot_des])

#         R = np.array([
#             [np.cos(psi) * np.cos(theta), -np.sin(psi) * np.cos(phi) + np.cos(psi) * np.sin(theta) * np.sin(phi),
#              np.sin(psi) * np.sin(phi) + np.cos(psi) * np.sin(theta) * np.cos(phi)],
#             [np.sin(psi) * np.cos(theta), np.cos(psi) * np.cos(phi) + np.sin(psi) * np.sin(theta) * np.sin(phi),
#              -np.cos(psi) * np.sin(phi) + np.sin(psi) * np.sin(theta) * np.cos(phi)],
#             [-np.sin(theta), np.cos(theta) * np.sin(phi), np.cos(theta) * np.cos(phi)]
#         ])

#         W = np.array([
#             [1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
#             [0, np.cos(phi), -np.sin(phi)],
#             [0, np.sin(phi) / np.cos(theta), np.cos(phi) / np.cos(theta)]
#         ])

#         v_body = np.linalg.inv(R) @ v_world
#         omega_body = np.linalg.inv(W) @ omega_world

#         return np.hstack((v_body, omega_body))

#     def run(self):
#         rate = rospy.Rate(20)

#         umin = np.array([-0.5, -0.2, -0.3, -0.5, -0.5, -0.3])
#         umax = np.array([ 1.5,  0.2,  0.3,  0.5,  0.5,  0.3])

#         while not rospy.is_shutdown():
#             if self.pose is None or self.orientation is None:
#                 rospy.logwarn_throttle(2, 'Waiting for odometry...')
#                 continue

#             current_time = rospy.get_time()
#             dt = current_time - self.last_time
#             self.last_time = current_time

#             error_pos = self.target_position - self.pose
#             error_yaw = self.target_yaw - self.orientation[2]

#             x_dot = self.pid_x.compute(error_pos[0], dt)
#             y_dot = self.pid_y.compute(error_pos[1], dt)
#             z_dot = self.pid_z.compute(error_pos[2], dt)
#             psi_dot_des = self.pid_yaw.compute(error_yaw, dt)

#             p_dot_des = [x_dot, y_dot, z_dot]
#             phi_dot_des, theta_dot_des = 0.0, 0.0

#             phi, theta, psi = self.orientation

#             u_cmd = self.feedback_linearization(p_dot_des, phi_dot_des, theta_dot_des, psi_dot_des, phi, theta, psi)

#             u_cmd = np.clip(u_cmd, umin, umax)

#             twist_msg = Twist()
#             twist_msg.linear.x = u_cmd[0]
#             twist_msg.linear.y = u_cmd[1]
#             twist_msg.linear.z = u_cmd[2]
#             twist_msg.angular.x = u_cmd[3]
#             twist_msg.angular.y = u_cmd[4]
#             twist_msg.angular.z = u_cmd[5]

#             self.cmd_vel_pub.publish(twist_msg)

#             rate.sleep()

# class PIDController:
#     def __init__(self, Kp, Ki, Kd):
#         self.Kp = Kp
#         self.Ki = Ki
#         self.Kd = Kd

#         self.prev_error = 0
#         self.integral = 0

#     def compute(self, error, dt):
#         self.integral += error * dt

#         if dt < 1e-6:
#             derivative = 0.0
#         else:
#             derivative = (error - self.prev_error) / dt

#         output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

#         self.prev_error = error

#         return output

# if __name__ == '__main__':
#     node = FeedbackLinearizationTest()
#     node.run()


import numpy as np
import casadi as ca

class FeedbackLinearization:
    @staticmethod
    def transform_world_to_body(p_dot_des, phi_dot_des, theta_dot_des, psi_dot_des, phi, theta, psi):
        v_world = np.array(p_dot_des)
        omega_world = np.array([phi_dot_des, theta_dot_des, psi_dot_des])

        # Rotation matrix from body to world
        R = np.array([
            [np.cos(psi) * np.cos(theta), -np.sin(psi) * np.cos(phi) + np.cos(psi) * np.sin(theta) * np.sin(phi),
             np.sin(psi) * np.sin(phi) + np.cos(psi) * np.sin(theta) * np.cos(phi)],
            [np.sin(psi) * np.cos(theta), np.cos(psi) * np.cos(phi) + np.sin(psi) * np.sin(theta) * np.sin(phi),
             -np.cos(psi) * np.sin(phi) + np.sin(psi) * np.sin(theta) * np.cos(phi)],
            [-np.sin(theta), np.cos(theta) * np.sin(phi), np.cos(theta) * np.cos(phi)]
        ])

        # Transformation matrix for angular rates
        W = np.array([
            [1, np.sin(phi) * np.tan(theta) , np.cos(phi) * np.tan(theta)],
            [0, np.cos(phi), np.sin(phi)],
            [0, -np.sin(phi) / np.cos(theta), np.cos(phi)/np.cos(theta)]
        ])


        # Transform to body-frame
        v_body = np.linalg.inv(R) @ v_world
        omega_body = np.linalg.inv(W) @ omega_world

        return np.hstack((v_body, omega_body))

    @staticmethod
    def treat_constraints(phi, theta, psi, umin, umax):
        # Compute transformation matrix R using symbolic variables
        R = ca.vertcat(
            ca.horzcat(ca.cos(psi) * ca.cos(theta), -ca.sin(psi) * ca.cos(phi) + ca.cos(psi) * ca.sin(theta) * ca.sin(phi),
                   ca.sin(psi) * ca.sin(phi) + ca.cos(psi) * ca.sin(theta) * ca.cos(phi)),
            ca.horzcat(ca.sin(psi) * ca.cos(theta), ca.cos(psi) * ca.cos(phi) + ca.sin(psi) * ca.sin(theta) * ca.sin(phi),
                   -ca.cos(psi) * ca.sin(phi) + ca.sin(psi) * ca.sin(theta) * ca.cos(phi)),
            ca.horzcat(-ca.sin(theta), ca.cos(theta) * ca.sin(phi), ca.cos(theta) * ca.cos(phi))
        )

        # Compute transformation matrix W
        W = ca.vertcat(
            ca.horzcat(1, ca.sin(phi) * ca.tan(theta), ca.cos(phi) * ca.tan(theta)),
            ca.horzcat(0, ca.cos(phi), ca.sin(phi)),
            ca.horzcat(0, -ca.sin(phi) / ca.cos(theta), ca.cos(phi) / ca.cos(theta))
        )

        # Compute transformed constraints using CasADi inverse
        v_new_umin = ca.inv(R) @ umin[0:3]
        v_new_umax = ca.inv(R) @ umax[0:3]

        omega_new_umin = ca.inv(W) @ umin[3:6]
        omega_new_umax = ca.inv(W) @ umax[3:6]

        # Concatenate to get full 6x1 constraint
        new_umin = ca.vertcat(v_new_umin, omega_new_umin)
        new_umax = ca.vertcat(v_new_umax, omega_new_umax)

        return new_umin, new_umax