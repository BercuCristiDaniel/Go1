#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R

class Go1RobotData:
    def __init__(self):
        self.joint_state = None
        self.base_pos = None
        self.base_rot = None     # Rotation matrix
        self.base_quat = None    # Quaternion [x, y, z, w]
        self.base_acc = None     # Raw IMU acceleration (robot frame)

        # No force sensors in this interface
        self.foot_forces = {
            "front_left": None,
            "front_right": None,
            "rear_left": None,
            "rear_right": None
        }

        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)

    def joint_state_callback(self, msg):
        self.joint_state = msg

    def odom_callback(self, msg):
        self.base_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        self.base_quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]

        # Check if quaternion norm is non-zero
        quat_norm = np.linalg.norm(self.base_quat)
        if quat_norm > 0:
            # Normalize the quaternion and convert to rotation matrix
            self.base_quat = np.array(self.base_quat) / quat_norm
            self.base_rot = R.from_quat(self.base_quat).as_matrix()
        else:
            rospy.logwarn("Received zero norm quaternion, skipping odom update.")
            self.base_rot = np.eye(3)  # Default to identity matrix
            
    def imu_callback(self, msg):
        self.base_acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

    def get_robot_data(self):
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            if (
                self.joint_state and
                self.base_pos is not None and
                self.base_rot is not None and
                self.base_quat is not None and
                self.base_acc is not None
            ):
                return self.joint_state, self.base_pos, self.base_rot, self.base_quat, self.base_acc, self.foot_forces
            rate.sleep()

    def get_corrected_base_acc(self):
        base_acc_world = self.base_rot @ self.base_acc
        g_vec = np.array([0, 0, 9.81])
        return base_acc_world - g_vec


if __name__ == '__main__':
    rospy.init_node('go1_robot_data_listener', anonymous=True)

    robot_data_collector = Go1RobotData()
    js, base_pos, base_rot, base_quat, base_acc, feet = robot_data_collector.get_robot_data()

    print("Joint States:", js)
    print("Base Position:", base_pos)
    print("Base Rotation Matrix:\n", base_rot)
    print("Base Quaternion:", base_quat)
    print("Base Acceleration (IMU frame):", base_acc)

    corrected_acc = robot_data_collector.get_corrected_base_acc()
    print("Corrected Base Acceleration (world frame minus gravity):", corrected_acc)
