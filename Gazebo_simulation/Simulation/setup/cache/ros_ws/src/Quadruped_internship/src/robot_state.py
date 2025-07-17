#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import WrenchStamped
from gazebo_msgs.msg import ModelStates
from scipy.spatial.transform import Rotation as R

class Go1RobotData:
    def __init__(self):
        self.joint_state = None
        self.base_pos = None
        self.base_rot = None
        self.base_acc = None  # raw imu acceleration (robot frame)
        self.foot_forces = {
            "front_left": None,
            "front_right": None,
            "rear_left": None,
            "rear_right": None
        }

        rospy.Subscriber("/go1_gazebo/joint_states", JointState, self.joint_state_callback)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback)
        rospy.Subscriber("/trunk_imu", Imu, self.imu_callback)

        topic_map = {
            "front_left": "FL",
            "front_right": "FR",
            "rear_left": "RL",
            "rear_right": "RR"
        }
        for leg_name, topic_key in topic_map.items():
            topic = f"/visual/{topic_key}_foot_contact/the_force"
            rospy.Subscriber(topic, WrenchStamped, self.foot_force_callback, callback_args=leg_name)

    def joint_state_callback(self, msg):
        self.joint_state = msg

    def model_callback(self, msg):
        if "go1_gazebo" in msg.name:
            idx = msg.name.index("go1_gazebo")
            pose = msg.pose[idx]

            # Base position in world frame
            self.base_pos = np.array([
                pose.position.x,
                pose.position.y,
                pose.position.z
            ])

            # Orientation as rotation matrix
            quat = [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ]
            self.base_rot = R.from_quat(quat).as_matrix()

    def imu_callback(self, msg):
        self.base_acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

    def foot_force_callback(self, msg, leg_name):
        self.foot_forces[leg_name] = msg.wrench

    def get_robot_data(self):
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            if (
                self.joint_state and
                self.base_pos is not None and
                self.base_rot is not None and
                self.base_acc is not None and
                all(self.foot_forces.values())
            ):
                return self.joint_state, self.base_pos, self.base_rot, self.base_acc, self.foot_forces
            rate.sleep()

    def get_corrected_base_acc(self):
        # Rotate raw IMU acceleration to world frame
        base_acc_world = self.base_rot @ self.base_acc

        # Subtract gravity (9.81 m/s^2 along Z)
        g_vec = np.array([0, 0, 9.81])
        corrected_acc = base_acc_world - g_vec
        # corrected_acc = base_acc_world
        return corrected_acc


if __name__ == '__main__':
    rospy.init_node('go1_robot_data_listener', anonymous=True)

    robot_data_collector = Go1RobotData()
    js, base_pos, base_rot, base_acc, feet = robot_data_collector.get_robot_data()

    print("Joint States:", js)
    print("Base Position:", base_pos)
    print("Base Rotation Matrix:\n", base_rot)
    print("Base Acceleration (IMU frame):", base_acc)

    # Also show corrected acceleration for ZMP
    corrected_acc = robot_data_collector.get_corrected_base_acc()
    print("Corrected Base Acceleration (world frame minus gravity):", corrected_acc)

    print("Feet Forces:", feet)

