#!/usr/bin/python3

import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Imu
import numpy as np

class StateEstimator:
    def __init__(self):
        rospy.init_node('state_estimator', anonymous=True)

        # Subscribers
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        self.imu_sub = rospy.Subscriber('/trunk_imu', Imu, self.imu_callback)

        # Storage
        self.position = np.zeros(3)
        self.linear_velocity = np.zeros(3)
        self.orientation_matrix = np.eye(3)
        self.angular_velocity = np.zeros(3)

    def model_states_callback(self, msg):
        idx = msg.name.index('go1_gazebo')
        pose = msg.pose[idx]
        twist = msg.twist[idx]

        self.position = np.array([pose.position.x, pose.position.y, pose.position.z])
        self.linear_velocity = np.array([twist.linear.x, twist.linear.y, twist.linear.z])

    def imu_callback(self, msg):
        # Orientation (quaternion to rotation matrix)
        q = msg.orientation
        self.orientation_matrix = self.quaternion_to_rotation_matrix(q)

        # Angular velocity in body frame
        self.angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

    def quaternion_to_rotation_matrix(self, q):
        # Convert quaternion to rotation matrix
        w, x, y, z = q.w, q.x, q.y, q.z
        return np.array([
            [1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]
        ])

    def get_state(self):
        # Returns the full state vector as per the paper
        return {
            'p': self.position,
            'v': self.linear_velocity,
            'R': self.orientation_matrix,
            'omega': self.angular_velocity
        }

if __name__ == '__main__':
    estimator = StateEstimator()
    rospy.sleep(1)  # Give it time to receive some messages

    while not rospy.is_shutdown():
        state = estimator.get_state()
        rospy.loginfo(f"CoM Position: {state['p']}")
        rospy.loginfo(f"CoM Velocity: {state['v']}")
        rospy.loginfo(f"Rotation Matrix (R): \n{state['R']}")
        rospy.loginfo(f"Angular Velocity (Body frame): {state['omega']}")
        rospy.sleep(0.1)