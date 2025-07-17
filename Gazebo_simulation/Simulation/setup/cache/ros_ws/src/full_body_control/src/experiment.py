#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from unitree_legged_msgs.msg import MotorCmd
import numpy as np
from std_msgs.msg import Float64

class Go1WholeBodyController:
    def __init__(self):
        rospy.init_node('go1_wbc_node')

        # Base state
        self.base_position = None
        self.base_orientation = None
        self.base_linear_vel = None
        self.base_angular_vel = None

        # Joint state
        self.joint_positions = {}
        self.joint_velocities = {}

        # Foot contact forces
        self.foot_forces = {"FL": None, "FR": None, "RL": None, "RR": None}

        # Publishers for joint torque commands
        self.publishers = {}
        legs = ['FL', 'FR', 'RL', 'RR']
        joints = ['hip', 'thigh', 'calf']
        for leg in legs:
            for joint in joints:
                topic = f'/go1_gazebo/{leg}_{joint}_controller/command'
                self.publishers[f'{leg}_{joint}'] = rospy.Publisher(topic, MotorCmd, queue_size=1)

        # Subscribers
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)
        rospy.Subscriber("/go1_gazebo/joint_states", JointState, self.joint_states_callback)
        rospy.Subscriber("/visual/FL_foot_contact/the_force", WrenchStamped, self.contact_callback, 'FL')
        rospy.Subscriber("/visual/FR_foot_contact/the_force", WrenchStamped, self.contact_callback, 'FR')
        rospy.Subscriber("/visual/RL_foot_contact/the_force", WrenchStamped, self.contact_callback, 'RL')
        rospy.Subscriber("/visual/RR_foot_contact/the_force", WrenchStamped, self.contact_callback, 'RR')

    def model_states_callback(self, msg):
        try:
            idx = msg.name.index('go1_gazebo')
            self.base_position = msg.pose[idx].position
            self.base_orientation = msg.pose[idx].orientation
            self.base_linear_vel = msg.twist[idx].linear
            self.base_angular_vel = msg.twist[idx].angular
        except ValueError:
            rospy.logerr("go1_gazebo not found in model_states")

    def joint_states_callback(self, msg):
        for i, name in enumerate(msg.name):
            self.joint_positions[name] = msg.position[i]
            self.joint_velocities[name] = msg.velocity[i]

    def contact_callback(self, msg, leg):
        force = msg.wrench.force
        self.foot_forces[leg] = np.array([force.x, force.y, force.z])

    def send_torques(self, torques):
        for joint, tau in torques.items():
            msg = MotorCmd()
            msg.mode = 0x0A  # Torque control mode
            msg.q = 0.0  # Not used in torque control, but required
            msg.dq = 0.0  # Not used in torque control, but required
            msg.tau = tau  # The torque command

            self.publishers[joint].publish(msg)

    def run(self):
        rate = rospy.Rate(200)  # 200Hz control loop
        while not rospy.is_shutdown():
            if self.base_position is not None:
                rospy.loginfo_throttle(1, f"Base Position: {self.base_position.x}, {self.base_position.y}, {self.base_position.z}")

            # Example torque commands (replace with actual WBC output later)
            example_torques = {
                'FL_hip': 2.0, 'FL_thigh': 2.0, 'FL_calf': 2.0,
                'FR_hip': 2.0, 'FR_thigh': 2.0, 'FR_calf': 2.0,
                'RL_hip': 0.0, 'RL_thigh': 0.0, 'RL_calf': 0.0,
                'RR_hip': 0.0, 'RR_thigh': 0.0, 'RR_calf': 0.0
            }

            # Send torques (replace with actual output from QP or control logic)
            self.send_torques(example_torques)

            rate.sleep()


if __name__ == '__main__':
    controller = Go1WholeBodyController()
    controller.run()
