#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
from unitree_legged_msgs.msg import MotorCmd
import numpy as np

class Go1WholeBodyController:
    def __init__(self):
        rospy.init_node('go1_wbc_pd_controller', anonymous=True)

        # Base state variables
        self.base_position = None
        self.base_orientation = None
        self.base_linear_vel = None
        self.base_angular_vel = None

        # Joint states
        self.joint_positions = {}
        self.joint_velocities = {}

        # Foot contact forces
        self.foot_forces = {"FL": None, "FR": None, "RL": None, "RR": None}

        # PD Controller Gains
        self.Kp = 60.0  # Proportional gain
        self.Kd = 40.0   # Derivative gain

        # Publishers for joint control
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

        rospy.loginfo("Go1 Whole Body PD Controller Initialized")

    def model_states_callback(self, msg):
        """ Updates base position and velocity information """
        try:
            idx = msg.name.index('go1_gazebo')
            self.base_position = msg.pose[idx].position
            self.base_orientation = msg.pose[idx].orientation
            self.base_linear_vel = msg.twist[idx].linear
            self.base_angular_vel = msg.twist[idx].angular
        except ValueError:
            rospy.logerr("go1_gazebo not found in model_states")

    def joint_states_callback(self, msg):
        joint_map = {
            "FL_calf_joint": "FL_calf",
            "FL_hip_joint": "FL_hip",
            "FL_thigh_joint": "FL_thigh",
            "FR_calf_joint": "FR_calf",
            "FR_hip_joint": "FR_hip",
            "FR_thigh_joint": "FR_thigh",
            "RL_calf_joint": "RL_calf",
            "RL_hip_joint": "RL_hip",
            "RL_thigh_joint": "RL_thigh",
            "RR_calf_joint": "RR_calf",
            "RR_hip_joint": "RR_hip",
            "RR_thigh_joint": "RR_thigh",
        }

        rospy.loginfo_once(f"Joint Names in Message: {msg.name}")

        for i, name in enumerate(msg.name):
            if name in joint_map:  # Only store relevant joints
                joint_name = joint_map[name]
                self.joint_positions[joint_name] = msg.position[i]
                self.joint_velocities[joint_name] = msg.velocity[i]
            else:
                rospy.logwarn(f"Received unknown joint: {name}")

        rospy.loginfo_throttle(1, f"Updated Joint States: {self.joint_positions}")

    def contact_callback(self, msg, leg):
        """ Updates foot contact forces """
        force = msg.wrench.force
        self.foot_forces[leg] = np.array([force.x, force.y, force.z])

    def compute_pd_control(self, target_positions):
        """ Computes PD control for each joint """
        torques = {}
        for joint, desired_pos in target_positions.items():
            if joint in self.joint_positions and joint in self.joint_velocities:
                current_pos = self.joint_positions[joint]
                current_vel = self.joint_velocities[joint]


                position_error = desired_pos - current_pos
                velocity_error = -current_vel  
                torque = self.Kp * position_error + self.Kd * velocity_error
                torques[joint] = torque
            else:
                rospy.logwarn(f"No joint state data available for {joint}")
                torques[joint] = 0.0  # Default safe value

        return torques

    def send_pd_commands(self, target_positions):
        """ Sends PD-controlled torques to joints """
        torques = self.compute_pd_control(target_positions)
        for joint, tau in torques.items():
            msg = MotorCmd()
            msg.mode = 0x0A  # Position control mode
            msg.q = 0.0  # Desired position
            msg.dq = 0.0  # No desired velocity
            msg.tau = tau  # PD-controlled torque
            msg.Kp = self.Kp  # Set PD gains
            msg.Kd = self.Kd

            if joint in self.publishers:
                self.publishers[joint].publish(msg)
            else:
                rospy.logwarn(f"Publisher for {joint} not found!")

    def run(self):
        """ Control loop with PD controller """
        rate = rospy.Rate(30)  # 100Hz control loop
        while not rospy.is_shutdown():
            if self.base_position is not None:
                rospy.loginfo_throttle(1, f"Base Position: {self.base_position.x}, {self.base_position.y}, {self.base_position.z}")

            # Example smooth target positions for standing posture
            target_positions = {
                'FL_hip': 0.0, 'FL_thigh': 0.67, 'FL_calf': -1.3,
                'FR_hip': 0.0, 'FR_thigh': 0.67, 'FR_calf': -1.3,
                'RL_hip': 0.0, 'RL_thigh': 0.67, 'RL_calf': -1.3,
                'RR_hip': 0.0, 'RR_thigh': 0.67, 'RR_calf': -1.3,
            }

            # Send PD-controlled position commands
            self.send_pd_commands(target_positions)

            rate.sleep()


if __name__ == '__main__':
    controller = Go1WholeBodyController()
    controller.run()
