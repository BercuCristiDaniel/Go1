#!/usr/bin/python3

import rospy
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState, Imu
import numpy as np
from scipy.spatial.transform import Rotation as R
from time import time

class StateEstimator:
    def __init__(self):
        if not rospy.get_name(): 
            rospy.init_node('state_estimator', anonymous=True)

        #   Subscribe to Gazebo link states for base and feet
        self.link_states_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_states_callback)

        #   Subscribe to joint states (for calf angles)
        self.joint_states_sub = rospy.Subscriber('/go1_gazebo/joint_states', JointState, self.joint_states_callback)

        #   Subscribe to IMU for orientation
        self.imu_sub = rospy.Subscriber('/trunk_imu', Imu, self.imu_callback)

        #   Base and foot link names
        self.base_link_name = "go1_gazebo::base"
        self.foot_links = {  
            "FL": "go1_gazebo::FL_calf",  
            "FR": "go1_gazebo::FR_calf",  
            "RL": "go1_gazebo::RL_calf",  
            "RR": "go1_gazebo::RR_calf"
        }

        #   Joint names for calf angles
        self.calf_joints = {
            "FL": "FL_calf_joint",
            "FR": "FR_calf_joint",
            "RL": "RL_calf_joint",
            "RR": "RR_calf_joint"
        }

        #   Known parameters (calf length)
        self.calf_length = 0.23  # meters

        #   Initialize state variables
        self.position = np.zeros(3)  
        self.velocity = np.zeros(3)  
        self.orientation_matrix = np.eye(3)  # IMU-based torso rotation
        self.torso_orientation_matrix = np.eye(3)  # Gazebo-based torso rotation
        self.angular_velocity = np.zeros(3)  
        self.foot_positions = {leg: np.zeros(3) for leg in self.foot_links}
        self.foot_tips = {leg: np.zeros(3) for leg in self.foot_links}
        self.joint_states = {leg: 0.0 for leg in self.calf_joints}
        self.link_orientations = {leg: None for leg in self.foot_links}  # Stores calf orientations

        #   Time tracking for velocity calculation
        self.last_position = np.zeros(3)
        self.last_time = time()

    def link_states_callback(self, msg):
        """Extracts foot positions, orientations, and base CoM from Gazebo's /gazebo/link_states topic."""
        try:
            #   Extract Base (CoM) Position & Orientation
            if self.base_link_name in msg.name:
                index = msg.name.index(self.base_link_name)

                #   Extract base (torso) position
                base_position = np.array([
                    msg.pose[index].position.x,
                    msg.pose[index].position.y,
                    msg.pose[index].position.z
                ])
                
                #   Compute velocity using finite difference
                current_time = time()
                dt = current_time - self.last_time
                if dt > 0:  # Avoid division by zero
                    self.velocity = (base_position - self.last_position) / dt
                
                #   Update stored values
                self.position = base_position
                self.last_position = base_position
                self.last_time = current_time

                #   Extract and convert base (torso) orientation to a rotation matrix
                base_quaternion = msg.pose[index].orientation
                self.torso_orientation_matrix = R.from_quat([
                    base_quaternion.x, 
                    base_quaternion.y, 
                    base_quaternion.z, 
                    base_quaternion.w
                ]).as_matrix()

            #   Extract Foot Positions (Calf)
            for leg, link_name in self.foot_links.items():
                if link_name in msg.name:
                    index = msg.name.index(link_name)

                    #   Store foot (calf) position
                    self.foot_positions[leg] = np.array([
                        msg.pose[index].position.x,
                        msg.pose[index].position.y,
                        msg.pose[index].position.z
                    ])

                    #   Store foot (calf) orientation (quaternion)
                    self.link_orientations[leg] = msg.pose[index].orientation

            #   Compute Foot Tip Positions
            self.compute_foot_tips()

        except Exception as e:
            rospy.logwarn(f"Error in link_states_callback: {e}")

    def joint_states_callback(self, msg):
        """Extracts joint angles for the hip, thigh, and calf joints."""
        try:
            for i, name in enumerate(msg.name):
                if name in self.calf_joints.values() or "hip" in name or "thigh" in name:  
                    self.joint_states[name] = msg.position[i]
        except Exception as e:
            rospy.logwarn(f"Error in joint_states_callback: {e}")




    def imu_callback(self, msg):
        """Converts IMU quaternion to a rotation matrix (R) and extracts angular velocity."""
        try:
            q = msg.orientation  
            quaternion = np.array([q.x, q.y, q.z, q.w])  
            self.orientation_matrix = R.from_quat(quaternion).as_matrix()

            #   Extract angular velocity
            self.angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        except Exception as e:
            rospy.logwarn(f"IMU data error: {e}")
            self.orientation_matrix = np.eye(3)  

    def compute_foot_tips(self):
        """Computes foot tip positions using full orientation from Gazebo."""
        for leg in self.foot_links:
            if leg not in self.foot_positions or leg not in self.link_orientations:
                continue  # Skip if missing data

            #   Get calf position from Gazebo
            calf_pos = self.foot_positions[leg]

            #   Get calf orientation (quaternion) from Gazebo
            calf_quaternion = self.link_orientations[leg]
            if calf_quaternion is None:
                continue  # Skip if no orientation data is available

            #   Convert quaternion to rotation matrix
            rotation_matrix = R.from_quat([
                calf_quaternion.x, 
                calf_quaternion.y, 
                calf_quaternion.z, 
                calf_quaternion.w
            ]).as_matrix()

            #   Define foot tip offset in the local calf frame
            foot_offset_local = np.array([0, 0, -self.calf_length])

            #   Transform foot tip position to world frame
            foot_tip_world = calf_pos + rotation_matrix @ foot_offset_local

            #   Store the computed foot tip position
            self.foot_tips[leg] = foot_tip_world

    def get_state(self):
        """Returns the full estimated state."""
        return {
            'p': self.position,  
            'p_dot': self.velocity,  
            'R': self.orientation_matrix,  # IMU-based torso rotation
            'torso_R': self.torso_orientation_matrix,  # Gazebo-based torso rotation
            'omega': self.angular_velocity,  
            'foot_tips': np.concatenate([self.foot_tips[leg] for leg in ["FL", "FR", "RL", "RR"]])
        }

if __name__ == '__main__':
    estimator = StateEstimator()
    rospy.sleep(1)

    while not rospy.is_shutdown():
        state = estimator.get_state()
        rospy.loginfo(f"CoM Position: {state['p']}")
        rospy.loginfo(f"CoM Velocity: {state['p_dot']}")
        rospy.loginfo(f"Angular Velocity: {state['omega']}")
        rospy.loginfo(f"Torso Rotation Matrix (Gazebo): \n{state['torso_R']}")
        rospy.loginfo(f"Foot Tip Positions (World Frame): {state['foot_tips']}")
        rospy.sleep(0.1)
