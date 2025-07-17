#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry

class RobotCoordinates:
    def __init__(self):
        # Initialize node
        rospy.init_node('robot_coordinate_listener', anonymous=True)

        # Variables to store position and orientation
        self.position = None
        self.orientation = None

        # Subscribe to the /odom topic
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, data):
        # Extract the position
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z

        # Extract the orientation (quaternion)
        orientation_x = data.pose.pose.orientation.x
        orientation_y = data.pose.pose.orientation.y
        orientation_z = data.pose.pose.orientation.z
        orientation_w = data.pose.pose.orientation.w

        # Store the position and orientation
        self.position = (x, y, z)
        self.orientation = (orientation_x, orientation_y, orientation_z, orientation_w)
        # print(self.position)
    def get_coordinates(self):
        # Keep the node running until we receive data
        while not rospy.is_shutdown():
            if self.position and self.orientation:
                return self.position, self.orientation

def get_robot_coordinates():
    robot = RobotCoordinates()
    return robot.get_coordinates()

if __name__ == '__main__':
    get_robot_coordinates()
