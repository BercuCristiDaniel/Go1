#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry

class RobotCoordinates:
    def __init__(self):
        self.position = None
        self.orientation = None
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, data):
        # Extract position
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z

        # Extract orientation (quaternion)
        orientation_x = data.pose.pose.orientation.x
        orientation_y = data.pose.pose.orientation.y
        orientation_z = data.pose.pose.orientation.z
        orientation_w = data.pose.pose.orientation.w

        # Store the position and orientation
        self.position = (x, y, z)
        self.orientation = (orientation_x, orientation_y, orientation_z, orientation_w)

    def get_coordinates(self):
        """ Wait until position data is available, then return it. """
        while not rospy.is_shutdown():
            if self.position is not None and self.orientation is not None:
                return self.position, self.orientation
            rospy.sleep(0.01)  # Small sleep to avoid high CPU usage

def get_robot_coordinates():
    """ Returns the latest robot position and orientation. """
    robot = RobotCoordinates()
    return robot.get_coordinates()

if __name__ == '__main__':
    rospy.init_node('robot_coordinate_listener', anonymous=True)
    print(get_robot_coordinates())
