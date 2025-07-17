#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry


class RobotCoordinates:
    def __init__(self):
        # Do not call rospy.init_node here
        self.position = None
        self.orientation = None
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

    def odom_callback(self, data):
        self.position = (
            data.pose.pose.position.x,
            data.pose.pose.position.y,
            data.pose.pose.position.z,
        )
        self.orientation = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w,
        )

    def get_coordinates(self):
        while not rospy.is_shutdown():
            if self.position and self.orientation:
                return self.position, self.orientation


def get_robot_coordinates():
    robot = RobotCoordinates()
    return robot.get_coordinates()


if __name__ == "__main__":
    rospy.init_node("robot_coordinate_listener", anonymous=True)
    print(get_robot_coordinates())
