#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_robot(linear_velocity, angular_velocity_yaw, angular_velocity_roll):
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    velocity_command = Twist()
    velocity_command.linear.x = linear_velocity
    velocity_command.angular.x = angular_velocity_roll
    velocity_command.angular.z = angular_velocity_yaw
    cmd_vel_pub.publish(velocity_command)
