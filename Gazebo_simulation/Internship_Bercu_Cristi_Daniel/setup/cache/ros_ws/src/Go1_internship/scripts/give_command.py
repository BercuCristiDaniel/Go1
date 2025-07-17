#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist 

def move_robot(linear_velocity, angular_velocity_yaw):
    #rospy.init_node('robot_command_publisher', anonymous=True)
    
    # Create a publisher for the /cmd_vel topic
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Create a Twist message
    velocity_command = Twist()
    
    # Set linear and angular velocities
    velocity_command.linear.x = linear_velocity
    velocity_command.linear.y = 0.0
    velocity_command.linear.z = 0.0
    velocity_command.angular.x = 0.0
    velocity_command.angular.y = 0.0
    velocity_command.angular.z = angular_velocity_yaw

    # Publish the command at a rate of 10 Hz
    rate = rospy.Rate(100)
    cmd_vel_pub.publish(velocity_command)
    # rospy.loginfo("Command Published: linear.x: {:.2f}, angular.z: {:.2f}".format(
        # velocity_command.linear.x, velocity_command.angular.z, velocity_command.angular.x))
    rate.sleep()
