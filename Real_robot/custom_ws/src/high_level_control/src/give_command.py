#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist 

def move_robot(linear_velocity_x, linear_velocity_y, angular_velocity_yaw):
    """
        Publishes a velocity command to the /cmd_vel topic for the Go1 robot.
    """
    #rospy.init_node('robot_command_publisher', anonymous=True)
    
    # Create a publisher for the /cmd_vel topic
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    # Create a Twist message
    velocity_command = Twist()
    
    # Set linear and angular velocities
    velocity_command.linear.x = linear_velocity_x
    velocity_command.linear.y = linear_velocity_y
    velocity_command.linear.z = 0
    velocity_command.angular.x = 0
    velocity_command.angular.y = 0
    velocity_command.angular.z = angular_velocity_yaw

    rate = rospy.Rate(100)
    cmd_vel_pub.publish(velocity_command)
    # rospy.loginfo("Command Published: linear.x: {:.2f}, angular.z: {:.2f}".format(
        # velocity_command.linear.x, velocity_command.angular.z, velocity_command.angular.x))
    rate.sleep()
