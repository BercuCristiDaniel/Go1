#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("I heard: %s", data.data)

def listener():
    # Inițializează nodul
    rospy.init_node('listener', anonymous=True)
    
    # Creează un subscriber pentru topicul 'chatter'
    rospy.Subscriber('chatter', String, callback)
    
    # Spune-i lui ROS să nu termine nodul
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
