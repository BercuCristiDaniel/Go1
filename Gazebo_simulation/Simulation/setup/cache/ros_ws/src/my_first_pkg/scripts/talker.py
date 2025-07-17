#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def talker():
    # Inițializează nodul
    rospy.init_node('talker', anonymous=True)
    
    # Creează un publisher pe topicul 'chatter'
    pub = rospy.Publisher('chatter', String, queue_size=10)
    
    rate = rospy.Rate(1)  # 1 Hz
    i=0
    while not rospy.is_shutdown():

        message = "Hello ROS World! %s %d" % (rospy.get_time(), i)
        rospy.loginfo(message)
        
        # Publică mesajul pe topicul 'chatter'
        pub.publish(message)
        	
        i+=1 
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
