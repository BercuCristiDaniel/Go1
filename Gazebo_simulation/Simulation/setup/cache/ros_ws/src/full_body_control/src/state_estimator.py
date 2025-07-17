# #!/usr/bin/env python3
# import rospy
# import numpy as np
# from sensor_msgs.msg import Imu
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import PoseStamped, TwistStamped

# class StateEstimator:
#     def __init__(self):
#         rospy.init_node("state_estimator", anonymous=True)

#         # Subscribers (IMU, wheel odometry, vision, etc.)
#         rospy.Subscriber("/imu/data", Imu, self.imu_callback)
#         rospy.Subscriber("/odom", Odometry, self.odom_callback)

#         # Publishers (full-state estimation)
#         self.state_pub = rospy.Publisher("/full_state", PoseStamped, queue_size=10)

#         # State variables
#         self.state = np.zeros(6)  # [x, y, yaw, roll, pitch, yaw_rate]

#     def imu_callback(self, msg):
#         self.state[2] = msg.orientation.z  # Yaw
#         self.state[3] = msg.orientation.x  # Roll
#         self.state[4] = msg.orientation.y  # Pitch
#         self.state[5] = msg.angular_velocity.z  # Yaw rate

#     def odom_callback(self, msg):
#         self.state[0] = msg.pose.pose.position.x  # X
#         self.state[1] = msg.pose.pose.position.y  # Y

#     def publish_state(self):
#         rate = rospy.Rate(50)  # 50 Hz
#         while not rospy.is_shutdown():
#             state_msg = PoseStamped()
#             state_msg.header.stamp = rospy.Time.now()
#             state_msg.pose.position.x = self.state[0]
#             state_msg.pose.position.y = self.state[1]
#             state_msg.pose.orientation.z = self.state[2]  # Yaw

#             self.state_pub.publish(state_msg)
#             rate.sleep()

# if __name__ == "__main__":
#     estimator = StateEstimator()
#     estimator.publish_state()
