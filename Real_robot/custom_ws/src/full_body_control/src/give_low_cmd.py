import rospy
from unitree_legged_msgs.msg import MotorCmd

# Real robot motor mapping:
# FL: 3,4,5 | FR: 0,1,2 | RL: 9,10,11 | RR: 6,7,8
JOINT_ORDER = [
    "motor_3", "motor_4", "motor_5",   # FL
    "motor_0", "motor_1", "motor_2",   # FR
    "motor_9", "motor_10", "motor_11", # RL
    "motor_6", "motor_7", "motor_8"    # RR
]

motor_publishers = {}

def init_motor_publishers():
    for joint in JOINT_ORDER:
        topic = f"/{joint}/cmd"
        motor_publishers[joint] = rospy.Publisher(topic, MotorCmd, queue_size=1)
    rospy.sleep(0.1)

def send_motor_commands_from_msgs(cmd_msgs):
    if not motor_publishers:
        init_motor_publishers()

    if len(cmd_msgs) != len(JOINT_ORDER):
        rospy.logerr(f"[MotorCmd] Expected {len(JOINT_ORDER)} messages, got {len(cmd_msgs)}")
        return

    for joint, msg in zip(JOINT_ORDER, cmd_msgs):
        motor_publishers[joint].publish(msg)