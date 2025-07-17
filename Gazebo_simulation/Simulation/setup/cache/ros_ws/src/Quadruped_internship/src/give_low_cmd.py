import rospy
from unitree_legged_msgs.msg import MotorCmd

# Topic order must match the order you append msgs in cmd_msgs!
JOINT_ORDER = [
    "FL_hip", "FL_thigh", "FL_calf",
    "FR_hip", "FR_thigh", "FR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
    "RR_hip", "RR_thigh", "RR_calf"
]

motor_publishers = {}

def init_motor_publishers():
    for joint in JOINT_ORDER:
        topic = f"/go1_gazebo/{joint}_controller/command"
        motor_publishers[joint] = rospy.Publisher(topic, MotorCmd, queue_size=1)
    rospy.sleep(0.1)  # allow pub registration

def send_motor_commands_from_msgs(cmd_msgs):
    if not motor_publishers:
        init_motor_publishers()

    if len(cmd_msgs) != len(JOINT_ORDER):
        rospy.logerr(f"[MotorCmd] Expected {len(JOINT_ORDER)} messages, got {len(cmd_msgs)}")
        return

    for joint, msg in zip(JOINT_ORDER, cmd_msgs):
        motor_publishers[joint].publish(msg)