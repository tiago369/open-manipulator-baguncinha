#!/usr/bin/env python3
import rospy
import math
from std_msgs.msg import String
from sensor_msgs.msg import JointState

def joint_states(data):
    global data_name, data_position, data_velocity, data_effort
    data_name = data.name
    data_position = data.position
    data_velocity = data.velocity
    data_effort = data.effort


if __name__ == "__main__":
    rospy.init_node('malha_aberta', anonymous=True)

    rospy.Subscriber('/open_manipulator/joint_states', JointState, joint_states)
    
    freq = rospy.Rate(10)


    while not rospy.is_shutdown():
        freq.sleep()