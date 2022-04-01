#!/usr/bin/env python3
from ntpath import join
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import numpy as np

def main():

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("movit_joint_test", anonymous=True)
    robot = moveit_commander.RobotCommander()
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    while not rospy.is_shutdown():
        print("===HOME===")
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0 # Gira o braço
        joint_goal[1] = 0 # Levanta e abaixa o braço 
        joint_goal[2] = 0 # Abre e fecha o braço
        joint_goal[3] = 0 # Gira o proximo
        joint_goal[4] = 0 # Levanta e fecha a ferramenta
        joint_goal[5] = 0 # Gira a garra

        group.go(joint_goal, wait=True)
        group.stop()

        joint_goal[0] = pi/2 # Gira o braço

        group.go(joint_goal, wait=True)
        group.stop()

        joint_goal[2] = pi/2
        joint_goal[4] = -pi/3
        group.go(joint_goal, wait=True)
        group.stop()


        print("Searching.......")

        i_start = int(pi/2 * 10)
        i_start_2 = int(-pi/3 * 10)
        for j in range(i_start_2, -(i_start), -1):
            joint_goal[4] = (j/10)
            group.go(joint_goal, wait=True)
            group.stop()
            if rospy.is_shutdown():
                break
            for i in range(i_start, -(i_start), -1):
                joint_goal[0] = (i/10)
                group.go(joint_goal, wait=True)
                group.stop()
                if rospy.is_shutdown():
                    break

        print("Init pick algorithm")




        break

if __name__ == '__main__':
    main()
