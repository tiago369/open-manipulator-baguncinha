#!/usr/bin/env python3
# from _future_ import print_function
from numpy import double
from six.moves import input

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

def main():

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("movit_joint_test", anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"
    # move_group = moveit_commander.MoveGroupCommander(group_name)
    group = moveit_commander.MoveGroupCommander(group_name)
    

    while not rospy.is_shutdown():
        # print("============ Printing robot state")
        # print(robot.get_current_state())
        
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 0 #junta base
        joint_goal[1] = 0
        joint_goal[2] = 0
        joint_goal[3] = 0 #Junta antes da munheca /  gira munheca
        joint_goal[4] = 0   #Junta da munheca
        joint_goal[5] = 0
        # print(joint_goal)

        group.go(joint_goal, wait=True)
        group.stop()

        break

if __name__ == '__main__':
    main()
