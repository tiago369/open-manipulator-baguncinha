#!/usr/bin/env python3
from ast import Return
from ntpath import join
from pkgutil import ImpImporter
import sys
import copy

from urllib3 import Retry
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt
    tau = 2.0 * pi

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np

import tf
from smach_func.tf_world_box import tf_world_box

def aprox():
    robot = moveit_commander.RobotCommander()
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    while not rospy.is_shutdown():
        [trans, rot] = tf_world_box()
        pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.orientation.x = rot[0]
        # pose_goal.orientation.y = rot[1]
        # pose_goal.orientation.z = rot[2]
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = trans[0]
        pose_goal.position.y = trans[1] 
        pose_goal.position.z = 0.4
        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(1)

        print('.cu')

        joint_goal = group.get_current_joint_values()
        joint_goal[2] = joint_goal[2] - 10 * pi / 180
        joint_goal[3] = 90 *pi/180 # Gira o proximo
        joint_goal[4] = 70 * pi /180
        group.go(joint_goal, wait=True)
        rospy.sleep(1)
        group.stop()
        rospy.sleep(1)
        print('cuda')

        dif_x = pose_goal.position.x - trans[0]
        dif_y = pose_goal.position.y - trans[1]

        dif_x = ((dif_x)**2)**0.5
        dif_y = ((dif_y)**2)**0.5
        
        if dif_x < 0.001 and dif_y < 0.001:
            return 'cont'

        


if __name__ == '__main__':
    aprox()