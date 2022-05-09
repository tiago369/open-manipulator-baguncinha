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

def pose2pick():
    robot = moveit_commander.RobotCommander()
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    
    while not rospy.is_shutdown():
        print('aaaaa')
        current_pose = group.get_current_pose().pose
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = current_pose.orientation.x
        pose_goal.orientation.y = current_pose.orientation.y
        pose_goal.orientation.z = current_pose.orientation.z
        pose_goal.orientation.w = current_pose.orientation.w
        pose_goal.position.x = current_pose.position.x
        pose_goal.position.y = current_pose.position.y
        pose_goal.position.z = 0.12
        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()
        rospy.sleep(1)

        print('.deu')
        return 'cont'


if __name__ == '__main__':
    pose2pick()