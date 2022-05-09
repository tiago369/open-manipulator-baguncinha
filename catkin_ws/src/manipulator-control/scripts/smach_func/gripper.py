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

def gripper(a):
    moveit_commander.roscpp_initialize(sys.argv)
    robot = moveit_commander.RobotCommander()
    group_name = "gripper"
    group = moveit_commander.MoveGroupCommander(group_name)
    
    if a:
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = 90 * pi / 180
        group.go(joint_goal, wait=True)
        group.stop()


if __name__ == '__main__':
    gripper(1)