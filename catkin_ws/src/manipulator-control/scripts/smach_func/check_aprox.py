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

def check_aprox():
    robot = moveit_commander.RobotCommander()
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    [trans, rot] = tf_world_box()
    pose_goal = geometry_msgs.msg.Pose()

    dif_x = pose_goal.position.x - rot[0]
    dif_y = pose_goal.position.y - rot[1]

    dif_x = sqrt((dif_x)**2)
    dif_y = sqrt((dif_y)**2)
    
    if dif_x < 0.01 and dif_y < 0.01:
        return 'cont'

    return 'rep'

if __name__ == '__main__':
    check_aprox()