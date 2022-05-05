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

def search2():
    listener = tf.TransformListener()
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    joint_goal = group.get_current_joint_values()
    x = joint_goal[0] * 100
    y = 90 * pi / 180 * 100
    for i in range(int(x), int(y), 10):
        joint_goal[0] = i / 100
        group.go(joint_goal, wait=True)
        group.stop()

        try:
            rospy.sleep(1)
            (trans,rot) = listener.lookupTransform('/world', '/id_4', rospy.Time(0))
            print(trans)
            return 'cont'
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        if rospy.is_shutdown():
            break

    return 'rep'


if __name__ == '__main__':
    search2()