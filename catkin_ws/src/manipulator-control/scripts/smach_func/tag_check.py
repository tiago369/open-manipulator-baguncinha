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

def tag_check():
    # rospy.init_node('turtle_tf_listener')
    listener = tf.TransformListener()

    # rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        listener.waitForTransform('/world', '/id_4', rospy.Time(), rospy.Duration(5.0))
        try:
            (trans,rot) = listener.lookupTransform('/world', '/id_4', rospy.Time(0))
            print(trans)
            return 'cont'
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('a')
            return 'rep'
            continue
        # rate.sleep()


if __name__ == '__main__':
    tag_check()
