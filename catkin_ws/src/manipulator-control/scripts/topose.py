#!/usr/bin/env python3

from __future__ import print_function
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

from main_class import BlackThor
from main_class import all_close



def main():
    try:
        print("")
        print("----------------------------")
        print("Welcome to blackthor pose to goal")
        print("----------------------------")
        print("")
        input(
            "============ Press `Enter` to begin the control ..."
        )
        tutorial = BlackThor()

        input("============ Press `Enter` to execute a movement using a pose goal ...")

        # Standard pose to tag location
        tutorial.go_to_pose_goal(0.377, -0.115, 0.083, 0, 0, 0, 1)

        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()