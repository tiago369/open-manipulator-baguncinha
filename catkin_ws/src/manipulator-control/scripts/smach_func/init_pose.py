#!/usr/bin/env python3
import rospy
import moveit_commander

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt
    tau = 2.0 * pi

def init_pose():
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 90 * pi / 180
    # joint_goal[1] = 21 * pi / 180
    joint_goal[2] = -131 * pi / 180
    joint_goal[4] = 70 * pi /180

    group.go(joint_goal, wait=True)
    rospy.sleep(1)
    group.stop()
    rospy.sleep(1)

    return 'outcome2'


if __name__ == '__main__':
    init_pose()