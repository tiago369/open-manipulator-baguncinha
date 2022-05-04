#!/usr/bin/env python3
import rospy
import moveit_commander

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt
    tau = 2.0 * pi

def search2():
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    joint_goal = group.get_current_joint_values()
    x = joint_goal[0] * 100
    y = 90 * pi / 180 * 100
    for i in range(int(x), int(y), 10):
        joint_goal[0] = i / 100

        if rospy.is_shutdown():
            break

        group.go(joint_goal, wait=True)
        group.stop()

    return 'cont'


if __name__ == '__main__':
    search2()