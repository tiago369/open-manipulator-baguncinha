#!/usr/bin/env python3
from ntpath import join
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
import numpy as np

import tf

def tf_world_box():
    listener = tf.TransformListener()

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/world', '/box', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print("Rotation: ")
        print(rot)
        print("Translation: ")
        print(trans)

        return [trans, rot]

def main():
    # Moveit manipulator configuration
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("movit_joint_test", anonymous=True)
    robot = moveit_commander.RobotCommander()
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    while not rospy.is_shutdown():
        print("===HOME===")
        print(group.get_current_joint_values())
        joint_goal = group.get_current_joint_values()

        joint_goal[0] = 90 * pi / 180 
        group.go(joint_goal, wait=True)
        group.stop()

        joint_goal[0] = 0 # Gira o braço
        joint_goal[1] = 0 # Levanta e abaixa o braço 
        joint_goal[2] = 0 # Abre e fecha o braço
        joint_goal[3] = 0 # Gira o proximo
        joint_goal[4] = 0 # Levanta e fecha a ferramenta
        joint_goal[5] = 0 # Gira a garra

        group.go(joint_goal, wait=True)
        group.stop()

        joint_goal[0] = 90 * pi / 180
        joint_goal[1] = 21 * pi / 180
        joint_goal[2] = 46 * pi / 180 # Abre e fecha o braço
        joint_goal[4] = -40 * pi /180 # Levanta e fecha a ferramenta

        group.go(joint_goal, wait=True)
        group.stop()

        joint_goal[0] = 0

        group.go(joint_goal, wait=True)
        group.stop()

        (tr, rt) = tf_world_box()

        print('Use axis')
        print(tr[0])
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.position.x = tr[0]
        pose_goal.position.y = 0.4
        pose_goal.position.z = 0.4
        group.set_pose_target(pose_goal)
        group.go(pose_goal, wait=True)
        group.stop()
        group.clear_pose_targets()

        print('===SUCESS===')

        break




if __name__ == '__main__':
    main()
