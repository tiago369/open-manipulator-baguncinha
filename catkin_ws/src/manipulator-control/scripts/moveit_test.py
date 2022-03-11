#!/usr/bin/env python3
# from _future_ import print_function
from numpy import double
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

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("movit_main", anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    
    display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,)

    while not rospy.is_shutdown():
        wpose = move_group.get_current_pose().pose
        print('w: ', wpose.orientation.w)
        print('x: ', wpose.position.x)
        print('y: ', wpose.position.y)
        print('z: ', wpose.position.z)

        print('=====Get Goal Pose=====')
        p_w = double(input('w: '))
        p_x = double(input('x: '))
        p_y = double(input('y: '))
        p_z = double(input('z: '))

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = p_w
        # pose_goal.orientation.x = xq
        # pose_goal.orientation.y = yq
        # pose_goal.orientation.z = zq
        pose_goal.position.x = p_x
        pose_goal.position.y = p_y
        pose_goal.position.z = p_z

        move_group.set_pose_target(pose_goal)
        plan = move_group.go(wait=True)
        # move_group.execute(plan, wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
    

if __name__ == '__main__':
    main()
