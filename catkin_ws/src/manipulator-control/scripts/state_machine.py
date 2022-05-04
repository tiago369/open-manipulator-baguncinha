#!/usr/bin/env python3
import sys
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
import smach
import smach_ros

from smach_func.init_pose import init_pose
from smach_func.home import home
from smach_func.tag_check import tag_check
from smach_func.tf_world_box import tf_world_box
from smach_func.search1 import search1
from smach_func.sobe1 import sobe1
from smach_func.search2 import search2

import math
def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z

class Home(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cont'])

    def execute(self, userdata):
        x = home()
        return x
        

class Init_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cont', 'rep'])

    def execute(self, userdata):
        init_pose()
        # rospy.sleep(1)
        print('------------')
        x = tag_check()
        return x


class Search1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cont', 'rep'])

    def execute(self, userdata):
        search1()
        rospy.sleep(1)
        x = tag_check()
        return x

class Sobe1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cont', 'rep'])

    def execute(self, userdata):
        sobe1()
        rospy.sleep(1)
        x = tag_check()
        return x

class Search2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cont', 'rep'])

    def execute(self, userdata):
        search2()
        rospy.sleep(1)
        x = tag_check()
        return x

class Position(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])
    
    def execute(self, userdata):
        (tr, rt) = tf_world_box()
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = tr[0]+0.1
        pose_goal.position.y = tr[1]
        pose_goal.position.z = 0.3
        group.set_pose_target(pose_goal)
        plan = group.go(wait=True)
        group.stop()
        group.clear_pose_targets()

        joint_goal = group.get_current_joint_values()
        joint_goal[4] = joint_goal[4] + 90 * pi / 180
        [roll, pitch, yaw] = euler_from_quaternion(rt[0], rt[1], rt[2], rt[3])
        print(roll)
        # joint_goal[5] = roll 

        group.go(joint_goal, wait=True)
        group.stop()

        return 'outcome2'

def main():
    rospy.init_node("statemachine", anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    group_name = "arm"
    global group
    group = moveit_commander.MoveGroupCommander(group_name)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['cont', 'rep', 'outcome2', 'outcomex'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('HOME',
                               Home(),
                               transitions={'cont':'INIT_POSE'})
        smach.StateMachine.add('INIT_POSE', 
                               Init_pose(), 
                               transitions={'rep':'HOME',
                               'cont':'HOME'})
        # smach.StateMachine.add('SEARCH1', 
        #                        Search1(), 
        #                        transitions={'rep':'SOBE1',
        #                        'cont':'POSITION'})
        # smach.StateMachine.add('SOBE1', 
        #                        Sobe1(), 
        #                        transitions={'rep':'SEARCH2',
        #                        'cont':'POSITION'})
        # smach.StateMachine.add('SEARCH2', 
        #                        Search2(), 
        #                        transitions={'rep':'SEARCH1',
        #                        'cont':'POSITION'})
        # smach.StateMachine.add('POSITION', 
                            #    Position() 
                            #    )


    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()