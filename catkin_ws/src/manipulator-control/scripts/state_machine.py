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
from smach_func.tf_world_box import tf_world_box
from smach_func.search1 import search1
from smach_func.search2 import search2
from smach_func.aprox import aprox
from smach_func.check_aprox import check_aprox
from smach_func.pose2pick import pose2pick
from smach_func.place_pose import place_pose
from smach_func.pick import pick
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
        rospy.sleep(1)
        return x
        

class Init_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome2'])

    def execute(self, userdata):
        x = init_pose()
        rospy.sleep(1)
        return x

class Search1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cont', 'rep'])

    def execute(self, userdata):
        x = search1()
        rospy.sleep(1)
        # x = tag_check()
        return x

class Search2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cont', 'rep'])

    def execute(self, userdata):
        x = search2()
        rospy.sleep(1)
        # x = tag_check()
        return x

class Aprox(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cont'])

    def execute(self, userdata):
        x = aprox()
        # rospy.sleep(5)
        return x

class Pose2pick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cont'])

    def execute(self, userdata):
        x = pose2pick()
        # rospy.sleep(5)
        return x

class Pick(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cont'])

    def execute(self, userdata):
        x = pick()
        # rospy.sleep(5)
        return x

class Place_pose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['cont'])

    def execute(self, userdata):
        x = place_pose()
        rospy.sleep(5)
        return x


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
                               transitions={'outcome2':'SEARCH1'})
        smach.StateMachine.add('SEARCH1', 
                               Search1(), 
                               transitions={'rep':'SEARCH2', 'cont':'APROX'})
        smach.StateMachine.add('SEARCH2', 
                               Search2(), 
                               transitions={'rep':'SEARCH1',
                               'cont':'APROX'})
        smach.StateMachine.add('APROX', Aprox(),
                               transitions={'cont':'POSE2PICK'
                               })
        smach.StateMachine.add('POSE2PICK', 
                               Pose2pick(),
                               transitions={'cont':'PICK'
                               })
        smach.StateMachine.add('PICK',
                                Pick(),
                                transitions={'cont':'PLACEPOSE'})
        smach.StateMachine.add('PLACEPOSE', 
                               Place_pose())
                            #    transitions={'cont':'HOME'
                            #    })


    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()