#!/usr/bin/env python3
import rospy
import geometry_msgs.msg
from std_msgs.msg import String
import math
import tf

def tf_world_box():
    rospy.init_node('tf_listener')
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/world', '/box', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print("Rotation: ")
        print(rot[2])
        print("Translation: ")
        print(trans)
        rate.sleep()

        return [trans, rot]

if __name__ == '__main__':
    tf_world_box()

