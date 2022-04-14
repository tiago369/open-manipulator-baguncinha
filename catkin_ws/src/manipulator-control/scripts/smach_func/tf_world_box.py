#!/usr/bin/env python3
import rospy
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

if __name__ == '__main__':
    tf_world_box()