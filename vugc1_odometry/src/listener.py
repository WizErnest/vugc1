#!/usr/bin/env python

import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf_listener')
    print('[tf_listener] initialized')
    listener = tf.TransformListener()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
            print(trans, rot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print('[tf_listener]: exception={}'.format(e))
        rate.sleep()
