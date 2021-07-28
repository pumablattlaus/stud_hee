#!/usr/bin/env python
# # coding=latin-1
import copy
import math
import rospy
import geometry_msgs.msg
import tf

if __name__ == "__main__":
    rospy.init_node("TF Test")
    listener = tf.TransformListener()
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/map', '/miranda/panda/panda_link0', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print(trans)
        
        rate.sleep()
