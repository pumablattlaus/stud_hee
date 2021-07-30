#!/usr/bin/env python
# # coding=latin-1
import copy
import math
import rospy
import geometry_msgs.msg
import tf

if __name__ == "__main__":
    pos = (0,0,1)
    orient = (0,0,0,1)
    rospy.init_node("tf_test")
    listener = tf.TransformListener()
    publisher = tf.TransformBroadcaster()
    
    rate = rospy.Rate(10.0)
    
    # while(True):
    #     try:
    #         publisher.sendTransform(pos, orient, rospy.Time.now(), "/test_obj", "/map")
    #         print("Finished")
    #         break
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #         print("Exeption!")
    #     rate.sleep()
        
    
    while not rospy.is_shutdown():
        try:
            publisher.sendTransform(pos, orient, rospy.Time.now(), "/map","/test_obj")
            # (trans,rot) = listener.lookupTransform('/map', '/miranda/panda/panda_link0', rospy.Time(0))
            (trans,rot) = listener.lookupTransform('/map', '/miranda/mir/base_link', rospy.Time(0)) # enspricht /robot_pose
            (trans2, rot2) = listener.lookupTransform('/miranda/panda/panda_link0', '/miranda/mir/base_link', rospy.Time(0))
            (trans3, rot3) = listener.lookupTransform('/miranda/panda/panda_link0', '/miranda/panda/panda_hand', rospy.Time(0))
            test = listener.lookupTransform('/map', '/miranda/mir/base_link', rospy.Time(0)) # enspricht /robot_pose
            print("Looking up")
            (t_obj, r_obj) = listener.lookupTransform('/map', '/test_obj', rospy.Time(0))
            print("Finished")
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print("1:------")
        print(trans)
        print(rot)
        print("2:------")
        print(trans2)
        print(rot2)
        print("3:------")
        print(trans3)
        print(rot3)
        print("Map to object:")
        print(t_obj)
        print(r_obj)
        
        rate.sleep()
