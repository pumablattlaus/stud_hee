#!/usr/bin/env python
# coding=latin-1

import rospy
from sensor_msgs.msg import Image



def callback_depth(image=Image()):
    pass

rospy.init_node("Image Viewer")

sub_depth = rospy.Subscriber("/camera/depth/image_raw", Image, callback_depth)


