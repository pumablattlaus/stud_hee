#!/usr/bin/env python
# coding=latin-1

import rospy
from sensor_msgs.msg import Image

import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

from cv_bridge import CvBridge, CvBridgeError
import time


bridge = CvBridge()
def callback_depth(image=Image()):
    try:
        cv_img = bridge.imgmsg_to_cv2(image, desired_encoding='8UC1')
    except CvBridgeError as e:
        print(e)

    # (rows,cols) = cv_img.shape
    im_new = cv.normalize(cv_img, cv_img, 0, 254, cv.NORM_MINMAX)

    # # Edge Detection
    # https://docs.opencv.org/3.4/da/d22/tutorial_py_canny.html
    edges = cv.Canny(im_new, 50, 100)  # thresholds: sure non-edge and sure-edge

    plt.subplot(121), plt.imshow(im_new, cmap='gray')
    plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(122), plt.imshow(edges, cmap='gray')
    plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
    plt.show()

    plt.imshow(edges, 'gray')
    plt.show()

    time.sleep(1)


if __name__ == "__main__":
    rospy.init_node("ImageViewer")

    # sub_depth = rospy.Subscriber("/camera/depth/image_raw", Image, callback_depth)
    sub_depth = rospy.Subscriber("/camera/depth/image_rect_raw", Image, callback_depth)
    

    rospy.spin()
