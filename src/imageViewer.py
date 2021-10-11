#!/usr/bin/env python
# coding=latin-1

import rospy
from sensor_msgs.msg import Image

import cv2 as cv
from cv_bridge import CvBridge
import numpy as np
from matplotlib import pyplot as plt


def callback_depth(image=Image()):
    # So nicht, da negative Werte wegen Fokuslaenge: Erst in Entfernung umrechnen�
    disp_normalized = cv.normalize(disparity, None, 0, 255, cv.NORM_MINMAX, np.uint8())

    # Weit entfernte Objekte ausblenden
    # img_threshold = cv.threshold(disp_normalized, 210, 255, cv.THRESH_BINARY)
    img_threshold = cv.threshold(disp_normalized, 240, 255, cv.THRESH_TOZERO_INV)

    im_new = np.array(img_threshold[1])
    plt.imshow(im_new, 'gray')
    plt.show()

    # # Edge Detection
    # https://docs.opencv.org/3.4/da/d22/tutorial_py_canny.html
    edges = cv.Canny(im_new, 50, 100)  # thresholds: sure non-edge and sure-edge
    # edges = cv.Canny(imgL, 50, 250)
    plt.subplot(121), plt.imshow(im_new, cmap='gray')
    plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(122), plt.imshow(edges, cmap='gray')
    plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
    plt.show()

    plt.imshow(edges, 'gray')
    plt.show()
    
def cb_showImgDepth(msg=Image()):
    img = bridge.imgmsg_to_cv2(msg, "passthrough")
    cv.imshow("Depth", img)
    cv.waitKey(1)
    
    

bridge = CvBridge()
if __name__ == "__main__":
    rospy.init_node("Image_viewer")
    # sub_depth = rospy.Subscriber("/camera/depth/image_raw", Image, callback_depth)
    sub_depth = rospy.Subscriber("/cam/debug/depth_img", Image, cb_showImgDepth)
    
    rospy.spin()
    
    
