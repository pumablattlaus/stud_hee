#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point, Quaternion, Twist, Pose
from nav_msgs.msg import Odometry
import sys
import time
import numpy as np
import math

class myPose(Pose):
    def __init__(self, pos=(0,0,0), quatern = (0, 0, 0, 1)):
        point = Point(*pos)
        orient = Quaternion(*quatern)
        super(myPose, self).__init__(point, orient)

class Nav2Goal(object):

    def __init__(self):
        self.x = None
        self.y = None

        # Relative stable Poses to work from (use joint angles and relative Pose) 
        self.posesPandaRel = []
        self.posesPandaRel.append(myPose((0.5, 0, 0)))
        self.posesPandaRel.append(myPose((0.2, 0.6, 0)))
        self.posesPandaRel.append(myPose((0.2, -0.6, 0)))

        rospy.init_node('my_moveGoal', anonymous=False)
        self.pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1) 
        sub_odom = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.odom_callback) # get the messages of the robot pose in frame

        ############ -- get the current pose of the robot -- #################
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rospy.loginfo("------------------------------------------------")
        rospy.loginfo("pose x = " + str(self.x))
        rospy.loginfo("pose y = " + str(self.y))

    def sendGoalPos(self, pose):
        poseMsg = PoseStamped()
        poseMsg.header.frame_id = "map"
        poseMsg.pose = pose

        self.pub.publish(poseMsg)

    def getSendGoal(self):
        print("Position: ")
        x = int(input("X= "))
        y = int(input("Y= "))

        print("Orientation: 0-360")
        z = float(input("Z= "))

        z=(z%360)/360
        w = math.sqrt(1-z**2)

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.z = z
        pose.orientation.w = w

        self.sendGoalPos(pose)

    def goalReached():
        #  * /mobile_base_controller/cmd_vel [geometry_msgs/Twist]
        # * /move_base/feedback [move_base_msgs/MoveBaseActionFeedback]
        # * /move_base/goal [move_base_msgs/MoveBaseActionGoal]
        # * /move_base/result [move_base_msgs/MoveBaseActionResult]
        # * /move_base/status [actionlib_msgs/GoalStatusArray] == status_list --> letzter Status
            # GoalStatusArray.status_list.text=="Goal reached" oder .status == 3?
                # "Failed to find a valid plan. Even after executing recovery behaviors." .status==4
                # while .status < 3: sleep
            # sonst: dist(soll-ist < accuracy) via /robot_pose
        pass


    def calculateMirGrippingPose(self, gripPose):
        mirPose = Pose()




if __name__ == '__main__':
    my_nav = Nav2Goal()
    rate = rospy.Rate(0.5)

    pos=(0,0,0)
    point = Point(*pos)

    while not rospy.is_shutdown():
        my_nav.getSendGoal()